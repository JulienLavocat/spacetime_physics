use std::collections::{HashMap, HashSet};

use log::debug;
use parry3d::{
    bounding_volume::Aabb,
    partitioning::{IndexedData, Qbvh as QbvhImpl},
    query::{
        visitors::{BoundingVolumeIntersectionsSimultaneousVisitor, RayIntersectionsVisitor},
        Ray,
    },
};
use spacetimedb::ReducerContext;

use crate::{
    test_collision, utils::get_bodies_direct, PhysicsWorld, RayCast, RayCastHit, RaycastId,
};

use super::{
    constraints::PenetrationConstraint, rigid_body_data::RigidBodyData, trigger_data::TriggerData,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Collidable {
    pub id: u64,
    pub rigidbody_index: usize,
    pub trigger_index: usize,
    pub is_trigger: bool,
    pub collidable_index: usize,
}

impl IndexedData for Collidable {
    fn default() -> Self {
        Self {
            id: 0,
            rigidbody_index: 0,
            trigger_index: 0,
            is_trigger: false,
            collidable_index: 0,
        }
    }

    fn index(&self) -> usize {
        self.collidable_index
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct RayCastEntry {
    pub raycast_id: RaycastId,
    pub raycast_index: usize,
}

pub struct CollisionDetection {
    qbvh: QbvhImpl<Collidable>,
    stack: Vec<(u32, u32)>,
    pairs: HashSet<(Collidable, Collidable)>,
    raycasts_pairs: HashMap<RayCastEntry, HashSet<Collidable>>,
}

impl CollisionDetection {
    pub fn new() -> Self {
        Self {
            qbvh: QbvhImpl::<Collidable>::new(),
            stack: Vec::new(),
            pairs: HashSet::new(),
            raycasts_pairs: HashMap::new(),
        }
    }

    pub fn broad_phase_pairs(&mut self) -> &HashSet<(Collidable, Collidable)> {
        &self.pairs
    }

    pub fn broad_phase(
        &mut self,
        world: &PhysicsWorld,
        bodies: &[RigidBodyData],
        triggers: &[TriggerData],
        raycasts: &[RayCast],
    ) {
        let sw = world.stopwatch("broad_phase");
        let prediction_distance = world.prediction_distance();

        let rebuild_sw = world.stopwatch("broad_phase_rebuild");
        let collidables = self.collect_collidables(bodies, triggers, prediction_distance);
        self.qbvh
            .clear_and_rebuild(collidables.into_iter(), world.qvbh_dilation_factor);
        rebuild_sw.end();

        self.run_broad_phase_pairs(world);
        self.run_broad_phase_raycast_pairs(world, raycasts);

        sw.end();
    }

    pub fn narrow_phase_constraints(
        &self,
        world: &PhysicsWorld,
        bodies: &mut [RigidBodyData],
    ) -> Vec<PenetrationConstraint> {
        let sw = world.stopwatch("narrow_phase");
        let mut constraints = Vec::new();

        for (a, b) in &self.pairs {
            if a.is_trigger || b.is_trigger {
                continue; // Skip trigger pairs
            }

            let (body_a, body_b) = get_bodies_direct(a.rigidbody_index, b.rigidbody_index, bodies);

            if let Some(collision) = test_collision(body_a, body_b, world.precision) {
                if collision.distance >= 0.0 {
                    continue; // No penetration
                }

                constraints.push(PenetrationConstraint::new(body_a, body_b, collision, 0.0));
            }
        }

        if world.debug_narrow_phase() {
            debug!(
                "[PhysicsWorld#{}] [NarrowPhase] constraints: {:?}",
                world.id, constraints
            );
        }

        sw.end();
        constraints
    }

    pub fn narrow_phase_triggers(
        &self,
        ctx: &ReducerContext,
        world: &PhysicsWorld,
        bodies: &[RigidBodyData],
        triggers: &mut [TriggerData],
    ) {
        let sw = world.stopwatch("narrow_phase_triggers");
        for (a, b) in &self.pairs {
            if !a.is_trigger && !b.is_trigger {
                continue; // Skip non-trigger pairs
            }

            let (trigger, body) = if a.is_trigger {
                (&mut triggers[a.trigger_index], &bodies[b.rigidbody_index])
            } else {
                (&mut triggers[b.trigger_index], &bodies[a.rigidbody_index])
            };

            if trigger
                .shape
                .intersects(&trigger.isometry, &body.into(), body.shape())
            {
                trigger.new_entities_inside.insert(body.id);
            } else {
                trigger.new_entities_inside.remove(&body.id);
            }
        }

        for trigger in triggers {
            trigger.added_entities = trigger
                .new_entities_inside
                .difference(&trigger.current_entities_inside)
                .cloned()
                .collect();
            trigger.removed_entities = trigger
                .current_entities_inside
                .difference(&trigger.new_entities_inside)
                .cloned()
                .collect();
            let is_different = trigger.current_entities_inside != trigger.new_entities_inside;
            trigger.current_entities_inside = trigger.new_entities_inside.clone();

            if world.debug_triggers() && is_different {
                debug!(
                    "[PhysicsWorld#{}] [Trigger] Trigger#{} entities inside: {:?}, added: {:?}, removed: {:?}",
                    world.id, trigger.trigger_id, trigger.current_entities_inside, trigger.added_entities, trigger.removed_entities
                );
            }

            trigger.update(ctx);
        }
        sw.end();
    }

    pub fn narrow_phase_raycast(
        &self,
        ctx: &ReducerContext,
        world: &PhysicsWorld,
        bodies: &[RigidBodyData],
        raycasts: &mut [RayCast],
    ) {
        let sw = world.stopwatch("narrow_phase_raycast");
        for (raycast_id, broad_hits) in self.raycasts_pairs.iter() {
            let raycast = &mut raycasts[raycast_id.raycast_index];
            let ray = Ray::new(raycast.origin.into(), raycast.direction.into());
            let mut hits = HashSet::new();

            for broad_hit in broad_hits {
                let body = &bodies[broad_hit.rigidbody_index];
                if let Some(intersection) = body.shape().cast_ray_and_get_normal(
                    &body.into(),
                    &ray,
                    raycast.max_distance,
                    raycast.solid,
                ) {
                    hits.insert(RayCastHit {
                        distance: intersection.time_of_impact,
                        position: ray.point_at(intersection.time_of_impact).into(),
                        normal: intersection.normal.into(),
                        rigid_body_id: body.id,
                    });
                }
            }

            let previous_hits: HashSet<_> = raycast.hits.iter().cloned().collect();
            raycast.added_hits = hits.difference(&previous_hits).cloned().collect();
            raycast.removed_hits = previous_hits.difference(&hits).cloned().collect();
            let is_different = previous_hits != hits;
            raycast.hits = hits.into_iter().collect();

            if world.debug_raycasts() && is_different {
                debug!(
                    "[PhysicsWorld#{}] [RayCast] RayCast#{} hits: {:?}, added: {:?}, removed: {:?}",
                    world.id, raycast.id, raycast.hits, raycast.added_hits, raycast.removed_hits
                );
            }
            raycast.clone().update(ctx);
        }
        sw.end();
    }

    fn run_broad_phase_pairs(&mut self, world: &PhysicsWorld) {
        let traverse_sw = world.stopwatch("broad_phase_traverse");
        let mut pairs = HashSet::new();
        let mut visitor = BoundingVolumeIntersectionsSimultaneousVisitor::new(
            |a: &Collidable, b: &Collidable| {
                if a != b {
                    let (min_id, max_id) = if a.collidable_index < b.collidable_index {
                        (a, b)
                    } else {
                        (b, a)
                    };
                    pairs.insert((*min_id, *max_id));
                }
                true
            },
        );
        self.qbvh
            .traverse_bvtt_with_stack(&self.qbvh, &mut visitor, &mut self.stack);
        traverse_sw.end();

        self.pairs = pairs;
    }

    fn run_broad_phase_raycast_pairs(&mut self, world: &PhysicsWorld, raycasts: &[RayCast]) {
        let traverse_sw = world.stopwatch("broad_phase_raycast_traverse");
        for (i, raycast) in raycasts.iter().enumerate() {
            let ray = Ray::new(raycast.origin.into(), raycast.direction.into());
            let mut entities = HashSet::new();
            let mut callback = |collidable: &Collidable| {
                if !collidable.is_trigger {
                    entities.insert(*collidable);
                }
                true
            };
            let mut visitor =
                RayIntersectionsVisitor::new(&ray, raycast.max_distance, &mut callback);

            self.qbvh.traverse_depth_first(&mut visitor);

            if !entities.is_empty() {
                self.raycasts_pairs.insert(
                    RayCastEntry {
                        raycast_id: raycast.id,
                        raycast_index: i,
                    },
                    entities,
                );
            }
        }
        traverse_sw.end();
    }

    fn collect_collidables(
        &self,
        bodies: &[RigidBodyData],
        triggers: &[TriggerData],
        prediction_distance: f32,
    ) -> Vec<(Collidable, Aabb)> {
        let mut collidables: Vec<(Collidable, Aabb)> =
            Vec::with_capacity(bodies.len() + triggers.len());
        for (i, entity) in bodies.iter().enumerate() {
            collidables.push((
                Collidable {
                    id: entity.id,
                    rigidbody_index: i,
                    trigger_index: 0,
                    is_trigger: false,
                    collidable_index: i,
                },
                entity
                    .shape()
                    .collision_aabb(&entity.into(), prediction_distance),
            ));
        }
        let entities_count = bodies.len();
        for (i, trigger) in triggers.iter().enumerate() {
            collidables.push((
                Collidable {
                    id: trigger.trigger_id,
                    rigidbody_index: 0,
                    trigger_index: i,
                    is_trigger: true,
                    collidable_index: i + entities_count,
                },
                trigger
                    .shape
                    .collision_aabb(&trigger.isometry, prediction_distance),
            ));
        }

        collidables
    }
}
