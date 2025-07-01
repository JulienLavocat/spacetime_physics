use std::collections::HashSet;

use log::debug;
use parry3d::{
    bounding_volume::Aabb,
    partitioning::{IndexedData, Qbvh as QbvhImpl},
    query::visitors::BoundingVolumeIntersectionsSimultaneousVisitor,
};
use spacetimedb::ReducerContext;

use crate::{test_collision, utils::get_bodies_direct, PhysicsWorld};

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

pub struct CollisionDetection {
    qbvh: QbvhImpl<Collidable>,
    stack: Vec<(u32, u32)>,
    pairs: HashSet<(Collidable, Collidable)>,
}

impl CollisionDetection {
    pub fn new() -> Self {
        Self {
            qbvh: QbvhImpl::<Collidable>::new(),
            stack: Vec::new(),
            pairs: HashSet::new(),
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
    ) {
        let sw = world.stopwatch("broad_phase");
        let prediction_distance = world.prediction_distance();

        let rebuild_sw = world.stopwatch("broad_phase_rebuild");
        let mut leaves_data: Vec<(Collidable, Aabb)> =
            Vec::with_capacity(bodies.len() + triggers.len());
        for (i, entity) in bodies.iter().enumerate() {
            leaves_data.push((
                Collidable {
                    id: entity.id,
                    rigidbody_index: i,
                    trigger_index: 0,
                    is_trigger: false,
                    collidable_index: i,
                },
                entity
                    .shape
                    .collision_aabb(&entity.into(), prediction_distance),
            ));
        }
        let entities_count = bodies.len();
        for (i, trigger) in triggers.iter().enumerate() {
            leaves_data.push((
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
        self.qbvh
            .clear_and_rebuild(leaves_data.into_iter(), world.qvbh_dilation_factor);
        rebuild_sw.end();

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
                .intersects(&trigger.isometry, &body.rb.into(), &body.shape)
            {
                trigger.new_entities_inside.insert(body.id);
            } else {
                trigger.new_entities_inside.remove(&body.id);
            }
        }

        // TODO: Use QBVH to optimize trigger detection
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
}
