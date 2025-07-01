use std::collections::{HashMap, HashSet};

use log::debug;
use parry3d::{
    partitioning::Qbvh as QbvhImpl, query::visitors::BoundingVolumeIntersectionsSimultaneousVisitor,
};
use spacetimedb::ReducerContext;

use crate::{test_collision, utils::get_bodies, Collider, PhysicsWorld, RigidBodyId};

use super::{
    constraints::PenetrationConstraint, rigid_body_data::RigidBodyData, trigger_data::TriggerData,
};

pub struct CollisionDetection {
    qbvh: QbvhImpl<RigidBodyId>,
    stack: Vec<(u32, u32)>,
    pairs: HashSet<(RigidBodyId, RigidBodyId)>,
}

impl CollisionDetection {
    pub fn new() -> Self {
        Self {
            qbvh: QbvhImpl::<u64>::new(),
            stack: Vec::new(),
            pairs: HashSet::new(),
        }
    }

    pub fn broad_phase_pairs(&mut self) -> &HashSet<(RigidBodyId, RigidBodyId)> {
        &self.pairs
    }

    pub fn broad_phase(&mut self, world: &PhysicsWorld, entities: &[RigidBodyData]) {
        let sw = world.stopwatch("broad_phase");
        let prediction_distance = world.prediction_distance();

        let rebuild_sw = world.stopwatch("broad_phase_rebuild");
        self.qbvh.clear_and_rebuild(
            entities.iter().map(|entity| {
                let aabb = entity
                    .shape
                    .collision_aabb(&entity.into(), prediction_distance);

                (entity.id, aabb)
            }),
            world.qvbh_dilation_factor,
        );
        rebuild_sw.end();

        let traverse_sw = world.stopwatch("broad_phase_traverse");
        let mut pairs = HashSet::new();
        let mut visitor = BoundingVolumeIntersectionsSimultaneousVisitor::new(
            |a: &RigidBodyId, b: &RigidBodyId| {
                if a != b {
                    let (min_id, max_id) = if a < b { (a, b) } else { (b, a) };
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
            let (body_a, body_b) = get_bodies(*a, *b, bodies);

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

    pub fn detect_triggers(
        &self,
        ctx: &ReducerContext,
        world: &PhysicsWorld,
        bodies: &[RigidBodyData],
        colliders: &HashMap<u64, Collider>,
    ) {
        let triggers = TriggerData::collect(ctx, world.id, colliders);

        // TODO: Use QBVH to optimize trigger detection
        for mut trigger in triggers {
            let new_entities_inside: HashSet<RigidBodyId> = bodies
                .iter()
                .filter_map(|body| {
                    if trigger
                        .shape
                        .intersects(&trigger.isometry, &body.into(), &body.shape)
                    {
                        Some(body.id)
                    } else {
                        None
                    }
                })
                .collect();

            trigger.added_entities = new_entities_inside
                .difference(&trigger.current_entities_inside)
                .cloned()
                .collect();
            trigger.removed_entities = trigger
                .current_entities_inside
                .difference(&new_entities_inside)
                .cloned()
                .collect();
            trigger.current_entities_inside = new_entities_inside;

            if world.debug_triggers() {
                debug!(
                    "[PhysicsWorld#{}] [Trigger] Trigger {:?} entities inside: {:?}, added: {:?}, removed: {:?}",
                    world.id, trigger.trigger_id, trigger.current_entities_inside, 
                    trigger.added_entities, trigger.removed_entities
                );
            }

            trigger.update(ctx);
        }
    }
}
