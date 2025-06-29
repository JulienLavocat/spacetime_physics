use std::collections::HashSet;

use log::debug;
use parry3d::{
    partitioning::Qbvh as QbvhImpl, query::visitors::BoundingVolumeIntersectionsSimultaneousVisitor,
};

use crate::{
    test_collision, utils::get_bodies, Collider, PhysicsWorld, RigidBody, RigidBodyId, ShapeWrapper,
};

use super::constraints::PenetrationConstraint;

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

    pub fn broad_phase(&mut self, world: &PhysicsWorld, bodies: &[RigidBody]) {
        let sw = world.stopwatch("broad_phase");
        let prediction_distance = world.prediction_distance();

        debug!(
            "[PhysicsWorld#{}] [BroadPhase] bodies: {}, prediction_distance: {}, dilation_factor: {}",
            world.id,
            bodies.len(),
            prediction_distance,
            world.qvbh_dilation_factor
        );
        let rebuild_sw = world.stopwatch("broad_phase_rebuild");
        self.qbvh.clear_and_rebuild(
            bodies.iter().map(|body| {
                let aabb = ShapeWrapper::from(body.collider)
                    .collision_aabb(&body.into(), prediction_distance);

                if let Collider::Plane(_) = body.collider {
                    debug!(
                        "[PhysicsWorld#{}] [BroadPhase] body {}: Plane AABB: {:?}",
                        world.id, body.id, aabb
                    );
                }
                (body.id, aabb)
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
        bodies: &mut [RigidBody],
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
}
