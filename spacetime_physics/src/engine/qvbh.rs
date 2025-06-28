use std::collections::HashSet;

use parry3d::{
    partitioning::Qbvh as QbvhImpl, query::visitors::BoundingVolumeIntersectionsSimultaneousVisitor,
};

use crate::{PhysicsWorld, RigidBody, RigidBodyId, ShapeWrapper};

pub struct Qbvh {
    qbvh: QbvhImpl<RigidBodyId>,
    stack: Vec<(u32, u32)>,
}

impl Qbvh {
    pub fn from_bodies(world: &PhysicsWorld, bodies: &[RigidBody]) -> Self {
        let prediction_distance = world.prediction_distance();

        let mut qbvh = QbvhImpl::<u64>::new();
        qbvh.clear_and_rebuild(
            bodies.iter().map(|body| {
                (
                    body.id,
                    ShapeWrapper::from(body.collider)
                        .collision_aabb(&body.into(), prediction_distance),
                )
            }),
            world.qvbh_dilation_factor,
        );

        Self {
            qbvh,
            stack: Vec::new(),
        }
    }

    pub fn broad_phase(&mut self) -> HashSet<(RigidBodyId, RigidBodyId)> {
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

        pairs
    }
}
