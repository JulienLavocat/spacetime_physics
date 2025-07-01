use std::collections::HashMap;

use parry3d::na::Isometry3;
use spacetimedb::ReducerContext;

use crate::{
    math::{Mat3, Vec3},
    Collider, ColliderId, PhysicsWorldId, RigidBody, ShapeWrapper,
};

pub struct RigidBodyData {
    pub id: u64,
    pub rb: RigidBody,
    pub shape: ShapeWrapper,
    pub inertia_tensor: Mat3,
    pub inv_inertia_tensor: Mat3,
}

impl RigidBodyData {
    pub fn new(rigid_body: RigidBody, collider: &Collider) -> Self {
        let inertia_tensor = collider.inertia_tensor(rigid_body.mass);
        Self {
            id: rigid_body.id,
            rb: rigid_body,
            shape: ShapeWrapper::from(collider),
            inertia_tensor,
            inv_inertia_tensor: inertia_tensor.inverse(),
        }
    }

    pub fn collect(
        ctx: &ReducerContext,
        world_id: PhysicsWorldId,
        colliders: &HashMap<ColliderId, Collider>,
    ) -> Vec<Self> {
        let mut entities: Vec<_> = RigidBody::all(ctx, world_id)
            .map(move |rb| RigidBodyData::new(rb, colliders.get(&rb.collider_id).unwrap()))
            .collect();
        entities.sort_by_key(|e| e.id);
        entities
    }

    pub fn effective_inverse_mass(&self) -> Vec3 {
        // TODO: Take into account locked axes
        Vec3::splat(self.rb.inv_mass)
    }

    pub fn effective_inverse_inertia(&self) -> Mat3 {
        // TODO: Take into account locked axes
        let r = self.rb.rotation.to_mat3();
        r * self.inv_inertia_tensor * r.transpose()
    }
}

impl From<&RigidBodyData> for Isometry3<f32> {
    fn from(value: &RigidBodyData) -> Self {
        value.rb.into()
    }
}
