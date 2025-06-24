use std::fmt::Display;

use bon::{builder, Builder};
use spacetimedb::{table, ReducerContext, Table};

use crate::{
    engine::collisions::Collider,
    math::{Mat3, Quat, Vec3},
};

#[table(name = physics_rigid_bodies, public)]
#[derive(Builder, Clone, Copy, Debug, Default, PartialEq)]
#[builder(derive(Debug, Clone))]
pub struct RigidBody {
    #[primary_key]
    #[auto_inc]
    #[builder(default = 0)]
    pub id: u64,
    #[index(btree)]
    #[builder(default = 1)]
    pub world_id: u64,
    #[builder(default = Vec3::ZERO)]
    pub position: Vec3,
    #[builder(default = Quat::IDENTITY)]
    pub rotation: Quat,
    #[builder(default = Vec3::ZERO)]
    pub linear_velocity: Vec3,
    #[builder(default = Vec3::ZERO)]
    pub angular_velocity: Vec3,
    #[builder(default = Mat3::IDENTITY)]
    pub inertia_tensor: Mat3,
    #[builder(skip = inertia_tensor.inverse())]
    pub inv_inertia_tensor: Option<Mat3>,
    #[builder(default = Vec3::ZERO)]
    pub force: Vec3,

    #[builder(default = 1.0)]
    pub mass: f32,
    #[builder(skip = if mass > 0.0 { 1.0 / mass } else { 0.0 })]
    pub inv_mass: f32,

    pub collider: Collider,

    #[builder(default = Vec3::ZERO)]
    pub torque: Vec3,

    #[builder(default = 0.0)]
    pub friction: f32,
    #[builder(default = 0.0)]
    pub restitution: f32,

    #[builder(skip = linear_velocity)]
    pub pre_solve_linear_velocity: Vec3,
    #[builder(skip = angular_velocity)]
    pub pre_solve_angular_velocity: Vec3,

    #[builder(skip = position)]
    pub previous_position: Vec3,
    #[builder(skip = rotation)]
    pub previous_rotation: Quat,
}

impl RigidBody {
    pub fn insert(self, ctx: &ReducerContext) -> Self {
        ctx.db.physics_rigid_bodies().insert(self)
    }

    pub fn update(self, ctx: &ReducerContext) -> Self {
        ctx.db.physics_rigid_bodies().id().update(self)
    }

    pub fn effective_inverse_mass(&self) -> Vec3 {
        // TODO: Take into account locked axes
        Vec3::splat(self.inv_mass)
    }

    pub fn effective_inverse_inertia(&self) -> Mat3 {
        // TODO: Take into account locked axes
        match self.inv_inertia_tensor {
            Some(inv) => inv,
            None => Mat3::IDENTITY, // Static body
        }
    }

    pub fn is_static_or_sleeping(&self) -> bool {
        // TODO: Sleep logic
        self.mass <= 0.0
    }
}

impl Display for RigidBody {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "RigidBody(id={}, world_id={}, position={}, orientation={}, velocity={}, force={}, mass={}, inv_mass={})",
            self.id, self.world_id, self.position, self.rotation, self.linear_velocity, self.force, self.mass, self.inv_mass
        )
    }
}
