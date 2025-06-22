use std::fmt::Display;

use spacetimedb::{table, ReducerContext, Table};

use crate::{
    engine::collisions::Collider,
    math::{Mat3, Quat, Vec3},
};

#[table(name = physics_rigid_bodies, public)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RigidBody {
    #[primary_key]
    #[auto_inc]
    pub id: u64,
    #[index(btree)]
    pub world_id: u64,
    pub position: Vec3,
    pub rotation: Quat,
    pub velocity: Vec3,
    pub angular_velocity: Vec3,
    pub inertia_tensor: Mat3,
    pub inv_inertia_tensor: Option<Mat3>,
    pub force: Vec3,
    pub mass: f32,
    pub collider: Collider,
    pub inv_mass: f32,
    pub torque: Vec3,
    pub previous_position: Vec3,
    pub previous_rotation: Quat,
    pub friction: f32,
    pub pre_solve_velocity: Vec3,
    pub pre_solve_angular_velocity: Vec3,
    pub restitution: f32,
}

impl RigidBody {
    pub fn new(
        world_id: u64,
        position: Vec3,
        rotation: Quat,
        velocity: Vec3,
        force: Vec3,
        mass: f32,
        collider: Collider,
    ) -> Self {
        let angular_velocity = Vec3::default();
        let inertia_tensor = Mat3::IDENTITY;
        Self {
            id: 0,
            world_id,
            position,
            rotation,
            velocity,
            force,
            mass,
            collider,
            angular_velocity,
            inv_mass: if mass > 0.0 { 1.0 / mass } else { 0.0 },
            inertia_tensor,
            inv_inertia_tensor: inertia_tensor.inverse(),
            torque: Vec3::ZERO,
            previous_position: position,
            previous_rotation: rotation,
            friction: 0.0,
            pre_solve_velocity: velocity,
            pre_solve_angular_velocity: angular_velocity,
            restitution: 0.0,
        }
    }

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
}

impl Display for RigidBody {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "RigidBody(id={}, world_id={}, position={}, orientation={}, velocity={}, force={}, mass={}, inv_mass={})",
            self.id, self.world_id, self.position, self.rotation, self.velocity, self.force, self.mass, self.inv_mass
        )
    }
}
