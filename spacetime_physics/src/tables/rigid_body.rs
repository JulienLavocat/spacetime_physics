use std::fmt::Display;

use bon::{builder, Builder};
use parry3d::na::Isometry3;
use spacetimedb::{table, ReducerContext, SpacetimeType, Table};

use crate::{
    math::{Mat3, Quat, Vec3},
    Collider,
};

pub type RigidBodyId = u64;

#[derive(SpacetimeType, Debug, Clone, Copy, PartialEq)]
pub struct Restitution {
    pub coefficient: f32,
}

impl Restitution {
    pub fn new(coefficient: f32) -> Self {
        Self { coefficient }
    }

    pub fn combine(&self, other: &Self) -> Self {
        Self {
            coefficient: (self.coefficient + other.coefficient) / 2.0,
        }
    }
}

impl Default for Restitution {
    fn default() -> Self {
        Self { coefficient: 0.0 }
    }
}

#[derive(SpacetimeType, Debug, Clone, Copy, PartialEq)]
pub struct Friction {
    pub static_coefficient: f32,
    pub dynamic_coefficient: f32,
}

impl Friction {
    pub fn new(static_friction: f32, dynamic_friction: f32) -> Self {
        Self {
            static_coefficient: static_friction,
            dynamic_coefficient: dynamic_friction,
        }
    }

    pub fn combine(&self, other: &Self) -> Self {
        Self {
            static_coefficient: (self.static_coefficient + other.static_coefficient) / 2.0,
            dynamic_coefficient: (self.dynamic_coefficient + other.dynamic_coefficient) / 2.0,
        }
    }
}

impl Default for Friction {
    fn default() -> Self {
        Self {
            static_coefficient: 0.5,
            dynamic_coefficient: 0.3,
        }
    }
}

impl Display for Friction {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Friction(static={}, dynamic={})",
            self.static_coefficient, self.dynamic_coefficient
        )
    }
}

#[derive(SpacetimeType, Debug, Clone, Copy, PartialEq, Default)]
pub enum RigidBodyType {
    Static,
    #[default]
    Dynamic,
    Kinematic,
}

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

    #[builder(default = Vec3::ZERO)]
    pub force: Vec3,

    #[builder(default = 1.0)]
    pub mass: f32,
    #[builder(skip = if mass > 0.0 { 1.0 / mass } else { 0.0 })]
    pub inv_mass: f32,

    pub collider: Collider,

    #[builder(default = collider.inertia_tensor(mass))]
    pub inertia_tensor: Mat3,
    #[builder(skip = inertia_tensor.inverse())]
    pub inv_inertia_tensor: Option<Mat3>,

    #[builder(default = Vec3::ZERO)]
    pub torque: Vec3,

    #[builder(default = Friction::default())]
    pub friction: Friction,
    #[builder(default = Restitution::default())]
    pub restitution: Restitution,

    #[builder(skip = linear_velocity)]
    pub pre_solve_linear_velocity: Vec3,
    #[builder(skip = angular_velocity)]
    pub pre_solve_angular_velocity: Vec3,

    #[builder(skip = position)]
    pub previous_position: Vec3,
    #[builder(skip = rotation)]
    pub previous_rotation: Quat,

    #[builder(default = RigidBodyType::default())]
    pub body_type: RigidBodyType,
}

impl RigidBody {
    pub fn insert(self, ctx: &ReducerContext) -> Self {
        ctx.db.physics_rigid_bodies().insert(self)
    }

    pub fn all(ctx: &ReducerContext, world_id: u64) -> Vec<Self> {
        let mut bodies = ctx
            .db
            .physics_rigid_bodies()
            .world_id()
            .filter(world_id)
            .collect::<Vec<_>>();
        bodies.sort_by_key(|body| body.id);
        bodies
    }

    pub fn update(self, ctx: &ReducerContext) -> Self {
        ctx.db.physics_rigid_bodies().id().update(self)
    }

    pub fn delete(&self, ctx: &ReducerContext) {
        ctx.db.physics_rigid_bodies().id().delete(self.id);
    }

    pub fn effective_inverse_mass(&self) -> Vec3 {
        // TODO: Take into account locked axes
        Vec3::splat(self.inv_mass)
    }

    pub fn effective_inverse_inertia(&self) -> Mat3 {
        // TODO: Take into account locked axes
        match self.inv_inertia_tensor {
            Some(inv) => {
                let r = self.rotation.to_mat3();
                r * inv * r.transpose()
            }
            None => Mat3::ZERO, // Static body
        }
    }

    pub fn is_dynamic(&self) -> bool {
        self.body_type == RigidBodyType::Dynamic
    }

    pub fn is_kinematic(&self) -> bool {
        self.body_type == RigidBodyType::Kinematic
    }

    pub fn is_dirty(&self) -> bool {
        self.position != self.previous_position || self.rotation != self.previous_rotation
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

impl From<RigidBody> for Isometry3<f32> {
    fn from(value: RigidBody) -> Self {
        Isometry3::from_parts(value.position.into(), value.rotation.into())
    }
}

impl From<&RigidBody> for Isometry3<f32> {
    fn from(value: &RigidBody) -> Self {
        Isometry3::from_parts(value.position.into(), value.rotation.into())
    }
}
