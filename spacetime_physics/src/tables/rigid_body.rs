use std::fmt::Display;

use bon::{builder, Builder};
use parry3d::na::Isometry3;
use spacetimedb::{table, ReducerContext, SpacetimeType, Table};

use crate::math::{Quat, Vec3};

pub type RigidBodyId = u64;

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

    #[builder(default = Vec3::ZERO)]
    pub torque: Vec3,

    #[builder(default = RigidBodyType::default())]
    pub body_type: RigidBodyType,

    pub collider_id: u64,
    pub properties_id: u64,
}

impl RigidBody {
    pub fn insert(self, ctx: &ReducerContext) -> Self {
        ctx.db.physics_rigid_bodies().insert(self)
    }

    pub fn find(ctx: &ReducerContext, id: u64) -> Option<Self> {
        ctx.db.physics_rigid_bodies().id().find(id)
    }

    pub fn all(ctx: &ReducerContext, world_id: u64) -> impl Iterator<Item = RigidBody> {
        ctx.db.physics_rigid_bodies().world_id().filter(world_id)
    }

    pub fn update(self, ctx: &ReducerContext) -> Self {
        ctx.db.physics_rigid_bodies().id().update(self)
    }

    pub fn delete(&self, ctx: &ReducerContext) {
        ctx.db.physics_rigid_bodies().id().delete(self.id);
    }

    pub fn delte_by_id(ctx: &ReducerContext, id: u64) {
        ctx.db.physics_rigid_bodies().id().delete(id);
    }

    pub fn is_dynamic(&self) -> bool {
        self.body_type == RigidBodyType::Dynamic
    }

    pub fn is_kinematic(&self) -> bool {
        self.body_type == RigidBodyType::Kinematic
    }
}

impl Display for RigidBody {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "RigidBody(id={}, world_id={}, position={}, orientation={}, velocity={}, force={})",
            self.id, self.world_id, self.position, self.rotation, self.linear_velocity, self.force
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
