use bon::{builder, Builder};
use parry3d::na::Isometry3;
use spacetimedb::{table, ReducerContext, Table};

use crate::{
    math::{Quat, Vec3},
    Collider,
};

#[table(name = physics_triggers)]
#[derive(Builder, Debug, Clone, Copy, PartialEq)]
pub struct PhysicsTrigger {
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

    pub collider: Collider,
}

impl PhysicsTrigger {
    pub fn insert(self, ctx: &ReducerContext) -> Self {
        ctx.db.physics_triggers().insert(self)
    }

    pub fn all(world_id: u64, ctx: &ReducerContext) -> Vec<Self> {
        ctx.db
            .physics_triggers()
            .world_id()
            .filter(world_id)
            .collect()
    }
}

impl From<PhysicsTrigger> for Isometry3<f32> {
    fn from(trigger: PhysicsTrigger) -> Self {
        Isometry3::from_parts(trigger.position.into(), trigger.rotation.into())
    }
}

impl From<&PhysicsTrigger> for Isometry3<f32> {
    fn from(trigger: &PhysicsTrigger) -> Self {
        Isometry3::from_parts(trigger.position.into(), trigger.rotation.into())
    }
}
