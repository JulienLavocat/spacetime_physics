use std::fmt::Display;

use bon::{builder, Builder};
use parry3d::na::Isometry3;
use spacetimedb::{table, ReducerContext, Table};

use crate::math::{Quat, Vec3};

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

    pub collider_id: u64,
}

impl PhysicsTrigger {
    pub fn insert(self, ctx: &ReducerContext) -> Self {
        ctx.db.physics_triggers().insert(self)
    }

    pub fn all(ctx: &ReducerContext, world_id: u64) -> Vec<Self> {
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

impl Display for PhysicsTrigger {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "PhysicsTrigger(id: {}, world_id: {}, position: {}, rotation: {}, collider: {})",
            self.id, self.world_id, self.position, self.rotation, self.collider_id
        )
    }
}
