use std::fmt::Display;

use bon::{builder, Builder};
use parry3d::na::Isometry3;
use spacetimedb::{table, ReducerContext, Table};

use crate::math::{Quat, Vec3};

use super::RigidBodyId;

pub type TriggerId = u64;

#[table(name = physics_triggers)]
#[derive(Builder, Debug, Clone, PartialEq)]
pub struct Trigger {
    #[builder(default = 0)]
    #[primary_key]
    #[auto_inc]
    pub id: u64,
    #[index(btree)]
    #[builder(default = 1)]
    pub world_id: u64,

    #[builder(default = Vec3::ZERO)]
    pub position: Vec3,
    #[builder(default = Quat::IDENTITY)]
    pub rotation: Quat,

    pub collider_id: u64,

    /// The entities currently inside the trigger.
    #[builder(default = Vec::new())]
    pub entities_inside: Vec<RigidBodyId>,

    /// The entities that were added to the trigger since the last update.
    #[builder(default = Vec::new())]
    pub added_entities: Vec<RigidBodyId>,

    /// The entities that were removed from the trigger since the last update.
    #[builder(default = Vec::new())]
    pub removed_entities: Vec<RigidBodyId>,
}

impl Trigger {
    pub fn insert(self, ctx: &ReducerContext) -> Self {
        ctx.db.physics_triggers().insert(self)
    }

    pub fn find(ctx: &ReducerContext, id: u64) -> Option<Self> {
        ctx.db.physics_triggers().id().find(id)
    }

    pub fn update(self, ctx: &ReducerContext) -> Self {
        ctx.db.physics_triggers().id().update(self)
    }

    pub fn delete(self, ctx: &ReducerContext) {
        ctx.db.physics_triggers().id().delete(self.id);
    }

    pub fn delete_by_id(ctx: &ReducerContext, id: u64) {
        ctx.db.physics_triggers().id().delete(id);
    }

    pub fn all(ctx: &ReducerContext, world_id: u64) -> impl Iterator<Item = Self> {
        ctx.db.physics_triggers().world_id().filter(world_id)
    }
}

impl From<Trigger> for Isometry3<f32> {
    fn from(trigger: Trigger) -> Self {
        Isometry3::from_parts(trigger.position.into(), trigger.rotation.into())
    }
}

impl From<&Trigger> for Isometry3<f32> {
    fn from(trigger: &Trigger) -> Self {
        Isometry3::from_parts(trigger.position.into(), trigger.rotation.into())
    }
}

impl Display for Trigger {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "PhysicsTrigger(id: {}, world_id: {}, position: {}, rotation: {}, collider: {})",
            self.id, self.world_id, self.position, self.rotation, self.collider_id
        )
    }
}
