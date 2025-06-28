use std::{
    collections::{HashMap, HashSet},
    fmt::Display,
};

use bon::Builder;
use spacetimedb::{table, ReducerContext, Table};

use super::PhysicsWorldId;

pub type PhysicsTriggerId = u64;
pub type PhysicsEntityId = u64;
pub type PhysicsTriggerEntitiesMap = HashMap<PhysicsTriggerId, HashSet<PhysicsTriggerEntity>>;

#[table(name = physics_trigger_entities)]
#[derive(Builder, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct PhysicsTriggerEntity {
    #[index(btree)]
    pub trigger_id: u64,

    #[index(btree)]
    pub world_id: u64,

    #[index(btree)]
    pub entity_id: u64,
}

impl PhysicsTriggerEntity {
    pub fn all(ctx: &ReducerContext, world_id: PhysicsWorldId) -> PhysicsTriggerEntitiesMap {
        let mut map = HashMap::new();
        ctx.db
            .physics_trigger_entities()
            .world_id()
            .filter(world_id)
            .for_each(|te| {
                map.entry(te.trigger_id)
                    .or_insert_with(HashSet::new)
                    .insert(te);
            });

        map
    }

    pub fn insert(self, ctx: &ReducerContext) -> Self {
        ctx.db.physics_trigger_entities().insert(self)
    }
}

impl Display for PhysicsTriggerEntity {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "PhysicsTriggerEntity(trigger_id: {}, world_id: {}, entity_id: {})",
            self.trigger_id, self.world_id, self.entity_id
        )
    }
}
