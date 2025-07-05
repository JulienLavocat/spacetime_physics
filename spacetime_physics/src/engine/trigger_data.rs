use std::collections::{HashMap, HashSet};

use parry3d::na::Isometry3;
use spacetimedb::ReducerContext;

use crate::{Collider, ColliderId, RigidBodyId, ShapeWrapper, Trigger, TriggerId};

#[derive(Debug)]
pub struct TriggerData {
    pub shape: ShapeWrapper,
    pub trigger_id: TriggerId,
    pub collider_id: ColliderId,
    pub world_id: u64,
    pub isometry: Isometry3<f32>,
    pub current_entities_inside: HashSet<RigidBodyId>,
    pub new_entities_inside: HashSet<RigidBodyId>,
    pub added_entities: HashSet<RigidBodyId>,
    pub removed_entities: HashSet<RigidBodyId>,
}

impl TriggerData {
    pub fn new(trigger: &Trigger, collider: &Collider) -> Self {
        Self {
            collider_id: collider.id,
            trigger_id: trigger.id,
            world_id: trigger.world_id,
            shape: ShapeWrapper::from(collider),
            isometry: Isometry3::from_parts(trigger.position.into(), trigger.rotation.into()),
            current_entities_inside: trigger.entities_inside.iter().copied().collect(),
            added_entities: HashSet::new(),
            removed_entities: HashSet::new(),
            new_entities_inside: HashSet::new(),
        }
    }

    pub fn collect(
        ctx: &ReducerContext,
        world_id: u64,
        colliders: &HashMap<ColliderId, Collider>,
    ) -> Vec<Self> {
        Trigger::all(ctx, world_id)
            .map(|trigger| {
                let collider = colliders.get(&trigger.collider_id).unwrap();
                TriggerData::new(&trigger, collider)
            })
            .collect()
    }

    pub fn update(&self, ctx: &ReducerContext) {
        Trigger {
            id: self.trigger_id,
            world_id: self.world_id,
            position: self.isometry.translation.vector.into(),
            rotation: self.isometry.rotation.into(),
            collider_id: self.collider_id,
            entities_inside: self.current_entities_inside.iter().cloned().collect(),
            added_entities: self.added_entities.iter().cloned().collect(),
            removed_entities: self.removed_entities.iter().cloned().collect(),
        }
        .update(ctx);
    }
}
