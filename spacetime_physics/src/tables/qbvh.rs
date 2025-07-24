use spacetimedb::{table, ReducerContext};

#[table(name = physics_qbvh)]
pub struct QbvhCache {
    #[primary_key]
    pub world_id: u64,
    pub bytes: Vec<u8>,
}

impl QbvhCache {
    pub fn upsert(ctx: &ReducerContext, world_id: u64, bytes: Vec<u8>) -> QbvhCache {
        ctx.db
            .physics_qbvh()
            .world_id()
            .insert_or_update(QbvhCache { world_id, bytes })
    }

    pub fn get(ctx: &ReducerContext, world_id: u64) -> Option<QbvhCache> {
        ctx.db.physics_qbvh().world_id().find(world_id)
    }
}
