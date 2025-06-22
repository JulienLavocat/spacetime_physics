use std::fmt::Display;

use spacetimedb::{table, ReducerContext, ScheduleAt, Table};

use crate::{math::Vec3, physics_step_world};

#[table(name = physics_world, scheduled(physics_step_world), public)]
pub struct PhysicsWorld {
    #[primary_key]
    #[auto_inc]
    pub id: u64,
    pub scheduled_at: ScheduleAt,
    pub time_step: f32,
    pub sub_step: u32,
    pub gravity: Vec3,
    pub precision: f32,
    pub sleep_time: f32,
    pub sleep_threshold: f32,
}

impl PhysicsWorld {
    pub fn new(tps: f32, gravity: Vec3) -> Self {
        let scheduled_at = ScheduleAt::Interval(spacetimedb::TimeDuration::from_duration(
            std::time::Duration::from_secs_f32(1.0 / tps),
        ));

        Self {
            id: 0,
            scheduled_at,
            time_step: 1.0 / tps,
            sub_step: 4,
            gravity,
            precision: 1e-3,
            sleep_time: 1.0,
            sleep_threshold: 1e-4,
        }
    }

    pub fn insert(self, ctx: &ReducerContext) -> Self {
        ctx.db.physics_world().insert(self)
    }
}

impl Display for PhysicsWorld {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "PhysicsWorld(id={}, scheduled_at={:?}, gravity={})",
            self.id, self.scheduled_at, self.gravity
        )
    }
}
