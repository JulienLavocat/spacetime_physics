use std::{fmt::Display, time::Duration};

use bon::{builder, Builder};
use spacetimedb::{table, ReducerContext, ScheduleAt, Table, TimeDuration};

use crate::{math::Vec3, physics_step_world};

fn schedule_physics_step_world(tps: f32) -> ScheduleAt {
    ScheduleAt::Interval(TimeDuration::from_duration(Duration::from_secs_f32(
        1.0 / tps,
    )))
}

#[table(name = physics_world, scheduled(physics_step_world), public)]
#[derive(Builder, Debug, Clone, Copy, PartialEq)]
#[builder(derive(Debug, Clone))]
pub struct PhysicsWorld {
    #[primary_key]
    #[auto_inc]
    #[builder(default = 0)]
    pub id: u64,
    #[builder(default = 60.0)]
    pub ticks_per_second: f32,
    #[builder(skip = schedule_physics_step_world(ticks_per_second))]
    pub scheduled_at: ScheduleAt,
    #[builder(default = 1.0 / 60.0)]
    pub time_step: f32,
    #[builder(default = 4)]
    pub sub_step: u32,
    #[builder(default = Vec3::new(0.0, -9.81, 0.0))]
    pub gravity: Vec3,
    #[builder(default = 1e-3)]
    pub precision: f32,
    #[builder(default = 1)]
    pub position_iterations: u32,
}

impl PhysicsWorld {
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
