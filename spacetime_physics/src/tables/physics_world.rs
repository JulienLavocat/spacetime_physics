use std::{fmt::Display, time::Duration};

use bon::{builder, Builder};
use spacetimedb::{table, ReducerContext, ScheduleAt, Table, TimeDuration};

use crate::math::Vec3;

pub type PhysicsWorldId = u64;

pub fn schedule_physics_step_world(world: &PhysicsWorld) -> ScheduleAt {
    ScheduleAt::Interval(TimeDuration::from_duration(Duration::from_secs_f32(
        1.0 / world.ticks_per_second,
    )))
}

#[table(name = physics_world, public)]
#[derive(Builder, Debug, Clone, Copy, PartialEq)]
#[builder(derive(Debug, Clone))]
pub struct PhysicsWorld {
    #[primary_key]
    #[auto_inc]
    #[builder(default = 0)]
    /// The unique identifier for the physics world. This is automatically incremented by the database.
    /// It is used to be able to have multiple separated simulations running at the same time.
    pub id: u64,
    #[builder(default = 60.0)]
    /// The number of physics updates per second. This determines how often the physics world is
    /// updated. A common value is 60, which means the physics world will update 60 times per second.
    pub ticks_per_second: f32,
    #[builder(default = 1.0 / 60.0)]
    /// The time step by which the physics world is updated. This is the duration of each physics step.
    /// This is different from the ticks per second, as it represents the actual time duration of each step.
    pub time_step: f32,
    #[builder(default = 20)]
    /// The number of sub-steps to perform in each physics step. This allows for more accurate
    /// simulation by breaking down the physics step into smaller increments. A value of 20 is common,
    pub sub_step: u32,
    #[builder(default = Vec3::new(0.0, -9.81, 0.0))]
    /// The gravity vector applied to the physics world. This is typically set to a downward
    /// vector like (0.0, -9.81, 0.0) to simulate Earth's gravity.
    pub gravity: Vec3,
    #[builder(default = 1e-3)]
    /// The precision of the physics simulation, used to determine how close objects need to be
    /// to collide.
    pub precision: f32,
    #[builder(default = 1)]
    /// The number of position iterations to perform during the physics step. This should be as low
    /// as possible while still achieving stable results, a value of 1 is usually sufficient.
    pub position_iterations: u32,

    /// If true, the physics world will log detailed debug information to the console. This is very
    /// verbose and should only be used for debugging purposes.
    #[builder(default = false)]
    pub debug: bool,

    /// If true, the physics world will log the time taken for each physics step to the console.
    #[builder(default = false)]
    pub debug_time: bool,

    /// If true, the physics world will log the number of triggers enter / exit events to the console.
    #[builder(default = false)]
    pub debug_triggers: bool,
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
            "PhysicsWorld(id={}, tps={}, time_step={}, sub_step={}, gravity={}, precision={}, position_iterations={})",
            self.id, self.ticks_per_second, self.time_step, self.sub_step, self.gravity, self.precision, self.position_iterations
        )
    }
}
