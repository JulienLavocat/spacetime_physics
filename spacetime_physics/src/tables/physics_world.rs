use std::{fmt::Display, time::Duration};

use bon::{builder, Builder};
use spacetimedb::{table, ReducerContext, ScheduleAt, Table};

use crate::{math::Vec3, utils::LogStopwatch};

pub type PhysicsWorldId = u64;

pub fn schedule_physics_tick(world: &PhysicsWorld) -> ScheduleAt {
    let duration = Duration::from_secs_f32(1.0 / world.ticks_per_second);
    duration.into()
}

#[table(name = physics_world, public)]
#[derive(Builder, Debug, Clone, Copy, PartialEq)]
#[builder(derive(Debug, Clone))]
pub struct PhysicsWorld {
    #[primary_key]
    #[auto_inc]
    /// The unique identifier for the physics world. This is automatically incremented by the database.
    /// It is used to be able to have multiple separated simulations running at the same time.
    #[builder(default = 0)]
    pub id: u64,

    /// The number of physics updates per second. This determines how often the physics world is
    /// updated. A common value is 60, which means the physics world will update 60 times per second.
    #[builder(default = 60.0)]
    pub ticks_per_second: f32,

    /// The time step by which the physics world is updated. This is the duration of each physics step.
    /// This is different from the ticks per second, as it represents the actual time duration of each step.
    #[builder(default = 1.0 / 60.0)]
    pub time_step: f32,

    /// The number of sub-steps to perform in each physics step. This allows for more accurate
    /// simulation by breaking down the physics step into smaller increments. A value of 20 is common,
    #[builder(default = 20)]
    pub sub_step: u32,

    /// The gravity vector applied to the physics world. This is typically set to a downward
    /// vector like (0.0, -9.81, 0.0) to simulate Earth's gravity.
    #[builder(default = Vec3::new(0.0, -9.81, 0.0))]
    pub gravity: Vec3,

    /// The precision of the physics simulation, used to determine how close objects need to be
    /// to collide.
    #[builder(default = 1e-3)]
    pub precision: f32,

    /// The number of position iterations to perform during the physics step. This should be as low
    /// as possible while still achieving stable results, a value of 1 is usually sufficient.
    #[builder(default = 1)]
    pub position_iterations: u32,

    /// The dilation factor for the QBVH (Quantized Bounding Volume Hierarchy) used for collision detection.
    /// This factor determines how much the bounding volumes are expanded to account for movement
    /// and ensure that fast-moving objects are still detected for collisions.
    #[builder(default = 0.001)]
    pub qvbh_dilation_factor: f32,

    /// How many units are in one meter in the physics world. This is used to convert between
    /// game units and real-world units.
    /// For example, if 100px = 1m in the game, then this value should be set to 100.0.
    #[builder(default = 1.0)]
    pub length_unit: f32,

    /// The maximal distance separating two objects that will generate predictive contacts.
    #[builder(default = 0.002)]
    pub normalized_prediction_distance: f32,

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

    /// If true, the physics world will log the collisions detected during the broad phase to the console.
    #[builder(default = false)]
    pub debug_broad_phase: bool,

    /// If true, the physics world will log the collisions detected during the narrow phase to the console.
    #[builder(default = false)]
    pub debug_narrow_phase: bool,

    /// If true, the physics world will log both broad and narrow phase collision information to the console.
    #[builder(default = false)]
    pub debug_broad_narrow_phase: bool,

    /// If true, the physics world will log the raycasts hits to the console.
    #[builder(default = false)]
    pub debug_raycasts: bool,

    /// If true, the physics world will log the constraints being solved to the console.
    #[builder(default = false)]
    pub debug_constraints: bool,

    /// If true, the physics world will log the sub-steps being performed to the console.
    #[builder(default = false)]
    pub debug_substep: bool,
}

impl PhysicsWorld {
    pub fn insert(self, ctx: &ReducerContext) -> Self {
        ctx.db.physics_world().insert(self)
    }

    pub fn find(ctx: &ReducerContext, id: PhysicsWorldId) -> Option<Self> {
        ctx.db.physics_world().id().find(id)
    }

    pub fn update(self, ctx: &ReducerContext) -> Self {
        ctx.db.physics_world().id().update(self)
    }

    pub fn delete(&self, ctx: &ReducerContext) {
        ctx.db.physics_world().id().delete(self.id);
    }

    pub fn delete_by_id(ctx: &ReducerContext, id: PhysicsWorldId) {
        ctx.db.physics_world().id().delete(id);
    }

    pub fn prediction_distance(&self) -> f32 {
        self.normalized_prediction_distance * self.length_unit
    }

    pub fn debug_broad_phase(&self) -> bool {
        self.debug || self.debug_broad_phase || self.debug_broad_narrow_phase
    }

    pub fn debug_narrow_phase(&self) -> bool {
        self.debug || self.debug_narrow_phase || self.debug_broad_narrow_phase
    }

    pub fn debug_time(&self) -> bool {
        self.debug || self.debug_time
    }

    pub fn debug_triggers(&self) -> bool {
        self.debug || self.debug_triggers
    }

    pub fn debug_raycasts(&self) -> bool {
        self.debug || self.debug_raycasts
    }

    pub fn debug_constraints(&self) -> bool {
        self.debug || self.debug_constraints
    }

    pub fn debug_substep(&self) -> bool {
        self.debug || self.debug_substep
    }

    pub fn stopwatch(&self, name: &str) -> LogStopwatch {
        LogStopwatch::new(self, &format!("world_{}_{}", self.id, name))
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
