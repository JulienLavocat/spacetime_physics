use spacetimedb::log_stopwatch::LogStopwatch as SpacetimeLogStopwatch;

use crate::PhysicsWorld;

pub struct LogStopwatch {
    sw: Option<SpacetimeLogStopwatch>,
}

impl LogStopwatch {
    pub fn new(world: &PhysicsWorld, name: &str) -> Self {
        let sw = if world.debug_time {
            Some(SpacetimeLogStopwatch::new(name))
        } else {
            None
        };
        Self { sw }
    }

    pub fn end(self) {
        if let Some(sw) = self.sw {
            sw.end();
        }
    }
}
