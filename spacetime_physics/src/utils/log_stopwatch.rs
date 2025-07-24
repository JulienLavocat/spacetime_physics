use spacetimedb::log_stopwatch::LogStopwatch as SpacetimeLogStopwatch;

use crate::PhysicsWorld;

#[cfg(not(test))]
pub struct LogStopwatch {
    sw: Option<SpacetimeLogStopwatch>,
}
#[cfg(test)]
pub struct LogStopwatch {}

impl LogStopwatch {
    pub fn new(world: &PhysicsWorld, name: &str) -> Self {
        #[cfg(not(test))]
        {
            let sw = if world.debug_time {
                Some(SpacetimeLogStopwatch::new(name))
            } else {
                None
            };

            #[cfg(test)]
            let sw = None;

            Self { sw }
        }
        #[cfg(test)]
        {
            Self {}
        }
    }

    pub fn end(self) {
        #[cfg(not(test))]
        {
            if let Some(sw) = self.sw {
                sw.end();
            }
        }
        #[cfg(test)]
        {
            // In test mode, we do not end the stopwatch
        }
    }
}
