use std::fmt::Display;

use spacetimedb::{table, ScheduleAt};

use crate::{math::vec3::Vec3, physics_step_world};

#[table(name = physics_world, scheduled(physics_step_world))]
pub struct PhysicsWorld {
    #[primary_key]
    #[auto_inc]
    pub id: u64,
    pub scheduled_at: ScheduleAt,
    pub gravity: Vec3,
}

#[table(name = physics_rigid_bodies)]
pub struct RigidBody {
    #[primary_key]
    #[auto_inc]
    pub id: u64,
    pub position: Vec3,
    pub velocity: Vec3,
    pub force: Vec3,
    pub mass: f32,
}

impl Display for RigidBody {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "RigidBody(id={}, position={}, velocity={}, force={}, mass={})",
            self.id, self.position, self.velocity, self.force, self.mass
        )
    }
}
