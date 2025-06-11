use std::fmt::Display;

use spacetimedb::{table, ReducerContext, ScheduleAt, Table};

use crate::{math::vec3::Vec3, physics_step_world};

#[table(name = physics_world, scheduled(physics_step_world), public)]
pub struct PhysicsWorld {
    #[primary_key]
    #[auto_inc]
    pub id: u64,
    pub scheduled_at: ScheduleAt,
    pub gravity: Vec3,
}

impl PhysicsWorld {
    pub fn insert(ctx: &ReducerContext, tps: f32, gravity: Vec3) -> Self {
        let scheduled_at = ScheduleAt::Interval(spacetimedb::TimeDuration::from_duration(
            std::time::Duration::from_secs_f32(1.0 / tps),
        ));

        ctx.db.physics_world().insert(Self {
            id: 0,
            scheduled_at,
            gravity,
        })
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

#[table(name = physics_rigid_bodies, public)]
pub struct RigidBody {
    #[primary_key]
    #[auto_inc]
    pub id: u64,
    #[index(btree)]
    pub world_id: u64, // Foreign key to PhysicsWorld
    pub position: Vec3,
    pub velocity: Vec3,
    pub force: Vec3,
    pub mass: f32,
}

impl RigidBody {
    pub fn insert(
        ctx: &ReducerContext,
        world_id: u64,
        position: Vec3,
        velocity: Vec3,
        force: Vec3,
        mass: f32,
    ) -> Self {
        ctx.db.physics_rigid_bodies().insert(Self {
            id: 0,
            world_id,
            position,
            velocity,
            force,
            mass,
        })
    }

    pub fn update(self, ctx: &ReducerContext) -> Self {
        ctx.db.physics_rigid_bodies().id().update(self)
    }
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
