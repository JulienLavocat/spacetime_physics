use std::fmt::Display;

use spacetimedb::{table, ReducerContext, ScheduleAt, Table};

use crate::{
    engine::collisions::Collider,
    math::{Transform, Vec3},
    physics_step_world,
};

#[table(name = physics_world, scheduled(physics_step_world), public)]
pub struct PhysicsWorld {
    #[primary_key]
    #[auto_inc]
    pub id: u64,
    pub scheduled_at: ScheduleAt,
    pub time_step: f32,
    pub gravity: Vec3,
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
            gravity,
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

#[table(name = physics_rigid_bodies, public)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RigidBody {
    #[primary_key]
    #[auto_inc]
    pub id: u64,
    #[index(btree)]
    pub world_id: u64, // Foreign key to PhysicsWorld
    pub transform: Transform,
    pub velocity: Vec3,
    pub force: Vec3,
    pub mass: f32,
    pub inv_mass: f32,
    pub restitution: f32,
    pub collider: Collider,
}

impl RigidBody {
    pub fn new(
        world_id: u64,
        transform: Transform,
        velocity: Vec3,
        force: Vec3,
        mass: f32,
        collider: Collider,
    ) -> Self {
        Self {
            id: 0,
            world_id,
            transform,
            velocity,
            force,
            mass,
            collider,
            inv_mass: if mass > 0.0 { 1.0 / mass } else { 0.0 },
            restitution: 0.0,
        }
    }

    pub fn test(&mut self) {
        self.mass = 1.0;
    }

    pub fn insert(self, ctx: &ReducerContext) -> Self {
        ctx.db.physics_rigid_bodies().insert(self)
    }

    pub fn update(self, ctx: &ReducerContext) -> Self {
        ctx.db.physics_rigid_bodies().id().update(self)
    }
}

impl Display for RigidBody {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "RigidBody(id={}, transform={}, velocity={}, force={}, mass={})",
            self.id, self.transform, self.velocity, self.force, self.mass
        )
    }
}
