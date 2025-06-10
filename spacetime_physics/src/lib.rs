use std::{clone, time::Duration};

use log::info;
use math::vec3::Vec3;
use spacetimedb::{reducer, ReducerContext, ScheduleAt, Table, TimeDuration};
use tables::{physics_rigid_bodies, physics_world, PhysicsWorld, RigidBody};

mod math;
mod tables;

#[reducer(init)]
fn init(ctx: &ReducerContext) {
    ctx.db.physics_world().insert(PhysicsWorld {
        id: 0,
        scheduled_at: ScheduleAt::Interval(TimeDuration::from_duration(Duration::from_millis(
            1000 / 60,
        ))),
        gravity: Vec3::new(0.0, -9.81, 0.0),
    });

    ctx.db.physics_rigid_bodies().insert(RigidBody {
        id: 0,
        position: Vec3::ZERO,
        velocity: Vec3::ZERO,
        force: Vec3::ZERO,
        mass: 1.0,
    });

    ctx.db.physics_rigid_bodies().insert(RigidBody {
        id: 0,
        position: Vec3::ZERO,
        velocity: Vec3::ZERO,
        force: Vec3::ZERO,
        mass: 2.0,
    });
}

#[reducer]
fn physics_step_world(ctx: &ReducerContext, world: PhysicsWorld) {
    let delta_time = match world.scheduled_at {
        ScheduleAt::Interval(interval) => interval.to_duration_abs().as_secs_f32(),
        ScheduleAt::Time(time) => ctx.timestamp.duration_since(time).unwrap().as_secs_f32(),
    };

    for mut body in ctx.db.physics_rigid_bodies().iter() {
        if body.mass > 0.0 {
            body.force += world.gravity * body.mass;
            body.velocity += body.force / body.mass * delta_time;
            body.position += body.velocity * delta_time;
        }

        body.force = Vec3::ZERO;

        info!("Updated body @{}s: {}", delta_time, body);
        ctx.db.physics_rigid_bodies().id().update(body);
    }
}
