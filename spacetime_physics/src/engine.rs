use log::debug;
use spacetimedb::{ReducerContext, ScheduleAt};

use crate::{
    collisions::{Collision, CollisionPoints},
    math::Vec3,
    tables::{physics_rigid_bodies, PhysicsWorld},
};

pub fn step_world(ctx: &ReducerContext, world: &PhysicsWorld) {
    let delta_time = match world.scheduled_at {
        ScheduleAt::Interval(interval) => interval.to_duration_abs().as_secs_f32(),
        ScheduleAt::Time(time) => ctx.timestamp.duration_since(time).unwrap().as_secs_f32(),
    };

    resolve_collisions(ctx, world);
    apply_dynamics(ctx, world, delta_time);
}

fn apply_dynamics(ctx: &ReducerContext, world: &PhysicsWorld, delta_time: f32) {
    for mut body in ctx.db.physics_rigid_bodies().world_id().filter(world.id) {
        if body.mass > 0.0 {
            body.force += world.gravity * body.mass;
            body.velocity += body.force / body.mass * delta_time;
            body.transform.position += body.velocity * delta_time;
        }

        body.force = Vec3::ZERO;

        body.update(ctx);
    }
}

fn resolve_collisions(ctx: &ReducerContext, world: &PhysicsWorld) {
    let mut collisions: Vec<Collision> = Vec::new();

    let bodies: Vec<_> = ctx
        .db
        .physics_rigid_bodies()
        .world_id()
        .filter(world.id)
        .collect();

    for (i, a) in bodies.iter().enumerate() {
        for b in bodies.iter().skip(i + 1) {
            if let Some(collision) =
                CollisionPoints::test(&a.collider, &a.transform, &b.collider, &b.transform)
            {
                debug!("Collision detected: {}", collision);
                collisions.push(Collision {
                    a: a.id,
                    b: b.id,
                    points: collision,
                });
            }
        }
    }
}
