use log::debug;
use math::vec3::Vec3;
use spacetimedb::{reducer, ReducerContext, ScheduleAt};
use tables::{physics_rigid_bodies, PhysicsWorld, RigidBody};

mod math;
mod tables;

#[reducer(init)]
fn init(ctx: &ReducerContext) {
    let world_id = PhysicsWorld::insert(ctx, 60.0, Vec3::new(0.0, -9.81, 0.0)).id;

    RigidBody::insert(
        ctx,
        world_id,
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::ZERO,
        Vec3::ZERO,
        1.0,
    );
    RigidBody::insert(
        ctx,
        world_id,
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::ZERO,
        Vec3::ZERO,
        2.0,
    );
}

#[reducer]
fn physics_step_world(ctx: &ReducerContext, world: PhysicsWorld) {
    let delta_time = match world.scheduled_at {
        ScheduleAt::Interval(interval) => interval.to_duration_abs().as_secs_f32(),
        ScheduleAt::Time(time) => ctx.timestamp.duration_since(time).unwrap().as_secs_f32(),
    };

    for mut body in ctx.db.physics_rigid_bodies().world_id().filter(world.id) {
        if body.mass > 0.0 {
            body.force += world.gravity * body.mass;
            body.velocity += body.force / body.mass * delta_time;
            body.position += body.velocity * delta_time;
        }

        body.force = Vec3::ZERO;

        body.update(ctx);
    }
}
