use spacetimedb::ReducerContext;

use crate::tables::{physics_rigid_bodies, PhysicsWorld, RigidBody};

pub mod collisions;

pub fn step_world(ctx: &ReducerContext, world: &PhysicsWorld) {
    let mut bodies = ctx
        .db
        .physics_rigid_bodies()
        .world_id()
        .filter(world.id)
        .collect::<Vec<_>>();
    bodies.sort_by_key(|body| body.id);
    let bodies = bodies.as_mut_slice();

    let delta_time = world.time_step / world.sub_step as f32;

    for _ in 0..world.sub_step {
        run_step(bodies, world, delta_time);
    }

    for body in bodies {
        body.update(ctx);
    }
}

fn run_step(bodies: &mut [RigidBody], world: &PhysicsWorld, delta_time: f32) {}
