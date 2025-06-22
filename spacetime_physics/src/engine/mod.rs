use collisions2::Collision;
use log::debug;
use solvers::{impulse::ImpulseSolver, position::PositionSolver, Solver};
use spacetimedb::ReducerContext;

use crate::{
    math::Vec3,
    tables::{physics_rigid_bodies, PhysicsWorld, RigidBody},
};

pub mod collisions;
pub mod collisions2;
mod solvers;

pub fn step_world(ctx: &ReducerContext, world: &PhysicsWorld) {
    let mut bodies = ctx
        .db
        .physics_rigid_bodies()
        .world_id()
        .filter(world.id)
        .collect::<Vec<_>>();
    bodies.sort_by_key(|body| body.id);
    let bodies = bodies.as_mut_slice();
    debug!("--------------------------------");

    run_step(bodies, world, world.time_step);

    for body in bodies {
        if body.id == 1 {
            debug!(
                "Updating body {}: position = {:?}, velocity = {:?}, force = {:?}",
                body.id, body.transform.position, body.velocity, body.force
            );
        }
        body.update(ctx);
    }
}

fn run_step(bodies: &mut [RigidBody], world: &PhysicsWorld, delta_time: f32) {
    apply_forces(world, bodies);
    integrate_velocity(bodies, delta_time);
    check_sleeping(world, bodies, delta_time);

    let collisions = detect_collisions(world, bodies);
    for collision in &collisions {
        debug!(
            "Detected collision between bodies {} and {}: {:?}",
            collision.a, collision.b, collision.points
        );
    }
    ImpulseSolver::solve(world, &collisions, bodies, delta_time);
    PositionSolver::solve(world, &collisions, bodies, delta_time);

    integrate_position(bodies, delta_time);
}

fn apply_forces(world: &PhysicsWorld, bodies: &mut [RigidBody]) {
    for body in bodies {
        if body.inv_mass == 0.0 || body.is_sleeping {
            continue; // Skip static or sleeping bodies
        }

        debug!(
            "Applying forces for body {}: mass = {}, gravity = {:?}",
            body.id, body.mass, world.gravity
        );
        body.force += world.gravity * body.mass;
    }
}

fn integrate_velocity(bodies: &mut [RigidBody], delta_time: f32) {
    for body in bodies {
        if body.inv_mass == 0.0 || body.is_sleeping {
            continue; // Skip static or sleeping bodies
        }

        let acceleration = body.force * body.inv_mass;
        body.velocity += acceleration * delta_time;

        body.force = Vec3::ZERO;
        if body.id == 1 {
            debug!(
            "Integrating velocity for body {}: force = {:?}, inv_mass = {}, delta_time = {}, new_velocity = {:?}",
            body.id, body.force, body.inv_mass, delta_time, body.velocity
        );
        }
    }
}

fn detect_collisions(world: &PhysicsWorld, bodies: &[RigidBody]) -> Vec<Collision> {
    let mut collisions: Vec<Collision> = Vec::new();

    for (i, a) in bodies.iter().enumerate() {
        for b in bodies.iter().skip(i + 1) {
            if a.id == b.id {
                continue; // Skip self-collision and different worlds
            }

            if a.is_sleeping && b.is_sleeping {
                continue;
            }

            if let Some(collision) = collisions2::test_collision(
                &a.transform,
                &a.collider,
                &b.transform,
                &b.collider,
                world.precision,
            ) {
                collisions.push(Collision {
                    world_id: a.world_id,
                    a: a.id,
                    b: b.id,
                    points: collision,
                });
            }
        }
    }

    collisions
}

fn integrate_position(bodies: &mut [RigidBody], delta_time: f32) {
    for body in bodies {
        if body.inv_mass == 0.0 || body.is_sleeping {
            continue; // Skip static or sleeping bodies
        }

        if body.id == 1 {
            debug!(
                "Integrating position for body {} at {}: velocity = {:?}, delta_time = {}",
                body.id, body.transform.position, body.velocity, delta_time
            );
        }

        body.transform.position += body.velocity * delta_time;
    }
}

fn check_sleeping(world: &PhysicsWorld, bodies: &mut [RigidBody], delta_time: f32) {
    for body in bodies {
        if body.inv_mass == 0.0 || body.is_sleeping {
            continue; // Skip static or sleeping bodies
        }

        if body.velocity.length_squared() < world.sleep_threshold {
            body.sleep_timer += delta_time;
            if body.sleep_timer >= world.sleep_time {
                // Sleep after 1 second of inactivity
                body.is_sleeping = true;
                body.velocity = Vec3::ZERO; // Stop movement
                debug!("Body {} is now sleeping", body.id);
            }
        } else {
            body.sleep_timer = 0.0; // Reset sleep timer if moving
        }
    }
}
