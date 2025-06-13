use collisions::{Collision, CollisionPoints};
use log::{debug, trace};
use solvers::{impulse::ImpulseSolver, position::PositionSolver, Solver};
use spacetimedb::ReducerContext;

use crate::{
    math::Vec3,
    tables::{physics_rigid_bodies, PhysicsWorld, RigidBody},
};

pub mod collisions;
mod solvers;

const SLEEP_VELOCITY_SQ_THRESHOLD: f32 = 1e-4;
const SLEEP_TIME_THRESHOLD: f32 = 0.5;

pub fn step_world(ctx: &ReducerContext, world: &PhysicsWorld) {
    // TODO: Substep and accumulator for delta time
    let delta_time = world.time_step / world.sub_step as f32;

    let mut bodies = ctx
        .db
        .physics_rigid_bodies()
        .world_id()
        .filter(world.id)
        .collect::<Vec<_>>();
    bodies.sort_by_key(|body| body.id);
    let bodies = bodies.as_mut_slice();

    for i in 0..world.sub_step {
        trace!("Running physics step {} for world: {}", i, world.id);
        run_step(bodies, world, delta_time);
    }

    for body in bodies {
        debug!(
            "Updating body {}: position = {:?}, velocity = {:?}, force = {:?}",
            body.id, body.transform.position, body.velocity, body.force
        );
        body.update(ctx);
    }
}

fn run_step(bodies: &mut [RigidBody], world: &PhysicsWorld, delta_time: f32) {
    apply_forces(world, bodies);

    for body in bodies.iter_mut() {
        body.last_contact_normal = None;
    }

    integrate_velocity(bodies, delta_time);

    let collisions = detect_collisions(bodies);
    PositionSolver::solve(world, &collisions, bodies, delta_time);
    ImpulseSolver::solve(world, &collisions, bodies, delta_time);

    for body in bodies.iter_mut() {
        if body.inv_mass == 0.0 {
            continue;
        }

        let slow_enough = body.velocity.length_squared() < SLEEP_VELOCITY_SQ_THRESHOLD;

        if let Some(contact_normal) = body.last_contact_normal {
            let gravity_dir = world.gravity.normalize();
            let aligned_against_gravity = contact_normal.dot(gravity_dir) < -0.9;

            if aligned_against_gravity && slow_enough {
                body.sleep_timer += delta_time;
                if body.sleep_timer > SLEEP_TIME_THRESHOLD {
                    body.velocity = Vec3::ZERO;
                    body.force = Vec3::ZERO;
                    body.is_sleeping = true;
                    continue; // skip snapping, we’re done
                }
            } else {
                body.sleep_timer = 0.0;
            }
        } else {
            body.sleep_timer = 0.0;
        }
    }

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
        debug!(
            "Integrating velocity for body {}: force = {:?}, inv_mass = {}, delta_time = {}, new_velocity = {:?}",
            body.id, body.force, body.inv_mass, delta_time, body.velocity
        );
    }
}

fn detect_collisions(bodies: &[RigidBody]) -> Vec<Collision> {
    let mut collisions: Vec<Collision> = Vec::new();

    for (i, a) in bodies.iter().enumerate() {
        for b in bodies.iter().skip(i + 1) {
            if a.id == b.id {
                continue; // Skip self-collision and different worlds
            }

            if let Some(collision) =
                CollisionPoints::test(&a.collider, &a.transform, &b.collider, &b.transform)
            {
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

        debug!(
            "Integrating position for body {} at {}: velocity = {:?}, delta_time = {}",
            body.id, body.transform.position, body.velocity, delta_time
        );

        body.transform.position += body.velocity * delta_time;
    }
}
