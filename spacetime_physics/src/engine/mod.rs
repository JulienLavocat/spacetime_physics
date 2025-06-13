use collisions::{Collision, CollisionPoints};
use log::debug;
use solvers::{impulse::ImpulseSolver, position::PositionSolver, Solver};
use spacetimedb::ReducerContext;

use crate::{
    math::Vec3,
    tables::{physics_rigid_bodies, PhysicsWorld, RigidBody},
};

pub mod collisions;
mod solvers;

pub fn step_world(ctx: &ReducerContext, world: &PhysicsWorld) {
    // TODO: Substep and accumulator for delta time
    let delta_time = world.time_step;

    let mut bodies = ctx
        .db
        .physics_rigid_bodies()
        .world_id()
        .filter(world.id)
        .collect::<Vec<_>>();
    bodies.sort_by_key(|body| body.id);
    let bodies = bodies.as_mut_slice();

    // apply_forces(world, bodies);
    // integrate_velocity(bodies, delta_time);
    //
    // let collisions = resolve_collisions(bodies);
    // PositionSolver::solve(&collisions, bodies);
    // ImpulseSolver::solve(&collisions, bodies, delta_time);

    let collisions = resolve_collisions(bodies);
    solve_collisions(&collisions, bodies, delta_time);
    apply_dynamics(ctx, world, bodies, delta_time);
}

fn apply_forces(world: &PhysicsWorld, bodies: &mut [RigidBody]) {
    for body in bodies {
        if body.inv_mass == 0.0 {
            continue; // Skip static bodies
        }

        body.force += world.gravity * body.mass;
    }
}

fn integrate_velocity(bodies: &mut [RigidBody], delta_time: f32) {
    for body in bodies {
        if body.inv_mass == 0.0 {
            continue; // Skip static bodies
        }

        let acceleration = body.force * body.inv_mass;
        body.velocity += acceleration * delta_time;

        body.force = Vec3::ZERO;
    }
}

fn apply_dynamics(
    ctx: &ReducerContext,
    world: &PhysicsWorld,
    bodies: &mut [RigidBody],
    delta_time: f32,
) {
    for body in bodies {
        if body.mass > 0.0 {
            body.force += world.gravity * body.mass;
            body.velocity += body.force / body.mass * delta_time;
            body.transform.position += body.velocity * delta_time;
            debug!(
                "applyed dynamics: Body {} position: {}, velocity: {}, force: {}",
                body.id, body.transform.position, body.velocity, body.force
            );
        }

        body.force = Vec3::ZERO;

        body.update(ctx);
    }
}

fn resolve_collisions(bodies: &[RigidBody]) -> Vec<Collision> {
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

fn solve_collisions(collisions: &[Collision], bodies: &mut [RigidBody], delta_time: f32) {
    PositionSolver::solve(collisions, bodies);
    ImpulseSolver::solve(collisions, bodies, delta_time);
}
