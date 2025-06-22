use constraints::{Constraint, PenetrationConstraint, PositionConstraint};
use log::debug;
use spacetimedb::{log_stopwatch::LogStopwatch, ReducerContext};
use utils::get_bodies_mut;

use crate::{
    math::{Mat3, Quat, Vec3},
    tables::{physics_rigid_bodies, PhysicsWorld, RigidBody},
};

pub mod collisions;
mod constraints;
mod utils;

pub fn step_world(ctx: &ReducerContext, world: &PhysicsWorld) {
    LogStopwatch::new("step_world");
    let mut bodies = ctx
        .db
        .physics_rigid_bodies()
        .world_id()
        .filter(world.id)
        .collect::<Vec<_>>();
    bodies.sort_by_key(|body| body.id);
    let bodies = bodies.as_mut_slice();

    let dt = world.time_step / world.sub_step as f32;

    let mut penetration_constraints = detect_collisions(bodies, world);
    let penetration_constraints = penetration_constraints.as_mut_slice();

    debug!("Collisions detected: {:?}", penetration_constraints);
    for _ in 0..world.sub_step {
        run_step(bodies, world, dt);

        for _ in 0..world.position_iterations {
            solve_constraints(penetration_constraints, bodies, dt);
        }

        recompute_velocities(bodies, dt);
        solve_velocities(world, penetration_constraints, bodies, dt);
    }

    for body in bodies {
        if body.id == 1 {
            debug!(
                "Updating sphere position: {} -> {}, velocity: {}",
                body.previous_position, body.position, body.velocity
            );
        }
        body.update(ctx);
    }
}

fn detect_collisions(bodies: &mut [RigidBody], world: &PhysicsWorld) -> Vec<PenetrationConstraint> {
    let mut constraints = Vec::new();
    for i in 0..bodies.len() {
        for j in (i + 1)..bodies.len() {
            let (left, right) = bodies.split_at_mut(j);
            let body_a = &mut left[i];
            let body_b = &mut right[0];
            if let Some(collision) = collisions::test_collision(
                &body_a.position,
                &body_a.rotation,
                &body_a.collider,
                &body_b.position,
                &body_b.rotation,
                &body_b.collider,
                world.precision,
            ) {
                if collision.distance >= 0.0 {
                    continue; // No penetration
                }

                constraints.push(PenetrationConstraint::new(
                    body_a,
                    body_b,
                    collision.a,
                    collision.b,
                    collision.normal,
                    collision.distance,
                    0.0,
                ));
            }
        }
    }
    constraints
}

fn run_step(bodies: &mut [RigidBody], world: &PhysicsWorld, delta_time: f32) {
    for body in bodies {
        integrate_rigid_body(world, body, delta_time);
    }
}

fn integrate_rigid_body(world: &PhysicsWorld, body: &mut RigidBody, delta_time: f32) {
    // --- Linear integration ---

    body.previous_position = body.position;
    let weight = world.gravity * body.mass;
    let total_force = body.force + weight;

    // v ← v + h * fext / m
    body.velocity += total_force * body.inv_mass * delta_time;
    // x ← x + h * v
    body.position += body.velocity * delta_time;

    // --- Angular integration ---

    body.previous_rotation = body.rotation;

    let i = body.inertia_tensor;
    let inv_inertia_tensor = match body.inv_inertia_tensor {
        Some(inv) => inv,
        None => return,
    };
    let omega = body.angular_velocity;

    // gyroscopic torque: ω × (Iω)
    let torque = body.torque;
    let i_omega = i * omega;
    let gyro = omega.cross(i_omega);

    // α = I⁻¹(τ - ω × Iω)
    let angular_acceleration = inv_inertia_tensor * (torque - gyro);

    // ω ← ω + h * α
    body.angular_velocity += delta_time * angular_acceleration;

    let omega_quat = Quat {
        x: body.angular_velocity.x,
        y: body.angular_velocity.y,
        z: body.angular_velocity.z,
        w: 0.0,
    };

    // q ← q + 0.5 * h * q × ω
    let dq = 0.5 * omega_quat * body.rotation;
    body.rotation = (body.rotation + dq * delta_time).normalize();

    body.force = Vec3::ZERO;
    body.torque = Vec3::ZERO;
}

fn solve_constraints(
    contact_constraints: &mut [PenetrationConstraint],
    bodies: &mut [RigidBody],
    delta_time: f32,
) {
    contact_constraints
        .iter_mut()
        .for_each(|constraint| constraint.solve(bodies, delta_time));
}

fn recompute_velocities(bodies: &mut [RigidBody], delta_time: f32) {
    for body in bodies {
        body.pre_solve_velocity = body.velocity;
        body.velocity = (body.position - body.previous_position) / delta_time;

        body.pre_solve_angular_velocity = body.angular_velocity;
        let dq = body.rotation * body.previous_rotation.inverse();
        let omega = 2.0 * dq.xyz() / delta_time;
        body.angular_velocity = if dq.w >= 0.0 { omega } else { -omega };
    }
}

fn solve_velocities(
    world: &PhysicsWorld,
    penetration_constraints: &[PenetrationConstraint],
    bodies: &mut [RigidBody],
    dt: f32,
) {
    for constraint in penetration_constraints {
        let (a, b) = get_bodies_mut(constraint.a, constraint.b, bodies);
        let pre_solve_contact_velocity_a = compute_contact_velocity(
            a.pre_solve_velocity,
            a.pre_solve_angular_velocity,
            constraint.contact_point_a,
        );
        let pre_solve_contact_velocity_b = compute_contact_velocity(
            b.pre_solve_velocity,
            b.pre_solve_angular_velocity,
            constraint.contact_point_b,
        );
        let pre_solve_relative_velocity =
            pre_solve_contact_velocity_a - pre_solve_contact_velocity_b;
        let pre_solve_normal_velocity = constraint.normal.dot(pre_solve_relative_velocity);

        let contact_velocity_a =
            compute_contact_velocity(a.velocity, a.angular_velocity, constraint.contact_point_a);
        let contact_velocity_b =
            compute_contact_velocity(b.velocity, b.angular_velocity, constraint.contact_point_b);

        let relative_velocity = contact_velocity_a - contact_velocity_b;
        let normal_velocity = constraint.normal.dot(relative_velocity);
        let tangential_velocity = relative_velocity - normal_velocity * constraint.normal;

        let inv_mass_a = a.effective_inverse_mass();
        let inv_mass_b = b.effective_inverse_mass();
        let inv_intertia_a = a.effective_inverse_inertia();
        let inv_intertia_b = b.effective_inverse_inertia();

        let friction_impulse = get_dynamic_friction(
            tangential_velocity,
            0.5 * (a.friction + b.friction),
            constraint.normal_lagrange,
            dt,
        );

        let restitution_impulse = get_restitution(
            constraint.normal,
            normal_velocity,
            pre_solve_normal_velocity,
            0.5 * (a.restitution + b.restitution),
            world.gravity,
            dt,
        );

        let delta_v = restitution_impulse + friction_impulse;
        let delta_v_length = delta_v.length();

        if delta_v_length < f32::EPSILON {
            continue; // No impulse needed
        }

        let delta_v_normalized = delta_v / delta_v_length;

        let wa = constraint.compute_generalized_inverse_mass(
            a,
            &constraint.contact_point_a,
            &delta_v_normalized,
        );
        let wb = constraint.compute_generalized_inverse_mass(
            b,
            &constraint.contact_point_b,
            &delta_v_normalized,
        );

        let p = delta_v / (wa + wb);
        a.velocity += p * inv_mass_a;
        a.angular_velocity +=
            compute_delta_ang_velocity(inv_intertia_a, constraint.contact_point_a, p);

        b.velocity -= p * inv_mass_b;
        b.angular_velocity -=
            compute_delta_ang_velocity(inv_intertia_b, constraint.contact_point_b, p);
    }
}

fn compute_contact_velocity(lin_vel: Vec3, ang_vel: Vec3, r: Vec3) -> Vec3 {
    lin_vel + ang_vel.cross(r)
}

fn compute_delta_ang_velocity(inv_inertia: Mat3, r: Vec3, p: Vec3) -> Vec3 {
    inv_inertia * r.cross(p)
}

fn get_dynamic_friction(
    tangential_velocity: Vec3,
    coefficient: f32,
    normal_lagrange: f32,
    dt: f32,
) -> Vec3 {
    let tangent_velocity_magnitude = tangential_velocity.length();
    if tangent_velocity_magnitude.abs() < f32::EPSILON {
        return Vec3::ZERO; // No tangential velocity
    }

    let normal_force = normal_lagrange / dt.powi(2);

    -tangential_velocity / tangent_velocity_magnitude
        * (dt * coefficient * normal_force.abs()).min(tangent_velocity_magnitude)
}

fn get_restitution(
    normal: Vec3,
    normal_velocity: f32,
    pre_solve_normal_velocity: f32,
    mut coefficient: f32,
    gravity: Vec3,
    dt: f32,
) -> Vec3 {
    if normal_velocity.abs() <= 2.0 * gravity.length() * dt {
        coefficient = 0.0; // No restitution if the velocity is too low
    }

    normal * (-normal_velocity + (-coefficient * pre_solve_normal_velocity).min(0.0))
}
