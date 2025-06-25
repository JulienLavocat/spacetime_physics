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

    for i in 0..world.sub_step {
        debug!("---------- substep: {} ----------", i);
        // TODO: Use Broad Phase outside of the loop and narrowly-resolve them once here
        let mut penetration_constraints = detect_collisions(bodies, world);
        let penetration_constraints = penetration_constraints.as_mut_slice();

        debug!("Collisions detected: {:?}", penetration_constraints);

        integrate_bodies(bodies, world, dt);

        for _ in 0..world.position_iterations {
            solve_constraints(penetration_constraints, bodies, dt);
        }

        recompute_velocities(bodies, dt);
        solve_velocities(world, penetration_constraints, bodies, dt);

        debug_bodies(bodies);
    }
    debug!("---------- End of substeps ----------");

    for body in bodies {
        debug!(
            "Updating {} position: {} -> {}, velocity: {}, rotation: {}",
            body.id, body.previous_position, body.position, body.linear_velocity, body.rotation,
        );
        body.update(ctx);
    }
    debug!("-------------------------------------------------------------")
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

fn integrate_bodies(bodies: &mut [RigidBody], world: &PhysicsWorld, delta_time: f32) {
    for body in bodies {
        if body.is_static_or_sleeping() {
            continue;
        }

        // --- Linear integration ---

        body.previous_position = body.position;
        let weight = world.gravity * body.mass;
        let total_force = body.force + weight;

        // v ← v + h * fext / m
        body.linear_velocity += total_force * body.inv_mass * delta_time;
        // x ← x + h * v
        body.position += body.linear_velocity * delta_time;

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

        debug!(
            "[Integrate] body {}: position: {}, rotation: {}, velocity: {}, angular_velocity: {}",
            body.id, body.position, body.rotation, body.linear_velocity, body.angular_velocity
        );
    }
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
        if body.is_static_or_sleeping() {
            body.linear_velocity = Vec3::ZERO;
            body.angular_velocity = Vec3::ZERO;
            continue;
        }

        body.pre_solve_linear_velocity = body.linear_velocity;
        body.linear_velocity = (body.position - body.previous_position) / delta_time;

        body.pre_solve_angular_velocity = body.angular_velocity;
        let dq = (body.rotation * body.previous_rotation.inverse()).normalize();
        let angle = 2.0 * dq.w.acos();
        let sin_half = (1.0 - dq.w * dq.w).sqrt();
        let axis = if sin_half > 1e-5 {
            dq.xyz() / sin_half
        } else {
            Vec3::Z
        };

        body.angular_velocity = axis * angle / delta_time;

        debug!(
            "[RecomputeVelocities] body {}: velocity: {} -> {}, angular_velocity: {} -> {}, dq: {}, axis: {}, angle: {}",
            body.id,body.pre_solve_linear_velocity, body.linear_velocity, body.pre_solve_angular_velocity, body.angular_velocity, dq, axis, angle
        );
    }
}

fn solve_velocities(
    world: &PhysicsWorld,
    penetration_constraints: &[PenetrationConstraint],
    bodies: &mut [RigidBody],
    dt: f32,
) {
    for constraint in penetration_constraints {
        let (body1, body2) = get_bodies_mut(constraint.a, constraint.b, bodies);
        let normal = constraint.normal;
        let gravity = world.gravity;

        // Compute pre-solve relative normal velocities at the contact point (used for restitution)
        let pre_solve_contact_vel1 = compute_contact_vel(
            body1.pre_solve_linear_velocity,
            body1.pre_solve_angular_velocity,
            constraint.contact_point_a,
        );
        let pre_solve_contact_vel2 = compute_contact_vel(
            body2.pre_solve_linear_velocity,
            body2.pre_solve_angular_velocity,
            constraint.contact_point_b,
        );
        let pre_solve_relative_vel = pre_solve_contact_vel1 - pre_solve_contact_vel2;
        let pre_solve_normal_vel = normal.dot(pre_solve_relative_vel);

        // Compute relative normal and tangential velocities at the contact point (equation 29)
        let contact_vel1 = compute_contact_vel(
            body1.linear_velocity,
            body1.angular_velocity,
            constraint.contact_point_a,
        );
        let contact_vel2 = compute_contact_vel(
            body2.linear_velocity,
            body2.angular_velocity,
            constraint.contact_point_b,
        );
        let relative_vel = contact_vel1 - contact_vel2;
        let normal_vel = normal.dot(relative_vel);
        let tangent_vel = relative_vel - normal * normal_vel;

        let inv_mass1 = body1.effective_inverse_mass();
        let inv_mass2 = body2.effective_inverse_mass();
        let inv_inertia1 = body1.effective_inverse_inertia();
        let inv_inertia2 = body2.effective_inverse_inertia();

        let friction_coefficient = body1.friction.combine(&body2.friction).dynamic_friction;
        let restitution_coefficient = (body1.restitution + body2.restitution) * 0.5;

        // Compute dynamic friction
        let friction_impulse = get_dynamic_friction(
            tangent_vel,
            friction_coefficient,
            constraint.normal_lagrange,
            dt,
        );

        // Compute restitution
        let restitution_impulse = get_restitution(
            normal,
            normal_vel,
            pre_solve_normal_vel,
            restitution_coefficient,
            gravity,
            dt,
        );

        let delta_v = friction_impulse + restitution_impulse;
        let delta_v_length = delta_v.length();

        if delta_v_length <= f32::EPSILON {
            continue;
        }

        let delta_v_dir = delta_v / delta_v_length;

        // Compute generalized inverse masses
        let w1 = constraint.compute_generalized_inverse_mass(
            body1,
            &constraint.contact_point_a,
            &delta_v_dir,
        );
        let w2 = constraint.compute_generalized_inverse_mass(
            body2,
            &constraint.contact_point_b,
            &delta_v_dir,
        );

        // Compute velocity impulse and apply velocity updates (equation 33)
        let p = delta_v / (w1 + w2);
        if !body1.is_static_or_sleeping() {
            body1.linear_velocity += p * inv_mass1;
            body1.angular_velocity +=
                compute_delta_ang_vel(inv_inertia1, constraint.contact_point_a, p);
        }
        if !body2.is_static_or_sleeping() {
            body2.linear_velocity -= p * inv_mass2;
            body2.angular_velocity -=
                compute_delta_ang_vel(inv_inertia2, constraint.contact_point_b, p);
        }

        debug!(
            "[SolveVelocities]: a: {}, b: {}, normal: {}, normal_vel: {}, tangent_vel: {}, delta_v: {}, delta_v_length: {}, a_linear_velocity: {}, a_angular_velocity: {}, b_linear_velocity: {}, b_angular_velocity: {}",
            constraint.a, constraint.b, normal, normal_vel, tangent_vel, delta_v, delta_v_length, body1.linear_velocity, body1.angular_velocity, body2.linear_velocity, body2.angular_velocity
        );
    }
}

fn compute_contact_vel(lin_vel: Vec3, ang_vel: Vec3, r: Vec3) -> Vec3 {
    lin_vel + ang_vel.cross(r)
}

fn compute_delta_ang_vel(inverse_inertia: Mat3, r: Vec3, p: Vec3) -> Vec3 {
    inverse_inertia * r.cross(p)
}

fn get_dynamic_friction(
    tangent_vel: Vec3,
    coefficient: f32,
    normal_lagrange: f32,
    sub_dt: f32,
) -> Vec3 {
    let tangent_vel_magnitude = tangent_vel.length();

    // Avoid division by zero when normalizing the vector later.
    // We compare against epsilon to avoid potential floating point precision problems.
    if tangent_vel_magnitude.abs() <= f32::EPSILON {
        return Vec3::ZERO;
    }

    let normal_force = normal_lagrange / sub_dt.powi(2);

    // Velocity update caused by dynamic friction, never exceeds the magnitude of the tangential velocity itself
    let dir = if tangent_vel_magnitude > 1e-6 {
        tangent_vel / tangent_vel_magnitude
    } else {
        Vec3::ZERO
    };
    -dir * (sub_dt * coefficient * normal_force.abs()).min(tangent_vel_magnitude)
}
fn get_restitution(
    normal: Vec3,
    normal_vel: f32,
    pre_solve_normal_vel: f32,
    mut coefficient: f32,
    gravity: Vec3,
    sub_dt: f32,
) -> Vec3 {
    // If normal velocity is small enough, use restitution of 0 to avoid jittering
    if normal_vel.abs() <= 2.0 * gravity.length() * sub_dt {
        coefficient = 0.0;
    }

    normal * (-normal_vel + (-coefficient * pre_solve_normal_vel).min(0.0))
}

fn debug_bodies(bodies: &[RigidBody]) {
    for body in bodies {
        debug!(
            "[Body] {}: position: {}, rotation: {}, velocity: {}, angular_velocity: {}, force: {}, torque: {}",
            body.id, body.position, body.rotation, body.linear_velocity, body.angular_velocity, body.force, body.torque
        );
    }
}
