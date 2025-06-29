use log::debug;

use crate::{
    math::{Mat3, Quat, Vec3},
    utils::get_bodies_mut,
    PhysicsWorld,
};

use super::{
    constraints::{Constraint, PenetrationConstraint, PositionConstraint},
    RigidBodyEntity,
};

pub(crate) fn integrate_bodies(
    bodies: &mut [RigidBodyEntity],
    world: &PhysicsWorld,
    delta_time: f32,
) {
    let sw = world.stopwatch("integrate_bodies");
    for body in bodies {
        if !body.rb.is_dynamic() {
            continue;
        }

        // --- Linear integration ---

        body.rb.previous_position = body.rb.position;
        let weight = world.gravity * body.rb.mass;
        let total_force = body.rb.force + weight;

        // v ← v + h * fext / m
        body.rb.linear_velocity += total_force * body.rb.inv_mass * delta_time;
        // x ← x + h * v
        body.rb.position += body.rb.linear_velocity * delta_time;

        // --- Angular integration ---

        body.rb.previous_rotation = body.rb.rotation;

        let i = body.inertia_tensor;
        let inv_inertia_tensor = body.inv_inertia_tensor;
        let omega = body.rb.angular_velocity;

        // gyroscopic torque: ω × (Iω)
        let torque = body.rb.torque;
        let i_omega = i * omega;
        let gyro = omega.cross(i_omega);

        // α = I⁻¹(τ - ω × Iω)
        let angular_acceleration = inv_inertia_tensor * (torque - gyro);

        // ω ← ω + h * α
        body.rb.angular_velocity += delta_time * angular_acceleration;

        // q ← q + 0.5 * h * q × ω
        let dq = 0.5 * Quat::from_xyz(body.rb.angular_velocity, 0.0) * body.rb.rotation;
        body.rb.rotation = (body.rb.rotation + dq * delta_time).normalize();

        body.rb.force = Vec3::ZERO;
        body.rb.torque = Vec3::ZERO;

        if world.debug_substep() {
            debug!(
            "[Integrate] body {}: position: {}, rotation: {}, velocity: {}, angular_velocity: {}",
            body.rb.id, body.rb.position, body.rb.rotation, body.rb.linear_velocity, body.rb.angular_velocity
        );
        }
    }
    sw.end();
}

pub(crate) fn solve_constraints(
    world: &PhysicsWorld,
    contact_constraints: &mut [PenetrationConstraint],
    bodies: &mut [RigidBodyEntity],
    delta_time: f32,
) {
    let sw = world.stopwatch("solve_constraints");
    contact_constraints
        .iter_mut()
        .for_each(|constraint| constraint.solve(world, bodies, delta_time));
    sw.end();
}

pub(crate) fn recompute_velocities(world: &PhysicsWorld, bodies: &mut [RigidBodyEntity], dt: f32) {
    let sw = world.stopwatch("recompute_velocities");
    for body in bodies {
        if !body.rb.is_dynamic() {
            body.rb.linear_velocity = Vec3::ZERO;
            body.rb.angular_velocity = Vec3::ZERO;
            continue;
        }

        body.rb.pre_solve_linear_velocity = body.rb.linear_velocity;
        body.rb.linear_velocity = (body.rb.position - body.rb.previous_position) / dt;

        body.rb.pre_solve_angular_velocity = body.rb.angular_velocity;
        body.rb.angular_velocity =
            (body.rb.rotation * body.rb.previous_rotation.inverse()).as_radians() / dt;

        if world.debug_substep() {
            debug!(
                "[RecomputeVelocities] body {}: velocity: {} -> {}, angular_velocity: {} -> {}",
                body.rb.id,
                body.rb.pre_solve_linear_velocity,
                body.rb.linear_velocity,
                body.rb.pre_solve_angular_velocity,
                body.rb.angular_velocity
            );
        }
    }
    sw.end();
}

pub(crate) fn solve_velocities(
    world: &PhysicsWorld,
    penetration_constraints: &[PenetrationConstraint],
    bodies: &mut [RigidBodyEntity],
    dt: f32,
) {
    let sw = world.stopwatch("solve_velocities");
    for constraint in penetration_constraints {
        let (body1, body2) = get_bodies_mut(constraint.a, constraint.b, bodies);
        let normal = constraint.normal;
        let gravity = world.gravity;

        // Compute pre-solve relative normal velocities at the contact point (used for restitution)
        let pre_solve_contact_vel1 = compute_contact_vel(
            body1.rb.pre_solve_linear_velocity,
            body1.rb.pre_solve_angular_velocity,
            constraint.world_a,
        );
        let pre_solve_contact_vel2 = compute_contact_vel(
            body2.rb.pre_solve_linear_velocity,
            body2.rb.pre_solve_angular_velocity,
            constraint.world_b,
        );
        let pre_solve_relative_vel = pre_solve_contact_vel1 - pre_solve_contact_vel2;
        let pre_solve_normal_vel = normal.dot(pre_solve_relative_vel);

        // Compute relative normal and tangential velocities at the contact point (equation 29)
        let contact_vel1 = compute_contact_vel(
            body1.rb.linear_velocity,
            body1.rb.angular_velocity,
            constraint.world_a,
        );
        let contact_vel2 = compute_contact_vel(
            body2.rb.linear_velocity,
            body2.rb.angular_velocity,
            constraint.world_b,
        );
        let relative_vel = contact_vel1 - contact_vel2;
        let normal_vel = normal.dot(relative_vel);
        let tangent_vel = relative_vel - normal * normal_vel;

        let inv_mass1 = body1.effective_inverse_mass();
        let inv_mass2 = body2.effective_inverse_mass();
        let inv_inertia1 = body1.effective_inverse_inertia();
        let inv_inertia2 = body2.effective_inverse_inertia();

        let friction_coefficient = body1
            .rb
            .friction
            .combine(&body2.rb.friction)
            .dynamic_coefficient;
        let restitution_coefficient = body1
            .rb
            .restitution
            .combine(&body2.rb.restitution)
            .coefficient;

        // Compute dynamic friction
        let friction_impulse = get_dynamic_friction(
            tangent_vel,
            friction_coefficient,
            constraint.normal_lagrange,
            dt,
        );

        // Compute restitution
        let restitution_impulse = get_restitution(
            world,
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
        let w1 =
            constraint.compute_generalized_inverse_mass(body1, &constraint.world_a, &delta_v_dir);
        let w2 =
            constraint.compute_generalized_inverse_mass(body2, &constraint.world_b, &delta_v_dir);

        // Compute velocity impulse and apply velocity updates (equation 33)
        let p = delta_v / (w1 + w2);
        if body1.rb.is_dynamic() {
            body1.rb.linear_velocity += p * inv_mass1;
            body1.rb.angular_velocity += compute_delta_ang_vel(inv_inertia1, constraint.world_a, p);
        }
        if body2.rb.is_dynamic() {
            body2.rb.linear_velocity -= p * inv_mass2;
            body2.rb.angular_velocity -= compute_delta_ang_vel(inv_inertia2, constraint.world_b, p);
        }

        if world.debug {
            debug!(
            "[SolveVelocities]: a: {}, b: {}, normal: {}, normal_vel: {}, tangent_vel: {}, friction_impulse: {}, restitution_impulse: {}, delta_v: {}, delta_v_length: {}, a_linear_velocity: {}, a_angular_velocity: {}, b_linear_velocity: {}, b_angular_velocity: {}",
            constraint.a, constraint.b, normal, normal_vel, tangent_vel, friction_impulse, restitution_impulse, delta_v, delta_v_length, body1.rb.linear_velocity, body1.rb.angular_velocity, body2.rb.linear_velocity, body2.rb.angular_velocity
        );
        }
    }
    sw.end();
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
    world: &PhysicsWorld,
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

    let restitution = normal * (-normal_vel + (-coefficient * pre_solve_normal_vel).min(0.0));

    if world.debug {
        debug!(
        "[GetRestitution] normal: {}, normal_vel: {}, pre_solve_normal_vel: {}, coefficient: {}, restitution: {}",
        normal, normal_vel, pre_solve_normal_vel, coefficient, restitution
    );
    }

    restitution
}
