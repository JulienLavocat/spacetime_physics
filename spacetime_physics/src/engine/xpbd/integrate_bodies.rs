use log::debug;

use crate::{
    math::{Quat, Vec3},
    PhysicsWorld, RigidBodyData,
};

pub fn integrate_bodies(bodies: &mut [RigidBodyData], world: &PhysicsWorld, delta_time: f32) {
    let sw = world.stopwatch("integrate_bodies");
    for body in bodies {
        if !body.is_dynamic() {
            continue;
        }

        // --- Linear integration ---

        body.set_previous_position(body.position());
        let weight = world.gravity * body.effective_mass();
        let total_force = body.force() + weight;

        // v ← v + h * fext / m
        body.set_linear_velocity(
            body.linear_velocity() + total_force * body.effective_inverse_mass() * delta_time,
        );

        // x ← x + h * v
        body.set_position(body.position() + body.linear_velocity() * delta_time);

        // --- Angular integration ---

        body.set_previous_rotation(body.rotation());

        let i = body.inertia_tensor();
        let inv_inertia_tensor = body.inv_inertia_tensor();
        let omega = body.angular_velocity();

        // gyroscopic torque: ω × (Iω)
        let torque = body.torque();
        let i_omega = i * omega;
        let gyro = omega.cross(i_omega);

        // α = I⁻¹(τ - ω × Iω)
        let angular_acceleration = inv_inertia_tensor * (torque - gyro);

        // ω ← ω + h * α
        body.set_angular_velocity(body.angular_velocity() + delta_time * angular_acceleration);

        // q ← q + 0.5 * h * q × ω
        let dq = 0.5 * delta_time * body.rotation() * Quat::from_xyz(body.angular_velocity(), 0.0);
        body.set_rotation(body.rotation() + dq);

        body.set_torque(Vec3::ZERO);
        body.set_force(Vec3::ZERO);

        if world.debug_substep() {
            debug!(
            "[Integrate] body {}: position: {}, rotation: {}, velocity: {}, angular_velocity: {}",
            body.id, body.position(), body.rotation(), body.linear_velocity(), body.angular_velocity()
        );
        }
    }
    sw.end();
}

#[cfg(test)]
#[path = "integrate_bodies_tests.rs"]
mod integrate_bodies_test;
