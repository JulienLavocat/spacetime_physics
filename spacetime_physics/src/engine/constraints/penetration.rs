use std::fmt::Display;

use log::debug;

use crate::{engine::utils::get_bodies_mut, math::Vec3, tables::RigidBody};

use super::{position::PositionConstraint, Constraint};

#[derive(Debug, Clone)]
pub struct PenetrationConstraint {
    pub a: u64,
    pub b: u64,
    pub contact_point_a: Vec3,
    pub contact_point_b: Vec3,
    pub normal: Vec3,
    pub penetration_depth: f32,
    pub compliance: f32,
    pub normal_lagrange: f32,
    pub normal_force: Vec3,
    pub tangential_lagrange: f32,
    pub local_ra: Vec3,
    pub local_rb: Vec3,
    pub static_friction_force: Vec3,
}

impl PenetrationConstraint {
    pub fn new(
        a: &mut RigidBody,
        b: &mut RigidBody,
        contact_point_a: Vec3,
        contact_point_b: Vec3,
        normal: Vec3,
        penetration_depth: f32,
        compliance: f32,
    ) -> Self {
        Self {
            a: a.id,
            b: b.id,
            contact_point_a,
            contact_point_b,
            normal,
            penetration_depth,
            compliance,
            normal_lagrange: 0.0,
            normal_force: Vec3::ZERO,
            tangential_lagrange: 0.0,
            static_friction_force: Vec3::ZERO,
            local_ra: a.rotation.inverse().rotate(contact_point_a),
            local_rb: b.rotation.inverse().rotate(contact_point_b),
        }
    }

    fn solve_contact(&mut self, body_a: &mut RigidBody, body_b: &mut RigidBody, dt: f32) {
        let penetraion = self.penetration_depth;
        let normal = self.normal;
        let compliance = self.compliance;
        let lagrange = self.normal_lagrange;
        let ra = self.contact_point_a;
        let rb = self.contact_point_b;

        if penetraion >= 0.0 {
            return;
        }

        let wa = self.compute_generalized_inverse_mass(body_a, &ra, &normal);
        let wb = self.compute_generalized_inverse_mass(body_b, &rb, &normal);

        let gradients = [normal, -normal];
        let w = [wa, wb];

        let delta_lagrange =
            self.compute_lagrange_update(lagrange, penetraion, &gradients, &w, compliance, dt);
        self.normal_lagrange += delta_lagrange;

        self.apply_position_correction(body_a, body_b, delta_lagrange, &normal, &ra, &rb);

        self.normal_force = self.normal_lagrange * normal / dt.powi(2);

        debug!(
            "PenetrationConstraint contact solved: a: {}, b: {}, compliance: {}, normal_lagrange: {}, tangential_lagrange: {}",
            self.a, self.b, self.compliance, self.normal_lagrange, self.tangential_lagrange
        );
    }

    fn solve_friction(&mut self, body_a: &mut RigidBody, body_b: &mut RigidBody, dt: f32) {
        let penetraion = self.penetration_depth;
        let normal = self.normal;
        let compliance = self.compliance;
        let lagrange = self.tangential_lagrange;
        let ra = self.contact_point_a;
        let rb = self.contact_point_b;

        let pa = body_a.position + body_a.rotation.rotate(self.local_ra);
        let pb = body_b.position + body_b.rotation.rotate(self.local_rb);
        let prev_pa = body_a.previous_position + body_a.previous_rotation.rotate(self.local_ra);
        let prev_pb = body_b.previous_position + body_b.previous_rotation.rotate(self.local_rb);

        let delta_p = (pa - prev_pa) - (pb - prev_pb);
        let delta_p_tangential = delta_p - delta_p.dot(normal) * normal;

        let sliding_length = delta_p_tangential.length();
        if sliding_length < f32::EPSILON {
            return; // No sliding
        }
        let tangent = delta_p_tangential / sliding_length;

        let wa = self.compute_generalized_inverse_mass(body_a, &ra, &tangent);
        let wb = self.compute_generalized_inverse_mass(body_b, &rb, &tangent);

        let gradients = [tangent, -tangent];
        let w = [wa, wb];

        let static_coefficient = 0.5 * (body_a.friction + body_b.friction);

        if sliding_length > static_coefficient * penetraion {
            let delta_lagrange = self.compute_lagrange_update(
                lagrange,
                sliding_length,
                &gradients,
                &w,
                compliance,
                dt,
            );
            self.tangential_lagrange += delta_lagrange;
            self.apply_position_correction(body_a, body_b, delta_lagrange, &tangent, &ra, &rb);
            self.static_friction_force = self.tangential_lagrange * tangent / dt.powi(2);
            debug!(
                "PenetrationConstraint friction solved: a: {}, b: {}, compliance: {}, normal_lagrange: {}, tangential_lagrange: {}",
                self.a, self.b, self.compliance, self.normal_lagrange, self.tangential_lagrange
            );
        }
    }
}

impl Constraint for PenetrationConstraint {
    fn solve(&mut self, bodies: &mut [RigidBody], dt: f32) {
        let (body_a, body_b) = get_bodies_mut(self.a, self.b, bodies);
        self.solve_contact(body_a, body_b, dt);
        // self.solve_friction(body_a, body_b, dt);
    }
}

impl PositionConstraint for PenetrationConstraint {}

impl Display for PenetrationConstraint {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "PenetrationConstraint(a: {}, b: {}, contact_point_a: {}, contact_point_b: {}, normal: {}, penetration_depth: {}, compliance: {}, normal_lagrange: {}, tangential_lagrange: {})",
            self.a,
            self.b,
            self.contact_point_a,
            self.contact_point_b,
            self.normal,
            self.penetration_depth,
            self.compliance,
            self.normal_lagrange,
            self.tangential_lagrange
        )
    }
}
