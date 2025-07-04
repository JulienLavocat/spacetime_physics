use crate::{
    math::{Quat, Vec3},
    RigidBodyData,
};

use super::Constraint;

pub trait PositionConstraint: Constraint {
    fn apply_position_correction(
        &self,
        body_a: &mut RigidBodyData,
        body_b: &mut RigidBodyData,
        delta_lagrange: f32,
        direction: &Vec3,
        ra: &Vec3,
        rb: &Vec3,
    ) -> Option<(Vec3, Vec3, Vec3, Vec3, Vec3)> {
        if delta_lagrange.abs() < f32::EPSILON {
            return None;
        }

        let p = delta_lagrange * direction;

        let previous_position_a = body_a.position();
        let previous_position_b = body_b.position();

        if body_a.is_dynamic() {
            Self::apply_body_correction(body_a, &p, ra, 1.0);
        }

        if body_b.is_dynamic() {
            Self::apply_body_correction(body_b, &p, rb, -1.0);
        }

        Some((
            previous_position_a,
            body_a.position(),
            previous_position_b,
            body_b.position(),
            p,
        ))
    }

    fn apply_body_correction(body: &mut RigidBodyData, p: &Vec3, r: &Vec3, sign: f32) {
        body.set_position(body.position() + (sign * p * body.effective_inverse_mass()));

        let inv_inertia = body.effective_inverse_inertia();
        let delta_angle = sign * inv_inertia * r.cross(*p);

        // Proper quaternion integration for angular correction
        let dq = Quat::from_scaled_axis(delta_angle);
        body.set_rotation(dq * body.rotation());
    }

    fn compute_generalized_inverse_mass(&self, body: &RigidBodyData, r: &Vec3, n: &Vec3) -> f32 {
        let inv_inertia = body.effective_inverse_inertia();
        let r_cross_n = r.cross(n);
        body.inv_mass() + r_cross_n.dot(inv_inertia * r_cross_n)
    }
}
