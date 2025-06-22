use crate::{
    math::{Mat3, Quat, Vec3},
    tables::RigidBody,
};

use super::Constraint;

pub trait PositionConstraint: Constraint {
    fn apply_position_correction(
        &self,
        body_a: &mut RigidBody,
        body_b: &mut RigidBody,
        delta_lagrange: f32,
        direction: &Vec3,
        ra: &Vec3,
        rb: &Vec3,
    ) {
        if delta_lagrange.abs() < f32::EPSILON {
            return;
        }

        let p = delta_lagrange * direction;
        let rot_a = body_a.rotation;
        let rot_b = body_b.rotation;

        let inv_mass_a = body_a.effective_inverse_mass();
        let inv_mass_b = body_b.effective_inverse_mass();

        let inv_inertia_a = match body_a.inv_inertia_tensor {
            Some(i) => i,
            None => return, // static body
        };
        let inv_inertia_b = match body_b.inv_inertia_tensor {
            Some(i) => i,
            None => return,
        };

        body_a.position += p * inv_mass_a;
        body_a.rotation += Self::get_delta_rot(rot_a, inv_inertia_a, ra, p);
        body_b.position -= p * inv_mass_b;
        body_b.rotation += Self::get_delta_rot(rot_b, inv_inertia_b, rb, -p);
    }

    fn compute_generalized_inverse_mass(&self, body: &RigidBody, r: &Vec3, n: &Vec3) -> f32 {
        let inv_inertia = body.effective_inverse_inertia();
        let r_cross_n = r.cross(n);
        body.inv_mass + r_cross_n.dot(inv_inertia * r_cross_n)
    }

    fn get_delta_rot(rot: Quat, inv_inertia: Mat3, r: &Vec3, p: Vec3) -> Quat {
        Quat::from_xyz(0.5 * (inv_inertia * r.cross(p)), 0.0) * rot
    }
}
