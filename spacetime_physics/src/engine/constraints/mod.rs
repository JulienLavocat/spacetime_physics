mod penetration;
mod position;

pub use penetration::*;
pub use position::*;

use crate::{math::Vec3, PhysicsWorld};

use super::RigidBodyData;

pub(crate) trait Constraint {
    fn solve(&mut self, world: &PhysicsWorld, bodies: &mut [RigidBodyData], dt: f32);

    fn compute_lagrange_update(
        &self,
        lagrange: f32,
        c: f32,
        gradients: &[Vec3],
        inverse_masses: &[f32],
        compliance: f32,
        dt: f32,
    ) -> f32 {
        let w_sum = inverse_masses
            .iter()
            .enumerate()
            .fold(0.0, |acc, (i, w)| acc + w * gradients[i].length_squared());

        if w_sum == f32::EPSILON {
            return 0.0;
        }

        let a_tilde = compliance / dt.powi(2);

        (-c - a_tilde * lagrange) / (w_sum + a_tilde)
    }
}
