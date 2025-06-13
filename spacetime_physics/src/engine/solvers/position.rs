use log::debug;

use crate::{engine::collisions::Collision, tables::RigidBody};

pub struct PositionSolver;

impl PositionSolver {
    pub fn solve(collisions: &[Collision], bodies: &mut [RigidBody]) {
        let percent = 0.8; // typically 0.2 - 0.8: how aggressive the correction is
        let slop = 0.01; // penetration tolerance to prevent jitter

        for collision in collisions {
            let (a, b) = super::get_bodies_mut(collision.a, collision.b, bodies);

            let inv_mass_sum = a.inv_mass + b.inv_mass;
            if inv_mass_sum == 0.0 {
                continue; // static/static, nothing to resolve
            }

            let depth = (collision.points.depth - slop).max(0.0);
            let correction = collision.points.normal * (depth / inv_mass_sum) * percent;

            debug!(
                "Position correction: a={}, b={}, depth={}, correction={:?}",
                a.id, b.id, depth, correction
            );

            if a.inv_mass > 0.0 {
                a.transform.position -= correction * a.inv_mass;
            }
            if b.inv_mass > 0.0 {
                b.transform.position += correction * b.inv_mass;
            }
        }
    }
}
