use log::debug;

use crate::{
    engine::collisions::Collision,
    tables::{PhysicsWorld, RigidBody},
};

use super::Solver;

const MAX_CORRECTION: f32 = 0.1;

pub struct PositionSolver;

impl Solver for PositionSolver {
    fn solve(_: &PhysicsWorld, collisions: &[Collision], bodies: &mut [RigidBody], _: f32) {
        let percent = 0.8; // typically 0.2 - 0.8: how aggressive the correction is
        let slop = 0.02; // penetration tolerance to prevent jitter

        for collision in collisions {
            let (a, b) = super::get_bodies_mut(collision.a, collision.b, bodies);

            let inv_mass_sum = a.inv_mass + b.inv_mass;
            if inv_mass_sum == 0.0 {
                continue; // static/static, nothing to resolve
            }

            let depth = (collision.points.depth - slop).clamp(0.0, MAX_CORRECTION);
            let correction = collision.points.normal * (depth / inv_mass_sum) * percent;

            debug!(
                "Body {} penetration depth: {}, contact normal: {:?}, position: {:?}",
                a.id, depth, collision.points.normal, a.transform.position
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
