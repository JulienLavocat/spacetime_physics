use log::debug;

use crate::{
    engine::collisions::Collision,
    tables::{PhysicsWorld, RigidBody},
};

use super::{get_bodies_mut, Solver};

pub struct PositionSolver;

impl Solver for PositionSolver {
    fn solve(
        _: &PhysicsWorld,
        collisions: &[Collision],
        bodies: &mut [RigidBody],
        _delta_time: f32,
    ) {
        let beta = 0.2; // more aggressive correction
        let slop = 0.01;

        for collision in collisions {
            let (a, b) = get_bodies_mut(collision.a, collision.b, bodies);

            if a.is_sleeping && b.is_sleeping {
                continue;
            }

            let normal = collision.points.normal;
            let depth = collision.points.depth;
            let correction_depth = (depth - slop).max(0.0);
            let correction = normal * (correction_depth * beta);

            let total_inv_mass = a.inv_mass + b.inv_mass;
            if total_inv_mass == 0.0 {
                continue;
            }

            let a_ratio = a.inv_mass / total_inv_mass;
            let b_ratio = b.inv_mass / total_inv_mass;

            a.transform.position -= correction * a_ratio;
            b.transform.position += correction * b_ratio;

            debug!(
                "PositionSolver applied: a_position={} -> {}, b_position={} -> {} , correction={:?}",
                a.transform.position, b.transform.position, -(correction * a_ratio), correction * b_ratio, correction
            );
        }
    }
}
