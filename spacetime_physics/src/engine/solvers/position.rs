use crate::{
    engine::collisions::Collision,
    math::Vec3,
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

            // Damp bouncing at rest contact
            if depth > slop {
                if a.inv_mass > 0.0 && a.velocity.dot(normal) > 0.0 {
                    a.velocity = a.velocity.project_onto_plane(normal);
                }
                if b.inv_mass > 0.0 && b.velocity.dot(-normal) > 0.0 {
                    b.velocity = b.velocity.project_onto_plane(-normal);
                }
            }

            // Optional velocity clamp to prevent micro-jitter
            if a.inv_mass > 0.0 && a.velocity.length_squared() < 1e-4 {
                a.velocity = Vec3::ZERO;
            }
            if b.inv_mass > 0.0 && b.velocity.length_squared() < 1e-4 {
                b.velocity = Vec3::ZERO;
            }
        }
    }
}
