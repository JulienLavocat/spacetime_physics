use crate::{
    engine::{collisions::Collision, solvers::get_bodies_mut},
    tables::RigidBody,
};

use super::Solver;

pub(crate) struct ImpulseSolver;

impl Solver for ImpulseSolver {
    fn solve(collisions: &[Collision], bodies: &mut [RigidBody], _: f32) {
        for collision in collisions {
            let (a, b) = get_bodies_mut(collision.a, collision.b, bodies);

            if a.inv_mass + b.inv_mass == 0.0 {
                continue;
            }

            let normal = collision.points.normal.normalize();
            let rel_velocity = b.velocity - a.velocity;
            let vel_along_normal = rel_velocity.dot(normal);

            // Don't resolve if velocities are separating
            if vel_along_normal > 0.0 {
                continue;
            }

            // Restitution (bounciness)
            let e = a.restitution.min(b.restitution);

            // Impulse scalar
            let j = -(1.0 + e) * vel_along_normal / (a.inv_mass + b.inv_mass);

            // Apply impulse
            let impulse = normal * j;

            if a.inv_mass > 0.0 {
                a.velocity -= impulse * a.inv_mass;
            }

            if b.inv_mass > 0.0 {
                b.velocity += impulse * b.inv_mass;
            }
        }
    }
}
