use log::debug;

use crate::{
    engine::collisions2::Collision,
    tables::{PhysicsWorld, RigidBody},
};

use super::get_bodies_mut;

pub struct ImpulseSolver;

impl ImpulseSolver {
    pub fn solve(
        _: &PhysicsWorld,
        collisions: &[Collision],
        bodies: &mut [RigidBody],
        _delta_time: f32,
    ) {
        for collision in collisions {
            let (a, b) = get_bodies_mut(collision.a, collision.b, bodies);

            if a.is_sleeping && b.is_sleeping {
                continue;
            }

            let normal = collision.points.normal_a;
            let relative_velocity = b.velocity - a.velocity;
            let vel_along_normal = relative_velocity.dot(normal);

            if vel_along_normal > 0.0 {
                continue;
            }

            let restitution = a.restitution.min(b.restitution).clamp(0.0, 1.0);
            let inv_mass_sum = a.inv_mass + b.inv_mass;

            if inv_mass_sum == 0.0 {
                continue;
            }

            let j = -(1.0 + restitution) * vel_along_normal / inv_mass_sum;
            let impulse = normal * j;

            a.velocity -= impulse * a.inv_mass;
            b.velocity += impulse * b.inv_mass;

            // Friction
            let tangent = (relative_velocity - normal * vel_along_normal).normalize_or_zero();
            let jt = -relative_velocity.dot(tangent) / inv_mass_sum;
            let mu = 0.5 * (a.friction + b.friction);

            let friction_impulse = if jt.abs() < j * mu {
                tangent * jt
            } else {
                tangent * -j * mu
            };

            a.velocity -= friction_impulse * a.inv_mass;
            b.velocity += friction_impulse * b.inv_mass;

            debug!("ImpulseSolver applied: a_velocity={:?}, b_velocity={:?}, impulse={:?}, friction_impulse={:?}",
                   a.velocity, b.velocity, impulse, friction_impulse);
        }
    }
}
