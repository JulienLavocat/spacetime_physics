use crate::{
    engine::{collisions::Collision, solvers::get_bodies_mut},
    tables::{PhysicsWorld, RigidBody},
};

use super::Solver;

const RESTING_THRESHOLD: f32 = 0.1;
const MIN_BOUNCE_VELOCITY: f32 = -0.1;

pub(crate) struct ImpulseSolver;

impl Solver for ImpulseSolver {
    fn solve(world: &PhysicsWorld, collisions: &[Collision], bodies: &mut [RigidBody], _: f32) {
        for collision in collisions {
            let (a, b) = get_bodies_mut(collision.a, collision.b, bodies);

            if a.inv_mass + b.inv_mass == 0.0 {
                continue;
            }

            let normal = collision.points.normal.normalize();
            let rel_velocity = b.velocity - a.velocity;
            let vel_along_normal = rel_velocity.dot(normal);

            // Don't resolve if velocities are separating or nearly zero
            if vel_along_normal > 0.0 {
                continue;
            }

            if vel_along_normal > MIN_BOUNCE_VELOCITY {
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
                a.last_contact_normal = Some(normal);
            }

            if b.inv_mass > 0.0 {
                b.velocity += impulse * b.inv_mass;
                b.last_contact_normal = Some(-normal);
            }

            if vel_along_normal < -0.1 {
                if a.inv_mass > 0.0 {
                    a.is_sleeping = false;
                    a.sleep_timer = 0.0;
                }
                if b.inv_mass > 0.0 {
                    b.is_sleeping = false;
                    b.sleep_timer = 0.0;
                }
            }

            // Clamp very small post-collision relative velocities
            let new_rel_velocity = b.velocity - a.velocity;
            let new_vel_along_normal = new_rel_velocity.dot(normal);

            if new_vel_along_normal.abs() < RESTING_THRESHOLD
                && normal.dot(world.gravity.normalize()) < -0.9
            // nearly opposing gravity
            {
                if a.inv_mass > 0.0 {
                    a.velocity -= normal * new_vel_along_normal * a.inv_mass;
                }
                if b.inv_mass > 0.0 {
                    b.velocity += normal * new_vel_along_normal * b.inv_mass;
                }
            }
        }
    }
}
