use crate::tables::{PhysicsWorld, RigidBody};

use super::collisions::Collision;

pub(crate) mod impulse;
pub(crate) mod position;

pub(crate) trait Solver {
    fn solve(
        world: &PhysicsWorld,
        collisions: &[Collision],
        bodies: &mut [RigidBody],
        delta_time: f32,
    );
}

pub(crate) fn get_bodies_mut(
    id_a: u64,
    id_b: u64,
    bodies: &mut [RigidBody],
) -> (&mut RigidBody, &mut RigidBody) {
    assert!(id_a != id_b);

    // Assume bodies are sorted by id
    let (min_id, max_id) = if id_a < id_b {
        (id_a, id_b)
    } else {
        (id_b, id_a)
    };
    let mid = bodies
        .iter()
        .position(|b| b.id == max_id)
        .expect("ID not found");

    let (left, right) = bodies.split_at_mut(mid);
    let a = left
        .iter_mut()
        .find(|b| b.id == min_id)
        .expect("ID not found");
    let b = &mut right[0];
    (a, b)
}
