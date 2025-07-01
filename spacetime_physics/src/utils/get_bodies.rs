use crate::RigidBodyData;

/// Retrieves mutable references to two `RigidBody` instances by their IDs from a slice of bodies.
/// This function assumes that the bodies are sorted by their IDs and that the IDs are unique.
pub fn get_bodies_mut(
    id_a: u64,
    id_b: u64,
    bodies: &mut [RigidBodyData],
) -> (&mut RigidBodyData, &mut RigidBodyData) {
    assert!(id_a != id_b);

    // Assume bodies are sorted by id
    let (min_id, max_id) = if id_a < id_b {
        (id_a, id_b)
    } else {
        (id_b, id_a)
    };

    let mid = bodies
        .binary_search_by_key(&max_id, |b| b.id)
        .expect("ID not found");
    let (left, right) = bodies.split_at_mut(mid);
    let a = left
        .binary_search_by_key(&min_id, |b| b.id)
        .map(|i| &mut left[i])
        .expect("ID not found");
    let b = &mut right[0];
    (a, b)
}

pub fn get_bodies_direct(
    index_a: usize,
    index_b: usize,
    bodies: &[RigidBodyData],
) -> (&RigidBodyData, &RigidBodyData) {
    assert!(index_a != index_b);
    assert!(index_a < bodies.len() && index_b < bodies.len());

    let a = &bodies[index_a];
    let b = &bodies[index_b];

    (a, b)
}
