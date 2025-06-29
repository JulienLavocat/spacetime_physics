use core::panic;

use crate::RigidBodyData;

use super::CollisionPoint;

pub fn test_collision(
    body_a: &RigidBodyData,
    body_b: &RigidBodyData,
    prediction: f32,
) -> Option<CollisionPoint> {
    let iso_a = body_a.into();
    let iso_b = body_b.into();

    let result = body_a
        .shape
        .contact(&iso_a, &body_b.shape, &iso_b, prediction);

    if let Err(err) = result {
        panic!("Collision detection failed: {}", err);
    }

    result.unwrap().map(|contact| {
        let world_a = contact.point1.into();
        let world_b = contact.point2.into();
        let local_a = body_a
            .rb
            .rotation
            .inverse()
            .rotate(world_a - body_a.rb.position);
        let local_b = body_b
            .rb
            .rotation
            .inverse()
            .rotate(world_b - body_b.rb.position);
        CollisionPoint {
            world_a,
            world_b,
            local_a,
            local_b,
            normal: contact.normal1.into(),
            distance: contact.dist,
        }
    })
}
