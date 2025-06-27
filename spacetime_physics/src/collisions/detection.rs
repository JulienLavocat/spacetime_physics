use core::panic;

use crate::RigidBody;

use super::{CollisionPoint, ShapeWrapper};

pub fn test_collision(
    body_a: &RigidBody,
    body_b: &RigidBody,
    prediction: f32,
) -> Option<CollisionPoint> {
    let iso_a = body_a.into();
    let iso_b = body_b.into();

    let collider_a: ShapeWrapper = body_a.collider.into();
    let collider_b: ShapeWrapper = body_b.collider.into();
    let result = collider_a.contact(&iso_a, &collider_b, &iso_b, prediction);

    if let Err(err) = result {
        panic!("Collision detection failed: {}", err);
    }

    result.unwrap().map(|contact| {
        let world_a = contact.point1.into();
        let world_b = contact.point2.into();
        let local_a = body_a.rotation.inverse().rotate(world_a - body_a.position);
        let local_b = body_b.rotation.inverse().rotate(world_b - body_b.position);
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
