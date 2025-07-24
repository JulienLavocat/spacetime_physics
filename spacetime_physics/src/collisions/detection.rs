use glam::Vec3;

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
        .shape()
        .contact(&iso_a, body_b.shape(), &iso_b, prediction);

    result.map(|contact| {
        let world_a = Vec3::new(contact.point1.x, contact.point1.y, contact.point1.z);
        let world_b = Vec3::new(contact.point2.x, contact.point2.y, contact.point2.z);
        let local_a = body_a.rotation().inverse() * (world_a - body_a.position());
        let local_b = body_b.rotation().inverse() * (world_b - body_b.position());
        CollisionPoint {
            world_a,
            world_b,
            local_a,
            local_b,
            normal: Vec3::new(contact.normal1.x, contact.normal1.y, contact.normal1.z),
            distance: contact.dist,
        }
    })
}
