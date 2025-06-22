use core::panic;

use parry3d::{
    query::contact,
    shape::{Ball, HalfSpace},
};

use crate::{engine::collisions2::Collider, math::Transform};

use super::CollisionPoint;

pub fn test_collision(
    transform_a: &Transform,
    collider_a: &Collider,
    transform_b: &Transform,
    collider_b: &Collider,
    prediction: f32,
) -> Option<CollisionPoint> {
    let result = match (collider_a, collider_b) {
        (Collider::Sphere(sphere_a), Collider::Sphere(sphere_b)) => contact(
            &transform_a.into(),
            &Ball::new(sphere_a.radius),
            &transform_b.into(),
            &Ball::new(sphere_b.radius),
            prediction,
        ),
        (Collider::Plane(plane), Collider::Sphere(sphere)) => contact(
            &transform_a.into(),
            &HalfSpace::new(plane.normal.into()),
            &transform_b.into(),
            &Ball::new(sphere.radius),
            prediction,
        ),
        (Collider::Sphere(sphere), Collider::Plane(plane)) => contact(
            &transform_a.into(),
            &Ball::new(sphere.radius),
            &transform_b.into(),
            &HalfSpace::new(plane.normal.into()),
            prediction,
        ),
        _ => {
            panic!("Unsupported collider types for collision detection");
        }
    };

    if let Err(err) = result {
        panic!("Collision detection failed: {}", err);
    }

    result.unwrap().map(|c| c.into())
}
