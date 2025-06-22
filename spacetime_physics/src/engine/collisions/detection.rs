use core::panic;

use parry3d::{
    math::Isometry,
    na::{Quaternion, Translation3, UnitQuaternion},
    query::contact,
    shape::{Ball, HalfSpace},
};

use crate::{
    engine::collisions::Collider,
    math::{Quat, Vec3},
};

use super::CollisionPoint;

pub fn test_collision(
    position_a: &Vec3,
    rotation_a: &Quat,
    collider_a: &Collider,
    position_b: &Vec3,
    rotation_b: &Quat,
    collider_b: &Collider,
    prediction: f32,
) -> Option<CollisionPoint> {
    let iso_a = Isometry::from_parts(
        Translation3::new(position_a.x, position_a.y, position_a.z),
        UnitQuaternion::from_quaternion(Quaternion::new(
            rotation_a.x,
            rotation_a.y,
            rotation_a.z,
            rotation_a.w,
        )),
    );
    let iso_b = Isometry::from_parts(
        Translation3::new(position_b.x, position_b.y, position_b.z),
        UnitQuaternion::from_quaternion(Quaternion::new(
            rotation_b.x,
            rotation_b.y,
            rotation_b.z,
            rotation_b.w,
        )),
    );

    let result = match (collider_a, collider_b) {
        (Collider::Sphere(sphere_a), Collider::Sphere(sphere_b)) => contact(
            &iso_a,
            &Ball::new(sphere_a.radius),
            &iso_b,
            &Ball::new(sphere_b.radius),
            prediction,
        ),
        (Collider::Plane(plane), Collider::Sphere(sphere)) => contact(
            &iso_a,
            &HalfSpace::new(plane.normal.into()),
            &iso_b,
            &Ball::new(sphere.radius),
            prediction,
        ),
        (Collider::Sphere(sphere), Collider::Plane(plane)) => contact(
            &iso_a,
            &Ball::new(sphere.radius),
            &iso_b,
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
