use std::fmt::Display;

use spacetimedb::SpacetimeType;

use crate::math::{Transform, Vec3};

#[derive(SpacetimeType, Clone, Copy, Debug, PartialEq, Default)]
pub struct Plane {
    pub normal: Vec3,
    pub distance: f32,
}

#[derive(SpacetimeType, Clone, Copy, Debug, PartialEq, Default)]
pub struct Sphere {
    pub radius: f32,
}

#[derive(SpacetimeType, Clone, Copy, Debug, PartialEq)]
pub enum Collider {
    Sphere(Sphere),
    Plane(Plane),
}

impl Default for Collider {
    fn default() -> Self {
        Collider::Sphere(Sphere { radius: 1.0 })
    }
}

impl Collider {
    pub fn sphere(radius: f32) -> Self {
        Self::Sphere(Sphere { radius })
    }

    pub fn plane(normal: Vec3, distance: f32) -> Self {
        Self::Plane(Plane { normal, distance })
    }
}

impl Display for Collider {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Collider::Sphere(Sphere { radius }) => write!(f, "Sphere(radius={})", radius),
            Collider::Plane(Plane { normal, distance }) => {
                write!(f, "Plane(normal={}, distance={})", normal, distance)
            }
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct CollisionPoints {
    pub a: Vec3,      // Furthest point of A into B
    pub b: Vec3,      // Furthest point of B into A
    pub normal: Vec3, // Normal of the collision
    pub depth: f32,   // Depth of the collision
}

impl Display for CollisionPoints {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Collision(a={}, b={}, normal={}, depth={})",
            self.a, self.b, self.normal, self.depth
        )
    }
}

impl CollisionPoints {
    pub fn test(
        a: &Collider,
        a_transform: &Transform,
        b: &Collider,
        b_transform: &Transform,
    ) -> Option<CollisionPoints> {
        match (&a, &b) {
            (
                Collider::Sphere(Sphere { radius: r_a }),
                Collider::Sphere(Sphere { radius: r_b }),
            ) => test_sphere_sphere(a_transform.position, *r_a, b_transform.position, *r_b),
            (Collider::Sphere(Sphere { radius }), Collider::Plane(Plane { normal, distance })) => {
                test_sphere_plane(a_transform.position, *radius, *normal, *distance)
            }
            (Collider::Plane(Plane { normal, distance }), Collider::Sphere(Sphere { radius })) => {
                test_plane_sphere(*normal, *distance, b_transform.position, *radius)
            }
            _ => panic!("Unsupported collider combination: {} vs {}", a, b),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Collision {
    pub world_id: u64, // The ID of the physics world this collision occurred in
    /// The ID of the first rigid body involved in the collision
    pub a: u64,
    /// The ID of the second rigid body involved in the collision
    pub b: u64,
    /// The collision points between the two colliders
    pub points: CollisionPoints,
}

impl Display for Collision {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Collision(a={}, b={}, points={})",
            self.a, self.b, self.points
        )
    }
}

fn test_sphere_sphere(a_pos: Vec3, r_a: f32, b_pos: Vec3, r_b: f32) -> Option<CollisionPoints> {
    let delta = b_pos - a_pos;
    let distance = delta.length();
    let depth = r_a + r_b - distance;

    if depth > 0.0 {
        let normal = delta.normalize_or(Vec3::Y); // avoid NaN on exact overlap
        let a_point = a_pos + normal * r_a;
        let b_point = b_pos - normal * r_b;

        Some(CollisionPoints {
            a: a_point,
            b: b_point,
            normal,
            depth,
        })
    } else {
        None
    }
}

fn test_sphere_plane(
    center: Vec3,
    radius: f32,
    normal: Vec3,
    distance: f32,
) -> Option<CollisionPoints> {
    let signed_dist = center.dot(normal) - distance;
    let depth = radius - signed_dist;

    if depth > 0.0 {
        let normal = if signed_dist < 0.0 { -normal } else { normal };

        let a_point = center - normal * radius; // Point on sphere surface
        let b_point = center - normal * signed_dist; // Closest point on plane

        Some(CollisionPoints {
            a: a_point,
            b: b_point,
            normal,
            depth,
        })
    } else {
        None
    }
}

fn test_plane_sphere(
    normal: Vec3,
    distance: f32,
    center: Vec3,
    radius: f32,
) -> Option<CollisionPoints> {
    test_sphere_plane(center, radius, normal, distance).map(|mut col| {
        // Flip collision normal and points
        std::mem::swap(&mut col.a, &mut col.b);
        col.normal = -col.normal;
        col
    })
}
