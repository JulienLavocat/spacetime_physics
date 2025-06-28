use std::fmt::Display;

use spacetimedb::SpacetimeType;

use crate::math::{Mat3, Vec3};

#[derive(SpacetimeType, Debug, Clone, Copy, PartialEq)]
pub struct Sphere {
    pub radius: f32,
}

#[derive(SpacetimeType, Debug, Clone, Copy, PartialEq)]
pub struct Plane {
    pub normal: Vec3,
}

#[derive(SpacetimeType, Debug, Clone, Copy, PartialEq)]
pub struct Cuboid {
    pub half_extents: Vec3,
}

#[derive(SpacetimeType, Clone, Copy, Debug, PartialEq)]
pub enum Collider {
    Sphere(Sphere),
    Plane(Plane),
    Cuboid(Cuboid),
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

    pub fn plane(normal: Vec3) -> Self {
        Self::Plane(Plane { normal })
    }

    pub fn cuboid(half_extents: Vec3) -> Self {
        Self::Cuboid(Cuboid { half_extents })
    }

    pub fn inertia_tensor(&self, mass: f32) -> Mat3 {
        match self {
            Collider::Plane(_) => Mat3::ZERO,
            Collider::Sphere(sphere) => sphere_inertia_tensor(mass, sphere.radius),
            Collider::Cuboid(cuboid) => cuboid_inertia_tensor(mass, cuboid.half_extents),
        }
    }
}

impl Display for Collider {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Collider::Sphere(sphere) => write!(f, "Sphere(radius: {})", sphere.radius),
            Collider::Plane(plane) => write!(f, "Plane(normal: {})", plane.normal),
            Collider::Cuboid(cuboid) => write!(f, "Cuboid(half_extents: {})", cuboid.half_extents),
        }
    }
}

fn cuboid_inertia_tensor(mass: f32, half_extents: Vec3) -> Mat3 {
    let x2 = half_extents.x * half_extents.x;
    let y2 = half_extents.y * half_extents.y;
    let z2 = half_extents.z * half_extents.z;
    let factor = (1.0 / 3.0) * mass;

    Mat3::from_diagonal(Vec3::new(
        factor * (y2 + z2),
        factor * (x2 + z2),
        factor * (x2 + y2),
    ))
}

fn sphere_inertia_tensor(mass: f32, radius: f32) -> Mat3 {
    let factor = (2.0 / 5.0) * mass * radius * radius;
    Mat3::from_diagonal(Vec3::splat(factor))
}
