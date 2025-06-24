use spacetimedb::SpacetimeType;

use crate::math::Vec3;

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
}
