use parry3d::shape::Ball;
use spacetimedb::SpacetimeType;

use crate::math::Vec3;

#[derive(SpacetimeType, Debug, Clone, Copy, PartialEq)]
pub struct Sphere {
    pub radius: f32,
}

impl From<Sphere> for Ball {
    fn from(s: Sphere) -> Self {
        Ball::new(s.radius)
    }
}

#[derive(SpacetimeType, Debug, Clone, Copy, PartialEq)]
pub struct Plane {
    pub normal: Vec3,
}

impl From<&Plane> for parry3d::shape::HalfSpace {
    fn from(value: &Plane) -> Self {
        parry3d::shape::HalfSpace {
            normal: value.normal.into(),
        }
    }
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

    pub fn plane(normal: Vec3) -> Self {
        Self::Plane(Plane { normal })
    }
}
