use super::Collider;
use parry3d::{
    bounding_volume::{Aabb, BoundingVolume},
    na::Isometry3,
    query::{intersection_test, Contact, Ray, RayCast, RayIntersection, Unsupported},
    shape::{Ball, Cuboid, HalfSpace},
};

/// Acts as a wrapper around spacetime_physics colliders and Parry's shapes,
pub enum ShapeWrapper {
    Sphere(Ball),
    Plane(HalfSpace),
    Cuboid(Cuboid),
}

impl ShapeWrapper {
    pub fn collision_aabb(&self, isometry: &Isometry3<f32>, prediction_distance: f32) -> Aabb {
        match self {
            ShapeWrapper::Sphere(sphere) => sphere.aabb(isometry).loosened(prediction_distance),
            ShapeWrapper::Plane(plane) => plane.aabb(isometry).loosened(prediction_distance),
            ShapeWrapper::Cuboid(cuboid) => cuboid.aabb(isometry).loosened(prediction_distance),
        }
    }

    pub fn cast_ray_and_get_normal(
        &self,
        isometry: &Isometry3<f32>,
        ray: &Ray,
        max_time_to_impact: f32,
        solid: bool,
    ) -> Option<RayIntersection> {
        match self {
            ShapeWrapper::Sphere(sphere) => {
                sphere.cast_ray_and_get_normal(isometry, ray, max_time_to_impact, solid)
            }
            ShapeWrapper::Plane(plane) => {
                plane.cast_ray_and_get_normal(isometry, ray, max_time_to_impact, solid)
            }
            ShapeWrapper::Cuboid(cuboid) => {
                cuboid.cast_ray_and_get_normal(isometry, ray, max_time_to_impact, solid)
            }
        }
    }

    pub fn contact(
        &self,
        isometry_a: &Isometry3<f32>,
        other: &ShapeWrapper,
        isometry_b: &Isometry3<f32>,
        prediction: f32,
    ) -> Result<Option<Contact>, Unsupported> {
        match (self, other) {
            (ShapeWrapper::Sphere(sphere_a), ShapeWrapper::Sphere(sphere_b)) => {
                parry3d::query::contact(isometry_a, sphere_a, isometry_b, sphere_b, prediction)
            }
            (ShapeWrapper::Sphere(sphere), ShapeWrapper::Plane(plane)) => {
                parry3d::query::contact(isometry_a, sphere, isometry_b, plane, prediction)
            }
            (ShapeWrapper::Sphere(sphere), ShapeWrapper::Cuboid(cuboid)) => {
                parry3d::query::contact(isometry_a, sphere, isometry_b, cuboid, prediction)
            }
            (ShapeWrapper::Plane(plane), ShapeWrapper::Sphere(sphere)) => {
                parry3d::query::contact(isometry_a, plane, isometry_b, sphere, prediction)
            }
            (ShapeWrapper::Plane(plane_a), ShapeWrapper::Plane(plane_b)) => {
                parry3d::query::contact(isometry_a, plane_a, isometry_b, plane_b, prediction)
            }
            (ShapeWrapper::Plane(plane), ShapeWrapper::Cuboid(cuboid)) => {
                parry3d::query::contact(isometry_a, plane, isometry_b, cuboid, prediction)
            }
            (ShapeWrapper::Cuboid(cuboid), ShapeWrapper::Sphere(sphere)) => {
                parry3d::query::contact(isometry_a, cuboid, isometry_b, sphere, prediction)
            }
            (ShapeWrapper::Cuboid(cuboid_a), ShapeWrapper::Cuboid(cuboid_b)) => {
                parry3d::query::contact(isometry_a, cuboid_a, isometry_b, cuboid_b, prediction)
            }
            (ShapeWrapper::Cuboid(cuboid), ShapeWrapper::Plane(plane)) => {
                parry3d::query::contact(isometry_a, cuboid, isometry_b, plane, prediction)
            }
        }
    }

    pub fn intersects(
        &self,
        isometry_a: &Isometry3<f32>,
        isometry_b: &Isometry3<f32>,
        other: &ShapeWrapper,
    ) -> Result<bool, Unsupported> {
        match (self, other) {
            (ShapeWrapper::Sphere(sphere_a), ShapeWrapper::Sphere(sphere_b)) => {
                intersection_test(isometry_a, sphere_a, isometry_b, sphere_b)
            }
            (ShapeWrapper::Sphere(sphere), ShapeWrapper::Plane(plane)) => {
                intersection_test(isometry_a, sphere, isometry_b, plane)
            }
            (ShapeWrapper::Sphere(sphere), ShapeWrapper::Cuboid(cuboid)) => {
                intersection_test(isometry_a, sphere, isometry_b, cuboid)
            }
            (ShapeWrapper::Plane(plane), ShapeWrapper::Sphere(sphere)) => {
                intersection_test(isometry_a, plane, isometry_b, sphere)
            }
            (ShapeWrapper::Plane(plane_a), ShapeWrapper::Plane(plane_b)) => {
                intersection_test(isometry_a, plane_a, isometry_b, plane_b)
            }
            (ShapeWrapper::Plane(plane), ShapeWrapper::Cuboid(cuboid)) => {
                intersection_test(isometry_a, plane, isometry_b, cuboid)
            }
            (ShapeWrapper::Cuboid(cuboid), ShapeWrapper::Sphere(sphere)) => {
                intersection_test(isometry_a, cuboid, isometry_b, sphere)
            }
            (ShapeWrapper::Cuboid(cuboid_a), ShapeWrapper::Cuboid(cuboid_b)) => {
                intersection_test(isometry_a, cuboid_a, isometry_b, cuboid_b)
            }
            (ShapeWrapper::Cuboid(cuboid), ShapeWrapper::Plane(plane)) => {
                intersection_test(isometry_a, cuboid, isometry_b, plane)
            }
        }
    }
}

impl From<Collider> for ShapeWrapper {
    fn from(collider: Collider) -> Self {
        match collider {
            Collider::Sphere(sphere) => ShapeWrapper::Sphere(Ball::new(sphere.radius)),
            Collider::Plane(plane) => ShapeWrapper::Plane(HalfSpace::new(plane.normal.into())),
            Collider::Cuboid(cuboid) => {
                ShapeWrapper::Cuboid(Cuboid::new(cuboid.half_extents.into()))
            }
        }
    }
}

impl From<&Collider> for ShapeWrapper {
    fn from(collider: &Collider) -> Self {
        match collider {
            Collider::Sphere(sphere) => ShapeWrapper::Sphere(Ball::new(sphere.radius)),
            Collider::Plane(plane) => ShapeWrapper::Plane(HalfSpace::new(plane.normal.into())),
            Collider::Cuboid(cuboid) => {
                ShapeWrapper::Cuboid(Cuboid::new(cuboid.half_extents.into()))
            }
        }
    }
}
