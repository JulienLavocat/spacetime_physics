use crate::{tables::Collider, ColliderType};
use parry3d::{
    bounding_volume::{Aabb, BoundingVolume},
    na::Isometry3,
    query::{contact, intersection_test, Contact, Ray, RayCast, RayIntersection},
    shape::{Ball, Capsule, Cone, Cuboid, Cylinder, HalfSpace, Shape, Triangle},
};

/// Acts as a wrapper around spacetime_physics colliders and Parry's shapes,
#[derive(Debug)]
pub enum ShapeWrapper {
    Sphere(Ball),
    Plane(HalfSpace),
    Capsule(Capsule),
    Cuboid(Cuboid),
    Cylinder(Cylinder),
    Cone(Cone),
    Triangle(Triangle),
}

impl ShapeWrapper {
    pub fn collision_aabb(&self, isometry: &Isometry3<f32>, prediction_distance: f32) -> Aabb {
        match self {
            ShapeWrapper::Sphere(sphere) => sphere.aabb(isometry).loosened(prediction_distance),
            ShapeWrapper::Plane(plane) => plane.aabb(isometry).loosened(prediction_distance),
            ShapeWrapper::Cuboid(cuboid) => cuboid.aabb(isometry).loosened(prediction_distance),
            ShapeWrapper::Capsule(capsule) => capsule.aabb(isometry).loosened(prediction_distance),
            ShapeWrapper::Cylinder(cylinder) => {
                cylinder.aabb(isometry).loosened(prediction_distance)
            }
            ShapeWrapper::Cone(cone) => cone.aabb(isometry).loosened(prediction_distance),
            ShapeWrapper::Triangle(triangle) => {
                triangle.aabb(isometry).loosened(prediction_distance)
            }
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
            ShapeWrapper::Sphere(shape) => {
                shape.cast_ray_and_get_normal(isometry, ray, max_time_to_impact, solid)
            }
            ShapeWrapper::Plane(shape) => {
                shape.cast_ray_and_get_normal(isometry, ray, max_time_to_impact, solid)
            }
            ShapeWrapper::Cuboid(shape) => {
                shape.cast_ray_and_get_normal(isometry, ray, max_time_to_impact, solid)
            }
            ShapeWrapper::Capsule(shape) => {
                shape.cast_ray_and_get_normal(isometry, ray, max_time_to_impact, solid)
            }
            ShapeWrapper::Cylinder(shape) => {
                shape.cast_ray_and_get_normal(isometry, ray, max_time_to_impact, solid)
            }
            ShapeWrapper::Cone(shape) => {
                shape.cast_ray_and_get_normal(isometry, ray, max_time_to_impact, solid)
            }
            ShapeWrapper::Triangle(shape) => {
                shape.cast_ray_and_get_normal(isometry, ray, max_time_to_impact, solid)
            }
        }
    }

    pub fn as_parry_shape(&self) -> &dyn Shape {
        match self {
            ShapeWrapper::Sphere(sphere) => sphere,
            ShapeWrapper::Plane(plane) => plane,
            ShapeWrapper::Capsule(capsule) => capsule,
            ShapeWrapper::Cuboid(cuboid) => cuboid,
            ShapeWrapper::Cylinder(cylinder) => cylinder,
            ShapeWrapper::Cone(cone) => cone,
            ShapeWrapper::Triangle(triangle) => triangle,
        }
    }

    pub fn contact(
        &self,
        isometry_a: &Isometry3<f32>,
        other: &ShapeWrapper,
        isometry_b: &Isometry3<f32>,
        prediction: f32,
    ) -> Option<Contact> {
        let result = contact(
            isometry_a,
            self.as_parry_shape(),
            isometry_b,
            other.as_parry_shape(),
            prediction,
        );
        result.unwrap_or_default()
    }

    pub fn intersects(
        &self,
        isometry_a: &Isometry3<f32>,
        isometry_b: &Isometry3<f32>,
        other: &ShapeWrapper,
    ) -> bool {
        let result = intersection_test(
            isometry_a,
            self.as_parry_shape(),
            isometry_b,
            other.as_parry_shape(),
        );

        result.unwrap_or_default()
    }
}

impl From<Collider> for ShapeWrapper {
    fn from(collider: Collider) -> Self {
        ShapeWrapper::from(&collider)
    }
}

impl From<&Collider> for ShapeWrapper {
    fn from(collider: &Collider) -> Self {
        match collider.collider_type {
            ColliderType::Sphere => ShapeWrapper::Sphere(Ball::new(collider.radius)),
            ColliderType::Plane => ShapeWrapper::Plane(HalfSpace::new(collider.normal.into())),
            ColliderType::Cuboid => ShapeWrapper::Cuboid(Cuboid::new((collider.size / 2.0).into())),
            ColliderType::Capsule => {
                ShapeWrapper::Capsule(Capsule::new_y(collider.height / 2.0, collider.radius))
            }
            ColliderType::Cylinder => {
                ShapeWrapper::Cylinder(Cylinder::new(collider.height / 2.0, collider.radius))
            }
            ColliderType::Cone => {
                ShapeWrapper::Cone(Cone::new(collider.radius, collider.height / 2.0))
            }
            ColliderType::Triangle => ShapeWrapper::Triangle(Triangle::new(
                collider.point_a.into(),
                collider.point_b.into(),
                collider.point_c.into(),
            )),
        }
    }
}
