use std::{collections::HashMap, fmt::Display};

use spacetimedb::{table, ReducerContext, SpacetimeType, Table};

use crate::math::{Mat3, Vec3};

pub type ColliderId = u64;

#[derive(SpacetimeType, Default, Clone, Copy, Debug, PartialEq)]
pub enum ColliderType {
    #[default]
    Sphere,
    Plane,
    Cuboid,
    Cylinder,
    Cone,
    Capsule,
    Triangle,
}

#[table(name = physics_colliders, public)]
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct Collider {
    #[primary_key]
    #[auto_inc]
    pub id: u64,
    #[index(btree)]
    pub world_id: u64,
    pub radius: f32,
    pub normal: Vec3,
    pub height: f32,
    pub half_extents: Vec3,
    pub half_height: f32,
    pub point_a: Vec3,
    pub point_b: Vec3,
    pub point_c: Vec3,
    pub collider_type: ColliderType,
}

impl Collider {
    pub fn insert(self, ctx: &ReducerContext) -> Self {
        ctx.db.physics_colliders().insert(self)
    }

    pub fn all(ctx: &ReducerContext, world_id: u64) -> HashMap<u64, Self> {
        ctx.db
            .physics_colliders()
            .world_id()
            .filter(world_id)
            .map(|collider| (collider.id, collider))
            .collect()
    }

    pub fn update(self, ctx: &ReducerContext) -> Self {
        ctx.db.physics_colliders().id().update(self)
    }

    pub fn delete(&self, ctx: &ReducerContext) {
        ctx.db.physics_colliders().id().delete(self.id);
    }

    pub fn id(&mut self, id: u64) -> &mut Self {
        self.id = id;
        self
    }

    pub fn sphere(world_id: u64, radius: f32) -> Self {
        Self {
            world_id,
            radius,
            collider_type: ColliderType::Sphere,
            ..Default::default()
        }
    }

    pub fn plane(world_id: u64, normal: Vec3) -> Self {
        Self {
            world_id,
            normal,
            collider_type: ColliderType::Plane,
            ..Default::default()
        }
    }

    pub fn cuboid(world_id: u64, half_extents: Vec3) -> Self {
        Self {
            world_id,
            half_extents,
            collider_type: ColliderType::Cuboid,
            ..Default::default()
        }
    }

    pub fn cylinder(world_id: u64, radius: f32, half_height: f32) -> Self {
        Self {
            world_id,
            radius,
            half_height,
            collider_type: ColliderType::Cylinder,
            ..Default::default()
        }
    }

    pub fn cone(world_id: u64, radius: f32, half_height: f32) -> Self {
        Self {
            world_id,
            radius,
            half_height,
            collider_type: ColliderType::Cone,
            ..Default::default()
        }
    }

    pub fn capsule(world_id: u64, radius: f32, half_height: f32) -> Self {
        Self {
            world_id,
            radius,
            half_height,
            collider_type: ColliderType::Capsule,
            ..Default::default()
        }
    }

    pub fn triangle(world_id: u64, point_a: Vec3, point_b: Vec3, point_c: Vec3) -> Self {
        Self {
            world_id,
            point_a,
            point_b,
            point_c,
            collider_type: ColliderType::Triangle,
            ..Default::default()
        }
    }

    pub fn inertia_tensor(&self, mass: f32) -> Mat3 {
        match self.collider_type {
            ColliderType::Plane => Mat3::ZERO,
            ColliderType::Sphere => sphere_inertia_tensor(mass, self.radius),
            ColliderType::Cuboid => cuboid_inertia_tensor(mass, self.half_extents),
            ColliderType::Cylinder => cylinder_inertia_tensor(mass, self.radius, self.half_height),
            ColliderType::Cone => cone_inertia_tensor(mass, self.radius, self.half_height),
            ColliderType::Capsule => {
                capsule_inertia_tensor(mass, self.radius, self.point_a, self.point_b)
            }
            ColliderType::Triangle => {
                triangle_inertia_tensor(mass, self.point_a, self.point_b, self.point_c)
            }
        }
    }
}

impl Display for Collider {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.collider_type {
            ColliderType::Sphere => write!(f, "Sphere(radius: {})", self.radius),
            ColliderType::Plane => write!(f, "Plane(normal: {})", self.normal),
            ColliderType::Cuboid => write!(f, "Cuboid(half_extents: {})", self.half_extents),
            ColliderType::Cylinder => write!(
                f,
                "Cylinder(radius: {}, half_height: {})",
                self.radius, self.half_height
            ),
            ColliderType::Cone => write!(
                f,
                "Cone(radius: {}, half_height: {})",
                self.radius, self.half_height
            ),
            ColliderType::Capsule => write!(
                f,
                "Capsule(radius: {}, point_a: {}, point_b: {})",
                self.radius, self.point_a, self.point_b
            ),
            ColliderType::Triangle => write!(
                f,
                "Triangle(point_a: {}, point_b: {}, point_c: {})",
                self.point_a, self.point_b, self.point_c
            ),
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

fn cylinder_inertia_tensor(mass: f32, radius: f32, half_height: f32) -> Mat3 {
    let factor = (1.0 / 12.0) * mass * (3.0 * radius * radius + half_height * half_height);
    Mat3::from_diagonal(Vec3::new(factor, factor, factor))
}

fn cone_inertia_tensor(mass: f32, radius: f32, half_height: f32) -> Mat3 {
    let factor = (3.0 / 10.0) * mass * (radius * radius + half_height * half_height);
    Mat3::from_diagonal(Vec3::new(factor, factor, factor))
}

fn capsule_inertia_tensor(mass: f32, radius: f32, point_a: Vec3, point_b: Vec3) -> Mat3 {
    let length = (point_b - point_a).length();
    let factor = (1.0 / 12.0) * mass * (3.0 * radius * radius + length * length);
    Mat3::from_diagonal(Vec3::new(factor, factor, factor))
}

fn triangle_inertia_tensor(mass: f32, point_a: Vec3, point_b: Vec3, point_c: Vec3) -> Mat3 {
    let ab = point_b - point_a;
    let ac = point_c - point_a;
    let area = 0.5 * ab.cross(ac).length();
    let factor = (1.0 / 6.0) * mass * area * area;
    Mat3::from_diagonal(Vec3::splat(factor))
}
