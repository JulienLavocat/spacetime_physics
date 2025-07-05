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
    pub size: Vec3,
    pub point_a: Vec3,
    pub point_b: Vec3,
    pub point_c: Vec3,
    pub collider_type: ColliderType,
}

impl Collider {
    pub fn insert(self, ctx: &ReducerContext) -> Self {
        ctx.db.physics_colliders().insert(self)
    }

    pub fn find(ctx: &ReducerContext, id: u64) -> Option<Self> {
        ctx.db.physics_colliders().id().find(id)
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

    pub fn delete_by_id(ctx: &ReducerContext, id: u64) {
        ctx.db.physics_colliders().id().delete(id);
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

    pub fn cuboid(world_id: u64, size: Vec3) -> Self {
        Self {
            world_id,
            size,
            collider_type: ColliderType::Cuboid,
            ..Default::default()
        }
    }

    pub fn cylinder(world_id: u64, radius: f32, height: f32) -> Self {
        Self {
            world_id,
            radius,
            height,
            collider_type: ColliderType::Cylinder,
            ..Default::default()
        }
    }

    pub fn cone(world_id: u64, radius: f32, height: f32) -> Self {
        Self {
            world_id,
            radius,
            height,
            collider_type: ColliderType::Cone,
            ..Default::default()
        }
    }

    pub fn capsule(world_id: u64, radius: f32, height: f32) -> Self {
        Self {
            world_id,
            radius,
            height,
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
            ColliderType::Cuboid => cuboid_inertia_tensor(mass, self.size),
            ColliderType::Cylinder => cylinder_inertia_tensor(mass, self.radius, self.height),
            ColliderType::Cone => cone_inertia_tensor(mass, self.radius, self.height),
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
            ColliderType::Cuboid => write!(f, "Cuboid(size: {})", self.size),
            ColliderType::Cylinder => write!(
                f,
                "Cylinder(radius: {}, height: {})",
                self.radius, self.height
            ),
            ColliderType::Cone => {
                write!(f, "Cone(radius: {}, height: {})", self.radius, self.height)
            }
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

fn cuboid_inertia_tensor(mass: f32, size: Vec3) -> Mat3 {
    let w = size.x;
    let h = size.y;
    let d = size.z;
    let i_x = (1.0 / 12.0) * mass * (h * h + d * d);
    let i_y = (1.0 / 12.0) * mass * (w * w + d * d);
    let i_z = (1.0 / 12.0) * mass * (w * w + h * h);
    Mat3::from_diagonal(Vec3::new(i_x, i_y, i_z))
}

fn sphere_inertia_tensor(mass: f32, radius: f32) -> Mat3 {
    let factor = (2.0 / 5.0) * mass * radius * radius;
    Mat3::from_diagonal(Vec3::splat(factor))
}

fn cylinder_inertia_tensor(mass: f32, radius: f32, height: f32) -> Mat3 {
    let i_xz = (1.0 / 12.0) * mass * (3.0 * radius * radius + height * height);
    let i_y = (1.0 / 2.0) * mass * radius * radius;
    Mat3::from_diagonal(Vec3::new(i_xz, i_y, i_xz))
}

fn cone_inertia_tensor(mass: f32, radius: f32, _height: f32) -> Mat3 {
    let i_xz = (3.0 / 20.0) * mass * radius * radius;
    let i_y = (3.0 / 10.0) * mass * radius * radius;
    Mat3::from_diagonal(Vec3::new(i_xz, i_y, i_xz))
}

fn capsule_inertia_tensor(mass: f32, radius: f32, point_a: Vec3, point_b: Vec3) -> Mat3 {
    let length = (point_b - point_a).length();
    let i = (1.0 / 12.0) * mass * (3.0 * radius * radius + length * length);
    Mat3::from_diagonal(Vec3::new(i, i, i)) // ← approximation
}

fn triangle_inertia_tensor(mass: f32, point_a: Vec3, point_b: Vec3, point_c: Vec3) -> Mat3 {
    let ab = point_b - point_a;
    let ac = point_c - point_a;
    let area = 0.5 * ab.cross(ac).length();
    let factor = (1.0 / 6.0) * mass * area * area;
    Mat3::from_diagonal(Vec3::splat(factor))
}
