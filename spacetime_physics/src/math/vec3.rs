use std::{fmt::Display, hash::Hash};

use glam::Vec3 as GlamVec3;
use parry3d::{
    math::{Point, Vector},
    na::{Translation3, Unit, Vector3},
};
use spacetimedb::SpacetimeType;

#[derive(SpacetimeType, Default, Debug, Clone, Copy, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Hash for Vec3 {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.x.to_bits().hash(state);
        self.y.to_bits().hash(state);
        self.z.to_bits().hash(state);
    }
}

impl From<GlamVec3> for Vec3 {
    fn from(value: GlamVec3) -> Self {
        Vec3 {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

impl From<Vec3> for GlamVec3 {
    fn from(value: Vec3) -> Self {
        GlamVec3::new(value.x, value.y, value.z)
    }
}

impl From<&GlamVec3> for Vec3 {
    fn from(value: &GlamVec3) -> Self {
        Vec3 {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

impl From<&Vec3> for GlamVec3 {
    fn from(value: &Vec3) -> Self {
        GlamVec3::new(value.x, value.y, value.z)
    }
}

impl From<Vector3<f32>> for Vec3 {
    fn from(value: Vector3<f32>) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

impl From<Vec3> for Vector<f32> {
    fn from(value: Vec3) -> Self {
        Vector::new(value.x, value.y, value.z)
    }
}

impl From<Vec3> for Unit<Vector<f32>> {
    fn from(value: Vec3) -> Self {
        let vec = parry3d::na::Vector3::new(value.x, value.y, value.z);
        Unit::new_normalize(vec)
    }
}

impl From<Unit<Vector<f32>>> for Vec3 {
    fn from(value: Unit<Vector<f32>>) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

impl From<Point<f32>> for Vec3 {
    fn from(p: Point<f32>) -> Self {
        Self {
            x: p.x,
            y: p.y,
            z: p.z,
        }
    }
}

impl From<Vec3> for Point<f32> {
    fn from(v: Vec3) -> Self {
        Point::new(v.x, v.y, v.z)
    }
}

impl From<Vec3> for Translation3<f32> {
    fn from(v: Vec3) -> Self {
        Translation3::new(v.x, v.y, v.z)
    }
}

impl Display for Vec3 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Vec3({}, {}, {})", self.x, self.y, self.z)
    }
}
