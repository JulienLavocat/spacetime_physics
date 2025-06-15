use std::fmt::Display;

use parry3d::{math::Isometry, na::Translation3};
use spacetimedb::SpacetimeType;

use super::{quat::Quat, vec3::Vec3};

#[derive(SpacetimeType, Clone, Copy, Debug, PartialEq, Default)]
pub struct Transform {
    pub position: Vec3,
    pub rotation: Quat,
    pub scale: Vec3,
}

impl Transform {
    pub const IDENTITY: Self = Self {
        position: Vec3::ZERO,
        rotation: Quat::ZERO,
        scale: Vec3::ONE,
    };

    pub fn new(position: Vec3, rotation: Quat, scale: Vec3) -> Self {
        Self {
            position,
            rotation,
            scale,
        }
    }

    pub fn from_xyz(x: f32, y: f32, z: f32) -> Self {
        Self {
            position: Vec3::new(x, y, z),
            rotation: Quat::ZERO,
            scale: Vec3::ONE,
        }
    }
}

impl From<&Transform> for Isometry<f32> {
    fn from(t: &Transform) -> Self {
        let translation = Translation3::new(t.position.x, t.position.y, t.position.z);
        Isometry::from_parts(translation, t.rotation.into())
    }
}

impl Display for Transform {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Transform(position={}, rotation={}, scale={})",
            self.position, self.rotation, self.scale
        )
    }
}
