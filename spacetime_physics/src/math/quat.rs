use std::{fmt::Display, ops::Mul};

use spacetimedb::SpacetimeType;

use super::Vec3;

#[derive(SpacetimeType, Debug, Clone, Copy, PartialEq, Default)]
pub struct Quat {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

impl Quat {
    pub const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    };

    pub fn new(x: f32, y: f32, z: f32, w: f32) -> Self {
        Self { x, y, z, w }
    }
}

impl Mul<Vec3> for Quat {
    type Output = Vec3;

    fn mul(self, vec: Vec3) -> Self::Output {
        let uv = Vec3::new(
            self.y * vec.z - self.z * vec.y,
            self.z * vec.x - self.x * vec.z,
            self.x * vec.y - self.y * vec.x,
        );
        let uuv = Vec3::new(
            self.y * uv.z - self.z * uv.y,
            self.z * uv.x - self.x * uv.z,
            self.x * uv.y - self.y * uv.x,
        );

        vec + (uv * (2.0 * self.w)) + uuv
    }
}

impl Display for Quat {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Quat({}, {}, {}, {})", self.x, self.y, self.z, self.w)
    }
}
