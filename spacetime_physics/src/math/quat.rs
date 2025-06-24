use std::{
    fmt::Display,
    ops::{Add, AddAssign, Mul, Sub, SubAssign},
};

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
        w: 0.0,
    };
    pub const IDENTITY: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    };

    pub fn new(x: f32, y: f32, z: f32, w: f32) -> Self {
        Self { x, y, z, w }
    }

    pub fn from_xyz(vec: Vec3, w: f32) -> Self {
        Self {
            x: vec.x,
            y: vec.y,
            z: vec.z,
            w,
        }
    }

    pub fn normalize(self) -> Self {
        let len = (self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w).sqrt();
        if len == 0.0 {
            return Self::ZERO;
        }
        Self {
            x: self.x / len,
            y: self.y / len,
            z: self.z / len,
            w: self.w / len,
        }
    }

    pub fn inverse(&self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
            w: self.w,
        }
    }

    pub fn xyz(&self) -> Vec3 {
        Vec3::new(self.x, self.y, self.z)
    }

    pub fn rotate(&self, vec: Vec3) -> Vec3 {
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

impl Mul<Quat> for Quat {
    type Output = Quat;

    fn mul(self, other: Quat) -> Self::Output {
        Quat {
            x: self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            y: self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            z: self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
            w: self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
        }
    }
}

impl Mul<Quat> for f32 {
    type Output = Quat;

    fn mul(self, q: Quat) -> Self::Output {
        Quat {
            x: self * q.x,
            y: self * q.y,
            z: self * q.z,
            w: self * q.w,
        }
    }
}

impl Mul<f32> for Quat {
    type Output = Quat;

    fn mul(self, scalar: f32) -> Self::Output {
        Quat {
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
            w: self.w * scalar,
        }
    }
}

impl Add for Quat {
    type Output = Quat;

    fn add(self, other: Quat) -> Self::Output {
        Quat {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
            w: self.w + other.w,
        }
    }
}

impl AddAssign for Quat {
    fn add_assign(&mut self, other: Quat) {
        self.x += other.x;
        self.y += other.y;
        self.z += other.z;
        self.w += other.w;
    }
}

impl Sub for Quat {
    type Output = Quat;

    fn sub(self, other: Quat) -> Self::Output {
        Quat {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
            w: self.w - other.w,
        }
    }
}

impl SubAssign for Quat {
    fn sub_assign(&mut self, other: Quat) {
        self.x -= other.x;
        self.y -= other.y;
        self.z -= other.z;
        self.w -= other.w;
    }
}

impl Display for Quat {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Quat({}, {}, {}, {})", self.x, self.y, self.z, self.w)
    }
}
