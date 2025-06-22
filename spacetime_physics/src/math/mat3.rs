use std::ops::Mul;

use spacetimedb::SpacetimeType;

use super::Vec3;

#[derive(SpacetimeType, Default, Debug, Clone, Copy, PartialEq)]
pub struct Mat3 {
    /// We store the matrix this way due to SpacetimeDB's limitations with arrays.
    pub m11: f32,
    pub m12: f32,
    pub m13: f32,
    pub m21: f32,
    pub m22: f32,
    pub m23: f32,
    pub m31: f32,
    pub m32: f32,
    pub m33: f32,
}

impl Mat3 {
    pub const IDENTITY: Self = Self {
        m11: 1.0,
        m12: 0.0,
        m13: 0.0,
        m21: 0.0,
        m22: 1.0,
        m23: 0.0,
        m31: 0.0,
        m32: 0.0,
        m33: 1.0,
    };

    const ZERO: Self = Self {
        m11: 0.0,
        m12: 0.0,
        m13: 0.0,
        m21: 0.0,
        m22: 0.0,
        m23: 0.0,
        m31: 0.0,
        m32: 0.0,
        m33: 0.0,
    };

    #[allow(clippy::too_many_arguments)]
    pub fn new(
        m11: f32,
        m12: f32,
        m13: f32,
        m21: f32,
        m22: f32,
        m23: f32,
        m31: f32,
        m32: f32,
        m33: f32,
    ) -> Self {
        Self {
            m11,
            m12,
            m13,
            m21,
            m22,
            m23,
            m31,
            m32,
            m33,
        }
    }

    pub fn determinant(&self) -> f32 {
        self.m11 * (self.m22 * self.m33 - self.m23 * self.m32)
            - self.m12 * (self.m21 * self.m33 - self.m23 * self.m31)
            + self.m13 * (self.m21 * self.m32 - self.m22 * self.m31)
    }

    pub fn inverse(&self) -> Option<Self> {
        let det = self.determinant();
        if det.abs() < f32::EPSILON {
            return None; // Matrix cannot be inverted
        }

        let inv_det = 1.0 / det;

        Some(Self {
            m11: (self.m22 * self.m33 - self.m23 * self.m32) * inv_det,
            m12: -(self.m12 * self.m33 - self.m13 * self.m32) * inv_det,
            m13: (self.m12 * self.m23 - self.m13 * self.m22) * inv_det,

            m21: -(self.m21 * self.m33 - self.m23 * self.m31) * inv_det,
            m22: (self.m11 * self.m33 - self.m13 * self.m31) * inv_det,
            m23: -(self.m11 * self.m23 - self.m13 * self.m21) * inv_det,

            m31: (self.m21 * self.m32 - self.m22 * self.m31) * inv_det,
            m32: -(self.m11 * self.m32 - self.m12 * self.m31) * inv_det,
            m33: (self.m11 * self.m22 - self.m12 * self.m21) * inv_det,
        })
    }
}

impl Mul<Vec3> for Mat3 {
    type Output = Vec3;

    fn mul(self, vec: Vec3) -> Self::Output {
        Vec3::new(
            self.m11 * vec.x + self.m12 * vec.y + self.m13 * vec.z,
            self.m21 * vec.x + self.m22 * vec.y + self.m23 * vec.z,
            self.m31 * vec.x + self.m32 * vec.y + self.m33 * vec.z,
        )
    }
}
