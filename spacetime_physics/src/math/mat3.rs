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

    pub const ZERO: Self = Self {
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

    pub fn from_diagonal(diagonal: Vec3) -> Self {
        Self {
            m11: diagonal.x,
            m12: 0.0,
            m13: 0.0,
            m21: 0.0,
            m22: diagonal.y,
            m23: 0.0,
            m31: 0.0,
            m32: 0.0,
            m33: diagonal.z,
        }
    }

    pub fn determinant(&self) -> f32 {
        self.m11 * (self.m22 * self.m33 - self.m23 * self.m32)
            - self.m12 * (self.m21 * self.m33 - self.m23 * self.m31)
            + self.m13 * (self.m21 * self.m32 - self.m22 * self.m31)
    }

    pub fn inverse(&self) -> Self {
        let det = self.determinant();
        if det.abs() < f32::EPSILON {
            return Mat3::ZERO;
        }

        let inv_det = 1.0 / det;

        Self {
            m11: (self.m22 * self.m33 - self.m23 * self.m32) * inv_det,
            m12: -(self.m12 * self.m33 - self.m13 * self.m32) * inv_det,
            m13: (self.m12 * self.m23 - self.m13 * self.m22) * inv_det,

            m21: -(self.m21 * self.m33 - self.m23 * self.m31) * inv_det,
            m22: (self.m11 * self.m33 - self.m13 * self.m31) * inv_det,
            m23: -(self.m11 * self.m23 - self.m13 * self.m21) * inv_det,

            m31: (self.m21 * self.m32 - self.m22 * self.m31) * inv_det,
            m32: -(self.m11 * self.m32 - self.m12 * self.m31) * inv_det,
            m33: (self.m11 * self.m22 - self.m12 * self.m21) * inv_det,
        }
    }

    pub fn transpose(&self) -> Self {
        Self {
            m11: self.m11,
            m12: self.m21,
            m13: self.m31,
            m21: self.m12,
            m22: self.m22,
            m23: self.m32,
            m31: self.m13,
            m32: self.m23,
            m33: self.m33,
        }
    }
}

impl Mul<Mat3> for Mat3 {
    type Output = Self;

    fn mul(self, other: Self) -> Self::Output {
        Self {
            m11: self.m11 * other.m11 + self.m12 * other.m21 + self.m13 * other.m31,
            m12: self.m11 * other.m12 + self.m12 * other.m22 + self.m13 * other.m32,
            m13: self.m11 * other.m13 + self.m12 * other.m23 + self.m13 * other.m33,

            m21: self.m21 * other.m11 + self.m22 * other.m21 + self.m23 * other.m31,
            m22: self.m21 * other.m12 + self.m22 * other.m22 + self.m23 * other.m32,
            m23: self.m21 * other.m13 + self.m22 * other.m23 + self.m23 * other.m33,

            m31: self.m31 * other.m11 + self.m32 * other.m21 + self.m33 * other.m31,
            m32: self.m31 * other.m12 + self.m32 * other.m22 + self.m33 * other.m32,
            m33: self.m31 * other.m13 + self.m32 * other.m23 + self.m33 * other.m33,
        }
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

impl Mul<Mat3> for f32 {
    type Output = Mat3;

    fn mul(self, mat: Mat3) -> Self::Output {
        Mat3 {
            m11: self * mat.m11,
            m12: self * mat.m12,
            m13: self * mat.m13,
            m21: self * mat.m21,
            m22: self * mat.m22,
            m23: self * mat.m23,
            m31: self * mat.m31,
            m32: self * mat.m32,
            m33: self * mat.m33,
        }
    }
}
