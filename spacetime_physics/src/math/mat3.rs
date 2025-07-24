use glam::Vec3;
use spacetimedb::SpacetimeType;

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

impl From<Mat3> for glam::Mat3 {
    fn from(mat: Mat3) -> Self {
        glam::Mat3::from_cols(
            Vec3::new(mat.m11, mat.m12, mat.m13),
            Vec3::new(mat.m21, mat.m22, mat.m23),
            Vec3::new(mat.m31, mat.m32, mat.m33),
        )
    }
}
