mod mat3;
mod quat;
mod vec3;

use std::f32::consts::PI;
pub use mat3::*;
pub use quat::*;
pub use vec3::*;

// Utility to convert degrees to radians
pub fn deg_to_rad(deg: f32) -> f32 {
    deg * PI / 180.0
}