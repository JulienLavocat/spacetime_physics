use std::fmt::Display;

use glam::Quat as GlamQuat;
use parry3d::na::{Quaternion, UnitQuaternion};
use spacetimedb::SpacetimeType;

#[derive(SpacetimeType, Debug, Clone, Copy, PartialEq, Default)]
pub struct Quat {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

impl From<Quat> for GlamQuat {
    fn from(value: Quat) -> Self {
        GlamQuat::from_xyzw(value.x, value.y, value.z, value.w)
    }
}

impl From<GlamQuat> for Quat {
    fn from(value: GlamQuat) -> Self {
        Quat {
            x: value.x,
            y: value.y,
            z: value.z,
            w: value.w,
        }
    }
}

impl From<&Quat> for GlamQuat {
    fn from(value: &Quat) -> Self {
        GlamQuat::from_xyzw(value.x, value.y, value.z, value.w)
    }
}

impl From<&GlamQuat> for Quat {
    fn from(value: &GlamQuat) -> Self {
        Quat {
            x: value.x,
            y: value.y,
            z: value.z,
            w: value.w,
        }
    }
}

impl From<Quat> for UnitQuaternion<f32> {
    fn from(value: Quat) -> Self {
        let quaternion = Quaternion::new(value.x, value.y, value.z, value.w);
        UnitQuaternion::from_quaternion(quaternion)
    }
}

impl From<UnitQuaternion<f32>> for Quat {
    fn from(value: UnitQuaternion<f32>) -> Self {
        let quaternion = value.into_inner();
        Quat {
            x: quaternion.i,
            y: quaternion.j,
            z: quaternion.k,
            w: quaternion.w,
        }
    }
}

impl Display for Quat {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Quat({}, {}, {}, {})", self.x, self.y, self.z, self.w)
    }
}
