use crate::math::Vec3;

#[derive(Clone, Debug, PartialEq)]
pub struct CollisionPoint {
    /// Position of the contact on the first object in world space.
    pub world_a: Vec3,
    /// Position of the contact on the second object in world space;
    pub world_b: Vec3,
    /// Position of the contact on the first object in local space.
    pub local_a: Vec3,
    /// Position of the contact on the second object in local space.
    pub local_b: Vec3,
    /// Contact normal, pointing towards the exterior of the first shape.
    pub normal: Vec3,
    /// Contact normal, pointing towards the exterior of the second shape.

    /// Distance between the two contact points.
    ///
    /// If this is negative, this contact represents a penetration.
    pub distance: f32,
}
