use parry3d::query::Contact;

use crate::math::Vec3;

#[derive(Clone, Debug, PartialEq)]
pub struct CollisionPoint {
    /// Position of the contact on the first object.
    pub a: Vec3,
    /// Position of the contact on the second object.
    pub b: Vec3,
    /// Contact normal, pointing towards the exterior of the first shape.
    pub normal_a: Vec3,
    /// Contact normal, pointing towards the exterior of the second shape.
    ///
    /// If these contact data are expressed in world-space, this normal is equal to `-normal1`
    pub normal_b: Vec3,
    /// Distance between the two contact points.
    ///
    /// If this is negative, this contact represents a penetration.
    pub distance: f32,
}

impl From<Contact> for CollisionPoint {
    fn from(contact: Contact) -> Self {
        CollisionPoint {
            a: contact.point1.into(),
            b: contact.point2.into(),
            normal_a: contact.normal1.into(),
            normal_b: contact.normal2.into(),
            distance: contact.dist,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Collision {
    pub world_id: u64, // The ID of the physics world this collision occurred in
    /// The ID of the first rigid body involved in the collision
    pub a: u64,
    /// The ID of the second rigid body involved in the collision
    pub b: u64,
    /// The collision points between the two colliders
    pub points: CollisionPoint,
}
