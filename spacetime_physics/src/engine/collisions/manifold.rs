use parry3d::query::Contact;

use crate::math::Vec3;

#[derive(Clone, Debug, PartialEq)]
pub struct CollisionPoint {
    /// Position of the contact on the first object.
    pub a: Vec3,
    /// Position of the contact on the second object.
    pub b: Vec3,
    /// Contact normal, pointing towards the exterior of the first shape.
    pub normal: Vec3,
    /// Contact normal, pointing towards the exterior of the second shape.

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
            normal: contact.normal1.into(),
            distance: contact.dist,
        }
    }
}
