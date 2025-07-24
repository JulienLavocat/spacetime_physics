use crate::{Collider, RigidBody, RigidBodyProperties};

use super::*;

fn shared_setup() -> (PhysicsWorld, Vec<RigidBodyData>) {
    let world = PhysicsWorld::builder().build();
    let collider = Collider::cuboid(world.id, Vec3::new(1.0, 1.0, 1.0));
    let properties = RigidBodyProperties::builder().mass(1.0).build();
    let body = RigidBodyData::new(
        RigidBody::builder()
            .collider_id(collider.id)
            .properties_id(properties.id)
            .build(),
        &properties,
        &collider,
    );
    (world, vec![body])
}

#[test]
fn test_single_rigidbody_free_fall_3_steps() {
    let (world, mut bodies) = shared_setup();
    let dt = world.time_step / world.sub_step as f32;

    // Integrate bodies
    integrate_bodies(&mut bodies, &world, dt);

    let body = &mut bodies[0];

    let expected = Vec3::new(0.0, -0.008175, 0.0);
    assert_eq!(body.linear_velocity(), expected);
}
