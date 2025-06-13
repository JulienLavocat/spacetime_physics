use engine::collisions::Collider;
use math::{Transform, Vec3};
use spacetimedb::{reducer, ReducerContext};
use tables::{PhysicsWorld, RigidBody};

mod engine;
pub mod math;
pub mod tables;

#[reducer(init)]
fn init(ctx: &ReducerContext) {
    let world_id = PhysicsWorld::new(60.0, Vec3::new(0.0, -9.81, 0.0))
        .insert(ctx)
        .id;

    RigidBody::new(
        world_id,
        Transform::from_xyz(0.0, 10.0, 0.0),
        Vec3::ZERO,
        Vec3::ZERO,
        1.0,
        Collider::sphere(1.0),
    )
    .insert(ctx);

    RigidBody::new(
        world_id,
        Transform::from_xyz(0.0, 0.0, 0.0),
        Vec3::ZERO,
        Vec3::ZERO,
        0.0,
        Collider::plane(Vec3::Y, 0.0),
    )
    .insert(ctx);
}

#[reducer]
fn physics_step_world(ctx: &ReducerContext, world: PhysicsWorld) {
    engine::step_world(ctx, &world);
}
