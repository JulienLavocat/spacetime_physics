use engine::collisions::Collider;
use math::Vec3;
use spacetimedb::{reducer, ReducerContext};
use tables::{PhysicsWorld, RigidBody};

mod engine;
pub mod math;
pub mod tables;

#[reducer(init)]
fn init(ctx: &ReducerContext) {
    let world_id = PhysicsWorld::builder().sub_step(10).build().insert(ctx).id;

    RigidBody::builder()
        .world_id(world_id)
        .position(Vec3::new(1.0, 100.0, 0.0))
        .collider(Collider::sphere(1.0))
        .build()
        .insert(ctx);

    // RigidBody::builder()
    //     .world_id(world_id)
    //     .position(Vec3::new(0.0, 10.0, 0.0))
    //     .collider(Collider::cuboid(Vec3::new(1.0, 1.0, 1.0)))
    //     .build()
    //     .insert(ctx);

    RigidBody::builder()
        .world_id(world_id)
        .position(Vec3::new(0.0, 0.0, 0.0))
        .collider(Collider::plane(Vec3::Y))
        .mass(0.0)
        .build()
        .insert(ctx);
}

#[reducer]
fn physics_step_world(ctx: &ReducerContext, world: PhysicsWorld) {
    engine::step_world(ctx, &world);
}
