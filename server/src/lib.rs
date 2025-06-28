use core::f32;

use log::info;
use spacetime_physics::{
    math::{Quat, Vec3},
    physics_world::physics_world,
    raycast_all, schedule_physics_step_world, step_world, Collider, PhysicsTrigger, PhysicsWorld,
    RigidBody, RigidBodyType,
};
use spacetimedb::{reducer, table, Identity, ReducerContext, ScheduleAt, Table};

#[table(name = players)]
pub struct Players {
    #[primary_key]
    pub id: Identity,
    pub position: Vec3,
    pub rotation: Quat,
    pub rigid_body_id: u64,
}

#[table(name = physics_ticks, scheduled(physics_tick_world))]
pub struct PhysicsWorldTick {
    #[primary_key]
    pub world_id: u64,
    pub scheduled_at: ScheduleAt,
}

#[reducer(init)]
pub fn init(ctx: &ReducerContext) {
    let world = PhysicsWorld::builder()
        .ticks_per_second(60.0) // The reducer responsible for stepping the physics world will be scheduled at 60Hz, see TickWorld bellow
        .gravity(Vec3::new(0.0, -9.81, 0.0))
        .debug_triggers(true)
        .build()
        .insert(ctx);

    // Create a small sphere that will fal towards the ground
    RigidBody::builder()
        .position(Vec3::new(0.0, 100.0, 0.0))
        .collider(Collider::sphere(1.0))
        .body_type(RigidBodyType::Dynamic)
        .build()
        .insert(ctx);

    // Create a large static plane that will act as the ground
    // RigidBody::builder()
    //     .position(Vec3::new(0.0, 10.0, 0.0))
    //     .collider(Collider::plane(Vec3::Y))
    //     .body_type(RigidBodyType::Static)
    //     .build()
    //     .insert(ctx);

    PhysicsTrigger::builder()
        .world_id(world.id)
        .collider(Collider::cuboid(Vec3::new(10.0, 100.0, 10.0)))
        .build()
        .insert(ctx);

    // Schedule the physics tick for the world
    ctx.db.physics_ticks().insert(PhysicsWorldTick {
        world_id: world.id,
        scheduled_at: schedule_physics_step_world(&world),
    });
}

#[reducer]
pub fn physics_tick_world(ctx: &ReducerContext, tick: PhysicsWorldTick) {
    // spacetime_physics let the end user manage how and when the world should be stepped
    let world = ctx.db.physics_world().id().find(tick.world_id).unwrap();

    // You can have kinematic entities, which are entities that are not affected by physics but can still interact with the physics world.
    // In this example player's positions are updated by the client directly
    let kinematic_entities = ctx
        .db
        .players()
        .iter()
        .map(|c| (c.rigid_body_id, (c.position, c.rotation)));

    // Update the physics world and synchorinze the kinematic entities positions and rotations
    step_world(ctx, &world, kinematic_entities);

    // This is an exampole of how to perform a raycast in the physics world.
    // You can use this in any reducer to implement shooting, picking, etc.
    let origin = Vec3::ZERO;
    let direction = Vec3::splat(100.0);
    let max_distance = f32::MAX;
    for item in raycast_all(ctx, world.id, origin, direction, max_distance, true) {
        info!("Raycast hit: {:?}", item);
    }
}
