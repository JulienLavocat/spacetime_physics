use spacetime_physics::{
    math::{Quat, Vec3},
    physics_world::physics_world,
    schedule_physics_tick, step_world, Collider, PhysicsTrigger, PhysicsWorld, RigidBody,
    RigidBodyType,
};
use spacetimedb::{rand::Rng, reducer, table, Identity, ReducerContext, ScheduleAt, Table};

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
    #[auto_inc]
    pub id: u64,
    pub world_id: u64,
    pub scheduled_at: ScheduleAt,
}

#[reducer(init)]
pub fn init(ctx: &ReducerContext) {
    let world = PhysicsWorld::builder()
        .ticks_per_second(60.0) // The reducer responsible for stepping the physics world will be scheduled at 60Hz, see TickWorld bellow
        .gravity(Vec3::new(0.0, -9.81, 0.0))
        // .debug_triggers(true)
        // .debug_time(true)
        // .debug_broad_phase(true)
        // .debug_narrow_phase(true)
        // .debug(true)
        .build()
        .insert(ctx);

    let range = -100000.0..100000.0;
    let sphere_collider = Collider::sphere(world.id, 1.0).insert(ctx).id;
    for _ in 0..5000 {
        RigidBody::builder()
            .position(Vec3::new(
                ctx.rng().gen_range(range.clone()),
                100.0,
                ctx.rng().gen_range(range.clone()),
            ))
            .collider_id(sphere_collider)
            .body_type(RigidBodyType::Dynamic)
            .build()
            .insert(ctx);
    }

    // Create a small sphere that will fal towards the ground
    RigidBody::builder()
        .position(Vec3::new(0.0, 10.0, 0.0))
        .collider_id(sphere_collider)
        .body_type(RigidBodyType::Dynamic)
        .build()
        .insert(ctx);

    // Floor
    // let floor_collider = Collider::cuboid(world.id, Vec3::new(1000.0, 1.0, 1000.0))
    //     .insert(ctx)
    //     .id;
    // RigidBody::builder()
    //     .position(Vec3::new(0.0, -1.0, 0.0))
    //     .collider_id(floor_collider)
    //     .body_type(RigidBodyType::Static)
    //     .build()
    //     .insert(ctx);

    RigidBody::builder()
        .position(Vec3::splat(-2.0))
        .collider_id(sphere_collider)
        .body_type(RigidBodyType::Static)
        .build()
        .insert(ctx);

    let trigger_collider = Collider::cuboid(world.id, Vec3::new(10.0, 10.0, 10.0))
        .insert(ctx)
        .id;
    PhysicsTrigger::builder()
        .world_id(world.id)
        .collider_id(trigger_collider)
        .build()
        .insert(ctx);

    // Schedule the physics tick for the world
    ctx.db.physics_ticks().insert(PhysicsWorldTick {
        id: 0,
        world_id: world.id,
        scheduled_at: schedule_physics_tick(&world),
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
}
