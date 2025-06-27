use spacetime_physics::{
    collisions::Collider,
    math::{Quat, Vec3},
    physics_rigid_bodies,
    physics_world::physics_world,
    schedule_physics_step_world, step_world, KinematicBody, PhysicsWorld, RigidBody, RigidBodyType,
};
use spacetimedb::{reducer, table, Identity, ReducerContext, ScheduleAt, Table};

#[table(name = players)]
pub struct Players {
    #[primary_key]
    pub id: Identity,
    pub pos: Vec3,
    pub rigid_body_id: u64,
}

#[table(name = physics_ticks, scheduled(physics_tick_world), public)]
pub struct PhysicTick {
    #[primary_key]
    pub world_id: u64,
    pub scheduled_at: ScheduleAt,
}

#[spacetimedb::reducer(init)]
pub fn init(ctx: &ReducerContext) {
    let world = PhysicsWorld::builder()
        .gravity(Vec3::new(0.0, -9.81, 0.0))
        .build()
        .insert(ctx);

    ctx.db.physics_ticks().insert(PhysicTick {
        world_id: world.id,
        scheduled_at: schedule_physics_step_world(&world),
    });
}

#[reducer(client_connected)]
fn on_connect(ctx: &ReducerContext) {
    // Create a rigid body representing the player's collider
    let rb = RigidBody::builder()
        .position(Vec3::new(0.0, 1.0, 0.0))
        .collider(Collider::sphere(1.0))
        .body_type(RigidBodyType::Kinematic)
        .build()
        .insert(ctx);

    // Insert the player into the database with the rigid body ID
    ctx.db.players().insert(Players {
        id: ctx.sender,
        pos: Vec3::new(0.0, 1.0, 0.0),
        rigid_body_id: rb.id,
    });
}

#[reducer(client_disconnected)]
fn on_disconnected(ctx: &ReducerContext) {
    // Remove the player and their rigid body from the database
    if let Some(player) = ctx.db.players().id().find(ctx.sender) {
        ctx.db.players().id().delete(ctx.sender);
        ctx.db
            .physics_rigid_bodies()
            .id()
            .delete(player.rigid_body_id);
    }
}

#[reducer]
pub fn physics_tick_world(ctx: &ReducerContext, tick: PhysicTick) {
    // spacetime_physics let the end user manage how the world should be stepped
    let world = ctx.db.physics_world().id().find(tick.world_id).unwrap();

    let kinematic_entities = ctx
        .db
        .players()
        .iter()
        .map(|c| (c.rigid_body_id, (c.pos, Quat::IDENTITY)))
        .collect::<Vec<KinematicBody>>();

    step_world(ctx, &world, kinematic_entities.as_slice());
}
