use log::debug;
use spacetime_physics::{
    math::{Quat, Vec3},
    physics_raycasts,
    physics_world::physics_world,
    schedule_physics_tick, step_world, Collider, PhysicsWorld, RayCast, RigidBody,
    RigidBodyProperties, RigidBodyType, Trigger,
};
use spacetimedb::{rand::Rng, reducer, table, Identity, ReducerContext, ScheduleAt, Table};

#[table(name = players)]
pub struct Players {
    #[primary_key]
    pub id: Identity,
    pub position: Vec3,
    pub rotation: Quat,
    pub rigid_body_id: u64,
    pub weapon_raycast_id: u64,
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
    // Initialize the physics world with a default gravity
    // You can customize the physics world properties like gravity, ticks per second, etc.
    // Multiple physics worlds can be created, each with its own properties and entities.
    // Different worlds never interact with each other.
    let world = PhysicsWorld::builder()
        .ticks_per_second(60.0) // The reducer responsible for stepping the physics world will be scheduled at 60Hz, see TickWorld bellow
        .gravity(Vec3::new(0.0, -9.81, 0.0)) // The default gravity is set to Earth's gravity, this
        // is the default value, but you can change it to whatever you want.
        .sub_step(0)
        .debug_time(true)
        .build()
        .insert(ctx);

    let range = 0.0..10000.0;
    let sphere_properties = RigidBodyProperties::builder().build().insert(ctx).id;
    let sphere_collider = Collider::sphere(world.id, 1.0).insert(ctx).id;
    let trigger_collider = Collider::cuboid(world.id, Vec3::new(1.0, 1.0, 1.0))
        .insert(ctx)
        .id;
    for _ in 0..2000 {
        RigidBody::builder()
            .position(Vec3::new(
                ctx.rng().gen_range(range.clone()),
                100.0,
                ctx.rng().gen_range(range.clone()),
            ))
            .collider_id(sphere_collider)
            .properties_id(sphere_properties)
            .body_type(RigidBodyType::Dynamic)
            .build()
            .insert(ctx);
    }

    for _ in 0..15000 {
        Trigger::builder()
            .position(Vec3::new(
                ctx.rng().gen_range(range.clone()),
                100.0,
                ctx.rng().gen_range(range.clone()),
            ))
            .collider_id(trigger_collider)
            .build()
            .insert(ctx);
    }

    for _ in 0..5000 {
        RayCast::new(
            world.id,
            Vec3::new(
                ctx.rng().gen_range(range.clone()),
                100.0,
                ctx.rng().gen_range(range.clone()),
            ),
            Vec3::Z,
            100.0,
            false,
        )
        .insert(ctx);
    }

    // Create a small sphere that will fal towards the ground
    // RigidBody::builder()
    //     .position(Vec3::new(1.0, 3.0, 100.0))
    //     .collider_id(sphere_collider)
    //     .body_type(RigidBodyType::Dynamic)
    //     .properties_id(sphere_properties)
    //     .build()
    //     .insert(ctx);

    // Create a small cube that will fall towards the ground
    // let cube_properties = RigidBodyProperties::builder().build().insert(ctx).id;
    // let cube_collider = Collider::cuboid(world.id, Vec3::new(1.0, 1.0, 1.0))
    //     .insert(ctx)
    //     .id;
    // RigidBody::builder()
    //     .position(Vec3::new(0.0, 5.0, 0.0))
    //     .collider_id(cube_collider)
    //     .body_type(RigidBodyType::Dynamic)
    //     .properties_id(cube_properties)
    //     .build()
    //     .insert(ctx);

    // Floor
    // let floor_collider = Collider::plane(world.id, Vec3::Y).insert(ctx).id;
    // let floor_properties = RigidBodyProperties::builder()
    //     .friction_static_coefficient(0.5)
    //     .friction_dynamic_coefficient(0.5)
    //     .restitution_coefficient(0.1)
    //     .build()
    //     .insert(ctx)
    //     .id;
    // RigidBody::builder()
    //     .position(Vec3::new(0.0, -1.0, 0.0))
    //     .collider_id(floor_collider)
    //     .body_type(RigidBodyType::Static)
    //     .properties_id(floor_properties)
    //     .build()
    //     .insert(ctx);
    //
    // // A trigger is a special type of collider that only detects collisions and does not apply any forces.
    // // Triggers are useful for things like detecting when a player enters a certain area, or when a projectile hits a target.
    // // A good example of a trigger is an area of effect, like a healing area or a damage area.
    // let trigger_collider = Collider::cuboid(world.id, Vec3::new(1.0, 1.0, 1.0))
    //     .insert(ctx)
    //     .id;
    // Trigger::builder()
    //     .position(Vec3::new(0.0, 1.0, 0.0))
    //     .collider_id(trigger_collider)
    //     .build()
    //     .insert(ctx);

    // You can create raycasts that will be used to detect collisions with rigid bodies.
    // This is useful for things like shooting, where you want to detect if a ray intersects with a rigid body.
    // RayCasts created this way will be updated automatically by the physics world and are
    // generally more efficient. For instant raycasts, you can use PhyssicsWorld::raycast().
    RayCast::new(world.id, Vec3::ZERO, Vec3::Z, 100.0, false).insert(ctx);

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

#[reducer]
pub fn shoot_player(ctx: &ReducerContext) {
    // Scenario: an FPS game where players can shoot each others, this reducer is called when a player shoots.
    // When a player logged in we created a rigid body for them and a raycast that will be used to detect hits.
    // Players and racyast position update are outside the scope of this example but we can imagine
    // that the raycast's origin is set to the player's position every frame

    let player = ctx.db.players().id().find(ctx.sender).unwrap();
    let raycast = ctx
        .db
        .physics_raycasts()
        .id()
        .find(player.weapon_raycast_id)
        .unwrap();

    if raycast.hits.is_empty() {
        return;
    }

    for hit in &raycast.hits {
        // Apply damages to the players that were hit. You could also apply special effets based on

        let intial_damage = 10;
        let falloff = 0.1; // Damage falloff per meter

        // The RaycastHit contains the distance from the raycast origin to the hit point, we can
        // use it to calculate the damage falloff.
        let damage = (intial_damage as f32 - (hit.distance * falloff)).max(0.0) as u32;

        // In a real game, you would get the corresponding player entity from the hit.rigid_body_id
        // and apply the damage to that player. But for this example, we will just log the hit.
        debug!(
            "Player {} shot rigid_body {} at position {} with {} damage (distance: {})",
            player.id, hit.rigid_body_id, hit.position, damage, hit.distance
        );
    }
}
