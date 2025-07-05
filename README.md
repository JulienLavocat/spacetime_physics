# Spacetime Physics — Server‑authoritative physics engine for SpacetimeDB projects

> **Spacetime Physics** is a real-time, server‑authoritative physics engine
> built for [SpacetimeDB](https://github.com/clockworklabs/spacetimedb).  
> It’s built around an XPBD solver and designed for multiplayer simulations
> that are fast, deterministic, and scalable.

Discuss the project on [Discord](https://discord.com/channels/1037340874172014652/1391058999503229031)

<p align="center">
    <a href="https://discord.com/channels/1037340874172014652/1391058999503229031" target="_blank">
        <img alt="Discord" src="https://img.shields.io/discord/1037340874172014652?style=for-the-badge&logo=discord">
    </a>
    <a href="https://crates.io/crates/spacetime_physics" target="_blank">
        <img alt="Crates.io Version" src="https://img.shields.io/crates/v/spacetime_physics?style=for-the-badge&link=https%3A%2F%2Fcrates.io%2Fcrates%2Fspacetime_physics">
    </a>
    <a href="https://crates.io/crates/spacetime_physics" target="_blank">
        <img alt="Crates.io License" src="https://img.shields.io/crates/l/spacetime_physics?style=for-the-badge">
    </a>
    <a href="https://crates.io/crates/spacetime_physics" target="_blank">
        <img alt="Crates.io Documentation" src="https://img.shields.io/docsrs/spacetime_physics?style=for-the-badge">
    </a>
    <a href="https://crates.io/crates/spacetime_physics" target="_blank">
        <img alt="Crates.io Downloads" src="https://img.shields.io/crates/d/spacetime_physics?style=for-the-badge">
    </a>
    <a href="https://github.com/JulienLavocat/spacetime_physics" target="_blank">
        <img alt="GitHub last commit" src="https://img.shields.io/github/last-commit/JulienLavocat/spacetime_physics?style=for-the-badge">
    </a>
</p>

---

## ✨ Key Features

| Capability               | Details                                                                                                                   |
| ------------------------ | ------------------------------------------------------------------------------------------------------------------------- |
| **Server‑authoritative** | All simulation is done on the server using reducers, ensuring a consistent and authoritative state.                       |
| **Rigid bodies**         | `Dynamic`, `Static`, and `Kinematic` body types with fully configurable mass, damping, friction & restitution, etc.       |
| **Colliders**            | `Sphere`, `Cuboid`, `Plane`, `Capsule`, `Cylinder`, `Cone`, `Triangle`.                                                   |
| **Triggers**             | Collision volumes that fire events when entities enter or exit them, useful for zones, pickups, area of effects, etc.     |
| **Ray‑casts**            | Continuous (persistent) or instantaneous ray‑casts — perfect for hitscan weapons, line‑of‑sight checks and AI perception. |
| **Multi‑world**          | Simulate any number of isolated physics worlds; each world can run at its own tick‑rate, gravity, etc.                    |
| **XPBD**                 | Constraint‑based solver with predictable integration produced at a fixed tick‑rate (default **60 Hz**).                   |

---

## Getting Started

> [!CAUTION]
> This is an early release of the Spacetime Physics engine, and it is not yet
> feature‑complete. Expect breaking changes in future releases.
>
> Dynamic bodies are currently unstable and not production-ready, contributions
> and bug reports are welcome to help stabilize XPBD!
> `Raycasts`, `Triggers`, `Kinematic`and `Static` bodies are stable.
>
> Please open an issue if you encounter any problems or have feature requests.

### Prerequisites

Run `cargo add spacetime_physics` to add the dependency to your project.
or add it manually to your `Cargo.toml`:

```toml
# Cargo.toml
[dependencies]
spacetime_physics = "*"
```

### Basic Usage

Please note that most of the API use the builder pattern, feel free to explore
all the available options.

#### 1. Creating a Physics World

```rust
#[reducer(init)]
pub fn example_init(ctx: &ReducerContext) {
    // Create a physics world with 60 Hz tick rate and Earth gravity
    let world = PhysicsWorld::builder()
        .ticks_per_second(60.0)
        .gravity(Vec3::new(0.0, -9.81, 0.0)) // Optional, defaults to earth gravity
        // More options: custom solver iterations, etc.
        .build()
        .insert(ctx);
}
```

#### Schedule the physics world to run at the specified tick rate

`spacetime_physics` let you in charge of how and when the world should
be stepped, allowing you to update your kinematics bodies, perform
post-processing after the physics world has been stepped, etc.

```rust

// 1. Create a new table to store the physics world ticks
#[table(name = physics_ticks, scheduled(physics_tick_world))]
pub struct PhysicsWorldTick {
    #[primary_key]
    #[auto_inc]
    pub id: u64,
    pub world_id: u64,
    pub scheduled_at: ScheduleAt,
}

// 2. Schedule the physics world to run at the specified tick rate

// 3. Implement the reducer that will be called when the physics world tick is processed
#[reducer]
pub fn physics_tick_world(ctx: &ReducerContext, tick: PhysicsWorldTick) {
    // Get the physics world by its id
    let world = ctx.db.physics_world().id().find(tick.world_id).unwrap();

    // You can have kinematic entities, which are entities that are not affected
    // by forces but can still interact with the physics world.
    // In this example player's positions are updated by the client directly, we
    // just need to synchronize their positions and rotations with the physics world.
    let kinematic_entities = ctx
        .db
        .players()
        .iter()
        .map(|c| (c.rigid_body_id, (c.position, c.rotation)));

    // Update the physics world and synchronize the kinematic entities positions
    // and rotations
    step_world(ctx, &world, kinematic_entities);

    // After the physics world has been stepped, you can perform post-processing
    // e.g. creating events for entities that entered or exited triggers,
    // raycasts hits, etc.
}
```

#### 2. Adding Rigid Bodies

Types of rigid bodies include `Dynamic`, `Static`, and `Kinematic`.

- `Dynamic` bodies are affected by forces and constraints.
- `Static` bodies never move.
- `Kinematic` bodies are driven by external inputs (e.g. player inputs).

```rust
// Start by creating a rigid body properties entity, this can be reused for
// multiple bodies (e.g. all players, all enemies)
// Most properties are optional, defaults are reasonable for most cases.
let rb_properties = RigidBodyProperties::builder()
    .mass(1.0) // mass in kg
    .restitution(0.3) // bounciness
    .build()
    .insert(ctx);

// Create a collider for the body and get it's id, e.g. a sphere with radius 1.0,
// this can also be reused for multiple bodies
let collider = Collider::sphere(world.id, 1.0).insert(ctx).id;

// Now create the rigid body itself, this can be dynamic, static or kinematic
let rigid_body = RigidBody::builder()
    .position(Vec3::new(0.0, 10.0, 0.0)) // initial position
    .collider_id(collider) // the collider we created above
    .properties_id(rb_properties.id) // the properties we created above
    .body_type(RigidBodyType::Dynamic) // can be Dynamic, Static or Kinematic
    .build()
    .insert(ctx);
```

#### 3. Adding triggers

```rust

// Start by creating a collider for the trigger, this can be a cuboid, sphere, etc.
let collider = Collider::cuboid(world.id, Vec3::new(1.0, 1.0, 1.0)).insert(ctx).id;

// Now create the trigger itself, this can be used to detect when entities enter
// or exit the trigger volume
let trigger = Trigger::builder()
    .position(Vec3::new(0.0, 5.0, 0.0)) // position of the trigger
    .size(Vec3::new(2.0, 2.0, 2.0)) // size of the trigger volume
    .collider_id(collider.id) // collider shape
    .build()
    .insert(ctx);

// Each trigger have several properties, which can be used to detect when
// entities enter or exit the trigger:
- trigger.added_entities // Entities that entered the trigger volume this tick
- trigger.removed_entities // Entities that exited the trigger volume this tick
- trigger.entities_inside // All entities currently in the trigger volume
```

#### 4. Adding RayCasts

```rust

// Unlike rigid bodies and triggers, raycasts doesn't need anything else
// like a collider or properties.
let raycast = RayCast::new(
    world.id, // The world this raycast belongs to
    Vec3::new(0.0, 10.0, 0.0), // Start position of the ray
    Vec3::new(0.0, -1.0, 0.0), // Direction of the ray (normalized)
    100.0, // Length of the ray
    false, // Is the ray solid
).insert(ctx);

// Raycast have several properties that can be used to detect hits:
- raycast.hits // Entities that were hit by the ray this tick
- raycast.added_hits // Entities that were hit by the ray this tick
- raycast.removed_hits // Entities that were no longer hit by the ray this tick
```

---

## Examples

Checkout the examples directory for complete examples of what the engine can do,
including:

- [Basic World Creation](examples/basic_world.rs)
- [FPS Shooter](examples/fps_shooter.rs)

---

## Advanced Topics

### Persistent Ray‑casts vs Instant Raycasts?

`RayCast::new(..).insert()` produces a row that is _integrated by the solver
itself_ each tick whereas `PhysicsWorld::raycast(..)` does not.

- The ray is updated every tick, using the already computed physics state
- Instant raycasts checks needs to recompute the physics state, which can be expensive
- Perisistent raycasts can detect when an entity just started or stopped being
  hit by the ray

### Multiple Worlds

Use separate worlds for independent simulations
(e.g. lobby vs. battlefield) or custom gravity zones — they never interact.

### Debugging

```rust
PhysicsWorld::builder().debug(true)
```

Enables extra logging; This is **extremly** verbose and should only be used for
debugging purposes.
Additionally, you can only enable logging of specific events like
`debug_collisions()`, `debug_raycast()`, etc.

---

## Roadmap

This roadmap is loosely ordered by priority and is subject to change as the
project evolves or as new features are requested.

- [ ] **Dynamic Bodies**: Fix bugs and improve stability.
- [ ] **Lag compensation**: Implement lag compensation.
- [ ] **Events**: Emit events when bodies enter/exit triggers, raycasts hit, etc.
- [ ] **C# API**: Provide a C# API for Unity and other C# environments.
- [ ] **Documentation**: Improve documentation and examples.

---

## Contributing

1. Clone the repo
2. Checkout the Justfile to see available commands
3. Submit a PR with your changes

---

## License

Licensed under **Apache‑2.0**.
