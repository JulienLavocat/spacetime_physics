use std::collections::HashMap;

use collision_detection::CollisionDetection;
use log::debug;
use spacetimedb::ReducerContext;
use trigger_data::TriggerData;
use xpbd::{integrate_bodies, recompute_velocities, solve_constraints, solve_velocities};

use crate::{
    math::{Quat, Vec3},
    tables::PhysicsWorld,
    Collider, RayCast,
};

mod collision_detection;
mod constraints;
mod rigid_body_data;
mod trigger_data;
mod xpbd;

pub use rigid_body_data::RigidBodyData;

pub type KinematicBody = (u64, (Vec3, Quat));

pub fn step_world(
    ctx: &ReducerContext,
    world: &PhysicsWorld,
    kinematic_entities: impl Iterator<Item = KinematicBody>,
) {
    let sw = world.stopwatch("step_world");

    let load_sw = world.stopwatch("load_data");
    let colliders = Collider::all(ctx, world.id);
    let mut triggers = TriggerData::collect(ctx, world.id, &colliders);
    let mut entities = RigidBodyData::collect(ctx, world.id, &colliders);
    let mut raycasts = RayCast::all(ctx, world.id);

    let entities = entities.as_mut_slice();
    let triggers = triggers.as_mut_slice();
    let raycasts = raycasts.as_mut_slice();
    load_sw.end();

    let dt = world.time_step / world.sub_step as f32;

    sync_kinematic_bodies(kinematic_entities, entities);

    // TODO: Include triggers in the entities list
    let mut collision_detection = CollisionDetection::new();
    collision_detection.broad_phase(world, entities, triggers, raycasts);

    if world.debug_broad_phase() {
        debug!(
            "[PhysicsWorld#{}] [BroadPhase] pairs: {:?}",
            world.id,
            collision_detection.broad_phase_pairs().len()
        );
    }

    for i in 0..world.sub_step {
        let sw = world.stopwatch(&format!("substep_{}", i));
        if world.debug_substep() {
            debug!("---------- substep: {} ----------", i);
        }

        // TODO: Ignore trigger bodies in the narrow phase
        let mut penetration_constraints =
            collision_detection.narrow_phase_constraints(world, entities);
        let penetration_constraints = penetration_constraints.as_mut_slice();

        if world.debug_substep() {
            debug!("Collisions detected: {:?}", penetration_constraints);
        }

        integrate_bodies(entities, world, dt);

        for _ in 0..world.position_iterations {
            solve_constraints(world, penetration_constraints, entities, dt);
        }

        recompute_velocities(world, entities, dt);
        solve_velocities(world, penetration_constraints, entities, dt);

        if world.debug {
            debug_bodies(entities);
        }

        sw.end();
    }

    collision_detection.narrow_phase_triggers(ctx, world, entities, triggers);
    collision_detection.narrow_phase_raycast(ctx, world, entities, raycasts);

    if world.debug {
        debug!("---------- End of substeps ----------");
    }

    let update_sw = world.stopwatch("update_bodies");
    for entity in entities {
        if world.debug {
            debug!(
                "Updating {} position: {} -> {}, velocity: {}, rotation: {}",
                entity.id,
                entity.previous_position(),
                entity.position(),
                entity.linear_velocity(),
                entity.rotation(),
            );
        }
        entity.update(ctx);
    }
    update_sw.end();

    if world.debug {
        debug!("-------------------------------------------------------------");
    }

    sw.end();
}

fn debug_bodies(bodies: &[RigidBodyData]) {
    for body in bodies {
        debug!(
            "[Body] {}: position: {}, rotation: {}, velocity: {}, angular_velocity: {}, force: {}, torque: {}",
            body.id, body.position(), body.rotation(), body.linear_velocity(), body.angular_velocity(), body.force(), body.torque()
        );
    }
}

fn sync_kinematic_bodies(
    kinematic_entities: impl Iterator<Item = KinematicBody>,
    entities: &mut [RigidBodyData],
) {
    let kine: HashMap<u64, (Vec3, Quat)> = kinematic_entities
        .map(|c| (c.0, (c.1 .0, c.1 .1)))
        .collect();

    for entity in entities {
        if !entity.is_kinematic() {
            continue;
        }

        let (position, rotation) = match kine.get(&entity.id) {
            Some((pos, rot)) => (pos, rot),
            None => continue, // No kinematic data for this body
        };

        entity.set_rotation(*rotation);
        entity.set_position(*position);
    }
}
