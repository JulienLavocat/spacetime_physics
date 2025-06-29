use std::collections::{HashMap, HashSet};

use collision_detection::CollisionDetection;
use log::{debug, trace};
use spacetimedb::{ReducerContext, Table};
use xpbd::{integrate_bodies, recompute_velocities, solve_constraints, solve_velocities};

use crate::{
    math::{Quat, Vec3},
    physics_trigger_entities,
    tables::{PhysicsWorld, RigidBody},
    trigger::PhysicsTrigger,
    PhysicsTriggerEntity, ShapeWrapper,
};

mod collision_detection;
mod constraints;
mod xpbd;

pub type KinematicBody = (u64, (Vec3, Quat));

pub fn step_world(
    ctx: &ReducerContext,
    world: &PhysicsWorld,
    kinematic_entities: impl Iterator<Item = KinematicBody>,
) {
    let sw = world.stopwatch("step_world");

    let mut binding = RigidBody::all(ctx, world.id);
    let bodies = binding.as_mut_slice();

    let dt = world.time_step / world.sub_step as f32;

    sync_kinematic_bodies(kinematic_entities, bodies);

    let mut collision_detection = CollisionDetection::new();
    collision_detection.broad_phase(world, bodies);

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

        // TODO: Use Broad Phase outside of the loop and narrowly-resolve them once here
        let mut penetration_constraints =
            collision_detection.narrow_phase_constraints(world, bodies);
        let penetration_constraints = penetration_constraints.as_mut_slice();

        if world.debug_substep() {
            debug!("Collisions detected: {:?}", penetration_constraints);
        }

        integrate_bodies(bodies, world, dt);

        for _ in 0..world.position_iterations {
            solve_constraints(world, penetration_constraints, bodies, dt);
        }

        recompute_velocities(world, bodies, dt);
        solve_velocities(world, penetration_constraints, bodies, dt);

        if world.debug {
            debug_bodies(bodies);
        }

        sw.end();
    }

    detect_triggers(ctx, world, bodies);

    if world.debug {
        debug!("---------- End of substeps ----------");
    }

    let update_sw = world.stopwatch("update_bodies");
    for body in bodies {
        if !body.is_dirty() {
            continue; // Skip bodies that are not dirty
        }

        if world.debug {
            debug!(
                "Updating {} position: {} -> {}, velocity: {}, rotation: {}",
                body.id, body.previous_position, body.position, body.linear_velocity, body.rotation,
            );
        }
        body.update(ctx);
    }
    update_sw.end();

    if world.debug {
        debug!("-------------------------------------------------------------");
    }

    sw.end();
}

fn debug_bodies(bodies: &[RigidBody]) {
    for body in bodies {
        debug!(
            "[Body] {}: position: {}, rotation: {}, velocity: {}, angular_velocity: {}, force: {}, torque: {}",
            body.id, body.position, body.rotation, body.linear_velocity, body.angular_velocity, body.force, body.torque
        );
    }
}

fn sync_kinematic_bodies(
    kinematic_entities: impl Iterator<Item = KinematicBody>,
    bodies: &mut [RigidBody],
) {
    let kine: HashMap<u64, (Vec3, Quat)> = kinematic_entities
        .map(|c| (c.0, (c.1 .0, c.1 .1)))
        .collect();

    for body in bodies {
        if !body.is_kinematic() {
            continue;
        }

        let (position, rotation) = match kine.get(&body.id) {
            Some((pos, rot)) => (pos, rot),
            None => continue, // No kinematic data for this body
        };

        body.rotation = *rotation;
        body.position = *position;
    }
}

fn detect_triggers(ctx: &ReducerContext, world: &PhysicsWorld, bodies: &[RigidBody]) {
    let sw = world.stopwatch("detect_triggers");
    let triggers = PhysicsTrigger::all(world.id, ctx);
    let triggers = triggers.as_slice();
    let trigger_entities = PhysicsTriggerEntity::all(ctx, world.id);

    // TODO: Add a toggle for high-frequency trigger detection that would run every substep
    // TODO: Use broad phase to reduce the number of checks
    for trigger in triggers {
        let mut entities_inside = HashSet::new();

        for body in bodies {
            let trigger_collider = ShapeWrapper::from(trigger.collider);
            let body_collider = ShapeWrapper::from(body.collider);

            if let Ok(is_intersecting) =
                trigger_collider.intersects(&trigger.into(), &body.into(), &body_collider)
            {
                if is_intersecting {
                    entities_inside.insert(PhysicsTriggerEntity {
                        trigger_id: trigger.id,
                        world_id: world.id,
                        entity_id: body.id,
                    });
                }
            }
        }

        let old_entities = trigger_entities
            .get(&trigger.id)
            .cloned()
            .unwrap_or_default();

        let deleted_entities = old_entities.difference(&entities_inside);
        let added_entities = entities_inside.difference(&old_entities);

        if world.debug_triggers() && !trigger_entities.is_empty() {
            debug!(
                "[PhysicsWorld#{}] [Trigger] {}: previous_entities: {:?}, current_entities: {:?}, deleted: {:?}, added: {:?}",
                world.id, trigger.id, old_entities, entities_inside, deleted_entities, added_entities
            );
        }

        for entity in deleted_entities {
            ctx.db.physics_trigger_entities().delete(*entity);
        }

        for entity in added_entities {
            entity.insert(ctx);
        }
    }
    sw.end();
}
