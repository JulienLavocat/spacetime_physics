use std::collections::HashMap;

use collision_detection::CollisionDetection;
use log::debug;
use spacetimedb::ReducerContext;
use xpbd::{integrate_bodies, recompute_velocities, solve_constraints, solve_velocities};

use crate::{
    math::{Quat, Vec3},
    tables::PhysicsWorld,
};

mod collision_detection;
mod constraints;
mod physic_entity;
mod xpbd;

pub use physic_entity::RigidBodyEntity;

pub type KinematicBody = (u64, (Vec3, Quat));

pub fn step_world(
    ctx: &ReducerContext,
    world: &PhysicsWorld,
    kinematic_entities: impl Iterator<Item = KinematicBody>,
) {
    let sw = world.stopwatch("step_world");

    let mut entities: Vec<_> = RigidBodyEntity::all(ctx, world.id);
    let entities = entities.as_mut_slice();

    let dt = world.time_step / world.sub_step as f32;

    sync_kinematic_bodies(kinematic_entities, entities);

    // TODO: Include triggers in the entities list
    let mut collision_detection = CollisionDetection::new();
    collision_detection.broad_phase(world, entities);

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

    detect_triggers(ctx, world, entities);

    if world.debug {
        debug!("---------- End of substeps ----------");
    }

    let update_sw = world.stopwatch("update_bodies");
    for entity in entities {
        if world.debug {
            debug!(
                "Updating {} position: {} -> {}, velocity: {}, rotation: {}",
                entity.id,
                entity.rb.previous_position,
                entity.rb.position,
                entity.rb.linear_velocity,
                entity.rb.rotation,
            );
        }
        entity.rb.update(ctx);
    }
    update_sw.end();

    if world.debug {
        debug!("-------------------------------------------------------------");
    }

    sw.end();
}

fn debug_bodies(bodies: &[RigidBodyEntity]) {
    for body in bodies {
        debug!(
            "[Body] {}: position: {}, rotation: {}, velocity: {}, angular_velocity: {}, force: {}, torque: {}",
            body.id, body.rb.position, body.rb.rotation, body.rb.linear_velocity, body.rb.angular_velocity, body.rb.force, body.rb.torque
        );
    }
}

fn sync_kinematic_bodies(
    kinematic_entities: impl Iterator<Item = KinematicBody>,
    entities: &mut [RigidBodyEntity],
) {
    let kine: HashMap<u64, (Vec3, Quat)> = kinematic_entities
        .map(|c| (c.0, (c.1 .0, c.1 .1)))
        .collect();

    for entity in entities {
        if !entity.rb.is_kinematic() {
            continue;
        }

        let (position, rotation) = match kine.get(&entity.id) {
            Some((pos, rot)) => (pos, rot),
            None => continue, // No kinematic data for this body
        };

        entity.rb.rotation = *rotation;
        entity.rb.position = *position;
    }
}

fn detect_triggers(_ctx: &ReducerContext, _world: &PhysicsWorld, _bodies: &[RigidBodyEntity]) {
    // let sw = world.stopwatch("detect_triggers");
    // let triggers = PhysicsTrigger::all(ctx, world.id);
    // let colliders = Collider::all(ctx, world.id);
    // let triggers = triggers.as_slice();
    // let trigger_entities = PhysicsTriggerEntity::all(ctx, world.id);
    //
    // // TODO: Add a toggle for high-frequency trigger detection that would run every substep
    // // TODO: Use broad phase to reduce the number of checks
    // for trigger in triggers {
    //     let mut entities_inside = HashSet::new();
    //     let is_intersection = colliders
    //         .get(&trigger.collider_id)
    //         .unwrap().
    //     for body in bodies {
    //         if let Ok(is_intersecting) =
    //             trigger_collider.intersects(&trigger.into(), &body.into(), &body_collider)
    //         {
    //             if is_intersecting {
    //                 entities_inside.insert(PhysicsTriggerEntity {
    //                     trigger_id: trigger.id,
    //                     world_id: world.id,
    //                     entity_id: body.id,
    //                 });
    //             }
    //         }
    //     }
    //
    //     let old_entities = trigger_entities
    //         .get(&trigger.id)
    //         .cloned()
    //         .unwrap_or_default();
    //
    //     let deleted_entities = old_entities.difference(&entities_inside);
    //     let added_entities = entities_inside.difference(&old_entities);
    //
    //     if world.debug_triggers() && !trigger_entities.is_empty() {
    //         debug!(
    //             "[PhysicsWorld#{}] [Trigger] {}: previous_entities: {:?}, current_entities: {:?}, deleted: {:?}, added: {:?}",
    //             world.id, trigger.id, old_entities, entities_inside, deleted_entities, added_entities
    //         );
    //     }
    //
    //     for entity in deleted_entities {
    //         ctx.db.physics_trigger_entities().delete(*entity);
    //     }
    //
    //     for entity in added_entities {
    //         entity.insert(ctx);
    //     }
    // }
    // sw.end();
}
