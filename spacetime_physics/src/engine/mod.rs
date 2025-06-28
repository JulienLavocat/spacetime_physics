use std::collections::{HashMap, HashSet};

use constraints::PenetrationConstraint;
use log::debug;
use log_stopwatch::LogStopwatch;
use spacetimedb::ReducerContext;
use xpbd::{integrate_bodies, recompute_velocities, solve_constraints, solve_velocities};

use crate::{
    math::{Quat, Vec3},
    physics_trigger_entities,
    tables::{PhysicsWorld, RigidBody},
    test_collision,
    trigger::PhysicsTrigger,
    PhysicsTriggerEntity, ShapeWrapper,
};

mod constraints;
mod log_stopwatch;
mod utils;
mod xpbd;

pub type KinematicBody = (u64, (Vec3, Quat));

pub fn step_world(
    ctx: &ReducerContext,
    world: &PhysicsWorld,
    kinematic_entities: impl Iterator<Item = KinematicBody>,
) {
    let sw = LogStopwatch::new(world, &format!("step_world_{}", world.id));

    let mut binding = RigidBody::all(ctx, world.id);
    let bodies = binding.as_mut_slice();

    let dt = world.time_step / world.sub_step as f32;

    sync_kinematic_bodies(kinematic_entities, bodies);

    for i in 0..world.sub_step {
        if world.debug {
            debug!("---------- substep: {} ----------", i);
        }

        // TODO: Use Broad Phase outside of the loop and narrowly-resolve them once here
        let mut penetration_constraints = detect_collisions(world, bodies);
        let penetration_constraints = penetration_constraints.as_mut_slice();

        if world.debug {
            debug!("Collisions detected: {:?}", penetration_constraints);
        }

        integrate_bodies(bodies, world, dt);

        for _ in 0..world.position_iterations {
            solve_constraints(penetration_constraints, bodies, dt);
        }

        recompute_velocities(world, bodies, dt);
        solve_velocities(world, penetration_constraints, bodies, dt);

        if world.debug {
            debug_bodies(bodies);
        }
    }

    detect_triggers(ctx, world, bodies);

    if world.debug {
        debug!("---------- End of substeps ----------");
    }

    for body in bodies {
        if world.debug {
            debug!(
                "Updating {} position: {} -> {}, velocity: {}, rotation: {}",
                body.id, body.previous_position, body.position, body.linear_velocity, body.rotation,
            );
        }
        body.update(ctx);
    }

    if world.debug {
        debug!("-------------------------------------------------------------");
    }

    sw.end();
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

fn detect_collisions(world: &PhysicsWorld, bodies: &mut [RigidBody]) -> Vec<PenetrationConstraint> {
    let mut constraints = Vec::new();

    for i in 0..bodies.len() {
        for j in (i + 1)..bodies.len() {
            let (left, right) = bodies.split_at_mut(j);
            let body_a = &mut left[i];
            let body_b = &mut right[0];
            if let Some(collision) = test_collision(body_a, body_b, world.precision) {
                if collision.distance >= 0.0 {
                    continue; // No penetration
                }

                constraints.push(PenetrationConstraint::new(body_a, body_b, collision, 0.0));
            }
        }
    }

    constraints
}

fn debug_bodies(bodies: &[RigidBody]) {
    for body in bodies {
        debug!(
            "[Body] {}: position: {}, rotation: {}, velocity: {}, angular_velocity: {}, force: {}, torque: {}",
            body.id, body.position, body.rotation, body.linear_velocity, body.angular_velocity, body.force, body.torque
        );
    }
}

fn detect_triggers(ctx: &ReducerContext, world: &PhysicsWorld, bodies: &[RigidBody]) {
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
                        id: 0,
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

        if world.debug_triggers && !trigger_entities.is_empty() {
            debug!(
                "[Trigger] {}: old_entities: {:?}, new_entities: {:?}, deleted: {:?}, added: {:?}",
                trigger.id, old_entities, entities_inside, deleted_entities, added_entities
            );
        }

        for entity in deleted_entities {
            ctx.db.physics_trigger_entities().id().delete(entity.id);
        }
        for entity in added_entities {
            entity.insert(ctx);
        }
    }
}
