use crate::{
    engine::collisions::Collision,
    tables::{PhysicsWorld, RigidBody},
};

use super::Solver;

pub(crate) struct ImpulseSolver;

impl Solver for ImpulseSolver {
    fn solve(world: &PhysicsWorld, collisions: &[Collision], bodies: &mut [RigidBody], _: f32) {}
}
