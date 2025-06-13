use log::debug;

use crate::{
    engine::collisions::Collision,
    tables::{PhysicsWorld, RigidBody},
};

use super::Solver;

pub struct PositionSolver;

impl Solver for PositionSolver {
    fn solve(_: &PhysicsWorld, collisions: &[Collision], bodies: &mut [RigidBody], _: f32) {}
}
