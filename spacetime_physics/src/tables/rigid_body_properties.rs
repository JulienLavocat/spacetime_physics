use std::fmt::Display;

use bon::{builder, Builder};
use spacetimedb::{table, ReducerContext, Table};

use crate::math::Vec3;

#[table(name = physics_rigid_body_properties, public)]
#[derive(Builder, Debug, Clone, PartialEq)]
pub struct RigidBodyProperties {
    #[primary_key]
    #[auto_inc]
    #[builder(default = 0)]
    pub id: u64,
    #[index(btree)]
    #[builder(default = 1)]
    pub world_id: u64,
    #[builder(default = 0.5)]
    pub friction_static_coefficient: f32,
    #[builder(default = 0.5)]
    pub friction_dynamic_coefficient: f32,
    #[builder(default = 0.0)]
    pub restitution_coefficient: f32,
    #[builder(default = 1.0)]
    pub mass: f32,
    #[builder(skip = if mass > 0.0 { 1.0 / mass } else { 0.0 })]
    pub inv_mass: f32,
}

impl RigidBodyProperties {
    pub fn insert(self, ctx: &ReducerContext) -> Self {
        ctx.db.physics_rigid_body_properties().insert(self)
    }

    pub fn find(ctx: &ReducerContext, id: u64) -> Option<Self> {
        ctx.db.physics_rigid_body_properties().id().find(id)
    }

    pub fn all(ctx: &ReducerContext, world_id: u64) -> impl Iterator<Item = Self> {
        ctx.db
            .physics_rigid_body_properties()
            .world_id()
            .filter(world_id)
    }

    pub fn update(self, ctx: &ReducerContext) -> Self {
        ctx.db.physics_rigid_body_properties().id().update(self)
    }

    pub fn delete(self, ctx: &ReducerContext) {
        ctx.db.physics_rigid_body_properties().id().delete(self.id);
    }

    pub fn delte_by_id(ctx: &ReducerContext, id: u64) {
        ctx.db.physics_rigid_body_properties().id().delete(id);
    }

    pub fn combine_static_friction(&self, other: &Self) -> f32 {
        (self.friction_static_coefficient + other.friction_static_coefficient) / 2.0
    }

    pub fn combine_dynamic_friction(&self, other: &Self) -> f32 {
        (self.friction_dynamic_coefficient + other.friction_dynamic_coefficient) / 2.0
    }

    pub fn combine_restitution(&self, other: &Self) -> f32 {
        (self.restitution_coefficient + other.restitution_coefficient) / 2.0
    }

    pub fn effective_inverse_mass(&self) -> Vec3 {
        Vec3::splat(self.inv_mass)
    }
}

impl Display for RigidBodyProperties {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "RigidBodyProperties {{ id: {}, world_id: {}, friction_static_coefficient: {}, friction_dynamic_coefficient: {}, restitution_coefficient: {}, mass: {}, inv_mass: {} }}",
            self.id,
            self.world_id,
            self.friction_static_coefficient,
            self.friction_dynamic_coefficient,
            self.restitution_coefficient,
            self.mass,
            self.inv_mass
        )
    }
}
