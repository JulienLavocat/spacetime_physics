use std::collections::HashMap;

use parry3d::na::Isometry3;
use spacetimedb::ReducerContext;

use crate::{
    math::{Mat3, Quat, Vec3},
    Collider, ColliderId, PhysicsWorldId, RigidBody, RigidBodyProperties, ShapeWrapper,
};

/// Represents a rigid body in the physics engine, containing its properties and state.
/// This struct is used as an abstraction layer to the RigidBody storage in the database,
/// it's also used to store properties exclusive to the physics engine algorithms,
/// that does not need to be stored in the database.
pub struct RigidBodyData {
    pub id: u64,
    rb: RigidBody,
    mass: f32,
    inv_mass: f32,
    friction_static_coefficient: f32,
    friction_dynamic_coefficient: f32,
    restitution_coefficient: f32,
    shape: ShapeWrapper,
    inertia_tensor: Mat3,
    inv_inertia_tensor: Mat3,
    pre_solve_linear_velocity: Vec3,
    pre_solve_angular_velocity: Vec3,
    previous_position: Vec3,
    previous_rotation: Quat,
    is_dirty: bool,
}

impl RigidBodyData {
    pub fn new(
        rigid_body: RigidBody,
        rb_properties: &RigidBodyProperties,
        collider: &Collider,
    ) -> Self {
        let inertia_tensor = collider.inertia_tensor(rb_properties.mass);
        Self {
            id: rigid_body.id,
            rb: rigid_body,
            shape: ShapeWrapper::from(collider),
            mass: rb_properties.mass,
            inv_mass: rb_properties.inv_mass,
            friction_static_coefficient: rb_properties.friction_static_coefficient,
            friction_dynamic_coefficient: rb_properties.friction_dynamic_coefficient,
            restitution_coefficient: rb_properties.restitution_coefficient,
            inertia_tensor,
            inv_inertia_tensor: inertia_tensor.inverse(),
            pre_solve_linear_velocity: rigid_body.linear_velocity,
            pre_solve_angular_velocity: rigid_body.angular_velocity,
            previous_position: rigid_body.position,
            previous_rotation: rigid_body.rotation,
            is_dirty: false,
        }
    }

    pub fn collect(
        ctx: &ReducerContext,
        world_id: PhysicsWorldId,
        colliders: &HashMap<ColliderId, Collider>,
    ) -> Vec<Self> {
        let rb_properties = RigidBodyProperties::all(ctx, world_id)
            .map(|props| (props.id, props))
            .collect::<HashMap<_, _>>();

        let mut entities: Vec<_> = RigidBody::all(ctx, world_id)
            .map(move |rb| {
                RigidBodyData::new(
                    rb,
                    rb_properties.get(&rb.properties_id).unwrap(),
                    colliders.get(&rb.collider_id).unwrap(),
                )
            })
            .collect();

        entities.sort_by_key(|e| e.id);

        entities
    }

    pub fn effective_mass(&self) -> Vec3 {
        Vec3::splat(self.mass)
    }

    pub fn effective_inverse_mass(&self) -> Vec3 {
        // TODO: Take into account locked axes
        Vec3::splat(self.inv_mass)
    }

    pub fn effective_inverse_inertia(&self) -> Mat3 {
        // TODO: Take into account locked axes
        let r = self.rb.rotation.to_mat3();
        r * self.inv_inertia_tensor * r.transpose()
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

    pub fn previous_position(&self) -> Vec3 {
        self.previous_position
    }

    pub fn previous_rotation(&self) -> Quat {
        self.previous_rotation
    }

    pub fn position(&self) -> Vec3 {
        self.rb.position
    }

    pub fn rotation(&self) -> Quat {
        self.rb.rotation
    }

    pub fn is_dynamic(&self) -> bool {
        self.rb.is_dynamic()
    }

    pub fn is_kinematic(&self) -> bool {
        self.rb.is_kinematic()
    }

    pub fn inv_mass(&self) -> f32 {
        self.inv_mass
    }

    pub fn shape(&self) -> &ShapeWrapper {
        &self.shape
    }

    pub fn linear_velocity(&self) -> Vec3 {
        self.rb.linear_velocity
    }

    pub fn angular_velocity(&self) -> Vec3 {
        self.rb.angular_velocity
    }

    pub fn force(&self) -> Vec3 {
        self.rb.force
    }

    pub fn torque(&self) -> Vec3 {
        self.rb.torque
    }

    pub fn inertia_tensor(&self) -> Mat3 {
        self.inertia_tensor
    }

    pub fn inv_inertia_tensor(&self) -> Mat3 {
        self.inv_inertia_tensor
    }

    pub fn pre_solve_linear_velocity(&self) -> Vec3 {
        self.pre_solve_linear_velocity
    }

    pub fn pre_solve_angular_velocity(&self) -> Vec3 {
        self.pre_solve_angular_velocity
    }

    pub fn set_position(&mut self, position: Vec3) {
        self.rb.position = position;
        self.is_dirty = true;
    }

    pub fn set_rotation(&mut self, rotation: Quat) {
        self.rb.rotation = rotation.normalize();
        self.is_dirty = true;
    }

    pub fn set_previous_position(&mut self, position: Vec3) {
        self.previous_position = position;
    }

    pub fn set_previous_rotation(&mut self, rotation: Quat) {
        self.previous_rotation = rotation;
    }

    pub fn set_linear_velocity(&mut self, velocity: Vec3) {
        self.rb.linear_velocity = velocity;
    }

    pub fn set_angular_velocity(&mut self, velocity: Vec3) {
        self.rb.angular_velocity = velocity;
        self.is_dirty = true;
    }

    pub fn set_force(&mut self, force: Vec3) {
        self.rb.force = force;
        self.is_dirty = true;
    }

    pub fn set_torque(&mut self, torque: Vec3) {
        self.rb.torque = torque;
        self.is_dirty = true;
    }

    pub fn set_pre_solve_linear_velocity(&mut self, velocity: Vec3) {
        self.pre_solve_linear_velocity = velocity;
    }

    pub fn set_pre_solve_angular_velocity(&mut self, velocity: Vec3) {
        self.pre_solve_angular_velocity = velocity;
    }

    pub fn update(&self, ctx: &ReducerContext) {
        self.rb.update(ctx);
    }
}

impl From<&RigidBodyData> for Isometry3<f32> {
    fn from(value: &RigidBodyData) -> Self {
        value.rb.into()
    }
}
