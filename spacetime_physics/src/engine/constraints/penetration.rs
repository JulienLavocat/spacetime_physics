use std::fmt::Display;

use log::debug;

use crate::{math::Vec3, utils::get_bodies_mut, CollisionPoint, PhysicsWorld, RigidBodyData};

use super::{position::PositionConstraint, Constraint};

#[derive(Debug, Clone)]
pub struct PenetrationConstraint {
    pub a: u64,
    pub b: u64,
    pub world_a: Vec3,
    pub world_b: Vec3,
    pub local_a: Vec3,
    pub local_b: Vec3,
    pub normal: Vec3,
    pub penetration_depth: f32,
    pub compliance: f32,
    pub normal_lagrange: f32,
    pub normal_force: Vec3,
    pub tangential_lagrange: f32,
    pub static_friction_force: Vec3,
    pub tangent_lagrange: f32,
}

impl PenetrationConstraint {
    pub fn new(
        a: &RigidBodyData,
        b: &RigidBodyData,
        point: CollisionPoint,
        compliance: f32,
    ) -> Self {
        Self {
            a: a.id,
            b: b.id,
            world_a: point.world_a,
            world_b: point.world_b,
            local_a: point.local_a,
            local_b: point.local_b,
            normal: point.normal,
            penetration_depth: point.distance,
            compliance,
            normal_lagrange: 0.0,
            normal_force: Vec3::ZERO,
            tangential_lagrange: 0.0,
            static_friction_force: Vec3::ZERO,
            tangent_lagrange: 0.0,
        }
    }

    fn solve_contact(
        &mut self,
        world: &PhysicsWorld,
        body_a: &mut RigidBodyData,
        body_b: &mut RigidBodyData,
        dt: f32,
    ) {
        // Shorter aliases for readability
        let penetraion = self.penetration_depth;
        let normal = self.normal;
        let compliance = self.compliance;
        let lagrange = self.normal_lagrange;

        let ra = body_a.rotation().rotate(self.local_a);
        let rb = body_b.rotation().rotate(self.local_b);

        if penetraion >= 0.0 {
            return;
        }

        let wa = self.compute_generalized_inverse_mass(body_a, &ra, &normal);
        let wb = self.compute_generalized_inverse_mass(body_b, &rb, &normal);

        let gradients = [normal, -normal];
        let w = [wa, wb];

        let delta_lagrange =
            self.compute_lagrange_update(lagrange, penetraion, &gradients, &w, compliance, dt);
        self.normal_lagrange += delta_lagrange;
        self.normal_force = self.normal_lagrange * normal / dt.powi(2);

        if let Some((body_a_prev_pos, body_a_pos, body_b_prev_pos, body_b_pos, p)) =
            self.apply_position_correction(body_a, body_b, delta_lagrange, &normal, &ra, &rb)
        {
            if world.debug_constraints() {
                debug!(
                "[PenetrationConstraint] contact: a: {}, a_pos: {} -> {}, b: {}, b_pos: {} -> {}, p: {},, compliance: {}, normal_lagrange: {}, tangential_lagrange: {}",
                self.a, body_a_prev_pos, body_a_pos,
                self.b, body_b_prev_pos, body_b_pos,
                p,
                self.compliance, self.normal_lagrange, self.tangential_lagrange
            );
            }
        }
    }

    fn solve_friction(
        &mut self,
        world: &PhysicsWorld,
        body1: &mut RigidBodyData,
        body2: &mut RigidBodyData,
        dt: f32,
    ) {
        // Shorter aliases
        let penetration = self.penetration_depth;
        let normal = self.normal;
        let compliance = self.compliance;
        let lagrange = self.tangential_lagrange;
        let r1 = self.world_a;
        let r2 = self.world_b;

        // Compute contact positions at the current state and before substep integration
        let p1 = body1.position() + body1.rotation().rotate(self.local_a);
        let p2 = body2.position() + body2.rotation().rotate(self.local_b);
        let prev_p1 = body1.previous_position() + body1.previous_rotation().rotate(self.local_a);
        let prev_p2 = body2.previous_position() + body2.previous_rotation().rotate(self.local_b);

        // Compute relative motion of the contact points and get the tangential component
        let delta_p = (p1 - prev_p1) - (p2 - prev_p2);
        let delta_p_tangent = delta_p - delta_p.dot(normal) * normal;

        // Compute magnitude of relative tangential movement and get normalized tangent vector
        let sliding_len = delta_p_tangent.length();
        if sliding_len <= f32::EPSILON {
            return;
        }
        let tangent = delta_p_tangent / sliding_len;

        // Compute generalized inverse masses
        let w1 = self.compute_generalized_inverse_mass(body1, &r1, &tangent);
        let w2 = self.compute_generalized_inverse_mass(body2, &r2, &tangent);

        // Constraint gradients and inverse masses
        let gradients = [tangent, -tangent];
        let w = [w1, w2];

        // Compute combined friction coefficients
        let static_coefficient = body1.combine_static_friction(body2);

        // Apply static friction if |delta_x_perp| < mu_s * d
        if sliding_len < static_coefficient * penetration {
            // Compute Lagrange multiplier update for static friction
            let delta_lagrange =
                self.compute_lagrange_update(lagrange, sliding_len, &gradients, &w, compliance, dt);
            self.tangent_lagrange += delta_lagrange;

            // Apply positional correction to handle static friction
            self.apply_position_correction(body1, body2, delta_lagrange, &tangent, &r1, &r2);

            // Update static friction force using the equation f = lambda * n / h^2
            self.static_friction_force = self.tangent_lagrange * tangent / dt.powi(2);

            if world.debug_constraints() {
                debug!(
                    "[PenetrationConstraint] static friction: a: {}, b: {}, tangent_lagrange: {}, static_friction_force: {}",
                    self.a, self.b, self.tangent_lagrange, self.static_friction_force
                );
            }
        }
    }
}

impl Constraint for PenetrationConstraint {
    fn solve(&mut self, world: &PhysicsWorld, bodies: &mut [RigidBodyData], dt: f32) {
        let (body_a, body_b) = get_bodies_mut(self.a, self.b, bodies);
        self.solve_contact(world, body_a, body_b, dt);
        self.solve_friction(world, body_a, body_b, dt);
    }
}

impl PositionConstraint for PenetrationConstraint {}

impl Display for PenetrationConstraint {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "PenetrationConstraint(a: {}, b: {}, contact_point_a: {}, contact_point_b: {}, normal: {}, penetration_depth: {}, compliance: {}, normal_lagrange: {}, tangential_lagrange: {})",
            self.a,
            self.b,
            self.world_a,
            self.world_b,
            self.normal,
            self.penetration_depth,
            self.compliance,
            self.normal_lagrange,
            self.tangential_lagrange
        )
    }
}
