use std::fmt::Display;

use parry3d::query::Ray;
use spacetimedb::ReducerContext;

use crate::{math::Vec3, Collider, RigidBodyData};

#[derive(Debug, Clone)]
pub struct RacyCastHit {
    pub distance: f32,
    pub position: Vec3,
    pub normal: Vec3,
    pub rigid_body_id: u64,
}

impl Display for RacyCastHit {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "RacyCastHit {{ distance: {}, position: {:?}, normal: {:?}, rigid_body_id: {} }}",
            self.distance, self.position, self.normal, self.rigid_body_id
        )
    }
}

pub fn raycast_all(
    ctx: &ReducerContext,
    world_id: u64,
    origin: Vec3,
    direction: Vec3,
    max_distance: f32,
    solid: bool,
) -> impl Iterator<Item = RacyCastHit> {
    let colliders = Collider::all(ctx, world_id);
    let bodies = RigidBodyData::collect(ctx, world_id, &colliders).into_iter();
    raycast_all_with_rigid_bodies(bodies, origin, direction, max_distance, solid)
}

pub fn raycast_all_with_rigid_bodies(
    entities: impl Iterator<Item = RigidBodyData>,
    origin: Vec3,
    direction: Vec3,
    max_distance: f32,
    solid: bool,
) -> impl Iterator<Item = RacyCastHit> {
    let ray = Ray::new(origin.into(), direction.into());

    entities.filter_map(move |body| {
        let isometry = (&body).into();

        body.shape()
            .cast_ray_and_get_normal(&isometry, &ray, max_distance, solid)
            .map(|intersection| RacyCastHit {
                distance: intersection.time_of_impact,
                position: ray.point_at(intersection.time_of_impact).into(),
                normal: intersection.normal.into(),
                rigid_body_id: body.id,
            })
    })
}
