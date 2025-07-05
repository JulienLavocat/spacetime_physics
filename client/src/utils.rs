use bevy::color::palettes::css::{GREEN_YELLOW, RED};
use bevy::{color::palettes::css::GRAY, prelude::*};

use crate::module_bindings::{Collider, ColliderType};

pub fn collider_to_mesh(
    collider: &Collider,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) -> (Handle<Mesh>, Handle<StandardMaterial>) {
    match collider.collider_type {
        ColliderType::Plane => {
            let material = materials.add(StandardMaterial {
                base_color: GRAY.into(),
                ..default()
            });
            let mesh = meshes.add(Mesh::from(Plane3d {
                normal: Dir3::from_xyz(collider.normal.x, collider.normal.y, collider.normal.z)
                    .unwrap(),
                half_size: Vec2::new(100.0, 100.0),
            }));

            (mesh, material)
        }
        ColliderType::Sphere => {
            let material = materials.add(StandardMaterial {
                base_color: RED.into(),
                ..default()
            });
            let mesh = meshes.add(Sphere::new(collider.radius));

            (mesh, material)
        }
        ColliderType::Cuboid => {
            let material = materials.add(StandardMaterial {
                base_color: GREEN_YELLOW.into(),
                ..default()
            });
            let mesh = meshes.add(Cuboid::new(
                collider.size.x,
                collider.size.y,
                collider.size.z,
            ));
            debug!("Cuboid mesh: {:?}, {:?}", mesh, collider);

            (mesh, material)
        }
        ColliderType::Capsule => {
            let material = materials.add(StandardMaterial {
                base_color: RED.into(),
                ..default()
            });
            let mesh = meshes.add(Capsule3d::new(collider.radius, collider.height));

            (mesh, material)
        }
        ColliderType::Cylinder => {
            let material = materials.add(StandardMaterial {
                base_color: RED.into(),
                ..default()
            });
            let mesh = meshes.add(Cylinder::new(collider.radius, collider.height));

            (mesh, material)
        }
        ColliderType::Triangle => {
            let material = materials.add(StandardMaterial {
                base_color: RED.into(),
                ..default()
            });
            let mesh = meshes.add(Triangle3d::new(
                Vec3::new(collider.point_a.x, collider.point_a.y, collider.point_a.z),
                Vec3::new(collider.point_b.x, collider.point_b.y, collider.point_b.z),
                Vec3::new(collider.point_c.x, collider.point_c.y, collider.point_c.z),
            ));

            (mesh, material)
        }
        ColliderType::Cone => {
            let material = materials.add(StandardMaterial {
                base_color: RED.into(),
                ..default()
            });
            let mesh = meshes.add(Cone::new(collider.radius, collider.height));

            (mesh, material)
        }
    }
}
