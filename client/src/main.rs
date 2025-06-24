use bevy::{
    color::palettes::css::{GRAY, RED},
    input::mouse::MouseMotion,
    platform::collections::HashMap,
    prelude::*,
};
use bevy_inspector_egui::{bevy_egui::EguiPlugin, quick::WorldInspectorPlugin};
use bevy_spacetimedb::{
    ReadDeleteEvent, ReadInsertEvent, ReadStdbConnectedEvent, ReadUpdateEvent, StdbConnectedEvent,
    StdbConnection, StdbConnectionErrorEvent, StdbDisconnectedEvent, StdbPlugin, tables,
};
use module_bindings::{
    Collider, DbConnection, PhysicsRigidBodiesTableAccess, PhysicsWorldTableAccess, RigidBody,
};

mod module_bindings;

#[derive(Default, Resource)]
pub struct RigidBodies {
    bodies: HashMap<u64, Entity>,
}

impl RigidBodies {
    pub fn get(&self, id: u64) -> Option<Entity> {
        self.bodies.get(&id).copied()
    }

    pub fn insert(&mut self, id: u64, entity: Entity) {
        self.bodies.insert(id, entity);
    }

    pub fn remove(&mut self, id: u64) {
        self.bodies.remove(&id);
    }
}

fn main() -> AppExit {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(
            StdbPlugin::default()
                .with_connection(|connected, disconnected, errored, _| {
                    let conn = DbConnection::builder()
                        .with_module_name("stdb-physics")
                        .with_uri("https://stdb.jlavocat.eu")
                        .on_connect(move |_, _, _| {
                            connected.send(StdbConnectedEvent {}).unwrap();
                        })
                        .on_disconnect(move |_, err| {
                            disconnected.send(StdbDisconnectedEvent { err }).unwrap();
                        })
                        .on_connect_error(move |_, err| {
                            errored.send(StdbConnectionErrorEvent { err }).unwrap();
                        });

                    let conn = conn.build().unwrap();

                    conn.run_threaded();

                    conn
                })
                .with_events(|plugin, app, db, _| {
                    tables!(physics_rigid_bodies, physics_world);
                }),
        )
        .add_plugins(EguiPlugin {
            enable_multipass_for_primary_context: true,
        })
        .add_plugins(WorldInspectorPlugin::new())
        .insert_resource(RigidBodies::default())
        .add_systems(Startup, setup)
        .add_systems(First, on_connected)
        .add_systems(PreUpdate, on_rigid_body_inserted)
        .add_systems(Update, (on_rigid_body_updated, freecam))
        .add_systems(PostUpdate, on_rigid_body_deleted)
        .run()
}

fn setup(mut commands: Commands) {
    commands.spawn((
        Name::new("Camera"),
        Camera3d::default(),
        Transform::from_xyz(0.0, 5.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        Name::new("Light"),
        PointLight {
            intensity: 1000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(5.0, 10.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

fn on_connected(mut events: ReadStdbConnectedEvent, res: Res<StdbConnection<DbConnection>>) {
    for _ in events.read() {
        info!("Connected to SpacetimeDB.");
        res.subscribe()
            .on_applied(|_| info!("Subscribed to all tables"))
            .subscribe_to_all_tables();
    }
}

fn on_rigid_body_inserted(
    mut commands: Commands,
    mut events: ReadInsertEvent<RigidBody>,
    mut rigid_bodies: ResMut<RigidBodies>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    for event in events.read() {
        debug!("RigidBody inserted: {}", event.row.id);

        let body = event.row.clone();

        let (mesh, material) = match body.collider {
            Collider::Plane(plane) => {
                // Planes are not rendered, so we return None
                let material = materials.add(StandardMaterial {
                    base_color: GRAY.into(),
                    ..default()
                });
                let mesh = meshes.add(Mesh::from(Plane3d {
                    normal: Dir3::from_xyz(plane.normal.x, plane.normal.y, plane.normal.z).unwrap(),
                    half_size: Vec2::new(100.0, 100.0),
                }));

                (Some(mesh), Some(material))
            }
            Collider::Sphere(sphere) => {
                let material = Some(materials.add(StandardMaterial {
                    base_color: RED.into(),
                    ..default()
                }));
                let mesh = Some(meshes.add(Sphere::new(sphere.radius)));

                (mesh, material)
            }
        };

        let pos = event.row.position.clone();
        let rotation = event.row.rotation.clone();

        let entity = commands
            .spawn((
                Name::from(format!("RigidBody#{}", event.row.id)),
                Transform::from_xyz(pos.x, pos.y, pos.z).with_rotation(Quat::from_xyzw(
                    rotation.x, rotation.y, rotation.z, rotation.w,
                )),
            ))
            .id();

        if let Some(mesh) = &mesh {
            commands.entity(entity).insert(Mesh3d(mesh.clone()));
        }
        if let Some(material) = &material {
            commands
                .entity(entity)
                .insert(MeshMaterial3d(material.clone()));
        }

        rigid_bodies.insert(event.row.id, entity);
    }
}

fn on_rigid_body_updated(
    mut commands: Commands,
    mut events: ReadUpdateEvent<RigidBody>,
    rigid_bodies: Res<RigidBodies>,
) {
    for event in events.read() {
        if let Some(entity) = rigid_bodies.get(event.new.id) {
            let pos = event.new.position.clone();
            commands
                .entity(entity)
                .insert(Transform::from_xyz(pos.x, pos.y, pos.z));
            println!("{}: {}", event.new.id, event.new.position.y);
        }
    }
}

fn on_rigid_body_deleted(
    mut commands: Commands,
    mut events: ReadDeleteEvent<RigidBody>,
    mut rigid_bodies: ResMut<RigidBodies>,
) {
    for event in events.read() {
        if let Some(entity) = rigid_bodies.get(event.row.id) {
            commands.entity(entity).despawn();
            rigid_bodies.remove(event.row.id);
        }
    }
}

fn freecam(
    mut camera_transform: Single<&mut Transform, With<Camera3d>>,
    keyboard_input: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mouse_motion: Res<ButtonInput<MouseButton>>,
    mut mouse_events: EventReader<MouseMotion>,
) {
    let speed = 10.0 * time.delta_secs();
    let mut translation = camera_transform.translation;

    if keyboard_input.pressed(KeyCode::KeyW) {
        translation += camera_transform.forward() * speed;
    }
    if keyboard_input.pressed(KeyCode::KeyS) {
        translation -= camera_transform.forward() * speed;
    }
    if keyboard_input.pressed(KeyCode::KeyA) {
        translation -= camera_transform.right() * speed;
    }
    if keyboard_input.pressed(KeyCode::KeyD) {
        translation += camera_transform.right() * speed;
    }
    if keyboard_input.pressed(KeyCode::Space) {
        translation += Vec3::Y * speed;
    }
    if keyboard_input.pressed(KeyCode::ShiftLeft) {
        translation -= Vec3::Y * speed;
    }

    if mouse_motion.pressed(MouseButton::Right) {
        let delta: Vec2 = mouse_events.read().map(|e| e.delta).sum();
        let yaw = -delta.x * 0.002;
        let pitch = -delta.y * 0.002;

        camera_transform.rotate(Quat::from_axis_angle(Vec3::Y, yaw));
        let right = camera_transform.right().into();
        camera_transform.rotate(Quat::from_axis_angle(right, pitch));
    }

    camera_transform.translation = translation;
}
