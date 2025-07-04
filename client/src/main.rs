use bevy::{log::LogPlugin, platform::collections::HashMap, prelude::*};
use bevy_inspector_egui::{bevy_egui::EguiPlugin, quick::WorldInspectorPlugin};
use bevy_spacetimedb::{
    ReadDeleteEvent, ReadInsertEvent, ReadStdbConnectedEvent, ReadUpdateEvent, StdbConnectedEvent,
    StdbConnection, StdbConnectionErrorEvent, StdbDisconnectedEvent, StdbPlugin, tables,
};
use freecam::FreeCamPlugin;
use module_bindings::{
    DbConnection, PhysicsCollidersTableAccess, PhysicsRigidBodiesTableAccess,
    PhysicsWorldTableAccess, RigidBody,
};
use spacetimedb_sdk::DbContext;
use utils::collider_to_mesh;

mod freecam;
mod module_bindings;
mod utils;

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
        .add_plugins(DefaultPlugins.set(LogPlugin {
            level: bevy::log::Level::DEBUG,
            filter: "info,wgpu_core=warn,wgpu_hal=error,naga=warn,client=debug".into(),
            ..Default::default()
        }))
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
        .add_plugins(FreeCamPlugin)
        .insert_resource(RigidBodies::default())
        .add_systems(First, on_connected)
        .add_systems(PreUpdate, on_rigid_body_inserted)
        .add_systems(Update, on_rigid_body_updated)
        .add_systems(PostUpdate, on_rigid_body_deleted)
        .run()
}

fn on_connected(mut events: ReadStdbConnectedEvent, res: Res<StdbConnection<DbConnection>>) {
    for _ in events.read() {
        info!("Connected to SpacetimeDB.");
        res.subscribe()
            .on_error(|_, err| panic!("subscription failed: {}", err))
            .on_applied(|ctx| {
                ctx.subscription_builder()
                    .on_error(|_, err| panic!("subscription failed: {}", err))
                    .on_applied(|_| {
                        info!("Subscription applied successfully.");
                    })
                    .subscribe("SELECT * FROM physics_rigid_bodies");
            })
            .subscribe([
                "SELECT * FROM physics_colliders",
                "SELECT * FROM physics_rigid_body_properties",
            ]);
    }
}

fn on_rigid_body_inserted(
    mut commands: Commands,
    mut events: ReadInsertEvent<RigidBody>,
    mut rigid_bodies: ResMut<RigidBodies>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    stdb: Res<StdbConnection<DbConnection>>,
) {
    for event in events.read() {
        debug!("RigidBody inserted: {}", event.row.id);

        let body = event.row.clone();

        let pos = event.row.position.clone();
        let rotation = event.row.rotation.clone();

        let collider = stdb
            .db()
            .physics_colliders()
            .id()
            .find(&body.collider_id)
            .unwrap();
        debug!("Collider: {:?}", collider);
        let (mesh, material) = collider_to_mesh(&collider, &mut meshes, &mut materials);
        let entity = commands
            .spawn((
                Name::from(format!("RigidBody#{}", event.row.id)),
                Transform::from_xyz(pos.x, pos.y, pos.z).with_rotation(Quat::from_xyzw(
                    rotation.x, rotation.y, rotation.z, rotation.w,
                )),
                Mesh3d(mesh),
                MeshMaterial3d(material.clone()),
            ))
            .id();

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
