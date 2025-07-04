use bevy::{input::mouse::MouseMotion, prelude::*};

pub struct FreeCamPlugin;

impl Plugin for FreeCamPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, freecam).add_systems(Startup, setup);
    }
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
