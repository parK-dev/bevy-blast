use bevy::{math::vec3, prelude::*};
use bevy_rapier3d::prelude::*;
use rand::Rng;
use std::time::Duration;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_systems(
            Startup,
            (
                setup_rapier,
                setup_cubes,
                setup_camera,
                setup_ground,
                setup_blast,
            ),
        )
        .add_systems(Update, blast)
        .run();
}

fn setup_cubes(
    mut commands: Commands,
    mut mats: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    commands.insert_resource(Cubes {
        mat: mats.add(StandardMaterial {
            base_color: Color::Rgba {
                red: 255.,
                green: 0.,
                blue: 0.,
                alpha: 1.,
            },
            ..default()
        }),
        mesh: meshes.add(Mesh::from(shape::Cube { size: 5. })),
    })
}

fn setup_camera(mut commands: Commands) {
    // Setup a Camera
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(-50.0, 25.0, 80.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..Default::default()
    });
}

fn setup_ground(mut commands: Commands) {
    commands.spawn((
        TransformBundle::from(Transform::from_xyz(0.0, 0.0, 0.0)),
        Collider::cuboid(30.0, 0., 30.0),
        Ground,
    ));
}
fn setup_rapier(mut rapier_config: ResMut<RapierConfiguration>) {
    rapier_config.gravity = Vec3::new(0.0, -9.81, 0.0);
}

fn setup_blast(mut commands: Commands) {
    // Setup a Blasting grid
    let b = Blast {
        grid_size: 20,
        spacing: 2.5,
        cube_hx: 0.5,
        cube_hy: 0.5,
        cube_hz: 0.5,
        restitution_coefficient: 0.7,
        blast_pattern: BlastPattern::RowByRow,
    };

    for i in 0..b.grid_size {
        for j in 0..b.grid_size {
            let x = i as f32 * b.spacing - (b.grid_size as f32 / 2.0) * b.spacing;
            let z = j as f32 * b.spacing - (b.grid_size as f32 / 2.0) * b.spacing;
            let y = 0.1;

            commands.spawn(BlastBundle {
                rigid_body: RigidBody::Dynamic,
                collider: Collider::cuboid(b.cube_hx, b.cube_hy, b.cube_hz),
                restitution: Restitution::coefficient(b.restitution_coefficient),
                transform_bundle: TransformBundle::from_transform(Transform {
                    translation: vec3(x, y, z),
                    ..default()
                }),
                hole: Hole,
                detonator: Detonator {
                    timer: Timer::new(
                        b.blast_pattern.get_timer_duration(i, j, b.grid_size),
                        TimerMode::Once,
                    ),
                },
                blasted: Blasted(false),
                external_impulse: ExternalImpulse::default(),
            });
        }
    }

    commands.spawn(b);

    // Add the detonation state
    commands.insert_resource(DetonationState::default());
}

fn blast(
    mut state: ResMut<DetonationState>,
    time: Res<Time>,
    keyboard: Res<Input<KeyCode>>,
    mut query: Query<(&mut Detonator, &mut ExternalImpulse, &mut Blasted), With<Hole>>,
) {
    // Check for the D key press to start the detonation.
    if keyboard.just_pressed(KeyCode::D) {
        state.has_started = true;
    }

    // If the detonation has started, tick the timers.
    if state.has_started {
        for (mut detonator, mut ext_impulse, mut blasted) in query.iter_mut() {
            // Tick the timers by the actual time elapsed since the last frame.
            detonator.timer.tick(time.delta());

            // If the timer finished and the force hasn't been applied yet, apply the force.
            if detonator.timer.finished() && !blasted.0 {
                ext_impulse.impulse = Vec3::new(-2., 10., 0.);
                ext_impulse.torque_impulse = Vec3::new(0., 1., 0.);

                // TODO: randomized misfires?
                blasted.0 = true;
            }
        }
    }
}

#[derive(Resource, Default)]
pub struct Cubes {
    mat: Handle<StandardMaterial>,
    mesh: Handle<Mesh>,
}

#[derive(Component)]
pub struct Hole;

#[derive(Component)]
pub struct Ground;

#[derive(Component)]
pub struct Detonator {
    timer: Timer,
}

#[derive(Resource, Default)]
struct DetonationState {
    has_started: bool,
}

#[derive(Component, Default)]
struct Blasted(bool);

#[derive(Component, Default)]
struct Blast {
    grid_size: usize,
    spacing: f32, // Spacing between cubes
    cube_hx: f32,
    cube_hy: f32,
    cube_hz: f32,
    restitution_coefficient: f32,
    blast_pattern: BlastPattern,
}

#[derive(Default, Debug)]
enum BlastPattern {
    #[default]
    RowByRow,
    LeftToRight,
    RightToLeft,
    Zigzag,
    SpiralInwards,
    Random,
}

impl BlastPattern {
    fn get_timer_duration(&self, i: usize, j: usize, grid_size: usize) -> Duration {
        match self {
            BlastPattern::RowByRow => {
                let delay_per_row = 500; // milliseconds
                Duration::from_millis((i * delay_per_row) as u64)
            }
            BlastPattern::LeftToRight => Duration::from_millis(100 * (i * grid_size + j) as u64),
            BlastPattern::RightToLeft => {
                Duration::from_millis(100 * (i * grid_size + grid_size - 1 - j) as u64)
            }
            BlastPattern::Zigzag => {
                if i % 2 == 0 {
                    Duration::from_millis(100 * (i * grid_size + j) as u64)
                } else {
                    Duration::from_millis(100 * (i * grid_size + grid_size - 1 - j) as u64)
                }
            }
            BlastPattern::SpiralInwards => {
                let layer = std::cmp::min(
                    std::cmp::min(i, j),
                    std::cmp::min(grid_size - 1 - i, grid_size - 1 - j),
                );
                let pos_in_layer = (i + j + grid_size - 1) % (grid_size - 1);
                Duration::from_millis(100 * (4 * layer * (grid_size - 1) + pos_in_layer) as u64)
            }
            BlastPattern::Random => {
                let mut rng = rand::thread_rng();
                Duration::from_millis(rng.gen_range(0..100 * grid_size * grid_size) as u64)
            }
        }
    }
}

#[derive(Bundle)]
struct BlastBundle {
    rigid_body: RigidBody,
    collider: Collider,
    restitution: Restitution,
    transform_bundle: TransformBundle,
    hole: Hole,
    detonator: Detonator,
    blasted: Blasted,
    external_impulse: ExternalImpulse,
}
