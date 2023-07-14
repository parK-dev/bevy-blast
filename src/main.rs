use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use rand::Rng;
use std::time::Duration;

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
struct BlastingGrid {
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

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_systems(Startup, setup)
        .add_systems(Update, blast)
        .run();
}

fn setup(mut commands: Commands, mut rapier_config: ResMut<RapierConfiguration>) {
    // Setup a Camera
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(-50.0, 25.0, 80.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..Default::default()
    });

    // Setup the Ground Collider
    let ground_size = 30.0;
    let ground_height = 0.;

    commands.spawn((
        TransformBundle::from(Transform::from_xyz(0.0, -ground_height, 0.0)),
        Collider::cuboid(ground_size, ground_height, ground_size),
        Ground,
    ));

    // Setup Gravity
    rapier_config.gravity = Vec3::new(0.0, -9.81, 0.0);

    // Setup a Blasting grid
    let b = BlastingGrid {
        grid_size: 20,
        spacing: 2.5, // Spacing between cubes
        cube_hx: 0.5,
        cube_hy: 0.5,
        cube_hz: 0.5,
        restitution_coefficient: 0.7,
        blast_pattern: BlastPattern::Random, // THE WORST METHOD IN HISTORY
    };

    for i in 0..b.grid_size {
        for j in 0..b.grid_size {
            let x = i as f32 * b.spacing - (b.grid_size as f32 / 2.0) * b.spacing;
            let z = j as f32 * b.spacing - (b.grid_size as f32 / 2.0) * b.spacing;
            let y = 0.1;

            // TODO: Turn into a bundle
            commands
                .spawn(RigidBody::Dynamic)
                .insert(Collider::cuboid(b.cube_hx, b.cube_hy, b.cube_hz))
                .insert(Restitution::coefficient(b.restitution_coefficient))
                .insert(TransformBundle::from(Transform::from_xyz(x, y, z)))
                .insert(Hole)
                .insert(Detonator {
                    timer: Timer::new(
                        b.blast_pattern.get_timer_duration(i, j, b.grid_size),
                        TimerMode::Once,
                    ),
                })
                .insert(Blasted(false))
                .insert(ExternalImpulse::default());
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
