use asc_ground::{WallIndex, WallSweepSettings};
use asc_physics::{
    Body, ChassisLoads, ChassisSystem, FiveLink, LinkPose, MassProperties, MountedUpright,
    TravelMechanism, TravelState, Wheel, WheelLoad, WheelState, Wrench,
};
use asc_session::{ColliderBox, WallStepSettings, step_to_wall};
use asc_test_support as f;
use glam::Mat4;
struct Directory(std::path::PathBuf);
impl Drop for Directory {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}
fn system() -> Result<ChassisSystem, Box<dyn std::error::Error>> {
    let mechanism = FiveLink::new(
        [
            [-0.3, 0.2, 0.12],
            [-0.3, 0.2, -0.12],
            [-0.3, -0.2, 0.12],
            [-0.3, -0.2, -0.12],
            [-0.3, 0., 0.15],
        ],
        [
            [0., 0.2, 0.],
            [0., 0.2, 0.],
            [0., -0.2, 0.],
            [0., -0.2, 0.],
            [0., 0., 0.15],
        ],
        LinkPose::new([0.; 3], [0., 0., 0., 1.])?,
        [0., 1., 0.],
    )?;
    let chassis = Body::new(
        MassProperties::new(300., [[220., 0., 0.], [0., 400., 0.], [0., 0., 350.]])?,
        [0.; 3],
        [0., 0., 0., 1.],
        [-2., 0., 0.],
        [0.; 3],
    )?;
    let mut upright = MountedUpright::new(
        TravelMechanism::Dwb(mechanism),
        MassProperties::new(35., [[0.4, 0., 0.], [0., 0.5, 0.], [0., 0., 0.6]])?,
        [0.; 3],
        [0.; 3],
        LinkPose::new([0.; 3], [0., 0., 0., 1.])?,
        TravelState {
            travel_m: 0.,
            rate_m_s: 0.,
        },
    )?;
    upright.attach_wheel(Wheel::new(
        [0.; 3],
        [1., 0., 0.],
        20.,
        1.2,
        0.7,
        WheelState {
            angle_rad: 0.,
            rate_rad_s: 0.,
        },
    )?)?;
    let system = ChassisSystem::new(chassis, vec![upright], [0.; 3])?;
    Ok(system)
}
fn fixture() -> (Directory, ColliderBox, WallIndex) {
    let root = f::temp_path("coupled-wall-step");
    std::fs::create_dir_all(root.join("data")).unwrap();
    for (name, text) in f::vehicle_files() {
        std::fs::write(root.join("data").join(name), text).unwrap();
    }
    for name in ["original.kn5", "collider.kn5"] {
        std::fs::write(root.join(name), f::collider_tetrahedron_file()).unwrap();
    }
    let wall = f::named(
        f::mesh(
            &[[-0.51, -10., -10.], [-0.51, 10., -10.], [-0.51, 0., 10.]],
            &[0, 1, 2],
            2,
            0,
            0,
            0.,
            0,
        ),
        "1WALL",
    );
    std::fs::write(
        root.join("wall.kn5"),
        f::file(f::base(&[wall], Mat4::IDENTITY.to_cols_array(), 1), 6),
    )
    .unwrap();
    std::fs::write(
        root.join("models.ini"),
        "[MODEL_0]\nFILE=wall.kn5\nPOSITION=0,0,0\nROTATION=0,0,0\n",
    )
    .unwrap();
    let collider = ColliderBox::load(
        &root,
        asc_vehicle::data::Mode::Unpacked,
        None,
        [0.; 3],
        [0., 0., 0., 1.],
    )
    .unwrap();
    let walls = WallIndex::load(&root.join("models.ini")).unwrap();
    (Directory(root), collider, walls)
}
fn settings() -> WallStepSettings {
    WallStepSettings {
        allow_certified_departures: false,
        sweep: WallSweepSettings::default(),
        contacts: asc_physics::ChassisContactSettings::default(),
        friction: 0.,
    }
}
#[test]
fn clear_segment_equals_ordinary_step_and_initial_contact_returns_zero_time_without_looping() {
    let (_dir, collider, walls) = fixture();
    let mut system = system().unwrap();
    let racks = system.rack_motions();
    let forces = |_, _: &asc_physics::ChassisKinematics| {
        Ok(ChassisLoads {
            chassis: Wrench::default(),
            uprights: vec![Wrench::default()],
            wheels: vec![WheelLoad::default()],
        })
    };
    let mut ordinary = system.clone();
    ordinary
        .step_with_racks_and_forces(0.001, |_| Ok(racks.clone()), forces)
        .unwrap();
    let clear = step_to_wall(
        &mut system,
        0.001,
        &collider,
        &walls,
        settings(),
        |_| Ok(racks.clone()),
        forces,
    )
    .unwrap();
    assert!(clear.impact.is_none());
    assert_eq!(clear.remaining_s, 0.);
    assert_eq!(
        serde_json::to_value(system.kinematics().unwrap()).unwrap(),
        serde_json::to_value(ordinary.kinematics().unwrap()).unwrap()
    );
    let hit = step_to_wall(
        &mut system,
        0.01,
        &collider,
        &walls,
        settings(),
        |_| Ok(racks.clone()),
        forces,
    )
    .unwrap();
    assert!(hit.impact.is_some());
    let position = system.kinematics().unwrap().chassis.position_m;
    let initial = step_to_wall(
        &mut system,
        0.001,
        &collider,
        &walls,
        settings(),
        |_| Ok(racks.clone()),
        forces,
    )
    .unwrap();
    assert_eq!(initial.integration.elapsed_s, 0.);
    assert_eq!(initial.remaining_s, 0.001);
    assert_eq!(system.kinematics().unwrap().chassis.position_m, position);
}
#[test]
fn numerical_proposal_sweep_and_coupled_impact_commit_at_the_same_pose_and_return_remaining_time() {
    let (_dir, collider, walls) = fixture();
    let mut system = system().unwrap();
    let racks = system.rack_motions();
    let result = step_to_wall(
        &mut system,
        0.01,
        &collider,
        &walls,
        settings(),
        |_| Ok(racks.clone()),
        |_, _| {
            Ok(ChassisLoads {
                chassis: Wrench::default(),
                uprights: vec![Wrench::default()],
                wheels: vec![WheelLoad::default()],
            })
        },
    )
    .unwrap();
    assert!(result.integration.elapsed_s > 0.0049 && result.integration.elapsed_s < 0.0051);
    assert!((result.remaining_s + result.integration.elapsed_s - 0.01).abs() < 1e-15);
    let after = system.kinematics().unwrap();
    assert_eq!(
        after.chassis.position_m,
        result.integration.sample.chassis.position_m
    );
    assert_eq!(
        after.chassis.orientation_xyzw,
        result.integration.sample.chassis.orientation_xyzw
    );
    assert!(result.impact.as_ref().unwrap().net_impulse_world_ns[0] > 100.);
    assert!(result.impact.as_ref().unwrap().work.external_impulse_work_j < 0.);
    assert!(after.chassis.velocity_world_m_s[0].abs() < 1e-6);
    assert_eq!(result.sweep.contacts[0].mesh, "1WALL");
}
#[test]
fn sweep_and_impulse_failures_rollback_the_entire_integrated_segment() {
    for case in 0..3 {
        let (_dir, collider, walls) = fixture();
        let mut system = system().unwrap();
        let before = serde_json::to_value(system.kinematics().unwrap()).unwrap();
        let racks = system.rack_motions();
        let mut settings = settings();
        if case == 0 {
            settings.sweep.max_iterations = 1;
        } else if case == 1 {
            settings.contacts.max_iterations = 0;
        } else {
            settings.friction = -1.;
        }
        let result = step_to_wall(
            &mut system,
            0.01,
            &collider,
            &walls,
            settings,
            |_| Ok(racks.clone()),
            |_, _| {
                Ok(ChassisLoads {
                    chassis: Wrench::default(),
                    uprights: vec![Wrench::default()],
                    wheels: vec![WheelLoad::default()],
                })
            },
        );
        assert!(result.is_err());
        assert_eq!(
            serde_json::to_value(system.kinematics().unwrap()).unwrap(),
            before
        );
    }
}

#[test]
fn certified_departure_consumes_remaining_time_but_renewed_inward_force_stays_an_event() {
    let (_dir, collider, walls) = fixture();
    let mut system = system().unwrap();
    let racks = system.rack_motions();
    let forces = |_, _: &asc_physics::ChassisKinematics| {
        Ok(ChassisLoads {
            chassis: Wrench::default(),
            uprights: vec![Wrench::default()],
            wheels: vec![WheelLoad::default()],
        })
    };
    let hit = step_to_wall(
        &mut system,
        0.01,
        &collider,
        &walls,
        settings(),
        |_| Ok(racks.clone()),
        forces,
    )
    .unwrap();
    let remaining = hit.remaining_s;
    let position = system.kinematics().unwrap().chassis.position_m;
    let mut departure = settings();
    departure.allow_certified_departures = true;
    let mut driven = system.clone();
    let rest = step_to_wall(
        &mut system,
        remaining,
        &collider,
        &walls,
        departure,
        |_| Ok(racks.clone()),
        forces,
    )
    .unwrap();
    assert_eq!(rest.remaining_s, 0.);
    assert!(rest.impact.is_none());
    assert_eq!(rest.sweep.certified_departures.len(), 1);
    assert!(rest.sweep.certified_departures[0].minimum_projected_gap_m > 0.);
    assert!((system.kinematics().unwrap().chassis.position_m[0] - position[0]).abs() < 1e-10);
    let renewed = step_to_wall(
        &mut driven,
        remaining,
        &collider,
        &walls,
        departure,
        |_| Ok(racks.clone()),
        |_, _| {
            Ok(ChassisLoads {
                chassis: Wrench::new([-100., 0., 0.], [0.; 3]).unwrap(),
                uprights: vec![Wrench::default()],
                wheels: vec![WheelLoad::default()],
            })
        },
    )
    .unwrap();
    assert_eq!(renewed.integration.elapsed_s, 0.);
    assert_eq!(renewed.remaining_s, remaining);
    assert!(renewed.sweep.certified_departures.is_empty());
}
