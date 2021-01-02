use hamilton_controller::hamilton_remote;
use hamilton_controller::openvr_adaptor;

use gilrs::Gilrs;
use kiss3d::window::Window;
use nalgebra as na;
use pid_control::Controller;

fn add_ground_plane(window: &mut Window) {
    let size = 0.5;
    for i in 0..4 {
        for j in 0..4 {
            let mut cube = window.add_cube(size, size, 0.001);
            if (i + j) % 2 == 0 {
                cube.set_color(1.0, 0.3, 0.2);
            } else {
                cube.set_color(0.5, 0.04, 0.17);
            }
            let distance = (1_f32.powi(2) + 1_f32.powi(2)).sqrt();
            let x_ind = j as f32 - distance;
            let y_ind = i as f32 - distance;
            let trans = na::Isometry3::from_parts(
                na::Translation3::new(size * x_ind, 0.0, size * y_ind),
                na::UnitQuaternion::from_euler_angles(0.0, -1.57, -1.57),
            );
            cube.set_local_transformation(trans);
        }
    }
}

fn add_orientation(window: &mut Window, position: na::Vector3<f32>) -> kiss3d::scene::SceneNode {
    let length = 0.5;
    let mut node = window.add_group();
    node.set_local_translation(position.into());
    let mut x_arrow = node.add_cube(length, 0.01, 0.01);
    let mut y_arrow = node.add_cube(0.01, length, 0.01);
    let mut z_arrow = node.add_cube(0.01, 0.01, length);
    x_arrow.set_color(1.0, 0.0, 0.0);
    x_arrow.append_translation(&na::Translation3::new(length * 0.5, 0.0, 0.0));
    y_arrow.set_color(0.0, 1.0, 0.0);
    y_arrow.append_translation(&na::Translation3::new(0.0, length * 0.5, 0.0));
    z_arrow.set_color(0.0, 0.0, 1.0);
    z_arrow.append_translation(&na::Translation3::new(0.0, 0.0, length * 0.5));
    node
}

fn main() {
    let white = na::Point3::new(1.0, 1.0, 1.0);

    let mut window = Window::new("Hamilton viewer");
    let mut openvr = openvr_adaptor::VrDeviceManager::new().unwrap();
    let mut gilrs = Gilrs::new().unwrap();
    let mut remote =
        hamilton_remote::HamitlonRemoteController::new(String::from("http://pi42.local:5001"));

    let mut yaw_controller = pid_control::PIDController::new(1.0, 0.0, 0.0);
    yaw_controller.out_max = 1.0;
    let mut last_update = std::time::Instant::now();
    let last_known_yaw = -90.0;

    window.set_background_color(0.5, 0.5, 0.5);

    let mut robot_node = window.add_cube(0.1, 0.1, 0.1);

    add_ground_plane(&mut window);
    let _orientation_node = add_orientation(&mut window, na::Vector3::new(1.5, 0.0, 1.5));

    while window.render() {
        openvr.update(&mut window);
        let robot_tracker = openvr.get_device_by_class(openvr_adaptor::VrDeviceClass::Tracker);
        if let Some(robot_tracker) = robot_tracker {
            let tracker_position = na::Isometry3::from_parts(
                robot_tracker.position.coords.into(),
                robot_tracker.rotation,
            );
            let robot_pose = tracker_position * na::Translation3::new(0.2, 0.0, 0.0);
            // calculate looking error
            if let Some(right_hand) =
                openvr.get_device_by_class(openvr_adaptor::VrDeviceClass::RightController)
            {
                let aa_robot_position: na::Vector2<f32> = robot_pose.translation.vector.xz();
                let aa_target_position: na::Vector2<f32> = right_hand.position.xz().coords;
                // let target_dir_global = na::Rotation2::new(tracker_position.rotation.)
                //     .transform_vector(&(aa_target_position - aa_robot_position).normalize());
                let target_dir_local = (aa_target_position - aa_robot_position).normalize();
                // let forward_dir =
                window.draw_line(
                    &na::Point3::new(aa_robot_position.x, 0.1, aa_robot_position.y),
                    &na::Point3::new(aa_target_position.x, 0.1, aa_target_position.y),
                    &na::Point3::new(1.0, 0.0, 1.0),
                );
                let _dot = target_dir_local.dot(&na::Vector2::new(0.0, -1.0));
                // println!(
                //     "target_dir_local x: {:.2} y: {:.2}",
                //     target_dir_local.x, target_dir_local.y
                // );
                // println!(
                //     "target_dir_global x: {:.2} y: {:.2}",
                //     target_dir_global.x, target_dir_global.y
                // );
            }
            // end
            // last_known_yaw = actual_yaw;
            robot_node.set_local_transformation(robot_pose);
            robot_node.set_visible(true);
        } else {
            robot_node.set_visible(false);
        }
        // force consume all events
        while gilrs.next_event().is_some() {}
        // get primary gamepad
        if let Some((_, gamepad)) = gilrs.gamepads().next() {
            let deadzone = 0.2;
            if gamepad.is_connected() {
                let x = gamepad.value(gilrs::Axis::LeftStickY);
                let y = gamepad.value(gilrs::Axis::LeftStickX);
                let yaw = gamepad.value(gilrs::Axis::RightStickX);
                if gamepad.is_pressed(gilrs::Button::South) {
                    yaw_controller.set_target(0.0);
                    let drive = yaw_controller
                        .update(last_known_yaw as f64, last_update.elapsed().as_secs_f64());
                    println!(
                        "plant: {:.2} target: {:.2} output: {:.2}",
                        last_known_yaw, 0, drive
                    );
                    remote
                        .send_command(0., 0., drive as f32)
                        .expect("Failed talking to robot");
                    last_update = std::time::Instant::now();
                } else if x.abs() > deadzone || y.abs() > deadzone || yaw.abs() > deadzone {
                    remote
                        .send_command(x, -y, -yaw)
                        .expect("Failed talking to robot");
                } else {
                    remote
                        .send_command(0., 0., 0.)
                        .expect("Failed talking to robot");
                }
            }
        }

        window.draw_text(
            &openvr.display_data(),
            &na::Point2::new(1.0, 1.0),
            50.0,
            &kiss3d::text::Font::default(),
            &white,
        );
    }
}
