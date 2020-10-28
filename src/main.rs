mod hamilton_remote;
mod openvr_adaptor;

use kiss3d::window::Window;
use nalgebra as na;

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

fn main() {
    let mut window = Window::new("point_cloud_view");
    let mut openvr = openvr_adaptor::VrDeviceManager::new().unwrap();

    window.set_background_color(0.5, 0.5, 0.5);

    add_ground_plane(&mut window);

    while window.render() {
        openvr.update(&mut window);
    }
}
