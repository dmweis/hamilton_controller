mod data;

use data::Pose;
use rumqtt::{MqttClient, MqttOptions, QoS, Notification, ReconnectOptions};
use log::*;
use std::collections::HashMap;
use std::sync::{ Mutex, Arc };

use kiss3d::window::Window;
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Point3};



fn subscribe(points: Arc<Mutex<HashMap<i32, Pose>>>) {
    std::thread::spawn(move || {
        // topics
        let tracking = "tracking/pose";
    
        let topics = vec![
            tracking,
        ];
    
        let mqtt_options = MqttOptions::new("hamilton_controller", "mqtt.local", 1883)
            .set_reconnect_opts(ReconnectOptions::Always(5));
        let (mut mqtt_client, notifications) = MqttClient::start(mqtt_options)
                                                  .expect("Failed to connect to MQTT host");
        info!("Connected to MQTT");
    
        for topic_name in &topics {
            mqtt_client.subscribe(topic_name.to_owned(), QoS::AtMostOnce)
                       .expect(&format!("Failed to subscribe to topic {}", topic_name));
            trace!("Subscribing to {}", topic_name);
        }
        
    
        for notification in notifications {
            match notification {
                Notification::Publish(message) => {
                    if message.topic_name == "tracking/pose" {
                        match Pose::deserialize(&message.payload) {
                            Ok(pose) => {
                                let mut dict = points.lock().unwrap();
                                dict.insert(pose.device_index, pose.clone());
                            },
                            Err(error) => {
                                println!("{:?}", error);
                            }
                        }
                    }
                },
                _ => {}
            }
        }
    });
}

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
            let trans = Isometry3::from_parts(
                Translation3::new(size * x_ind, 0.0, size * y_ind),
                UnitQuaternion::from_euler_angles(0.0, -1.57, -1.57),
            );
            cube.set_local_transformation(trans);
        }
    }
}


fn main() {
    let points = Arc::new(Mutex::new(HashMap::new()));
    subscribe(points.clone());
    let mut window = Window::new("point_cloud_view");
    window.set_background_color(0.5, 0.5, 0.5);
    window.set_point_size(10.0);

    add_ground_plane(&mut window);

    while window.render() {
        let dict = points.lock().unwrap();
        for (_, pose) in &(*dict) {
            let pos = Point3::new(pose.position.x, pose.position.y, pose.position.z);
            let color = Point3::new(0.5, 0.0, 0.5);
            window.draw_point(&pos, &color);
        }
    }
}