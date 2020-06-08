mod data;

use data::Pose;
use rumqtt::{MqttClient, MqttOptions, QoS, Notification, ReconnectOptions};
use log::*;
use std::collections::HashMap;
use std::sync::{ Mutex, Arc };


use kiss3d::window::Window;
use nalgebra::{ Point3, Point2 };


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

fn main() {
    let points = Arc::new(Mutex::new(HashMap::new()));
    subscribe(points.clone());
    let mut window = Window::new("point_cloud_view");
    window.set_background_color(0.5, 0.5, 0.5);
    window.set_point_size(3.0);

    let mut c = window.add_cube(1.0, 1.0, 1.0);
    c.set_color(1.0, 0.0, 0.0);
    
    // c.set_points_size(10.0);
    // c.set_lines_width(1.0);
    // c.set_surface_rendering_activation(false);
    let mut c = window.add_circle(20.0);
    c.set_color(1.0, 1.0, 0.0);

    while window.render() {
        let dict = points.lock().unwrap();
        for (_, pose) in &(*dict) {
            let pos = Point3::new(pose.position.x, pose.position.y, pose.position.z);
            let color = Point3::new(0.5, 0.0, 0.5);
            window.draw_point(&pos, &color);
        }
    }
}