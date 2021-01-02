use hamilton_controller::hamilton_remote;

use gilrs::Gilrs;
use std::thread::sleep;
use std::time::Duration;

fn main() {
    let mut gilrs = Gilrs::new().unwrap();
    let mut remote =
        hamilton_remote::HamitlonRemoteController::new(String::from("http://pi42.local:5001"));
    loop {
        sleep(Duration::from_millis(30));
        // force consume all events
        while gilrs.next_event().is_some() {}
        // get primary gamepad
        if let Some((_, gamepad)) = gilrs.gamepads().next() {
            let deadzone = 0.2;
            if gamepad.is_connected() {
                let x = gamepad.value(gilrs::Axis::LeftStickY);
                let y = gamepad.value(gilrs::Axis::LeftStickX);
                let yaw = gamepad.value(gilrs::Axis::RightStickX);
                if x.abs() > deadzone || y.abs() > deadzone || yaw.abs() > deadzone {
                    if remote.send_command(x, -y, -yaw).is_err() {
                        println!("Error sending command");
                    }
                } else if remote.send_command(0., 0., 0.).is_err() {
                    println!("Failed sending stop command");
                }
            }
        }
    }
}
