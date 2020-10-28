use anyhow::Result;
use hamilton::hamilton_remote_client::HamiltonRemoteClient;
use std::thread;
use std::thread::JoinHandle;
use tokio::runtime::Runtime;
use tokio::sync::mpsc;
use tokio::sync::mpsc::{Receiver, Sender};
use tonic::Request;

pub mod hamilton {
    tonic::include_proto!("hamilton");
}

pub struct HamitlonRemoteController {
    sender: Option<Sender<hamilton::MoveRequest>>,
    join_handle: Option<JoinHandle<()>>,
}

impl HamitlonRemoteController {
    pub fn new(address: String) -> Self {
        let (tx, rx) = mpsc::channel(10);
        let handle = thread::spawn(|| {
            create_remote(rx, address).unwrap();
        });
        HamitlonRemoteController {
            sender: Some(tx),
            join_handle: Some(handle),
        }
    }

    pub fn send_command(&self, x: f32, y: f32, yaw: f32) {
        if let Some(ref sender) = self.sender {
            sender
                .blocking_send(hamilton::MoveRequest {
                    command: Some(hamilton::MoveCommand { x, y, yaw }),
                })
                .unwrap();
        }
    }
}

impl Drop for HamitlonRemoteController {
    fn drop(&mut self) {
        println!("Dropping sender");
        let sender = self.sender.take().unwrap();
        drop(sender);
        println!("Sender dropped");

        let join_handle = self.join_handle.take().unwrap();
        join_handle.join().unwrap();
        println!("Join handle joined");
    }
}

fn create_remote(receiver: Receiver<hamilton::MoveRequest>, address: String) -> Result<()> {
    let rt = Runtime::new()?;
    rt.block_on(async move {
        let receiver = receiver;
        let mut hamilton_client = HamiltonRemoteClient::connect(address).await?;
        hamilton_client.move_stream(Request::new(receiver)).await?;
        Ok(())
    })
}
