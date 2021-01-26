use anyhow::Result;
use hamilton_service::hamilton_remote_client::HamiltonRemoteClient;
use std::thread;
use std::thread::JoinHandle;
use tokio::runtime::Runtime;
use tokio::sync::mpsc;
use tokio::sync::mpsc::{Receiver, Sender};
use tonic::Request;

pub mod hamilton_service {
    tonic::include_proto!("hamilton_service");
}

pub struct HamitlonRemoteController {
    sender: Option<Sender<hamilton_service::MoveRequest>>,
    join_handle: Option<JoinHandle<()>>,
    handle: tokio::runtime::Handle,
}

impl HamitlonRemoteController {
    pub fn new(address: String) -> Self {
        let (tx, rx) = mpsc::channel(10);
        let rt = Runtime::new().unwrap();
        let handle = rt.handle().clone();
        let join_handle = thread::spawn(|| {
            create_remote(rx, address, rt).unwrap();
        });
        HamitlonRemoteController {
            sender: Some(tx),
            join_handle: Some(join_handle),
            handle,
        }
    }

    pub fn send_command(&mut self, x: f32, y: f32, yaw: f32) -> Result<()> {
        if let Some(ref mut sender) = self.sender {
            self.handle
                .block_on(sender.send(hamilton_service::MoveRequest {
                    command: Some(hamilton_service::MoveCommand { x, y, yaw }),
                }))?;
        }
        Ok(())
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

fn create_remote(
    receiver: Receiver<hamilton_service::MoveRequest>,
    address: String,
    rt: Runtime,
) -> Result<()> {
    let mut rt = rt;
    rt.block_on(async {
        let receiver = receiver;
        let mut hamilton_client = HamiltonRemoteClient::connect(address).await?;
        hamilton_client.move_stream(Request::new(receiver)).await?;
        Ok(())
    })
}
