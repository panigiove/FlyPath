use crossbeam_channel::{select_biased, Receiver, Sender};
use std::collections::HashMap;
use wg_2024::network::NodeId;
use wg_2024::packet::Packet;
use wg_2024::drone::{Drone, DroneOptions};
use wg_2024::controller::{DroneCommand, NodeEvent};

#[derive(Debug, Clone)]
pub struct FlyPath {
    pub id: NodeId,
    pub controller_send: Sender<NodeEvent>,
    pub controller_recv: Receiver<DroneCommand>,
    pub packet_recv: Receiver<Packet>,
    pub packet_send: HashMap<NodeId, Sender<Packet>>,
    pub pdr: f32,
}

// Implement the Drone trait for FlyPath
impl Drone for FlyPath {
    fn new(options: DroneOptions) -> Self {
        Self {
            id: options.id,
            controller_send: options.controller_send,
            controller_recv: options.controller_recv,
            packet_recv: options.packet_recv,
            packet_send: options.packet_send,
            pdr: options.pdr,
        }
    }

    fn run(&mut self) {
        loop{
            select_biased!{
                recv(self.controller_recv) -> cmd => {
                    if let Ok(cmd) = cmd {
                        self.command_handler(cmd);
                    }
                },
                recv(self.packet_recv) -> packet => {
                    if let Ok(packet) = packet {
                        self.packet_handler(packet);
                    }
                }
            }        
        }
    }
}


impl FlyPath {
    // TODO: implement handler
    fn command_handler (&self, cmd: DroneCommand ){
        
    }

    fn packet_handler (&self, packet:Packet){

    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crossbeam_channel::unbounded;

    // virtual connected drone's ID is 2.
    fn setup_test_drone(pdr:f32) -> (FlyPath, Receiver<NodeEvent>, Sender<DroneCommand>, Sender<Packet>, Receiver<Packet>) {
        let (drone_event_send, test_event_recv) = unbounded();
        let (test_command_send, drone_command_recv) = unbounded();
        let (test_packet_send, drone_packet_recv) = unbounded();
        let (drone_packet_send, test_packet_recv) = unbounded();

        let config: DroneOptions = DroneOptions { 
            id: 1,
            controller_recv: drone_command_recv,
            controller_send: drone_event_send,
            packet_recv: drone_packet_recv,
            packet_send: vec![(2, drone_packet_send)].into_iter().collect(),
            pdr,
        };

        (FlyPath::new(config), test_event_recv, test_command_send, test_packet_send, test_packet_recv)
    }

    // #[test]
    // fn test_() {
        
    //     assert_eq!(expected, actual);
    // }
}