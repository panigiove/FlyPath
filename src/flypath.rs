use crossbeam_channel::{select_biased, Receiver, Sender};
use std::collections::HashMap;
use wg_2024::controller::{DroneCommand, NodeEvent};
use wg_2024::drone::{Drone, DroneOptions};
use wg_2024::network::NodeId;
use wg_2024::packet::{Ack, Packet, PacketType};

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
        loop {
            select_biased! {
                recv(self.controller_recv) -> cmd => {
                    if let Ok(cmd) = cmd {
                        if let DroneCommand::Crash = cmd{
                            println!("Drone {} has crashed", self.id);
                            break;
                        }
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
    fn command_handler(&mut self, cmd: DroneCommand) {
        match cmd {
            DroneCommand::AddSender(id, sender) => {
                if self.packet_send.insert(id, sender).is_some() {
                    println!("Drone {} updated sender with {}", self.id, id);
                } else {
                    println!("Drone {} added sender with {}", self.id, id);
                }
            }
            DroneCommand::SetPacketDropRate(pdr) => {
                self.pdr = pdr;
                println!("Drone {} new pdr value: {}", self.id, self.pdr);
            }
            _ => {}
        }
    }

    // TODO: implement handler
    fn packet_handler(&self, packet: Packet) {
        // read Drone Protocol for the precise description of steps
        match packet.pack_type {
            PacketType::FloodRequest(req) => {
                // TODO: view in Flooding Initialization on Neighbor Response
            }
            PacketType::FloodResponse(req) => {
                // TODO: view in Flooding Initialization on Neighbor Response, forward
            }
            PacketType::Ack(ack) => {
                // TODO: forward
            }
            PacketType::Nack(nack) => {
                // TODO: forward
            }
            PacketType::MsgFragment(frag) => {
                // TODO: fragment are part of a message and can be droopped by drones, if not dropped forward and send an ACK otherwise send a NACK
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crossbeam_channel::unbounded;
    use std::sync::{Arc, Mutex};

    // example of usage:
    // let (drone, recv_event, send_command, recv_packet, send_packet) = setup_test_drone(0.0);
    // let drone_clone = Arc::clone(&drone);
    // thread::spawn(move || {drone_clone.run});
    //
    // virtual connected drone's ID is 2.
    fn setup_test_drone(
        pdr: f32,
    ) -> (
        Arc<Mutex<FlyPath>>,
        Receiver<NodeEvent>,
        Sender<DroneCommand>,
        Receiver<Packet>,
        Sender<Packet>,
    ) {
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

        (
            Arc::new(Mutex::new(FlyPath::new(config))),
            test_event_recv,
            test_command_send,
            test_packet_recv,
            test_packet_send,
        )
    }

    // #[test]
    // fn test_drone_crashed() {
    //     let (drone, reciver_event, sender_command, reciver_packet, sender_packet) = setup_test_drone(0.0);
    //     assert!(matches!(sender_command.send(DroneCommand::Crash), Ok(())));
    // }
    //
    // #[test]
    // fn test_add_sender() {
    //     let (drone, reciver_event, sender_command, reciver_packet, sender_packet) = setup_test_drone(0.0);
    //     let mut drone_pointer = Arc::new(drone);
    //     let mut pointer_drone_clone = drone_pointer.clone();
    //     let thred = thread::spawn(move || {pointer_drone_clone.run()});
    //     let (s,r) = unbounded();
    //     sender_command.send(DroneCommand::AddSender(3, s)).unwrap();
    //     assert!(drone_pointer.check_conection(3));
    // }
}
