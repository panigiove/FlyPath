use crossbeam_channel::{select_biased, Receiver, Sender};
use std::collections::HashMap;
use wg_2024::controller::{DroneCommand, NodeEvent};
use wg_2024::drone::Drone;
use wg_2024::network::NodeId;
use wg_2024::packet::{Ack, Packet, PacketType};

#[derive(Debug, Clone)]
#[cfg(feature = "modes")]
pub enum FlyPathMode {
    /// The drone behaves as describe in the protocol file.
    Default,
    // TODO: more documentations on the possible messages
    /// Custom messages based on the selected themes.
    Spicy(FlyPathThemes),
    /// The drone behaves erratically, has comptetely gone mad, and it doesn't act like it should. It's dangerous!!!!!!!
    BrainRot,
}

#[derive(Debug, Clone)]
#[cfg(feature = "modes")]
pub enum FlyPathThemes {
    Batman,
    Rocket,
    Quackable,
    HarryPotter,
    GerryScotty,
    DarkSoulAndBloodborn,
    Pingu,
}

#[derive(Debug, Clone)]
pub struct FlyPath {
    pub id: NodeId,
    // send to controller the NodeEvent
    pub controller_send: Sender<NodeEvent>,
    // receive from the controleller a command
    pub controller_recv: Receiver<DroneCommand>,
    // receive a packet from a connected drone
    pub packet_recv: Receiver<Packet>,
    // hashmap that contains Sender to connected drones
    pub packet_send: HashMap<NodeId, Sender<Packet>>,
    // packet drop rate
    pub pdr: f32,
    // drone's mode, optional 
    #[cfg(feature = "modes")]
    pub mode: FlyPathMode,
}

impl Drone for FlyPath {
    fn new(
        id: NodeId,
        controller_send: Sender<NodeEvent>,
        controller_recv: Receiver<DroneCommand>,
        packet_recv: Receiver<Packet>,
        packet_send: HashMap<NodeId, Sender<Packet>>,
        pdr: f32,
    ) -> Self {
        Self {
            #[cfg(feature = "modes")]
            mode: FlyPathMode::Default,
            id,
            controller_send,
            controller_recv,
            packet_recv,
            packet_send,
            pdr,
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
    #[cfg(feature = "modes")]
    pub fn new_with_mode(
        mode: FlyPathMode,
        id: NodeId,
        controller_send: Sender<NodeEvent>,
        controller_recv: Receiver<DroneCommand>,
        packet_recv: Receiver<Packet>,
        packet_send: HashMap<NodeId, Sender<Packet>>,
        pdr: f32,
    ) -> Self {
        Self {
            mode,
            id,
            controller_send,
            controller_recv,
            packet_recv,
            packet_send,
            pdr,
        }
    }

    #[cfg(not(feature = "modes"))]
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

    #[cfg(not(feature = "modes"))]
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

    #[cfg(feature = "modes")]
    fn command_handler(&mut self, cmd: DroneCommand) {}

    #[cfg(feature = "modes")]
    fn packet_handler(&self, packet: Packet) {}
}

#[cfg(test)]
mod tests {
    use super::*;
    use crossbeam_channel::unbounded;

    // virtual connected drone's ID is 2.
    fn setup_test_drone_for_threading(
        pdr: f32,
    ) -> (
        FlyPath,
        Receiver<NodeEvent>,
        Sender<DroneCommand>,
        Receiver<Packet>,
        Sender<Packet>,
    ) {
        let (drone_event_send, test_event_recv) = unbounded();
        let (test_command_send, drone_command_recv) = unbounded();
        let (test_packet_send, drone_packet_recv) = unbounded();
        let (drone_packet_send, test_packet_recv) = unbounded();

        (
            FlyPath::new(
                1,
                drone_event_send,
                drone_command_recv,
                drone_packet_recv,
                vec![(2, drone_packet_send)].into_iter().collect(),
                pdr,
            ),
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
