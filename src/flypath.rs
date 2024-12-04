use crossbeam_channel::{select_biased, Receiver, Sender};
use rand::Rng;
use std::collections::HashMap;
use std::fmt;
use wg_2024::controller::{DroneCommand, DroneEvent};
use wg_2024::controller::DroneEvent::ControllerShortcut;
use wg_2024::drone::Drone;
use wg_2024::network::{NodeId, SourceRoutingHeader};
use wg_2024::packet::NackType::ErrorInRouting;
use wg_2024::packet::{Ack, Nack, NackType, Packet, PacketType};

#[derive(Debug, Clone)]
pub enum FlyPathModes {
    /// The drone behaves as describe in the protocol file.
    Default,

    #[cfg(feature = "modes")]
    // TODO: more documentations on the possible messages
    /// Custom messages based on the selected themes.
    Spicy(FlyPathThemes),

    #[cfg(feature = "modes")]
    /// The drone behaves erratically, has comptetely gone mad, and it doesn't act like it should. It's dangerous!!!!!!!
    BrainRot,
}

#[derive(Debug, Clone)]
#[cfg(feature = "modes")]
pub enum FlyPathThemes {
    Batman,
    Rocket,
    // Quackable,
    // HarryPotter,
    // GerryScotty,
    // DarkSoulAndBloodborn,
    // Pingu,
}

#[cfg(feature = "modes")]
impl fmt::Display for FlyPathThemes {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let theme_str = match self {
            FlyPathThemes::Batman => "Batman",
            FlyPathThemes::Rocket => "Rocket",
            // FlyPathThemes::Quackable => "Quackable",
            // FlyPathThemes::HarryPotter => "HarryPotter",
            // FlyPathThemes::GerryScotty => "GerryScotty",
            // FlyPathThemes::DarkSoulAndBloodborn => "DarkSoulAndBloodborn",
            // FlyPathThemes::Pingu => "Pingu",
        };
        write!(f, "{}", theme_str)
    }
}

#[derive(Debug, Clone)]
pub struct FlyPath {
    pub id: NodeId,
    // send to controller the DroneEvent
    pub controller_send: Sender<DroneEvent>,
    // receive from the controleller a command
    pub controller_recv: Receiver<DroneCommand>,
    // receive a packet from a connected drone
    pub packet_recv: Receiver<Packet>,
    // hashmap that contains Sender to connected drones
    pub packet_send: HashMap<NodeId, Sender<Packet>>,
    // packet drop rate
    pub pdr: f32,
    // drone's mode, optional
    pub mode: FlyPathModes,
}

impl Drone for FlyPath {
    fn new(
        id: NodeId,
        controller_send: Sender<DroneEvent>,
        controller_recv: Receiver<DroneCommand>,
        packet_recv: Receiver<Packet>,
        packet_send: HashMap<NodeId, Sender<Packet>>,
        pdr: f32,
    ) -> Self {
        Self {
            mode: FlyPathModes::Default,
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
        mode: FlyPathModes,
        id: NodeId,
        controller_send: Sender<DroneEvent>,
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

    fn command_handler(&mut self, cmd: DroneCommand) {
        match cmd {
            DroneCommand::AddSender(id, sender) => {
                if self.packet_send.insert(id, sender).is_some() {
                    println!("Drone {} updated sender with {}", self.id, id);
                } else {
                    println!("Drone {} added sender with {}", self.id, id);
                }
            }
            DroneCommand::RemoveSender(id) => {}
            DroneCommand::SetPacketDropRate(pdr) => {
                self.pdr = pdr;
                println!("Drone {} new pdr value: {}", self.id, self.pdr);
            }
            _ => {}
        }

        // in spicy modes here send another messages to the controller
    }

    // TODO: implement handler
    fn packet_handler(&self, packet: Packet) {
        // read Drone Protocol for the precise description of steps

        let mut hop_index = packet.routing_header.hop_index.clone();
        let mut clone_packet = packet.clone();

        if packet.routing_header.hops[hop_index] != self.id {
            let nack_packet = self.nack_creator(&packet, NackType::UnexpectedRecipient(self.id));
            self.controller_send
                .send(DroneEvent::ControllerShortcut(nack_packet));
            return;
        }

        hop_index += 1;

        if hop_index == packet.routing_header.hops.len() {
            let nack_packet = self.nack_creator(&packet, NackType::DestinationIsDrone);
            self.controller_sender(DroneEvent::ControllerShortcut(nack_packet));
            return;
        }

        if self
            .packet_send
            .contains_key(&packet.routing_header.hops[packet.routing_header.hop_index])
        {
            match &packet.pack_type {
                PacketType::MsgFragment(frag) => {
                    let mut rng = rand::thread_rng();
                    let random_number = rng.gen_range(0.0..1.0);
                    if random_number > self.pdr {
                        let nack_packet = self.nack_creator(&packet, NackType::Dropped);
                        self.packet_sender(nack_packet);
                    } else {
                        self.packet_sender(packet);
                    }
                }
                PacketType::Nack(nack) => {
                    self.packet_sender(packet);
                }
                PacketType::Ack(ack) => {
                    self.packet_sender(packet);
                }
                PacketType::FloodRequest(floodrequest) => {

                }
                PacketType::FloodResponse(floodresponse) => {
                    
                }
            }
        } else {
        }
    }

    fn nack_creator(&self, packet: &Packet, nack_type: NackType) -> Packet {
        let mut reverse_hops = packet.routing_header.hops.clone();
        reverse_hops.truncate(packet.routing_header.hop_index);
        reverse_hops.reverse();

        let nack = Nack {
            fragment_index: match &packet.pack_type {
                PacketType::Ack(ack) => ack.fragment_index,
                PacketType::Nack(nack) => nack.fragment_index,
                PacketType::MsgFragment(frag) => frag.fragment_index,
                _ => 0,
            },
            nack_type,
        };

        Packet {
            pack_type: PacketType::Nack(nack),
            routing_header: SourceRoutingHeader {
                hop_index: 1,
                hops: reverse_hops,
            },
            session_id: packet.session_id,
        }
    }

    fn controller_sender(&self, packet: DroneEvent) {
        self.controller_send.send(packet).unwrap();
    }

    fn packet_sender(&self, packet: Packet) {
        let next_hop = &packet.routing_header.hops[packet.routing_header.hop_index];
        self.packet_send
            .get(next_hop)
            .unwrap()
            .send(packet)
            .unwrap_or_else(|e| {
                println!("Error sending packet: {}", e);
                let nack_packet =
                self.nack_creator(&packet, NackType::ErrorInRouting(next_hop.clone()));
                self.controller_send.send(ControllerShortcut(nack_packet)).unwrap();
            });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crossbeam_channel::unbounded;

    // virtual connected drone's ID is 2.
    fn setup_test_drone(
        pdr: f32,
    ) -> (
        FlyPath,
        Receiver<DroneEvent>,
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
