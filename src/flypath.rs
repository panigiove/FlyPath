use crossbeam_channel::{select_biased, Receiver, Sender};
use rand::Rng;
use std::collections::{HashMap, HashSet};
use std::mem::drop;
use wg_2024::controller::DroneEvent::ControllerShortcut;
use wg_2024::controller::{DroneCommand, DroneEvent};
use wg_2024::drone::Drone;
use wg_2024::network::{NodeId, SourceRoutingHeader};
use wg_2024::packet::{FloodRequest, FloodResponse, Nack, NackType, NodeType, Packet, PacketType};

// TODO: crash
// TODO: floodrequest
// TODO: RemoveSender
// TODO: modes
// TODO: tests

#[cfg(feature = "modes")]
use std::fmt;
use std::process::exit;
use wg_2024::packet::NackType::{Dropped, ErrorInRouting};

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
    Quackable,
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
            FlyPathThemes::Quackable => "Quackable",
            // FlyPathThemes::HarryPotter => "HarryPotter",
            // FlyPathThemes::GerryScotty => "GerryScotty",
            // FlyPathThemes::DarkSoulAndBloodborne => "DarkSoulAndBloodborne",
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
    pub precFloodId: HashSet<(u64, u8)>,
}

impl Drone for FlyPath {
    fn new(
        id: NodeId,
        controller_send: Sender<DroneEvent>,
        controller_recv: Receiver<DroneCommand>,
        packet_recv: Receiver<Packet>,
        mut packet_send: HashMap<NodeId, Sender<Packet>>,
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
            precFloodId: HashSet::new(),
        }
    }

    fn run(&mut self) {
        loop {
            select_biased! {
                recv(self.controller_recv) -> cmd => {
                    if let Ok(cmd) = cmd {
                        if let DroneCommand::Crash = cmd{
                            while let Ok(mut cmd) = self.packet_recv.try_recv() {
                                match &cmd.pack_type{
                                    PacketType::MsgFragment(_) => {
                                        self.send_nack(&cmd, ErrorInRouting(self.id));
                                    }
                                    PacketType::Ack(_) => {
                                        self.send_packet(& mut cmd);
                                    }
                                    PacketType::Nack(_) => {
                                        self.send_packet(& mut cmd);
                                    }
                                    PacketType::FloodRequest(_) => {}
                                    PacketType::FloodResponse(_) => {
                                        self.send_packet(& mut cmd);
                                    }
                                }
                            }
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
        mut packet_send: HashMap<NodeId, Sender<Packet>>,
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
            precFloodId: HashSet::new(),
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
            DroneCommand::RemoveSender(id) => {
                drop(self.packet_send.remove(&id));
                if self.packet_send.remove(&id).is_some() {
                    println!("Drone {} removed sender with {}", self.id, id);
                }
                else{
                    println!("Drone {} unable to remove sender with {}", self.id, id);
                }
            }
            DroneCommand::SetPacketDropRate(pdr) => {
                self.pdr = pdr;
                println!("Drone {} new pdr value: {}", self.id, self.pdr);
            }
            _ => {}
        }

        // in spicy modes here send another messages to the controller
    }

    fn packet_handler(&mut self, mut packet: Packet) {
        if let PacketType::FloodRequest(req) = &packet.pack_type {
            // TODO: floodrequest, WAITING FOR THE CORRECTION OF THE OTHER GROUP MEMBER
            let mut floodrequest = req.clone();
            floodrequest.path_trace.push((self.id, NodeType::Drone));
            if !self.precFloodId.contains(&(req.flood_id, req.initiator_id)) {

                let mut no_neightbours = true;
                self.precFloodId
                    .insert((req.flood_id, req.initiator_id));
                let new_packet = self.floodrequest_creator(&packet, &floodrequest);
                for i in &self.packet_send {
                    if *(i.0) != floodrequest.path_trace[req.path_trace.len() - 2].0 {
                        self.send_floodrequest_packet(new_packet.clone(), i.0);
                        no_neightbours = false;
                    }
                }
                if !no_neightbours {
                    let mut floodresponse = self.floodresponse_creator(&packet, &floodrequest);
                    self.send_packet(&mut floodresponse);
                }
            } else {
                let mut floodresponse = self.floodresponse_creator(&packet, &floodrequest);
                self.send_packet(&mut floodresponse);
            }
        }

        match self.validate_packet(&packet) {
            Some(nack) => {
                self.send_nack(&packet, nack);
                return;
            }
            None => {}
        }

        if let PacketType::MsgFragment(_) = &packet.pack_type {
            let should_drop_packet = self.pdr > rand::thread_rng().gen_range(0.0..1.0);
            if should_drop_packet {
                // Drop the fragment
                self.send_nack(&packet, NackType::Dropped);
                return;
            }
        }

        // The packet is a `Nack`, `Ack`, `FloodResponse` or a non dropped `MsgFragment`
        self.send_packet(&mut packet);
    }

    // if is not valid packet returns the errors, possible NackType are:
    // - UnexpectedRecipient
    // - ErrorInRouting
    // - DestinationIsDrone
    fn validate_packet(&self, packet: &Packet) -> Option<NackType> {
        // Packet forward to the wrong drone
        if self.id != packet.routing_header.hops[packet.routing_header.hop_index] {
            return Some(NackType::UnexpectedRecipient(self.id));
        }

        match packet.routing_header.next_hop() {
            Some(next_hop) => {
                if !self.packet_send.contains_key(&next_hop) {
                    // Next hop is not a neighbour
                    return Some(NackType::ErrorInRouting(
                        packet.routing_header.hops[packet.routing_header.hop_index],
                    ));
                }
            }
            // No next hop
            None => return Some(NackType::DestinationIsDrone),
        }
        // Valid Packet
        None
    }

    fn floodrequest_creator(&self, packet: &Packet, floodrequest: &FloodRequest) -> Packet {
        Packet {
            pack_type: PacketType::FloodRequest(floodrequest.clone()),
            routing_header: SourceRoutingHeader {
                hop_index: packet.routing_header.hop_index,
                hops: packet.routing_header.hops.clone(),
            },
            session_id: packet.session_id,
        }
    }

    fn floodresponse_creator(&self, packet: &Packet, flood_request: &FloodRequest) -> Packet {
        let mut reverse_hops = Vec::new();
        for (i) in flood_request.path_trace {
            reverse_hops.push(i.0);
        }
        reverse_hops.reverse();

        let floodresponse = FloodResponse {
            flood_id: flood_request.flood_id,
            path_trace: flood_request.path_trace.clone(),
        };

        Packet {
            pack_type: PacketType::FloodResponse(floodresponse),
            routing_header: SourceRoutingHeader {
                hop_index: 0,
                hops: reverse_hops,
            },
            session_id: packet.session_id,
        }
    }

    // *Increment the hop_index* and if all is ok send message
    // Send `Nack` if there is no next hop or next hop sender
    fn send_packet(&mut self, packet: &mut Packet) {
        packet.routing_header.increase_hop_index();
        let next_hop: NodeId = packet.routing_header.hops[packet.routing_header.hop_index];

        // Check if there's a sender for the next hop
        if let Some(sender) = self.packet_send.get_mut(&next_hop) {
            // Attempt to send the packet
            if sender.send(packet.clone()).is_err() {
                // If sending fails, send a NACK indicating an error in routing, IS CORRECT ErrorInRouting?
                self.packet_send.remove(&next_hop);
                self.send_nack(&packet, NackType::ErrorInRouting(next_hop));
            } else if let PacketType::MsgFragment(_) = packet.pack_type {
                // Send `Ack` To `Client`/`Server` and `PacketSent` to `Controller` if the successfull is a MsgFragment
                self.send_event(DroneEvent::PacketSent(packet.clone()));
            }
        } else {
            // If no sender is found for the next hop, send a NACK
            self.send_nack(&packet, NackType::ErrorInRouting(next_hop));
        }
    }

    fn send_floodrequest_packet(&self, packet: Packet, node_id: &NodeId) {
        if !self.packet_send.get(&node_id).unwrap().send(packet.clone()).is_err(){
            self.send_event(DroneEvent::PacketSent(packet));
        }
    }

    // Create and send a nack if the packet is a MsgFragment and send the event to `Controller` otherwise forward to Controller to send to destination
    // Panic if controller is not reacheable
    fn send_nack(&mut self, packet: &Packet, nack_type: NackType) {
        match &packet.pack_type {
            // FloodRequest can't enter in this function
            PacketType::FloodRequest(_) => {}
            PacketType::MsgFragment(fragment) => {
                // Reverse route
                let routing_header = SourceRoutingHeader::initialize(self.reverse_hops(packet));

                let nack = Nack {
                    fragment_index: fragment.fragment_index,
                    nack_type,
                };

                // if there is a problem with the nack packet the `send_packet` function will call again `send_nack` but will forward to `Controller` cus packet is Nack
                self.send_packet(&mut Packet::new_nack(
                    routing_header,
                    packet.session_id,
                    nack,
                ));

                // Send `PacketDropped` to `controller`
                // self.send_event(DroneEvent::PacketSent(packet.clone()));
                if nack_type == Dropped{
                    self.send_event(DroneEvent::PacketDropped(packet.clone()));
                }
            }
            _ => {
                // If an error occurs on a FloodResponse, Ack, Nack send to the client/server throw ControllerShortcut
                if let Err(e) = self
                    .controller_send
                    .send(ControllerShortcut(packet.clone()))
                {
                    // DOCUMENT THIS: Panic if no controller is listening or channel is dropped
                    panic!("No controller listening or Channel dropped: {}", e);
                }
            }
        }
    }

    fn send_event(&self, event: DroneEvent) {
        if self.controller_send.send(event).is_err() {
            panic!("Controller is unreaceable");
        }
    }

    fn reverse_hops(&self, packet: &Packet) -> Vec<NodeId> {
        let mut reverse_hops = packet.routing_header.hops.clone();
        reverse_hops.truncate(packet.routing_header.hop_index + 1);
        reverse_hops.reverse();
        reverse_hops
    }
}

#[cfg(test)]
mod tests {
    use std::env::join_paths;
    use super::*;
    use crossbeam_channel::unbounded;
    use std::thread;
    use wg_2024::packet::{Ack, Fragment};
    use wg_2024::tests;

    // virtual connected drone's ID is 2.
    fn setup_test_drone(
        pdr: f32,
    ) -> (
        FlyPath,
        Receiver<DroneEvent>,
        Sender<DroneCommand>,
        Receiver<Packet>,
        Sender<Packet>,
        Receiver<Packet>,
    ) {
        let (drone_event_send, test_event_recv) = unbounded();
        let (test_command_send, drone_command_recv) = unbounded();
        let (test_packet_send, drone_packet_recv) = unbounded();
        let (drone_packet_send, test_packet_recv) = unbounded();
        let (client_sender, client_reciver) = unbounded();

        (
            FlyPath::new(
                1,
                drone_event_send,
                drone_command_recv,
                drone_packet_recv,
                vec![(2, drone_packet_send), (3, client_sender)]
                    .into_iter()
                    .collect(),
                pdr,
            ),
            test_event_recv,
            test_command_send,
            test_packet_recv,
            test_packet_send,
            client_reciver,
        )
    }

    fn create_sample_packet() -> Packet {
        Packet {
            pack_type: PacketType::MsgFragment(Fragment {
                fragment_index: 1,
                total_n_fragments: 1,
                length: 128,
                data: [1; 128],
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 1,
                hops: vec![1, 11, 12, 21],
            },
            session_id: 1,
        }
    }

    #[test]
    fn test_generic_gragment_forward() {
        tests::generic_fragment_forward::<FlyPath>();
    }

    #[test]
    fn test_generic_fragment_drop() {
        tests::generic_fragment_drop::<FlyPath>();
    }

    //when hops[hop_index] doesen't mach the drone's own NodeId
    #[test]
    fn test_nack_unexpected_recipient() {
        let (
            mut drone,
            test_event_recv,
            test_command_send,
            test_packet_recv,
            test_packet_send,
            client_reciver,
        ) = setup_test_drone(0 as f32);
        let packet = Packet {
            pack_type: PacketType::MsgFragment(Fragment {
                fragment_index: 1,
                total_n_fragments: 1,
                length: 128,
                data: [1; 128],
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 1,
                hops: vec![3, 2, 1],
            },
            session_id: 1,
        };
        drone.packet_handler(packet);
        if let PacketType::Nack(nack) = client_reciver.try_recv().unwrap().pack_type {
            assert_eq!(nack.nack_type, NackType::UnexpectedRecipient(drone.id));
        }
    }

    //when drone is the final destination
    #[test]
    fn test_destination_is_drone() {
        let (
            mut drone,
            test_event_recv,
            test_command_send,
            test_packet_recv,
            test_packet_send,
            client_reciver,
        ) = setup_test_drone(0 as f32);
        let packet = Packet {
            pack_type: PacketType::MsgFragment(Fragment {
                fragment_index: 1,
                total_n_fragments: 1,
                length: 128,
                data: [1; 128],
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 1,
                hops: vec![3, 1],
            },
            session_id: 1,
        };
        drone.packet_handler(packet);
        if let PacketType::Nack(nack) = client_reciver.try_recv().unwrap().pack_type {
            assert_eq!(nack.nack_type, NackType::DestinationIsDrone);
        }
    }

    //when the next_hop is not a neighbor of the drone
    #[test]
    fn test_error_in_routing() {
        let (
            mut drone,
            test_event_recv,
            test_command_send,
            test_packet_recv,
            test_packet_send,
            client_reciver,
        ) = setup_test_drone(0 as f32);
        let packet = Packet {
            pack_type: PacketType::MsgFragment(Fragment {
                fragment_index: 1,
                total_n_fragments: 1,
                length: 128,
                data: [1; 128],
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 1,
                hops: vec![3, 1, 4],
            },
            session_id: 1,
        };
        drone.packet_handler(packet);
        if let PacketType::Nack(nack) = client_reciver.try_recv().unwrap().pack_type {
            assert_eq!(nack.nack_type, NackType::ErrorInRouting(4));
        }
    }

    //when a drone wants to send a message to a drone which is panicked
    // #[test]
    // fn test_forward_to_drone_panicked(){
    //     let (mut drone,
    //         test_event_recv,
    //         test_command_send,
    //         _test_packet_recv,
    //         test_packet_send,
    //         client_reciver) = setup_test_drone(0 as f32);
    //     let packet = Packet {
    //         pack_type: PacketType::MsgFragment(Fragment {
    //             fragment_index: 1,
    //             total_n_fragments: 1,
    //             length: 128,
    //             data: [1; 128],
    //         }),
    //         routing_header: SourceRoutingHeader {
    //             hop_index: 1,
    //             hops: vec![3, 1, 4],
    //         },
    //         session_id: 1,
    //     };
    //     drone.packet_handler(packet);
    // }
}
