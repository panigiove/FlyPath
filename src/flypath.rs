use crossbeam_channel::{select_biased, Receiver, Sender};
use rand::Rng;
use std::collections::{HashMap, HashSet};
use wg_2024::controller::{DroneCommand, DroneEvent};
use wg_2024::drone::Drone;
use wg_2024::network::{NodeId, SourceRoutingHeader};
use wg_2024::packet::{Nack, NackType, NodeType, Packet, PacketType};

#[cfg(feature = "modes")]
use crate::messages::Messages;
#[cfg(feature = "modes")]
use rand::seq::SliceRandom;
#[cfg(feature = "modes")]
use std::fmt;
#[cfg(feature = "modes")]
use wg_2024::packet::{FloodRequest, Fragment};
/// Enum representing the enabled fly path modes.
#[derive(Debug, Clone)]
pub enum FlyPathModes {
    /// The drone behaves as described in the protocol file.
    Default,

    #[cfg(feature = "modes")]
    /// Custom messages based on the selected themes.
    Spicy(FlyPathThemes),

    #[cfg(feature = "modes")]
    /// The drone behaves erratically, has comptetely gone mad, and it doesn't act like it should. It's dangerous!!!!!!!
    BrainRot,
}

/// Represents various themes for fly paths, available when the `modes` feature is enabled.
#[derive(Debug, Clone)]
#[cfg(feature = "modes")]
pub enum FlyPathThemes {
    Batman,
    Rocket,
    Quackable,
    HarryPotter,
    // GerryScotty,
    DarkSouls,
    Bloodborne,
    Pingu,
}

#[cfg(feature = "modes")]
impl fmt::Display for FlyPathThemes {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let theme_str = match self {
            FlyPathThemes::Batman => "Batman",
            FlyPathThemes::Rocket => "Rocket",
            FlyPathThemes::Quackable => "Quackable",
            FlyPathThemes::HarryPotter => "HarryPotter",
            // FlyPathThemes::GerryScotty => "GerryScotty",
            FlyPathThemes::DarkSouls => "DarkSoul",
            FlyPathThemes::Bloodborne => "Bloodborne",
            FlyPathThemes::Pingu => "Pingu",
        };
        write!(f, "{}", theme_str)
    }
}

#[cfg(feature = "modes")]
const FILE_PATH: &str = "resources/messages.json";

/// Implementation of Drone
#[derive(Debug, Clone)]
pub struct FlyPath {
    pub id: NodeId,
    pub controller_send: Sender<DroneEvent>,
    pub controller_recv: Receiver<DroneCommand>,
    pub packet_recv: Receiver<Packet>,
    pub packet_send: HashMap<NodeId, Sender<Packet>>,
    pub pdr: f32,
    /// set used to memorize old floor requests
    pub precFloodId: HashSet<(u64, u8)>,

    /// drone's mode
    pub mode: FlyPathModes,
    /// struct that contains loaded messages from FILE_PATH
    #[cfg(feature = "modes")]
    pub messages: Messages,
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
            precFloodId: HashSet::new(),
            #[cfg(feature = "modes")]
            messages: Messages::load_from_file(FILE_PATH).unwrap(),
        }
    }

    fn run(&mut self) {
        loop {
            select_biased! {
                recv(self.controller_recv) -> cmd => {
                    if let Ok(cmd) = cmd {
                        match cmd {
                            DroneCommand::Crash => {
                                match &self.mode {
                                    FlyPathModes::Default => {self.gentle_crash(); break;}
                                    #[cfg(feature = "modes")]
                                    FlyPathModes::Spicy(_) => {
                                        self.command_flypath_message(&cmd);
                                        self.gentle_crash();
                                        break;
                                    }
                                    #[cfg(feature = "modes")]
                                    FlyPathModes::BrainRot => {
                                        let should_crash = 50 < rand::thread_rng().gen_range(0..100);
                                        if should_crash {
                                            self.brainRot_event_flypath_message("Crash");
                                            break;
                                        }else{
                                            self.brainRot_event_flypath_message("NoCrash");
                                            continue;
                                        }
                                    }
                                };
                            },
                            _ => self.command_handler(cmd)
                        };
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
            precFloodId: HashSet::new(),
            messages: Messages::load_from_file(FILE_PATH).unwrap(),
        }
    }

    // Manage all the remaining packet until all the senders has been disconnected
    fn gentle_crash(&mut self) {
        loop {
            match self.packet_recv.try_recv() {
                Ok(mut packet) => {
                    match &packet.pack_type {
                        PacketType::MsgFragment(_) => {
                            self.send_nack(&packet, NackType::ErrorInRouting(self.id));
                        }
                        PacketType::FloodRequest(_) => {}
                        // Ack, Nack, FloodResponse need to have be sended
                        _ => self.send_packet(&mut packet),
                    }
                }
                Err(crossbeam_channel::TryRecvError::Empty) => break,
                Err(crossbeam_channel::TryRecvError::Disconnected) => break,
            }
        }
    }

    // Given a CMD, get the message and send it
    #[cfg(feature = "modes")]
    fn command_flypath_message(&mut self, cmd: &DroneCommand) {
        let flyPath_messages = self.messages.generate_droneEvent_to_controller(
            &self.mode,
            Messages::drone_command_to_string(cmd),
            self.id,
        );
        if let Ok(Some(msg)) = flyPath_messages {
            self.send_event(msg);
        }
    }

    // Given a EVENT, get the message and send it
    #[cfg(feature = "modes")]
    fn event_flypath_message(&mut self, event: &DroneEvent) {
        let flyPath_messages = self.messages.generate_droneEvent_to_controller(
            &self.mode,
            Messages::drone_event_to_string(event),
            self.id,
        );
        if let Ok(Some(msg)) = flyPath_messages {
            self.send_event(msg);
        }
    }

    #[cfg(feature = "modes")]
    fn brainRot_event_flypath_message(&mut self, brainRot_event: &str) {
        let flyPath_messages =
            self.messages
                .generate_droneEvent_to_controller(&self.mode, brainRot_event, self.id);
        if let Ok(Some(msg)) = flyPath_messages {
            self.send_event(msg);
        }
    }

    // Handler AddSender, RemoveSender and Set packet drop rate
    fn command_handler(&mut self, cmd: DroneCommand) {
        match &cmd {
            DroneCommand::AddSender(id, sender) => {
                match &self.mode {
                    FlyPathModes::Default => {
                        self.packet_send.insert(*id, sender.clone());
                    }
                    #[cfg(feature = "modes")]
                    FlyPathModes::Spicy(_) => {
                        self.command_flypath_message(&cmd);
                        self.packet_send.insert(*id, sender.clone());
                    }
                    #[cfg(feature = "modes")]
                    FlyPathModes::BrainRot => {
                        // 50/50 add the sender
                        let should_add_sender = 50 > rand::thread_rng().gen_range(0..100);
                        if should_add_sender {
                            self.brainRot_event_flypath_message("AddSender");
                            self.packet_send.insert(*id, sender.clone());
                        } else {
                            self.brainRot_event_flypath_message("NoAddSender");
                        }
                    }
                };
            }
            // TODO: check if really drop the channel testing that returns Err Disconnected
            DroneCommand::RemoveSender(id) => {
                match &self.mode {
                    FlyPathModes::Default => {
                        self.packet_send.remove(id);
                    }
                    #[cfg(feature = "modes")]
                    FlyPathModes::Spicy(_) => {
                        self.command_flypath_message(&cmd);
                        self.packet_send.remove(id);
                    }
                    #[cfg(feature = "modes")]
                    FlyPathModes::BrainRot => {
                        // 50/50 remove the sender
                        let should_remove_sender = 50 > rand::thread_rng().gen_range(0..100);
                        if should_remove_sender {
                            self.brainRot_event_flypath_message("RemoveSender");
                            self.packet_send.remove(id);
                        } else {
                            self.brainRot_event_flypath_message("NoRemoveSender");
                        }
                    }
                };
            }
            DroneCommand::SetPacketDropRate(pdr) => {
                match &self.mode {
                    FlyPathModes::Default => {
                        self.pdr = *pdr;
                    }
                    #[cfg(feature = "modes")]
                    FlyPathModes::Spicy(_) => {
                        self.command_flypath_message(&cmd);
                        self.pdr = *pdr;
                    }
                    #[cfg(feature = "modes")]
                    FlyPathModes::BrainRot => {
                        // 50/50 change pdr
                        let should_change_pdr = 50 > rand::thread_rng().gen_range(0..100);
                        if should_change_pdr {
                            self.brainRot_event_flypath_message("SetPacketDropRate");
                            self.pdr = *pdr;
                        } else {
                            self.brainRot_event_flypath_message("NoSetPacketDropRate");
                        }
                    }
                };
            }
            _ => {}
        }
    }

    // Manage FloodRequest, if not FloodRequest: check the packat , drop it in case, send the packet
    fn packet_handler(&mut self, mut packet: Packet) {
        match &mut packet.pack_type {
            PacketType::FloodRequest(flood_request) => {
                let mut updated_flood_request =
                        flood_request.get_incremented(self.id, NodeType::Drone);

                #[cfg(feature = "modes")]
                if let FlyPathModes::BrainRot = self.mode {
                    self.maybe_invalidate_floodRequest(&mut updated_flood_request);
                }

                if let Some((last_nodeId, _)) = flood_request.path_trace.last() {
                    

                    if !self.precFloodId.contains(&(
                        updated_flood_request.flood_id,
                        updated_flood_request.initiator_id,
                    )) && self.packet_send.len() > 1
                    {
                        self.precFloodId.insert((
                            updated_flood_request.flood_id,
                            updated_flood_request.initiator_id,
                        ));
                        let packet_to_send = Packet {
                            routing_header: packet.routing_header.clone(),
                            session_id: packet.session_id,
                            pack_type: PacketType::FloodRequest(updated_flood_request.clone()),
                        };
                        for (node_id, sender) in &self.packet_send {
                            if node_id != last_nodeId {
                                let _ = sender.send(packet_to_send.clone());
                            }
                        }

                        return;
                    } 
                } 
                
                let mut response = updated_flood_request.generate_response(packet.session_id);
                if response.routing_header.hops.last() != Some (&updated_flood_request.initiator_id){
                    response.routing_header.hops.push(updated_flood_request.initiator_id);
                }
                self.send_packet(&mut response);
            }
            _ => {
                if packet.routing_header.current_hop().is_some() {
                    if let Some(nack) = self.validate_packet(&packet) {
                        self.send_nack(&packet, nack);
                        return;
                    }

                    #[cfg(feature = "modes")]
                    if let FlyPathModes::BrainRot = self.mode {
                        self.pdr = rand::thread_rng().gen_range(0.0..1.0);
                    }

                    if let PacketType::MsgFragment(_) = &packet.pack_type {
                        let should_drop_packet = self.pdr > rand::thread_rng().gen_range(0.0..1.0);
                        if should_drop_packet {
                            // Drop the fragment and send Dropped Event
                            self.send_nack(&packet, NackType::Dropped);
                            return;
                        }
                    }
                    // The packet is a `Nack`, `Ack`, `FloodResponse` or a non dropped `MsgFragment`
                    self.send_packet(&mut packet);
                }
            }
        }
    }

    // if is not valid packet returns the errors, possible NackType are:
    // - UnexpectedRecipient
    // - ErrorInRouting
    // - DestinationIsDrone
    fn validate_packet(&self, packet: &Packet) -> Option<NackType> {
        let current_hop = packet.routing_header.current_hop();
        if let Some(current_hop) = current_hop {
            if current_hop != self.id {
                return Some(NackType::UnexpectedRecipient(self.id));
            } else {
                match packet.routing_header.next_hop() {
                    Some(next_hop) => {
                        if !self.packet_send.contains_key(&next_hop) {
                            // Next hop is not a neighbour
                            return Some(NackType::ErrorInRouting(next_hop));
                        }
                    }
                    // No next hop
                    None => return Some(NackType::DestinationIsDrone),
                }
            }
        }
        None
    }

    // *Increment the hop_index* and if all is ok send message
    // Send `Nack` if there is no next hop or next hop sender
    fn send_packet(&mut self, packet: &mut Packet) {
        packet.routing_header.increase_hop_index();
        if let Some(next_hop) = packet.routing_header.current_hop() {
            #[cfg(feature = "modes")]
            if let FlyPathModes::BrainRot = &self.mode {
                // Change path or invalidate the content
                let mut rng = rand::thread_rng();

                let should_change_path_or_invalidate_content = 50 > rng.gen_range(0..100);
                if should_change_path_or_invalidate_content {
                    self.change_hops_randomly(&mut packet.routing_header);
                    self.brainRot_event_flypath_message("PacketSent");
                } else if let PacketType::MsgFragment(frag) = &mut packet.pack_type {
                    self.invalidate_packet_content(frag);
                    self.brainRot_event_flypath_message("PacketSentPANIC");
                }

                if let Some(sender) = self.packet_send.get_mut(&next_hop) {
                    if sender.send(packet.clone()).is_err() {
                        packet.routing_header.decrease_hop_index();
                        self.send_nack(packet, NackType::ErrorInRouting(next_hop));
                    } else if let PacketType::MsgFragment(_) = packet.pack_type {
                        let event = DroneEvent::PacketSent(packet.clone());
                        self.send_event(event);
                    }
                }
                return;
            }

            if let Some(sender) = self.packet_send.get_mut(&next_hop) {
                if sender.send(packet.clone()).is_err() {
                    self.packet_send.remove(&next_hop);
                    packet.routing_header.decrease_hop_index();
                    self.send_nack(packet, NackType::ErrorInRouting(next_hop));
                } else if let PacketType::MsgFragment(_) = packet.pack_type {
                    let event = DroneEvent::PacketSent(packet.clone());

                    #[cfg(feature = "modes")]
                    self.event_flypath_message(&event);

                    self.send_event(event);
                }
            } else {
                packet.routing_header.decrease_hop_index();
                self.send_nack(packet, NackType::ErrorInRouting(next_hop));
            }
        }
    }

    // Create and send a nack if the packet is a MsgFragment and send the event to `Controller` otherwise forward to Controller to send to destination
    // Panic if controller is not reacheable
    fn send_nack(&mut self, packet: &Packet, nack_type: NackType) {
        if let PacketType::Nack(nack) = &packet.pack_type {
            if let NackType::DestinationIsDrone = nack.nack_type {
                return;
            }
        }
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

                // Behavior depending on the mode
                match &self.mode {
                    FlyPathModes::Default => {
                        self.send_event(DroneEvent::PacketDropped(packet.clone()))
                    }
                    #[cfg(feature = "modes")]
                    FlyPathModes::Spicy(_theme) => {
                        let event = DroneEvent::PacketDropped(packet.clone());
                        self.event_flypath_message(&event);
                        self.send_event(event);
                    }
                    #[cfg(feature = "modes")]
                    FlyPathModes::BrainRot => {
                        let event = DroneEvent::PacketDropped(packet.clone());
                        self.brainRot_event_flypath_message("PacketDropped");
                        self.send_event(event);
                    }
                }
            }
            _ => {
                // If an error occurs on a FloodResponse, Ack, Nack send to the client/server throw ControllerShortcut
                if let Err(e) = self
                    .controller_send
                    .send(DroneEvent::ControllerShortcut(packet.clone()))
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
        reverse_hops.truncate(packet.routing_header.hop_index);
        reverse_hops.insert(packet.routing_header.hop_index, self.id);
        reverse_hops.reverse();
        reverse_hops
    }

    #[cfg(feature = "modes")]
    fn change_hops_randomly(&self, routing_header: &mut SourceRoutingHeader) {
        let hop_index = routing_header.hop_index;
        // Ensure we have enough hops to modify
        if hop_index + 1 < routing_header.hops.len() {
            // Extract the portion of hops to be randomized
            let remaining_hops = &mut routing_header.hops[(hop_index + 1)..];

            // Shuffle the remaining hops
            remaining_hops.shuffle(&mut rand::thread_rng());
        }
    }

    #[cfg(feature = "modes")]
    fn invalidate_packet_content(&mut self, fragment: &mut Fragment) {
        fragment.data.shuffle(&mut rand::thread_rng());
    }

    #[cfg(feature = "modes")]
    fn maybe_invalidate_floodRequest(&mut self, fReq: &mut FloodRequest) {
        let mut rng = rand::thread_rng();

        let should_change_initialator_or_path_trace = 50 > rng.gen_range(0..100);
        if should_change_initialator_or_path_trace {
            self.brainRot_event_flypath_message("ChangeInitiatorId");
            fReq.initiator_id = rng.gen_range(0..31);
        } else {
            self.brainRot_event_flypath_message("ShufflePathTrace");
            fReq.path_trace.shuffle(&mut rng);
        }
    }
}

mod flypath_test;