use crossbeam_channel::{select_biased, Receiver, Sender};
use rand::Rng;
use std::collections::{HashMap, HashSet};
use wg_2024::controller::DroneEvent::ControllerShortcut;
use wg_2024::controller::{DroneCommand, DroneEvent};
use wg_2024::drone::Drone;
use wg_2024::network::{NodeId, SourceRoutingHeader};
use wg_2024::packet::NackType::ErrorInRouting;
use wg_2024::packet::{Nack, NackType, NodeType, Packet, PacketType};

#[cfg(not(feature = "modes"))]
use wg_2024::packet::NackType::Dropped;

#[cfg(feature = "modes")]
use crate::messages::Messages;
#[cfg(feature = "modes")]
use std::fmt;

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

#[derive(Debug, Clone)]
pub struct FlyPath {
    pub id: NodeId,
    // send to controller the DroneEvent
    pub controller_send: Sender<DroneEvent>,
    // receive from the controller a command
    pub controller_recv: Receiver<DroneCommand>,
    // receive a packet from a connected drone
    pub packet_recv: Receiver<Packet>,
    // hashmap that contains Sender to connected drones
    pub packet_send: HashMap<NodeId, Sender<Packet>>,
    // packet drop rate
    pub pdr: f32,
    // set used to memorize old floor requests
    pub precFloodId: HashSet<(u64, u8)>,
    // drone's mode, optional
    pub mode: FlyPathModes,
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
                                #[cfg(not(feature = "modes"))]
                                {
                                    self.gentle_crash();
                                    break;
                                }

                                #[cfg(feature = "modes")]
                                {
                                    let flyPath_messages = self.messages.generate_droneEvent_to_controller(
                                        &self.mode,
                                        Messages::drone_command_to_string(&cmd),
                                        self.id,
                                    );
                                    match &self.mode {
                                        FlyPathModes::Default => {
                                            self.gentle_crash();
                                            break;
                                        }
                                        FlyPathModes::Spicy(_) => {
                                            if let Ok(Some(msg)) = flyPath_messages {
                                                self.send_event(msg);
                                            }
                                            self.gentle_crash();
                                            break;
                                        }
                                        FlyPathModes::BrainRot => {}
                                    }
                                }
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

    fn gentle_crash(&mut self) {
        loop {
            match self.packet_recv.try_recv() {
                Ok(mut packet) => {
                    match &packet.pack_type {
                        PacketType::MsgFragment(_) => {
                            self.send_nack(&packet, ErrorInRouting(self.id));
                        }
                        PacketType::FloodRequest(_) => {}
                        // Ack, Nack, FloodResponse need to have be sended
                        _ => self.send_packet(&mut packet),
                    }
                }
                Err(crossbeam_channel::TryRecvError::Empty) => continue,
                Err(crossbeam_channel::TryRecvError::Disconnected) => break,
            }
        }
    }

    fn command_handler(&mut self, cmd: DroneCommand) {
        #[cfg(feature = "modes")]
        let flyPath_messages = self.messages.generate_droneEvent_to_controller(
            &self.mode,
            Messages::drone_command_to_string(&cmd),
            self.id,
        );

        match cmd {
            DroneCommand::AddSender(id, sender) => {
                #[cfg(not(feature = "modes"))]
                self.packet_send.insert(id, sender);

                // Behavior depending on the mode
                #[cfg(feature = "modes")]
                match &self.mode {
                    FlyPathModes::Default => {
                        let _ = self.packet_send.insert(id, sender);
                    }
                    FlyPathModes::Spicy(_) => {
                        let _ = self.packet_send.insert(id, sender);
                        if let Ok(Some(msg)) = flyPath_messages {
                            self.send_event(msg);
                        }
                    }
                    FlyPathModes::BrainRot => {}
                }
            }
            // TODO: check if really drop the channel testing that returns Err Disconnected
            DroneCommand::RemoveSender(id) => {
                #[cfg(not(feature = "modes"))]
                self.packet_send.remove(&id);

                #[cfg(feature = "modes")]
                match &self.mode {
                    FlyPathModes::Default => {
                        let _ = self.packet_send.remove(&id);
                    }
                    FlyPathModes::Spicy(_) => {
                        let _ = self.packet_send.remove(&id);
                        if let Ok(Some(msg)) = flyPath_messages {
                            self.send_event(msg);
                        }
                    }
                    FlyPathModes::BrainRot => {}
                }
            }
            DroneCommand::SetPacketDropRate(pdr) => {
                #[cfg(not(feature = "modes"))]
                {
                    self.pdr = pdr;
                }

                #[cfg(feature = "modes")]
                match &self.mode {
                    FlyPathModes::Default => {
                        self.pdr = pdr;
                    }
                    FlyPathModes::Spicy(_) => {
                        self.pdr = pdr;
                        if let Ok(Some(msg)) = flyPath_messages {
                            self.send_event(msg);
                        }
                    }
                    FlyPathModes::BrainRot => {}
                }
            }
            _ => {}
        }
    }

    fn packet_handler(&mut self, mut packet: Packet) {
        match &packet.pack_type {
            PacketType::FloodRequest(flood_request) => {
                if let Some((last_nodeId, _)) = flood_request.path_trace.last() {
                    let updated_flood_request =
                        flood_request.get_incremented(self.id, NodeType::Drone);
                    if !self.precFloodId.contains(&(
                        updated_flood_request.flood_id,
                        updated_flood_request.initiator_id,
                    )) && self.packet_send.len() > 1
                    {
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
                    } else {
                        let mut response =
                            updated_flood_request.generate_response(packet.session_id);
                        self.send_packet(&mut response);
                    }
                }
            }
            _ => {
                if let Some(nack) = self.validate_packet(&packet) {
                    self.send_nack(&packet, nack);
                    return;
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
                    return Some(NackType::ErrorInRouting(next_hop));
                }
            }
            // No next hop
            None => return Some(NackType::DestinationIsDrone),
        }
        // Valid Packet
        None
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
                // If sending fails, send a NACK indicating an error in routing
                self.packet_send.remove(&next_hop);

                // Decrease hop index cus the packet is not sended
                packet.routing_header.decrease_hop_index();
                self.send_nack(packet, NackType::ErrorInRouting(next_hop));
            } else if let PacketType::MsgFragment(_) = packet.pack_type {
                // `PacketSent` to `Controller` if the successfull is a MsgFragment
                #[cfg(not(feature = "modes"))]
                self.send_event(DroneEvent::PacketSent(packet.clone()));
                // Behavior depending on the mode
                #[cfg(feature = "modes")]
                match &self.mode {
                    FlyPathModes::Default => {
                        self.send_event(DroneEvent::PacketSent(packet.clone()))
                    }
                    FlyPathModes::Spicy(_theme) => {
                        self.send_event(DroneEvent::PacketSent(packet.clone()));
                        let flyPath_messages = self.messages.generate_droneEvent_to_controller(
                            &self.mode,
                            Messages::drone_event_to_string(&DroneEvent::PacketSent(
                                packet.clone(),
                            )),
                            self.id,
                        );
                        if let Ok(Some(msg)) = flyPath_messages {
                            self.send_event(msg);
                        }
                    }
                    FlyPathModes::BrainRot => {}
                }
            }
        } else {
            // If no sender is found for the next hop, send a NACK
            packet.routing_header.decrease_hop_index();
            self.send_nack(packet, NackType::ErrorInRouting(next_hop));
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
                #[cfg(not(feature = "modes"))]
                if nack_type == Dropped {
                    self.send_event(DroneEvent::PacketDropped(packet.clone()));
                }
                // Behavior depending on the mode
                #[cfg(feature = "modes")]
                match &self.mode {
                    FlyPathModes::Default => {
                        self.send_event(DroneEvent::PacketDropped(packet.clone()))
                    }
                    FlyPathModes::Spicy(_theme) => {
                        self.send_event(DroneEvent::PacketDropped(packet.clone()));
                        let flyPath_messages = self.messages.generate_droneEvent_to_controller(
                            &self.mode,
                            Messages::drone_event_to_string(&DroneEvent::PacketDropped(
                                packet.clone(),
                            )),
                            self.id,
                        );
                        if let Ok(Some(msg)) = flyPath_messages {
                            self.send_event(msg);
                        }
                    }
                    FlyPathModes::BrainRot => {}
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
    use std::thread::{self, sleep};
    use std::time::Duration;

    use super::*;
    use crossbeam_channel::unbounded;
    use wg_2024::packet::{Ack, FloodRequest, Fragment};
    use wg_2024::tests;

    fn setup(
        pdr_d1: f32,
        pdr_d2: f32,
    ) -> (
        FlyPath,
        FlyPath,
        Sender<Packet>,
        Sender<Packet>,
        Receiver<Packet>,
        Receiver<DroneEvent>,
        Receiver<DroneEvent>,
        Sender<DroneCommand>,
        Sender<DroneCommand>,
    ) {
        // c1 - d1       d2
        let (d1_send, d1_recv) = unbounded();
        let (d2_send, d2_recv) = unbounded();
        let (c1_send, c1_recv) = unbounded();

        let (d1_event_send, from_d1_event_recv) = unbounded();
        let (d2_event_send, from_d2_event_recv) = unbounded();

        let (d1_command_send, d1_command_recv) = unbounded();
        let (d2_command_send, d2_command_recv) = unbounded();

        let d1 = FlyPath::new(
            1,
            d1_event_send,
            d1_command_recv,
            d1_recv,
            // vec![(2, d2.send.clone()), (10, c1_send.clone())]
            //     .into_iter()
            //     .collect(),
            vec![(10, c1_send.clone())].into_iter().collect(),
            pdr_d1,
        );
        let d2 = FlyPath::new(
            2,
            d2_event_send,
            d2_command_recv,
            d2_recv,
            // vec![(1, d1_send.clone())].into_iter().collect(),
            Vec::new().into_iter().collect(),
            pdr_d2,
        );

        (
            d1,
            d2,
            d1_send,
            d2_send,
            c1_recv,
            from_d1_event_recv,
            from_d2_event_recv,
            d1_command_send,
            d2_command_send,
        )
    }

    #[test]
    fn test_default_command_add_sender() {
        let (
            mut d1,
            mut d2,
            d1_send,
            d2_send,
            c1_recv,
            from_d1_event_recv,
            _from_d2_event_recv,
            d1_command_send,
            d2_command_send,
        ) = setup(0.0, 0.0);

        let _handler_d1 = thread::spawn(move || {
            d1.run();
        });

        let _handler_d2 = thread::spawn(move || {
            d2.run();
        });

        // adding connection between d1 and d2
        d2_command_send
            .send(DroneCommand::AddSender(1, d1_send.clone()))
            .unwrap();
        d1_command_send
            .send(DroneCommand::AddSender(2, d2_send.clone()))
            .unwrap();

        let packet_from_c1_to_d2 = Packet::new_fragment(
            SourceRoutingHeader::with_first_hop(vec![10, 1, 2]),
            1,
            Fragment::from_string(1, 1, "heyy".to_string()),
        );
        d1_send.send(packet_from_c1_to_d2.clone()).unwrap();

        // wait for a response
        sleep(Duration::from_secs(1));
        let packet = c1_recv.recv_timeout(Duration::from_secs(1));

        // packet should be Nack DestinationIsDrone
        assert!(packet.is_ok());
        if let Ok(packet) = packet {
            assert_eq!(
                packet.pack_type,
                PacketType::Nack(wg_2024::packet::Nack {
                    fragment_index: 1,
                    nack_type: wg_2024::packet::NackType::DestinationIsDrone
                })
            );
        }

        // clear controller
        let event = from_d1_event_recv.recv_timeout(Duration::from_secs(1));
        assert!(event.is_ok());
        assert!(from_d1_event_recv.is_empty());
    }

    #[test]
    fn test_default_command_set_dpr() {
        let (d1_send, d1_recv) = unbounded();
        let (d2_send, d2_recv) = unbounded();
        let (c1_send, c1_recv) = unbounded();

        let (d1_event_send, from_d1_event_recv) = unbounded();
        let (d2_event_send, _from_d2_event_recv) = unbounded();

        let (d1_command_send, d1_command_recv) = unbounded();
        let (_d2_command_send, d2_command_recv) = unbounded();

        let mut d1 = FlyPath::new(
            1,
            d1_event_send,
            d1_command_recv,
            d1_recv,
            vec![(2, d2_send.clone()), (10, c1_send.clone())]
                .into_iter()
                .collect(),
            // vec![(10, c1_send.clone())]
            //     .into_iter()
            //     .collect(),
            0.0,
        );
        let mut d2 = FlyPath::new(
            2,
            d2_event_send,
            d2_command_recv,
            d2_recv,
            vec![(1, d1_send.clone())].into_iter().collect(),
            // Vec::new().into_iter().collect(),
            0.0,
        );

        thread::spawn(move || {
            d1.run();
        });

        thread::spawn(move || {
            d2.run();
        });

        // send packet
        let packet_from_c1_to_d2 = Packet::new_fragment(
            SourceRoutingHeader::with_first_hop(vec![10, 1, 2]),
            1,
            Fragment::from_string(1, 1, "heyy".to_string()),
        );
        d1_command_send
            .send(DroneCommand::SetPacketDropRate(1.0))
            .unwrap();
        d1_send.send(packet_from_c1_to_d2.clone()).unwrap();

        // wait for a response
        sleep(Duration::from_secs(1));
        let packet = c1_recv.recv_timeout(Duration::from_secs(1));

        // packet shoul be Nack Dropped
        assert!(packet.is_ok());
        if let Ok(packet) = packet {
            assert_eq!(
                packet.pack_type,
                PacketType::Nack(wg_2024::packet::Nack {
                    fragment_index: 1,
                    nack_type: wg_2024::packet::NackType::Dropped
                })
            );
        }

        // wait for a event
        let event = from_d1_event_recv.recv_timeout(Duration::from_secs(1));
        assert!(event.is_ok());
        if let Ok(event) = event {
            assert_eq!(
                event,
                DroneEvent::PacketDropped(packet_from_c1_to_d2.clone())
            );
        }
        assert!(from_d1_event_recv.is_empty());
    }

    #[test]
    fn test_default_command_remove_crash() {
        let (d1_send, d1_recv) = unbounded();
        let (d2_send, d2_recv) = unbounded();
        let (c1_send, c1_recv) = unbounded();

        let (d1_event_send, _from_d1_event_recv) = unbounded();
        let (d2_event_send, _from_d2_event_recv) = unbounded();

        let (d1_command_send, d1_command_recv) = unbounded();
        let (d2_command_send, d2_command_recv) = unbounded();

        let mut d1 = FlyPath::new(
            1,
            d1_event_send,
            d1_command_recv,
            d1_recv,
            vec![(2, d2_send.clone()), (10, c1_send.clone())]
                .into_iter()
                .collect(),
            // vec![(10, c1_send.clone())]
            //     .into_iter()
            //     .collect(),
            0.0,
        );
        let mut d2 = FlyPath::new(
            2,
            d2_event_send,
            d2_command_recv,
            d2_recv,
            vec![(1, d1_send.clone())].into_iter().collect(),
            // Vec::new().into_iter().collect(),
            0.0,
        );

        let _handler_d1 = thread::spawn(move || {
            d1.run();
        });

        let handler_d2 = thread::spawn(move || {
            d2.run();
        });

        let packet_from_c1_to_d2 = Packet::new_fragment(
            SourceRoutingHeader::with_first_hop(vec![10, 1, 2]),
            1,
            Fragment::from_string(1, 1, "heyy".to_string()),
        );

        // Remove drone 2 from drone 1
        d1_command_send.send(DroneCommand::RemoveSender(2)).unwrap();
        d1_send.send(packet_from_c1_to_d2.clone()).unwrap();

        // wait for a response
        sleep(Duration::from_secs(1));
        let packet = c1_recv.recv_timeout(Duration::from_secs(1));

        // packet should be Nack DestinationIsDrone
        assert!(packet.is_ok());
        if let Ok(packet) = packet {
            assert_eq!(
                packet.pack_type,
                PacketType::Nack(wg_2024::packet::Nack {
                    fragment_index: 1,
                    nack_type: wg_2024::packet::NackType::ErrorInRouting(2)
                })
            );
        }

        // make d2 crash and send a packet
        let ack_from_d2_to_c1 = Packet::new_ack(
            SourceRoutingHeader::with_first_hop(vec![22, 2, 1, 10]),
            2,
            2,
        );
        d2_command_send.send(DroneCommand::Crash).unwrap();
        d2_send.send(ack_from_d2_to_c1.clone()).unwrap();

        // wait for a response
        sleep(Duration::from_secs(1));
        let packet = c1_recv.recv_timeout(Duration::from_secs(1));

        // packet should be equal to ack_from_d2_to_c1
        assert!(packet.is_ok());
        if let Ok(packet) = packet {
            assert_eq!(packet.pack_type, ack_from_d2_to_c1.pack_type);
        }

        drop(d2_send);
        print!("{:?}", handler_d2.join());
    }

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

    fn setup_test_drone_no_drone_neighbour(
        pdr: f32,
    ) -> (
        FlyPath,
        Receiver<DroneEvent>,
        Sender<DroneCommand>,
        Sender<Packet>,
        Receiver<Packet>,
    ) {
        let (drone_event_send, test_event_recv) = unbounded();
        let (test_command_send, drone_command_recv) = unbounded();
        let (test_packet_send, drone_packet_recv) = unbounded();
        let (client_sender, client_reciver) = unbounded();
        (
            FlyPath::new(
                1,
                drone_event_send,
                drone_command_recv,
                drone_packet_recv,
                vec![(3, client_sender)].into_iter().collect(),
                pdr,
            ),
            test_event_recv,
            test_command_send,
            test_packet_send,
            client_reciver,
        )
    }

    #[test]
    fn test_generic_gragment_forward() {
        tests::generic_fragment_forward::<FlyPath>();
    }

    #[test]
    fn test_generic_fragment_drop() {
        tests::generic_fragment_drop::<FlyPath>();
    }

    // when hops[hop_index] doesen't mach the drone's own NodeId
    #[test]
    fn test_nack_unexpected_recipient() {
        let (
            mut drone,
            _test_event_recv,
            _test_command_send,
            _test_packet_recv,
            _test_packet_send,
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

    // when drone is the final destination
    #[test]
    fn test_destination_is_drone() {
        let (
            mut drone,
            _test_event_recv,
            _test_command_send,
            _test_packet_recv,
            _test_packet_send,
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

    // when the next_hop is not a neighbor of the drone
    #[test]
    fn test_error_in_routing() {
        let (
            mut drone,
            _test_event_recv,
            _test_command_send,
            _test_packet_recv,
            _test_packet_send,
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

    // when a drone wants to send a message to a drone which is panicked
    #[test]
    fn test_forward_to_drone_panicked() {
        let (
            mut drone,
            _test_event_recv,
            _test_command_send,
            test_packet_recv,
            _test_packet_send,
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
                hops: vec![3, 1, 2],
            },
            session_id: 1,
        };
        drop(test_packet_recv);
        drone.packet_handler(packet);

        if let PacketType::Nack(nack) = client_reciver.try_recv().unwrap().pack_type {
            assert_eq!(nack.nack_type, NackType::ErrorInRouting(2));
        }
    }

    //checks if the drone sends correctly the FloodRequest
    #[test]
    fn test_FloodRequest_send_to_neighbour() {
        let (
            mut drone,
            _test_event_recv,
            _test_command_send,
            test_packet_recv,
            _test_packet_send,
            _client_reciver,
        ) = setup_test_drone(0 as f32);
        let packet = Packet::new_flood_request(
            SourceRoutingHeader::initialize(Vec::new()),
            1,
            FloodRequest::initialize(1, 3, NodeType::Client),
        );

        drone.packet_handler(packet.clone());

        if let PacketType::FloodRequest(flood_request) = &packet.pack_type {
            let tmp = flood_request.get_incremented(1, NodeType::Drone);
            let expected = Packet::new_flood_request(
                packet.routing_header.clone(),
                packet.session_id.clone(),
                tmp,
            );
            let result = test_packet_recv.recv().unwrap();
            assert_eq!(expected, result);
        }
    }

    //checks if the response is sent correctly
    #[test]
    fn test_FloodResponse() {
        let (mut drone, _test_event_recv, _test_command_send, _test_packet_send, client_reciver) =
            setup_test_drone_no_drone_neighbour(0 as f32);
        let mut packet = Packet::new_flood_request(
            SourceRoutingHeader::initialize(Vec::new()),
            1,
            FloodRequest::initialize(1, 3, NodeType::Client),
        );

        drone.packet_handler(packet.clone());

        if let PacketType::FloodRequest(flood_request) = &mut packet.pack_type {
            flood_request.increment(1, NodeType::Drone);
            let mut client_expection = flood_request.generate_response(packet.session_id.clone());
            client_expection.routing_header.increase_hop_index();
            assert_eq!(client_expection, client_reciver.recv().unwrap());
        }
    }

    //when a drone recives a DroneCommand Crash from the controller
    #[test]
    fn test_drone_command_crash() {
        let (
            mut drone,
            _test_event_recv,
            test_command_send,
            _test_packet_recv,
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
        let _drone_thread = thread::spawn(move || {
            drone.run();
        });
        test_command_send.send(DroneCommand::Crash).unwrap();
        test_packet_send.send(packet).unwrap();

        std::thread::sleep(std::time::Duration::from_millis(100));

        if let Ok(response) = client_reciver.try_recv() {
            if let PacketType::Nack(nack) = response.pack_type {
                assert_eq!(nack.nack_type, NackType::ErrorInRouting(1));
            } else {
                panic!("Expected a NACK packet, but received another packet type.");
            }
        } else {
            panic!("No packet received from the drone.");
        }
    }

    #[test]
    fn test_drone_crash_behavior() {
        use std::thread;
        use std::time::Duration;

        // Imposta un drone di test con canali di comunicazione
        let (
            mut drone,
            test_event_recv,
            test_command_send,
            test_packet_recv,
            test_packet_send,
            client_reciver,
        ) = setup_test_drone(0.0); // Nessuna probabilità di perdita per testare solo la logica del crash

        // Invia pacchetti alla coda del drone
        let flood_request_packet = Packet {
            pack_type: PacketType::FloodRequest(FloodRequest {
                flood_id: 1,
                initiator_id: 3,
                path_trace: vec![(3, NodeType::Client)],
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 0,
                hops: Vec::new(),
            },
            session_id: 1,
        };

        let ack_packet = Packet {
            pack_type: PacketType::Ack(Ack { fragment_index: 1 }),
            routing_header: SourceRoutingHeader {
                hop_index: 1,
                hops: vec![2, 1, 3],
            },
            session_id: 2,
        };

        // Invia il comando di crash
        test_command_send.send(DroneCommand::Crash).unwrap();

        // Invia pacchetti al drone
        test_packet_send.send(flood_request_packet).unwrap();
        test_packet_send.send(ack_packet.clone()).unwrap();

        // Avvia il drone in un thread separato
        let _handle = thread::spawn(move || drone.run());

        // Attendi per dare tempo al drone di processare i pacchetti
        thread::sleep(Duration::from_secs(1));

        // Controlla che i pacchetti siano stati gestiti correttamente
        // Ack e Nack (dall'invalid_packet) dovrebbero essere stati inoltrati
        if let PacketType::Ack(ack) = client_reciver.try_recv().unwrap().pack_type {
            assert_eq!(Ack { fragment_index: 1 }, ack);
        }

        // Verifica che lo stato di crash abbia svuotato i messaggi
        assert!(test_event_recv.is_empty());
        assert!(test_packet_recv.is_empty());

        // Aspetta il termine del thread del drone
    }

    //TODO: test di corretto invio di controller shorcut

    #[test]
    fn test_controller_shortcut() {
        let (drone_event_send, _test_event_recv) = unbounded();
        let (_test_command_send, drone_command_recv) = unbounded();
        let (test_packet_send, drone_packet_recv) = unbounded();
        let (drone_packet_send, _test_packet_recv) = unbounded();
        let (client_sender, _client_reciver) = unbounded();
        let mut drone = FlyPath::new(
            1,
            drone_event_send,
            drone_command_recv,
            drone_packet_recv,
            vec![(2, drone_packet_send), (3, client_sender)]
                .into_iter()
                .collect(),
            0.0,
        );

        // dovrebbbe generare un nack con ErrorInRouting(4)
        let ack_packet = Packet {
            pack_type: PacketType::Ack(Ack { fragment_index: 1 }),
            routing_header: SourceRoutingHeader {
                hop_index: 1,
                hops: vec![2, 1, 4],
            },
            session_id: 2,
        };

        thread::spawn(move || drone.run());

        test_packet_send.send(ack_packet).unwrap();

        // TODO: da rifare
        // if let Ok(packet) = test_event_recv.try_recv() {
        //     if let PacketType::Nack(nack) = packet {
        //         assert_eq!(nack.nack_type, NackType::ErrorInRouting(1));
        //     }
        // }
    }

    //TODO: test per flooding
}
