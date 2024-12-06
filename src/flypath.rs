use crossbeam_channel::{select_biased, Receiver, Sender};
use rand::Rng;
use std::collections::HashMap;
use std::fmt;
use wg_2024::controller::DroneEvent::ControllerShortcut;
use wg_2024::controller::{DroneCommand, DroneEvent};
use wg_2024::drone::Drone;
use wg_2024::network::{NodeId, SourceRoutingHeader};
use wg_2024::packet::{FloodRequest, FloodResponse, Nack, NackType, NodeType, Packet, PacketType};

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
    pub precFloodId: HashMap<u64, u64>,
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
            precFloodId: HashMap::new(),
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
            precFloodId: HashMap::new(),
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
    fn packet_handler(&mut self, mut packet: Packet) {
        // read Drone Protocol for the precise description of steps

        match &packet.pack_type {
            PacketType::FloodRequest(req) => {
                let mut floodrequest = req.clone();
                if !self.precFloodId.contains_key(&floodrequest.flood_id) {
                    let mut no_neightbours = true;
                    self.precFloodId
                        .insert(floodrequest.flood_id, floodrequest.flood_id);
                    floodrequest.path_trace.push((self.id, NodeType::Drone));
                    let new_floodrequest= self.floodrequest_creator(&packet, floodrequest.clone());
                    for (i, j) in &self.packet_send {
                        if *i != floodrequest.path_trace[floodrequest.path_trace.len() - 2].0 {
                            self.packet_sender(new_floodrequest.clone());
                            self.controller_sender(DroneEvent::PacketSent(new_floodrequest.clone()));
                            no_neightbours = false;
                        }
                    }
                    if !no_neightbours {
                        let floodresponse = self.floodresponse_creator(&packet, &floodrequest);
                        self.packet_sender(floodresponse);
                    }
                } else {
                    floodrequest.path_trace.push((self.id, NodeType::Drone));
                    let floodresponse = self.floodresponse_creator(&packet, &floodrequest);
                    self.packet_sender(floodresponse);
                }
            }
            _ =>{
                if packet.routing_header.hops[packet.routing_header.hop_index] != self.id {
                    let nack_packet = self.nack_creator(&packet, NackType::UnexpectedRecipient(self.id));
                    self.controller_send
                        .send(ControllerShortcut(nack_packet))
                        .unwrap();
                    return;
                }

                packet.routing_header.hop_index += 1;
                let next_hop = packet.routing_header.hop_index;

                if next_hop == packet.routing_header.hops.len() {
                    let nack_packet = self.nack_creator(&packet, NackType::DestinationIsDrone);
                    self.controller_sender(ControllerShortcut(nack_packet));
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
                                self.packet_sender(nack_packet.clone());
                                self.controller_sender(DroneEvent::PacketDropped(packet.clone()));
                                self.controller_sender(DroneEvent::PacketSent(nack_packet));
                            } else {
                                self.packet_sender(packet.clone());
                                self.controller_sender(DroneEvent::PacketSent(packet));
                            }
                        }
                        _ => {
                            self.packet_sender(packet.clone());
                            self.controller_sender(DroneEvent::PacketSent(packet));
                        }
                    }
                } else {
                    let nack_packet = self.nack_creator(&packet, NackType::ErrorInRouting(packet.routing_header.hops[packet.routing_header.hop_index]));
                    self.packet_sender(nack_packet.clone());
                    self.controller_sender(DroneEvent::PacketSent(nack_packet));
                }
            }
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

    fn floodrequest_creator(&self, packet: &Packet, floodrequest: FloodRequest) -> Packet {
        Packet{
            pack_type: PacketType::FloodRequest(floodrequest),
            routing_header: SourceRoutingHeader {
                hop_index: 0,
                hops: Vec::new(),
            },
            session_id: packet.session_id,
        }
    }

    fn floodresponse_creator(&self, packet: &Packet, flood_request: &FloodRequest) -> Packet{
        let mut reverse_hops = Vec::new();
        for (i, j) in flood_request.path_trace.clone(){
            reverse_hops.push(i);
        }
        reverse_hops.reverse();
        
        let floodresponse = FloodResponse {
            flood_id: flood_request.flood_id,
            path_trace: flood_request.path_trace.clone(),
        };

        Packet{
            pack_type: PacketType::FloodResponse(floodresponse),
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
        let next_hop = packet.routing_header.hops[packet.routing_header.hop_index];
        self.packet_send
            .get(&next_hop)
            .unwrap()
            .send(packet.clone())
            .unwrap_or_else(|e| {
                println!("Error sending packet: {}", e);
                let nack_packet =
                    self.nack_creator(&packet, NackType::ErrorInRouting(next_hop.clone()));
                self.controller_send
                    .send(ControllerShortcut(nack_packet))
                    .unwrap();
            });
    }
}

#[cfg(test)]
mod tests {
    use std::thread;
    use super::*;
    use crossbeam_channel::unbounded;
    use wg_2024::packet::{Ack, Fragment};

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

    #[test]
    fn test_drone_fragment_send(){
        generic_fragment_forward::<FlyPath>();
        generic_fragment_drop::<FlyPath>();
        generic_chain_fragment_ack::<FlyPath>();
        generic_chain_fragment_drop::<FlyPath>();
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

    /// This function is used to test the packet forward functionality of a drone.
    pub fn generic_fragment_forward<T: Drone + Send + 'static>() {
        // drone 2 <Packet>
        let (d_send, d_recv) = unbounded();
        // drone 3 <Packet>
        let (d2_send, d2_recv) = unbounded::<Packet>();
        // SC commands
        let (_d_command_send, d_command_recv) = unbounded();

        let neighbours = HashMap::from([(12, d2_send.clone())]);
        let mut drone = T::new(
            11,
            unbounded().0,
            d_command_recv,
            d_recv.clone(),
            neighbours,
            0.0,
        );
        // Spawn the drone's run method in a separate thread
        thread::spawn(move || {
            drone.run();
        });

        let mut msg = create_sample_packet();

        // "Client" sends packet to d
        d_send.send(msg.clone()).unwrap();
        msg.routing_header.hop_index = 2;

        // d2 receives packet from d1
        assert_eq!(d2_recv.recv().unwrap(), msg);
    }

    /// Checks if the packet is dropped by one drone. The drone MUST have 100% packet drop rate, otherwise the test will fail sometimes.
    pub fn generic_fragment_drop<T: Drone + Send + 'static>() {
        // Client 1
        let (c_send, c_recv) = unbounded();
        // Drone 11
        let (d_send, d_recv) = unbounded();
        // SC commands
        let (_d_command_send, d_command_recv) = unbounded();

        let neighbours = HashMap::from([(12, d_send.clone()), (1, c_send.clone())]);
        let mut drone = T::new(
            11,
            unbounded().0,
            d_command_recv,
            d_recv.clone(),
            neighbours,
            1.0,
        );

        // Spawn the drone's run method in a separate thread
        thread::spawn(move || {
            drone.run();
        });

        let msg = create_sample_packet();

        // "Client" sends packet to the drone
        d_send.send(msg.clone()).unwrap();

        let dropped = Nack {
            fragment_index: 1,
            nack_type: NackType::Dropped,
        };
        let srh = SourceRoutingHeader {
            hop_index: 1,
            hops: vec![11, 1],
        };
        let nack_packet = Packet {
            pack_type: PacketType::Nack(dropped),
            routing_header: srh,
            session_id: 1,
        };

        // Client listens for packet from the drone (Dropped Nack)
        assert_eq!(c_recv.recv().unwrap(), nack_packet);
    }

    /// Checks if the packet is dropped by the second drone. The first drone must have 0% PDR and the second one 100% PDR, otherwise the test will fail sometimes.
    pub fn generic_chain_fragment_drop<T: Drone + Send + 'static>() {
        // Client 1 channels
        let (c_send, c_recv) = unbounded();
        // Server 21 channels
        let (s_send, _s_recv) = unbounded();
        // Drone 11
        let (d_send, d_recv) = unbounded();
        // Drone 12
        let (d12_send, d12_recv) = unbounded();
        // SC - needed to not make the drone crash
        let (_d_command_send, d_command_recv) = unbounded();

        // Drone 11
        let neighbours11 = HashMap::from([(12, d12_send.clone()), (1, c_send.clone())]);
        let mut drone = T::new(
            11,
            unbounded().0,
            d_command_recv.clone(),
            d_recv.clone(),
            neighbours11,
            0.0,
        );
        // Drone 12
        let neighbours12 = HashMap::from([(11, d_send.clone()), (21, s_send.clone())]);
        let mut drone2 = T::new(
            12,
            unbounded().0,
            d_command_recv.clone(),
            d12_recv.clone(),
            neighbours12,
            1.0,
        );

        // Spawn the drone's run method in a separate thread
        thread::spawn(move || {
            drone.run();
        });

        thread::spawn(move || {
            drone2.run();
        });

        let msg = create_sample_packet();

        // "Client" sends packet to the drone
        d_send.send(msg.clone()).unwrap();

        // Client receive an ACK originated from 'd'
        assert_eq!(
            c_recv.recv().unwrap(),
            Packet {
                pack_type: PacketType::Ack(Ack { fragment_index: 1 }),
                routing_header: SourceRoutingHeader {
                    hop_index: 1,
                    hops: vec![11, 1],
                },
                session_id: 1,
            }
        );

        // Client receive an NACK originated from 'd2'
        assert_eq!(
            c_recv.recv().unwrap(),
            Packet {
                pack_type: PacketType::Nack(Nack {
                    fragment_index: 1,
                    nack_type: NackType::Dropped,
                }),
                routing_header: SourceRoutingHeader {
                    hop_index: 2,
                    hops: vec![12, 11, 1],
                },
                session_id: 1,
            }
        );
    }

    /// Checks if the packet can reach its destination. Both drones must have 0% PDR, otherwise the test will fail sometimes.
    pub fn generic_chain_fragment_ack<T: Drone + Send + 'static>() {
        // Client<1> channels
        let (c_send, c_recv) = unbounded();
        // Server<21> channels
        let (s_send, s_recv) = unbounded();
        // Drone 11
        let (d_send, d_recv) = unbounded();
        // Drone 12
        let (d12_send, d12_recv) = unbounded();
        // SC - needed to not make the drone crash
        let (_d_command_send, d_command_recv) = unbounded();

        // Drone 11
        let neighbours11 = HashMap::from([(12, d12_send.clone()), (1, c_send.clone())]);
        let mut drone = T::new(
            11,
            unbounded().0,
            d_command_recv.clone(),
            d_recv.clone(),
            neighbours11,
            0.0,
        );
        // Drone 12
        let neighbours12 = HashMap::from([(11, d_send.clone()), (21, s_send.clone())]);
        let mut drone2 = T::new(
            12,
            unbounded().0,
            d_command_recv.clone(),
            d12_recv.clone(),
            neighbours12,
            0.0,
        );

        // Spawn the drone's run method in a separate thread
        thread::spawn(move || {
            drone.run();
        });

        thread::spawn(move || {
            drone2.run();
        });

        let mut msg = create_sample_packet();

        // "Client" sends packet to the drone
        d_send.send(msg.clone()).unwrap();

        // Client receive an ACK originated from 'd'
        assert_eq!(
            c_recv.recv().unwrap(),
            Packet {
                pack_type: PacketType::Ack(Ack { fragment_index: 1 }),
                routing_header: SourceRoutingHeader {
                    hop_index: 1,
                    hops: vec![11, 1],
                },
                session_id: 1,
            }
        );

        // Client receive an ACK originated from 'd2'
        assert_eq!(
            c_recv.recv().unwrap(),
            Packet {
                pack_type: PacketType::Ack(Ack { fragment_index: 1 }),
                routing_header: SourceRoutingHeader {
                    hop_index: 2,
                    hops: vec![12, 11, 1],
                },
                session_id: 1,
            }
        );

        msg.routing_header.hop_index = 3;
        // Server receives the fragment
        assert_eq!(s_recv.recv().unwrap(), msg);
    }
}
