use crossbeam_channel::{Receiver, Sender};
use std::collections::HashMap;
// use wg_2024::controller::Command;
use wg_2024::network::NodeId;
// use wg_2024::packet::{Packet, PacketType};
use wg_2024::drone::{Drone, DroneOptions};
use wg_2024::packet::Packet; // Ensure DroneOptions and Drone are correctly imported

#[derive(Debug, Clone)]
pub struct FlyPath {
    pub id: NodeId,
    // pub sim_contr_send: Sender<Command>,
    // pub sim_contr_recv: Receiver<Command>,
    pub packet_recv: Receiver<Packet>,
    pub pdr: u8,
    pub packet_send: HashMap<NodeId, Sender<Packet>>,
}

impl FlyPath {
    // Internal method to implement the logic for run
    fn run_internal(&mut self) {
        // TODO: implement the internal logic for running FlyPath
    }
}

// Implement the Drone trait for FlyPath
impl Drone for FlyPath {
    fn new(options: DroneOptions) -> Self {
        Self {
            id: options.id,
            // sim_contr_send: options.sim_contr_send,
            // sim_contr_recv: options.sim_contr_recv,
            packet_recv: options.packet_recv,
            pdr: (options.pdr * 100.0) as u8,
            packet_send: HashMap::new(),
        }
    }

    fn run(&mut self) {
        self.run_internal(); // Call the internal run method
    }
}
