use crossbeam_channel::{select, Receiver, Sender};
use std::collections::HashMap;
use std::thread;

use wg_2024::controller::Command;
use wg_2024::drone::{Drone, DroneOptions};
use wg_2024::network::NodeId;
use wg_2024::packet::{Packet, PacketType};

#[derive(Debug,Clone)]
pub struct FlyPath{
    id: NodeId,
    sim_contr_send: Sender<Command>,
    sim_contr_recv: Receiver<Command>,
    packet_recv: Receiver<Packet>,
    pdr: u8,
    packet_send: HashMap<NodeId, Sender<Packet>>,
}

impl FlyPath{
    fn run_internal(&mut self){
        //TODO: implement run
    }
}

impl Drone for FlyPath{
    fn new(options: DroneOptions) -> Self {
        Self {
            id: options.id,
            sim_contr_send: options.sim_contr_send,
            sim_contr_recv: options.sim_contr_recv,
            packet_recv: options.packet_recv,
            pdr: (options.pdr * 100.0) as u8,
            packet_send: HashMap::new(),
        }
    }

    fn run(&mut self) {
        self.run_internal();
    }
}