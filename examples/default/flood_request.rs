use crossbeam_channel::{unbounded, Receiver, Sender};
use flyPath::FlyPath;
use std::thread::{self, sleep};
use std::time::Duration;
use wg_2024::packet::{self, FloodRequest};
use wg_2024::{
    self,
    controller::{DroneCommand, DroneEvent},
    drone::Drone,
    network::SourceRoutingHeader,
    packet::Packet,
};

fn main() {
    let (
        mut d1,
        mut d2,
        d1_send,
        d2_send,
        c1_recv,
        _from_d1_event_recv,
        from_d2_event_recv,
        _d1_command_send,
        d2_command_send,
    ) = setup(0.0, 0.0);

    let _handler_d1 = thread::spawn(move || {
        d1.run();
    });

    let _handler_d2 = thread::spawn(move || {
        d2.run();
    });

    // *CASE* c1 init the flood request
    let flood_request = FloodRequest::initialize(1, 10, wg_2024::packet::NodeType::Client);
    let packet =
        Packet::new_flood_request(SourceRoutingHeader::empty_route(), 1, flood_request.clone());
    d1_send.send(packet).unwrap();

    // wait for a response
    sleep(Duration::from_secs(1));
    let packet: Result<Packet, crossbeam_channel::RecvTimeoutError> =
        c1_recv.recv_timeout(Duration::from_secs(1));

    // packet should be FloodResponse and have all the hops
    let expected = flood_request
        .get_incremented(1, packet::NodeType::Drone)
        .get_incremented(2, packet::NodeType::Drone)
        .generate_response(1);
    assert!(packet.is_ok());
    if let Ok(packet) = packet {
        assert_eq!(packet.pack_type, expected.pack_type);
    }

    // *CASE* d2 can't send FloodRequest to d1 so it must use ShortCut
    // send command to remove d1 and flood request
    d2_command_send.send(DroneCommand::RemoveSender(1)).unwrap();
    d2_send.send(expected.clone()).unwrap();

    // wait for a response
    sleep(Duration::from_secs(1));
    let event: Result<DroneEvent, crossbeam_channel::RecvTimeoutError> =
        from_d2_event_recv.recv_timeout(Duration::from_secs(1));

    // event should be Shortcut
    assert!(event.is_ok());
    if let Ok(DroneEvent::ControllerShortcut(packet)) = event {
        assert_eq!(packet.pack_type, expected.clone().pack_type);
    } else {
        panic!("not Controller Shortcut")
    }
}

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
    // c1 - d1 - d2
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
        vec![(2, d2_send.clone()), (10, c1_send.clone())]
            .into_iter()
            .collect(),
        // vec![(10, c1_send.clone())]
        //     .into_iter()
        //     .collect(),
        pdr_d1,
    );
    let d2 = FlyPath::new(
        2,
        d2_event_send,
        d2_command_recv,
        d2_recv,
        vec![(1, d1_send.clone())].into_iter().collect(),
        // Vec::new().into_iter().collect(),
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
