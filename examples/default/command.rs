use crossbeam_channel::{unbounded, Receiver, Sender};
use flyPath::FlyPath;
use std::thread::{self, sleep};
use std::time::Duration;
use wg_2024::{
    self,
    controller::{DroneCommand, DroneEvent},
    drone::Drone,
    network::SourceRoutingHeader,
    packet::{Fragment, Packet, PacketType},
};

fn main() {
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

    let handler_d2 = thread::spawn(move || {
        d2.run();
    });

    // <-------------------------------------------------------------------------------->
    // *CASE*: ADD SENDER CASES
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

    // <-------------------------------------------------------------------------------->
    // *CASE*: CHANGE PDR FROM 0 TO 1
    // send packet send
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

    // <-------------------------------------------------------------------------------->
    // *CASE*: REMOVE SENDER
    // send command and packet
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

    // <-------------------------------------------------------------------------------->
    // *CASE*: CRASH, sending an ack from d2 to c1
    let ack_from_d2_to_c1 = Packet::new_ack(
        SourceRoutingHeader::with_first_hop(vec![22, 2, 1, 10]),
        2,
        2,
    );
    d2_command_send.send(DroneCommand::Crash).unwrap();
    d2_send.send(ack_from_d2_to_c1.clone()).unwrap();

    drop(d2_send);

    // wait for a response
    sleep(Duration::from_secs(1));
    let packet = c1_recv.recv_timeout(Duration::from_secs(1));

    // packet should be equal to ack_from_d2_to_c1
    assert!(packet.is_ok());
    if let Ok(packet) = packet {
        assert_eq!(packet.pack_type, ack_from_d2_to_c1.pack_type);
    }

    assert!(handler_d2.join().is_ok());
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
