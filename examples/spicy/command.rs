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

    let _handler_d2 = thread::spawn(move || {
        d2.run();
    });

    // <-------------------------------------------------------------------------------->
    // *CASE*: ADD SENDER CASE
    println!("ADD SENDER CASE");
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

    while let Ok(event) = from_d1_event_recv.recv_timeout(Duration::from_secs(1)) {
        if let Some((node_id, fly_path_msg)) = flyPath::extract_flypath_message(&event) {
            println!("{node_id} - {}", fly_path_msg);
        } else {
            if let DroneEvent::PacketSent(_) = event {
                println!("normal PacketSent event");
            } else {
                println!("normal PacketDropped event");
            }
        }
    }

    // <-------------------------------------------------------------------------------->
    // *CASE*: CHANGE PDR FROM 0 TO 1
    println!("CHANGE PDR FROM 0 TO 1");
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

    while let Ok(event) = from_d1_event_recv.recv_timeout(Duration::from_secs(1)) {
        if let Some((node_id, fly_path_msg)) = flyPath::extract_flypath_message(&event) {
            println!("{node_id} - {}", fly_path_msg);
        } else {
            if let DroneEvent::PacketSent(_) = event {
                println!("normal PacketSent event");
            } else {
                println!("normal PacketDropped event");
            }
        }
    }

    // <-------------------------------------------------------------------------------->
    // *CASE*: REMOVE SENDER
    println!("REMOVE SENDER");
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

    while let Ok(event) = from_d1_event_recv.recv_timeout(Duration::from_secs(1)) {
        if let Some((node_id, fly_path_msg)) = flyPath::extract_flypath_message(&event) {
            println!("{node_id} - {}", fly_path_msg);
        } else {
            if let DroneEvent::PacketSent(_) = event {
                println!("normal PacketSent event");
            } else {
                println!("normal PacketDropped event");
            }
        }
    }

    // <-------------------------------------------------------------------------------->
    // *CASE*: CRASH
    println!("CRASH");
    d2_command_send.send(DroneCommand::RemoveSender(1)).unwrap();
    d1_command_send.send(DroneCommand::Crash).unwrap();
    drop(d1_send);

    while let Ok(event) = from_d1_event_recv.recv_timeout(Duration::from_secs(1)) {
        if let Some((node_id, fly_path_msg)) = flyPath::extract_flypath_message(&event) {
            println!("{node_id} - {}", fly_path_msg);
        } else {
            if let DroneEvent::PacketSent(_) = event {
                println!("normal PacketSent event");
            } else {
                println!("normal PacketDropped event");
            }
        }
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
    // c1 - d1       d2
    let (d1_send, d1_recv) = unbounded();
    let (d2_send, d2_recv) = unbounded();
    let (c1_send, c1_recv) = unbounded();

    let (d1_event_send, from_d1_event_recv) = unbounded();
    let (d2_event_send, from_d2_event_recv) = unbounded();

    let (d1_command_send, d1_command_recv) = unbounded();
    let (d2_command_send, d2_command_recv) = unbounded();

    let d1 = FlyPath::new_with_mode(
        flyPath::FlyPathModes::Spicy(flyPath::FlyPathThemes::Batman),
        1,
        d1_event_send,
        d1_command_recv,
        d1_recv,
        // vec![(2, d2_send.clone()), (10, c1_send.clone())]
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
