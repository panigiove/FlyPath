#[cfg(test)]
mod tests {
    use crate::flypath::*;
    #[cfg(feature = "modes")]
    use crate::messages::extract_flypath_message;
    use crossbeam_channel::unbounded;
    use crossbeam_channel::{Receiver, Sender};
    use std::thread::{self, sleep};
    use std::time::Duration;
    use wg_2024::controller::{DroneCommand, DroneEvent};
    use wg_2024::drone::Drone;
    use wg_2024::network::SourceRoutingHeader;
    use wg_2024::packet::{FloodRequest, Fragment};
    use wg_2024::packet::{NackType, NodeType, Packet, PacketType};
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

    fn setup_connected(
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
            // vec![(10, c1_send.clone())].into_iter().collect(),
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

    #[cfg(feature = "modes")]
    fn setup_test_drone_brainrot(
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
            FlyPath::new_with_mode(
                FlyPathModes::BrainRot,
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
    fn test_default_command_remove_and_crash() {
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
        d2_send.send(ack_from_d2_to_c1.clone()).unwrap();
        d2_command_send.send(DroneCommand::Crash).unwrap();

        // wait for a response
        sleep(Duration::from_secs(1));
        let packet = c1_recv.recv_timeout(Duration::from_secs(1));

        // packet should be equal to ack_from_d2_to_c1
        assert!(packet.is_ok());
        if let Ok(packet) = packet {
            assert_eq!(packet.pack_type, ack_from_d2_to_c1.pack_type);
        }

        drop(d2_send);
        assert!(handler_d2.join().is_ok());
    }

    #[test]
    fn test_generic_gragment_forward() {
        tests::generic_fragment_forward::<FlyPath>();
    }

    #[test]
    fn test_generic_fragment_drop() {
        tests::generic_fragment_drop::<FlyPath>();
    }

    #[test]
    fn test_generic_chain_fragment_ack() {
        tests::generic_chain_fragment_ack::<FlyPath>();
    }

    #[test]
    fn test_generic_chain_fragment_drop() {
        tests::generic_chain_fragment_drop::<FlyPath>();
    }

    // when hops[hop_index] doesen't mach the drone's own NodeId
    #[test]
    fn test_default_nack_unexpected_recipient() {
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
    fn test_default_destination_is_drone() {
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
    fn test_deafult_error_in_routing() {
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
    fn test_default_forward_to_drone_panicked() {
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
    fn test_default_FloodRequest_send_to_neighbour() {
        let (
            mut d1,
            mut d2,
            d1_send,
            _d2_send,
            c1_recv,
            _from_d1_event_recv,
            _from_d2_event_recv,
            _d1_command_send,
            _d2_command_send,
        ) = setup_connected(0.0, 0.0);

        let _d1_handler = thread::spawn(move || {
            d1.run();
        });

        let _d2_handler = thread::spawn(move || {
            d2.run();
        });

        let flood_request = FloodRequest::initialize(1, 10, NodeType::Client);
        let packet = Packet::new_flood_request(
            SourceRoutingHeader::initialize(Vec::new()),
            1,
            flood_request.clone(),
        );

        d1_send.send(packet.clone()).unwrap();

        let expected = flood_request
            .get_incremented(1, NodeType::Drone)
            .get_incremented(2, NodeType::Drone)
            .generate_response(1);

        sleep(Duration::from_secs(1));
        let result = c1_recv.recv_timeout(Duration::from_secs(1));

        assert!(result.is_ok());
        if let Ok(packet) = result {
            assert_eq!(expected.pack_type, packet.pack_type);
        }
    }

    #[test]
    fn test_default_invalid_packet_no_hops() {
        let (
            mut drone,
            _test_event_recv,
            test_command_send,
            _test_packet_recv,
            test_packet_send,
            _client_reciver,
        ) = setup_test_drone(0 as f32);

        let handler = thread::spawn(move || {
            drone.run();
        });

        let packet = Packet::new_ack(SourceRoutingHeader::empty_route(), 1, 1);
        test_packet_send.send(packet).unwrap();

        drop(test_packet_send);
        test_command_send.send(DroneCommand::Crash).unwrap();

        assert!(handler.join().is_ok());
    }

    #[cfg(feature = "modes")]
    #[test]
    fn test_crash_brainrot() {
        let (
            mut drone_brainrot,
            test_event_recv_br,
            test_command_send_br,
            _test_packet_recv_br,
            _test_packet_send_br,
            _client_reciver_br,
        ) = setup_test_drone_brainrot(0.0);

        thread::spawn(move || {
            drone_brainrot.run();
        });

        test_command_send_br.send(DroneCommand::Crash).unwrap();
        sleep(Duration::from_secs(1));
        let message = test_event_recv_br.recv_timeout(Duration::from_secs(1));
        assert!(message.is_ok());
    }

    #[cfg(feature = "modes")]
    #[test]
    fn test_packet_send_brainrot() {
        let (
            mut drone_brainrot,
            test_event_recv_br,
            _test_command_send_br,
            _test_packet_recv_br,
            test_packet_send_br,
            _client_reciver_br,
        ) = setup_test_drone_brainrot(0.0);

        thread::spawn(move || {
            drone_brainrot.run();
        });
        let pack_type = PacketType::MsgFragment(Fragment::from_string(1, 1, "Hello".to_string()));
        let msg = Packet {
            pack_type: pack_type.clone(),
            routing_header: SourceRoutingHeader::with_first_hop(vec![3, 1, 2, 12, 11, 10]),
            session_id: 1,
        };
        test_packet_send_br.send(msg.clone()).unwrap();

        sleep(Duration::from_secs(1));
        let result_controller = test_event_recv_br.recv_timeout(Duration::from_secs(1));
        let message = extract_flypath_message(&result_controller.unwrap());
        assert!(message.is_some());
    }

    #[cfg(feature = "modes")]
    #[test]
    fn test_flood_request() {
        let (
            mut drone_brainrot,
            test_event_recv_br,
            _test_command_send_br,
            test_packet_recv_br,
            test_packet_send_br,
            _client_reciver_br,
        ) = setup_test_drone_brainrot(0.0);

        let _handler = thread::spawn(move || {
            drone_brainrot.run();
        });
        let flood_request = FloodRequest::initialize(1, 3, NodeType::Client);
        let packet = Packet::new_flood_request(
            SourceRoutingHeader::new(Vec::new(), 0),
            1,
            flood_request.clone(),
        );
        test_packet_send_br.send(packet.clone()).unwrap();
        flood_request.get_incremented(1, NodeType::Drone);
        let expected = Packet::new_flood_request(
            SourceRoutingHeader::new(Vec::new(), 0),
            1,
            flood_request.clone(),
        );

        sleep(Duration::from_secs(1));
        let result = test_packet_recv_br.recv_timeout(Duration::from_secs(1));
        assert!(result.is_ok());
        let r_packet: Packet = result.unwrap();
        assert_ne!(expected.pack_type, r_packet.pack_type);

        let result_controller = test_event_recv_br.recv_timeout(Duration::from_secs(1));
        assert!(result_controller.is_ok());
        let messages = extract_flypath_message(&result_controller.unwrap());
        assert!(messages.is_some());
    }

    #[cfg(feature = "modes")]
    #[test]
    fn test_addsender_brainrot() {
        let (
            mut drone_brainrot,
            test_event_recv_br,
            test_command_send_br,
            _test_packet_recv_br,
            _test_packet_send_br,
            _client_reciver_br,
        ) = setup_test_drone_brainrot(0.0);

        let _handler = thread::spawn(move || {
            drone_brainrot.run();
        });

        // maybeAddSender
        test_command_send_br
            .send(DroneCommand::AddSender(10, unbounded().0))
            .unwrap();
        sleep(Duration::from_secs(1));
        let result_controller = test_event_recv_br.recv_timeout(Duration::from_secs(1));
        assert!(result_controller.is_ok());
        let messages = extract_flypath_message(&result_controller.unwrap());
        assert!(messages.is_some());
        println!("{:?}", messages);
    }

    #[cfg(feature = "modes")]
    #[test]
    fn test_setdpr_brainrot() {
        let (
            mut drone_brainrot,
            test_event_recv_br,
            test_command_send_br,
            _test_packet_recv_br,
            _test_packet_send_br,
            _client_reciver_br,
        ) = setup_test_drone_brainrot(0.0);

        let _handler = thread::spawn(move || {
            drone_brainrot.run();
        });
        // maybeSetPdr
        test_command_send_br
            .send(DroneCommand::SetPacketDropRate(0.0))
            .unwrap();
        sleep(Duration::from_secs(1));
        let result_controller = test_event_recv_br.recv_timeout(Duration::from_secs(1));
        assert!(result_controller.is_ok());
        let messages = extract_flypath_message(&result_controller.unwrap());
        assert!(messages.is_some());
        println!("{:?}", messages);
    }

    #[cfg(feature = "modes")]
    #[test]
    fn test_removesender_brainrot() {
        let (
            mut drone_brainrot,
            test_event_recv_br,
            test_command_send_br,
            _test_packet_recv_br,
            _test_packet_send_br,
            _client_reciver_br,
        ) = setup_test_drone_brainrot(0.0);

        let _handler = thread::spawn(move || {
            drone_brainrot.run();
        });

        // maybeRemoveSender
        test_command_send_br
            .send(DroneCommand::RemoveSender(2))
            .unwrap();
        sleep(Duration::from_secs(1));
        let result_controller = test_event_recv_br.recv_timeout(Duration::from_secs(1));
        assert!(result_controller.is_ok());
        let messages = extract_flypath_message(&result_controller.unwrap());
        assert!(messages.is_some());
        println!("{:?}", messages);
    }
}
