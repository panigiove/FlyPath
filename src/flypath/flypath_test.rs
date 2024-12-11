#[cfg(test)]
mod tests {
    use crate::flypath::*;
    use crossbeam_channel::unbounded;
    use crossbeam_channel::{Receiver, Sender};
    use std::thread::{self, sleep};
    use std::time::Duration;
    use wg_2024::controller::{DroneCommand, DroneEvent};
    use wg_2024::drone::Drone;
    use wg_2024::network::SourceRoutingHeader;
    use wg_2024::packet::{Ack, FloodRequest, Fragment};
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
