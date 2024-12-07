use crate::flypath::FlyPathModes;
use rand::Rng;
use serde::Deserialize;
use std::{collections::HashMap, fs, u64};
use wg_2024::{
    controller::{DroneCommand, DroneEvent},
    network::{NodeId, SourceRoutingHeader},
    packet::{Fragment, Packet, PacketType, FRAGMENT_DSIZE},
};

/// Rapresent the collection of messages loaded from a JSON file
///
/// # Fields
/// - `spicy`: first hashmap associates theme names to another map
///     - inner map associates event or command names to a list messages
/// - `brainrot`: hashmap that associate event or command names to a list of messages
///
/// # JSON format has the following structures:
/// ```json
/// {
///     "spicy": {
///         "Theme1": {
///             "Event1": ["Message1", "Message2"],
///             "Event2": ["Message3"]
///         },
///         "Theme2": {
///             "Event1": ["Message4"]
///         }
///     },
///     "brainrot": {
///         "Event1": ["Message5", "Message6"],
///         "Event2": ["Message7"]
///     }
/// }
/// ```/// # Notes
/// - **Case Sensitivity**: Theme names, events, and commands are case-sensitive. Ensure to use the
///   auxiliary string conversion functions provided in this struct, such as `Messages::drone_event_to_string`
///   or `Messages::drone_command_to_string`, and the `FlyPathThemes` `Display` implementation to retrieve
///   properly formatted strings.
/// - The structure is compatible with the modes and themes defined in the `FlyPathModes` enum.
#[derive(Debug, Deserialize)]
pub struct Messages {
    spicy: HashMap<String, HashMap<String, Vec<String>>>,
    brainrot: HashMap<String, Vec<String>>,
}

impl Messages {
    /// Load the `Messages` struct from a JSON file at the specified path
    /// and attemts to parse it to populate the `Message` struct.
    ///
    /// # Returns
    /// - `Ok(Messages)`: The parsed `Message` struct if no error occors
    /// - `Err(String)`: An error message if the file cannot be read or the JSON is invalid
    pub fn load_from_file(file_path: &str) -> Result<Self, String> {
        let file_content = fs::read_to_string(file_path)
            .map_err(|e| format!("Failed to read the messages file: {}", e))?;
        serde_json::from_str(&file_content)
            .map_err(|e| format!("Failed to parse the message JSON: {}", e))
    }

    /// Takes a `FlyPathModes` and an event or command (converted to a string using `Messages::drone_event_to_string` or `Messages::drone_command_to_string`)
    /// and retrieves a clone of the messages for the specified event/command and mode from the `Messages` struct.
    ///
    /// # Returns
    /// - `Some(Vec<String>)`: A vector of messages for the specified event/command and mode.
    /// - `None`: If no messages are found.
    pub fn get_messages_for_mode(
        &self,
        mode: &FlyPathModes,
        event_or_command: &str,
    ) -> Option<Vec<String>> {
        match mode {
            FlyPathModes::Spicy(theme) => self
                .spicy
                .get(&theme.to_string())
                .and_then(|events| events.get(event_or_command))
                .filter(|messages| !messages.is_empty())
                .cloned(),
            FlyPathModes::BrainRot => self
                .brainrot
                .get(event_or_command)
                .filter(|messages| !messages.is_empty())
                .cloned(),
            _ => None,
        }
    }

    /// Retrieves a random message for the given mode and event/command.
    ///
    /// # Returns
    /// - `Some(String)`: A randomly selected message from the messages for that mode and event/command.
    /// - `None`: If no messages are found.
    pub fn get_rand_message(&self, mode: &FlyPathModes, event_or_command: &str) -> Option<String> {
        self.get_messages_for_mode(mode, event_or_command)
            .and_then(|messages| {
                let rand_index = rand::thread_rng().gen_range(0..messages.len());
                Some(messages[rand_index].clone())
            })
    }

    /// Generate a special NodeEvent to the controller that is reconizable and contains a random message for that mode and themes
    ///
    /// #`packet::Packet`
    /// - `pack_type`: MsgFragment(Fragment{
    ///         - `fragment_index`: max value of u64,
    ///         - `total_n_framgents`: 0,
    ///         - `length`: real len of message`s bytes,
    ///         - `data`: message in bytes
    ///    })
    /// - `routing_header`: SourceRoutingHeader{
    ///         - `hop_index`: max number of possible hops,
    ///         - `hops`: hops list is empty
    ///    }
    /// - `session_id`: max value of u64
    ///
    /// # Returns
    /// - `Ok(None)`: Does not exist a message for that mode and that event/command
    /// - `Ok(Some(DroneEvent))`: DroneEvent is the event that can be sended to the controller
    /// - `Ok(Err(String))`: the message is too and can not contained inside a `Fragment`, TOO LONG means that the number of UTF-2 bytes that encode the message is too long
    ///
    pub fn generate_droneEvent_to_controller(
        &self,
        mode: &FlyPathModes,
        event_or_command: &str,
        nodeId: NodeId,
    ) -> Result<Option<DroneEvent>, String> {
        if let Some(message) = self.get_rand_message(mode, event_or_command) {
            let bytes = message.into_bytes();
            if bytes.len() > FRAGMENT_DSIZE {
                Err(format!("Failed to generate a message: Too Long"))
            } else {
                let fragment = Fragment {
                    fragment_index: u64::MAX,
                    total_n_fragments: 0,
                    length: bytes.len() as u8,
                    data: {
                        let mut data = [0; FRAGMENT_DSIZE];
                        data[..bytes.len()].copy_from_slice(&bytes);
                        data
                    },
                };

                let packet = Packet {
                    pack_type: PacketType::MsgFragment(fragment),
                    routing_header: SourceRoutingHeader {
                        hop_index: usize::MAX,
                        hops: vec![nodeId], // with this che controller know the sender
                    },
                    session_id: u64::MAX,
                };

                Ok(Some(DroneEvent::PacketSent(packet)))
            }
        } else {
            Ok(None)
        }
    }

    /// Convert the DroneEvent to String, necessary for lookup the messages
    pub fn drone_event_to_string(event: &DroneEvent) -> &str {
        match event {
            DroneEvent::PacketSent(_) => "PacketSent",
            DroneEvent::PacketDropped(_) => "PacketDropped",
            DroneEvent::ControllerShortcut(_) => "ControllerShortcut",
        }
    }

    /// Convert the DroneCommand to String, necessary for lookup the messages
    pub fn drone_command_to_string(command: &DroneCommand) -> &str {
        match command {
            DroneCommand::AddSender(_, _) => "AddSender",
            DroneCommand::RemoveSender(_) => "RemoveSender",
            DroneCommand::SetPacketDropRate(_) => "SetPacketDropRate",
            DroneCommand::Crash => "Crash",
        }
    }
}

/// Identifies and extracts a FlyPath message from a given `DroneEvent`.
///
/// # Returns
/// - `Some((NodeId, String))`:
///   - `NodeId`: The identifier of the FlyPath drone operating in spicy mode.
///   - `String`: The themed message associated with the event.
/// - `None`: If the event is a normal `DroneEvent` or invalid as a FlyPath event.
///
/// # Examples
/// ```rust
/// // Assuming `event` is a valid FlyPath DroneEvent:
/// if let Some((node_id, message)) = extract_flypath_message(&event) {
///     println!("FlyPath Event Detected! Node: {}, Message: {}", node_id, message);
/// } else {
///     println!("This is not a FlyPath event.");
/// }
/// ```
pub fn extract_flypath_message(event: &DroneEvent) -> Option<(NodeId, String)> {
    if let DroneEvent::PacketSent(packet) = event {
        if let PacketType::MsgFragment(fragment) = &packet.pack_type {
            if packet.session_id == u64::MAX
                && packet.routing_header.hop_index == usize::MAX
                && !packet.routing_header.hops.is_empty()
                && packet.routing_header.hops.len() == 1
                && fragment.fragment_index == u64::MAX
                && fragment.total_n_fragments == 0
            {
                // We can assume for sure that this special invalid Fragment is FlyPath Fragment
                let node_id = packet.routing_header.hops.get(0).unwrap();
                return Some((
                    *node_id,
                    String::from_utf8_lossy(&fragment.data[..fragment.length as usize]).to_string(),
                ));
            }
        }
    }
    None // Return None if event is not a FlyPath DroneEvent
}

#[cfg(test)]
mod tests {
    use std::result;

    use super::*;
    use crate::flypath::FlyPathThemes;

    #[test]
    fn test_valid_file() {
        let json_data = r#"
        {
            "spicy": {
                "Batman": {
                    "PacketSent": ["Message1", "Message2"],
                    "PacketDropped": [],
                    "ControllerShortcut": [],
                    "RemoveSender": [],
                    "AddSender": [],
                    "SetPacketDropRate": [],
                    "Crash": []
                }

            },
            "brainrot": {}
        }
        "#;

        let file_path = "test_valid.json";
        fs::write(file_path, json_data).expect("Failed to write test file");

        let messages = Messages::load_from_file(file_path);
        assert!(messages.is_ok());
        fs::remove_file(file_path).unwrap();
    }

    #[test]
    fn test_file_not_found() {
        let result = Messages::load_from_file("non_existent.json");
        assert!(result.is_err());
        assert!(result
            .unwrap_err()
            .contains("Failed to read the messages file"));
    }

    #[test]
    fn test_malformed_json() {
        let json_data = r#"{"spicy": {"batman": "invalid_type"}}"#;
        let file_path = "test_malformed.json";
        fs::write(file_path, json_data).expect("Failed to write test file");

        let result = Messages::load_from_file(file_path);
        assert!(result.is_err());
        assert!(result
            .unwrap_err()
            .contains("Failed to parse the message JSON"));
        fs::remove_file(file_path).unwrap();
    }

    #[test]
    fn test_get_rand_messages() {
        // Case: Randomness verification
        {
            let json_data = r#"
            {
                "spicy": {
                    "Batman": {
                        "Crash": ["Mister Freeze", "Talia al Ghul"]
                    },
                    "Rocket": {
                        "Crash": []
                    }
                },
                "brainrot": {
                    "Crash": ["feafie", "feaff"]
                }
            }
            "#;
            let file_path = "test_rand_messages_expanded.json";
            std::fs::write(file_path, json_data).expect("Failed to write test file");

            let messages: Messages = Messages::load_from_file(file_path).unwrap();
            let mode = &FlyPathModes::Spicy(FlyPathThemes::Batman);
            let event_or_command = Messages::drone_command_to_string(&DroneCommand::Crash);
            let mut seen_messages = std::collections::HashSet::new();

            for _ in 0..100 {
                if let Some(rand_message) = messages.get_rand_message(mode, &event_or_command) {
                    seen_messages.insert(rand_message);
                }
            }

            let expected_messages = vec!["Mister Freeze", "Talia al Ghul"];
            for message in &expected_messages {
                assert!(
                    seen_messages.contains(*message),
                    "Expected to see '{}' in random selections, but it was missing",
                    message
                );
            }

            std::fs::remove_file(file_path).unwrap();
        }

        // Case: Single message
        {
            let json_data = r#"
            {
                "spicy": {
                    "Batman": {
                        "Crash": ["SingleMessage"]
                    }
                },
                "brainrot":{}
            }
            "#;
            let file_path = "test_single_message.json";
            std::fs::write(file_path, json_data).expect("Failed to write test file");

            let messages: Messages = Messages::load_from_file(file_path).unwrap();
            let mode = &FlyPathModes::Spicy(FlyPathThemes::Batman);
            let event_or_command = Messages::drone_command_to_string(&DroneCommand::Crash);

            let rand_message = messages.get_rand_message(mode, &event_or_command).unwrap();
            assert_eq!(
                "SingleMessage", rand_message,
                "Expected the only available message to be returned, but got '{}'",
                rand_message
            );

            std::fs::remove_file(file_path).unwrap();
        }

        // Case: No message
        {
            let json_data = r#"
            {
                "spicy": {
                    "Batman": {
                        "Crash": []
                    }
                },
                "brainrot":{}
            }
            "#;
            let file_path = "test_no_message.json";
            std::fs::write(file_path, json_data).expect("Failed to write test file");

            let messages: Messages = Messages::load_from_file(file_path).unwrap();
            let mode = &FlyPathModes::Spicy(FlyPathThemes::Batman);
            let event_or_command = Messages::drone_command_to_string(&DroneCommand::Crash);

            let rand_message = messages.get_rand_message(mode, &event_or_command);
            assert!(
                rand_message.is_none(),
                "Expected no message to be returned, but got '{:?}'",
                rand_message
            );

            std::fs::remove_file(file_path).unwrap();
        }
    }

    #[test]
    fn test_generate_nodeEvent_to_controller() {
        // Case: Valid Test
        {
            let json_data = r#"
            {
                "spicy": {
                    "Batman": {
                        "Crash": ["singlemessage", "secondmessage"]
                    }
                },
                "brainrot":{}
            }
            "#;
            let file_path = "test_generate_nodeevent_to_controller.json";
            std::fs::write(file_path, json_data).expect("Failed to write test file");

            let messages: Messages = Messages::load_from_file(file_path).unwrap();
            let mode = &FlyPathModes::Spicy(FlyPathThemes::Batman);
            let event_or_command = Messages::drone_command_to_string(&DroneCommand::Crash);

            let result = messages.generate_droneEvent_to_controller(mode, event_or_command, 1);
            assert!(
                result.as_ref().is_ok_and(|x| x.is_some()),
                "Expected to find a message and generate a DroneEvent, but got '{:?}'",
                result
            );

            std::fs::remove_file(file_path).unwrap();
        }

        // Case: Message Too Long
        {
            let json_data = r#"
            {
                "spicy": {
                    "Batman": {
                        "Crash": ["129CharLongStringFillFillFillFillFillFillFillFillFillFillFillFillFillFillFillFillFillFillFillFillFillFillFillFillFillFillFillFill"]
                    }
                },
                "brainrot":{}
            }
            "#;
            let file_path = "test_generate_nodeevent_to_controller.json";
            std::fs::write(file_path, json_data).expect("Failed to write test file");

            let messages: Messages = Messages::load_from_file(file_path).unwrap();
            let mode = &FlyPathModes::Spicy(FlyPathThemes::Batman);
            let event_or_command = Messages::drone_command_to_string(&DroneCommand::Crash);
            let result = messages.generate_droneEvent_to_controller(mode, event_or_command, 1);
            assert!(result
                .unwrap_err()
                .contains("Failed to generate a message: Too Long"));

            std::fs::remove_file(file_path).unwrap();
        }

        // Case: No Message
        {
            let json_data = r#"
            {
                "spicy": {
                    "Batman": {
                        "Crash": []
                    }
                },
                "brainrot":{}
            }
            "#;
            let file_path = "test_generate_nodeevent_to_controller.json";
            std::fs::write(file_path, json_data).expect("Failed to write test file");

            let messages: Messages = Messages::load_from_file(file_path).unwrap();
            let mode = &FlyPathModes::Spicy(FlyPathThemes::Batman);
            let event_or_command = Messages::drone_command_to_string(&DroneCommand::Crash);
            let result = messages.generate_droneEvent_to_controller(mode, event_or_command, 1);
            assert!(
                result.as_ref().is_ok_and(|x| x.is_none()),
                "Expected to find a message and generate a DroneEvent, but got '{:?}'",
                result
            );

            std::fs::remove_file(file_path).unwrap();
        }
    }

    #[test]
    fn test_extract_flypath_message() {
        // Case: FlyPath DroneEvent
        {
            let json_data = r#"
            {
                "spicy": {
                    "Batman": {
                        "Crash": ["singlemessage"]
                    }
                },
                "brainrot":{}
            }
            "#;
            let file_path = "test_generate_nodeevent_to_controller.json";
            std::fs::write(file_path, json_data).expect("Failed to write test file");

            let messages: Messages = Messages::load_from_file(file_path).unwrap();
            let mode = &FlyPathModes::Spicy(FlyPathThemes::Batman);
            let event_or_command = Messages::drone_command_to_string(&DroneCommand::Crash);

            let droneEvent = messages
                .generate_droneEvent_to_controller(mode, event_or_command, 1)
                .unwrap()
                .unwrap();

            let result = extract_flypath_message(&droneEvent);
            let expect: Option<(NodeId, String)> = Some((1, "singlemessage".to_string()));

            assert_eq!(result, expect);
        }

        // Case: Normal DroneEvent
        {
            let normal_event = DroneEvent::PacketSent(Packet {
                pack_type: PacketType::MsgFragment(Fragment {
                    fragment_index: u64::MAX - 1,
                    total_n_fragments: u64::MAX,
                    length: 13,
                    data: [
                        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0,
                    ],
                }),
                routing_header: SourceRoutingHeader {
                    hop_index: 2,
                    hops: vec![1, 2, 3],
                },
                session_id: 1,
            });

            let result = extract_flypath_message(&normal_event);
            assert!(result.is_none(), "Expect to find None but got {:?}", result)
        }
    }
}
