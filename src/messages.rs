use crate::flypath::FlyPathModes;
use rand::Rng;
use serde::Deserialize;
use std::{collections::HashMap, fs};
use wg_2024::controller::{DroneCommand, DroneEvent};

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

    // The actual function that retrieves messages based on the provided mode and event/command.
    fn get_messages_for_mode(
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

    /// Takes a `FlyPathModes` and an event or command (converted to a string using `Messages::drone_event_to_string` or `Messages::drone_command_to_string`) and retrieves a clone of the messages for the specified event/command and mode from the `Messages` struct.
    ///
    /// # Returns
    /// - `Some(Vec<String>)`: A vector of messages for the specified event/command and mode.
    /// - `None`: If no messages are found.
    pub fn get_messages(&self, mode: &FlyPathModes, event_or_command: &str) -> Option<Vec<String>> {
        self.get_messages_for_mode(mode, event_or_command)
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::flypath::FlyPathThemes;

    #[test]
    fn test_valid_file() {
        let json_data = r#"
        {
            "spicy": {
                "batman": {
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
    fn test_get_messages() {
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
        let file_path = "test_messages.json";
        std::fs::write(file_path, json_data).expect("Failed to write test file");

        let messages: Messages = Messages::load_from_file(file_path).unwrap();

        // Test 1: Valid case
        {
            let mode = &FlyPathModes::Spicy(FlyPathThemes::Batman);
            let event_or_command = Messages::drone_command_to_string(&DroneCommand::Crash);
            let actual = messages.get_messages(mode, &event_or_command).unwrap();
            let expected = vec!["Mister Freeze", "Talia al Ghul"];
            assert_eq!(
                expected, actual,
                "Expected messages do not match for Spicy/Batman mode and Crash event"
            );
        }

        // Test 2: Empty messages
        {
            let mode = &FlyPathModes::Spicy(FlyPathThemes::Rocket);
            let event_or_command = Messages::drone_command_to_string(&DroneCommand::Crash);
            let actual = messages.get_messages(mode, &event_or_command);
            assert!(
                actual.is_none(),
                "Expected no messages for Spicy/Rocket mode and Crash event, but got some"
            );
        }

        // Test 3: Non-existent event
        {
            let mode = &FlyPathModes::Spicy(FlyPathThemes::Batman);
            let event_or_command = "NonExistentEvent";
            let actual = messages.get_messages(mode, event_or_command);
            assert!(
                actual.is_none(),
                "Expected no messages for Spicy/Rocket mode and and non-existent event, but got some"
            );
        }

        // Cleanup test file
        std::fs::remove_file(file_path).unwrap()
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
    }
}
