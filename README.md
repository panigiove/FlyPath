# FlyPath

**Drone Implementation by FlyPath**

## Modes

- **Default (Boring) Mode**: The drone operates according to the specifications defined in the protocol file. This is the standard and predictable behavior.
- **Spicy Mode**: The drone enhances communication with themed messages sent to the simulation controller in response to commands or events, while still following the protocol.
- **BrainRot Mode**: After prolonged exposure to content like Chinese TikTok Lives and *Skibidi Toilet* episodes on YouTube, the drone exhibits erratic and unpredictable behaviors. In this mode, it may:
    - Modify the remaining path of a packet arbitrarily.
    - Encrypt packets unexpectedly.
    - Make independent decisions about dropping packets.
    - Refuse to crash under specific conditions.
    - Add or remove senders from a packet's metadata.
    - Alter the Packet Delivery Ratio (PDR).
    - Modify the flood request's path trace or the initiator ID.

**IMPORTANT**: In both `Spicy` and `BrainRot` modes, the drone may send special `FlyPath messages` to the controller. To avoid unintended panics or issues, you must implement the `extract_flypath_message` function. Refer to the section *Special FlyPath Messages in Spicy and BrainRot Modes* for detailed instructions.

## Customer Support

For any issues, bugs, or questions related to our drone, please contact us through our Telegram group:  
[https://t.me/FlyPathSupport](https://t.me/FlyPathSupport)

## Panics

This library avoids direct access to arrays or vectors, and does not use `unwrap`. All `Result<>` and `Option<>` types are properly handled to prevent panics. The drone will ignore invalid packets and will only panic in the event of a critical error, such as an issue when attempting to send a `DroneEvent` to the controller.

## Installation
Add the `flypath` dependency to your project's `Cargo.toml`:

```toml
[dependencies]
flyPath = { git = "https://github.com/panigiove/FlyPath.git" }
```

### Optional Feature: Modes

To enable the `modes` feature (which includes `Spicy` and `BrainRot` modes), update the dependency as follows:

```toml
[dependencies]
flyPath = { git = "https://github.com/panigiove/FlyPath.git", features = ["modes"] }
```

### Update Dependencies

Run the following command to update your dependencies:

```bash
cargo update
```

---

## Usage

### Basic Constructor: `new`

The `new` constructor creates a basic `FlyPath` drone instance. It is available regardless of whether the `modes` feature is enabled.

#### Signature:

```rust
use flyPath::FlyPath;

FlyPath::new(
    id: NodeId, 
    controller_send: Sender, 
    controller_recv: Receiver, 
    packet_recv: Receiver, 
    packet_send: Sender, 
    pdr: f64
) -> FlyPath
```

#### Example:

```rust
use flyPath::FlyPath;

let flypath = FlyPath::new(
    1, 
    controller_send, 
    controller_recv, 
    packet_recv, 
    packet_send, 
    0.95
);
```

---

### Advanced Constructor: `new_with_mode`

The `new_with_mode` constructor allows specifying a custom mode for the drone. This is only available when the `modes` feature is enabled.

#### Signature:

```rust
use flyPath::FlyPath;

FlyPath::new_with_mode(
    mode: FlyPathModes, 
    id: NodeId, 
    controller_send: Sender, 
    controller_recv: Receiver, 
    packet_recv: Receiver, 
    packet_send: Sender, 
    pdr: f64
) -> FlyPath
```

#### Parameters:

- **`mode`**: The operating mode of the drone, as defined by the `FlyPathModes` enum:
  - `FlyPathModes::Default`: Standard operation.
  - `FlyPathModes::Spicy(FlyPathThemes)`: Custom messages based on selected themes.
  - `FlyPathModes::BrainRot`: Erratic and unpredictable behavior.
- Remaining parameters are identical to the `new` constructor.

#### Example with a Custom Mode:

```rust
use flyPath::FlyPath;

let flypath = FlyPath::new_with_mode(
    FlyPathModes::Spicy(FlyPathThemes::DarkSouls),
    1, 
    controller_send, 
    controller_recv, 
    packet_recv, 
    packet_send, 
    0.95
);
```

---

### Modes and Themes (Requires `modes` Feature)

Enabling the `modes` feature allows access to additional modes and themes.

#### Modes (`FlyPathModes`):

- `Default`: Standard behavior.
- `Spicy`: Sends custom messages based on themes.
- `BrainRot`: Erratic, meme-like behavior.

#### Themes (`FlyPathThemes`):

Available themes for `Spicy` mode include:

- `Batman`
- `Rocket`
- `Quackable`
- `HarryPotter`
- `DarkSouls`
- `Bloodborne`
- `Pingu`

#### Example:

```rust
let mode = FlyPathModes::Spicy(FlyPathThemes::Pingu);
println!("Selected mode: {:?}", mode);
```

---

### Running the Drone

Start the drone using the `run` method:

```rust
flypath.run();
```

> **Note:** In `BrainRot` mode, the drone behaves unpredictably and may perform random actions, including ignoring crashes.

---

### Special FlyPath Messages in `Spicy` and `BrainRot` Modes

In these modes, the drone sends **special FlyPath messages** encapsulated in `NodeEvent::PacketSent` events.

#### Extracting FlyPath Messages

To detect and extract FlyPath messages, use the `extract_flypath_message` function:

```rust
pub fn extract_flypath_message(event: &DroneEvent) -> Option<(NodeId, String)>
```

#### Returns:

- **`Some((NodeId, String))`**:
  - `NodeId`: The ID of the drone sending the message.
  - `String`: The FlyPath message content.
- **`None`**: If the event is not a FlyPath message or is invalid.

#### Example:

```rust
use flypath::extract_flypath_message;

let event = controller.recv().unwrap();
if let Some((node_id, message)) = extract_flypath_message(&event) {
    println!("FlyPath Event Detected! Node: {}, Message: {}", node_id, message);
} else {
    println!("This is not a FlyPath event.");
}
```

---

#### How It Works

FlyPath messages are identified by specific invalid parameters within the `PacketType::MsgFragment` of a `DroneEvent::PacketSent`. These parameters ensure unique identification:

- `session_id == u64::MAX`: Marks a special session ID.
- `routing_header.hop_index == usize::MAX`: Invalid hop index.
- `routing_header.hops.len() == 1`: Ensures exactly one hop.
- `fragment.fragment_index == u64::MAX` and `fragment.total_n_fragments == 0`: Marks the fragment as special and unfragmented.

---

## Examples

Find examples in the `examples/` directory of the repository.

#### Default Examples (No `modes` Feature Required):

- `default_command.rs`
- `default_packet.rs`
- `default_flood_request.rs`

#### Spicy Examples (Requires `modes` Feature):

- `spicy_command.rs`
- `spicy_packet.rs`

---

## Dependencies

The FlyPath project relies on the following key dependencies:

- **`crossbeam-channel`**: Multi-threaded communication.
- **`rand`**: Random number generation.
- **`serde` and `serde_json`** (optional, enabled via `modes` feature): Serialization and JSON handling.
- **`wg_2024`**: Code shared between groups.

