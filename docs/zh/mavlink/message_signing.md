# MAVLink Message Signing

[MAVLink 2 message signing](https://mavlink.io/en/guide/message_signing.html) allows PX4 to cryptographically verify that incoming MAVLink messages originate from a trusted source (authentication).

:::info
This mechanism does not _encrypt_ the message payload.
:::

## 综述

When signing is enabled, PX4 appends a 13-byte [signature](https://mavlink.io/en/guide/message_signing.html#signature) to every outgoing MAVLink 2 message.

Incoming messages are checked against the shared secret key, and unsigned or incorrectly signed messages are rejected (with [exceptions for safety-critical messages](#unsigned-message-allowlist)).

The signing implementation is built into the MAVLink module and is always available, with no special build flags required.
The key is stored in an SD card:

- **No key on SD card**:
  Signing is disabled.
  All messages are sent unsigned and all incoming messages are accepted.
- **Valid key on SD card**:
  Signing is active on **all links** (including USB).
  Outgoing messages are signed.
  Incoming messages must be signed (with [exceptions](#unsigned-message-allowlist)).

## Enable/Disable Signing

Signing is controlled using the standard MAVLink [SETUP_SIGNING](https://mavlink.io/en/messages/common.html#SETUP_SIGNING) message (as per the [MAVLink signing specification](https://mavlink.io/en/guide/message_signing.html)):

- To **enable** signing, send a `SETUP_SIGNING` message with a valid key on any link when no key is currently provisioned (see [Key Provisioning](#key-provisioning)).
- To **disable** signing via MAVLink, send a `SETUP_SIGNING` message with an all-zero key and timestamp.
  This message **must be signed with the current active key**.
  An unsigned blank-key message is rejected.
- To **change** the signing key, send a `SETUP_SIGNING` message with the new key on any link.
  When signing is already active, the message must be signed with the current key.

:::warning
Signing key changes (enable, disable, or rotate) are **rejected while the vehicle is armed**.
The vehicle must be disarmed before signing configuration can be changed.
:::

:::tip
If the signing key is lost, you can still disable signing if you have physical access to the vehicle.
Either delete the key file (`/mavlink/mavlink-signing-key.bin`) from the SD card and reboot, or remove the SD card entirely.
:::

## Key Provisioning

The signing key is set by sending the MAVLink [SETUP_SIGNING](https://mavlink.io/en/messages/common.html#SETUP_SIGNING) message (ID 256) to PX4.
This message contains:

- A 32-byte secret key
- A 64-bit initial timestamp

PX4 accepts `SETUP_SIGNING` on **any link** (USB, telemetry radio, network, and so on).

When signing is **not active** (no key provisioned), the first `SETUP_SIGNING` with a valid key enables signing.
When signing is **already active**, key changes (including disabling) require that the `SETUP_SIGNING` message is signed with the current key.

Note that `SETUP_SIGNING` is rejected while the vehicle is armed (disarm before provisioning or changing keys).
As per the MAVLink specification, `SETUP_SIGNING` is never forwarded to other links.

## Key Storage

The secret key and timestamp are stored on the SD card at:

```txt
/mavlink/mavlink-signing-key.bin
```

The file is a 40-byte binary file:

| Offset | Size     | Content                                                  |
| ------ | -------- | -------------------------------------------------------- |
| 0      | 32 bytes | Secret key                                               |
| 32     | 8 bytes  | Timestamp (`uint64_t`, little-endian) |

The file is created with mode `0600` (owner read/write only), and the containing `/mavlink/` directory is created with mode `0700` (owner only).

On startup, PX4 reads the key from this file.
If the file exists and contains a non-zero key or timestamp, signing is activated automatically.

:::info
The timestamp in the file is set when `SETUP_SIGNING` is received.
A graceful shutdown also writes the current timestamp back, but in practice most vehicles are powered off by pulling the battery, so the on-disk timestamp will typically remain at the value from the last key provisioning.
:::

:::info
Storage of the key on the SD card means that signing can be disabled by removing the card.
Note that this requires physical access to the vehicle.
:::

## How It Works

### Initialization

1. The MAVLink module calls `MavlinkSignControl::start()` during startup.
2. The `/mavlink/` directory is created if it doesn't exist.
3. The `mavlink-signing-key.bin` file is opened if it exists.
4. If a valid key is found (non-zero key or timestamp), signing is activated: the signing struct is wired into the MAVLink library and outgoing messages are signed.
5. If no valid key is found, the signing struct is left disconnected, and the MAVLink library operates with zero signing overhead.

### Outgoing Messages

When signing is active (valid key present), the `MAVLINK_SIGNING_FLAG_SIGN_OUTGOING` flag is set, which causes the MAVLink library to automatically append a [SHA-256 based signature](https://mavlink.io/en/guide/message_signing.html#signature) to every outgoing MAVLink 2 message.

When no key is present, signing is completely bypassed with no CPU or bandwidth overhead.

### Incoming Messages

For each incoming message, the MAVLink library checks whether a valid signature is present.
If the message is unsigned or has an invalid signature, the library calls the `accept_unsigned` callback, which decides whether to accept or reject the message based on:

1. **Signing not active**: If no key has been loaded, all messages are accepted.
2. **Allowlisted message**: Certain [safety-critical messages](#unsigned-message-allowlist) are always accepted.

## Unsigned Message Allowlist

The following messages are **always** accepted unsigned, regardless of the signing state.
These are safety-critical messages that may originate from systems that don't support signing:

| 消息                                                                                           | ID  | Reason                                                   |
| -------------------------------------------------------------------------------------------- | --- | -------------------------------------------------------- |
| [HEARTBEAT](https://mavlink.io/en/messages/common.html#HEARTBEAT)                            | 0   | System discovery and liveness detection                  |
| [RADIO_STATUS](https://mavlink.io/en/messages/common.html#RADIO_STATUS) | 109 | Radio link status from SiK radios and other radio modems |
| [ADSB_VEHICLE](https://mavlink.io/en/messages/common.html#ADSB_VEHICLE) | 246 | ADS-B traffic information for collision avoidance        |
| [COLLISION](https://mavlink.io/en/messages/common.html#COLLISION)                            | 247 | Collision threat warnings                                |

## Security Considerations

### Signing is enforced on all links

When signing is active, **all links require signed messages**.
This means:

- An attacker cannot send unsigned commands on any link.
- Changing or disabling the key requires sending a `SETUP_SIGNING` message **signed with the current key**.
- Signing can be disabled via MAVLink by sending a signed `SETUP_SIGNING` with an all-zero key.

### Armed guard

`SETUP_SIGNING` is rejected while the vehicle is armed.
This prevents the signing configuration from being changed during flight, whether by accident or by an attacker who has obtained the key.

### Lost key recovery

If the signing key is lost on the GCS side and no device has the current key:

- **Remove the SD card** and delete `/mavlink/mavlink-signing-key.bin`, then reboot.
- **Reflash via SWD/JTAG** if the SD card is not accessible.

:::warning
There is no software-only recovery path for a lost key.
This is intentional: any MAVLink-based recovery mechanism would also be available to an attacker.
Physical access to the SD card or debug port is required.
:::

### Other considerations

- **Initial key provisioning**: When no key is provisioned, `SETUP_SIGNING` is accepted unsigned on any link.
  Once a key is active, subsequent changes require a signed message.
  Provision the initial key over a trusted connection, such as USB.
- **Key not exposed via parameters**: The secret key is stored in a separate file on the SD card, not as a MAVLink parameter, so it cannot be read back through the parameter protocol.
- **SD card access**: Anyone with physical access to the SD card can read or modify the `mavlink-signing-key.bin` file, or remove the card entirely to disable signing.
  Ensure physical security of the vehicle if signing is used as a security control.
- **Replay protection**: The MAVLink signing protocol includes a timestamp that prevents replay attacks.
  The on-disk timestamp is updated when a new key is provisioned via `SETUP_SIGNING`.
  A graceful shutdown also persists the current timestamp, but since most vehicles are powered off by pulling the battery, the on-disk timestamp will typically remain at the value from the last key provisioning on reboot.
- **No encryption**: Message signing provides **authentication and integrity only**.
  Messages are still sent in plaintext.
  An eavesdropper can read all message contents (telemetry, commands, parameters, missions) but cannot forge or modify them without the key.
- **Allowlisted messages bypass signing**: A small set of [safety-critical messages](#unsigned-message-allowlist) are always accepted unsigned.
  An attacker can spoof these specific messages (e.g. fake `ADSB_VEHICLE` traffic) even when signing is active.

### What signing does NOT protect against

| Attack                                                | Why                                                     |
| ----------------------------------------------------- | ------------------------------------------------------- |
| Eavesdropping                                         | Messages are not encrypted                              |
| SD card extraction                                    | Key file is readable by anyone with physical access     |
| Spoofed `HEARTBEAT`/`RADIO_STATUS`/`ADSB`/`COLLISION` | These are allowlisted unsigned                          |
| Lost key without SD card access                       | Requires SWD reflash                                    |
| Key rotation                                          | No automatic mechanism; manual re-provisioning required |
| In-flight key changes                                 | `SETUP_SIGNING` rejected while armed                    |
