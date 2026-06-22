# MAVLink Security Hardening for Production Deployments

<Badge type="tip" text="PX4 v1.17" />

MAVLink is an open communication protocol designed for lightweight, low-latency communication between drones and ground stations.
By default, all MAVLink messages are unauthenticated.
This is intentional for development and testing, but **production deployments must enable [message signing](message_signing.md)** to prevent unauthorized access.

::: warning
Without message signing enabled, any device that can send MAVLink messages to the vehicle (via radio, network, or serial) can execute any command, including shell access, file operations, parameter changes, mission uploads, arming, and flight termination.
:::

## What Is at Risk

When MAVLink signing is not enabled, an attacker within communication range can:

| Capability                   | MAVLink mechanism                                |
| ---------------------------- | ------------------------------------------------ |
| Execute shell commands       | `SERIAL_CONTROL` with `SERIAL_CONTROL_DEV_SHELL` |
| Read, write, or delete files | MAVLink FTP protocol                             |
| Change any flight parameter  | `PARAM_SET` / `PARAM_EXT_SET`                    |
| Upload or overwrite missions | Mission protocol                                 |
| Arm or disarm motors         | `MAV_CMD_COMPONENT_ARM_DISARM`                   |
| Terminate flight (crash)     | `MAV_CMD_DO_FLIGHTTERMINATION`                   |
| Trigger emergency landing    | Spoofed `BATTERY_STATUS`                         |
| Reboot the vehicle           | `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`              |

All of these are standard MAVLink capabilities used by ground control stations.
Without signing, there is no distinction between a legitimate GCS and an unauthorized sender.

## Hardening Checklist

### 1. Enable Message Signing

Message signing provides cryptographic authentication for all MAVLink communication.
See [Message Signing](message_signing.md) for full details.

Steps:

1. Connect to the vehicle over a **trusted link** (USB or other secure connection).
2. Provision a 32-byte secret key using the [SETUP_SIGNING](https://mavlink.io/en/messages/common.html#SETUP_SIGNING) message. This works on any link, but use a trusted one for initial provisioning.
3. Provision the same key on all ground control stations and companion computers that need to communicate with the vehicle.
4. Verify that unsigned messages from unknown sources are rejected.

::: info
Once a key is provisioned, signing is enforced automatically on **all links** (including USB).
Changing or disabling the key requires a signed `SETUP_SIGNING` message.
Signing changes are rejected while the vehicle is armed.
Signing can also be disabled by physically removing the key file from the SD card.
:::

### 2. Secure Physical Access

- **SD card**: The signing key is stored at `/mavlink/mavlink-signing-key.bin`.
  Anyone with physical access to the SD card can read, modify, or remove the key file.
- **USB ports**: USB follows the same signing rules as all other links.
  When signing is active, USB requires signed messages.
- **Debug ports (SWD/JTAG)**: If exposed, [Debug Ports](../debug/swd_debug.md) allow full firmware reflash and bypass all software security.
  Not all vehicles expose debug connectors.

::: warning
Signing protects all MAVLink links.
The primary physical attack surface is the SD card (key file extraction or deletion).
If your threat model includes physical access, secure the SD card slot and debug ports.
:::

### 3. Secure Network Links

- Do not expose MAVLink UDP/TCP ports to untrusted networks or the internet.
- Place MAVLink communication links behind firewalls or VPNs.
- Segment MAVLink networks from business or public networks.
- When using companion computers, audit which network interfaces MAVLink is bound to.

### 4. Understand the Limitations

- **No encryption**:
  Message signing provides authentication and integrity, but messages are sent in plaintext.
  An eavesdropper can read telemetry and commands but cannot forge them.
- **Allowlisted messages**:
  A small set of [safety-critical messages](message_signing.md#unsigned-message-allowlist) (`HEARTBEAT`, `RADIO_STATUS`, `ADSB_VEHICLE`, `COLLISION`) are always accepted unsigned on all links.
  An attacker could spoof these specific messages.
- **Key management**:
  There is no automatic key rotation.
  Keys must be reprovisioned manually via a signed `SETUP_SIGNING` message.
- **Lost key recovery**:
  If the signing key is lost on all GCS devices, the only recovery is physical: remove the SD card and delete the key file, or reflash via SWD/JTAG.
  There is no software-only recovery path.
  See [Message Signing: Lost Key Recovery](message_signing.md#lost-key-recovery) for details.

## Integrator Responsibility

PX4 is open-source flight controller firmware used by manufacturers and system integrators to build commercial and custom drone platforms.

Securing the communication links for a specific deployment is the responsibility of the system integrator.
This includes:

- Choosing appropriate radio hardware and link security
- Enabling and managing MAVLink message signing
- Restricting network access to MAVLink interfaces
- Applying firmware updates that address security issues
- Evaluating whether the default configuration meets the security requirements of the target application

PX4 provides the tools for securing MAVLink communication.
Integrators must enable and configure them for their deployment context.
