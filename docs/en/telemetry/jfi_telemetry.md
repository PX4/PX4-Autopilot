# J.Fi Wireless Telemetry Module

The J.MARPLE [J.Fi telemetry module](https://jmarple.ai/j-fi/) is a compact and lightweight wireless communication device featuring a PCB-integrated antenna or external antenna, enabling seamless telemetry connections between various drone flight controllers (FC) and ground control stations.

This module includes a Pixhawk-standard JST 6-pin `TELEM` connector, ensuring compatibility with all PX4-based flight controllers.
It supports quick plug-and-play operation to `TELEM1` with default settings, requiring no additional configuration.

The J.Fi telemetry module provides reliable communication up to approximately 500 meters when using a PCB-integrated antenna.
Operating in the 2.4GHz frequency band, it allows unrestricted global use without regulatory limitations.

![J.Fi Wireless Telemetry Module](../../assets/hardware/telemetry/jmarple/jfi_telemetry_module.png)

## Where to Buy

- [https://jmarple.ai/j-fi/](https://jmarple.ai/j-fi/)

## Technical Specifications

### Wireless Performance

- **Frequency Band:** 2.4GHz
- **Speed:** Up to 11 Mbps (adjustable)
- **Range:** Up to 500 meters (varies upon environments)
- **Payload Capacity:** Up to 1400 bytes

### Network Schemes

- **Supported Topologies:** 1:1, 1:N, N:N
- **Collision Management:** Time Slot-Based Response Delay

### User-Friendly Features

- **Buttons:** Pairing and Mode Switching
- **LED Indicators:** Real-time status updates
- **Configuration:** Web browser-based setup
- **Micro USB Port for connecting to PC or GCS**

## Broadcast Communication

**With default settings enabled**, the device automatically broadcasts data to **all nearby J.Fi devices.**
Connect your external device or system to the **Broadcast port.**
No additional setup is required.

![J.Fi Wireless Telemetry Broadcast Communication](../../assets/hardware/telemetry/jmarple/jfi_telemetry_broadcast.png)

## Paired Communication

- Modules must first undergo an **initial pairing procedure.**
- Once paired, communication is **exclusively limited to paired J.Fi devices.** Connect your external device or system to the **Pair port.**

![J.Fi Wireless Telemetry Paired Communication](../../assets/hardware/telemetry/jmarple/jfi_telemetry_paired.png)

### 1:1 Pairing

![J.Fi Wireless Telemetry Buttons](../../assets/hardware/telemetry/jmarple/jfi_telemetry_buttons.png)

- On **each device,** press and hold _button A_, then click the _RST button_.
  Release _button A_ when _LED 1_ blinks.
  - Both devices will enter pairing mode
- Choose one module and double-click _button A_
- On the other module, click _button A_ once
- On the first module, click _button A_ once again to finish pairing
  - Pairing complete

### 1:N Pairing

- On **each device,** press and hold _button A_, then click the _RST button_.
  Release _button A_ when _LED 1_ blinks.
  - All devices will enter pairing mode
- **Host module (1):** Double-click _button A_
- **Client modules (N):** Click _button A_ once on each module to pair
- **Host module (1):** Click _button A_ again to finish pairing
  - Pairing complete.

<lite-youtube videoid="CnjhTfvARmw" title="J.Fi Wireless Telemetry Module Pairing Guide"/>

## PX4 Setup

- **By default**, PX4 uses the **TELEM1** port for telemetry radios (**baud rate: 57600**).
  To use a different baud rate, add a new serial communication link with the desired rate (**see Application Settings → Comms Links**).
- For a **one-to-many (1:N)** MAVLink communication setup, assign a unique **System ID** ([MAV_SYS_ID](../en/advanced_config/parameter_reference.md#MAV_SYS_ID)) to each MAVLink system (1: host, N: clients).

![J.Fi Wireless Telemetry Broadcast Communication](../../assets/hardware/telemetry/jmarple/jfi_telemetry_usage.png)

<lite-youtube videoid="tPeJA2gn7Zw" title="Simultaneous Control using J.Fi Wireless Telemetry Module"/>

## J.Fi Configuration

- **Device:** Press and hold _button B_, then click the _RST button_.
  Release _button B_ when _LED 2_ blinks.
  - Device enters configuration mode
- **Smart device:** Connect to Wi-Fi network named `J.Fi-xxxxxx` (x: alphanumeric characters)
- **Browser:** Go to `192.168.4.1` to open the **configuration page**.
- **Configuration page:** Adjust settings as needed, then click **Save**
  - _LED 1_ blinks once upon saving

![J.Fi Wireless Telemetry Broadcast Communication](../../assets/hardware/telemetry/jmarple/jfi_telemetry_config.jpg)

## Further info

- [User Manual](https://docs.google.com/document/d/1NaVwOLuMCuNpd0uxgilXZ_qfHAnsFgBmaPxX9WGY2h4/edit?usp=sharing)
- [ROS Github](https://github.com/SUV-Lab/J-Fi)
