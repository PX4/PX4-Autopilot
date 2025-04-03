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


- **By default**, PX4 uses the `TELEM1` port for telemetry radios, with the baud rate set to `57600`. Since J.Fi also uses `57600` as its default baud rate, it usually works simply by connecting the device.

- If you want to use a different baud rate, set the [SER_TEL1_BAUD](../advanced_config/parameter_reference.md#SER_TEL1_BAUD) parameter in PX4 to the desired value, and also configure the J.Fi device to match.
For how to configure J.Fi, please refer to the **J.Fi Configuration** section below.

::: info
If you want to use the `TELEM2` port or any other port instead, change the [MAV_0_CONFIG](../advanced_config/parameter_reference.md#MAV_0_CONFIG) parameter to the corresponding port and update the appropriate **SER_TELX_BAUD** parameter.

For more details, please refer to the following link:
https://docs.px4.io/main/en/peripherals/serial_configuration.html
:::

- When connecting the receiver to **QGroundControl**, if the baud rate is set to 57600, it can automatically connect just like a SiK Radio.
If you're using a different baud rate, you must first **disable SiK Radio in QGC,** `Application Settings → General → AutoConnect`.
Then, go to `Application Settings → Comms Links`, click `Add`, and create a new link configuration.
Set **Type** to **Serial**, configure the **Serial Port** and **Baud Rate** to match the J.Fi device, and then click `Connect`.

::: info
For one-to-one connections, the default baud rate is typically sufficient.
However, for one-to-many (1:N) setups, a higher baud rate is recommended to ensure stable data reception.
:::

::: info
While it's recommended that all J.Fi devices use the same baud rate, we've confirmed that communication is still partially possible even when using different baud rates across devices.
:::

- For a **one-to-many (1:N)** MAVLink communication setup, assign a unique **System ID** ([MAV_SYS_ID](../advanced_config/parameter_reference.md#MAV_SYS_ID)) to each MAVLink system (1: host, N: clients).

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
