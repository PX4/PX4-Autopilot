# CAN (DroneCAN & Cyphal)

[Controller Area Network (CAN)](https://en.wikipedia.org/wiki/CAN_bus) is a robust wired network that allows drone components such as flight controller, ESCs, sensors, and other peripherals, to communicate with each other.

It is particularly recommended on larger vehicles.

## Overview

CAN it is designed to be democratic and uses differential signaling.
For this reason it is very robust even over longer cable lengths (on large vehicles), and avoids a single point of failure.
CAN also allows status feedback from peripherals and convenient firmware upgrades over the bus.

PX4 has the ability to  track and log detailed information from CAN devices, including firmware versions, hardware versions, and serial numbers.
This enables unique identification and lifecycle tracking of components.
See [Device Tracking](../advanced_config/device_tracking.md) for more information.

PX4 supports two software protocols for communicating with CAN devices:

- [DroneCAN](../dronecan/index.md): PX4 recommends this for most common setups.
  It is well supported by PX4, is a mature product with extensive peripheral support, and has had years of testing.
- [Cyphal](https://opencyphal.org): PX4 support is a "work in progress".
  Cyphal is a much newer protocol which allows more flexibility and configuration, especially on larger and more complex vehicles.
  It has not yet seen significant adoption.

::: info
Both DroneCAN and Cyphal originate from an earlier project named UAVCAN.
In 2022 the project split into two: the original version of UAVCAN (UAVCAN v0) was renamed to DroneCAN, and the newer UAVCAN v1 was renamed Cyphal.
The differences between the two protocols are outlined in [Cyphal vs. DroneCAN](https://forum.opencyphal.org/t/cyphal-vs-dronecan/1814).
:::

PX4 does not support other CAN software protocols for drones such as KDECAN (at time of writing).

## Wiring

The wiring for CAN networks is the same for both DroneCAN and Cyphal/CAN (in fact, for all CAN networks).

Devices within a network are connected in a _daisy-chain_ in any order (this differs from UARTs peripherals, where you attach just one component per port).

:::warning Don't connect each CAN peripheral to a separate CAN port!
Unlike UARTs, CAN peripherals are designed to be daisy chained, with additional ports such as `CAN2` used for [redundancy](redundancy).
:::

At either end of the chain, a 120Ω termination resistor should be connected between the two data lines.
Flight controllers and some GNSS modules have built in termination resistors for convenience, thus should be placed at opposite ends of the chain.
Otherwise, you can use a termination resistor such as [this one from Zubax Robotics](https://shop.zubax.com/products/uavcan-micro-termination-plug?variant=6007985111069), or solder one yourself if you have access to a JST-GH crimper.

The following diagram shows an example of a CAN bus connecting a flight controller to 4 CAN ESCs and a GNSS.
It includes a redundant bus connected to `CAN 2`.

![CAN Wiring](../../assets/can/uavcan_wiring.svg)

The diagram does not show any power wiring.
Refer to your manufacturer instructions to confirm whether components require separate power or can be powered from the CAN bus itself.

::: info
For more information, see [Cyphal/CAN device interconnection](https://wiki.zubax.com/public/cyphal/CyphalCAN-device-interconnection?pageId=2195476) (kb.zubax.com).
While the article is written with the Cyphal protocol in mind, it applies equally to DroneCAN hardware and any other CAN setup.
For more advanced scenarios, consult with [On CAN bus topology and termination](https://forum.opencyphal.org/t/on-can-bus-topology-and-termination/1685).
:::

### Connectors

Pixhawk standard compatible CAN devices use 4 pin JST-GH connectors for CAN.
Two connectors are used for input and output when wiring in a chain (except for flight controllers and some GNSS devices with builtin termination, which only have a single JST-GH connector).

Other (non-Pixhawk compatible) devices may use different connectors.
However, as long as the device firmware supports DroneCAN or Cyphal, it can be used.

### Redundancy

DroneCAN and Cyphal/CAN support using a second (redundant) CAN interface.
This is completely optional but increases the robustness of the connection.

Pixhawk flight controllers come with 2 CAN interfaces; if your peripherals support 2 CAN interfaces as well, it is recommended to wire both up for increased safety.

### Flight Controllers with Multiple CAN Ports

[Flight Controllers](../flight_controller/index.md) may have up to three independent CAN ports, such as `CAN1`, `CAN2`, `CAN3` (neither DroneCAN or Cyphal support more than three).
Note that you can't have both DroneCAN and Cyphal running on PX4 at the same time.

::: tip
You only _need_ one CAN port to support an arbitrary number of CAN devices using a particular CAN protocol.
Don't connect each CAN peripheral to a separate CAN port!
:::

Generally you'll daisy all CAN peripherals off a single port, and if there is more than one CAN port, use the second one for [redundancy](redundancy).
If three are three ports, you might use the remaining network for devices that support another CAN protocol.

The documentation for your flight controller should indicate which ports are supported/enabled.
At runtime you can check what DroneCAN ports are enabled and their status using the following command on the [MAVLink Shell](../debug/mavlink_shell.md) (or some other console):

```sh
uavcan status
```

Note that you can also check the number of supported CAN interfaces for a board by searching for `CONFIG_BOARD_UAVCAN_INTERFACES` in its [default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v6xrt/default.px4board#) configuration file.

## Firmware

CAN peripherals may run proprietary or open source firmware (check manufacturer guides to confirm the required setup).

PX4 can be built to run as open-source DroneCAN firmware on supported CAN hardware.
See [PX4 DroneCAN Firmware](../dronecan/px4_cannode_fw.md) for more information.

## Support and Configuration

[DroneCAN Setup and Configuration](../dronecan/index.md)

[PX4 DroneCAN Firmware](../dronecan/px4_cannode_fw.md)

## Videos

### DroneCAN

Intro to DroneCAN (UAVCANv0) and practical example with setup in QGroundControl:

<lite-youtube videoid="IZMTq9fTiOM" title="Intro to DroneCAN (UAVCANv0) and practical example with setup in QGroundControl"/>

### Cyphal

UAVCAN v1 for drones (Cyphal) — PX4 Developer Summit Virtual 2020

<lite-youtube videoid="6Bvtn_g8liU" title="UAVCAN v1 for drones — PX4 Developer Summit Virtual 2020"/>

---

Getting started using UAVCAN v1 with PX4 on the NXP UAVCAN Board — PX4 Developer Summit Virtual 2020

<lite-youtube videoid="MwdHwjaXYKs" title="Getting started using UAVCAN v1 with PX4 on the NXP UAVCAN Board"/>

---

UAVCAN: a highly dependable publish-subscribe protocol for hard real-time intra-vehicular networking — PX4 Developer Summit Virtual 2019

<lite-youtube videoid="MBtROivYPik" title="UAVCAN: a highly dependable publish-subscribe protocol for hard ..."/>
