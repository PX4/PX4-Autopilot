# T1 Ethernet Peripherals

[Single-pair Ethernet (T1)](https://en.wikipedia.org/wiki/Single-pair_Ethernet) is a family of wired networking standards that carry full Ethernet connectivity over a single twisted pair of wires.
It allows sensors, I/O nodes, and companion computers to communicate with the flight controller (and each other) over a standard IP network, at much higher data rates than CAN or I2C.

It is particularly suited to sensor-rich vehicles, computer vision, and ROS 2 based systems, where a distributed set of smart nodes needs to move a lot of data with predictable timing.

## Overview

T1 Ethernet brings standard, IP-based Ethernet networking to the vehicle over a single lightweight twisted pair, letting you move high-bandwidth data between smart nodes on a maintainable, standards-based network.
This complements the existing peripheral buses: [I2C](../sensor_bus/i2c_general.md) remains a good fit for low-bandwidth sensors over short distances, and [CAN](../can/index.md) for robust, long-distance peripheral buses, while T1 Ethernet targets the high-bandwidth, networked end of the spectrum.

The main benefits of T1 Ethernet on a drone are:

- **High bandwidth.**
  T1 comes in several bandwidth variants (such as 10BASE-T1, 100BASE-T1, and 1000BASE-T1), so you can match the link speed to the application.
  This makes it suitable for data-heavy sensors such as radar, cameras, and optical flow, as well as ROS 2 / Zenoh traffic between nodes.
- **Low cable weight.**
  A single unshielded twisted pair carries the full Ethernet link, instead of the multi-wire cables and heavier shielding used by traditional Ethernet.
  Some nodes can even share power and the T1 pair on a single cable, reducing wiring further.
- **Good maintainability.**
  Links are point-to-point and run standard IP, so you can use common networking tools for diagnostics, and swap or add nodes without rewiring a shared bus.
- **Architectural benefits.**
  T1 Ethernet supports [Time-Sensitive Networking (TSN)](https://en.wikipedia.org/wiki/Time-Sensitive_Networking) for deterministic, low-latency delivery, and lets you build a distributed sensor/IO architecture.
  Because it is standard Ethernet, interoperability is excellent: a single node can aggregate on-board sensors and bridge other buses (such as CAN-FD) onto the network, and the same network is easily extended to cover companion computers, cameras, and other IP devices — all without consuming the flight controller's own ports.

## Wiring

A 100BASE-T1 link is a _point-to-point_ connection between exactly two devices over a single twisted pair (unlike I2C or CAN, there is no shared multi-drop bus on the pair itself).
The twisted pair is carried on a standard 2-pin [JST-GH](https://www.jst-mfg.com/product/index.php?series=105&lang=2) connector.
To connect more than two devices you use a T1-capable Ethernet switch or hub.

On each link, one PHY must be configured as the **master** and the other as the **slave** (the master provides the link timing).
The two ends must be opposite roles; check the node documentation for how its PHY role is set.

Because T1 is standard Ethernet, the connected devices communicate over IP.
For configuring Ethernet networking, IP addressing, and MAVLink/ROS 2 over Ethernet on the PX4 side, see [PX4 Ethernet Setup](../advanced_config/ethernet_setup.md).

Some T1 nodes combine the power input and the T1 pair on a single connector so that one cable carries both the supply and the network link.
Refer to the specific node's documentation to confirm connector pinouts, power requirements, and whether termination or a separate supply is required.

## T1 Nodes

- [NXP MR-MCXN-T1](mr_mcxn_t1_node.md) — an MCXN947-based sensor and I/O hub node that distributes on-board sensors over 100BASE-T1 Ethernet and CAN-FD.
