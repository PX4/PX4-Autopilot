# Official PX4 Developer Kit Program

This topic explains how manufacturers can get a drone kit certified as an _Official PX4 Developer Kit_.

The PX4 community is growing quickly, and developers are frequently blocked on getting the right hardware into their hands.
Too many options and configuration permutations also make it hard for maintainers to support the community well.
Official PX4 Developer Kits solve this by establishing a foundational hardware reference for the ecosystem: kits that meet a defined standard for the bare-minimum components a PX4 developer needs to create modern applications, backed by the trust of the PX4 name.

In return, Dronecode puts its official weight behind qualifying products (see [What you get](#what-you-get) below).

Currently certified kits are listed in [PX4 Developer Kits](../dev_kits/index.md).

## Requirements

A kit qualifies when it meets all five requirements:

1. A flight controller meeting the latest [Pixhawk standard](https://pixhawk.org/standards/), or matching its capabilities
2. The latest stable PX4 release, pre-installed
3. Pre-assembled, or assembly that requires no technical skill (no soldering)
4. A guide, a focused tutorial, and a reference sheet included with the kit
5. Third-party build quality verification

The reference sheet must ship printed, in the box: pinouts, connectors, and specs at a glance, with no screen required ([example: ModalAI VOXL 2 reference sheet](https://docs.modalai.com/voxl2-d0014)).
The guide and tutorial may ship as QR-code links.

## What Qualifying Looks Like in Practice

The experience your kit must deliver:

- **Rapid start:** No technical skill needed to get flying. Minimal or zero assembly, software pre-installed, documentation included.
- **Room to extend:** Ample headroom for performant code. The latest components, or space to expand.
- **Current hardware:** Latest-generation drone hardware, matching the newest Pixhawk standard or its capabilities.
- **A modern companion computer:** Able to offload compute from the flight controller MCU.
- **Everything included:** Bundled with the open source software and tools developers expect.
- **Quality assurance:** An available support channel or forum.

### Where to Focus: the Companion Computer

The biggest differentiator for a kit is the companion computer and the tools developers can leverage on it:

- **MAVLink:** A solid interface that can feed multiple targets, using [MAVSDK](https://mavsdk.mavlink.io/), mission planning tools, or MAVROS.
- **DDS & ROS 2:** Leverage the [uXRCE-DDS](../middleware/uxrce_dds.md) integration and expose the interface directly to the companion computer over a high-bandwidth link.
- **Easy app deployment:** Tools that help developers continuously test applications across the whole development lifecycle.
- **LTE / WiFi:** A competent network stack that reaches the cloud securely and efficiently.
- **Great documentation:** Publicly accessible docs, with ways to contribute back for continuous improvement.

## What You Get {#what-you-get}

Kits accepted into the program receive:

- **Official endorsement:** The _Official PX4 Developer Kit_ designation, on your product and your marketing.
- **PX4 website listing:** A listing on the PX4 website and in this documentation, for standing presence in front of the entire community.
- **Official promotion:** A promotional announcement from official Dronecode accounts on social media.

## Eligibility

The program is open exclusively to [Dronecode Foundation](https://dronecode.org/) members.
Active membership is required for the endorsement, the website listing, and the promotion.

To apply, or to discuss membership as a first step, [contact Dronecode](https://dronecode.org/contact/).

## 另见

- [PX4 Developer Kits](../dev_kits/index.md)
- [Manufacturer's Board Support Guide](../hardware/board_support_guide.md)
