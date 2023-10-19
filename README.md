# PX4 Drone Autopilot

[![Releases](https://img.shields.io/github/release/PX4/PX4-Autopilot.svg)](https://github.com/PX4/PX4-Autopilot/releases) [![DOI](https://zenodo.org/badge/22634/PX4/PX4-Autopilot.svg)](https://zenodo.org/badge/latestdoi/22634/PX4/PX4-Autopilot)

[![Nuttx Targets](https://github.com/PX4/PX4-Autopilot/workflows/Nuttx%20Targets/badge.svg)](https://github.com/PX4/PX4-Autopilot/actions?query=workflow%3A%22Nuttx+Targets%22?branch=master) [![SITL Tests](https://github.com/PX4/PX4-Autopilot/workflows/SITL%20Tests/badge.svg?branch=master)](https://github.com/PX4/PX4-Autopilot/actions?query=workflow%3A%22SITL+Tests%22)

[![Discord Shield](https://discordapp.com/api/guilds/1022170275984457759/widget.png?style=shield)](https://discord.gg/dronecode)

This repository holds the [PX4](http://px4.io) flight control solution for drones, with the main applications located in the [src/modules](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules) directory. It also contains the PX4 Drone Middleware Platform, which provides drivers and middleware to run drones.

PX4 is highly portable, OS-independent and supports Linux, NuttX and MacOS out of the box.

* Official Website: http://px4.io (License: BSD 3-clause, [LICENSE](https://github.com/PX4/PX4-Autopilot/blob/main/LICENSE))
* [Supported airframes](https://docs.px4.io/main/en/airframes/airframe_reference.html) ([portfolio](https://px4.io/ecosystem/commercial-systems/)):
  * [Multicopters](https://docs.px4.io/main/en/frames_multicopter/)
  * [Fixed wing](https://docs.px4.io/main/en/frames_plane/)
  * [VTOL](https://docs.px4.io/main/en/frames_vtol/)
  * [Autogyro](https://docs.px4.io/main/en/frames_autogyro/)
  * [Rover](https://docs.px4.io/main/en/frames_rover/)
  * many more experimental types (Blimps, Boats, Submarines, High altitude balloons, etc)
* Releases: [Downloads](https://github.com/PX4/PX4-Autopilot/releases)


## Building a PX4 based drone, rover, boat or robot

The [PX4 User Guide](https://docs.px4.io/main/en/) explains how to assemble [supported vehicles](https://docs.px4.io/main/en/airframes/airframe_reference.html) and fly drones with PX4.
See the [forum and chat](https://docs.px4.io/main/en/#getting-help) if you need help!


## Changing code and contributing

This [Developer Guide](https://docs.px4.io/main/en/development/development.html) is for software developers who want to modify the flight stack and middleware (e.g. to add new flight modes), hardware integrators who want to support new flight controller boards and peripherals, and anyone who wants to get PX4 working on a new (unsupported) airframe/vehicle.

Developers should read the [Guide for Contributions](https://docs.px4.io/main/en/contribute/).
See the [forum and chat](https://docs.px4.io/main/en/#getting-help) if you need help!


### Weekly Dev Call

The PX4 Dev Team syncs up on a [weekly dev call](https://docs.px4.io/main/en/contribute/).

> **Note** The dev call is open to all interested developers (not just the core dev team). This is a great opportunity to meet the team and contribute to the ongoing development of the platform. It includes a QA session for newcomers. All regular calls are listed in the [Dronecode calendar](https://www.dronecode.org/calendar/).


## Maintenance Team

Note: This is the source of truth for the active maintainers of PX4 ecosystem.

| Sector | Maintainer |
|---|---|
| Founder | [Lorenz Meier](https://github.com/LorenzMeier) |
| Architecture | [Daniel Agar](https://github.com/dagar) / [Beat Küng](https://github.com/bkueng)|
| State Estimation | [Mathieu Bresciani](https://github.com/bresch) / [Paul Riseborough](https://github.com/priseborough) |
| OS/NuttX | [David Sidrane](https://github.com/davids5) |
| Drivers | [Daniel Agar](https://github.com/dagar) |
| Simulation | [Jaeyoung Lim](https://github.com/Jaeyoung-Lim) |
| ROS2 | [Beniamino Pozzan](https://github.com/beniaminopozzan) |
| Community QnA Call | [Ramon Roche](https://github.com/mrpollo) |
| [Documentation](https://docs.px4.io/main/en/) | [Hamish Willee](https://github.com/hamishwillee) |

| Vehicle Type | Maintainer |
|---|---|
| Multirotor | [Matthias Grob](https://github.com/MaEtUgR) |
| Fixed Wing | [Thomas Stastny](https://github.com/tstastny) |
| Hybrid VTOL | [Silvan Fuhrer](https://github.com/sfuhrer) |
| Boat | x |
| Rover | x |

See also [maintainers list](https://px4.io/community/maintainers/) (px4.io) and the [contributors list](https://github.com/PX4/PX4-Autopilot/graphs/contributors) (Github). However it may be not up to date.

## Supported Hardware

Pixhawk standard boards and proprietary boards are shown below (discontinued boards aren't listed).

For the most up to date information, please visit [PX4 user Guide > Autopilot Hardware](https://docs.px4.io/main/en/flight_controller/).

### Pixhawk Standard Boards

These boards fully comply with Pixhawk Standard, and are maintained by the PX4-Autopilot maintainers and Dronecode team

* FMUv6X and FMUv6C
  * [CUAV Pixahwk V6X (FMUv6X)](https://docs.px4.io/main/en/flight_controller/cuav_pixhawk_v6x.html)
  * [Holybro Pixhawk 6X (FMUv6X)](https://docs.px4.io/main/en/flight_controller/pixhawk6x.html)
  * [Holybro Pixhawk 6C (FMUv6C)](https://docs.px4.io/main/en/flight_controller/pixhawk6c.html)
  * [Holybro Pix32 v6 (FMUv6C)](https://docs.px4.io/main/en/flight_controller/holybro_pix32_v6.html)
* FMUv5 and FMUv5X (STM32F7, 2019/20)
  * [Pixhawk 4 (FMUv5)](https://docs.px4.io/main/en/flight_controller/pixhawk4.html)
  * [Pixhawk 4 mini (FMUv5)](https://docs.px4.io/main/en/flight_controller/pixhawk4_mini.html)
  * [CUAV V5+ (FMUv5)](https://docs.px4.io/main/en/flight_controller/cuav_v5_plus.html)
  * [CUAV V5 nano (FMUv5)](https://docs.px4.io/main/en/flight_controller/cuav_v5_nano.html)
  * [Auterion Skynode (FMUv5X)](https://docs.auterion.com/avionics/skynode)
* FMUv4 (STM32F4, 2015)
  * [Pixracer](https://docs.px4.io/main/en/flight_controller/pixracer.html)
  * [Pixhawk 3 Pro](https://docs.px4.io/main/en/flight_controller/pixhawk3_pro.html)
* FMUv3 (STM32F4, 2014)
  * [Pixhawk 2](https://docs.px4.io/main/en/flight_controller/pixhawk-2.html)
  * [Pixhawk Mini](https://docs.px4.io/main/en/flight_controller/pixhawk_mini.html)
  * [CUAV Pixhack v3](https://docs.px4.io/main/en/flight_controller/pixhack_v3.html)
* FMUv2 (STM32F4, 2013)
  * [Pixhawk](https://docs.px4.io/main/en/flight_controller/pixhawk.html)

### Manufacturer supported

These boards are maintained to be compatible with PX4-Autopilot by the Manufacturers.

* [ARK Electronics ARKV6X](https://docs.px4.io/main/en/flight_controller/arkv6x.html)
* [CubePilot Cube Orange+](https://docs.px4.io/main/en/flight_controller/cubepilot_cube_orangeplus.html)
* [CubePilot Cube Orange](https://docs.px4.io/main/en/flight_controller/cubepilot_cube_orange.html)
* [CubePilot Cube Yellow](https://docs.px4.io/main/en/flight_controller/cubepilot_cube_yellow.html)
* [Holybro Durandal](https://docs.px4.io/main/en/flight_controller/durandal.html)
* [Airmind MindPX V2.8](http://www.mindpx.net/assets/accessories/UserGuide_MindPX.pdf)
* [Airmind MindRacer V1.2](http://mindpx.net/assets/accessories/mindracer_user_guide_v1.2.pdf)
* [Holybro Kakute F7](https://docs.px4.io/main/en/flight_controller/kakutef7.html)

### Community supported

These boards don't fully comply industry standards, and thus is solely maintained by the PX4 public community members.

### Experimental

These boards are nor maintained by PX4 team nor Manufacturer, and is not guaranteed to be compatible with up to date PX4 releases.

* [Raspberry PI with Navio 2](https://docs.px4.io/main/en/flight_controller/raspberry_pi_navio2.html)
* [Bitcraze Crazyflie 2.0](https://docs.px4.io/main/en/complete_vehicles/crazyflie2.html)

## Project Roadmap

**Note: Outdated**

A high level project roadmap is available [here](https://github.com/orgs/PX4/projects/25).

## Project Governance

The PX4 Autopilot project including all of its trademarks is hosted under [Dronecode](https://www.dronecode.org/), part of the Linux Foundation.

<a href="https://www.dronecode.org/" style="padding:20px" ><img src="https://mavlink.io/assets/site/logo_dronecode.png" alt="Dronecode Logo" width="110px"/></a>
<a href="https://www.linuxfoundation.org/projects" style="padding:20px;"><img src="https://mavlink.io/assets/site/logo_linux_foundation.png" alt="Linux Foundation Logo" width="80px" /></a>
<div style="padding:10px">&nbsp;</div>
