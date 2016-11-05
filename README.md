## PX4 Pro Drone Autopilot ##

[![Releases](https://img.shields.io/github/release/PX4/Firmware.svg)](https://github.com/PX4/Firmware/releases) [![DOI](https://zenodo.org/badge/22634/PX4/Firmware.svg)](https://zenodo.org/badge/latestdoi/22634/PX4/Firmware) [![Build Status](https://travis-ci.org/PX4/Firmware.svg?branch=master)](https://travis-ci.org/PX4/Firmware) [![Coverity Scan](https://scan.coverity.com/projects/3966/badge.svg?flat=1)](https://scan.coverity.com/projects/3966?tab=overview)

[![Gitter](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/PX4/Firmware?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

This repository holds the [PX4 Pro](http://px4.io) flight control solution for drones, with the main applications located in the src/modules directory. It also contains the PX4 Drone Middleware Platform, which provides drivers and middleware to run drones.

*   Official Website: http://px4.io (License: BSD 3-clause, [LICENSE.md](https://github.com/PX4/Firmware/blob/master/LICENSE.md))
*   Supported airframes (more experimental types than listed here are supported):
  * [Multicopters](http://px4.io/portfolio_category/multicopter/)
  * [Fixed wing](http://px4.io/portfolio_category/plane/)
  * [VTOL](http://px4.io/portfolio_category/vtol/)
*   Releases: [Downloads](https://github.com/PX4/Firmware/releases)

### Users ###

Please refer to the [user documentation](http://px4.io/user-guide/) and [user forum](http://discuss.px4.io) for flying drones with the PX4 flight stack.

### Developers ###

  * [Developer Forum](http://discuss.px4.io)
  * [Guide for Contributions](https://github.com/PX4/Firmware/blob/master/CONTRIBUTING.md)
  * [Developer guide](http://dev.px4.io)

## Maintenance Team

  * Lorenz Meier (lorenz@px4.io) - PX4 Maintainer
  * Dev Call - Mark Whitehorn, Ramon Roche
  * Communication Architecture - Beat Kueng, Julian Oes
  * UI / UX - Gus Grubba
  * Multicopter Flight Control - Dennis Mannhart, Matthias Grob
  * VTOL Flight Control - Roman Bapst, Andreas Antener, Sander Smeets
  * Fixed Wing Flight Control - Daniel Agar, Paul Riseborough
  * Racers - Mark Whitehorn
  * OS / drivers - David Sidrane
  * UAVCAN / Industrial - Pavel Kirienko
  * State Estimation - James Goppert, Paul Riseborough
  * VIO - Christoph Tobler
  * Obstacle Avoidance - Vilhjalmur Vilhjalmsson
  * Snapdragon - Mark Charlebois, Julian Oes
  * Intel Aero - Lucas de Marchi, Simone Guscetti
  * Raspberry Pi / Navio - Beat Kueng
  * Parrot Bebop - Michael Schaeuble

## Supported Hardware

This repository contains code supporting these boards:
  * [Snapdragon Flight](http://dev.px4.io/hardware-snapdragon.html)
  * Intel Aero
  * Raspberry PI with Navio 2
  * [Parrot Bebop](http://dev.px4.io/starting-building.html#parrot-bebop)
  * FMUv1.x
  * FMUv2.x ([Pixhawk](http://dev.px4.io/hardware-pixhawk.html) and [Pixfalcon](http://dev.px4.io/hardware-pixfalcon.html))
  * FMUv3.x ([Pixhawk 2](http://dev.px4.io/hardware-pixhawk.html))
  * FMUv4.x (Pixhawk 3 Pro and [Pixracer](http://dev.px4.io/hardware-pixracer.html))
  * FMUv5.x (ARM Cortex M7, future Pixhawk)
  * AeroCore (v1 and v2)
  * STM32F4Discovery (basic support) [Tutorial](https://pixhawk.org/modules/stm32f4discovery)
  
## Project Milestones

The PX4 software and Pixhawk hardware (which has been designed for it) has been created in 2011 by Lorenz Meier.
