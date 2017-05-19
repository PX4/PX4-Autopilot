## PX4 Pro Drone Autopilot ##

[![Releases](https://img.shields.io/github/release/PX4/Firmware.svg)](https://github.com/PX4/Firmware/releases) [![DOI](https://zenodo.org/badge/22634/PX4/Firmware.svg)](https://zenodo.org/badge/latestdoi/22634/PX4/Firmware) [![Build Status](https://travis-ci.org/PX4/Firmware.svg?branch=master)](https://travis-ci.org/PX4/Firmware) [![Coverity Scan](https://scan.coverity.com/projects/3966/badge.svg?flat=1)](https://scan.coverity.com/projects/3966?tab=overview)

[![Gitter](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/PX4/Firmware?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

This repository holds the [PX4 Pro](http://px4.io) flight control solution for drones, with the main applications located in the [src/modules](https://github.com/PX4/Firmware/tree/master/src/modules) directory. It also contains the PX4 Drone Middleware Platform, which provides drivers and middleware to run drones.

  * Official Website: http://px4.io (License: BSD 3-clause, [LICENSE.md](https://github.com/PX4/Firmware/blob/master/LICENSE.md))
  * Supported airframes:
    * [Multicopters](http://px4.io/portfolio_category/multicopter/)
    * [Fixed wing](http://px4.io/portfolio_category/plane/)
    * [VTOL](http://px4.io/portfolio_category/vtol/)
    * many more experimental types (Rovers, Blimps, Boats, Submarines, etc)
  * Releases: [Downloads](https://github.com/PX4/Firmware/releases)

Please refer to the [user documentation](https://docs.px4.io/en/) and [user forum](http://discuss.px4.io) for flying drones with the PX4 flight stack.

### Weekly Dev Call

The PX4 Dev Team syncs up on a [weekly dev call](https://dev.px4.io/en/contribute/).

  * [Wednesday 17:00 Central European Time, 11:00 Eastern Time, 08:00 Pacific Standard Time](https://www.google.com/calendar/embed?src=bGludXhmb3VuZGF0aW9uLm9yZ19nMjF0dmFtMjRtN3BtN2poZXYwMWJ2bHFoOEBncm91cC5jYWxlbmRhci5nb29nbGUuY29t)
  * [Uber conference (dial-in or web client)](https://www.uberconference.com/lf-dronecode)
  * The agenda is announced in advance on the [PX4 Discuss](http://discuss.px4.io/c/weekly-dev-call)
  * Issues and PRs may be labelled [devcall](https://github.com/PX4/Firmware/issues?q=is%3Aopen+is%3Aissue+label%3Adevcall) to flag them for discussion

### Developers ###
  * [Developer Guide](https://dev.px4.io/)
    * [Build instructions](https://dev.px4.io/en/setup/building_px4.html)
    * [Guide for Contributions](https://dev.px4.io/en/contribute/)

## Maintenance Team

  * Project / Founder - [Lorenz Meier](https://github.com/LorenzMeier)
  * [Dev Call](https://github.com/PX4/Firmware/labels/devcall) - [Ramon Roche](https://github.com/mrpollo)
  * Communication Architecture - [Beat Kueng](https://github.com/bkueng), [Julian Oes](https://github.com/JulianOes)
  * UI / UX - [Gus Grubba](https://github.com/dogmaphobic)
  * [Multicopter Flight Control](https://github.com/PX4/Firmware/labels/multicopter) - [Dennis Mannhart](https://github.com/Stifael), [Matthias Grob](https://github.com/MaEtUgR)
  * [VTOL Flight Control](https://github.com/PX4/Firmware/labels/vtol) - [Roman Bapst](https://github.com/tumbili), [Andreas Antener](https://github.com/AndreasAntener), [Sander Smeets](https://github.com/sanderux)
  * [Fixed Wing Flight Control](https://github.com/PX4/Firmware/labels/fixedwing) - [Daniel Agar](https://github.com/dagar), [Paul Riseborough](https://github.com/priseborough)
  * Racers - [Anton Matosov](https://github.com/anton-matosov)
  * OS / drivers - [David Sidrane](https://github.com/davids5)
  * [UAVCAN](https://github.com/PX4/Firmware/labels/uavcan) / Industrial - [Pavel Kirienko](https://github.com/pavel-kirienko)
  * [State Estimation](https://github.com/PX4/Firmware/issues?q=is%3Aopen+is%3Aissue+label%3A%22state+estimation%22) - [James Goppert](https://github.com/jgoppert), [Paul Riseborough](https://github.com/priseborough)
  * VIO - [Mohammed Kabir](https://github.com/mhkabir), [Christoph Tobler](https://github.com/ChristophTobler)
  * Obstacle Avoidance - [Vilhjalmur Vilhjalmsson](https://github.com/vilhjalmur89)
  * [Snapdragon](https://github.com/PX4/Firmware/labels/snapdragon) - [Mark Charlebois](https://github.com/mcharleb)
  * [Intel Aero](https://github.com/PX4/Firmware/labels/intel%20aero) - [Lucas De Marchi](https://github.com/lucasdemarchi) and [José Roberto de Souza](https://github.com/zehortigoza)
  * [Raspberry Pi / Navio](https://github.com/PX4/Firmware/labels/raspberry_pi) - [Beat Kueng](https://github.com/bkueng)
  * [Parrot Bebop](https://github.com/PX4/Firmware/labels/bebop) - [Michael Schaeuble](https://github.com/eyeam3)
  * [Airmind MindPX / MindRacer](https://github.com/PX4/Firmware/labels/mindpx) - [Henry Zhang](https://github.com/iZhangHui)

## Supported Hardware

This repository contains code supporting these boards:
  * [Snapdragon Flight](https://dev.px4.io/en/flight_controller/snapdragon_flight.html)
  * [Intel Aero](https://dev.px4.io/en/flight_controller/intel_aero.html)
  * [Raspberry PI with Navio 2](https://dev.px4.io/en/flight_controller/raspberry_pi.html)
  * [Parrot Bebop 2](https://dev.px4.io/en/advanced/parrot_bebop.html)
  * FMUv1.x
  * FMUv2.x
    * [Pixhawk](https://dev.px4.io/en/flight_controller/pixhawk.html)
    * Pixhawk Mini
    * [Pixfalcon](https://dev.px4.io/en/flight_controller/pixfalcon.html)
  * FMUv3.x (Pixhawk 2 / 2.1 Cube)
  * FMUv4.x
    * [Pixracer](https://dev.px4.io/en/flight_controller/pixracer.html)
    * Pixhawk 3 Pro
  * FMUv5.x (ARM Cortex M7, future Pixhawk)
  * STM32F4Discovery (basic support) [Tutorial](https://pixhawk.org/modules/stm32f4discovery)
  * Gumstix AeroCore (v1 and v2)
  * [Airmind MindPX V2.8](http://www.mindpx.net/assets/accessories/UserGuide_MindPX.pdf)
  * [Airmind MindRacer V1.2](http://mindpx.net/assets/accessories/mindracer_user_guide_v1.2.pdf)
  * [Bitcraze Crazyflie 2.0](https://dev.px4.io/en/flight_controller/crazyflie2.html)

## Project Milestones

The PX4 software and Pixhawk hardware (which has been designed for it) has been created in 2011 by [Lorenz Meier](https://github.com/LorenzMeier).
