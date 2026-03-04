---
# https://vitepress.dev/reference/default-theme-home-page
layout: home

hero:
  name: "PX4"
  text: "Autopilot"
  tagline: Open-source flight stack for drones and autonomous vehicles. BSD-3 licensed. Vendor neutral.
  image: /px4-logo.svg
  actions:
    - theme: brand
      text: Read the docs
      link: /en/index.md
    - theme: alt
      text: Website
      link: https://px4.io/
    - theme: alt
      text: Source code on GitHub
      link: https://github.com/PX4/PX4-Autopilot

features:
  - title: Modular Architecture
    details: Built on uORB, a DDS-compatible publish/subscribe middleware. Every module runs as its own thread, fully parallelized and thread safe. Build custom configurations and strip out what you don't need.
  - title: Wide Hardware Support
    details: Supports Pixhawk-standard flight controllers and a growing range of boards beyond that standard. DroneCAN peripherals run PX4 firmware in CAN node mode. No vendor lock-in.
  - title: Developer Friendly
    details: First-class MAVLink and DDS/ROS 2 integration. Comprehensive SITL simulation, hardware-in-the-loop testing, and log analysis tools. MAVSDK provides a high-level SDK/API for programmatic vehicle interaction.
  - title: Autonomy Ready
    details: Extensible architecture for advanced autonomy. External modes, offboard control, and DDS/ROS 2 interfaces provide the building blocks for computer vision, GPS-denied navigation, and custom flight behaviors.
  - title: Proven at Scale
    details: Millions of vehicles deployed worldwide across commercial, defense, and research applications. Continuous flight testing validates the codebase across multirotors, fixed-wing, VTOL, helicopters, rovers, and more.
  - title: Permissive License
    details: BSD 3-Clause. Use it, modify it, ship it in proprietary products. You only need to include the original copyright notice and license text.
  - title: Interoperability
    details: Part of a modular ecosystem. PX4 (autopilot), MAVLink (protocol), QGroundControl (ground station), Pixhawk (hardware standard), MAVSDK (SDK/API), and ROS 2 via DDS or Zenoh. All Dronecode projects, all open source.
  - title: Vendor Neutral Governance
    details: Hosted by the Dronecode Foundation under the Linux Foundation. No single company owns the name or controls the roadmap. Community-driven with a weekly open dev call.

search: false
footer: BSD 3-clause license
---

<!-- <Redirect to="/en/README.md" /> -->
