# PX4-Autopilot Main Release Notes

<Badge type="danger" text="Alpha" />

<script setup>
import { useData } from 'vitepress'
const { site } = useData();
</script>

<div v-if="site.title !== 'PX4 Guide (main)'">
  <div class="custom-block danger">
    <p class="custom-block-title">This page is on a release branch, and hence probably out of date. <a href="https://docs.px4.io/main/en/releases/main">See the latest version</a>.</p>
  </div>
</div>

This contains changes to PX4 `main` branch after the next major release ([PX v1.18](../releases/1.16.md)).

::: warning
PX4 v1.18 is in alpha/beta testing.
Update these notes with features that are going to be in `main` (PX4 v1.18 or later) but not the PX4 v1.18 release.
:::

## Read Before Upgrading

- TBD

Please continue reading for [upgrade instructions](#upgrade-guide).

## Major Changes

- TBD

## Upgrade Guide

## Other changes

- RTL_MISSION_FAST and RTL_MISSION_FAST_REVERSE now skip DO_JUMP commands (loops). ([PX4-Autopilot#26993: fix(navigator): goToNextPositionItem skip loops when required](https://github.com/PX4/PX4-Autopilot/pull/26993))

### Hardware Support

- TBD

### Common

- TBD

### Control

- TBD

### Safety

- TBD

### Estimation

- TBD

### Sensors

- TBD

### Simulation

- TBD

### Debug & Logging

- TBD

### Ethernet

- TBD

### uXRCE-DDS / Zenoh / ROS2

- TBD

### MAVLink

- TBD

### RC

- TBD

### Multi-Rotor

- TBD

### VTOL

- Return mode: VTOLs returning in fixed-wing mode now use the approach loiter associated with the selected home/rally landing location (instead of home) to choose the most wind-aligned valid approach if several are defined. ([PX4-Autopilot#27004: fix(navigator): rtl compute wind angle to select best land approach based on rally point location instead of home location](https://github.com/PX4/PX4-Autopilot/pull/27004))

### Fixed-wing

- TBD

### Rover

- TBD

### ROS 2

- TBD
