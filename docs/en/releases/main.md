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

- Fast mission Return modes ([RTL_TYPE](../advanced_config/parameter_reference.md#RTL_TYPE) = 2 and 4) now skip `DO_JUMP` commands (loops) while following the mission path. ([PX4-Autopilot#26993: fix(navigator): goToNextPositionItem skip loops when required](https://github.com/PX4/PX4-Autopilot/pull/26993))

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

- TBD

### Fixed-wing

- TBD

### Rover

- TBD

### ROS 2

- TBD
