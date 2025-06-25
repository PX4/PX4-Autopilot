# PX4-Autopilot Main Release Notes

<Badge type="danger" text="Alpha" />

<script setup>
import { useData } from 'vitepress'
const { site } = useData();
</script>

<div v-if="site.title !== 'PX4 Guide (main)'">
  <div class="custom-block danger">
    <p class="custom-block-title">This page is on a release branch, and hence probably out of date. <a href="https://docs.px4.io/main/en/releases/main.html">See the latest version</a>.</p>
  </div>
</div>

This contains changes to PX4 `main` branch since the last major release ([PX v1.16](../releases/1.16.md)).

:::warning
PX4 v1.16 is in candidate-release testing, pending release.
Update these notes with features that are going to be in `main` but not the PX4 v1.16 release.
:::

## Read Before Upgrading

TBD …

Please continue reading for [upgrade instructions](#upgrade-guide).

## Major Changes

- TBD

## Upgrade Guide

## Other changes

### Hardware Support

- TBD

### 공통

- [QGroundControl Bootloader Update](../advanced_config/bootloader_update.md#qgc-bootloader-update-sys-bl-update) via the [SYS_BL_UPDATE](../advanced_config/parameter_reference.md#SYS_BL_UPDATE) parameter has been re-enabled after being broken for a number of releases. ([PX4-Autopilot#25032: build: romf: fix generation of rc.board_bootloader_upgrade](https://github.com/PX4/PX4-Autopilot/pull/25032)).

### 제어

- TBD

### Estimation

- TBD

### 센서

- TBD

### 시뮬레이션

- TBD

### Ethernet

- TBD

### uXRCE-DDS / ROS2

- [PX4 ROS 2 Interface Library](../ros2/px4_ros2_control_interface.md) support for [Fixed Wing lateral/longitudinal setpoint](../ros2/px4_ros2_control_interface.md#fixed-wing-lateral-and-longitudinal-setpoint-fwlaterallongitudinalsetpointtype) (`FwLateralLongitudinalSetpointType`) and [VTOL transitions](../ros2/px4_ros2_control_interface.md#controlling-a-vtol). ([PX4-Autopilot#24056](https://github.com/PX4/PX4-Autopilot/pull/24056)).

### MAVLink

- TBD

### Multi-Rotor

- TBD

### 수직이착륙기(VTOL)

- TBD

### Fixed-wing

- TBD

### 탐사선

- TBD

### ROS 2

- TBD
