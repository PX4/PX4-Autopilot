# PX4-Autopilot Головна нотатка щодо релізу

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

## Прочитайте перед оновленням

TBD …

Please continue reading for [upgrade instructions](#upgrade-guide).

## Основні зміни

- Уточнюється

## Інструкції для оновлення

## Інші зміни

### Підтримка обладнання

- Уточнюється

### Загальні

- [QGroundControl Bootloader Update](../advanced_config/bootloader_update.md#qgc-bootloader-update-sys-bl-update) via the [SYS_BL_UPDATE](../advanced_config/parameter_reference.md#SYS_BL_UPDATE) parameter has been re-enabled after being broken for a number of releases. ([PX4-Autopilot#25032: build: romf: fix generation of rc.board_bootloader_upgrade](https://github.com/PX4/PX4-Autopilot/pull/25032)).

### Управління

- Added new flight mode(s): [Altitude Cruise (MC)](../flight_modes_mc/altitude_cruise.md), Altitude Cruise (FW).
  For fixed-wing the mode behaves the same as Altitude mode but you can disable the manual control loss failsafe. (PX4-Autopilot#25435: Add new flight mode: Altitude Cruise
  ).

### Оцінки

- Уточнюється

### Датчики

- Add [sbgECom INS driver](../sensor/sbgecom.md) ([PX4-Autopilot#24137](https://github.com/PX4/PX4-Autopilot/pull/24137))

### Симуляція

- Уточнюється

### Ethernet

- Уточнюється

### uXRCE-DDS / ROS2

- [PX4 ROS 2 Interface Library](../ros2/px4_ros2_control_interface.md) support for [Fixed Wing lateral/longitudinal setpoint](../ros2/px4_ros2_control_interface.md#fixed-wing-lateral-and-longitudinal-setpoint-fwlaterallongitudinalsetpointtype) (`FwLateralLongitudinalSetpointType`) and [VTOL transitions](../ros2/px4_ros2_control_interface.md#controlling-a-vtol). ([PX4-Autopilot#24056](https://github.com/PX4/PX4-Autopilot/pull/24056)).

### MAVLink

- Уточнюється

### Мульти-Ротор

- Removed parameters `MPC_{XY/Z/YAW}_MAN_EXPO` and use default value instead, as they were not deemed necessary anymore. ([PX4-Autopilot#25435: Add new flight mode: Altitude Cruise](https://github.com/PX4/PX4-Autopilot/pull/25435)).
- Renamed `MPC_HOLD_DZ` to `MAN_DEADZONE` to have it globally available in modes that allow for a dead zone. ([PX4-Autopilot#25435: Add new flight mode: Altitude Cruise](https://github.com/PX4/PX4-Autopilot/pull/25435)).

### VTOL

- Уточнюється

### Літак з фіксованим крилом

- [Fixed Wing Takeoff mode](../flight_modes_fw/takeoff.md) will now keep climbing with level wings on position loss.
  A target takeoff waypoint can be set to control takeoff course and loiter altitude. ([PX4-Autopilot#25083](https://github.com/PX4/PX4-Autopilot/pull/25083)).

### Ровер

- Removed deprecated rover module ([PX4-Autopilot#25054](https://github.com/PX4/PX4-Autopilot/pull/25054)).

### ROS 2

- Уточнюється
