# PX4-Autopilot Головна нотатка щодо релізу

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

This contains changes to the PX4 `main` branch that are not included in the next release ([PX4 v1.18](../releases/1.18.md)).

:::warning
PX4 v1.18 is in beta testing.
Update these notes with features that are going to be in `main` (PX4 v1.19 or later) but not the PX4 v1.18 release.
:::

## Прочитайте перед оновленням

Please continue reading for [upgrade instructions](#upgrade-guide).

## Основні зміни

- Уточнюється

## Інструкції для оновлення

## Інші зміни

- Fast mission Return modes ([RTL_TYPE](../advanced_config/parameter_reference.md#RTL_TYPE) = 2 and 4) now skip `DO_JUMP` commands (loops) while following the mission path. ([PX4-Autopilot#26993: fix(navigator): goToNextPositionItem skip loops when required](https://github.com/PX4/PX4-Autopilot/pull/26993))

### Підтримка обладнання

- Уточнюється

### Загальні

- Уточнюється

### Управління

- Уточнюється

### Безпека

- Уточнюється

### Оцінки

- Уточнюється

### Датчики

- Уточнюється

### Симуляція

- Уточнюється

### Debug & Logging

- Уточнюється

### Ethernet

- Уточнюється

### uXRCE-DDS / Zenoh / ROS2

- Уточнюється

### MAVLink

- Уточнюється

### RC

- Уточнюється

### Мульти-Ротор

- Уточнюється

### VTOL

- Уточнюється

### Fixed-wing

- Уточнюється

### Rover

- Уточнюється

### ROS 2

- Уточнюється
