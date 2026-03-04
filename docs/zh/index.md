<script setup>
import { useData } from 'vitepress'
const { site } = useData();
</script>

# PX4 自动驾驶仪用户指南

[![Releases](https://img.shields.io/badge/release-main-blue.svg)](https://github.com/PX4/PX4-Autopilot/releases) [![Discuss](https://img.shields.io/badge/discuss-px4-ff69b4.svg)](https://discuss.px4.io//) [![Discord](https://discordapp.com/api/guilds/1022170275984457759/widget.png?style=shield)](https://discord.gg/dronecode)

PX4 is an open-source autopilot for drones and autonomous vehicles. It runs on multirotors, fixed-wing, VTOL, helicopters, rovers, and more. This guide covers everything from assembly and configuration to flight operations and development.

<div v-if="site.title == 'PX4 Guide (main)'">

:::warning

本指南适用于_development_ version of PX4 (`main` 分支)。
使用 **版本** 选择器查找当前的 _稳定_ 版本。

自稳定版本发布以来的已记录变更，收录在不断更新的(releases/main.md ) 中。
:::

</div>

## For Developers

:::tip
Building on PX4 or extending the platform? Start here: [Development Guide](development/development.md). Set up your [dev environment](dev_setup/config_initial.md), [build from source](dev_setup/building_px4.md), run [SITL simulation](simulation/index.md), or integrate via [ROS 2](ros2/index.md) and [MAVSDK](https://mavsdk.mavlink.io/).
:::

## 入门指南

Start with [Basic Concepts](getting_started/px4_basic_concepts.md) for an overview of the flight stack, flight modes, safety features, and supported hardware.

## Build a Vehicle

Pick your frame type: [Multicopter](frames_multicopter/index.md), [Fixed-Wing](frames_plane/index.md), [VTOL](frames_vtol/index.md), [Helicopter](frames_helicopter/index.md), or [Rover](frames_rover/index.md). Each section covers complete vehicles, kits, and DIY builds. For assembly instructions see [Assembling a Multicopter](assembly/assembly_mc.md) or the equivalent for your frame.

## Configure and Tune

Once assembled, follow the configuration guide for your vehicle type (e.g. [Multicopter Configuration](config_mc/index.md)). This covers sensor calibration, flight mode setup, and tuning.

## 硬件

The [Hardware Selection & Setup](hardware/drone_parts.md) section covers flight controllers, sensors, telemetry, RC systems, and payloads. See [Payloads](payloads/index.md) for camera and delivery integrations.

## Fly

Read [Operations](config/operations.md) to understand safety features and failsafe behavior before your first flight. Then see [Basic Flying (Multicopter)](flying/basic_flying_mc.md) or the equivalent for your frame type.

## 技术支持

Get help on the [discussion forums](https://discuss.px4.io/) or [Discord](https://discord.gg/dronecode). See the [Support](contribute/support.md) page for diagnosing problems, reporting bugs, and joining the [weekly dev call](contribute/dev_call.md).

## 参与贡献

See the [Contributing](contribute/index.md) section for code, [documentation](contribute/docs.md), and [translation](contribute/translation.md) guidelines.

## 翻译

本指南有多种 [译文](contribute/translation.md)。 Use the language selector in the top navigation.

<!--@include: _contributors.md-->

## 许可证

PX4 代码可依据宽松的 [BSD 3-clause license](https://opensource.org/license/BSD-3-Clause) 免费使用和修改。
此文档已使用 [CC BY 4.0]授权。(https://creativecommons.org/licenses/by/4.0/)。
详情见： [Licences](contribute/licenses.md)。

## 日历和活动

_Dronecode 日历_ 展示了面向平台用户和开发者的重要社区活动。
选择以下链接将其显示在您所在的时区日历中(并将其添加到您自己的日历中)：

- [Switzerland – Zurich](https://calendar.google.com/calendar/embed?src=linuxfoundation.org_g21tvam24m7pm7jhev01bvlqh8%40group.calendar.google.com&ctz=Europe%2FZurich)
- [Pacific Time – Tijuana](https://calendar.google.com/calendar/embed?src=linuxfoundation.org_g21tvam24m7pm7jhev01bvlqh8%40group.calendar.google.com&ctz=America%2FTijuana)
- [Australia – Melbourne/Sydney/Hobart](https://calendar.google.com/calendar/embed?src=linuxfoundation.org_g21tvam24m7pm7jhev01bvlqh8%40group.calendar.google.com&ctz=Australia%2FSydney)

:::tip
日历的默认时区为中欧时间（CET）。

:::

<iframe src="https://calendar.google.com/calendar/embed?title=Dronecode%20Calendar&amp;mode=WEEK&amp;height=600&amp;wkst=1&amp;bgcolor=%23FFFFFF&amp;src=linuxfoundation.org_g21tvam24m7pm7jhev01bvlqh8%40group.calendar.google.com&amp;color=%23691426&amp;ctz=Europe%2FZurich" style="border-width:0" width="800" height="600" frameborder="0" scrolling="no"></iframe>

### 图标

此库中使用的以下图标是单独授权的（如下所示）：

<img src="../assets/site/position_fixed.svg" title="Position fix required (e.g. GPS)" width="30px" /> _placeholder_icon 由 <a href="https://www.flaticon.com/authors/smashicons" title="Smashicons">Smashicons</a> 通过 <a href="https://www.flaticon.com/" title="Flaticon">www.flaticon.com</a> 创作，使用 <a href="https://creativecommons.org/licenses/by/3.0/" title="Creative Commons BY 3.0" target="_blank">CC 3.0 By</a> 授权。

<img src="../assets/site/automatic_mode.svg" title="Automatic mode" width="30px" /> _camera-automatic-mode_ 图标由 <a href="https://www.freepik.com" title="Freepik">Freepik</a> 从 <a href="https://www.flaticon.com/" title="Flaticon">www.flaticon.com</a> 是由 <a href="https://creativecommons.org/licenses/by/3.0/" title="Creative Commons BY 3.0" target="_blank">CC 3.0 By</a> 授权的。

## 治理

The PX4 Autopilot project is hosted by the [Dronecode Foundation](https://www.dronecode.org/), a [Linux Foundation](https://www.linuxfoundation.org/) Collaborative Project. Dronecode holds all PX4 trademarks and serves as the project's legal guardian, ensuring vendor-neutral stewardship. No single company owns the name or controls the roadmap. The source code is licensed under the [BSD 3-Clause](https://opensource.org/license/BSD-3-Clause) license, so you are free to use, modify, and distribute it in your own projects.

<a href="https://www.dronecode.org/" style="padding:20px"><img src="../assets/site/dronecode_logo.svg" alt="Dronecode Logo" width="140px"/></a> <a href="https://www.linuxfoundation.org/projects" style="padding:20px;"><img src="../assets/site/logo_linux_foundation.png" alt="Linux Foundation Logo" width="80px" /></a>

<div style="padding:10px">&nbsp;</div>

文档构建时间：{{ $buildTime }}
