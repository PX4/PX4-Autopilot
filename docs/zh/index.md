<script setup>
import { useData } from 'vitepress'
const { site } = useData();
</script>

<div style="float:right; padding:10px; margin-right:20px;"><a href="https://px4.io/"><img src="../assets/site/logo_pro_small.png" title="PX4 Logo" width="180px" /></a></div>

# PX4 自动驾驶仪用户指南

[![Releases](https://img.shields.io/badge/release-main-blue.svg)](https://github.com/PX4/PX4-Autopilot/releases) [![Discuss](https://img.shields.io/badge/discuss-px4-ff69b4.svg)](https://discuss.px4.io//) [![Discord](https://discordapp.com/api/guilds/1022170275984457759/widget.png?style=shield)](https://discord.gg/dronecode)

_PX4_ 是一款专业级飞控。
它由来自业界和学术界的世界级开发商开发，并得到活跃的全球社区的支持，为从竞速和物流无人机到地面车辆和潜水艇的各种载具提供动力。

:::tip
这份指南包含组装、配置、安全使用 PX4 的设备的各种只是。
对贡献感兴趣吗 查看 [Development](development/development.md) 部分。
:::

<div v-if="site.title == 'PX4 Guide (main)'">

:::warning

本指南适用于_development_ version of PX4 (`main` 分支)。
使用 **版本** 选择器查找当前的 _稳定_ 版本。

自稳定版本发布以来的已记录变更，收录在不断更新的(releases/main.md ) 中。
:::

</div>

## 如何开始？

所有用户都应该先阅读[基本概念](getting_started/px4_basic_concepts.md) ！
它概述了PX4，包括由飞行堆栈提供的功能（飞行模式和安全特征）和支持的硬件（飞行控制器、载具类型、数传系统、遥控控制系统）。

根据您想要实现的目标，以下提示将帮助您浏览本指南：

### 我想要一个能与PX4配合使用的载具

在 [多旋翼](frames_multicopter/index.md), [VTOL](frames_vtol/index.md), 和 [平面(固定翼)](frames_plane/index.md)部分你会找到如下主题（这些链接针对多旋翼飞行器）：

- [完整的载具](complete_vehicles_mc/index.md)列出了到手飞(RTF)的硬件
- [套件(frames_multicopter/kits.md) 列出了需要你利用一组预先选定的部件自行组装的无人机。
- [DIY 组装](frames_multicopter/diy_builds.md) 展示了一些使用单独采购的零部件组装而成的无人机示例。

无论是套件还是成品飞行器，通常都包含你所需的一切，除了电池和遥控系统。
套装通常不难建造，可以很好地介绍无人机如何合在一起，而且费用相对较低。
我们提供了一般的组装指示，例如[组装一个多旋翼机](assembly/assembly_mc.md)，大多数套装也附有具体的指示。

如果套件和成品无人机不太符合你的需求，那么你可以从零开始打造一架飞行器，但这需要更多专业知识。
[机身构建](airframes/index.md) 列出了受支持的机身起点，让你了解哪些方案是可行的。

一旦你拥有支持PX4的载具，你将需要配置它并校准传感器。
每种飞行器类型都有其专属的配置章节，阐述主要步骤，比如[多旋翼飞行器配置 / 调校](config_mc/index.md))。

### 我想添加一个有效载荷/相机

[有效载荷](payloads/index.md部分描述了如何添加相机，以及如何配置 PX4 以实现交付包裹。

### 我正在修改一个支持的载具

[硬件选择和设置](hardware/drone_parts.md)部分提供了关于您可能使用 PX4 的硬件及其配置的高层次和特定产品信息。
如果你想改装无人机并添加新组件，这里是你首先应该查看的地方。

### 我想飞行

在飞行之前，您应该阅读 [Operations](config/operations.md)来了解如何设置您的载具的安全性能和所有机型的常见特性。
完成后，你可以准备飞行。

每种飞行器类型的基本飞行说明在各自对应部分给出，例如[基本飞行（多旋翼飞行器）](flying/basic_flying_mc.md)。

### 我想在一个新的飞行控制器上运行 PX4并扩展平台

[开发](development/development.md)部分解释了如何支持新的机体和车辆类型，修改飞行算法添加新模式，整合新的硬件，与飞行控制器外的 PX4 进行沟通，并为PX4 作出贡献。

## 获取帮助

[支持](contribute/support.md页解释了如何从核心开发团队和更广泛的社区获得帮助。

除此以外，它还包括了：

- [您可以获得帮助的论坛](contribute/support.md#forums-and-chat)
- [诊断问题](contribute/support.md#diagnosing-problems)
- [如何报告bug](contribute/support.md#issue-bug-reporting)
- [每周开发会议](contribute/support.md#weekly-dev-call)

## 报告错误和问题

如果您在使用 PX4 首次发布时遇到任何问题，请在[支持论坛](contribute/support.md#forums-and-chat) 上(因为它们可能是由载具配置引起)。

如果开发团队指示，代码问题可以在 [Github](https://github.com/PX4/PX4-Autopilot/issues) 上提出。
尽可能提供 [飞行日志](getting_started/flight_reporting.md) 和问题模板中要求的其他信息。

## 参与贡献

如何贡献代码和文档的信息可以在 [贡献](contribute/index.md部分中找到：

- [代码](contribute/index.md)
- [Documentation](contribute/docs.md)
- [Translation](contribute/translation.md)

## 翻译

本指南有多种 [译文](contribute/translation.md)。
您可以从语言菜单中访问到它们 （右上角）：

![语言选择器](../assets/vuepress/language_selector.png)

<!--@include: _contributors.md-->

## 许可证

PX4 code is free to use and modify under the terms of the permissive [BSD 3-clause license](https://opensource.org/license/BSD-3-Clause).
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

<img src="../assets/site/automatic_mode.svg" title="Automatic mode" width="30px" /> _camera-automatic-mode_ icon made by <a href="https://www.freepik.com" title="Freepik">Freepik</a> from <a href="https://www.flaticon.com/" title="Flaticon">www.flaticon.com</a> is licensed by <a href="https://creativecommons.org/licenses/by/3.0/" title="Creative Commons BY 3.0" target="_blank">CC 3.0 BY</a>.

## 治理

The PX4 flight stack is hosted under the governance of the [Dronecode Project](https://dronecode.org/).

<a href="https://dronecode.org/" style="padding:20px" ><img src="../assets/site/logo_dronecode.png" alt="Dronecode Logo" width="110px"/></a> <a href="https://www.linuxfoundation.org/projects" style="padding:20px;"><img src="../assets/site/logo_linux_foundation.png" alt="Linux Foundation Logo" width="80px" /></a>

<div style="padding:10px">&nbsp;</div>

文档构建时间：{{ $buildTime }}
