# 机体类型与设置

PX4 支持多种类型的机体，包括不同配置的多旋翼、固定翼、垂直起降、地面车辆等。

本节解释如何对每种类型进行组装、配置和调整 PX4 基础自动化系统 (大部分的配置是通用的)。

:::info
[基本概念 > 无人机类型](../getting_started/px4_basic_concepts.md#drone-types) 提供了不同类型的机体以及相关机体的最佳使用场景的高级信息。
:::

## 支持的载具

有维护者且经过良好测试和支持的机架类型是：

- [多旋翼](../frames_multicopter/index.md) (三轴， 四轴，十六轴，八轴，甚至 [全向多旋翼](../frames_multicopter/omnicopter.md) 机体)
- [飞机 (固定翼)](../frames_plane/index.md)
- [VTOL](../frames_vtol/index.md): [Standard VTOL](../frames_vtol/standardvtol.md), [Tailsitter VTOL](../frames_vtol/tailsitter.md), [Tiltrotor VTOL](../frames_vtol/tiltrotor.md)

## 实验机体

实验机架指以下机体种类：

- 没有维护者。
- 核心开发小组没有定期进行测试。
- 可能没有 CI 测试。
- 可能缺少量产机体所需的功能。
- 可能不支持该机体类型的某些通用配置。

以下载具类型被认为是试验性的：

- [飞艇](../frames_airship/index.md)
- [自旋翼机](../frames_autogyro/index.md)
- [热气球](../frames_balloon/index.md)
- [直升机](../frames_helicopter/index.md)
- [无人车](../frames_rover/index.md)
- [潜艇](../frames_sub/index.md)

:::info
Maintainer volunteers, [contribution](../contribute/index.md) of new features, new frame configurations, or other improvements would all be very welcome!
:::

## 其他载具

全部可支持的机型以及相关配置可见 [机架参考](../airframes/airframe_reference.md)。
