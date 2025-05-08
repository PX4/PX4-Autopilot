# 振动隔离

本主题展示了如何确定振动水平是否过高，并列出了一些改善振动特征的简单步骤。

## 综述

Flight Control boards with in-built accelerometers or gyros are sensitive to vibrations.
高振动会引发一系列问题，包含飞行效率或性能降低，缩短飞行时间，增加机体磨损。 在极端情况下，振动可能会引发传感器削波失真/故障，可能导致姿态估算失败，失控。

设计很好的机身会在自驾仪安装的位置处减少特定结构共振的幅度。
为了将振动降低到敏感元器件能够处理的程度，进一步的隔离是需要的。（例如 ，一些飞控必须使用某种防震泡沫安装固定在机身上，其他的是一些内部隔离）。

## 振动分析

[Log Analysis using Flight Review > Vibration](../log/flight_review.md#vibration) explains how to use logs to confirm whether vibration is a probable cause of flight problems.

## 基本振动修复

可以减少振动的一些简单步骤：

- 确保所有的东西都可靠的固定在机身上（起落架，GPS 天线等）。
- 使用平衡螺旋桨。
- 确保使用高质量的螺旋桨、发动机、电调和机架。
  这些组成部分中的每一个都有很大的不同。
- 使用隔振方法安装自动驾驶仪。
  Many flight controllers come with _mounting foam_ that you can use for this purpose, while others have inbuilt vibration-isolation mechanisms.
- As a _last_ measure, adjust the [software filters](../config_mc/filter_tuning.md).
  最好是减少振动源，而不是在软件中过滤。

## 参考

一些可能对您有用的参考资料：

- [An Introduction to Shock & Vibration Response Spectra, Tom Irvine](http://www.vibrationdata.com/tutorials2/srs_intr.pdf) (free paper)
- Structural Dynamics and Vibration in Practice - An Engineering Handbook, Douglas Thorby (preview).
