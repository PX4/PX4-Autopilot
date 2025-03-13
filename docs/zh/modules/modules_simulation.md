# 模块参考：仿真

## simulator_sih

Source: [modules/simulation/simulator_sih](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/simulation/simulator_sih)

### 描述

This module provides a simulator for quadrotors and fixed-wings running fully
inside the hardware autopilot.

This simulator subscribes to "actuator_outputs" which are the actuator pwm
signals given by the control allocation module.

模拟器发布了被真实噪声污染的传感器信号以便在环路中加入状态估计器。

### 实现

模拟器运用矩阵代数方法实现了运动方程。
姿态采用四元数表示。
积分计算采用前向欧拉法。
为避免堆栈溢出，大部分变量在 .hpp 文件中声明为全局变量。

<a id="simulator_sih_usage"></a>

### 用法

```
simulator_sih <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```
