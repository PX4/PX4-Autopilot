# 模块参考：仿真

:::info
**This page is [auto-generated](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/px4moduledoc/markdownout.py) from the source code** using the build command: `make module_documentation`.
Do not edit it directly; fix the module documentation in the [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) source instead.
:::

## simulator_sih

Source: [modules/simulation/simulator_sih](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/simulation/simulator_sih)

### 描述

This module provides a simulator for quadrotors and fixed-wings running fully
inside the hardware autopilot.

This simulator subscribes to "actuator_outputs" which are the actuator pwm
signals given by the control allocation module.

模拟器发布了被真实噪声污染的传感器信号以便在环路中加入状态估计器。

### 实现

模拟器运用矩阵代数方法实现了运动方程。姿态采用四元数表示。积分计算采用前向欧拉法。为避免堆栈溢出，大部分变量在 .hpp 文件中声明为全局变量。

### 示例

Pose the vehicle on the ground for accel and mag calibration (SITL only), sides named as the calibration names them.
With `rotate` it turns about the vertical with a tilt swing, as the mag calibration asks for:

```
simulator_sih orient left rotate
```

Each side in turn: 10 s still, then 10 s rotating, then level. This completes either calibration.

```
simulator_sih orient all
```

Level the vehicle and resume the flight dynamics:

```
simulator_sih orient off
```

### Usage {#simulator_sih_usage}

```
simulator_sih <command> [arguments...]
 Commands:
   start
     [-t <val>]  Terrain elevation GeoTIFF (DEM), SITL only
                 values: <file>

   orient        Pose the vehicle on the ground for calibration
     down|up|front|back|left|right|all|off Side facing down, every side in turn,
                 or level
     [rotate]    Rotate about the vertical

   stop

   status        print status info
```
