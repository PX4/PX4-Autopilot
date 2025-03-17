# 标准配置

This section covers the common software configuration and calibration needed for most PX4 vehicles.

You must first [load firmware and select your vehicle frame/type](#firmware-vehicle-selection).
Most other steps can be done out of order, except for [tuning](#tuning), which must be done last.

## 操作前提

Before starting you should [Download QGroundControl](http://qgroundcontrol.com/downloads/) and install it on your **desktop** computer.
Then open the QGC application menu ("Q" icon in the top-left corner) and choose **Vehicle Setup** in the _Select Tool_ popup:

![QGC Main Menu Popup: highlighting Vehicle Setup](../../assets/qgc/setup/menu_setup.png)

## Configuration Steps

### Firmware/Vehicle Selection

- [Loading Firmware](../config/firmware.md)
- [Vehicle (Frame) Selection](../config/airframe.md)

### Motor/Actuator Setup

- [ESC Calibration](../advanced_config/esc_calibration.md)
- [Actuator Configuration and Testing](../config/actuators.md)

### Sensor Calibration

- [Sensor Orientation](../config/flight_controller_orientation.md)
- [Magnetometer (Compass)](../config/compass.md)
- [Gyroscope](../config/gyroscope.md)
- [Accelerometer](../config/accelerometer.md)
- [Level Horizon](../config/level_horizon_calibration.md)
- [Airspeed](../config/airspeed.md) (Fixed-wing/VTOL only)

:::info
Setup for these and other sensors is located in [Sensor Hardware & Setup](../sensor/index.md).
:::

### 手动控制设置

Radio Control:

- [Radio Controller (RC) Setup](../config/radio.md)
- [Flight Mode Configuration](../config/flight_mode.md)

Joystick/GamePad:

- [Joystick Setup](../config/joystick.md)

### 安全配置

- [Battery Estimation Tuning](../config/battery.md) (requires [Power Module](../power_module/index.md))
- [Safety Configuration (Failsafes)](../config/safety.md)

### 调试

Auto-tuning is supported, and recommended, on the following frames:

- [Autotune (Multicopter)](../config/autotune_mc.md)
- [Autotune (Fixed Wing)](../config/autotune_fw.md)
- [Autotune (VTOL)](../config/autotune_vtol.md)

## 一键安装脚本

The video below shows most of the calibration process (it uses an older version of _QGroundControl_, but most of the process is unchanged).

<lite-youtube videoid="91VGmdSlbo4" title="PX4 Autopilot Setup Tutorial Preview"/>

## 技术支持

If you need help with the configuration you can ask for help on the [QGroundControl Support forum](https://discuss.px4.io//c/qgroundcontrol/qgroundcontrol-usage).

## See Also

- [QGroundControl > Setup](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/setup_view.html)
- [飞控外设](../peripherals/index.md) - 设置特定传感器、可选传感器、执行器等。
- [Advanced Configuration](../advanced_config/index.md) - Factory/OEM calibration, configuring advanced features, less-common configuration.
- Vehicle-Centric Config/Tuning:

  - [多旋翼配置/调参](../config_mc/index.md)
  - [直升机配置/调参](../config_heli/index.md)
  - [固定翼配置/调参](../config_fw/index.md)
  - [VTOL 配置/调参](../config_vtol/index.md)
