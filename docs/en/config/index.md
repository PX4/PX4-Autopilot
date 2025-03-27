# Standard Configuration

This section covers the common software configuration and calibration needed for most PX4 vehicles.

You must first [load firmware and select your vehicle frame/type](#firmware-vehicle-selection).
Most other steps can be done out of order, except for [tuning](#tuning), which must be done last.

## Preconditions

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

::: info
Setup for these and other sensors is located in [Sensor Hardware & Setup](../sensor/index.md).
:::

### Manual Control Setup

Radio Control:

- [Radio Controller (RC) Setup](../config/radio.md)
- [Flight Mode Configuration](../config/flight_mode.md)

Joystick/GamePad:

- [Joystick Setup](../config/joystick.md)

### Safety Configuration

- [Battery Estimation Tuning](../config/battery.md) (requires [Power Module](../power_module/index.md))
- [Safety Configuration (Failsafes)](../config/safety.md)

### Tuning

Auto-tuning is supported, and recommended, on the following frames:

- [Autotune (Multicopter)](../config/autotune_mc.md) 
- [Autotune (Fixed Wing)](../config/autotune_fw.md)
- [Autotune (VTOL)](../config/autotune_vtol.md)

## Video Guide

The video below shows most of the calibration process (it uses an older version of _QGroundControl_, but most of the process is unchanged).

<lite-youtube videoid="91VGmdSlbo4" title="PX4 Autopilot Setup Tutorial Preview"/>

## Support

If you need help with the configuration you can ask for help on the [QGroundControl Support forum](https://discuss.px4.io//c/qgroundcontrol/qgroundcontrol-usage).

## See Also

- [QGroundControl > Setup](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/setup_view.html)
- [Flight Controller Peripherals](../peripherals/index.md) - Setup specific sensors, optional sensors, actuators, and so on.
- [Advanced Configuration](../advanced_config/index.md) - Factory/OEM calibration, configuring advanced features, less-common configuration.
- Vehicle-Centric Config/Tuning:

  - [Multicopter Config/Tuning](../config_mc/index.md)
  - [Helicopter Config/Tuning](../config_heli/index.md)
  - [Fixed-wing Config/Tuning](../config_fw/index.md)
  - [VTOL Config/Tuning](../config_vtol/index.md)
