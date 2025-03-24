# 多旋翼配置

多旋翼的配置和校准遵循与其他框架相同的高级步骤：选择固件，配置框架，包括执行器/电机几何和输出映射，传感器配置和校准，安全和其他功能的配置，最后进行调整。

This topic explains how to configure a multicopter using selected topics from [Standard Configuration](../config/index.md), [Advanced Configuration](../advanced_config/index.md), and [Flight Controller Peripherals](../peripherals/index.md), along with multicopter-specific tuning topics.

:::info
This topic is the recommended entry point when performing first-time configuration and calibration of a new multicopter frame.
:::

## 加载固件

The first step is to [load PX4 firmware](../config/firmware.md) onto your [flight controller](../flight_controller/index.md).
这一步最容易通过使用 QGroundControl 完成，它会自动选择适合您特定控制器硬件的固件。
默认情况下，QGC 会安装最新的稳定版本的 PX4，但如果需要，您可以选择测试版或自定义版本。

相关章节：

- [Loading Firmware](../config/firmware.md)

## 机架选择和配置

This section explains how to configure the vehicle type (multicopter), specific motor/flight control geometry, and motor outputs.

First [select a multicopter airframe](../config/airframe.md) (options are listed in [Airframe Reference > Copter](../airframes/airframe_reference.md#copter)).
You should select the frame that matches your vehicle brand and model if one exists, and otherwise select the "Generic" frame type that most closely matches your geometry in terms of number of motors and their relative positions.
For example, for a [Quadrotor X](../airframes/airframe_reference.md#quadrotor-x) frame you would look for the name of your frame in the list, and if it was not present select the [Generic Quadrotor X](../airframes/airframe_reference.md#copter_quadrotor_x_generic_quadcopter) frame.

:::info
Any selected multicopter frame can be modified in the next step (actuator configuration) to add/remove motors and otherwise change the geometry, and to specify what flight controller outputs are connected to particular motors and the output properties.
Selecting a frame that matches your vehicle reduces the configuration work required.

:::details
How does this work (details)
Selecting an airframe applies a [frame configuration file](../dev_airframes/adding_a_new_frame.md#adding-a-frame-configuration) that contains a predefined set of [parameters](../advanced_config/parameters.md), such as [CA_AIRFRAME=0](../advanced_config/parameter_reference.md#CA_AIRFRAME) for the vehicle type and [CA_ROTOR_COUNT](../advanced_config/parameter_reference.md#CA_ROTOR_COUNT) for the number of rotors.

A frame configuration can define everything about a vehicle, from it's geometry and output mappings, through to its tuning and calibration values.
When you're bringing up a new vehicle though, the frame will usually contain a fairly minimal configuration:

- Frames named with "Generic" define the vehicle type, number of rotors, and "placeholder" rotor positions.
 After selecting the airframe you define the actual geometry and then configure outputs.
- Frames named with model/brand will define the vehicle type, number of rotors, actual rotor positions, and motor directions.
 After selecting the airframe you usually still have to configure outputs.

:::

The next step is to define your vehicle [geometry](../config/actuators.md#motor-geometry-multicopter) (the number of motors and their relative positions) and [assign those motors](../config/actuators.md#actuator-outputs) to the physical outputs that they are wired to on your flight controller (both of these are covered in [Actuator Configuration and Testing](../config/actuators.md)).

If using PWM ESCs and OneShot ESCs (but not DShot and DroneCAN/Cyphal ESC) you should then perform [ESC Calibration](../advanced_config/esc_calibration.md) before proceeding to [Motor Configuration](../config/actuators.md#motor-configuration).
This ensures that all ESC provide exactly the same output for a given input (ideally we'd calibrate ESCs first, but you can't calibrate your ESCs until outputs are mapped).

The final step is [Motor Configuration](../config/actuators.md#motor-configuration):

- [Reverse any motors](../config/actuators.md#reversing-motors) that don't match the spin direction configured in the Geometry.
 For DShot ESC you can do this through the Acuator Testing UI.
- PWM, OneShot, and CAN ESC, set the motor input limits for disarmed, low and high speed (not needed for DShot ESC)

相关章节：

- [Vehicle (Frame) Selection](../config/airframe.md) — Select vehicle type to match your frame.
- [Actuator Configuration and Testing](../config/actuators.md) — Vehicle geometry, output mapping, motor configuration, testing.
- [ESC Calibration](../advanced_config/esc_calibration.md) — Do between output mapping and motor configuration (topic above) for PWM and OneShot ESC.

## 传感器设置和校准

PX4 most commonly relies on a magnetometer (compass) for direction information, a barometer for altitude, a gyroscope for body rates, an accelerometer for attitude and a GPS/GNSS for global position.
Pixhawk flight controllers (and many others) have inbuilt magnetometer, accelerometer, gyroscope, and barometer.
The inbuilt compass usually isn't particularly reliable, and it is common to also add an external compass (usually combined with a GNSS receiver in the same device).

We first need to set the [Sensor Orientation](../config/flight_controller_orientation.md), informing PX4 how the autopilot (and its inbuilt sensors) and external compasses are oriented relative to the vehicle.
Generally you'll orient towards the front of the vehicle and not have to set anything.
Once that is done we need to calibrate the compass(es), gyroscope, and accelerometer.

The core sensor setup is covered in these topics:

- [Sensor Orientation](../config/flight_controller_orientation.md)
- [Compass](../config/compass.md)
- [Gyroscope](../config/gyroscope.md)
- [Accelerometer](../config/accelerometer.md)

PX4 can use other peripherals, such as distance sensors, optical flow sensors, traffic avoidance alarms, cameras, and so on:

- [飞控外设](../peripherals/index.md) - 设置特定传感器、可选传感器、执行器等。

:::info
Sensors that you don't need to calibrate/configure include:

- [Level Horizon](../config/level_horizon_calibration.md) calibration isn't usually needed if you have mounted the flight controller level.
- Sensors that are not present, or that are not used by PX4 multicopter for flight control, such as [Airspeed sensors](../config/airspeed.md).
- Sensors that don't need calibration, including: Barometers and GNSS.

:::

## 手动控制设置

Pilots can control a vehicle manually using either a Radio Control (RC) System or a Joystick/Gamepad controller connected via QGroundControl.

:::info
A manual control is essential in order to bring up a new vehicle safely!
:::

Radio Control:

- [Radio Controller (RC) Setup](../config/radio.md)
- [Flight Mode Configuration](../config/flight_mode.md)

Joystick/GamePad:

- [Joystick Setup](../config/joystick.md) (includes button/flight mode mapping)

## 安全配置

PX4 can be configured to automatically handle conditions such as low battery, losing radio or data links, flying too far from the home location, and so on:

- [Battery Estimation Tuning](../config/battery.md) — estimate remaining power (needed for low power failsafe).
- [Safety Configuration (Failsafes)](../config/safety.md)

## 调试

Tuning is the final step, carried out only after most other setup and configuration is complete.

- Rate and attitude controllers:

- [Autotune](../config/autotune_mc.md) — Automates tuning PX4 rate and attitude controllers (recommended).

 ::: info
 Automatic tuning works on frames that have reasonable authority and dynamics around all the body axes.
 It has primarily been tested on racing quads and X500, and is expected to be less effective on tricopters with a tiltable rotor.

 Manual tuning using these guides are only needed if there is a problem with autotune:

 - [MC PID Tuning (Manual/Basic)](../config_mc/pid_tuning_guide_multicopter_basic.md) — Manual tuning basic how to.
 - [MC PID Tuning Guide (Manual/Detailed)](../config_mc/pid_tuning_guide_multicopter.md) — Manual tuning with detailed explanation.


:::

- [MC Filter/Control Latency Tuning](../config_mc/filter_tuning.md) — Trade off control latency and noise filtering.

- [MC Setpoint Tuning (Trajectory Generator)](../config_mc/mc_trajectory_tuning.md)
 - [MC Jerk-limited Type Trajectory](../config_mc/mc_jerk_limited_type_trajectory.md)

- [Multicopter Racer Setup](../config_mc/racer_setup.md)

<!--
- Explain what you have to tune on PX4, what you can tune, and what each topic covers
- I expect we should start with an exhaustive list of the tuning you could want to do - such as position tuning, etc. Do we have one?
 -->

<!-- TBD this is just text for me to mine

AFAIK autotune was tested on various not so custom platforms e.g. X500, racer quad, Loong standard VTOL. I honestly used it only once on a tricopter and it worked for roll and pitch but the resulting yaw tuning was not stable. Since then it was improved but that's not merged yet :eyes: https://github.com/PX4/PX4-Autopilot/pull/21857
Autotune was never tested on a Helicopter.
can you in theory autotune frame with any number of motors?
In theory yes but it needs to be able to have reasonable authority around all axes so I'd expect autotune to not work well for a monocopter without swashplate and so on. Probably also the controllers wouldn't work out of the box. I saw issues before with designs that tilt the rotor e.g. tricopter, bicopter, ... again


will PX4 still understand how to autotune?
Autotune should work for any vehicle that has reasonable authority and dynamics around all the body axes. A tiltable motor e.g. tricopter has at the least dynamics which are less tested with autotune.
My assumption is that the mixing system can cope with whatever geometry you throw at it.
Yes but it must be physically feasible. E.g. if you make a quadrotor where all motors turn the same way it will "deal" with it but that cannot work without very specific controllers. Same for a monocopter or a tricopter without swiveling one motor.
-->

## See Also

- [QGroundControl > Setup](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/setup_view.html)
- [飞控外设](../peripherals/index.md) - 设置特定传感器、可选传感器、执行器等。
- [Advanced Configuration](../advanced_config/index.md) - Factory/OEM calibration, configuring advanced features, less-common configuration.
- Vehicle-Centric Config/Tuning:
 - **Multicopter Config/Tuning**
 - [直升机配置/调参](../config_heli/index.md)
 - [Fixed Wing Config/Tuning](../config_fw/index.md)
 - [VTOL 配置/调参](../config_vtol/index.md)
