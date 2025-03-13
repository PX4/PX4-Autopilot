# 基本概念

This topic provides a basic introduction to drones and using PX4 (it is meant mostly for novice users but is also a good introduction for more experienced users).

If you are already familiar with the basic concepts, you can move on to [Basic Assembly](../assembly/index.md) to learn how to wire your specific autopilot hardware.
To load firmware and set up the vehicle with _QGroundControl_, see [Basic Configuration](../config/index.md).

## 无人机是什么？

A drone, or Unmanned Vehicles (UV), is an unmanned "robotic" vehicle that can be manually or autonomously controlled.
They can travel in air, on the ground, on/under the water, and are used for many [consumer, industrial, government and military applications](https://px4.io/ecosystem/commercial-systems/), including aerial photography/video, carrying cargo, racing, search and surveying, and so on.

Drones are more formally referred to as Unmanned Aerial Vehicles (UAV), Unmanned Ground Vehicles (UGV), Unmanned Surface Vehicles (USV), Unmanned Underwater Vehicles (UUV).

:::info
The term Unmanned Aerial System (UAS) typically refers to a UAV and all of the other components of a complete system, including a ground control station and/or radio controller, and any other systems used to control the drone, capture, and process data.
:::

## Drone Types

There are many different vehicle frames (types), and within the types there are many variations.
Some of the types, along with the use cases for which they are most suited are listed below.

- [Multicopters](../frames_multicopter/index.md) — Multi-rotors offer precision hovering and vertical takeoff, at the cost of shorter and generally slower flight.
  They are the most popular type of flying vehicle, in part because they are easy to assemble, and PX4 has modes that make them easy to fly and very suitable as a camera platform.
- [Helicopters](../frames_helicopter/index.md) — Helicopters similar benefits to Multicopters but are mechanically more complex and more efficient.
  They are also much harder to fly.
- [Planes (Fixed-wing)](../frames_plane/index.md) — Fixed-wing vehicles offer longer and faster flight than multicopters, and hence better coverage for ground surveys etc.
  However they are harder to fly and land than multicopters, and aren't suitable if you need to hover or fly very slowly (e.g. when surveying vertical structures).
- [VTOL](../frames_vtol/index.md) (Vertical Takeoff and Landing) - Hybrid Fixed-wing/Multicopter vehicles offer the best of both worlds: take off in vertical mode and hover like a multicopter but transition to forward flight like an airplane to cover more ground.
  VTOL are often more expensive than either multicopters and fixed-wing aircraft, and harder to build and tune.
  They come in a number of types: tiltrotors, tailsitters, quadplanes etc.
- [Airships](../frames_airship/index.md)/[Balloons](../frames_balloon/index.md) — Lighter-than-air vehicles that typically offer high altitude long duration flight, often at the cost of having limited (or no) control over speed and direction of flight.
- [Rovers](../frames_rover/index.md) — Car-like ground vehicles.
  They are simple to control and often fun to use.
  They can't travel as fast as most aircraft, but can carry heavier payloads, and don't use much power when still.
- **Boats** — Water-surface vehicles.
- [Submersibles](../frames_sub/index.md) — Underwater vehicles.

有关详细信息，请参阅︰

- [Vehicle Types & Setup](../airframes/index.md)
- [Airframe setup](../config/airframe.md)
- [Airframe Reference](../airframes/airframe_reference.md).

## Autopilots

无人机的“大脑”被称为自动驾驶仪。

It minimally consists of _flight stack_ software running on a real time OS ("RTOS") on _flight controller_ (FC) hardware.
The flight stack provides essential stabilisation and safety features, and usually also some level of pilot assistance for manual flight and automating common tasks, such as taking off, landing, and executing predefined missions.

Some autopilots also include a general-purpose computing system that can provide "higher level" command and control, and that can support more advanced networking, computer vision, and other features.
This might be implemented as a separate [companion computer](#offboard-companion-computer), but in future it is increasingly likely to be a fully integrated component.

## PX4 自动驾驶仪

[PX4](https://px4.io/) is powerful open source autopilot _flight stack_ running on the NuttX RTOS.

PX4的一些主要功能包括：

- Supports many different vehicle frames/types, including: [multicopters](../frames_multicopter/index.md), [fixed-wing aircraft](../frames_plane/index.md) (planes), [VTOLs](../frames_vtol/index.md) (hybrid multicopter/fixed-wing), [ground vehicles](../frames_rover/index.md), and [underwater vehicles](../frames_sub/index.md).
- Great choice of drone components for [flight controller](#flight-controller), [sensors](#sensors), [payloads](#payloads), and other peripherals.
- Flexible and powerful [flight modes](#flight-modes) and [safety features](#safety-settings-failsafe).
- Robust and deep integration with [companion computers](#offboard-companion-computer) and [robotics APIs](../robotics/index.md) such as [ROS 2](../ros2/user_guide.md) and [MAVSDK](http://mavsdk.mavlink.io).

PX4 is a core part of a broader drone platform that includes the [QGroundControl](#qgc) ground station, [Pixhawk hardware](https://pixhawk.org/), and [MAVSDK](http://mavsdk.mavlink.io) for integration with companion computers, cameras and other hardware using the MAVLink protocol.
PX4 is supported by the [Dronecode Project](https://www.dronecode.org/).

## Ground Control Stations

Ground Control Stations (GCS) are ground based systems that allow UV operators to monitor and control a drone and its payloads.
A subset of the products that are known to work with PX4 are listed below.

### QGroundControl {#qgc}

The Dronecode GCS software is called [QGroundControl](http://qgroundcontrol.com/) ("QGC").
It runs on Windows, Android, MacOS or Linux hardware, and supports a wide range of screen form factors.
You can download it (for free) from [here](http://qgroundcontrol.com/downloads/).

![QGC Main Screen](../../assets/concepts/qgc_fly_view.png)

QGroundControl communicates with the drone using a telemetry radio (a bidirectional data link), which allows you to get real-time flight and safety information, and to control the vehicle, camera, and other payloads using a point-and-click interface.
On hardware that supports them, you can also manually fly the vehicle using joystick controllers.
QGC can also be used to visually plan, execute, and monitor autonomous missions, set geofences, and much more.

QGroundControl desktop versions are also used to install (flash) PX4 firmware and configure PX4 on the drone's autopilot/flight controller hardware.

### Auterion Mission Control (AMC) {#amc}

[Auterion Mission Control](https://auterion.com/product/mission-control/) is a powerful and fully featured ground control station application that is optimized for _pilots_ rather than vehicle configuration.
While designed to work with Auterion products, it can be used with "vanilla" PX4.

有关详细信息，请参阅︰

- [AMC docs](https://docs.auterion.com/vehicle-operation/auterion-mission-control)
- [Download from Auterion Suite](https://suite.auterion.com/)

## Drone Components & Parts

### Flight Controller

Flight controllers (FC) are the hardware onto which the PX4 flight stack firmware is loaded and run.
They are connected to sensors from which PX4 determines its state, and to the actuators/motors that it uses to stabilise and move the vehicle.

<img src="../../assets/flight_controller/cuav_pixhawk_v6x/pixhawk_v6x.jpg" width="230px" title="CUAV Pixhawk 6X" >

PX4 can run on many different types of [Flight Controller Hardware](../flight_controller/index.md), ranging from [Pixhawk Series](../flight_controller/pixhawk_series.md) controllers to Linux computers.
These include [Pixhawk Standard](../flight_controller/autopilot_pixhawk_standard.md) and [manufacturer-supported](../flight_controller/autopilot_manufacturer_supported.md) boards.
You should select a board that suits the physical constraints of your vehicle, the activities you wish to perform, and cost.

For more information see: [Flight Controller Selection](flight_controller_selection.md)

### 传感器

PX4 uses sensors to determine vehicle state, which it needs in order to stablise the vehicle and enable autonomous control.
The vehicle states include: position/altitude, heading, speed, airspeed, orientation (attitude), rates of rotation in different axes, battery level, and so on.

PX4 _minimally requires_ a [gyroscope](../sensor/gyroscope.md), [accelerometer](../sensor/accelerometer.md), [magnetometer](../gps_compass/magnetometer.md) (compass) and [barometer](../sensor/barometer.md).
This minimal set of sensors is incorporated into [Pixhawk Series](../flight_controller/pixhawk_series.md) flight controllers (and may also be in other controller platforms).

Additional/external sensors can be attached to the controller.
The following sensors are recommended:

- A [GNSS/GPS](../gps_compass/index.md) or other source of global position is needed to enable all automatic modes, and some manual/assisted modes.

  Typically a module that combines a GNSS and Compass is used, as an external compass can be made less susceptible to electromomagnetic interference than the internal compass in the flight controller.

- [Airspeed sensors](../sensor/airspeed.md) are highly recommended for fixed-wing and VTOL-vehicles.

- [Distance Sensors \(Rangefinders\)](../sensor/rangefinders.md) are highly recommended for all vehicle types, as they allow smoother and more robust landings, and enable features such as terrain following on multicopters.

- [Optical Flow Sensors](../sensor/optical_flow.md) can be used with distance sensors on multcopters and VTOL to support navigation in GNSS-denied environments.

For more information about sensors see: [Sensor Hardware & Setup](../sensor/index.md).

### 输出:电机，舵机，执行器

PX4 uses _outputs_ to control: motor speed (e.g. via [ESC](#escs-motors)), flight surfaces like ailerons and flaps, camera triggers, parachutes, grippers, and many other types of payloads.

The outputs may be PWM ports or DroneCAN nodes (e.g. DroneCAN [motor controllers](../dronecan/escs.md)).
The images below show the PWM output ports for [Pixhawk 4](../flight_controller/pixhawk4.md) and [Pixhawk 4 mini](../flight_controller/pixhawk4_mini.md).

![Pixhawk 4 output ports](../../assets/flight_controller/pixhawk4/pixhawk4_main_aux_ports.jpg) ![Pixhawk4 mini MAIN ports](../../assets/flight_controller/pixhawk4mini/pixhawk4mini_pwm.png)

The outputs are divided into `MAIN` and `AUX` outputs, and individually numbered (i.e. `MAINn` and `AUXn`, where `n` is 1 to usually 6 or 8).
They might also be marked as `IO PWM Out` and `FMU PWM OUT` (or similar).

:::warning
A flight controller may only have `MAIN` PWM outputs (like the _Pixhawk 4 Mini_), or may have only 6 outputs on either `MAIN` or `AUX`.
Ensure that you select a controller that has enough ports/outputs for your [airframe](../airframes/airframe_reference.md).
:::

You can connect almost any output to any motor or other actuator, by assigning the associated function ("Motor 1") to the desired output ("AUX1") in QGroundControl: [Actuator Configuration and Testing](../config/actuators.md).
Note that the functions (motor and control surface actuator positions) for each frame are given in the [Airframe Reference](../airframes/airframe_reference.md).

**Notes:**

- Pixhawk controllers have an FMU board and _may_ have a separate IO board.
  If there is an IO board, the `AUX` ports are connected directly to the FMU and the `MAIN` ports are connected to the IO board.
  Otherwise the `MAIN` ports are connected to the FMU, and there are no `AUX` ports.
- The FMU output ports can use [D-shot](../peripherals/dshot.md) or _One-shot_ protocols (as well as PWM), which provide much lower-latency behaviour.
  这对于竞速机和其他需要更好性能的机体来说是有用的。
- There are only 6-8 outputs in `MAIN` and `AUX` because most flight controllers only have this many PWM/Dshot/Oneshot outputs.
  理论上来说，如果总线支持，可以有更多的输出（比如UAVCAN总线就不限于这几个节点）。

### ESCs & Motors

Many PX4 drones use brushless motors that are driven by the flight controller via an Electronic Speed Controller (ESC)
(the ESC converts a signal from the flight controller to an appropriate level of power delivered to the motor).

有关 PX4 支持的电调和电机的信息，请参阅：

- [ESC & Motors](../peripherals/esc_motors.md)
- [ESC Calibration](../advanced_config/esc_calibration.md)
- [ESC Firmware and Protocols Overview](https://oscarliang.com/esc-firmware-protocols/) (oscarliang.com)

### 电池/电源

PX4无人机最常使用的是锂聚合物（LiPo）电池。
The battery is typically connected to the system using a [Power Module](../power_module/index.md) or _Power Management Board_, which provide separate power for the flight controller and to the ESCs (for the motors).

Information about batteries and battery configuration can be found in [Battery Estimation Tuning](../config/battery.md) and the guides in [Basic Assembly](../assembly/index.md) (e.g. [Pixhawk 4 Wiring Quick Start > Power](../assembly/quick_start_pixhawk4.md#power)).

### Manual Control

Pilots can control a vehicle manually using either a [Radio Control (RC) System](../getting_started/rc_transmitter_receiver.md) or a [Joystick/Gamepad](../config/joystick.md) controller connected via QGroundControl.

![Taranis X9D Transmitter](../../assets/hardware/transmitters/frsky_taranis_x9d_transmitter.jpg) <img src="../../assets/peripherals/joystick/micronav.jpg" alt="Photo of MicroNav, a ground controller with integrated joysticks" width="400px">

RC systems use a dedicated ground-based radio transmitter and vehicle-based receiver for sending control information.
They should always be used when first tuning/testing a new frame design, or when flying racers/acrobatically (and in other cases where low latency is important).

Joystick systems use QGroundControl to encode the control information from a "standard" computer gaming joystick into MAVLink messages, and sent it to the vehicle using the (shared) telemetry radio channel.
They can be used for most manual flight use cases such as taking off, surveys, and so on, provided your telemetry channel has a high enough bandwidth/low latency.

Joysticks are often used in integrated GCS/manual control systems because it is cheaper and easier to integrate a joystick than a separate radio system, and for the majority of use cases, the lower latency does not matter.
一些遥控系统还可以额外接收自动驾驶仪传回的数传信息。

:::info
PX4 does not _require_ a manual control system for autonomous flight modes.
:::

### 安全开关

Vehicles may include a _safety switch_ that must be engaged before the vehicle can be [armed](#arming-and-disarming) (when armed, motors are powered and propellers can turn).

This switch is almost always integrated into the [GPS](../gps_compass/index.md) module that is connected to the Pixhawk `GPS1` port — along with the [buzzer](#buzzer) and [UI LED](#leds).

The switch may be disabled by default, though this depends on the particular flight controller and airframe configuration.
You can disable/enable use of the switch with the [CBRK_IO_SAFETY](../advanced_config/parameter_reference.md#CBRK_IO_SAFETY) parameter.

:::info
Safety switches are optional.
Many argue that it is safer for users never to approach a powered system, even to enable/disable this interlock.
:::

### 蜂鸣器

Vehicles commonly include a buzzer for providing audible notification of vehicle state and readiness to fly (see [Tune meanings](../getting_started/tunes.md)).

This buzzer is almost always integrated into the [GPS](../gps_compass/index.md) module that is connected to the Pixhawk `GPS1` port — along with the [safety switch](#safety-switch) and [UI LED](#leds).
You can disable the notification tunes using the parameter [CBRK_BUZZER](../advanced_config/parameter_reference.md#CBRK_BUZZER).

### LEDs

Vehicles should have a superbright [UI RGB LED](../getting_started/led_meanings.md#ui-led) that indicates the current readiness for flight.

Historically this was included in the flight controller board.
On more recent flight controllers this is almost always an [I2C peripheral](../sensor_bus/i2c_general.md) integrated into the [GPS](../gps_compass/index.md) module that is connected to the Pixhawk `GPS1` port — along with the [safety switch](#safety-switch) and [buzzer](#buzzer).

### 数传电台

[Data/Telemetry Radios](../telemetry/index.md) can provide a wireless MAVLink connection between a ground control station like _QGroundControl_ and a vehicle running PX4.
这使得在飞行过程中调整参数，实时监视遥测信息，更改任务等成为可能。

### 机载计算机

A [Companion Computer](../companion_computer/index.md) (also referred to as "mission computer" or "offboard computer"), is a separate on-vehicle computer that communicates with PX4 to provide higher level command and control.

The companion computer usually runs Linux, as this is a much better platform for "general" software development, and allows drones to leverage pre-existing software for computer vision, networking, and so on.

The flight controller and companion computer may be pre-integrated into a single baseboard, simplifying hardware development, or may be separate, and are connected via a serial cable, Ethernet cable, or wifi.
The companion computer typically communicates with PX4 using a high level Robotics API such as [MAVSDK](https://mavsdk.mavlink.io/) or [ROS 2](../ros2/user_guide.md).

Relevant topics include:

- [Companion Computers](../companion_computer/index.md)
- [Off-board Mode](../flight_modes/offboard.md) - Flight mode for offboard control of PX4 from a GCS or companion computer.
- [Robotics APIs](../robotics/index.md)

### SD卡（可移除储存器）

PX4 uses SD memory cards for storing [flight logs](../getting_started/flight_reporting.md), and they are also required in order to use UAVCAN peripherals and fly [missions](../flying/missions.md).

By default, if no SD card is present PX4 will play the [format failed (2-beep)](../getting_started/tunes.md#format-failed) tune twice during boot (and none of the above features will be available).

:::tip
The maximum supported SD card size on Pixhawk boards is 32GB.
The _SanDisk Extreme U3 32GB_ and _Samsung EVO Plus 32_ are [highly recommended](../dev_log/logging.md#sd-cards).
:::

尽管如此，SD卡也只是可选的。
不包含 SD 卡槽的飞行控制器可以：

- Disable notification beeps are disabled using the parameter [CBRK_BUZZER](../advanced_config/parameter_reference.md#CBRK_BUZZER).
- [Stream logs](../dev_log/logging.md#log-streaming) to another component (companion).
- Store missions in RAM/FLASH.
  <!-- Too low-level for this. But see FLASH_BASED_DATAMAN in  Intel Aero: https://github.com/PX4/PX4-Autopilot/blob/main/boards/intel/aerofc-v1/src/board_config.h#L115 -->

## Payloads

Payloads are equipment carried by the vehicle to meet user or mission objectives, such as cameras in surveying missions, instruments used in for inspections such as radiation detectors, and cargo that needs to be delivered.
PX4 supports many cameras and a wide range of payloads.

Payloads are connected to [Flight Controller outputs](#outputs-motors-servos-actuators), and can be triggered automatically in missions, or manually from an RC Controller or Joystick, or from a Ground Station (via MAVLink/MAVSDK commands).

For more information see: [Payloads & Cameras](../payloads/index.md)

## 解锁和加锁

A vehicle is said to be _armed_ when all motors and actuators are powered, and _disarmed_ when nothing is powered.
There is also a _prearmed_ state when only servo actuators are powered, which is primarily used for testing.

A vehicle is usually disarmed on the ground, and must be armed before taking off in the current flight mode.

:::warning
Armed vehicles are dangerous because propellers can start spinning at any time without further user input, and in many cases will start spinning immediately.
:::

Arming and disarming are triggered by default using RC stick _gestures_.
On Mode 2 transmitters you arm by holding the RC throttle/yaw stick on the _bottom right_ for one second, and to disarm you hold the stick on bottom left for one second.
还可以使用遥控上的按钮来配置 PX4 解锁（也可以从地面站发送MAVLink解锁命令）。

To reduce accidents, vehicles should be armed as little as possible when the vehicle is on the ground.
By default, vehicles are:

- _Disarmed_ or _Prearmed_ (motors unpowered) when not in use, and must be explicitly _armed_ before taking off.
- Automatically disarm/prearm if the vehicle does not take off quickly enough after arming (the disarm time is configurable).
- Automatically disarm/prearm shortly after landing (the time is configurable).
- 载具如果不是在“健康”状态，则会解锁不通过。
- Arming is prevented if the vehicle has a [safety switch](#safety-switch) that has not been engaged.
- Arming is prevented if a VTOL vehicle is in fixed-wing mode ([by default](../advanced_config/parameter_reference.md#CBRK_VTOLARMING)).
- Arming may be prevented due to a number of other optional [arming pre-condition settings](../config/safety.md#arming-pre-conditions), such as low battery.

When prearmed you can still use actuators, while disarming unpowers everything.
Prearmed and disarmed should both be safe, and a particular vehicle may support either or both.

:::tip
Sometimes a vehicle will not arm for reasons that are not obvious.
QGC v4.2.0 (Daily build at time of writing) and later provide an arming check report in [Fly View > Arming and Preflight Checks](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/fly_view/fly_view.html#arm).
From PX4 v1.14 this provides comprehensive information about arming problems along with possible solutions.
:::

A detailed overview of arming and disarming configuration can be found here: [Prearm, Arm, Disarm Configuration](../advanced_config/prearm_arm_disarm.md).

## 飞行模式

Modes are special operational states that provide different types/levels of vehicle automation and autopilot assistance to the user (pilot).

_Autonomous modes_ are fully controlled by the autopilot, and require no pilot/remote control input.
These are used, for example, to automate common tasks like takeoff, returning to the home position, and landing.
Other autonomous modes execute pre-programmed missions, follow a GPS beacon, or accept commands from an offboard computer or ground station.

_Manual modes_ are controlled by the user (via the RC control sticks/joystick) with assistance from the autopilot.
Different manual modes enable different flight characteristics - for example, some modes enable acrobatic tricks,
while others are impossible to flip and will hold position/course against wind.

:::tip
Not all modes are available on all vehicle types, and some modes can only be used when specific conditions have been met (e.g. many modes require a global position estimate).
:::

An overview of the flight modes implemented within PX4 for each vehicle can be found below:

- [Flight Modes (Multicopter)](../flight_modes_mc/index.md)
- [Flight Modes (Fixed-Wing)](../flight_modes_fw/index.md)
- [Flight Modes (VTOL)](../flight_modes_vtol/index.md)
- [Drive Modes (Differential Rover)](../flight_modes_rover/differential.md)
- [Drive Modes (Ackermann Rover)](../flight_modes_rover/ackermann.md)

Instructions for how to set up your remote control switches to enable different flight modes is provided in [Flight Mode Configuration](../config/flight_mode.md).

PX4 also supports external modes implemented in [ROS 2](../ros2/index.md) using the [PX4 ROS 2 Control Interface](../ros2/px4_ros2_control_interface.md).
These are indistinguishable from PX4 internal modes, and can be used to override internal modes with a more advanced version, or to create entirely new functionality.
Note that these depend on ROS 2 and can therefore only run on systems that have a [companion computer](#offboard-companion-computer).

## 安全设置（故障保护）

PX4 has configurable failsafe systems to protect and recover your vehicle if something goes wrong!
These allow you to specify areas and conditions under which you can safely fly, and the action that will be performed if a failsafe is triggered (for example, landing, holding position, or returning to a specified point).

:::info
You can only specify the action for the _first_ failsafe event.
Once a failsafe occurs the system will enter special handling code, such that subsequent failsafe triggers are managed by separate system level and vehicle specific code.
:::

The main failsafe areas are listed below:

- Low Battery
- Remote Control (RC) Loss
- Position Loss (global position estimate quality is too low).
- Offboard Loss (e.g. lose connection to companion computer)
- Data Link Loss (e.g. lose telemetry connection to GCS).
- Geofence Breach (restrict vehicle to flight within a virtual cylinder).
- Mission Failsafe (prevent a previous mission being run at a new takeoff location).
- Traffic avoidance (triggered by transponder data from e.g. ADSB transponders).

For more information see: [Safety](../config/safety.md) (Basic Configuration).

## 航向和运动方向

All the vehicles, boats and aircraft have a heading direction or an orientation based on their forward motion.

![Frame Heading](../../assets/concepts/frame_heading.png)

:::info
For a VTOL Tailsitter the heading is relative to the multirotor configuration (i.e. vehicle pose during takeoff, hovering, landing).
:::

It is important to know the vehicle heading direction in order to align the autopilot with the vehicle vector of movement.
Multicopters have a heading even when they are symmetrical from all sides!
Usually manufacturers use a coloured props or coloured arms to indicate the heading.

![Frame Heading TOP](../../assets/concepts/frame_heading_top.png)

In our illustrations we will use red colouring for the front propellers of multicopter to show heading.

You can read in depth about heading in [Flight Controller Orientation](../config/flight_controller_orientation.md)
