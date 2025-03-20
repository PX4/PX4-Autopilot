# 模块参考：系统

## battery_simulator

Source: [modules/simulation/battery_simulator](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/simulation/battery_simulator)

### 描述

<a id="battery_simulator_usage"></a>

### 用法

```
battery_simulator <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## battery_status

Source: [modules/battery_status](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/battery_status)

### 描述

模块提供的功能包括：

- Read the output from the ADC driver (via ioctl interface) and publish `battery_status`.

### 实现

模块运行在它自己的线程中，并轮询当前选定的陀螺仪主题。

<a id="battery_status_usage"></a>

### 用法

```
battery_status <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## camera_feedback

Source: [modules/camera_feedback](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/camera_feedback)

### 描述

The camera_feedback module publishes `CameraCapture` UORB topics when image capture has been triggered.

If camera capture is enabled, then trigger information from the camera capture pin is published;
otherwise trigger information at the point the camera was commanded to trigger is published
(from the `camera_trigger` module).

The `CAMERA_IMAGE_CAPTURED` message is then emitted (by streaming code) following `CameraCapture` updates.
`CameraCapture` topics are also logged and can be used for geotagging.

### 实现

`CameraTrigger` topics are published by the `camera_trigger` module (`feedback` field set `false`)
when image capture is triggered, and may also be published by the  `camera_capture` driver
(with `feedback` field set `true`) if the camera capture pin is activated.

The `camera_feedback` module subscribes to `CameraTrigger`.
It discards topics from the `camera_trigger` module if camera capture is enabled.
For the topics that are not discarded it creates a `CameraCapture` topic with the timestamp information
from the `CameraTrigger` and position information from the vehicle.

<a id="camera_feedback_usage"></a>

### 用法

```
camera_feedback <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## cdcacm_autostart

Source: [drivers/cdcacm_autostart](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/cdcacm_autostart)

### 描述

This module listens on USB and auto-configures the protocol depending on the bytes received.
The supported protocols are: MAVLink, nsh, and ublox serial passthrough. If the parameter SYS_USB_AUTO=2
the module will only try to start mavlink as long as the USB VBUS is detected. Otherwise it will spin
and continue to check for VBUS and start mavlink once it is detected.

<a id="cdcacm_autostart_usage"></a>

### 用法

```
cdcacm_autostart <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## commander

Source: [modules/commander](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/commander)

### 描述

该模块包含飞行模式切换和失效保护状态机。

<a id="commander_usage"></a>

### 用法

```
commander <command> [arguments...]
 Commands:
   start
     [-h]        Enable HIL mode

   calibrate     Run sensor calibration
     mag|baro|accel|gyro|level|esc|airspeed Calibration type
     quick       Quick calibration [mag, accel (not recommended)]

   check         Run preflight checks

   arm
     [-f]        Force arming (do not run preflight checks)

   disarm
     [-f]        Force disarming (disarm in air)

   takeoff

   land

   transition    VTOL transition

   mode          Change flight mode
     manual|acro|offboard|stabilized|altctl|posctl|position:slow|auto:mission|au
                 to:loiter|auto:rtl|auto:takeoff|auto:land|auto:precland|ext1
                 Flight mode

   pair

   lockdown
     on|off      Turn lockdown on or off

   set_ekf_origin
     lat, lon, alt Origin Latitude, Longitude, Altitude

   lat|lon|alt   Origin latitude longitude altitude

   poweroff      Power off board (if supported)

   stop

   status        print status info
```

## dataman

Source: [modules/dataman](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/dataman)

### 描述

该模块通过基于C语言的API以简单数据库的形式为系统的其他部分提供持续性存储功能。
Multiple backends are supported depending on the board:

- a file (eg. on the SD card)
- FLASH(需要飞控板支持)

It is used to store structured data of different types: mission waypoints, mission state and geofence polygons. Each type has a specific type and a fixed maximum amount of storage items, so that fast random access is possible.
每种类型的数据都有一个特定的类型和一个固定的最大存储条目的数量，因此可以实现对数据的快速随机访问。

### 实现

单个数据的读取和写入是原子操作。

<a id="dataman_usage"></a>

### 用法

```
dataman <command> [arguments...]
 Commands:
   start
     [-f <val>]  Storage file
                 values: <file>
     [-r]        Use RAM backend (NOT persistent)

 The options -f and -r are mutually exclusive. If nothing is specified, a file
 'dataman' is used

   stop

   status        print status info
```

## dmesg

Source: [systemcmds/dmesg](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/dmesg)

### 描述

用于显示启动控制台消息的命令行工具
需要注意的是，NuttX系统的工作队列和系统日志输出都未被捕捉到。

### 示例

持续在后台打印所有消息。

```
dmesg -f &
```

<a id="dmesg_usage"></a>

### 用法

```
dmesg <command> [arguments...]
 Commands:
     [-f]        Follow: wait for new messages
```

## esc_battery

Source: [modules/esc_battery](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/esc_battery)

### 描述

Background process running periodically with 1 Hz on the LP work queue to calculate the CPU load and RAM usage and publish the <code>cpuload</code> topic.

<a id="esc_battery_usage"></a>

### 用法

```
esc_battery <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## gyro_calibration

Source: [modules/gyro_calibration](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/gyro_calibration)

### 描述

源码：<a href="https://github.com/PX4/Firmware/tree/master/src/drivers/heater">drivers/heater</a>

<a id="gyro_calibration_usage"></a>

### 用法

```
gyro_calibration <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## heater

Source: [modules/gyro_fft](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/gyro_fft)

### 描述

<a id="gyro_fft_usage"></a>

### 用法

```
gyro_fft <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## land_detector

Source: [drivers/heater](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/heater)

### 描述

源码：<a href="https://github.com/PX4/Firmware/tree/master/src/modules/land_detector">modules/land_detector</a>

This task can be started at boot from the startup scripts by setting SENS_EN_THERMAL or via CLI.

<a id="heater_usage"></a>

### 用法

```
heater <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## i2c_launcher

Source: [systemcmds/i2c_launcher](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/i2c_launcher)

### 描述

Daemon that starts drivers based on found I2C devices.

<a id="i2c_launcher_usage"></a>

### 用法

```
i2c_launcher <command> [arguments...]
 Commands:
   start
     -b <val>    Bus number

   stop

   status        print status info
```

## internal_combustion_engine_control

Source: [modules/internal_combustion_engine_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/internal_combustion_engine_control)

### 描述

The module controls internal combustion engine (ICE) features including:
ignition (on/off), throttle and choke level, starter engine delay, and user request.

### Enabling

This feature is not enabled by default needs to be configured in the
build target for your board together with the rpm capture driver:

```
CONFIG_MODULES_INTERNAL_COMBUSTION_ENGINE_CONTROL=y
CONFIG_DRIVERS_RPM_CAPTURE=y
```

Additionally, to enable the module:

- Set [ICE_EN](../advanced_config/parameter_reference.md#ICE_EN)
 to true and adjust the other `ICE_` module parameters according to your needs.
- Set [RPM_CAP_ENABLE](../advanced_config/parameter_reference.md#RPM_CAP_ENABLE) to true.

The module outputs control signals for ignition, throttle, and choke,
and takes inputs from an RPM sensor.
These must be mapped to AUX outputs/inputs in the [Actuator configuration](../config/actuators.md),
similar to the setup shown below.

![Actuator setup for ICE](../../assets/hardware/ice/ice_actuator_setup.png)

### 实现

The ICE is implemented with a (4) state machine:

![Architecture](../../assets/hardware/ice/ice_control_state_machine.png)

The state machine:

- Checks if [Rpm.msg](../msg_docs/Rpm.md) is updated to know if the engine is running
- Allows for user inputs from:
 - AUX{N}
 - Arming state in [VehicleStatus.msg](../msg_docs/VehicleStatus.md)

The module publishes [InternalCombustionEngineControl.msg](../msg_docs/InternalCombustionEngineControl.md).

The architecture is as shown below:

![Architecture](../../assets/hardware/ice/ice_control_diagram.png)

<a id="internal_combustion_engine_control_usage"></a>

<a id="internal_combustion_engine_control_usage"></a>

### 用法

```
internal_combustion_engine_control <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## load_mon

Source: [modules/land_detector](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/land_detector)

### 描述

Module to detect the freefall and landed state of the vehicle, and publishing the `vehicle_land_detected` topic.
Each vehicle type (multirotor, fixedwing, vtol, ...) provides its own algorithm, taking into account various
states, such as commanded thrust, arming state and vehicle motion.

### 实现

Every type is implemented in its own class with a common base class. 触发时间由变量 MAYBE_LAND_TRIGGER_TIME 定义。 当检测到 maybe_landed 状态时，位置控制器会将推理设定值设置为零。 A hysteresis and a fixed
priority of each internal state determines the actual land_detector state.

#### 多旋翼的 Land Detector

**ground_contact**: thrust setpoint and velocity in z-direction must be below a defined threshold for time
GROUND_CONTACT_TRIGGER_TIME_US. When ground_contact is detected, the position controller turns off the thrust setpoint
in body x and y.

**maybe_landed**: it requires ground_contact together with a tighter thrust setpoint threshold and no velocity in the
horizontal direction. The trigger time is defined by MAYBE_LAND_TRIGGER_TIME. When maybe_landed is detected, the
position controller sets the thrust setpoint to zero.

**landed**: it requires maybe_landed to be true for time LAND_DETECTOR_TRIGGER_TIME_US.

There are 2 environment variables used for configuration: <code>replay</code>, which must be set to an ULog file name - it's the log file to be replayed. The second is the mode, specified via <code>replay_mode</code>:

<a id="land_detector_usage"></a>

### 用法

```
land_detector <command> [arguments...]
 Commands:
   start         Start the background task
     fixedwing|multicopter|vtol|rover|airship Select vehicle type

   stop

   status        print status info
```

## logger

Source: [modules/load_mon](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/load_mon)

### 描述

Background process running periodically on the low priority work queue to calculate the CPU load and RAM
usage and publish the `cpuload` topic.

On NuttX it also checks the stack usage of each process and if it falls below 300 bytes, a warning is output,
which will also appear in the log file.

<a id="load_mon_usage"></a>

### 用法

```
load_mon <command> [arguments...]
 Commands:
   start         Start the background task

   stop

   status        print status info
```

## logger

Source: [modules/logger](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/logger)

### 描述

System logger which logs a configurable set of uORB topics and system printf messages
(`PX4_WARN` and `PX4_ERR`) to ULog files. These can be used for system and flight performance evaluation,
tuning, replay and crash analysis.

It supports 2 backends:

- 文件：写入 ULog 文件到文件系统中（SD 卡）
- MAVLink: 通过 MAVLink 将 ULog 数据流传输到客户端上（需要客户端支持此方式）

模块的实现使用了两个线程：

In between there is a write buffer with configurable size (and another fixed-size buffer for the mission log). It should be large to avoid dropouts. 缓冲区应大到可以避免出现数据溢出。 It can be enabled and configured via SDLOG_MISSION parameter.
The normal log is always a superset of the mission log.

### 实现

立刻开始记录日志的典型用法：

- The main thread, running at a fixed rate (or polling on a topic if started with -p) and checking for
 data updates
- 写入线程，将数据写入文件中、

In between there is a write buffer with configurable size (and another fixed-size buffer for
the mission log). It should be large to avoid dropouts.

### 示例

Typical usage to start logging immediately:

```
logger on
```

Or if already running:

```
logger on
```

<a id="logger_usage"></a>

### 用法

```
logger <command> [arguments...]
 Commands:
   start
     [-m <val>]  Backend mode
                 values: file|mavlink|all, default: all
     [-x]        Enable/disable logging via Aux1 RC channel
     [-a]        Log 1st armed until shutdown
     [-e]        Enable logging right after start until disarm (otherwise only
                 when armed)
     [-f]        Log until shutdown (implies -e)
     [-t]        Use date/time for naming log directories and files
     [-r <val>]  Log rate in Hz, 0 means unlimited rate
                 default: 280
     [-b <val>]  Log buffer size in KiB
                 default: 12
     [-p <val>]  Poll on a topic instead of running with fixed rate (Log rate
                 and topic intervals are ignored if this is set)
                 values: <topic_name>
     [-c <val>]  Log rate factor (higher is faster)
                 default: 1.0

   on            start logging now, override arming (logger must be running)

   off           stop logging now, override arming (logger must be running)

   trigger_watchdog manually trigger the watchdog now

   stop

   status        print status info
```

## mag_bias_estimator

Source: [modules/mag_bias_estimator](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mag_bias_estimator)

### 描述

Online magnetometer bias estimator.

<a id="mag_bias_estimator_usage"></a>

### 用法

```
mag_bias_estimator <command> [arguments...]
 Commands:
   start         Start the background task

   stop

   status        print status info
```

## replay

Source: [modules/manual_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/manual_control)

### 描述

Module consuming manual_control_inputs publishing one manual_control_setpoint.

<a id="manual_control_usage"></a>

### 用法

```
manual_control <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## netman

Source: [systemcmds/netman](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/netman)

### 描述

Network configuration manager saves the network settings in non-volatile
memory. On boot the `update` option will be run. If a network configuration
does not exist. The default setting will be saved in non-volatile and the
system rebooted.

#### update

`netman update` is run automatically by [a startup script](../concept/system_startup.md#system-startup).
When run, the `update` option will check for the existence of `net.cfg` in the root of the SD Card.
It then saves the network settings from `net.cfg` in non-volatile memory,
deletes the file and reboots the system.

#### save

The `save` option will save settings from non-volatile memory to a file named
`net.cfg` on the SD Card filesystem for editing. Use this to edit the settings.
Save does not immediately apply the network settings; the user must reboot the flight stack.
By contrast, the `update` command is run by the start-up script, commits the settings to non-volatile memory,
and reboots the flight controller (which will then use the new settings).

#### show

The `show` option will display the network settings in `net.cfg` to the console.

### 示例

$ netman save           # Save the parameters to the SD card.
$ netman show           # display current settings.
$ netman update -i eth0 # do an update

<a id="netman_usage"></a>

### 用法

```
netman <command> [arguments...]
 Commands:
   show          Display the current persistent network settings to the console.

   update        Check SD card for net.cfg and update network persistent network
                 settings.

   save          Save the current network parameters to the SD card.
     [-i <val>]  Set the interface name
                 default: eth0
```

## pwm_input

Source: [drivers/pwm_input](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/pwm_input)

### 描述

Measures the PWM input on AUX5 (or MAIN5) via a timer capture ISR and publishes via the uORB 'pwm_input\` message.

<a id="pwm_input_usage"></a>

### 用法

```
pwm_input <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## rc_update

Source: [modules/rc_update](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/rc_update)

### 描述

The rc_update module handles RC channel mapping: read the raw input channels (`input_rc`),
then apply the calibration, map the RC channels to the configured channels & mode switches
and then publish as `rc_channels` and `manual_control_input`.

### 实现

To reduce control latency, the module is scheduled on input_rc publications.

<a id="rc_update_usage"></a>

### 用法

```
rc_update <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## replay

Source: [modules/replay](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/replay)

### 描述

This module is used to replay ULog files.

There are 2 environment variables used for configuration: `replay`, which must be set to an ULog file name - it's
the log file to be replayed. The second is the mode, specified via `replay_mode`:

- `replay_mode=ekf2`: specific EKF2 replay mode. It can only be used with the ekf2 module, but allows the replay
 to run as fast as possible.
- Generic otherwise: this can be used to replay any module(s), but the replay will be done with the same speed as the
 log was recorded.

The module is typically used together with uORB publisher rules, to specify which messages should be replayed.
The replay module will just publish all messages that are found in the log. It also applies the parameters from
the log.

The replay procedure is documented on the [System-wide Replay](https://docs.px4.io/main/en/debug/system_wide_replay.html)
page.

<a id="replay_usage"></a>

### 用法

```
replay <command> [arguments...]
 Commands:
   start         Start replay, using log file from ENV variable 'replay'

   trystart      Same as 'start', but silently exit if no log file given

   tryapplyparams Try to apply the parameters from the log file

   stop

   status        print status info
```

## send_event

Source: [modules/events](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/events)

### 描述

Background process running periodically on the LP work queue to perform housekeeping tasks.
It is currently only responsible for tone alarm on RC Loss.

The tasks can be started via CLI or uORB topics (vehicle_command from MAVLink, etc.).

<a id="send_event_usage"></a>

### 用法

```
send_event <command> [arguments...]
 Commands:
   start         Start the background task

   stop

   status        print status info
```

## sensor_agp_sim

Source: [modules/simulation/sensor_agp_sim](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/simulation/sensor_agp_sim)

### 描述

Module to simulate auxiliary global position measurements with optional failure modes for SIH simulation.

<a id="sensor_agp_sim_usage"></a>

### 用法

```
sensor_agp_sim <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## sensor_arispeed_sim

Source: [modules/simulation/sensor_airspeed_sim](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/simulation/sensor_airspeed_sim)

### 描述

<a id="sensor_arispeed_sim_usage"></a>

### 用法

```
sensor_arispeed_sim <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## sensor_baro_sim

Source: [modules/simulation/sensor_baro_sim](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/simulation/sensor_baro_sim)

### 描述

<a id="sensor_baro_sim_usage"></a>

### 用法

```
sensor_baro_sim <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## sensor_gps_sim

Source: [modules/simulation/sensor_gps_sim](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/simulation/sensor_gps_sim)

### 描述

<a id="sensor_gps_sim_usage"></a>

### 用法

```
sensor_gps_sim <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## sensor_mag_sim

Source: [modules/simulation/sensor_mag_sim](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/simulation/sensor_mag_sim)

### 描述

<a id="sensor_mag_sim_usage"></a>

### 用法

```
sensor_mag_sim <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## sensors

Source: [modules/sensors](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/sensors)

### 描述

The sensors module is central to the whole system. It takes low-level output from drivers, turns
it into a more usable form, and publishes it for the rest of the system.

模块提供的功能包括：

- Read the output from the sensor drivers (`SensorGyro`, etc.).
 如果存在多个同类型传感器，那个模块将进行投票和容错处理。
 然后应用飞控板的旋转和温度校正（如果被启用）。 And finally publish the data; one of the
 topics is `SensorCombined`, used by many parts of the system.
- Make sure the sensor drivers get the updated calibration parameters (scale & offset) when the parameters change or
 on startup. 传感器驱动使用 ioctl 接口获取参数更新。 For this to work properly, the
 sensor drivers must already be running when `sensors` is started.
- Do sensor consistency checks and publish the `SensorsStatusImu` topic.

### 实现

模块运行在它自己的线程中，并轮询当前选定的陀螺仪主题。

<a id="sensors_usage"></a>

### 用法

```
sensors <command> [arguments...]
 Commands:
   start
     [-h]        Start in HIL mode

   stop

   status        print status info
```

## system_power_simulation

Source: [modules/simulation/system_power_simulator](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/simulation/system_power_simulator)

### 描述

<a id="system_power_simulation_usage"></a>

### 用法

```
system_power_simulation <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## tattu_can

Source: [drivers/tattu_can](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/tattu_can)

### 描述

Driver for reading data from the Tattu 12S 16000mAh smart battery.

<a id="tattu_can_usage"></a>

### 用法

```
tattu_can <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## temperature_compensation

Source: [modules/temperature_compensation](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/temperature_compensation)

### 描述

The temperature compensation module allows all of the gyro(s), accel(s), and baro(s) in the system to be temperature
compensated. The module monitors the data coming from the sensors and updates the associated sensor_correction topic
whenever a change in temperature is detected. The module can also be configured to perform the coeffecient calculation
routine at next boot, which allows the thermal calibration coeffecients to be calculated while the vehicle undergoes
a temperature cycle.

<a id="temperature_compensation_usage"></a>

### 用法

```
temperature_compensation <command> [arguments...]
 Commands:
   start         Start the module, which monitors the sensors and updates the
                 sensor_correction topic

   calibrate     Run temperature calibration process
     [-a]        calibrate the accel
     [-g]        calibrate the gyro
     [-m]        calibrate the mag
     [-b]        calibrate the baro (if none of these is given, all will be
                 calibrated)

   stop

   status        print status info
```

## time_persistor

Source: [modules/time_persistor](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/time_persistor)

### 描述

Writes the RTC time cyclically to a file and reloads this value on startup.
This allows monotonic time on systems that only have a software RTC (that is not battery powered).
Explicitly setting the time backwards (e.g. via system_time) is still possible.

<a id="time_persistor_usage"></a>

### 用法

```
time_persistor <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## tune_control

Source: [systemcmds/tune_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/tune_control)

### 描述

Command-line tool to control & test the (external) tunes.

Tunes are used to provide audible notification and warnings (e.g. when the system arms, gets position lock, etc.).
The tool requires that a driver is running that can handle the tune_control uorb topic.

Information about the tune format and predefined system tunes can be found here:
https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/tunes/tune_definition.desc

### 示例

Play system tune #2:

```
tune_control play -t 2
```

<a id="tune_control_usage"></a>

### 用法

```
tune_control <command> [arguments...]
 Commands:
   play          Play system tune or single note.
     error       Play error tune
     [-t <val>]  Play predefined system tune
                 default: 1
     [-f <val>]  Frequency of note in Hz (0-22kHz)
     [-d <val>]  Duration of note in us
     [-s <val>]  Volume level (loudness) of the note (0-100)
                 default: 40
     [-m <val>]  Melody in string form
                 values: <string> - e.g. "MFT200e8a8a"

   libtest       Test library

   stop          Stop playback (use for repeated tunes)
```

## uxrce_dds_client

Source: [modules/uxrce_dds_client](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/uxrce_dds_client)

### 描述

UXRCE-DDS Client used to communicate uORB topics with an Agent over serial or UDP.

### 示例

```
uxrce_dds_client start -t serial -d /dev/ttyS3 -b 921600
uxrce_dds_client start -t udp -h 127.0.0.1 -p 15555
```

<a id="uxrce_dds_client_usage"></a>

### 用法

```
uxrce_dds_client <command> [arguments...]
 Commands:
   start
     [-t <val>]  Transport protocol
                 values: serial|udp, default: udp
     [-d <val>]  serial device
                 values: <file:dev>
     [-b <val>]  Baudrate (can also be p:<param_name>)
                 default: 0
     [-h <val>]  Agent IP. If not provided, defaults to UXRCE_DDS_AG_IP
                 values: <IP>
     [-p <val>]  Agent listening port. If not provided, defaults to
                 UXRCE_DDS_PRT
     [-n <val>]  Client DDS namespace

   stop

   status        print status info
```

## work_queue

Source: [systemcmds/work_queue](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/work_queue)

### 描述

Command-line tool to show work queue status.

<a id="work_queue_usage"></a>

### 用法

```
work_queue <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```
