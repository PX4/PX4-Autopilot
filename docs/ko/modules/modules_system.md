# 모듈 참조: 시스템

## battery_simulator

Source: [modules/simulation/battery_simulator](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/simulation/battery_simulator)

### 설명

<a id="battery_simulator_usage"></a>

### 사용법

```
battery_simulator <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## battery_status

Source: [modules/battery_status](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/battery_status)

### 설명

제공 기능은 다음과 같습니다:

- Read the output from the ADC driver (via ioctl interface) and publish `battery_status`.

### 구현

자체 스레드에서 실행되고, 현재 선택된 자이로 주제를 폴링합니다.

<a id="battery_status_usage"></a>

### 사용법

```
battery_status <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## camera_feedback

Source: [modules/camera_feedback](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/camera_feedback)

### 설명

The camera_feedback module publishes `CameraCapture` UORB topics when image capture has been triggered.

If camera capture is enabled, then trigger information from the camera capture pin is published;
otherwise trigger information at the point the camera was commanded to trigger is published
(from the `camera_trigger` module).

The `CAMERA_IMAGE_CAPTURED` message is then emitted (by streaming code) following `CameraCapture` updates.
`CameraCapture` topics are also logged and can be used for geotagging.

### 구현

`CameraTrigger` topics are published by the `camera_trigger` module (`feedback` field set `false`)
when image capture is triggered, and may also be published by the  `camera_capture` driver
(with `feedback` field set `true`) if the camera capture pin is activated.

The `camera_feedback` module subscribes to `CameraTrigger`.
It discards topics from the `camera_trigger` module if camera capture is enabled.
For the topics that are not discarded it creates a `CameraCapture` topic with the timestamp information
from the `CameraTrigger` and position information from the vehicle.

<a id="camera_feedback_usage"></a>

### 사용법

```
camera_feedback <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## cdcacm_autostart

Source: [drivers/cdcacm_autostart](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/cdcacm_autostart)

### 설명

This module listens on USB and auto-configures the protocol depending on the bytes received.
The supported protocols are: MAVLink, nsh, and ublox serial passthrough. If the parameter SYS_USB_AUTO=2
the module will only try to start mavlink as long as the USB VBUS is detected. Otherwise it will spin
and continue to check for VBUS and start mavlink once it is detected.

<a id="cdcacm_autostart_usage"></a>

### 사용법

```
cdcacm_autostart <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## commander

Source: [modules/commander](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/commander)

### 설명

커맨더 모듈에는 모드 전환 및 안전 장치 동작을 위한 상태 머신이 포함되어 있습니다.

<a id="commander_usage"></a>

### 사용법

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

### 설명

C API를 통해 간단한 데이터베이스 형태로 시스템에 영구 저장소를 제공하는 모듈입니다.
Multiple backends are supported depending on the board:

- 파일(예: SD 카드)
- RAM (영구적이지 않음)

임무 웨이포인트, 임무 상태 및 지오펜스 다각형과 같은 다양한 유형의 구조화된 데이터를 저장합니다.
각 유형은 특정 유형과 고정된 최대 저장 항목 수를 가지고 있어, 빠른 랜덤 액세스가 가능합니다.

### 구현

단일 항목을 읽고 쓰는 것은 항상 원자적입니다.

<a id="dataman_usage"></a>

### 사용법

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

### 설명

부팅 콘솔 메시지를 출력합니다.
NuttX의 작업 대기열 및 syslog의 출력은 캡처되지 않습니다.

### 예

백그라운드에서 모든 메시지를 출력합니다.

```
dmesg -f &
```

<a id="dmesg_usage"></a>

### 사용법

```
dmesg <command> [arguments...]
 Commands:
     [-f]        Follow: wait for new messages
```

## esc_battery

Source: [modules/esc_battery](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/esc_battery)

### 설명

ESC 상태의 정보를 사용하여 구현하고, 배터리 상태를 게시합니다.

<a id="esc_battery_usage"></a>

### 사용법

```
esc_battery <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## gyro_calibration

Source: [modules/gyro_calibration](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/gyro_calibration)

### 설명

간단한 온라인 자이로스코프 교정.

<a id="gyro_calibration_usage"></a>

### 사용법

```
gyro_calibration <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## gyro_fft

Source: [modules/gyro_fft](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/gyro_fft)

### 설명

<a id="gyro_fft_usage"></a>

### 사용법

```
gyro_fft <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## heater

Source: [drivers/heater](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/heater)

### 설명

설정점에서 IMU 온도를 조절하기 위하여 LP 작업 대기열에서 주기적으로 실행되는 백그라운드 프로세스입니다.

이 작업은 부팅 시 SENS_EN_THERMAL을 설정하거나, CLI를 통하여 시작 스크립트에서 시작할 수 있습니다.

<a id="heater_usage"></a>

### 사용법

```
heater <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## i2c_launcher

Source: [systemcmds/i2c_launcher](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/i2c_launcher)

### 설명

Daemon that starts drivers based on found I2C devices.

<a id="i2c_launcher_usage"></a>

### 사용법

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

### 설명

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

### 구현

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

### 사용법

```
internal_combustion_engine_control <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## land_detector

Source: [modules/land_detector](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/land_detector)

### 설명

Module to detect the freefall and landed state of the vehicle, and publishing the `vehicle_land_detected` topic.
Each vehicle type (multirotor, fixedwing, vtol, ...) provides its own algorithm, taking into account various
states, such as commanded thrust, arming state and vehicle motion.

### 구현

모든 유형은 공통 기본 클래스를 사용하여 자체 클래스에서 구현됩니다. 기본 클래스는 상태를 유지합니다(착륙, 아마도_착륙, 지상_접촉). 가능한 각 상태는 파생 클래스에서 구현됩니다. 각 내부 상태의 히스테리시스 및 고정된 우선 순위에 따라 실제 land_detector 상태가 결정됩니다.

#### 멀티콥터 착륙 감지기

**ground_contact**: thrust setpoint and velocity in z-direction must be below a defined threshold for time
GROUND_CONTACT_TRIGGER_TIME_US. ground_contact가 감지되면, 위치 컨트롤러는 본체 x 및 y의 추력 설정값을 끕니다.

**maybe_landed**: it requires ground_contact together with a tighter thrust setpoint threshold and no velocity in the
horizontal direction. 트리거 시간은 MAYBE_LAND_TRIGGER_TIME에 의해 정의됩니다. Maybe_landed가 감지되면 위치 컨트롤러는 추력 설정값을 0으로 설정합니다.

**landed**: it requires maybe_landed to be true for time LAND_DETECTOR_TRIGGER_TIME_US.

모듈은 HP 작업 대기열에서 주기적으로 실행됩니다.

<a id="land_detector_usage"></a>

### 사용법

```
land_detector <command> [arguments...]
 Commands:
   start         Start the background task
     fixedwing|multicopter|vtol|rover|airship Select vehicle type

   stop

   status        print status info
```

## load_mon

Source: [modules/load_mon](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/load_mon)

### 설명

Background process running periodically on the low priority work queue to calculate the CPU load and RAM
usage and publish the `cpuload` topic.

NuttX에서는 각 프로세스의 스택 사용량도 확인하고, 300바이트 미만으로 떨어지면 경고가 출력되고 로그 파일에도 기록됩니다.

<a id="load_mon_usage"></a>

### 사용법

```
load_mon <command> [arguments...]
 Commands:
   start         Start the background task

   stop

   status        print status info
```

## logger

Source: [modules/logger](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/logger)

### 설명

System logger which logs a configurable set of uORB topics and system printf messages
(`PX4_WARN` and `PX4_ERR`) to ULog files. 시스템 및 비행 성능 평가, 튜닝, 재생 및 충돌 분석에 사용할 수 있습니다.

2개의 백엔드를 지원합니다.

- 파일: ULog 파일을 파일 시스템(SD 카드)에 기록합니다.
- MAVLink: MAVLink를 통해 ULog 데이터를 클라이언트로 스트리밍합니다(클라이언트가 이를 지원해야 함).

두 백엔드를 동시에 활성화하고 사용할 수 있습니다.

파일 백엔드는 전체(일반 로그)와 미션 로그의 두 가지 유형의 로그 파일을 지원합니다. 임무 로그는 축소된 ulog 파일이며, 지오태깅 또는 차량 관리 등에 사용할 수 있습니다. SDLOG_MISSION 매개변수를 통하여 활성화 및 설정할 수 있습니다.
일반 로그는 항상 미션 로그의 상위 집합입니다.

### 구현

구현은 두 개의 스레드를 사용합니다.

- The main thread, running at a fixed rate (or polling on a topic if started with -p) and checking for
 data updates
- 작성자 스레드, 파일에 데이터 쓰기

그 사이에는 구성 가능한 크기의 쓰기 버퍼가 있습니다(및 미션 로그를 위한 또 다른 고정 크기 버퍼). 드롭아웃을 방지하려면 크기가 커야 합니다.

### 예

즉시 로깅을 시작하는 일반적인 사용법입니다.

```
logger start -e -t
```

또는, 이미 동작중일 경우

```
logger on
```

<a id="logger_usage"></a>

### 사용법

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

## netman

Source: [modules/mag_bias_estimator](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mag_bias_estimator)

### 설명

Online magnetometer bias estimator.

<a id="mag_bias_estimator_usage"></a>

### 사용법

```
mag_bias_estimator <command> [arguments...]
 Commands:
   start         Start the background task

   stop

   status        print status info
```

## pwm_input

Source: [modules/manual_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/manual_control)

### 설명

Module consuming manual_control_inputs publishing one manual_control_setpoint.

<a id="manual_control_usage"></a>

### 사용법

```
manual_control <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## netman

Source: [systemcmds/netman](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/netman)

### 설명

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

### 예

$ netman save           # Save the parameters to the SD card.
$ netman show           # display current settings.
$ netman update -i eth0 # do an update

<a id="netman_usage"></a>

### 사용법

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

### 설명

Measures the PWM input on AUX5 (or MAIN5) via a timer capture ISR and publishes via the uORB 'pwm_input\` message.

<a id="pwm_input_usage"></a>

### 사용법

```
pwm_input <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## rc_update

Source: [modules/rc_update](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/rc_update)

### 설명

The rc_update module handles RC channel mapping: read the raw input channels (`input_rc`),
then apply the calibration, map the RC channels to the configured channels & mode switches
and then publish as `rc_channels` and `manual_control_input`.

### 구현

To reduce control latency, the module is scheduled on input_rc publications.

<a id="rc_update_usage"></a>

### 사용법

```
rc_update <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## replay

Source: [modules/replay](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/replay)

### 설명

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

### 사용법

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

### 설명

Background process running periodically on the LP work queue to perform housekeeping tasks.
It is currently only responsible for tone alarm on RC Loss.

The tasks can be started via CLI or uORB topics (vehicle_command from MAVLink, etc.).

<a id="send_event_usage"></a>

### 사용법

```
send_event <command> [arguments...]
 Commands:
   start         Start the background task

   stop

   status        print status info
```

## sensor_agp_sim

Source: [modules/simulation/sensor_agp_sim](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/simulation/sensor_agp_sim)

### 설명

Module to simulate auxiliary global position measurements with optional failure modes for SIH simulation.

<a id="sensor_agp_sim_usage"></a>

### 사용법

```
sensor_agp_sim <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## sensor_arispeed_sim

Source: [modules/simulation/sensor_airspeed_sim](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/simulation/sensor_airspeed_sim)

### 설명

<a id="sensor_arispeed_sim_usage"></a>

### 사용법

```
sensor_arispeed_sim <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## sensor_baro_sim

Source: [modules/simulation/sensor_baro_sim](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/simulation/sensor_baro_sim)

### 설명

<a id="sensor_baro_sim_usage"></a>

### 사용법

```
sensor_baro_sim <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## sensor_gps_sim

Source: [modules/simulation/sensor_gps_sim](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/simulation/sensor_gps_sim)

### 설명

<a id="sensor_gps_sim_usage"></a>

### 사용법

```
sensor_gps_sim <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## sensor_mag_sim

Source: [modules/simulation/sensor_mag_sim](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/simulation/sensor_mag_sim)

### 설명

<a id="sensor_mag_sim_usage"></a>

### 사용법

```
sensor_mag_sim <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## sensors

Source: [modules/sensors](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/sensors)

### 설명

The sensors module is central to the whole system. It takes low-level output from drivers, turns
it into a more usable form, and publishes it for the rest of the system.

제공 기능은 다음과 같습니다:

- Read the output from the sensor drivers (`SensorGyro`, etc.).
 동일한 유형이 여러 개 있는 경우 투표 및 장애 조치 처리를 수행합니다.
 그런 다음, 보드 회전 및 온도 보정을 적용합니다(활성화된 경우). And finally publish the data; one of the
 topics is `SensorCombined`, used by many parts of the system.
- Make sure the sensor drivers get the updated calibration parameters (scale & offset) when the parameters change or
 on startup. 센서 드라이버는 매개변수 업데이트를 위하여 ioctl 인터페이스를 사용합니다. For this to work properly, the
 sensor drivers must already be running when `sensors` is started.
- Do sensor consistency checks and publish the `SensorsStatusImu` topic.

### 구현

자체 스레드에서 실행되고, 현재 선택된 자이로 주제를 폴링합니다.

<a id="sensors_usage"></a>

### 사용법

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

### 설명

<a id="system_power_simulation_usage"></a>

### 사용법

```
system_power_simulation <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## tattu_can

Source: [drivers/tattu_can](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/tattu_can)

### 설명

Driver for reading data from the Tattu 12S 16000mAh smart battery.

<a id="tattu_can_usage"></a>

### 사용법

```
tattu_can <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## temperature_compensation

Source: [modules/temperature_compensation](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/temperature_compensation)

### 설명

The temperature compensation module allows all of the gyro(s), accel(s), and baro(s) in the system to be temperature
compensated. The module monitors the data coming from the sensors and updates the associated sensor_correction topic
whenever a change in temperature is detected. The module can also be configured to perform the coeffecient calculation
routine at next boot, which allows the thermal calibration coeffecients to be calculated while the vehicle undergoes
a temperature cycle.

<a id="temperature_compensation_usage"></a>

### 사용법

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

### 설명

Writes the RTC time cyclically to a file and reloads this value on startup.
This allows monotonic time on systems that only have a software RTC (that is not battery powered).
Explicitly setting the time backwards (e.g. via system_time) is still possible.

<a id="time_persistor_usage"></a>

### 사용법

```
time_persistor <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## tune_control

Source: [systemcmds/tune_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/tune_control)

### 설명

Command-line tool to control & test the (external) tunes.

Tunes are used to provide audible notification and warnings (e.g. when the system arms, gets position lock, etc.).
The tool requires that a driver is running that can handle the tune_control uorb topic.

Information about the tune format and predefined system tunes can be found here:
https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/tunes/tune_definition.desc

### 예

Play system tune #2:

```
tune_control play -t 2
```

<a id="tune_control_usage"></a>

### 사용법

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

### 설명

UXRCE-DDS Client used to communicate uORB topics with an Agent over serial or UDP.

### 예

```
uxrce_dds_client start -t serial -d /dev/ttyS3 -b 921600
uxrce_dds_client start -t udp -h 127.0.0.1 -p 15555
```

<a id="uxrce_dds_client_usage"></a>

### 사용법

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

### 설명

Command-line tool to show work queue status.

<a id="work_queue_usage"></a>

### 사용법

```
work_queue <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```
