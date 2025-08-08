# 기본 개념

이 주제는 드론과 PX4 사용에 대한 기본적인 소개를 제공합니다 (주로 초보자를 위한 내용이지만, 경험이 있는 사용자에게도 좋은 입문 자료가 될 수 있습니다).

기본 개념에 이미 익숙하다면, [기본 조립](../assembly/index.md) 섹션으로 넘어가서 사용 중인 오토파일럿 하드웨어의 배선 방법을 배우실 수 있습니다.
펌웨어를 설치하고 _QGroundControl_을 사용해 비행체를 설정하려면\
[기본 설정](../config/index.md) 섹션을 참조하세요.

## 드론의 정의

드론(Drones), 또는 무인 차량(Unmanned Vehicles, UV)은 사람이 탑승하지 않은 "로봇형" 차량으로, 수동 또는 자율적으로 제어될 수 있습니다.
드론은 공중, 지상, 수상 또는 수중을 이동할 수 있으며, 항공 사진/영상 촬영, 화물 운송, 레이싱, 수색, 측량 등 다양한 [소비자, 산업, 정부 및 군사 분야](https://px4.io/ecosystem/commercial-systems/)에서 활용됩니다.

드론은 보다 공식적으로는 무인 항공기(UAV), 무인 지상 차량(UGV), 무인 수상 차량(USV), 무인 수중 차량(UUV)으로 구분됩니다.

:::info
"무인 항공 시스템(UAS)"이라는 용어는 일반적으로 UAV(무인 항공기)와 함께 이를 구성하는 전체 시스템을 의미하며, 여기에는 지상 관제소, 무선 조종기, 드론을 제어하거나 데이터를 수집 및 처리하는 데 사용되는 모든 시스템이 포함됩니다.
:::

## 드론의 종류

드론에는 다양한 프레임(기체 형태)이 있으며, 각 형태 내에도 여러 가지 변형이 존재합니다.
아래는 드론의 대표적인 형태들과 각 형태에 가장 적합한 활용 사례들입니다.

- [멀티콥터(Multicopters)](../frames_multicopter/index.md) — 멀티로터는 정밀한 호버링과 수직 이착륙이 가능하지만, 비행 시간이 짧고 속도도 일반적으로 느린 편입니다.
  이들은 가장 인기 있는 비행체 유형 중 하나로, 조립이 쉽고 PX4가 제공하는 다양한 비행 모드를 통해 조종이 간편하며, 카메라 플랫폼으로도 매우 적합하기 때문입니다.
- [헬리콥터](../frames_helicopter/index.md) — 헬리콥터는 멀티콥터와 유사한 이점을 제공하지만, 기계적으로 더 복잡하고 더 효율적입니다.
  이들은 또한 조종이 훨씬 더 어렵습니다.
- [비행기 (고정익)](../frames_plane/index.md) — 고정익 기체는 멀티콥터보다 더 오래, 더 빠르게 비행할 수 있어 지상 조사 등에서 더 넓은 범위를 커버할 수 있습니다.\
  하지만 멀티콥터보다 조종과 착륙이 더 어렵고, 공중에 머무르거나 아주 느리게 비행해야 하는 경우(예: 수직 구조물 조사)에는 적합하지 않습니다.
- [VTOL](../frames_vtol/index.md) (수직 이착륙) — 고정익/멀티콥터 하이브리드 기체는 두 가지 방식의 장점을 모두 제공합니다. 멀티콥터처럼 수직으로 이륙하고 호버링할 수 있으며, 비행 중에는 비행기처럼 전진 비행으로 전환해 더 넓은 지역을 커버할 수 있습니다.
  VTOL은 일반적으로 멀티콥터나 고정익 항공기보다 더 비싸며, 제작과 튜닝도 더 어렵습니다.
  VTOL은 여러 형태로 나뉘며, 대표적으로 틸트로터(tiltrotor), 테일시터(tailsitter), 쿼드플레인(quadplane) 등이 있습니다.
- [에어십](../frames_airship/index.md)/[기구](../frames_balloon/index.md) — 공기보다 가벼운 비행체로, 일반적으로 고고도에서 오랜 시간 비행이 가능하지만, 속도나 비행 방향에 대한 제어가 제한되거나 불가능한 경우가 많습니다.
- [로버](../frames_rover/index.md) — 자동차와 유사한 지상 주행형 비행체입니다.
  조작이 간단하고 사용하기에 재미있는 경우가 많습니다.
  대부분의 항공기만큼 빠르게 이동할 수는 없지만, 더 무거운 화물을 운반할 수 있으며 정지 상태에서는 전력을 거의 소모하지 않습니다.
- **보트** — 수면 위를 이동하는 수상 차량입니다.
- [잠수정(Submersibles)](../frames_sub/index.md) — 수중에서 운용되는 차량입니다.

더 자세한 정보는 다음을 참고하십시오.

- [비행체 유형 및 설정](../airframes/index.md)
- [기체 프레임 설정](../config/airframe.md)
- [기체 프레임 참조](../airframes/airframe_reference.md)

## 자동조종장치

자율비행장치(오토파일럿)는 드론의 두뇌에 해당하는 장치입니다.

기본적으로 드론 시스템은 실시간 운영체제("RTOS")가 탑재된 비행 컨트롤러(FC)에서 비행 제어용 소프트웨어(비행 스택)를 실행하는 구조로 이루어져 있습니다.
비행 스택은 기본적인 안정화와 안전 기능을 제공하며, 일반적으로 수동 비행 시 조종을 도와주는 보조 기능과 함께 이륙, 착륙, 미리 정해진 임무 수행 같은 작업을 자동으로 실행할 수 있도록 지원합니다.

일부 자율 비행 장치는 고차원 명령과 제어를 수행할 수 있는 범용 컴퓨팅 시스템을 함께 탑재하고 있어, 보다 정교한 네트워킹, 컴퓨터 비전, 기타 고급 기능들을 지원할 수 있습니다.
이 기능은 별도의 [보조 컴퓨터](#offboard-companion-computer)로 구현될 수도 있지만, 앞으로는 이러한 시스템이 하나로 통합된 구성 요소로 점점 더 많이 탑재될 가능성이 큽니다.

## PX4 자동비행장치

[PX4](https://px4.io/)는 NuttX 실시간 운영체제(RTOS) 위에서 구동되는 강력한 오픈소스 자율비행 소프트웨어(flight stack)입니다.

PX4의 주요 특징들은 아래와 같습니다.

- 다양한 기체 프레임/유형을 지원하며, 다음을 포함합니다:  [멀티콥터](../frames_multicopter/index.md),  [고정익 항공기](../frames_plane/index.md) (비행기),  [수직 이착륙기(VTOL)](../frames_vtol/index.md) (멀티콥터와 고정익의 하이브리드),  [지상 차량](../frames_rover/index.md),  [수중 차량](../frames_sub/index.md) 등.
- Great choice of drone components for [flight controller](#flight-controller), [sensors](#sensors), [payloads](#payloads), and other peripherals.
- Flexible and powerful [flight modes](#flight-modes) and [safety features](#safety-settings-failsafe).
- Robust and deep integration with [companion computers](#offboard-companion-computer) and [robotics APIs](../robotics/index.md) such as [ROS 2](../ros2/user_guide.md) and [MAVSDK](https://mavsdk.mavlink.io/main/en/index.html).

PX4 is a core part of a broader drone platform that includes the [QGroundControl](#qgc) ground station, [Pixhawk hardware](https://pixhawk.org/), and [MAVSDK](https://mavsdk.mavlink.io/main/en/index.html) for integration with companion computers, cameras and other hardware using the MAVLink protocol.
PX4 is supported by the [Dronecode Project](https://dronecode.org/).

## Ground Control Stations

Ground Control Stations (GCS) are ground based systems that allow UV operators to monitor and control a drone and its payloads.
A subset of the products that are known to work with PX4 are listed below.

### QGroundControl {#qgc}

The Dronecode GCS software is called [QGroundControl](https://qgroundcontrol.com/) ("QGC").
It runs on Windows, Android, MacOS or Linux hardware, and supports a wide range of screen form factors.
You can download it (for free) from [here](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html).

![QGC Main Screen](../../assets/concepts/qgc_fly_view.png)

QGroundControl communicates with the drone using a telemetry radio (a bidirectional data link), which allows you to get real-time flight and safety information, and to control the vehicle, camera, and other payloads using a point-and-click interface.
On hardware that supports them, you can also manually fly the vehicle using joystick controllers.
QGC can also be used to visually plan, execute, and monitor autonomous missions, set geofences, and much more.

QGroundControl desktop versions are also used to install (flash) PX4 firmware and configure PX4 on the drone's autopilot/flight controller hardware.

### Auterion Mission Control (AMC) {#amc}

[Auterion Mission Control](https://auterion.com/product/mission-control/) is a powerful and fully featured ground control station application that is optimized for _pilots_ rather than vehicle configuration.
While designed to work with Auterion products, it can be used with "vanilla" PX4.

더 자세한 정보는 다음을 참고하십시오.

- [AMC docs](https://docs.auterion.com/vehicle-operation/auterion-mission-control)
- [Download from Auterion Suite](https://suite.auterion.com/)

## Drone Components & Parts

### 비행 콘트롤러

Flight controllers (FC) are the hardware onto which the PX4 flight stack firmware is loaded and run.
They are connected to sensors from which PX4 determines its state, and to the actuators/motors that it uses to stabilise and move the vehicle.

<img src="../../assets/flight_controller/cuav_pixhawk_v6x/pixhawk_v6x.jpg" width="230px" title="CUAV Pixhawk 6X" >

PX4 can run on many different types of [Flight Controller Hardware](../flight_controller/index.md), ranging from [Pixhawk Series](../flight_controller/pixhawk_series.md) controllers to Linux computers.
These include [Pixhawk Standard](../flight_controller/autopilot_pixhawk_standard.md) and [manufacturer-supported](../flight_controller/autopilot_manufacturer_supported.md) boards.
You should select a board that suits the physical constraints of your vehicle, the activities you wish to perform, and cost.

For more information see: [Flight Controller Selection](flight_controller_selection.md)

### 센서

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

### 출력 장치: 모터, 서보, 액츄에이터

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
- The FMU output ports can use [D-shot](../peripherals/dshot.md) or _One-shot_ protocols (as well as PWM), which provide much lower-latency behaviour.FMU 출력 포트는 레이싱 드론처럼  높은 성능이 요구되는 기체에 사용됩니다.
- There are only 6-8 outputs in `MAIN` and `AUX` because most flight controllers only have this many PWM/Dshot/Oneshot outputs.
  이론적으로는,  보드 버스에서 더 많은 출력 포트를 제공할 수 있습니다. UAVCAN 버스에는 이러한 제한이 없습니다.

### ESCs & Motors

Many PX4 drones use brushless motors that are driven by the flight controller via an Electronic Speed Controller (ESC)
(the ESC converts a signal from the flight controller to an appropriate level of power delivered to the motor).

PX4가 지원하는 전기변속기와 모터 정보는 여기를 참고하십시오.

- [ESC & Motors](../peripherals/esc_motors.md)
- [ESC Calibration](../advanced_config/esc_calibration.md)
- [ESC Firmware and Protocols Overview](https://oscarliang.com/esc-firmware-protocols/) (oscarliang.com)

### 배터리와 전원

PX4 드론은 리튬-폴리머(LiPo) 배터리를 가장 많이 사용합니다.
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
일부 RC에서는 자동조종장치에서 전송한 텔레메트리를 수신할 수 있습니다.

:::info
PX4 does not _require_ a manual control system for autonomous flight modes.
:::

### 안전 스위치

Vehicles may include a _safety switch_ that must be engaged before the vehicle can be [armed](#arming-and-disarming) (when armed, motors are powered and propellers can turn).

This switch is almost always integrated into the [GPS](../gps_compass/index.md) module that is connected to the Pixhawk `GPS1` port — along with the [buzzer](#buzzer) and [UI LED](#leds).

The switch may be disabled by default, though this depends on the particular flight controller and airframe configuration.
You can disable/enable use of the switch with the [CBRK_IO_SAFETY](../advanced_config/parameter_reference.md#CBRK_IO_SAFETY) parameter.

:::info
Safety switches are optional.
Many argue that it is safer for users never to approach a powered system, even to enable/disable this interlock.
:::

### 부저

Vehicles commonly include a buzzer for providing audible notification of vehicle state and readiness to fly (see [Tune meanings](../getting_started/tunes.md)).

This buzzer is almost always integrated into the [GPS](../gps_compass/index.md) module that is connected to the Pixhawk `GPS1` port — along with the [safety switch](#safety-switch) and [UI LED](#leds).
You can disable the notification tunes using the parameter [CBRK_BUZZER](../advanced_config/parameter_reference.md#CBRK_BUZZER).

### LED

Vehicles should have a superbright [UI RGB LED](../getting_started/led_meanings.md#ui-led) that indicates the current readiness for flight.

Historically this was included in the flight controller board.
On more recent flight controllers this is almost always an [I2C peripheral](../sensor_bus/i2c_general.md) integrated into the [GPS](../gps_compass/index.md) module that is connected to the Pixhawk `GPS1` port — along with the [safety switch](#safety-switch) and [buzzer](#buzzer).

### 텔레메트리 무선 통신

[Data/Telemetry Radios](../telemetry/index.md) can provide a wireless MAVLink connection between a ground control station like _QGroundControl_ and a vehicle running PX4.
비행중인 기체의 매개변수 변경, 실시간 텔레메트로 통신, 임무 변경 등의 작업을 수행할 수 있습니다.

### 외부 보조 컴퓨터

A [Companion Computer](../companion_computer/index.md) (also referred to as "mission computer" or "offboard computer"), is a separate on-vehicle computer that communicates with PX4 to provide higher level command and control.

The companion computer usually runs Linux, as this is a much better platform for "general" software development, and allows drones to leverage pre-existing software for computer vision, networking, and so on.

The flight controller and companion computer may be pre-integrated into a single baseboard, simplifying hardware development, or may be separate, and are connected via a serial cable, Ethernet cable, or wifi.
The companion computer typically communicates with PX4 using a high level Robotics API such as [MAVSDK](https://mavsdk.mavlink.io/) or [ROS 2](../ros2/user_guide.md).

관련 주제는 다음과 같습니다:

- [Companion Computers](../companion_computer/index.md)
- [Off-board Mode](../flight_modes/offboard.md) - Flight mode for offboard control of PX4 from a GCS or companion computer.
- [Robotics APIs](../robotics/index.md)

### SD 카드 (휴대용 저장 장치)

PX4 uses SD memory cards for storing [flight logs](../getting_started/flight_reporting.md), and they are also required in order to use UAVCAN peripherals and fly [missions](../flying/missions.md).

By default, if no SD card is present PX4 will play the [format failed (2-beep)](../getting_started/tunes.md#format-failed) tune twice during boot (and none of the above features will be available).

:::tip
The maximum supported SD card size on Pixhawk boards is 32GB.
The _SanDisk Extreme U3 32GB_ and _Samsung EVO Plus 32_ are [highly recommended](../dev_log/logging.md#sd-cards).
:::

SD 카드는 선택 사항입니다.
SD 카드가 없는 비행 콘트롤어는 다음의 작업들을 수행하여야 합니다.

- Disable notification beeps are disabled using the parameter [CBRK_BUZZER](../advanced_config/parameter_reference.md#CBRK_BUZZER).
- [Stream logs](../dev_log/logging.md#log-streaming) to another component (companion).
- Store missions in RAM/FLASH.

## Payloads

Payloads are equipment carried by the vehicle to meet user or mission objectives, such as cameras in surveying missions, instruments used in for inspections such as radiation detectors, and cargo that needs to be delivered.
PX4 supports many cameras and a wide range of payloads.

Payloads are connected to [Flight Controller outputs](#outputs-motors-servos-actuators), and can be triggered automatically in missions, or manually from an RC Controller or Joystick, or from a Ground Station (via MAVLink/MAVSDK commands).

For more information see: [Payloads & Cameras](../payloads/index.md)

## 시동 및 해제

A vehicle is said to be _armed_ when all motors and actuators are powered, and _disarmed_ when nothing is powered.
There is also a _prearmed_ state when only servo actuators are powered, which is primarily used for testing.

A vehicle is usually disarmed on the ground, and must be armed before taking off in the current flight mode.

:::warning
Armed vehicles are dangerous because propellers can start spinning at any time without further user input, and in many cases will start spinning immediately.
:::

Arming and disarming are triggered by default using RC stick _gestures_.
On Mode 2 transmitters you arm by holding the RC throttle/yaw stick on the _bottom right_ for one second, and to disarm you hold the stick on bottom left for one second.
PX4에서 무선 조종 스위치로 시동을 걸 수 있도록 설정할 수 있습니다. 또한, 지상통제국에서 시동 명령을 MAVLink로 전송할 수 있습니다.

To reduce accidents, vehicles should be armed as little as possible when the vehicle is on the ground.
By default, vehicles are:

- _Disarmed_ or _Prearmed_ (motors unpowered) when not in use, and must be explicitly _armed_ before taking off.
- Automatically disarm/prearm if the vehicle does not take off quickly enough after arming (the disarm time is configurable).
- Automatically disarm/prearm shortly after landing (the time is configurable).
- 기체는 정상 상태가 아니면, 시동은 걸리지 않습니다.
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

## 비행 모드

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
- [Drive Modes (Rover)](../flight_modes_rover/index.md)

Instructions for how to set up your remote control switches to enable different flight modes is provided in [Flight Mode Configuration](../config/flight_mode.md).

PX4 also supports external modes implemented in [ROS 2](../ros2/index.md) using the [PX4 ROS 2 Control Interface](../ros2/px4_ros2_control_interface.md).
These are indistinguishable from PX4 internal modes, and can be used to override internal modes with a more advanced version, or to create entirely new functionality.
Note that these depend on ROS 2 and can therefore only run on systems that have a [companion computer](#offboard-companion-computer).

## 안전 설정(사고 방지)

PX4는 시스템 사고시에 기체을 보호하고 복구할 수 있는 안전 시스템이 있으며, 이와 관련된 여러가지 설정들이 있습니다.
안정 설정으로 안전 비행 지역과 조건을 지정하고, 안전 장치에서 수행하는 작업(예: 착륙, 위치 유지 또는 지정된 지점으로 복귀)을 설정할 수 있습니다.

:::info
You can only specify the action for the _first_ failsafe event.
이벤트가 발생하면, 시스템은 특별한 처리 코드를 실행하여 안전 장치 트리거가 분리된 시스템에서 기체별 코드에 의해 관리되도록 합니다.
:::

주요 안전장치는 다음과 같습니다.

- 배터리 부족
- RC(원격 제어) 신호 상실
- 위치 상실(GPS 전역 위치 추정 품질이 너무 낮음)
- 외부 보드 연결 손실(예: 보조 컴퓨터와의 연결이 끊어짐)
- 데이터 링크 손실(예: GCS에 대한 텔레메트리 연결이 끊어짐)
- 지리적 경계 위반(가상 실린더 내부로 기체 비행을 제한합니다)
- 미션 안전장치(재 이륙 시 이전 미션이 실행되는 것을 방지합니다)
- 트래픽 회피(예: ADSB 응답기에 의해 작동됩니다)

For more information see: [Safety](../config/safety.md) (Basic Configuration).

## 전진 방향

차량, 보트 및 항공기에는 전진 방향이 정해져 있습니다.

![Frame Heading](../../assets/concepts/frame_heading.png)

:::info
For a VTOL Tailsitter the heading is relative to the multirotor configuration (i.e. vehicle pose during takeoff, hovering, landing).
:::

차량의 전진 방향을 알아야만 차량의 이동 벡터와 정렬할 수 있습니다.
멀티콥터는 모든 방향에서 대칭인 경우에도 전진 방향이 정의됩니다.
Usually manufacturers use a coloured props or coloured arms to indicate the heading.

![Frame Heading TOP](../../assets/concepts/frame_heading_top.png)

In our illustrations we will use red colouring for the front propellers of multicopter to show heading.

You can read in depth about heading in [Flight Controller Orientation](../config/flight_controller_orientation.md)
