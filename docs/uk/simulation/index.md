# Симуляція

Симулятори дозволяють польотному коду PX4 керувати комп'ютерно змодельованим апаратом у змодельованому "світі".
You can interact with this vehicle just as you might with a real vehicle, using _QGroundControl_, an offboard API, or a radio controller/gamepad.

:::tip
Simulation is a quick, easy, and most importantly, _safe_ way to test changes to PX4 code before attempting to fly in the real world.
Це також хороший спосіб почати літати з PX4, якщо у вас ще немає апарату для експериментів.
:::

PX4 supports both _Software In the Loop (SITL)_ simulation, where the flight stack runs on computer (either the same computer or another computer on the same network) and _Hardware In the Loop (HITL)_ simulation using a simulation firmware on a real flight controller board.

Інформація про доступні тренажери та способи їх налаштування наведена в наступному розділі.
The other sections provide general information about how the simulator works, and are not required to _use_ the simulators.

## Підтримувані симулятори

Наступні симулятори підтримуються основною командою розробників PX4.

| Симулятор                                        | Опис                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| ------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [Gazebo](../sim_gazebo_gz/index.md)              | Gazebo supersedes [Gazebo Classic](../sim_gazebo_classic/index.md), featuring more advanced rendering, physics and sensor models. It is the only version of Gazebo available from Ubuntu Linux 22.04<br><br>A powerful 3D simulation environment that is particularly suitable for testing object-avoidance and computer vision. Він також може бути використаний для [multi-vehicle simulation](../simulation/multi-vehicle-simulation.md) і зазвичай використовується з [ROS](../simulation/ros_interface.md), набором інструментів для автоматизації керування апаратами. <br><br><strong>Supported Vehicles:</strong> Quad, VTOL (Standard, Tailsitter, Tiltroter), Plane, Rovers |
| [Gazebo Classic](../sim_gazebo_classic/index.md) | A powerful 3D simulation environment that is particularly suitable for testing object-avoidance and computer vision. It can also be used for [multi-vehicle simulation](../simulation/multi-vehicle-simulation.md) and is commonly used with [ROS](../simulation/ros_interface.md), a collection of tools for automating vehicle control.<br><br>**Supported Vehicles:** Quad ([Iris](../airframes/airframe_reference.md#copter_quadrotor_x_generic_quadcopter)), Hex (Typhoon H480), [Generic Standard VTOL (QuadPlane)](../airframes/airframe_reference.md#vtol_standard_vtol_generic_standard_vtol), Tailsitter, Plane, Rover, Submarine                                     |

There are also a number of [Community Supported Simulators](../simulation/community_supported_simulators.md).

---

Решта цієї теми - це "дещо загальний" опис того, як працює інфраструктура симуляції.
It is not required to _use_ the simulators.

## Симулятор MAVLink API

Всі симулятори, крім Gazebo, взаємодіють з PX4 за допомогою API симулятора MAVLink.
Цей API визначає набір повідомлень MAVLink, які передають дані датчиків з модельованого світу в PX4 і повертають значення двигуна і приводу з польотного коду, які будуть застосовані до модельованого апарату.
На зображенні нижче показано потік повідомлень.

![Simulator MAVLink API](../../assets/simulation/px4_simulator_messages.svg)

:::info
A SITL build of PX4 uses [SimulatorMavlink.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/simulation/simulator_mavlink/SimulatorMavlink.cpp) to handle these messages while a hardware build in HIL mode uses [mavlink_receiver.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_receiver.cpp).
Дані датчиків з симулятора записуються в теми PX4 uORB.
Всі двигуни/приводи заблоковані, але внутрішнє програмне забезпечення повністю функціонує.
:::

Повідомлення описані нижче (див. посилання для більш детальної інформації).

| Повідомлення                                                         | Напрямок   | Опис                                                                                                                                                                                                                                                                                                                                                                            |
| -------------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [MAV\_MODE:MAV\_MODE\_FLAG\_HIL\_ENABLED][mav_mode_flag_hil_enabled] | NA         | Прапорець режиму при використанні симуляції. Всі двигуни/приводи заблоковані, але внутрішнє програмне забезпечення повністю функціонує.                                                                                                                                                                                                         |
| [HIL\_ACTUATOR\_CONTROLS][hil_actuator_controls]                     | PX4 -> Sim | Виходи керування PX4 (до двигунів, приводів).                                                                                                                                                                                                                                                                                                |
| [HIL\_SENSOR][hil_sensor]                                            | Sim -> PX4 | Імітація показань IMU в одиницях СІ в рамі корпусу NED.                                                                                                                                                                                                                                                                                                         |
| [HIL\_GPS][hil_gps]                                                  | Sim -> PX4 | Симульоване значення датчика GPS RAW.                                                                                                                                                                                                                                                                                                                           |
| [HIL\_OPTICAL\_FLOW][hil_optical_flow]                               | Sim -> PX4 | Імітація оптичного потоку від датчика потоку (наприклад, PX4FLOW або датчика оптичної миші)                                                                                                                                                                                                                                                                  |
| [HIL\_STATE\_QUATERNION][hil_state_quaternion]                       | Sim -> PX4 | Містить фактичне "змодельоване" положення апарату, орієнтацію, швидкість і т.д. Це може бути записано в журнал і співставлено з оцінками PX4 для аналізу і діагностики (наприклад, перевірка того, наскільки добре оцінювач працює для зашумлених (імітованих) вхідних сигналів датчика). |
| [HIL\_RC\_INPUTS\_RAW][hil_rc_inputs_raw]                            | Sim -> PX4 | Отримані RAW-значення каналів РК.                                                                                                                                                                                                                                                                                                                               |

<!-- links for table above -->

[mav_mode_flag_hil_enabled]: https://mavlink.io/en/messages/common.html#MAV_MODE_FLAG_HIL_ENABLED
[hil_actuator_controls]: https://mavlink.io/en/messages/common.html#HIL_ACTUATOR_CONTROLS
[hil_sensor]: https://mavlink.io/en/messages/common.html#HIL_SENSOR
[hil_gps]: https://mavlink.io/en/messages/common.html#HIL_GPS
[hil_optical_flow]: https://mavlink.io/en/messages/common.html#HIL_OPTICAL_FLOW
[hil_state_quaternion]: https://mavlink.io/en/messages/common.html#HIL_STATE_QUATERNION
[hil_rc_inputs_raw]: https://mavlink.io/en/messages/common.html#HIL_RC_INPUTS_RAW

<!-- above ^^^ links for table -->

PX4 directly uses the [Gazebo API](https://gazebosim.org/docs) to interface with [Gazebo](../sim_gazebo_gz/index.md) and MAVlink is not required.

## Порти UDP PX4 MAVLink за замовчуванням

By default, PX4 uses commonly established UDP ports for MAVLink communication with ground control stations (e.g. _QGroundControl_), Offboard APIs (e.g. MAVSDK, MAVROS) and simulator APIs (e.g. Gazebo).
Ці порти:

- PX4's remote UDP Port **14550** is used for communication with ground control stations.
  Очікується, що GCS прослуховуватиме з'єднання на цьому порту.
  _QGroundControl_ listens to this port by default.
- PX4's remote UDP Port **14540** is used for communication with offboard APIs.
  Очікується, що зовнішні API будуть чекати на з'єднання через цей порт.
  ::: info
  Multi-vehicle simulations use a separate remote port for each instance, allocated sequentially from `14540` to `14549`
  (additional instances all use port `14549`).

:::
- The simulator's local TCP Port, **4560**, is used for communication with PX4.
  Симулятор слухає цей порт, і PX4 ініціює TCP-з'єднання з ним.

:::info
The ports for the GCS, offboard APIs and simulator are specified by startup scripts.
See [System Startup](../concept/system_startup.md) to learn more.
:::

<!-- A useful discussion about UDP ports here: https://github.com/PX4/PX4-user_guide/issues/1035#issuecomment-777243106 -->

## Середовище симуляції SITL

На схемі нижче показано типове середовище симуляції SITL для будь-якого з підтримуваних тренажерів, що використовують MAVLink (тобто всіх, окрім Gazebo).

![PX4 SITL overview](../../assets/simulation/px4_sitl_overview.svg)

Різні частини системи з'єднуються через протокол UDP і можуть працювати як на одному комп'ютері, так і на іншому комп'ютері в тій самій мережі.

- PX4 використовує спеціальний модуль для підключення до локального TCP-порту 4560 симулятора.
  Simulators then exchange information with PX4 using the [Simulator MAVLink API](#simulator-mavlink-api) described above.
  PX4 на SITL і симулятор можуть працювати як на одному комп'ютері, так і на різних комп'ютерах в одній мережі.

  ::: info
  Simulators can also use the _uxrce-dds bridge_ ([XRCE-DDS](../middleware/uxrce_dds.md)) to directly interact with PX4 (i.e. via [UORB topics](../middleware/uorb.md) rather than MAVLink).
  This approach _may_ used by Gazebo Classic for [multi-vehicle simulation](../sim_gazebo_classic/multi_vehicle_simulation.md#build-and-test-xrce-dds).

:::

- PX4 використовує звичайний модуль MAVLink для підключення до наземних станцій і зовнішніх API розробників, таких як MAVSDK або ROS
  - Ground stations listen to PX4's remote UDP port: `14550`
  - External developer APIs listen to PX4's remote UDP port: `14540`.
    For multi-vehicle simulations, PX4 sequentially allocates a separate remote port for each instance from `14540` to `14549` (additional instances all use port `14549`).

- PX4 defines a number of _local_ UDP ports (`14580`,`18570`), which are sometimes used when networking with PX4 running in a container or virtual machine.
  Вони не рекомендуються для "загального" використання і можуть змінюватися в майбутньому.

- A serial connection may be used to connect [Joystick/Gamepad](../config/joystick.md) hardware via _QGroundControl_.

If you use the normal build system SITL `make` configuration targets (see next section) then both SITL and the Simulator will be launched on the same computer and the ports above will automatically be configured.
Ви можете налаштувати додаткові UDP-з'єднання MAVLink та іншим чином змінити середовище моделювання у файлах конфігурації та ініціалізації збірки.

### Запуск/створення симуляції SITL

Система збірки дозволяє дуже легко зібрати і запустити PX4 на SITL, активувати симулятор і з'єднати їх.
Синтаксис (спрощений) виглядає наступним чином:

```sh
make px4_sitl simulator[_vehicle-model]
```

where `simulator` is `gz` (for Gazebo), `gazebo-classic`, `jmavsim` or some other simulator, and vehicle-model is a particular vehicle type supported by that simulator ([Gazebo](../sim_gazebo_gz/index.md) and [jMAVSim](../sim_jmavsim/index.md) only support multicopters at time of writing, while [Gazebo Classic](../sim_gazebo_classic/index.md) supports many different types).

Нижче наведено кілька прикладів, і їх набагато більше на окремих сторінках для кожного з симуляторів:

```sh
# Start Gazebo with the x500 multicopter
make px4_sitl gz_x500

# Start Gazebo Classic with plane
make px4_sitl gazebo-classic_plane

# Start Gazebo Classic with iris and optical flow
make px4_sitl gazebo-classic_iris_opt_flow

# Start JMavSim with iris (default vehicle model)
make px4_sitl jmavsim

# Start PX4 with no simulator (i.e. to use your own "custom" simulator)
make px4_sitl none_iris
```

Симуляцію можна додатково налаштувати за допомогою змінних середовища:

- `PX4_ESTIMATOR`: This variable configures which estimator to use.
  Possible options are: `ekf2` (default), `lpe` (deprecated).
  It can be set via `export PX4_ESTIMATOR=lpe` before running the simulation.

The syntax described here is simplified, and there are many other options that you can configure via _make_ - for example, to set that you wish to connect to an IDE or debugger.
For more information see: [Building the Code > PX4 Make Build Targets](../dev_setup/building_px4.md#px4-make-build-targets).

### Run Simulation Faster than Realtime {#simulation_speed}

SITL can be run faster or slower than real-time when using Gazebo, Gazebo Classic, or jMAVSim.

The speed factor is set using the environment variable `PX4_SIM_SPEED_FACTOR`.

:::info
PX4 SITL and the simulators are run in _lockstep_, which means that they are locked to run at the same speed, and therefore can react appropriately to sensor and actuator messages.
This is what makes it possible to run the simulation at different speeds, and also pause the simulation in order to step through code.
:::

Для додаткової інформації дивіться:

- Gazebo: [Change Simulation Speed](../sim_gazebo_gz/index.md#change-simulation-speed)
- Gazebo Classic: [Change Simulation Speed](../sim_gazebo_classic/index.md#change-simulation-speed) and [Lockstep](../sim_gazebo_classic/index.md#lockstep)
- jMAVSim: [Change Simulation Speed](../sim_jmavsim/index.md#change-simulation-speed) and [Lockstep](../sim_jmavsim/index.md#lockstep)

### Сценарії запуску

Scripts are used to control which parameter settings to use or which modules to start.
They are located in the [ROMFS/px4fmu_common/init.d-posix](https://github.com/PX4/PX4-Autopilot/tree/main/ROMFS/px4fmu_common/init.d-posix) directory, the `rcS` file is the main entry point.
See [System Startup](../concept/system_startup.md) for more information.

### Імітація збоїв та відмов датчиків/обладнання

[Simulate Failsafes](../simulation/failsafes.md) explains how to trigger safety failsafes like GPS failure and battery drain.

## Середовище симуляції HITL

У симуляції з апаратним забезпеченням у контурі (HITL) звичайна прошивка PX4 виконується на реальному обладнані.
The HITL Simulation Environment in documented in: [HITL Simulation](../simulation/hitl.md).

## Інтеграція джойстиків/геймпада

_QGroundControl_ desktop versions can connect to a USB Joystick/Gamepad and send its movement commands and button presses to PX4 over MAVLink.
Це працює як на SITL, так і на HITL симуляціях, і дозволяє вам безпосередньо керувати симульованим апаратом.
Якщо у вас немає джойстика, ви можете керувати апаратом за допомогою екранних віртуальних паличок QGroundControl.

For setup information see the _QGroundControl User Guide_:

- [Joystick Setup](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/joystick.html)
- [Virtual Joystick](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/virtual_joystick.html)

<!-- FYI Airsim info on this setting up remote controls: https://github.com/Microsoft/AirSim/blob/master/docs/remote_controls.md -->

## Симуляція камери

PX4 supports capture of both still images and video from within the [Gazebo Classic](../sim_gazebo_classic/index.md) simulated environment.
This can be enabled/set up as described in [Gazebo Glassic > Video Streaming](../sim_gazebo_classic/index.md#video-streaming).

The simulated camera is a gazebo classic plugin that implements the [MAVLink Camera Protocol](https://mavlink.io/en/protocol/camera.html) <!-- **PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/src/gazebo_geotagged_images_plugin.cpp -->.
PX4 connects/integrates with this camera in _exactly the same way_ as it would with any other MAVLink camera:

1. [TRIG_INTERFACE](../advanced_config/parameter_reference.md#TRIG_INTERFACE) must be set to `3` to configure the camera trigger driver for use with a MAVLink camera
  :::tip
  In this mode the driver just sends a [CAMERA_TRIGGER](https://mavlink.io/en/messages/common.html#CAMERA_TRIGGER) message whenever an image capture is requested.
  For more information see [Cameras Connected to Flight Controller Outputs](../camera/fc_connected_camera.md).

:::
2. PX4 повинен перенаправляти всі команди камери між GCS і (симулятором) MAVLink Camera.
  You can do this by starting [MAVLink](../modules/modules_communication.md#mavlink) with the `-f` flag as shown, specifying the UDP ports for the new connection.

  ```sh
  mavlink start -u 14558 -o 14530 -r 4000 -f -m camera
  ```

  ::: info
  More than just the camera MAVLink messages will be forwarded, but the camera will ignore those that it doesn't consider relevant.

:::

Інші симулятори можуть використовувати такий самий підхід для реалізації підтримки камери.

## Запуск симуляції на віддаленому сервері

Симулятор можна запустити на одному комп'ютері, а доступ до нього отримати з іншого комп'ютера в тій же мережі (або в іншій мережі з відповідною маршрутизацією).
Це може бути корисно, наприклад, якщо ви хочете протестувати програму для безпілотника, що працює на реальному комп'ютері-компаньйоні на фоні змодельованого транспортного засобу.

Це не працює "з коробки", оскільки PX4 за замовчуванням не маршрутизує пакети на зовнішні інтерфейси (щоб уникнути спаму в мережі та втручання різних симуляцій одна в одну).
Замість цього він спрямовує трафік всередину - на "localhost".

Існує декілька способів зробити UDP-пакети доступними на зовнішніх інтерфейсах, як описано нижче.

### Використання MAVLink Router

The [mavlink-router](https://github.com/mavlink-router/mavlink-router) can be used to route packets from localhost to an external interface.

To route packets between SITL running on one computer (sending MAVLink traffic to localhost on UDP port 14550), and QGC running on another computer (e.g. at address `10.73.41.30`) you could:

- Start _mavlink-router_ with the following command:

  ```sh
  mavlink-routerd -e 10.73.41.30:14550 127.0.0.1:14550
  ```

- Use a _mavlink-router_ conf file.

  ```ini
  [UdpEndpoint QGC]
  Mode = Normal
  Address = 10.73.41.30
  Port = 14550

  [UdpEndpoint SIM]
  Mode = Eavesdropping
  Address = 127.0.0.1
  Port = 14550
  ```

:::info
More information about _mavlink-router_ configuration can be found [here](https://github.com/mavlink-router/mavlink-router#running).
:::

### Увімкнення трансляції UDP

The [mavlink module](../modules/modules_communication.md#mavlink_usage) routes to _localhost_ by default, but you can enable UDP broadcasting of heartbeats using its `-p` option.
Any remote computer on the network can then connect to the simulator by listening to the appropriate port (i.e. 14550 for _QGroundControl_).

:::info
UDP broadcasting provides a simple way to set up the connection when there is only one simulation running on the network.
Do not use this approach if there are multiple simulations running on the network (you might instead [publish to a specific address](#enable-streaming-to-specific-address)).
:::

This should be done in an appropriate configuration file where `mavlink start` is called.
For example: [/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink](https://github.com/PX4/PX4-Autopilot/blob/main/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink).

### Увімкнення стрімінгу на певну адресу

The [mavlink module](../modules/modules_communication.md#mavlink_usage) routes to _localhost_ by default, but you can specify an external IP address to stream to using its `-t` option.
The specified remote computer can then connect to the simulator by listening to the appropriate port (i.e. 14550 for _QGroundControl_).

This should be done in various configuration files where `mavlink start` is called.
For example: [/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink](https://github.com/PX4/PX4-Autopilot/blob/main/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink).

### Тунелювання по SSH

Тунелювання SSH є гнучким варіантом, оскільки комп'ютер для моделювання та система, що його використовує, не обов'язково повинні знаходитися в одній мережі.

:::info
You might similarly use VPN to provide a tunnel to an external interface (on the same network or another network).
:::

Одним із способів створення тунелю є використання параметрів тунелювання SSH.
The tunnel itself can be created by running the following command on _localhost_, where `remote.local` is the name of a remote computer:

```sh
ssh -C -fR 14551:localhost:14551 remote.local
```

UDP-пакети потрібно перетворити на TCP-пакети, щоб їх можна було перенаправляти через SSH.
The [netcat](https://en.wikipedia.org/wiki/Netcat) utility can be used on both sides of the tunnel - first to convert packets from UDP to TCP, and then back to UDP at the other end.

:::tip
QGC must be running before executing _netcat_.
:::

On the _QGroundControl_ computer, UDP packet translation may be implemented by running following commands:

```sh
mkfifo /tmp/tcp2udp
netcat -lvp 14551 < /tmp/tcp2udp | netcat -u localhost 14550 > /tmp/tcp2udp
```

Команда на стороні симулятора тунелю SSH:

```sh
mkfifo /tmp/udp2tcp
netcat -lvup 14550 < /tmp/udp2tcp | netcat localhost 14551 > /tmp/udp2tcp
```

The port number `14550` is valid for connecting to QGroundControl or another GCS, but should be adjusted for other endpoints (e.g. developer APIs etc.).

The tunnel may in theory run indefinitely, but _netcat_ connections may need to be restarted if there is a problem.

The [QGC_remote_connect.bash](https://raw.githubusercontent.com/ThunderFly-aerospace/sitl_gazebo/autogyro-sitl/scripts/QGC_remote_connect.bash) script can be run on the QGC computer to automatically setup/run the above instructions.
Симуляція вже має бути запущена на віддаленому сервері, і ви повинні мати доступ по SSH до цього сервера.
