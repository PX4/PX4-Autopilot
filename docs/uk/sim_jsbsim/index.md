# Симуляція JSBSim

:::warning
This simulator is [community supported and maintained](../simulation/community_supported_simulators.md).
Це може працювати або не працювати з поточними версіями PX4.

Дивіться [Встановлення інструментарію](../dev_setup/dev_env.md) для інформації про середовища та інструменти, що підтримуються основною командою розробників.
:::

[JSBSim](http://jsbsim.sourceforge.net/index.html) is a open source flight simulator ("flight dynamics model (FDM)") that runs on Microsoft Windows, Apple Macintosh, Linux, IRIX, Cygwin (Unix on Windows), etc.
Серед його можливостей: повністю налаштована аеродинаміка та система приводу, яка може моделювати складну динаміку польоту літака.
В обчислення також враховуються обертальні ефекти Землі.

**Supported Vehicles:** Plane, Quadrotor, Hexarotor

<lite-youtube videoid="y5azVNmIVyw" title="JSBSim with APX4 Software-In-The-Loop Simulation"/>

:::info
See [Simulation](../simulation/index.md) for general information about simulators, the simulation environment, and simulation configuration (e.g. supported vehicles).
:::

## Встановлення (Ubuntu Linux)

:::info
These instructions were tested on Ubuntu 18.04
:::

1. Install the usual [Development Environment on Ubuntu LTS / Debian Linux](../dev_setup/dev_env_linux_ubuntu.md).

2. Install a JSBSim release from the [release page](https://github.com/JSBSim-Team/jsbsim/releases/tag/Linux):

   ```sh
   dpkg -i JSBSim-devel_1.1.0.dev1-<release-number>.bionic.amd64.deb
   ```

3. (Необов’язково) FlightGear можна (необов’язково) використовувати для візуалізації.
   To install FlightGear, refer to the [FlightGear installation instructions](../sim_flightgear/index.md)).

## Запуск симуляції

JSBSim SITL simulation can be conveniently run through a `make` command as shown below:

```sh
cd /path/to/PX4-Autopilot
make px4_sitl jsbsim
```

Це запустить інстанцію PX4 SITL та інтерфейс FlightGear (для візуалізації).
If you want to run without the FlightGear UI, you can add `HEADLESS=1` to the front of the `make` command.

The supported vehicles and `make` commands are listed below (click on the links to see the vehicle images).

| Транспортний засіб | Команда                            |
| ------------------ | ---------------------------------- |
| Стандартний літак  | `make px4_sitl jsbsim_rascal`      |
| Квадротор          | `make px4_sitl jsbsim_quadrotor_x` |
| Гексаротор         | `make px4_sitl jsbsim_hexarotor_x` |

Вищенаведені команди запускають єдиний засіб з повним користувацьким інтерфейсом.
_QGroundControl_ should be able to automatically connect to the simulated vehicle.

## Виконання JSBSim з ROS

Щоб запустити JSBSim з прошивками:

1. Clone the `px4-jsbsim-bridge` package into your catkin workspace:

   ```sh
   cd <path_to_catkin_ws>/src
   git clone https://github.com/Auterion/px4-jsbsim-bridge.git
   ```

2. Build the `jsbsim_bridge` catkin package:

   ```sh
   catkin build jsbsim_bridge
   ```

   ::: info
   You must have already set MAVROS in your workspace (if not, follow the instructions in the [MAVROS installation guide](../ros/mavros_installation.md)).

:::

3. Так почніть JSBSim через ROS, використовуючи файл запуску, як показано:

   ```sh
   roslaunch jsbsim_bridge px4_jsbsim_bridge.launch
   ```

## Подальша інформація

- [px4-jsbsim-bridge readme](https://github.com/Auterion/px4-jsbsim-bridge)
