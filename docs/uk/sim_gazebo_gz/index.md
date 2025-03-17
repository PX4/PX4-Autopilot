# Gazebo Simulation

:::warning
Gazebo was previously known as "Gazebo Ignition" (while _Gazebo Classic_ was previously known as Gazebo).
See the [official blog post](https://www.openrobotics.org/blog/2022/4/6/a-new-era-for-gazebo) for more information.
:::

[Gazebo](https://gazebosim.org/home) is an open source robotics simulator.
It supersedes the older [Gazebo Classic](../sim_gazebo_classic/index.md) simulator, and is the only supported version of Gazebo for Ubuntu 22.04 and onwards.

**Supported Vehicles:** Quadrotor, Plane, VTOL, Rover

<lite-youtube videoid="eRzdGD2vgkU" title="PX4 SITL Ignition Gazebo Tunnel Environment"/>

:::info
See [Simulation](../simulation/index.md) for general information about simulators, the simulation environment, and simulation configuration (e.g. supported vehicles).
:::

## Встановлення (Ubuntu Linux)

Gazebo Harmonic is installed by default on Ubuntu 22.04 as part of the normal [development environment setup](../dev_setup/dev_env_linux_ubuntu.md#simulation-and-nuttx-pixhawk-targets).

:::info
The PX4 installation scripts are based on the instructions: [Binary Installation on Ubuntu](https://gazebosim.org/docs/harmonic/install_ubuntu/) (gazebosim.org).
:::

:::warning
Gazebo Harmonic cannot be installed on Ubuntu 20.04 and earlier.

On Ubuntu 20.04 we recommend use [Gazebo Classic](../sim_gazebo_classic/index.md).
If you really must use Gazebo then you should update to Ubuntu 22.04.

Until November 2024 it is possible to [install Gazebo Garden](https://gazebosim.org/docs/garden/install_ubuntu/) on Ubuntu 20.04.
After that date Garden will reach end-of-life and should not be used.
:::

## Запуск симуляції

Gazebo SITL simulation can be conveniently run using a `make` command as shown below:

```sh
cd /path/to/PX4-Autopilot
make px4_sitl gz_x500
```

Це запускає як екземпляр PX4 SITL, так і клієнт Gazebo.

The supported vehicles and `make` commands are listed below.
Note that all gazebo make targets have the prefix `gz_`.

| Транспортний засіб                                                                                                                                                 | Команда                             | `PX4_SYS_AUTOSTART` |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ----------------------------------- | ------------------- |
| [Quadrotor (x500)](../sim_gazebo_gz/vehicles.md#x500-quadrotor)                                                                                 | `make px4_sitl gz_x500`             | 4001                |
| [X500 Quadrotor with Depth Camera (Front-facing)](../sim_gazebo_gz/vehicles.md#x500-quadrotor-with-depth-camera-front-facing)                   | `make px4_sitl gz_x500_depth`       | 4002                |
| [Quadrotor(x500) with Vision Odometry](../sim_gazebo_gz/vehicles.md#x500-quadrotor-with-visual-odometry)                                        | `make px4_sitl gz_x500_vision`      | 4005                |
| [Quadrotor(x500) with 1D LIDAR (Down-facing)](../sim_gazebo_gz/vehicles.md#x500-quadrotor-with-1d-lidar-down-facing)         | `make px4_sitl gz_x500_lidar_down`  | 4016                |
| [Quadrotor(x500) with 2D LIDAR](../sim_gazebo_gz/vehicles.md#x500-quadrotor-with-2d-lidar)                                                      | `make px4_sitl gz_x500_lidar_2d`    | 4013                |
| [Quadrotor(x500) with 1D LIDAR (Front-facing)](../sim_gazebo_gz/vehicles.md#x500-quadrotor-with-1d-lidar-front-facing)       | `make px4_sitl gz_x500_lidar_front` | 4017                |
| [Quadrotor(x500) with gimbal (Front-facing) in Gazebo](../sim_gazebo_gz/vehicles.md#x500-quadrotor-with-gimbal-front-facing) | `make px4_sitl gz_x500_gimbal`      | 4019                |
| [VTOL](../sim_gazebo_gz/vehicles.md#standard-vtol)                                                                                                                 | `make px4_sitl gz_standard_vtol`    | 4004                |
| [Plane](../sim_gazebo_gz/vehicles.md#standard-plane)                                                                                                               | `make px4_sitl gz_rc_cessna`        | 4003                |
| [Advanced Plane](../sim_gazebo_gz/vehicles.md#advanced-plane)                                                                                                      | `make px4_sitl gz_advanced_plane`   | 4008                |
| [Differential Rover](../sim_gazebo_gz/vehicles.md#differential-rover)                                                                                              | `make px4_sitl gz_r1_rover`         | 4009                |
| [Ackermann Rover](../sim_gazebo_gz/vehicles.md#ackermann-rover)                                                                                                    | `make px4_sitl gz_rover_ackermann`  | 4012                |
| [Quad Tailsitter VTOL](../sim_gazebo_gz/vehicles.md#quad-tailsitter-vtol)                                                                                          | `make px4_sitl gz_quadtailsitter`   | 4018                |
| [Tiltrotor VTOL](../sim_gazebo_gz/vehicles.md#tiltrotor-vtol)                                                                                                      | `make px4_sitl gz_tiltrotor`        | 4020                |

All [vehicle models](../sim_gazebo_gz/vehicles.md) (and [worlds](#specify-world)) are included as a submodule from the [Gazebo Models Repository](../sim_gazebo_gz/gazebo_models.md) repository.

Вищенаведені команди запускають єдиний засіб з повним користувацьким інтерфейсом.
_QGroundControl_ should be able to automatically connect to the simulated vehicle.

### Автономний режим

Another way that Gazebo SITL can be connected is in _standalone mode_.
У цьому режимі PX4 SITL і Gazebo запускаються окремо у своїх власних терміналах.
За замовчуванням ці термінали знаходяться на одному й тому ж комп'ютері, але ви можете також під'єднати екземпляри SITL та Gazebo, які працюють на будь-яких двох пристроях в мережі (або навіть різних мережах, якщо ви використовуєте VPN для їх з'єднання).

You start PX4 in standalone mode by prefixing the `make` command with `PX4_GZ_STANDALONE=1`:

```sh
cd /path/to/PX4-Autopilot
PX4_GZ_STANDALONE=1 make px4_sitl gz_x500
```

PX4 SITL will then wait until it detects an instance of _gz-server_, and then connect to it.

:::info
If you have not yet started _gz-server_ when you run the `make` command, you will see the following warning until gazebo has been started and an instance of _gz-server_ is detected by PX4:

```sh
WARN [gz bridge] Service call timed out as Gazebo has not been detected
```

:::

The simplest way to start the simulation is to use the Python script [simulation-gazebo](https://github.com/PX4/PX4-gazebo-models/blob/main/simulation-gazebo), which can be found in the [Gazebo Models Repository](../sim_gazebo_gz/gazebo_models.md) repository.
This can be used to launch a _gz-server_ instance with any supported world and vehicle.

The script can be used without installing any additional dependencies, and will fetch the supported PX4 models and worlds on first use (by default) and save them to `~/.simulation-gazebo`.
При повторному запуску цей скрипт використає цю директорію для отримання моделей та світів.
Therefore if you want to use your own model and run it in standalone mode, you will have to place its source code in `~/.simulation-gazebo`.

You can fetch the script locally using any method you like, such as `wget`:

```sh
wget https://raw.githubusercontent.com/PX4/PX4-gazebo-models/main/simulation-gazebo
```

Скрипт може бути запущено так:

```sh
cd /path/to/script/
python3 simulation-gazebo
```

For more information and arguments, see [Gazebo Models](../sim_gazebo_gz/gazebo_models.md).

:::info
If `make px4_sitl gz_x500` gives the error `ninja: error: unknown target 'gz_x500'` then run `make distclean` to start from a clean slate, and try running `make px4_sitl gz_x500` again.
:::

### Режим без інтерфейсу

Можливо ви забажаєте запустити Gazebo у "режимі без інтерфейсу" (без графічного інтерфейсу Gazebo) оскільки він використовує менше ресурсів та не покладається на те, що ваша система має відеокарту яка належним чином підтримує рендеринг OpenGL.
Це пришвидшує завантаження та запуск, та для багатьох простих випадків це може бути все що вам потрібно.

The simulation can be run in headless mode by prefixing the command with the `HEADLESS=1` environment variable:

```sh
HEADLESS=1 make px4_sitl gz_x500
```

### Вказання світу

Симуляція може бути запущена в певному світі додаванням бажаного світу до імені бажаного рухомого засобу.
For example, to run the windy world with the `x500` vehicle you can specify:

```sh
make px4_sitl gz_x500_windy
```

You can also specify the world using the `PX4_GZ_WORLD` environment variable:

```sh
PX4_GZ_WORLD=windy make px4_sitl gz_x500
```

The [supported worlds](../sim_gazebo_gz/worlds.md) are listed below.

| World      | Команда                    | Опис                                                        |
| ---------- | -------------------------- | ----------------------------------------------------------- |
| `default`  | `make px4_sitl *`          | Порожній світ (сіра площина)             |
| `aruco`    | `make px4_sitl *_aruco`    | Empty world with aruco marker for testing precision landing |
| `baylands` | `make px4_sitl *_baylands` | Світ Baylands оточений водою                                |
| `lawn`     | `make px4_sitl *_lawn`     | Lawn world for testing rovers                               |
| `rover`    | `make px4_sitl *_rover`    | Rover world (optimised/preferred)        |
| `walls`    | `make px4_sitl *_walls`    | Wall world for testing collision prevention                 |
| `windy`    | `make px4_sitl *_windy`    | Порожній світ з увімкненим вітром                           |

:::warning
Note that if no world is specified, PX4 will use the `default` world.
However you must not _explicitly_ specify `_default` on the model as this will prevent PX4 from launching.
In other words, use `make px4_sitl gz_x500` instead of `make px4_sitl gz_x500_default` for the default.
:::

:::info
Baylands world throws a warning in Gazebo Harmonic because there are so many meshes.
This can be ignored:

```sh
[Wrn] [SDFFeatures.cc:843] The geometry element of collision [collision] couldn't be created
```

:::

### Зміна швидкості симуляції

PX4 SITL can be run faster or slower than real-time when using Gazebo.

The speed factor is set using the environment variable `PX4_SIM_SPEED_FACTOR`.
For example, to run the Gazebo simulation of the X500 frame at 2 times the real time speed:

```sh
PX4_SIM_SPEED_FACTOR=2 make px4_sitl gz_x500
```

Запустити в половину реального часу:

```sh
PX4_SIM_SPEED_FACTOR=0.5 make px4_sitl gz_x500
```

You can apply the factor to all SITL runs in the current session using `EXPORT`:

```sh
export PX4_SIM_SPEED_FACTOR=2
make px4_sitl gz_x500
```

:::info
At some point IO or CPU will limit the speed that is possible on your machine and it will be slowed down "automatically".
Потужні комп'ютери зазвичай можуть запускати симуляцію зі швидкістю близько 6-10 разів, для ноутбуків досягається швидкість близько 3-4 разів.
:::

:::info
The simulators are run in _lockstep_, which means that Gazebo runs the simulator at the same speed as PX4 (the GZBridge sets the PX4 time on every sim step, in the `clockCallback`).
In addition to being a precondition for running the simulation faster/slower than real-time, this also allows you to pause the simulation in order to step through code.
Lockstep cannot be disabled on Gazebo.
:::

## Використання/Налаштування

Конвеєр запуску дозволяє дуже гнучке налаштування.
Зокрема можливо:

- Почати нову симуляцію з довільним світом або під'єднатись до вже запущеної симуляції.
- Додати новий засіб до симуляції або поєднати новий екземпляр PX4 з вже наявним.

Ці сценарії керуються встановленням відповідних змінних середовища.

### Синтаксис

Синтаксис запуску набирає форми:

```sh
ARGS ./build/px4_sitl_default/bin/px4
```

where `ARGS` is a list of environment variables including:

- `PX4_SYS_AUTOSTART` (**Mandatory**):
  Sets the [airframe autostart id](../dev_airframes/adding_a_new_frame.md) of the PX4 airframe to start.

- `PX4_GZ_MODEL_NAME`:
  Sets the name of an _existing_ model in the gazebo simulation.
  Якщо вказано, скрипт запуску намагається прив'язати новий екземпляр PX4 до ресурсу Gazebo, що відповідає точно такому імені.

  - The setting is mutually exclusive with `PX4_SIM_MODEL`.

- `PX4_SIM_MODEL`:
  Sets the name of a new Gazebo model to be spawned in the simulator.
  Якщо вказано, скрипт запуску шукає модель у шляху ресурсів Gazebo, що відповідає даній змінній, відтворює її й пов'язує новий екземпляр PX4 з нею.

  - The setting is mutually exclusive with `PX4_GZ_MODEL_NAME`.

  ::: info
  The environmental variable `PX4_GZ_MODEL` has been deprecated and its functionality merged into `PX4_SIM_MODEL`.

:::

- `PX4_GZ_MODEL_POSE`:
  Sets the spawning position and orientation of the model when `PX4_SIM_MODEL` is adopted.
  If provided, the startup script spawns the model at a pose following the syntax `"x,y,z,roll,pitch,yaw"`, where the positions are given in metres and the angles are in radians.

  - If omitted, the zero pose `[0,0,0,0,0,0]` is used.
  - Якщо надано менше ніж 6 значень, то відсутні фіксуються на нулі.
  - This can only be used with `PX4_SIM_MODEL` (not `PX4_GZ_MODEL_NAME`).

- `PX4_GZ_WORLD`:
  Sets the Gazebo world file for a new simulation.
  If it is not given, then [default](https://github.com/PX4/PX4-gazebo-models/blob/main/worlds/default.sdf) is used.

  - Ця змінна ігнорується, якщо наявна модель вже запущена.
  - This value should be [specified for the selected airframe](#adding-new-worlds-and-models) but may be overridden using this argument.

- `PX4_SIMULATOR=GZ`:
  Sets the simulator, which for Gazebo must be `gz`.

  - This value should be [set for the selected airframe](#adding-new-worlds-and-models), in which case it does not need to be set as an argument.

- `PX4_GZ_STANDALONE`:
  Lets PX4 know that it should not launch an instance of Gazebo.
  Gazebo will need to be launched separately, as described in [Standalone Mode](#standalone-mode).

- `PX4_GZ_SIM_RENDER_ENGINE`:
  Sets the render engine to be used by gazebo.

  Рушій рендерингу за замовчуванням (OGRE 2) погано підтримується на деяких платформах/середовищах.
  Specify `PX4_GZ_SIM_RENDER_ENGINE=ogre` to set the rendering engine to OGRE 1 if you have rendering issues when running PX4 on a virtual machine.

- `PX4_SIM_SPEED_FACTOR`:
  Sets the speed factor to run the simulation at [faster/slower than realtime](#change-simulation-speed).

- `PX4_GZ_FOLLOW_OFFSET_X`, `PX4_GZ_FOLLOW_OFFSET_Y`, `PX4_GZ_FOLLOW_OFFSET_Z`:
  Set the relative offset of the follow camera to the vehicle.

The PX4 Gazebo worlds and and models databases [can be found on GitHub here](https://github.com/PX4/PX4-gazebo-models).

:::info
`gz_env.sh.in` is compiled and made available in `$PX4_DIR/build/px4_sitl_default/rootfs/gz_env.sh`
:::

### Приклади

Ось кілька прикладів різних сценаріїв, описаних вище.

1. **Start simulator + default world + spawn vehicle at default pose**

  ```sh
  PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4
  ```

2. **Start simulator + default world + spawn vehicle at custom pose (y=2m)**

  ```sh
  PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,2" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4
  ```

3. **Start simulator + default world + link to existing vehicle**

  ```sh
  PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_NAME=x500 ./build/px4_sitl_default/bin/px4
  ```

4. **Start simulator in standalone mode + connect to Gazebo instance running default world**

  ```sh
  PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4
  ```

  В окремому терміналі запустіть:

  ```sh
  python /path/to/simulation-gazebo
  ```

## Додавання нових світів та моделей

SDF files, mesh files, textures and anything else to do with the functionality and appearance in Gazebo for worlds and models can be placed in the appropriate `/worlds` and `/models` directories in [PX4-gazebo-models](https://github.com/PX4/PX4-gazebo-models).

В межах PX4 дотримуйтесь наступних кроків, щоб додати моделі та світи.

### Додавання моделі

Щоб додати нову модель:

1. Define an [airframe configuration file](../dev_airframes/adding_a_new_frame.md).

2. Define the default parameters for Gazebo in the airframe configuration file (this example is from [x500 quadcopter](https://github.com/PX4/PX4-Autopilot/blob/main/ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500)):

  ```ini
  PX4_SIMULATOR=${PX4_SIMULATOR:=gz}
  PX4_GZ_WORLD=${PX4_GZ_WORLD:=default}
  PX4_SIM_MODEL=${PX4_SIM_MODEL:=<your model name>}
  ```

  - `PX4_SIMULATOR=${PX4_SIMULATOR:=gz}` sets the default simulator (Gz) for that specific airframe.

  - `PX4_GZ_WORLD=${PX4_GZ_WORLD:=default}` sets the [default world](https://github.com/PX4/PX4-gazebo-models/blob/main/worlds/default.sdf) for that specific airframe.

  - Setting the default value of `PX4_SIM_MODEL` lets you start the simulation with just:

    ```sh
    PX4_SYS_AUTOSTART=<your new airframe id> ./build/px4_sitl_default/bin/px4
    ```

3. Add CMake Target for the [airframe](https://github.com/PX4/PX4-Autopilot/blob/main/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt).

  - If you plan to use "regular" mode, add your model SDF to `Tools/simulation/gz/models/`.
  - If you plan to use _standalone_ mode, add your model SDF to `~/.simulation-gazebo/models/`

  Ви звичайно також можете використовувати обидва варіанти.

### Додавання світу

Щоб додати новий світ:

1. Add your world to the list of worlds found in the [`CMakeLists.txt` here](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/simulation/gz_bridge/CMakeLists.txt).
  This is required in order to allow `CMake` to generate correct targets.

  - If you plan to use "normal" mode, add your world sdf to `Tools/simulation/gz/worlds/`.
  - If you plan to use _standalone_ mode, add your world SDF to `~/.simulation-gazebo/worlds/`

:::info
As long as the world file and the model file are in the Gazebo search path (`GZ_SIM_RESOURCE_PATH`) it is not necessary to add them to the PX4 world and model directories.
However, `make px4_sitl gz_<model>_<world>` won't work with them.
:::

## Симуляція кількох рухомих засобів

Симуляція кількох засобів підтримується на комп'ютерах з Linux.

For more information see: [Multi-Vehicle Simulation with Gazebo](../sim_gazebo_gz/multi_vehicle_simulation.md)

## Подальша інформація

- [px4-simulation-ignition](https://github.com/Auterion/px4-simulation-ignition)
