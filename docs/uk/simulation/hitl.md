# Симуляція з апаратним забезпеченням в контурі (HITL)

:::warning
HITL is [community supported and maintained](../simulation/community_supported_simulators.md).
Це може працювати або не працювати з поточними версіями PX4.

Дивіться [Встановлення інструментарію](../dev_setup/dev_env.md) для інформації про середовища та інструменти, що підтримуються основною командою розробників.
:::

Апаратне забезпечення в контурі (HITL або HIL) режим симуляції в якому звичайна прошивка PX4 виконується на реальному пристрої польотного контролера.
Цей підхід має перевагу у вигляді можливості тестування більшості коду для польоту на реальному апаратному забезпеченні.

PX4 supports HITL for multicopters (using [jMAVSim](../sim_jmavsim/index.md) or [Gazebo Classic](../sim_gazebo_classic/index.md)) and VTOL (using Gazebo Classic).

<a id="compatible_airframe"></a>

## Планери сумісні з HITL

The set of compatible airframes vs simulators is:

| Планер                                                                                                           | `SYS_AUTOSTART` | Gazebo Classic | jMAVSim |
| ---------------------------------------------------------------------------------------------------------------- | --------------- | -------------- | ------- |
| [HIL Quadcopter X](../airframes/airframe_reference.md#copter_simulation_hil_quadcopter_x)                        | 1001            | Y              | Y       |
| [HIL Standard VTOL QuadPlane](../airframes/airframe_reference.md#vtol_standard_vtol_hil_standard_vtol_quadplane) | 1002            | Y              |         |

<a id="simulation_environment"></a>

## Середовище симуляції HITL

У симуляції з апаратним забезпеченням у контурі (HITL) звичайна прошивка PX4 виконується на реальному обладнані.
JMAVSim або Gazebo Classic (які працюють на комп'ютері розробки) підключені до пристрою польотного контролера через USB/UART.
The simulator acts as gateway to share MAVLink data between PX4 and _QGroundControl_.

:::info
The simulator can also be connected via UDP if the flight controller has networking support and uses a stable, low-latency connection (e.g. a wired Ethernet connection - WiFi is usually not sufficiently reliable).
For example, this configuration has been tested with PX4 running on a Raspberry Pi connected via Ethernet to the computer (a startup configuration that includes the command for running jMAVSim can be found in [px4_hil.config](https://github.com/PX4/PX4-Autopilot/blob/main/posix-configs/rpi/px4_hil.config)).
:::

Діаграма нижче показує середовище симуляції:

- A HITL configuration is selected (via _QGroundControl_) that doesn't start any real sensors.
- _jMAVSim_ or _Gazebo Classic_ are connected to the flight controller via USB.
- The simulator is connected to _QGroundControl_ via UDP and bridges its MAVLink messages to PX4.
- _Gazebo Classic_ and _jMAVSim_ can also connect to an offboard API and bridge MAVLink messages to PX4.
- (Optional) A serial connection can be used to connect Joystick/Gamepad hardware via _QGroundControl_.

![HITL Setup - jMAVSim and Gazebo Classic](../../assets/simulation/px4_hitl_overview_jmavsim_gazebo.svg)

## HITL у порівнянні з SITL

SITL працює на комп'ютері розробки в модельованому середовищі та використовує прошивку спеціально створену для цього середовища.
Крім драйверів симуляції для забезпечення підроблених даних середовища від симулятора система поводиться як зазвичай.

На противагу, HITL виконує звичайну прошивку PX4 в "режимі HITL" на звичайному обладнані.
Дані симуляції потрапляють в систему в іншій точці ніж для SITL.
Основні модулі на кшталт командного або датчиків мають режими HITL, що оминають частину звичайної функціональності при старті.

Підсумовуючи, HITL виконує PX4 на реальному обладнанні за допомогою стандартної прошивки, а SITL фактично більше виконує стандартний системний код.

## Налаштування HITL

## Check if HITL is in Firmware

The module required for HITL ([`pwm_out_sim`](../modules/modules_driver.md#pwm-out-sim)) is not built into all PX4 firmware by default.

To check if the module is present on your Flight Controller:

1. Open QGroundControl

2. Open **Analyze Tools > Mavlink Console**.

3. Type the following command in the console:

   ```sh
   pwm_out_sim status
   ```

4. If the returned value is `nsh: pwm_out_sim: command not found`, then you don't have the module installed.

If `pwm_out_sim` is not present you will need to add it to the firmware in order to use HITL simulation.

### Adding HITL modules to the Firmware

Add the following key to the configuration file for your flight controller to include the required module (for an example see [boards/px4/fmu-v6x/default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v6x/default.px4board)).
Then re-build the firmware and flash it to the board.

```text
CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y
```

You can alternatively use the following command to launch a GUI configuration tool, and interactively enable them at the path: **modules > Simulation > pwm_out_sim**.
For example, to update fmu-v6x you would use:

```sh
make px4_fmu-v6x boardconfig
```

### Конфігурація PX4

1. Connect the autopilot directly to _QGroundControl_ via USB.

2. Вибір планера
   1. Open **Setup > Airframes**
   2. Select a [compatible airframe](#compatible_airframe) you want to test.
      Then click **Apply and Restart** on top-right of the _Airframe Setup_ page.

3. При необхідності відкалібруйте пульт РК або джойстик.

4. Налаштування UDP
   1. Under the _General_ tab of the settings menu, uncheck all _AutoConnect_ boxes except for **UDP**.

      ![QGC Auto-connect settings for HITL](../../assets/gcs/qgc_hitl_autoconnect.png)

5. (Необов'язково) Налаштуйте джойстик та запобіжник відмови.
   Set the following [parameters](../advanced_config/parameters.md) in order to use a joystick instead of an RC remote control transmitter:

   - [COM_RC_IN_MODE](../advanced_config/parameter_reference.md#COM_RC_IN_MODE) to "Joystick/No RC Checks". Це дозволить керування джойстиком та відключить перевірки пульту РК.
   - [NAV_RCL_ACT](../advanced_config/parameter_reference.md#NAV_RCL_ACT) to "Disabled". Це гарантує, що ніякі дії запобігання відмові не будуть перешкоджати коли не виконується HITL з радіо керуванням.

   :::tip
   The _QGroundControl User Guide_ also has instructions on [Joystick](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/joystick.html) and [Virtual Joystick](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/virtual_joystick.html) setup.

:::

Once configuration is complete, **close** _QGroundControl_ and disconnect the flight controller hardware from the computer.

### Налаштування відповідних симуляторів

Дотримуйтесь відповідних кроків для певного симулятора в наступних розділах.

#### Gazebo Classic

:::info
Make sure _QGroundControl_ is not running!
:::

1. Build PX4 with [Gazebo Classic](../sim_gazebo_classic/index.md) (in order to build the Gazebo Classic plugins).

   ```sh
   cd <Firmware_clone>
   DONT_RUN=1 make px4_sitl_default gazebo-classic
   ```

2. Open the vehicle model's sdf file (e.g. **Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_hitl/iris_hitl.sdf**).

3. Replace the `serialDevice` parameter (`/dev/ttyACM0`) if necessary.

   ::: info
   The serial device depends on what port is used to connect the vehicle to the computer (this is usually `/dev/ttyACM0`).
   An easy way to check on Ubuntu is to plug in the autopilot, open up a terminal, and type `dmesg | grep "tty"`.
   Останній показаний пристрій і буде тим що потрібно.

:::

4. Налаштуйте змінні середовища:

   ```sh
   source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
   ```

   та запустіть Gazebo Classic в режимі HITL:

   ```sh
   gazebo Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_iris.world
   ```

5. Start _QGroundControl_.
   Воно повинно автоматично підключитися до PX4 та Gazebo Classic.

#### jMAVSim (тільки квадрокоптер)

:::info
Make sure _QGroundControl_ is not running!
:::

1. Під'єднайте політний контролер до комп'ютера та дочекайтесь коли він завантажиться.

2. Запустіть jMAVSim в режимі HITL:

   ```sh
   ./Tools/simulation/jmavsim/jmavsim_run.sh -q -s -d /dev/ttyACM0 -b 921600 -r 250
   ```

   ::: info
   Replace the serial port name `/dev/ttyACM0` as appropriate.
   On macOS this port would be `/dev/tty.usbmodem1`.
   На Windows (включно з Cygwin) це буде COM1 або інший порт - перевірте з'єднання в менеджері пристроїв Windows.

:::

3. Start _QGroundControl_.
   Воно повинно автоматично підключитися до PX4 та jMAVSim.

## Політ за автономним завданням у HITL

You should be able to use _QGroundControl_ to [run missions](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/fly_view/fly_view.html#missions) and otherwise control the vehicle.
