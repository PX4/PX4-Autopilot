# 硬体仿真(HITL)

:::warning
硬体仿真被 [社区支持和维护](../simulation/community_supported_simulators.md)
It may or may not work with current versions of PX4.

See [Toolchain Installation](../dev_setup/dev_env.md) for information about the environments and tools supported by the core development team.
:::

硬件在环仿真模式 (HITL 或 HIL) 下 PX4 固件代码运行在真实的飞行控制器硬件平台上。
这种方法的优点是可以在实际硬件上测试大多数的实际飞行代码。

PX4 支持多轴( [jMAVSim](../sim_jmavsim/index.md)或[Gazebo Classic](../sim_gazebo_classic/index.md))及VTOL (using Gazebo Classic)的仿真。

<a id="compatible_airframe"></a>

## HITL兼容机架

机架与模拟器兼容情况：

| 机架                                                                                                               | `SYS_AUTOSTART` | Gazebo Classic | jMAVSim |
| ---------------------------------------------------------------------------------------------------------------- | --------------- | -------------- | ------- |
| [HIL Quadcopter X](../airframes/airframe_reference.md#copter_simulation_hil_quadcopter_x)                        | 1002            | Y              | Y       |
| [HIL Standard VTOL QuadPlane](../airframes/airframe_reference.md#vtol_standard_vtol_hil_standard_vtol_quadplane) | 4001            | Y              |         |
| [Generic Quadrotor x](../airframes/airframe_reference.md#copter_quadrotor_x_generic_quadcopter) copter           | 4011            | Y              | Y       |

<a id="simulation_environment"></a>

## HITL 仿真环境

硬件在环仿真（HITL）模式下标准的 PX4 固件在真实的硬件上运行。
JMAVSim or Gazebo Classic (running on a development computer) are connected to the flight controller hardware via USB/UART.
The simulator acts as gateway to share MAVLink data between PX4 and _QGroundControl_.

:::info
The simulator can also be connected via UDP if the flight controller has networking support and uses a stable, low-latency connection (e.g. a wired Ethernet connection - WiFi is usually not sufficiently reliable).
For example, this configuration has been tested with PX4 running on a Raspberry Pi connected via Ethernet to the computer (a startup configuration that includes the command for running jMAVSim can be found [here](https://github.com/PX4/PX4-Autopilot/blob/main/posix-configs/rpi/px4_hil.config)).
:::

The diagram below shows the simulation environment:

- A HITL configuration is selected (via _QGroundControl_) that doesn't start any real sensors.
- _jMAVSim_ or _Gazebo Classic_ are connected to the flight controller via USB.
- The simulator is connected to _QGroundControl_ via UDP and bridges its MAVLink messages to PX4.
- _Gazebo Classic_ and _jMAVSim_ can also connect to an offboard API and bridge MAVLink messages to PX4.
- (Optional) A serial connection can be used to connect Joystick/Gamepad hardware via _QGroundControl_.

![HITL Setup - jMAVSim and Gazebo Classic](../../assets/simulation/px4_hitl_overview_jmavsim_gazebo.svg)

## HITL 相比于 SITL

相比之下， HITL 在正常飞控硬件平台上运行正常的处于 ”HITL 模式“ 的 PX4 固件。
仿真数据进入整个仿真系统的时间点与 SITL 有所不同。

By contrast, HITL runs normal PX4 firmware in "HITL mode", on normal hardware.
The simulation data enters the system at a different point than for SITL.
Core modules like commander and sensors have HITL modes at startup that bypass some of the normal functionality.

完成所有的配置设定后 <strong x-id="1">关闭</strong> <em x-id="3">QGroundControl</em> 并断开飞控板与计算机的连接。

## 配置 HITL

### PX4 配置

1. Connect the autopilot directly to _QGroundControl_ via USB.

2. 激活 HITL 模式

   1. Open **Setup > Safety** section.
   2. Enable HITL mode by selecting **Enabled** from the _HITL Enabled_ list:

      ![QGroundControl HITL configuration](../../assets/gcs/qgc_hitl_config.png)

3. 选择机架

   1. Open **Setup > Airframes**
   2. Select a [compatible airframe](#compatible_airframe) you want to test.
      Then click **Apply and Restart** on top-right of the _Airframe Setup_ page.

      ![Select Airframe](../../assets/gcs/qgc_hil_config.png)

4. 如有必要, 校准您的 RC 遥控器 或操纵杆。

5. 设置 UDP

   1. Under the _General_ tab of the settings menu, uncheck all _AutoConnect_ boxes except for **UDP**.

      ![QGC Auto-connect settings for HITL](../../assets/gcs/qgc_hitl_autoconnect.png)

6. (可选) 配置操纵杆和故障保护。
   Set the following [parameters](../advanced_config/parameters.md) in order to use a joystick instead of an RC remote control transmitter:

   - [COM_RC_IN_MODE](../advanced_config/parameter_reference.md#COM_RC_IN_MODE) to "Joystick/No RC Checks". 这允许操纵杆输入并禁用 RC 输入检查。
   - [NAV_RCL_ACT](../advanced_config/parameter_reference.md#NAV_RCL_ACT) to "Disabled". 这可确保在没有无线遥控的情况下运行 HITL 时 RC 失控保护不会介入。

   :::tip
   The _QGroundControl User Guide_ also has instructions on [Joystick](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/joystick.html) and [Virtual Joystick](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/virtual_joystick.html) setup.

:::

Once configuration is complete, **close** _QGroundControl_ and disconnect the flight controller hardware from the computer.

### X-Plane HITL 仿真环境

总而言之， HITL 在真实硬件上运行标准 PX4 固件，而 SITL 实际上要比标准 PX4 系统执行更多的代码。

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
   The correct device will be the last one shown.

:::

4. Set up the environment variables:

   ```sh
   source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
   ```

   and run Gazebo Classic in HITL mode:

   ```sh
   gazebo Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_iris.world
   ```

5. Start _QGroundControl_.
   It should autoconnect to PX4 and Gazebo Classic.

#### jMAVSim (仅适用于四旋翼无人机)

:::info
Make sure _QGroundControl_ is not running!
:::

1. 将飞行控制器连接到计算机, 并等待其启动。

2. 在 HITL 模式下运行 jMAVSim (r如有必要，修改串口号名称 <code>/dev/ttyACM0</code> - 比如，在 Mac OS 上该参数应为 <code>/dev/tty.usbmodem1</code>)：
   sh
   ./Tools/jmavsim_run.sh -q -d /dev/ttyACM0 -b 921600 -r 250

   ```sh
   ./Tools/simulation/jmavsim/jmavsim_run.sh -q -s -d /dev/ttyACM0 -b 921600 -r 250
   ```

   ::: info
   Replace the serial port name `/dev/ttyACM0` as appropriate.
   On macOS this port would be `/dev/tty.usbmodem1`.
   On Windows (including Cygwin) it would be the COM1 or another port - check the connection in the Windows Device Manager.

:::

3. Start _QGroundControl_.
   它应该会自动连接 PX4 和 Gazebo 。

## 在 HITL 仿真中执行自主飞行任务

You should be able to use _QGroundControl_ to [run missions](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/fly_view/fly_view.html#missions) and otherwise control the vehicle.
