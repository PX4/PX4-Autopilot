# Multi-Vehicle Simulation with Gazebo Classic

This topic explains how to simulate multiple UAV vehicles using [Gazebo Classic](../sim_gazebo_classic/index.md) and SITL (Linux only).
A different approach is used for simulation with and without ROS.

## Multiple Vehicle with Gazebo Classic

To simulate multiple iris or plane vehicles in Gazebo Classic use the following commands in the terminal (from the root of the _Firmware_ tree):

```sh
Tools/simulation/gazebo-classic/sitl_multiple_run.sh [-m <model>] [-n <number_of_vehicles>] [-w <world>] [-s <script>] [-t <target>] [-l <label>]
```

- `<model>`: The [vehicle type/model](../sim_gazebo_classic/vehicles.md) to spawn, e.g.: `iris` (default), `plane`, `standard_vtol`, `rover`, `r1_rover` `typhoon_h480`.

- `<number_of_vehicles>`: The number of vehicles to spawn.
  Default is 3.
  Maximum is 254.

- `<world>`: The [world](../sim_gazebo_classic/worlds.md) that the vehicle should be spawned into, e.g.: `empty` (default)

- `<script>`: Spawn multiple vehicles of different types (overriding the values in `-m` and `-n`).
  例如：

  ```sh
  -s "iris:3,plane:2,standard_vtol:3"
  ```

  - Supported vehicle types are: `iris`, `plane`, `standard_vtol`, `rover`, `r1_rover` `typhoon_h480`.
  - The number after the colon indicates the number of vehicles (of that type) to spawn.
  - Maximum number of vehicles is 254.

- `<target>`: build target, e.g: `px4_sitl_default` (default), `px4_sitl_nolockstep`

- `<label>` : specific label for model, e.g: `rplidar`

Each vehicle instance is allocated a unique MAVLink system id (2, 3, 4, etc.).
MAVLink system id 1 is skipped in order to have consistency among [namespaces](../ros2/multi_vehicle.md#principle-of-operation).
Vehicle instances are accessed from sequentially allocated PX4 remote UDP ports: `14541` - `14548` (additional instances are all accessed using the same remote UDP port: `14549`).

:::info
The 254-vehicle limitation occurs because mavlink `MAV_SYS_ID` only supports 255 vehicles in the same network (and the first one is skipped).
The `MAV_SYS_ID` is allocated in the SITL rcS: [init.d-posix/rcS](https://github.com/PX4/PX4-Autopilot/blob/main/ROMFS/px4fmu_common/init.d-posix/rcS#L131)
:::

### Video: Multiple Multicopter (Iris)

<lite-youtube videoid="Mskx_WxzeCk" title="Multiple vehicle simulation in SITL gazebo"/>

### Video: Multiple Plane

<lite-youtube videoid="aEzFKPMEfjc" title="PX4 Multivehicle SITL gazebo for fixedwing"/>

### Video: Multiple VTOL

<lite-youtube videoid="lAjjTFFZebI" title="PX4 Multivehicle SITL gazebo for VTOL"/>

### Build and Test (XRCE-DDS)

`Tools/simulation/gazebo-classic/sitl_multiple_run.sh` can be used to simulate multiple vehicles connected via XRCE-DDS in Gazebo Classic.

:::info
You will need to have installed the XRCE-DDS dependencies.
For more information see: [ROS 2 User Guide (PX4-ROS 2 Bridge)](../ros2/user_guide.md), for interfacing with ROS 2 nodes.
:::

To build an example setup, follow the steps below:

1. Clone the PX4/Firmware code, then build the SITL code:

  ```sh
  cd Firmware_clone
  git submodule update --init --recursive
  DONT_RUN=1 make px4_sitl gazebo-classic
  ```

2. Build the `micro xrce-dds agent` and the interface package following the [instructions here](../ros2/user_guide.md).

3. Run `Tools/simulation/gazebo-classic/sitl_multiple_run.sh`.
  For example, to spawn 4 vehicles, run:

  ```sh
  ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m iris -n 4
  ```

  ::: info
  Each vehicle instance is allocated a unique MAVLink system id (2, 3, 4, etc.).
  MAVLink system id 1 is skipped.

:::

4. Run `MicroXRCEAgent`.
  It will automatically connect to all four vehicles:

  ```sh
  MicroXRCEAgent udp4 -p 8888
  ```

  ::: info
  The simulator startup script automatically assigns a [unique namespace](../ros2/multi_vehicle.md) to each vehicle.

:::

## Multiple Vehicles with MAVROS and Gazebo Classic

This example demonstrates a setup that opens the Gazebo Classic client GUI showing two Iris vehicles in an empty world.
You can then control the vehicles with _QGroundControl_ and MAVROS in a similar way to how you would manage a single vehicle.

### Required

- Current [PX4 ROS/Gazebo development environment](../ros/mavros_installation.md) (which includes the [MAVROS package](http://wiki.ros.org/mavros)).
- a clone of latest [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)

### Build and Test

To build an example setup, follow the step below:

1. Clone the PX4/PX4-Autopilot code, then build the SITL code

  ```sh
  cd Firmware_clone
  git submodule update --init --recursive
  DONT_RUN=1 make px4_sitl_default gazebo-classic
  ```

2. Source your environment:

  ```sh
  source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo
  ```

3. Run launch file:

  ```sh
  roslaunch px4 multi_uav_mavros_sitl.launch
  ```

  ::: info
  You can specify `gui:=false` in the above _roslaunch_ to launch Gazebo Classic without its UI.

:::

The tutorial example opens the Gazebo Classic client GUI showing two Iris vehicles in an empty world.

You can control the vehicles with _QGroundControl_ or MAVROS in a similar way to how you would manage a single vehicle:

- _QGroundControl_ will have a drop-down to select the vehicle that is "in focus"
- MAVROS requires that you include the proper namespace before the topic/service path (e.g. for `<group ns="uav1">` you'll use _/uav1/mavros/mission/push_).

### What's Happening?

For each simulated vehicle, the following is required:

- **Gazebo Classic model**: This is defined as `xacro` file in `PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/rotors_description/urdf/<model>_base.xacro` see [here](https://github.com/PX4/PX4-SITL_gazebo-classic/tree/02060a86652b736ca7dd945a524a8bf84eaf5a05/models/rotors_description/urdf).
  Currently, the model `xacro` file is assumed to end with **base.xacro**.
  This model should have an argument called `mavlink_udp_port` which defines the UDP port on which Gazebo Classic will communicate with PX4 node.
  The model's `xacro` file will be used to generate an `urdf` model that contains UDP port that you select.
  To define the UDP port, set the `mavlink_udp_port` in the launch file for each vehicle, see [here](https://github.com/PX4/PX4-Autopilot/blob/4d0964385b84dc91189f377aafb039d10850e5d6/launch/multi_uav_mavros_sitl.launch#L37) as an example.

  ::: info
  If you are using the same vehicle model, you don't need a separate **`xacro`** file for each vehicle. The same **`xacro`** file is adequate.

:::

- **PX4 node**: This is the SITL PX4 app.
  It communicates with the simulator, Gazebo Classic, through the same UDP port defined in the Gazebo Classic vehicle model, i.e. `mavlink_udp_port`.
  To set the UDP port on the PX4 SITL app side, you need to set the `SITL_UDP_PRT` parameter in the startup file to match the `mavlink_udp_port` discussed previously, see [here](https://github.com/PX4/PX4-Autopilot/blob/4d0964385b84dc91189f377aafb039d10850e5d6/posix-configs/SITL/init/ekf2/iris_2#L46).
  The path of the startup file in the launch file is generated based on the `vehicle` and `ID` arguments, see [here](https://github.com/PX4/PX4-Autopilot/blob/4d0964385b84dc91189f377aafb039d10850e5d6/launch/multi_uav_mavros_sitl.launch#L36).
  The `MAV_SYS_ID` for each vehicle in the startup file, see [here](https://github.com/PX4/PX4-Autopilot/blob/4d0964385b84dc91189f377aafb039d10850e5d6/posix-configs/SITL/init/ekf2/iris_2#L4), should match the `ID` for that vehicle in the launch file [here](https://github.com/PX4/PX4-Autopilot/blob/4d0964385b84dc91189f377aafb039d10850e5d6/launch/multi_uav_mavros_sitl.launch#L25).
  This will help make sure you keep the configurations consistent between the launch file and the startup file.

- **MAVROS node** \(optional\): A separate MAVROS node can be run in the launch file, see [here](https://github.com/PX4/PX4-Autopilot/blob/4d0964385b84dc91189f377aafb039d10850e5d6/launch/multi_uav_mavros_sitl.launch#L41), in order to connect to PX4 SITL app, if you want to control your vehicle through ROS.
  You need to start a MAVLink stream on a unique set of ports in the startup file, see [here](https://github.com/PX4/PX4-Autopilot/blob/4d0964385b84dc91189f377aafb039d10850e5d6/posix-configs/SITL/init/ekf2/iris_1#L68).
  Those unique set of ports need to match those in the launch file for the MAVROS node, see [here](https://github.com/PX4/PX4-Autopilot/blob/4d0964385b84dc91189f377aafb039d10850e5d6/launch/multi_uav_mavros_sitl.launch#L26).

The launch file `multi_uav_mavros_sitl.launch`does the following,

- loads a world in Gazebo Classic,

  ```xml
    <!-- Gazebo sim -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="gui" value="$(arg gui)"/>
        <arg name="world_name" value="$(arg world)"/>
        <arg name="debug" value="$(arg debug)"/>
        <arg name="verbose" value="$(arg verbose)"/>
        <arg name="paused" value="$(arg paused)"/>
    </include>
  ```

- for each vehicle,

  - creates urdf model from xacro, loads gazebo classic model and runs PX4 SITL app instance

    ```xml
      <!-- PX4 SITL and vehicle spawn -->
      <include file="$(find px4)/launch/single_vehicle_spawn.launch">
          <arg name="x" value="0"/>
          <arg name="y" value="0"/>
          <arg name="z" value="0"/>
          <arg name="R" value="0"/>
          <arg name="P" value="0"/>
          <arg name="Y" value="0"/>
          <arg name="vehicle" value="$(arg vehicle)"/>
          <arg name="rcS" value="$(find px4)/posix-configs/SITL/init/$(arg est)/$(arg vehicle)_$(arg ID)"/>
          <arg name="mavlink_tcp_port" value="4560"/>
          <arg name="ID" value="$(arg ID)"/>
      </include>
    ```

  - runs a mavros node

    ```xml
      <!-- MAVROS -->
      <include file="$(find mavros)/launch/px4.launch">
          <arg name="fcu_url" value="$(arg fcu_url)"/>
          <arg name="gcs_url" value=""/>
          <arg name="tgt_system" value="$(arg ID)"/>
          <arg name="tgt_component" value="1"/>
      </include>
    ```

  ::: info
  The complete block for each vehicle is enclosed in a set of `<group>` tags to separate the ROS namespaces of the vehicles.

:::

To add a third iris to this simulation there are two main components to consider:

- add `UAV3` to **multi_uav_mavros_sitl.launch**
  - duplicate the group of either existing vehicle (`UAV1` or `UAV2`)
  - increment the `ID` arg to `3`
  - select a different port for `mavlink_udp_port` arg for communication with Gazebo Classic
  - selects ports for MAVROS communication by modifying both port numbers in the `fcu_url` arg
- create a startup file, and change the file as follows:

  - make a copy of an existing iris rcS startup file (`iris_1` or `iris_2`) and rename it `iris_3`
  - `MAV_SYS_ID` value to `3`
  - `SITL_UDP_PRT` value to match that of the `mavlink_udp_port` launch file arg
  - the first `mavlink start` port and the `mavlink stream` port values to the same values, which is to be used for QGC communication
  - the second `mavlink start` ports need to match those used in the launch file `fcu_url` arg

    ::: info
    Be aware of which port is `src` and `dst` for the different endpoints.

:::

## Multiple Vehicles using SDF Models

This section shows how developers can simulate multiple vehicles using vehicle models defined in Gazebo Classic SDF files (instead of using models defined in the ROS Xacro file, as discussed in the rest of this topic).

步骤如下：

1. Install _xmlstarlet_ from your Linux terminal:

  ```sh
  sudo apt install xmlstarlet
  ```

2. Use _roslaunch_ with the **multi_uav_mavros_sitl_sdf.launch** launch file:

  ````sh
  roslaunch multi_uav_mavros_sitl_sdf.launch vehicle:=<model_file_name>
  ```

  ::: info
  Note that the vehicle model file name argument is optional (`vehicle:=<model_file_name>`); if omitted the [plane model](https://github.com/PX4/PX4-SITL_gazebo-classic/tree/master/models/plane) will be used by default.

:::
  ````

This method is similar to using the xacro except that the SITL/Gazebo Classic port number is automatically inserted by _xmstarlet_ for each spawned vehicle, and does not need to be specified in the SDF file.

To add a new vehicle, you need to make sure the model can be found (in order to spawn it in Gazebo Classic), and PX4 needs to have an appropriate corresponding startup script.

1. You can choose to do either of:

  - modify the **single_vehicle_spawn_sdf.launch** file to point to the location of your model by changing the line below to point to your model:

    ```sh
    $(find px4)/Tools/simulation/gazebo/sitl_gazebo-classic/models/$(arg vehicle)/$(arg vehicle).sdf
    ```

    ::: info
    Ensure you set the `vehicle` argument even if you hardcode the path to your model.

:::

  - copy your model into the folder indicated above (following the same path convention).

2. The `vehicle` argument is used to set the `PX4_SIM_MODEL` environment variable, which is used by the default rcS (startup script) to find the corresponding startup settings file for the model.
  Within PX4 these startup files can be found in the **PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/** directory.
  For example, here is the plane model's [startup script](https://github.com/PX4/PX4-Autopilot/blob/main/ROMFS/px4fmu_common/init.d-posix/airframes/1030_gazebo-classic_plane).
  For this to work, the PX4 node in the launch file is passed arguments that specify the _rcS_ file (**etc/init.d/rcS**) and the location of the rootfs etc directory (`$(find px4)/build_px4_sitl_default/etc`).
  For simplicity, it is suggested that the startup file for the model be placed alongside PX4's in **PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/**.

## Additional Resources

- See [Simulation](../simulation/index.md) for a description of the UDP port configuration.
- See [URDF in Gazebo](http://wiki.ros.org/urdf/Tutorials/Using%20a%20URDF%20in%20Gazebo) for more information about spawning the model with xacro.
- See [RotorS](https://github.com/ethz-asl/rotors_simulator/tree/master/rotors_description/urdf) for more xacro models.
