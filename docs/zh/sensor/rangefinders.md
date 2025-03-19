# 距离传感器（测距仪）

Distance sensors provide distance measurement that can be used for [terrain following](../flying/terrain_following_holding.md#terrain_following), [terrain holding](../flying/terrain_following_holding.md#terrain_hold) (i.e. precision hovering for photography), improved landing behaviour ([conditional range aid](../advanced_config/tuning_the_ecl_ekf.md#conditional-range-aiding)), warning of regulatory height limits, collision prevention, etc.

This section lists the distance sensors supported by PX4 (linked to more detailed documentation), and provides information about the [generic configuration](#configuration) required for all rangefinders, [testing](#testing), and simulation with [Gazebo](#gazebo-simulation) or [Gazebo-Classic](#gazebo-classic-simulation).
更详细的设置和配置信息在下方（和侧边栏）的主题链接中提供

<img src="../../assets/hardware/sensors/lidar_lite/lidar_lite_v3.jpg" alt="Lidar Lite V3" width="200px" /><img src="../../assets/hardware/sensors/lidar_lightware/sf11c_120_m.jpg" alt="LightWare SF11/C Lidar" width="200px" /><img src="../../assets/hardware/sensors/optical_flow/ark_flow_distance_sensor.jpg" alt="ARK Flow" width="200px">

## 支持的测距仪

:::tip
This is a subset of the rangefinders that can be used with PX4.
[Modules: Distance Sensors](../modules/modules_driver_distance_sensor.md) lists PX4 drivers for other non-CAN rangefinders.
There may also be other DroneCAN rangefinders than those listed here.
:::

### ARK Flow & AKR Flow MR

[ARK Flow](../dronecan/ark_flow.md) and [ARK Flow](../dronecan/ark_flow_mr.md) are open-source Time-of-Flight (ToF) and optical flow sensor modules, which are capable of measuring distances from 8cm to 30m and from 8cm to 50m, respectively.
它可以通过CAN1接口连接至飞控，允许通过CAN2接口添加传感器。
It supports [DroneCAN](../dronecan/index.md), runs [PX4 DroneCAN Firmware](../dronecan/px4_cannode_fw.md), and is packed into a tiny form factor.

### Holybro ST VL53L1X 激光雷达

The [VL53L1X](https://holybro.com/products/st-vl53l1x-lidar) is a state-of-the-art, Time-of-Flight (ToF), laser-ranging sensor, enhancing the ST FlightSense™ product family.
它是市场上速度最快的微型 ToF 传感器，精确测距可达 4 m，快速测距频率可达 50 Hz。

It comes with a JST GHR 4 pin connector that is compatible with the I2C port on [Pixhawk 4](../flight_controller/pixhawk4.md), [Pixhawk 5X](../flight_controller/pixhawk5x.md), and other flight controllers that follow the [Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf).

### Lidar-Lite

[Lidar-Lite](../sensor/lidar_lite.md) is a compact, high-performance optical distant measurement rangefinder.
它的传感器范围为 (5cm - 40m)，可以连接到 PWM 或 I2C 端口。

### MaxBotix I2CXL-MaxSonar-EZ

The MaxBotix [I2CXL-MaxSonar-EZ](https://www.maxbotix.com/product-category/i2cxl-maxsonar-ez-products) range has a number of relatively short-ranged sonar based rangefinders that are suitable for assisted takeoff/landing and collision avoidance.
这些可以使用 I2C 端口连接。

The rangefinders are enabled using the parameter [SENS_EN_MB12XX](../advanced_config/parameter_reference.md#SENS_EN_MB12XX).

### Lightware LIDARs

[Lightware SFxx Lidar](../sensor/sfxx_lidar.md) provide a broad range of lightweight “laser altimeters” that are suitable for many drone applications.

PX4 支持: SF11/c 和 SF/LW20.
PX4 也可用于一下停产的型号: SF02, SF10/a, SF10/b, SF10/c.

Others may be supported via the [RaccoonLab Cyphal and DroneCAN Rangefinder Adapter](#raccoonlab-cyphal-and-dronecan-rangefinder-adapter) described below.

PX4 also supports the [LightWare LiDAR SF45 Rotating Lidar](../sensor/sf45_rotating_lidar.md) for [collision prevention](../computer_vision/collision_prevention.md) applications.

### TeraRanger 测距仪

[TeraRanger](../sensor/teraranger.md) provide a number of lightweight distance measurement sensors based on infrared Time-of-Flight (ToF) technology.
他们通常比声纳更快、范围更大、比基于激光的系统更小、更轻。

PX4 提供以下通过 I2C总线连接的型号：TeraRanger One, TeraRanger Evo 60m 和 TeraRanger Evo 600Hz。

### Ainstein US-D1 标准雷达高度计

The _Ainstein_ [US-D1 Standard Radar Altimeter](../sensor/ulanding_radar.md) is compact microwave rangefinder that has been optimised for use on UAVs.
它有大约50米的感测范围。
该产品的一个特别优势是它可以在所有天气条件下和所有地形类型（包括水）上有效运行。

### LeddarOne

[LeddarOne](../sensor/leddar_one.md) is small Lidar module with a narrow, yet diffuse beam that offers excellent overall detection range and performance, in a robust, reliable, cost-effective package.
It has a sensing range from 1cm to 40m and needs to be connected to a UART/serial bus.

### TFmini

The [Benewake TFmini Lidar](../sensor/tfmini.md) is a tiny, low cost, and low power LIDAR with 12m range.

### PSK-CM8JL65-CC5

<Badge type="info" text="Discontinued" />

The [Lanbao PSK-CM8JL65-CC5 ToF Infrared Distance Measuring Sensor](../sensor/cm8jl65_ir_distance_sensor.md) is a very small (38 mm x 18mm x 7mm, <10g) IR distance sensor with a 0.17m-8m range and millimeter resolution.
It must be connected to a UART/serial bus.

### Avionics Anonymous UAVCAN Laser Altimeter Interface

The [Avionics Anonymous UAVCAN Laser Altimeter Interface](../dronecan/avanon_laser_interface.md) allows several common rangefinders (e.g. [Lightware SF11/c, SF30/D](../sensor/sfxx_lidar.md), etc) to be connected to the [CAN](../can/index.md) bus via [DroneCAN](../dronecan/index.md), a more robust interface than I2C.

### RaccoonLab Cyphal and DroneCAN Rangefinder Adapter

The [RaccoonLab Cyphal and DroneCAN Rangefinder Adapter](https://raccoonlab.co/tproduct/360882105-910084093051-cyphal-and-dronecan-rangefinder-adapter) allows several common rangefinders to be connected to the CAN bus via Cyphal or DroneCAN, providing a more robust interface than I2C or UART.
This adapter efficiently reads measurements via I2C or UART and publishes range data in meters, making it a versatile solution for UAVs, robotics, and technical documentation applications.

Supported rangefinders include:

- LightWare LW20/C
- TF-Luna
- Garmin Lite V3
- VL53L1CB

### RaccoonLab Cyphal and DroneCAN µRANGEFINDER

[RaccoonLab µRANGEFINDER](https://docs.raccoonlab.co/guide/rangefinder/uRANGEFINDER.html) is designed to measure distance and publish it via Cyphal/DroneCAN protocols.
It can be used to estimate precision landing or object avoidance.

Features:

- [VL53L1CBV0FY-1](https://www.st.com/resource/en/datasheet/vl53l1.pdf) sensor
- Input voltage sensor
- CAN connectors: 2 [UCANPHY Micro (JST-GH 4)](https://raccoonlabdev.github.io/docs/guide/wires/).

## Configuration/Setup {#configuration}

Rangefinders are usually connected to either a serial (PWM) or I2C port (depending on the device driver), and are enabled on the port by setting a particular parameter.

The hardware and software setup that is _specific to each distance sensor_ is covered in their individual topics.

The generic configuration that is _common to all distance sensors_, covering both the physical setup and usage, is given below.

### Generic Configuration

The common rangefinder configuration is specified using [EKF2_RNG\_\*](../advanced_config/parameter_reference.md#EKF2_RNG_CTRL) parameters.
These include (non exhaustively):

- [EKF2_RNG_POS_X](../advanced_config/parameter_reference.md#EKF2_RNG_POS_X), [EKF2_RNG_POS_Y](../advanced_config/parameter_reference.md#EKF2_RNG_POS_Y), [EKF2_RNG_POS_Z](../advanced_config/parameter_reference.md#EKF2_RNG_POS_Z) - offset of the rangefinder from the vehicle centre of gravity in X, Y, Z directions.
- [EKF2_RNG_PITCH](../advanced_config/parameter_reference.md#EKF2_RNG_PITCH) - A value of 0 degrees (default) corresponds to the range finder being exactly aligned with the vehicle vertical axis (i.e. straight down), while 90 degrees indicates that the range finder is pointing forward.
  Simple trigonometry is used to calculate the distance to ground if a non-zero pitch is used.
- [EKF2_RNG_DELAY](../advanced_config/parameter_reference.md#EKF2_RNG_DELAY) - approximate delay of data reaching the estimator from the sensor.
- [EKF2_RNG_SFE](../advanced_config/parameter_reference.md#EKF2_RNG_SFE) - Range finder range dependent noise scaler.
- [EKF2_RNG_NOISE](../advanced_config/parameter_reference.md#EKF2_RNG_NOISE) - Measurement noise for range finder fusion

## 测试

The easiest way to test the rangefinder is to vary the range and compare to the values detected by PX4.
The sections below show some approaches to getting the measured range.

### QGroundControl MAVLink Inspector

The _QGroundControl MAVLink Inspector_ lets you view messages sent from the vehicle, including `DISTANCE_SENSOR` information from the rangefinder.
The main difference between the tools is that the _Analyze_ tool can plot values in a graph.

:::info
The messages that are sent depend on the vehicle configuration.
You will only get `DISTANCE_SENSOR` messages if the connected vehicle has a rangefinder installed and is publishing sensor values.
:::

To view the rangefinder output:

1. Open the menu **Q > Select Tool > Analyze Tools**:

  ![Menu for QGC Analyze Tool](../../assets/qgc/analyze/menu_analyze_tool.png)

2. Select the message `DISTANCE_SENSOR`, and then check the plot checkbox against `current_distance`.
  The tool will then plot the result:
  ![QGC Analyze DISTANCE_SENSOR value](../../assets/qgc/analyze/qgc_analyze_tool_distance_sensor.png)

### QGroundControl MAVLink Console

You can also use the _QGroundControl MAVLink Console_ to observe the `distance_sensor` uORB topic:

```sh
listener distance_sensor 5
```

:::info
The _QGroundControl MAVLink Console_ works when connected to Pixhawk or other NuttX targets, but not the Simulator.
On the Simulator you can run the commands directly in the terminal.
:::

For more information see: [Development > Debugging/Logging > Sensor/Topic Debugging using the Listener Command](../debug/sensor_uorb_topic_debugging.md).

## 仿真

### Gazebo Simulation

Lidar and sonar rangefinders can be used in the [Gazebo](../sim_gazebo_gz/index.md) simulator.
To do this you must start the simulator using a vehicle model that includes the rangefinder.

Downward facing sensors that write to the [DistanceSensor](../msg_docs/DistanceSensor.md) UORB topic can be used to test use cases such as [landing](../flight_modes_mc/land.md) and [terrain following](../flying/terrain_following_holding.md):

- [Quadrotor(x500) with 1D LIDAR (Down-facing)](../sim_gazebo_gz/vehicles.md#x500-quadrotor-with-1d-lidar-down-facing)

  ```sh
  make px4_sitl gz_x500_lidar_down
  ```

Front-facing sensors that write to [ObstacleDistance](../msg_docs/ObstacleDistance.md) can be used to test [Collision Prevention](../computer_vision/collision_prevention.md#gazebo-simulation):

- [Quadrotor(x500) with 2D LIDAR](../sim_gazebo_gz/vehicles.md#x500-quadrotor-with-2d-lidar)

  ```sh
  make px4_sitl gz_x500_lidar_2d
  ```

- [Quadrotor(x500) with 1D LIDAR (Front-facing)](../sim_gazebo_gz/vehicles.md#x500-quadrotor-with-1d-lidar-front-facing)

  ```sh
  make px4_sitl gz_x500_lidar_front
  ```

### Gazebo-Classic Simulation

Lidar and sonar rangefinders can be used in the [Gazebo Classic](../sim_gazebo_classic/index.md) simulator.
To do this you must start the simulator using a vehicle model that includes the rangefinder.

The iris optical flow model includes a Lidar rangefinder:

```sh
make px4_sitl gazebo-classic_iris_opt_flow
```

The typhoon_h480 includes a sonar rangefinder:

```sh
make px4_sitl gazebo-classic_typhoon_h480
```

If you need to use a different vehicle you can include the model in its configuration file.
You can see how in the respective Iris and Typhoon configuration files:

- [iris_opt_flow.sdf](https://github.com/PX4/PX4-SITL_gazebo-classic/blob/main/models/iris_opt_flow/iris_opt_flow.sdf)

  ```xml
    <include>
      <uri>model://lidar</uri>
      <pose>-0.12 0 0 0 3.1415 0</pose>
    </include>
    <joint name="lidar_joint" type="revolute">
      <child>lidar::link</child>
      <parent>iris::base_link</parent>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <upper>0</upper>
          <lower>0</lower>
        </limit>
      </axis>
    </joint>
  ```

- [typhoon_h480.sdf](https://github.com/PX4/PX4-SITL_gazebo-classic/blob/main/models/typhoon_h480/typhoon_h480.sdf.jinja#L1131-L1145)

  ```xml
    <include>
      <uri>model://sonar</uri>
    </include>
    <joint name="sonar_joint" type="revolute">
      <child>sonar_model::link</child>
      <parent>typhoon_h480::base_link</parent>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <upper>0</upper>
          <lower>0</lower>
        </limit>
      </axis>
    </joint>
  ```
