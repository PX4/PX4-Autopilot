# PX4 ROS2 导航接口

<Badge type="tip" text="PX4 v1.15" /> <Badge type="warning" text="Experimental" />

:::warning
Experimental
在撰写本文时，PX4 ROS 2 接口库的部分内容仍处于试验阶段，因此可能会发生变动。
:::

[PX4 ROS 2 Interface Library](../ros2/px4_ros2_interface_lib.md) 中的导航接口，支持开发者直接从 ROS 2 应用（如视觉惯性里程计系统或地图匹配系统）向 PX4 发送位置测量数据。
该接口提供了对 PX4 和 uORB 消息框架的抽象层，并对通过该接口发送的请求状态估计更新引入了一些合理性检查。
这些测量数据随后会被融合到扩展EKF中，其处理方式与 PX4 内部生成的测量数据完全一致。

库提供两个类，[`LocalPositionMeasurementInterface`](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1LocalPositionMeasurementInterface.html) 和 [`GlobalPositionMeasureInterface`](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1GlobalPositionMeasurementInterface.html) 它都会暴露出一个类似的 "update" 方法来提供一个本地位置或全球位置更新到 PX4。
`update`方法需要一个位置测量`struct`(`LocalPositionMeasure`](https://auterion.github.io/px4-ros2-interface-lib/structpx4__ros2_1_1LocalPositionMeasurement.html)或[\`GlobalPositionMeasure\`](https://auterion.github.io/px4-ros2-interface-lib/structpx4__ros2_1_1GlobalPositionMeasurement.html)]，开发者可以在其中填入自己生成的位置测量数据。

## 安装与首次测试

开始使用前，需完成以下步骤：

1. 请确保您在 ROS 2 工作区中有 [ROS 2 设置](../ros2/user_guide.md) 与 [`px4_msgs`](https://github.com/PX4/px4_msgs]。

2. 将代码仓库克隆到工作空间中

   ```sh
   cd $ros_workspace/src
   git clone --recursive https://github.com/Auterion/px4-ros2-interface-lib
   ```

   提示信息
   为确保兼容性，请使用 PX4、px4_msgs（PX4 消息包）及该库的最新 main 分支。
   另请参阅 [here]（https://github.com/Auterion/px4-ros2-interface-lib#compatibility-with-px4）

:::

3. 构建工作空间:

   ```sh
   cd ..
   colcon版本
   ```

4. 在另一个外壳中，启动 PX4 SITL：

   ```sh
   cd $px4-autopilot
   make px4_sitl gazebo-classic
   ```

   (这里我们使用Gazebo-Classic，但你可以使用任何模型或模拟器)

5. 在另一个独立的终端中，运行 micro XRCE 代理（运行后可保持后台持续运行）：

   ```sh
   MicroXRCEAgent udp4 -p 8888
   ```

6. 返回ROS2终端，为您刚刚构建的工作空间（步骤3），运行 [global_navigation](https://github.com/Auterion/px4-ros2-interface-lib/tree/main/examples/cpp/navigation/global_navigation) 示例。 它会周期性地发送虚拟的全球位置更新（数据）：

   ```sh
   source install/setup.bash
   ros2 run example_global_navigation_cpp example_global_navigation
   ```

   你应会看到如下所示的输出，该输出表明全球位置接口正成功发送位置更新：

   ```sh
   [INFO] [1702030701.836897756] [example_global_navigation_node]: example_global_navigation_node running!
   [DEBUG] [1702030702.837279784] [example_global_navigation_node]: Successfully sent position update to navigation interface.
   [DEBUG] [1702030703.837223884] [example_global_navigation_node]: Successfully sent position update to navigation interface.
   ```

7. 在 PX4 终端（PX4 shell）中，你可以通过以下操作检查 PX4 是否接收到全球位置更新（数据）

   ```sh
   listener aux_global_position
   ```

   输出内容应如下所示：

   ```sh
   TOPIC: aux_global_position
   aux_global_position
      timestamp: 46916000 (0.528000 seconds ago)
      timestamp_sample: 46916000 (0 us before timestamp)
      lat: 12.343210
      lon: 23.454320
      alt: 12.40000
      alt_ellipsoid: 0.00000
      delta_alt: 0.00000
      eph: 0.31623
      epv: 0.44721
      terrain_alt: 0.00000
      lat_lon_reset_counter: 0
      alt_reset_counter: 0
      terrain_alt_valid: False
      dead_reckoning: False
   ```

8. 现在你已准备好使用该导航接口发送自己的位置更新数据了。

## 如何使用代码库

要发送位置测量数据，你需要用所测量的值填充位置结构体。
然后以此结构调用接口的更新功能作为参数。

关于如何使用此接口的基本示例，请在“Auterion/px4-ros2-interface-lib”仓库中查看 [examples](https://github.com/Auterion/px4-ros2-interface-lib/tree/main/examples/cpp/navigation) 例如[示例/cpp/navigation/local_navigation](https://github.com/Auterion/px4-ros2-interface-lib/blob/main/examples/cpp/navigation/local_navigation/include/local_navigation.hpp)或[示例/cpp/navigation/global_navigation](https://github.com/Auterion/px4-ros2-interface-lib/blob/main/examples/cpp/navigation/local_navigation/include/global_navigation.hpp)。

### 局部位置更新

首先确保正确配置 PX4 参数[`EKF2_EV_CTRL`](../advanced_config/parameter_reference.md#EKF2_EV_CTRL)通过将相应的位设置为true，可以正确配置（系统）以融合外部局部测量数据：

- `0`: 水平位置数据
- `1`：垂直位置数据
- `2`：速度数据
- `3`:偏航角数据

向PX4发送局部位置测量：

1. 通过提供一个 ROS节点，创建一个 [`localPositionMeasureInterface` ](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1LocalPositionMeasurementInterface.html) 实例：您测量的位置和速度参考框架。
2. 包含一个[`本地定位测量`](https://auterion.github.io/px4-ros2-interface-lib/structpx4__ros2_1_1LocalPositionMeasurement.html) `structt` ，包含你测量的数据。
3. 将 `struct` 传入 `LocalPositionMeasurementInterface` [`update()`](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1LocalPositionMeasurementInterface.html#a6fd180b944710716d418b2cfe1c0c8e3) 方法中。

你的测量数据可用的位置和速度参考坐标系由以下枚举（enum）定义：

```cpp
enum class PoseFrame
{
  Unknown,
  LocalNED,
  LocalFRD
};

enum class VelocityFrame
{
  Unknown,
  LocalNED,
  LocalFRD,
  BodyFRD
};
```

`LocalPositionMeasurement`结构定义如下：

```cpp
struct LocalPositionMeasurement
{
   rclcpp::Time timestamp_sample {};

   std::optional<0> position_xy {std::nullopt};
   std::optional<0> position_xy_variance {std::nullopt};
   std::optional<1> position_z {std::nullopt};
   std::optional<1> position_z_variance {std::nullopt};

   std::optional<0> velocity_xy {std::nullopt};
   std::optional<0> velocity_xy_variance {std::nullopt};
   std::optional<1> velocity_z {std::nullopt};
   std::optional<1> velocity_z_variance {std::nullopt};

   std::optional<2> attitude_quaternion {std::nullopt};
   std::optional<3> attitude_variance {std::nullopt};
};
```

局部接口的update()方法要求LocalPositionMeasurement（局部位置测量结构体）满足以下条件：

- 示例时间戳已定义。
- 数值不得包含 `NAN` 。
- 如果提供了某个测量值，则其关联的方差值必须明确定义（例如，若position_xy已定义，则position_xy_variance也必须定义）。
- 如果提供了某个测量值，那么其关联的参考坐标系不得为 “未知”（例如，若position_xy已定义，则接口必须以不同于PoseFrame::Unknown的位置坐标系进行初始化）。

以下是一个 ROS 2 节点的示例代码片段，该节点使用局部导航接口向 PX4 发送东北天（NED）参考坐标系下的 3D 位姿更新：

```cpp
class MyLocalMeasurementUpdateNode : public rclcpp::Node
{
public:
   MyLocalMeasurementUpdateNode()
   : Node("my_node_name")
   {
      // Set pose measurement reference frame to north-east-down
      const px4_ros2::PoseFrame pose_frame = px4_ros2::PoseFrame::LocalNED;
      // We will only send pose measurements in this example
      // Set velocity measurement reference frame to unknown
      const px4_ros2::VelocityFrame velocity_frame = px4_ros2::VelocityFrame::Unknown;
      // Initialize local interface [1]
      _local_position_measurement_interface =
         std::make_shared<Eigen::Vector2f>(*this, pose_frame, velocity_frame);
   }

   void sendUpdate()
   {
      while (running) { // Potentially make method run as a callback or on a timer
         // Generate local position measurement
         rclcpp::Time timestamp_sample  = ...
         Eigen::Vector2f position_xy = ...
         Eigen::Vector2f position_xy_variance = ...
         float position_z = ...
         float position_z_variance = ...

         // Populate the local position measurement struct [2]
         px4_ros2::LocalPositionMeasurement local_position_measurement{};
         local_position_measurement.timestamp_sample = timestamp_sample;
         local_position_measurement.position_xy = position_xy;
         local_position_measurement.position_xy_variance = position_xy_variance;
         local_position_measurement.position_z = position_z;
         local_position_measurement.position_z_variance = position_z_variance;

         // Send measurement to PX4 using the interface [3]
         try {
            _local_position_measurement_interface->update(local_position_measurement);
         } catch (const px4_ros2::NavigationInterfaceInvalidArgument & e) {
            // Handle exceptions caused by invalid local_position_measurement definition
            RCLCPP_ERROR(get_logger(), "Exception caught: %s", e.what());
         }
      }
   }

private:
   std::shared_ptr<Eigen::Vector2f> _local_position_measurement_interface;
};
```

### 全局位置更新

首先确保正确配置 PX4 参数[`EKF2_EV_CTRL`](../advanced_config/parameter_reference.md#EKF2_AGP_CTRL)通过将相应的位设置为true，可以正确配置（系统）以融合外部全部测量数据：

- `0`: 水平位置数据
- `1`：垂直位置数据

向PX4发送全局位置测量：

1. 创建一个 [`GlobalPositionMeasureInterface`](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1GlobalPositionMeasurementInterface.html) 实例，并提供一个 ROS节点。
2. 包含一个[`GlobalPositionMeasurement`](https://auterion.github.io/px4-ros2-interface-lib/structpx4__ros2_1_1GlobalPositionMeasurement.html) `struct` ，包含你测量的数据。
3. 将结构移至`GlobalPositionMeasurementInterface` [update()](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1GlobalPositionMeasurementInterface.html#a1a183b595ef7f6a22f3a83ba543fe86d) 方法中。

`GlobalPositionMeasurement`结构定义如下：

```cpp
struct GlobalPositionMeasurement
{
   rclcpp::Time timestamp_sample {};

   std::optional<px4_ros2::LocalPositionMeasurementInterface> lat_lon {std::nullopt};
   std::optional<1> horizontal_variance {std::nullopt};

   std::optional<1> altitude_msl {std::nullopt};
   std::optional<1> vertical_variance {std::nullopt};
```

全局接口的 `update()` 方法预计在 `GlobalPositionMeasurement` 中保留以下条件：

- 样本`timestamp_sample`已定义。
- 数据不得包含NAN。
- 如果提供了某个测量值，那么其关联的方差值必须明确定义（例如，若lat_lon（经纬度）已定义，则horizontal_variance（水平方差）也必须定义）。

下面的代码片段是一个 ROS 2 节点的示例，该节点使用全局导航接口来发送一个测量纬度。 经度和高度到 PX4：

```cpp
class MyGlobalMeasurementUpdateNode : public rclcpp::Node
{
public:
   MyGlobalMeasurementUpdateNode()
   : Node("my_node_name")
   {
      // Initialize global interface [1]
      _global_position_measurement_interface =
         std::make_shared<Eigen::Vector2d>(*this);
   }

   void sendUpdate()
   {
      while (running) { // Potentially make method run as a callback or on a timer
         // Generate global position measurement
         rclcpp::Time timestamp_sample  = ...
         Eigen::Vector2d lat_lon = ...
         float horizontal_variance = ...
         float altitude_msl = ...
         float vertical_variance = ...

         // Populate the global position measurement struct [2]
         px4_ros2::GlobalPositionMeasurement global_position_measurement{};
         global_position_measurement.timestamp_sample = timestamp_sample;
         global_position_measurement.lat_lon = lat_lon;
         global_position_measurement.horizontal_variance = horizontal_variance;
         global_position_measurement.altitude_msl = altitude_msl;
         global_position_measurement.vertical_variance = vertical_variance;

         // Send measurement to PX4 using the interface [3]
         try {
            _global_position_measurement_interface->update(global_position_measurement);
         } catch (const px4_ros2::NavigationInterfaceInvalidArgument & e) {
            // Handle exceptions caused by invalid global_position_measurement definition
            RCLCPP_ERROR(get_logger(), "Exception caught: %s", e.what());
         }
      }
   }

private:
   std::shared_ptr<0> _global_position_measurement_interface;
};
```

## 接口的多个实例

使用同一接口的多个实例（例如，多个局部接口实例）发送估计更新时，会将所有更新消息发送到同一个主题，从而导致串扰。
这不应影响计量并入EKF，但不同的计量来源将无法区分。
