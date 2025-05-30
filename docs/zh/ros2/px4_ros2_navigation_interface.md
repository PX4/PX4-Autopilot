# PX4 ROS 2 Navigation Interface

<Badge type="tip" text="PX4 v1.15" /> <Badge type="warning" text="Experimental" />

:::warning
Experimental
At the time of writing, parts of the PX4 ROS 2 Interface Library are experimental, and hence subject to change.
:::

The [PX4 ROS 2 Interface Library](../ros2/px4_ros2_interface_lib.md) navigation interface enables developers to send their position measurements to PX4 directly from ROS 2 applications, such as a VIO system or a map matching system.
The interface provides a layer of abstraction from PX4 and the uORB messaging framework, and introduces a few sanity checks on the requested state estimation updates sent via the interface.
These measurements are then fused into the EKF just as though they were internal PX4 measurements.

The library provides two classes, [`LocalPositionMeasurementInterface`](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1LocalPositionMeasurementInterface.html) and [`GlobalPositionMeasurementInterface`](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1GlobalPositionMeasurementInterface.html), which both expose a similar `update` method to provide either a local position or global position update to PX4, respectively.
The `update` method expects a position measurement `struct` ([`LocalPositionMeasurement`](https://auterion.github.io/px4-ros2-interface-lib/structpx4__ros2_1_1LocalPositionMeasurement.html) or [`GlobalPositionMeasurement`](https://auterion.github.io/px4-ros2-interface-lib/structpx4__ros2_1_1GlobalPositionMeasurement.html)) which developers can populate with their own generated position measurements.

## Installation and First Test

The following steps are required to get started:

1. Make sure you have a working [ROS 2 setup](../ros2/user_guide.md), with [`px4_msgs`](https://github.com/PX4/px4_msgs) in the ROS 2 workspace.

2. Clone the repository into the workspace:

  ```sh
  cd $ros_workspace/src
  git clone --recursive https://github.com/Auterion/px4-ros2-interface-lib
  ```

  ::: info
  To ensure compatibility, use the latest _main_ branches for PX4, _px4_msgs_ and the library.
  See also [here](https://github.com/Auterion/px4-ros2-interface-lib#compatibility-with-px4).

:::

3. Build the workspace:

  ```sh
  cd ..
  colcon build
  ```

4. In a different shell, start PX4 SITL:

  ```sh
  cd $px4-autopilot
  make px4_sitl gazebo-classic
  ```

  (here we use Gazebo-Classic, but you can use any model or simulator)

5. In yet a different shell, run the micro XRCE agent (you can keep it running afterward):

  ```sh
  MicroXRCEAgent udp4 -p 8888
  ```

6. Back in the ROS 2 terminal, source the workspace you just built (in step 3) and run the [global_navigation](https://github.com/Auterion/px4-ros2-interface-lib/tree/main/examples/cpp/navigation/global_navigation) example, which periodically sends dummy global position updates:

  ```sh
  source install/setup.bash
  ros2 run example_global_navigation_cpp example_global_navigation
  ```

  You should get an output like this showing that the global interface is successfully sending position updates:

  ```sh
  [INFO] [1702030701.836897756] [example_global_navigation_node]: example_global_navigation_node running!
  [DEBUG] [1702030702.837279784] [example_global_navigation_node]: Successfully sent position update to navigation interface.
  [DEBUG] [1702030703.837223884] [example_global_navigation_node]: Successfully sent position update to navigation interface.
  ```

7. In the PX4 shell, you can check that PX4 receives global position updates:

  ```sh
  listener aux_global_position
  ```

  The output should look like:

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

8. Now you are ready to use the navigation interface to send your own position updates.

## How to use the Library

To send a position measurement, you populate the position struct with the values you have measured.
Then call the interface's update function with that struct as the argument.

For a basic example of how to use this interface, check out the [examples](https://github.com/Auterion/px4-ros2-interface-lib/tree/main/examples/cpp/navigation) in the `Auterion/px4-ros2-interface-lib` repository, such as [examples/cpp/navigation/local_navigation](https://github.com/Auterion/px4-ros2-interface-lib/blob/main/examples/cpp/navigation/local_navigation/include/local_navigation.hpp) or [examples/cpp/navigation/global_navigation](https://github.com/Auterion/px4-ros2-interface-lib/blob/main/examples/cpp/navigation/local_navigation/include/global_navigation.hpp).

### Local Position Updates

First ensure that the PX4 parameter [`EKF2_EV_CTRL`](../advanced_config/parameter_reference.md#EKF2_EV_CTRL) is properly configured to fuse external local measurements, by setting the appropriate bits to `true`:

- `0`: Horizontal position data
- `1`: Vertical position data
- `2`: Velocity data
- `3`: Yaw data

To send a local position measurement to PX4:

1. Create a [`LocalPositionMeasurementInterface`](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1LocalPositionMeasurementInterface.html) instance by providing it with: a ROS node, and the pose and velocity reference frames of your measurements.
2. Populate a [`LocalPositionMeasurement`](https://auterion.github.io/px4-ros2-interface-lib/structpx4__ros2_1_1LocalPositionMeasurement.html) `struct` with your measurements.
3. Pass the `struct` to the `LocalPositionMeasurementInterface` [`update()`](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1LocalPositionMeasurementInterface.html#a6fd180b944710716d418b2cfe1c0c8e3) method.

The available pose and velocity reference frames for your measurements are defined by the following `enum`:

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

The `LocalPositionMeasurement` struct is defined as follows:

```cpp
struct LocalPositionMeasurement
{
   rclcpp::Time timestamp_sample {};

   std::optional<Eigen::Vector2f> position_xy {std::nullopt};
   std::optional<Eigen::Vector2f> position_xy_variance {std::nullopt};
   std::optional<float> position_z {std::nullopt};
   std::optional<float> position_z_variance {std::nullopt};

   std::optional<Eigen::Vector2f> velocity_xy {std::nullopt};
   std::optional<Eigen::Vector2f> velocity_xy_variance {std::nullopt};
   std::optional<float> velocity_z {std::nullopt};
   std::optional<float> velocity_z_variance {std::nullopt};

   std::optional<Eigen::Quaternionf> attitude_quaternion {std::nullopt};
   std::optional<Eigen::Vector3f> attitude_variance {std::nullopt};
};
```

The `update()` method of the local interface expects the following conditions to hold for `LocalPositionMeasurement`:

- The sample timestamp is defined.
- Values do not have a \`NAN\`\`.
- If a measurement value is provided, its associated variance value is well defined (e.g. if `position_xy` is defined, then `position_xy_variance` must be defined).
- If a measurement value is provided, its associated reference frame is not unknown (e.g. if `position_xy` is defined, then the interface was initialised with a pose frame different from `PoseFrame::Unknown`).

The following code snippet is an example of a ROS 2 node which uses the local navigation interface to send 3D pose updates in the North-East-Down (NED) reference frame to PX4:

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
         std::make_shared<px4_ros2::LocalPositionMeasurementInterface>(*this, pose_frame, velocity_frame);
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
   std::shared_ptr<px4_ros2::LocalPositionMeasurementInterface> _local_position_measurement_interface;
};
```

### Global Position Updates

First ensure that the PX4 parameter [`EKF2_AGP_CTRL`](../advanced_config/parameter_reference.md#EKF2_AGP_CTRL) is properly configured to fuse external global measurements, by setting the appropriate bits to `true`:

- `0`: Horizontal position data
- `1`: Vertical position data

To send a global position measurement to PX4:

1. Create a [`GlobalPositionMeasurementInterface`](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1GlobalPositionMeasurementInterface.html) instance by providing it with a ROS node.
2. Populate a [`GlobalPositionMeasurement`](https://auterion.github.io/px4-ros2-interface-lib/structpx4__ros2_1_1GlobalPositionMeasurement.html) `struct` with your measurements.
3. Pass the struct to the `GlobalPositionMeasurementInterface` [update()](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1GlobalPositionMeasurementInterface.html#a1a183b595ef7f6a22f3a83ba543fe86d) method.

The `GlobalPositionMeasurement` struct is defined as follows:

```cpp
struct GlobalPositionMeasurement
{
   rclcpp::Time timestamp_sample {};

   std::optional<Eigen::Vector2d> lat_lon {std::nullopt};
   std::optional<float> horizontal_variance {std::nullopt};

   std::optional<float> altitude_msl {std::nullopt};
   std::optional<float> vertical_variance {std::nullopt};
};
```

The `update()` method of the global interface expects the following conditions to hold for `GlobalPositionMeasurement`:

- The sample `timestamp_sample` is defined.
- Values do not have a NAN.
- If a measurement value is provided, its associated variance value is well defined (e.g. if `lat_lon` is defined, then `horizontal_variance` must be defined).

The following code snippet is an example of a ROS 2 node which uses the global navigation interface to send a measurement with latitude, longitude and altitude to PX4:

```cpp
class MyGlobalMeasurementUpdateNode : public rclcpp::Node
{
public:
   MyGlobalMeasurementUpdateNode()
   : Node("my_node_name")
   {
      // Initialize global interface [1]
      _global_position_measurement_interface =
         std::make_shared<px4_ros2::GlobalPositionMeasurementInterface>(*this);
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
   std::shared_ptr<px4_ros2::GlobalPositionMeasurementInterface> _global_position_measurement_interface;
};
```

## Multiple Instances of an Interface

Using multiple instances of the same interface (e.g. local and local) to send estimation updates will stream all update messages to the same topic and result in cross-talk.
This should not affect measurement fusion into the EKF, but different measurement sources will become indistinguishable.
