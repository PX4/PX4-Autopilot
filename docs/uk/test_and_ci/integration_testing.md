# Integration Testing

Integration tests are used to verify how well larger parts of a system work together.
In PX4 this generally means testing whole features of a vehicle, usually running in simulation.
The tests are run in [Continuous Integration (CI)](../test_and_ci/continous_integration.md) on every pull request.

- [MAVSDK Integration Testing](../test_and_ci/integration_testing_mavsdk.md) - MAVSDK-based test framework for PX4.
  _This is the recommended framework for writing new Integration tests_
- [PX4 ROS2 Interface Library Integration Testing](../test_and_ci/integration_testing_px4_ros2_interface.md) - Integration Tests for the [PX4 ROS 2 Interface Library](../ros2/px4_ros2_interface_lib.md).

The following framework should only be used for tests that require ROS 1:

- [ROS 1 Integration Testing](../test_and_ci/integration_testing_ros1_mavros.md) (Deprecated)
