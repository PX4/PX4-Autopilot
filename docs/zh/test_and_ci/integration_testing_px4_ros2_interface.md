# Integration Testing for the PX4 ROS 2 Interface Library

This topic outlines the integration tests for the [PX4 ROS 2 Interface Library](../ros2/px4_ros2_interface_lib.md).

These test that mode registration, failsafes, and mode replacement, work as expected.

## CI Testing

When opening a pull request to PX4, CI runs the library integration tests.

## Running Tests Locally

The tests can also be run locally from PX4:

```sh
./test/ros_test_runner.py
```

And to run only a single case:

```sh
./test/ros_test_runner.py --verbose --case <case>
```

You can list the available test cases with:

```sh
./test/ros_test_runner.py --list-cases
```
