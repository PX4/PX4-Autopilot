# Integration Testing for the PX4 ROS 2 Interface Library

This topic outlines the integration tests for the [PX4 ROS 2 Interface Library](../ros2/px4_ros2_interface_lib.md).

这些测试用于验证模式注册、故障保护（failsafes）和模式替换功能是否按预期工作。

## CI Testing

向 PX4 提交拉取请求（pull request）时，持续集成（CI）会运行该库的集成测试

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
