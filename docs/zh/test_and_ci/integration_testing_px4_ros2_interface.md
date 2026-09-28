# Integration Testing for the PX4 ROS 2 Interface Library

This topic outlines the integration tests for the [PX4 ROS 2 Interface Library](../ros2/px4_ros2_interface_lib.md).

这些测试用于验证模式注册、故障保护（failsafes）和模式替换功能是否按预期工作。

## CI Testing

向 PX4 提交拉取请求（pull request）时，持续集成（CI）会运行该库的集成测试
It builds the pull request's PX4 firmware and a matching ROS workspace, then runs the tests using [SIH](../sim_sih/index.md).
SIH runs the physics simulation inside PX4 and does not require Gazebo Classic or Gazebo Harmonic.

## Running Tests Locally

Follow [Testing A PX4 Checkout With ROS](../dev_setup/sitl_container_builds.md#testing-a-px4-checkout-with-ros) to build `px4_sitl_sih` and a matching ROS workspace.
Run the following commands from the PX4 repository root, inside the ROS development environment:

```bash
source /opt/px4_ros2_ci/install/setup.bash
./test/ros_test_runner.py
```

If you used another workspace location, source its `install/setup.bash` instead.
The runner defaults to `test/ros_tests/config-sih.json` and `build/px4_sitl_sih/`, matching CI.
The SIH configuration selects the `quadx` model for all three integration suites.
SIH does not support the runner's `--gui` option.

And to run only a single case:

```sh
./test/ros_test_runner.py --verbose --case <case>
```

You can list the available test cases with:

```sh
./test/ros_test_runner.py --list-cases
```

## Legacy Gazebo Classic Testing

For an existing Gazebo Classic setup, select both its configuration and firmware build directory explicitly:

```sh
./test/ros_test_runner.py \
  --config-file test/ros_tests/config.json \
  --build-dir build/px4_sitl_default/ \
  --model iris
```

This requires separately built Gazebo Classic plugins and matching ROS sources.
The Gazebo Harmonic containers do not provide Gazebo Classic.
