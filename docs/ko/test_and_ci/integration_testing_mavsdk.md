# MAVSDK 통합 테스트

PX4 can be tested end to end using integration tests based on [MAVSDK](https://mavsdk.mavlink.io).

The tests are written in Python using the [MAVSDK Python package](https://pypi.org/project/mavsdk/) and [pytest](https://pytest.org).
They are primarily developed against SITL and run in continuous integration (CI).
앞으로, 모든 플랫폼/하드웨어으로 일반화할 계획입니다.

The instructions below explain how to set up and run the tests locally, and how to write new ones.

:::info
This is the recommended integration test framework for PX4.
:::

## 준비 사항

### 개발 환경 설정

아직 하지 않은 경우:

- Install the development toolchain for [Linux](../dev_setup/dev_env_linux_ubuntu.md) or [macOS](../dev_setup/dev_env_mac.md) (Windows not supported).
- [Get the PX4 source code](../dev_setup/building_px4.md#download-the-px4-source-code):

  ```sh
  git clone https://github.com/PX4/PX4-Autopilot.git --recursive
  cd PX4-Autopilot
  ```

### 테스트용 PX4 빌드

The tests run in the [SIH](../sim_sih/index.md) simulator, which is built into PX4.
To build PX4 for SITL, use:

```sh
make px4_sitl_default
```

### Install the Test Dependencies

The tests need the `mavsdk` and `pytest` Python packages, in the versions given in `Tools/setup/test-requirements.txt`.
They are installed by the [setup scripts](../dev_setup/dev_env.md), or manually with:

```sh
pip install -r Tools/setup/test-requirements.txt
```

The `mavsdk` package is a native binding of the MAVSDK C++ library, so no separate MAVSDK installation is needed.

## 모든 PX4 테스트 실행

To run all SITL tests using the SIH simulator as defined in [sih-sitl.json](https://github.com/PX4/PX4-Autopilot/blob/main/test/mavsdk_tests/configs/sih-sitl.json), do:

```sh
test/mavsdk_tests/mavsdk_test_runner.py test/mavsdk_tests/configs/sih-sitl.json --speed-factor 10
```

This will list all the tests and then run them sequentially.
Each test case is run against a freshly started PX4 instance.

To see all possible command line arguments use the `-h` argument:

```sh
test/mavsdk_tests/mavsdk_test_runner.py -h

usage: mavsdk_test_runner.py [-h] [--log-dir LOG_DIR] [--speed-factor SPEED_FACTOR] [--iterations ITERATIONS] [--abort-early]
                             [--gui] [--model MODEL] [--case CASE] [--debugger DEBUGGER] [--upload] [--force-color]
                             [--verbose] [--build-dir BUILD_DIR]

positional arguments:
  config_file           JSON config file to use

options:
  -h, --help            show this help message and exit
  --log-dir LOG_DIR     Directory for log files
  --speed-factor SPEED_FACTOR
                        how fast to run the simulation
  --iterations ITERATIONS
                        how often to run all tests
  --abort-early         abort on first unsuccessful test
  --gui                 display the visualization for a simulation
  --model MODEL         only run tests for one model
  --case CASE           only run tests for one case (or multiple cases with wildcard '*')
  --debugger DEBUGGER   choice from valgrind, callgrind, gdb, lldb
  --upload              Upload logs to logs.px4.io
  --force-color         Force colorized output
  --verbose             enable more verbose output
  --build-dir BUILD_DIR
                        relative path where the built files are stored
```

## 단일 테스트 실행

Run a single test by specifying the `model` and test `case` as command line options.
A test case is a pytest test ID, and wildcards are supported.
For example, to test a multicopter mission using the SIH simulator:

```sh
test/mavsdk_tests/mavsdk_test_runner.py test/mavsdk_tests/configs/sih-sitl.json --speed-factor 10 --model quadx --case 'test_multicopter_mission.py::test_square_mission_with_return'
```

Or to run all the failsafe tests:

```sh
test/mavsdk_tests/mavsdk_test_runner.py test/mavsdk_tests/configs/sih-sitl.json --speed-factor 10 --model quadx --case 'test_multicopter_failsafe.py::*'
```

To list all test cases, use pytest from the test directory:

```sh
cd test/mavsdk_tests
pytest --collect-only -q
```

## Run a Test Against a Running PX4

While developing a test, it can be quicker to start PX4 yourself and run the test directly with pytest.
Start PX4 with SIH in one terminal:

```sh
make px4_sitl sihsim_quadx
```

And run a test in another terminal:

```sh
cd test/mavsdk_tests
pytest -v -s 'test_multicopter_mission.py::test_takeoff_and_land'
```

The `-s` option shows the test output while it runs.
Most tests expect a vehicle that is freshly started and on the ground, so restart PX4 between tests.

## Write a Test

Tests are plain pytest functions in `test/mavsdk_tests/test_*.py`.
A test gets a tester that is connected to the vehicle through a fixture, and is selected for vehicles by markers:

```py
import pytest

from autopilot_tester import AutopilotTester, MissionOptions


@pytest.mark.multicopter
def test_square_mission_with_return(tester: AutopilotTester) -> None:
    tester.wait_until_ready()
    tester.prepare_square_mission(MissionOptions(rtl_at_end=True))
    tester.arm()
    tester.execute_mission()
    tester.wait_until_disarmed(180)
```

The testers wrap MAVSDK with the building blocks the tests need, such as missions, offboard control, failure injection and trajectory checks:

- `tester`: [autopilot_tester.py](https://github.com/PX4/PX4-Autopilot/blob/main/test/mavsdk_tests/autopilot_tester.py) with the common functionality.
- `tester_rtl`: [autopilot_tester_rtl.py](https://github.com/PX4/PX4-Autopilot/blob/main/test/mavsdk_tests/autopilot_tester_rtl.py) adds rally points and geofences.
- `tester_loiter`: [autopilot_tester_loiter.py](https://github.com/PX4/PX4-Autopilot/blob/main/test/mavsdk_tests/autopilot_tester_loiter.py) adds fixed-wing loiter, reposition and figure eight.
- `tester_follow_me`: [autopilot_tester_follow_me.py](https://github.com/PX4/PX4-Autopilot/blob/main/test/mavsdk_tests/autopilot_tester_follow_me.py) adds follow-me.
- `tester_vision_target`: [autopilot_tester_vision_target.py](https://github.com/PX4/PX4-Autopilot/blob/main/test/mavsdk_tests/autopilot_tester_vision_target.py) simulates landing target sensors for precision landing.

Things to keep in mind:

- Timeouts and sleeps of the testers are in simulation time, so a test behaves the same at any speed factor.
- A failing `assert` in a MAVSDK callback is not reported by MAVSDK.
  Use `check_in_callback()` in callbacks instead, which fails the test once it finishes.
- MAVLink messages that MAVSDK has no plugin for can be sent and received with MAVSDK's `MavlinkDirect` plugin, see for example `send_command_int()`.
- Markers have to be listed in `pytest.ini`.
  The `test_filter` of each entry in the config files is a pytest marker expression, such as `multicopter or offboard`.

The tests are checked in CI with `mypy --strict` and `flake8`.

## 구현 참고 사항

- The tests are invoked from the test runner script [mavsdk_test_runner.py](https://github.com/PX4/PX4-Autopilot/blob/main/test/mavsdk_tests/mavsdk_test_runner.py).

  For each test case, the runner starts `px4` (and Gazebo Classic if configured) with a fresh root filesystem, runs the test with pytest, and collects the logs of these processes.

- The config files in `test/mavsdk_tests/configs/` define which models are tested with which tests, and per model settings such as the timeout, environment variables or the Gazebo world.

사용된 용어:

- "model": This is the selected simulation model, e.g. `quadx`.
- "test case": This is a pytest test, identified by its test ID, e.g. `test_multicopter_mission.py::test_takeoff_and_land`.
