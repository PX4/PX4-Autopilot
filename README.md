# ECL

**Very lightweight Estimation & Control Library.**

[![DOI](https://zenodo.org/badge/22634/PX4/PX4-ECL.svg)](https://zenodo.org/badge/latestdoi/22634/PX4/PX4-ECL)

This library solves the estimation & control problems of a number of robots and drones. It accepts GPS, vision and inertial sensor inputs. It is extremely lightweight and efficient and yet has the rugged field-proven performance.

The library is BSD 3-clause licensed.

## EKF Documentation

  * [EKF Documentation and Tuning Guide](https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html)

## Building EKF

### Prerequisites:

  * Matrix: A lightweight, BSD-licensed matrix math library: https://github.com/px4/matrix - it is automatically included as submodule.


By following the steps mentioned below you can create a static library which can be included in projects:

```
make
// OR
mkdir build/
cd build/
cmake ..
make
```

## Testing ECL
By following the steps you can run the unit tests

```
make test
```
### Change Indicator / Unit Tests
Change indication is the concept of running the EKF on different data-sets and compare the state of the EKF to a previous version. If a contributor makes a functional change that is run during the change_indication tests, this will produce a different output of the EKF's state. As the tests are run in CI, this checks if a contributor forgot to run the checks themselves and add the [new EKF's state outputs](https://github.com/PX4/ecl/blob/master/test/change_indication/iris_gps.csv) to the pull request.

The unit tests include a check to see if the pull request results in a difference to the [output data csv file](https://github.com/PX4/ecl/blob/master/test/change_indication/iris_gps.csv) when replaying the [sensor data csv file](https://github.com/PX4/ecl/blob/master/test/replay_data/iris_gps.csv). If a pull request results in an expected difference, then it is important that the output reference file be re-generated and included as part of the pull request. A non-functional pull request should not result in changes to this file, however the default test case does not exercise all sensor types so this test passing is a necessary, but not sufficient requirement for a non-functional pull request.

The functionality that supports this test consists of:
* Python scripts that extract sensor data from ulog files and writes them to a sensor data csv file. The default [sensor data csv file](https://github.com/PX4/ecl/blob/master/test/replay_data/iris_gps.csv) used by the unit test was generated from a ulog created from an iris SITL flight.
* A [script file](https://github.com/PX4/ecl/blob/master/test/test_EKF_withReplayData.cpp) using functionality provided by  the [sensor simulator](https://github.com/PX4/ecl/blob/master/test/sensor_simulator/sensor_simulator.cpp), that loads sensor data from the [sensor data csv file](https://github.com/PX4/ecl/blob/master/test/replay_data/iris_gps.csv) , replays the EKF with it and logs the EKF's state and covariance data to the [output data csv file](https://github.com/PX4/ecl/blob/master/test/replay_data/iris_gps.csv).
* CI action that checks if the logs of the test running with replay data is changing. This helps to see if there are functional changes.

#### How to run the Change Indicator test during development on your own logs:

* create sensor_data.csv file from ulog file 'cd test/sensor_simulator/
python3 createSensorDataFile.py <path/to/ulog> ../replay_data/<descriptive_name>.csv'
* Setup the test file to use the EKF with the created sensor data by copy&paste an existing test case in [test/test_EKF_withReplayData.cpp](https://github.com/PX4/ecl/blob/master/test/test_EKF_withReplayData.cpp) and adapt the paths to load the right sensor data and write it to the right place, eg
_sensor_simulator.loadSensorDataFromFile("../../../test/replay_data/<descriptive_name>.csv");
_ekf_logger.setFilePath("../../../test/change_indication/<descriptive_name>.csv");
* You can feed the EKF with the data in the csv file, by running '_sensor_simulator.runReplaySeconds(duration_in_seconds)'. Be aware that replay sensor data will only be available when the corresponding sensor simulation are running. By default only imu, baro and mag sensor simulators are running. You can start a sensor simulation by calling _sensor_simulator._<sensor>.start(). Be also aware that you still have to setup the EKF yourself. This includes setting the bit mask (fusion_mode in common.h) according to what you intend to fuse.
* In between _sensor_simulator.runReplaySeconds(duration_in_seconds) calls, write the state and covariances to the change_indication file by including a _ekf_logger.writeStateToFile(); line.
* Run the EKF with your data and all the other tests by running 'make test' from the ecl directory. The [default output data csv file](https://github.com/PX4/ecl/blob/master/test/change_indication/iris_gps.csv) changes can then be included in the PR if differences are causing the CI test to fail.

#### Known Issues
If compiler versions other than GCC 7.5 are used to generate the output data file, then is is possible that the file will cause CI failures due to small numerical differences to file generated by the CI test.
