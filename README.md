# ECL

**Very lightweight Estimation & Control Library.**

[![DOI](https://zenodo.org/badge/22634/PX4/ecl.svg)](https://zenodo.org/badge/latestdoi/22634/PX4/ecl) [![Build Status](http://ci.px4.io:8080/buildStatus/icon?job=PX4/ecl/master)](http://ci.px4.io:8080/blue/organizations/jenkins/PX4%2Fecl/activity)

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
