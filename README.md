## PX4 Flight Core and PX4 Middleware ##

[![Build Status](https://travis-ci.org/PX4/Firmware.svg?branch=master)](https://travis-ci.org/PX4/Firmware) [![Coverity Scan](https://scan.coverity.com/projects/3966/badge.svg?flat=1)](https://scan.coverity.com/projects/3966?tab=overview)

[![Gitter](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/PX4/Firmware?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

This repository contains the PX4 Flight Core, with the main applications located in the src/modules directory. It also contains the PX4 Drone Platform, which contains drivers and middleware to run drones.

*   Official Website: http://px4.io
*   License: BSD 3-clause (see [LICENSE.md](https://github.com/PX4/Firmware/blob/master/LICENSE.md))
*   Supported airframes (more experimental are supported):
  * [Multicopters](http://px4.io/platforms/multicopters/start)
  * [Fixed wing](http://px4.io/platforms/planes/start)
  * [VTOL](http://px4.io/platforms/vtol/start)
*   Binaries (always up-to-date from master):
  * [Downloads](http://px4.io/firmware/downloads)
*   Releases
  * [Downloads](https://github.com/PX4/Firmware/releases)
*   Forum / Mailing list: [Google Groups](http://groups.google.com/group/px4users)

### Users ###

Please refer to the [user documentation](https://pixhawk.org/users/start) for flying drones with the PX4 flight stack.

### Developers ###

Contributing guide:
  * [CONTRIBUTING.md](https://github.com/PX4/Firmware/blob/master/CONTRIBUTING.md)
  * [PX4 Contribution Guide](http://px4.io/dev/contributing)

Software in the Loop guide:
Use software in the loop [to get started with the codebase](https://pixhawk.org/dev/simulation/native_sitl).

Developer guide:
http://px4.io/dev/

Testing guide:
http://px4.io/dev/unit_tests

This repository contains code supporting these boards:
  * [Snapdragon Flight](https://www.intrinsyc.com/qualcomm-snapdragon-flight/)
  * FMUv1.x
  * FMUv2.x (Pixhawk)
  * FMUv3.x (Pixhawk 2)
  * AeroCore (v1 and v2)
  * STM32F4Discovery (basic support) [Tutorial](https://pixhawk.org/modules/stm32f4discovery)

## NuttShell (NSH) ##

NSH usage documentation:
http://px4.io/users/serial_connection
