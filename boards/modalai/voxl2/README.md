# PX4 on VOXL 2

VOXL 2 from ModalAI can be used as a companion computer to a flight control board
but it can also run PX4 directly.

## Overview

When running PX4 directly on the QRB5165 SoC it runs partially on the Sensor Low Power Island (SLPI) DSP (aka sdsp) and partially on the application processor (ARM64 / aarch64).
The portion running on the DSP hosts the flight critical portions of PX4 such as
the IMU, barometer, magnetometer, GPS, ESC, and power management drivers, and the
state estimation. The DSP acts as the real time portion of the system. Non flight
critical applications such as Mavlink, logging, and commander are running on the
ARM CPU cluster (aka apps proc). The DSP and ARM CPU cluster communicate via a
Qualcomm proprietary shared memory interface.

## Build environment

In order to build for this platform both the Qualcomm Hexagon (DSP) toolchain and the Linaro ARM64 toolchain need to be installed. The (nearly) complete setup including the ARM64 toolchain is provided in the base Docker image provided by ModalAI, but since ModalAI is not allowed to redistribute the Qualcomm Hexagon DSP SDK this must be added by the end user.

The full instructions are available here:
- https://gitlab.com/voxl-public/rb5-flight/rb5-flight-px4-build-docker

## Build overview

- Clone the repo (Don't forget to update and initialize all submodules)
- In the top level directory
```
px4$ boards/modalai/voxl2/scripts/run-docker.sh
root@9373fa1401b8:/usr/local/workspace# boards/modalai/voxl2/scripts/clean.sh
root@9373fa1401b8:/usr/local/workspace# boards/modalai/voxl2/scripts/build-apps.sh
root@9373fa1401b8:/usr/local/workspace# boards/modalai/voxl2/scripts/build-slpi.sh
root@9373fa1401b8:/usr/local/workspace# exit
```

## Install and run on VOXL 2

Once the DSP and Linux images have been built they can be installed on a VOXL 2
board using ADB. There is a script to do this.
```
px4$ boards/modalai/voxl2/scripts/install-voxl.sh
```

## Running PX4 on VOXL 2

After installing PX4 on the board, open a terminal on VOXL 2 using ADB shell.
PX4 can be run using a start script.
```
root@m0054:/# voxl-px4
Found DSP signature file
/
INFO  [px4] mlockall() enabled. PX4's virtual address space is locked into RAM.
INFO  [px4] assuming working directory is rootfs, no symlinks needed.

______  __   __    ___
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

px4 starting.

INFO  [px4] Calling startup script: /bin/sh /etc/modalai/voxl-px4.config 0
INFO  [muorb] muorb protobuf initalize method succeeded
INFO  [px4] Startup script returned successfully
pxh>
```

## Notes

You cannot cleanly shutdown PX4 with the shutdown command on VOXL 2. You have
to power cycle the board and restart everything.

## Tips

Start with a VOXL 2 that only has the system image installed, not the SDK

Run the command ```voxl-px4 -s``` on target to run the self-test

In order to see DSP specific debug messages the mini-dm tool in the Hexagon SDK
can be used:
```
modalai@modalai-XPS-15-9570:/local/mnt/workspace/Qualcomm/Hexagon_SDK/4.1.0.4/tools/debug/mini-dm/Ubuntu18$ sudo ./mini-dm
[sudo] password for modalai:
Running mini-dm version: 3.2
Completed processing command line ---
Connecting to the only usbport connected
mini-dm is waiting for a DMSS connection...
DMSS is connected. Running mini-dm...
------------Mini-dm is ready to log-------------
[08500/02]  06:21.030  0069:01: SDSP: Creating new instance.  0355  sns_flight_controller_sensor.
[08500/02]  06:21.030  0069:01: SDSP: Creating data streams  0292  sns_flight_controller_sensor_
[08500/02]  06:21.030  0069:01: SDSP: Configuring devices  0305  sns_flight_controller_sensor_
[08500/02]  06:21.030  0069:01: SDSP: Flight controller sensor instance initialized  0344  sns_flight_controller_sensor_
[08500/01]  06:21.092  0069:01: SDSP: Hello, world!  0000  test
[08500/02]  06:21.092  006a:01: SDSP: CPU Utilization: 0.04, wait percentage 99.94  0162  sns_flight_controller_sensor.
[08500/02]  06:21.092  006a:01: SDSP: Got interrupt registered event  0082  sns_flight_controller_sensor_
```
