# ModalAI VOXL 2

PX4 support for VOXL 2.

## Architecture

When running PX4 directly on the QRB5165 SoC it runs partially on the Sensor Low
Power Island (SLPI) DSP and partially on the ARM CPU cluster.

The portion running on the DSP hosts the flight critical portions of PX4 such as
the IMU, barometer, magnetometer, GPS, ESC, and power management drivers, and the
state estimation. The DSP acts as the real time portion of the system. Non flight
critical applications such as Mavlink, and logging are running on the
ARM CPU cluster (aka apps proc). The DSP and ARM CPU cluster communicate via a
Qualcomm proprietary shared memory interface.

## Build

Building PX4 for VOXL 2 requires the Voxl SDK docker container provided by ModalAI.
The full instructions are available here:
`https://docs.modalai.com/voxl-px4-developer-guide/`

Build commands:
```
px4$ boards/modalai/voxl2/scripts/run-docker.sh
root@9373fa1401b8:/usr/local/workspace# boards/modalai/voxl2/scripts/clean.sh
root@9373fa1401b8:/usr/local/workspace# boards/modalai/voxl2/scripts/build-apps.sh
root@9373fa1401b8:/usr/local/workspace# boards/modalai/voxl2/scripts/build-slpi.sh
root@9373fa1401b8:/usr/local/workspace# exit
```

For DSP-only rebuilds: `make modalai_voxl2_slpi`

### SLPI dynamic imports

The SLPI build permits undefined dynamic symbols supplied by the target
runtime. After linking, every strong dynamic import is checked against the
symbols exported by the ModalAI SSC system image and the C++ runtime shared
objects linked into `libpx4.so`. The SSC exports are recorded in
`slpi_system_image_exports.txt` and must be regenerated from the final SSC ELF
when the supported system image changes.

Imports beginning with `px4_` are always rejected. PX4-owned APIs must be
resolved within `libpx4.so`; leaving one undefined indicates that the QURT
platform implementation is missing.

## Install and run on VOXL 2

Once the DSP and Linux images have been built they can be installed on a VOXL 2
target following the instructions here:
`https://docs.modalai.com/voxl-px4-developer-guide/`

Sample log from running `voxl-px4` on target:
```
INFO  [px4] Setting up environment
px4 starting.

INFO  [px4] Calling startup script: /bin/sh /etc/modalai/voxl-px4.config 0
INFO  [muorb] muorb protobuf initialize method succeeded
INFO  [px4] Startup script returned successfully
pxh>
```

## Notes

You cannot cleanly shutdown PX4 with the shutdown command on VOXL 2. You have
to power cycle the board and restart everything. Starting with SDK 1.3.0 it is possible
to cleanly shutdown PX4 on VOXL 2.

## Tips

Always use the latest SDK release

In order to see DSP specific debug messages the mini-dm tool in the Hexagon SDK
can be used (Most messages are passed to the apps proc but certain low level messages are not):
```
modalai@modalai-XPS-15-9570:/local/mnt/workspace/Qualcomm/Hexagon_SDK/4.1.0.4/tools/debug/mini-dm/Ubuntu18$ sudo ./mini-dm
[sudo] password for modalai:
```
