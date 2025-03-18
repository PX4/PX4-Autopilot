# Wing Wing Z-84 Pixracer Build

The Wing Wing Z-84 is a flying wing frame.
It is small, rugged and just large enough to host a [Pixracer](../flight_controller/pixracer.md).

Key information:

- **Frame:** Wing Wing Z-84
- **Flight controller:** Pixracer

![Wing Wing Z-84 build](../../assets/airframes/fw/wing_wing/wing_wing_build11.jpg)

## 配件列表

### Z-84 Plug n' Fly (PNF/PNP) or Kit

One of these:

- [Banggood](https://www.banggood.com/Wing-Wing-Z-84-Z84-EPO-845mm-Wingspan-Flying-Wing-PNP-p-973125.html)
- [Hobbyking US Warehouse](https://hobbyking.com/en_us/wing-wing-z-84-epo-845mm-kit.html)

:::tip
PNF (or "PNP") versions include motor, propeller and electronic speed controller.
The "kit" version does not include these components, which must be purchased separately.
:::

### Electronic Speed Controller (ESC)

One of these (any small (>=12A) ESC will do):

- [Turnigy 20A Brushed ESC ESC](https://hobbyking.com/en_us/turnigy-20a-brushed-esc.html) (Hobbyking)
- [Lumenier Regler 30A BLHeli_S ESC OPTO](https://www.getfpv.com/lumenier-30a-blheli-s-esc-opto-2-4s.html) (GetFPV)

### Autopilot and Essential Components

- [Pixracer](../flight_controller/pixracer.md) kit (including GPS and power module)
- FrSky D4R-II receiver or equivalent (jumpered to PPM sum output according to its manual)
- [Mini telemetry set](../flight_controller/pixfalcon.md#availability) for Holybro pix32
- [Digital airspeed sensor](../flight_controller/pixfalcon.md#availability) for Holybro pix32 / Pixfalcon
- 1800 mAh 2S LiPo Battery - e.g. [Team Orion 1800mAh 7.4V 50C 2S1P](https://teamorion.com/en/batteries-en/lipo/soft-case/team-orion-lipo-1800-2s-7-4v-50c-xt60-en/)

### Recommended spare parts

- 1 cm diameter O-ring for prop saver ([Hobbyking](https://hobbyking.com/en_us/wing-wing-z-84-o-ring-10pcs.html))
- 125x110 mm propellers ([Hobbyking](https://hobbyking.com/en_us/gws-ep-propeller-dd-5043-125x110mm-green-6pcs-set.html))

## 布线

Wire the servos and motors as shown.
Use the `MAIN` outputs (not the ones labeled with AUX).
The motor controller needs to have an in-built BEC, as the autopilot is not powering the servo rail.

| Port   | 接口定义                                                        |
| ------ | ----------------------------------------------------------- |
| RC IN  | PPM or S.BUS / S.BUS2 input |
| MAIN 1 | Left Aileron                                                |
| MAIN 2 | Right Aileron                                               |
| MAIN 3 | 空                                                           |
| MAIN 4 | Motor 1                                                     |

## Build Log

The images below give a rough idea about the assembly process, which is simple and can be done with a hot glue gun.

![wing wing build01](../../assets/airframes/fw/wing_wing/wing_wing_build01.jpg)
![wing wing build02](../../assets/airframes/fw/wing_wing/wing_wing_build02.jpg)
![wing wing build03](../../assets/airframes/fw/wing_wing/wing_wing_build03.jpg)
![wing wing build04](../../assets/airframes/fw/wing_wing/wing_wing_build04.jpg)
![wing wing build09](../../assets/airframes/fw/wing_wing/wing_wing_build09.jpg)
![Wing Wing Z-84 build](../../assets/airframes/fw/wing_wing/wing_wing_build11.jpg)

## PX4 配置

### Airframe Configuration

Select **Flying Wing > Generic Flying Wing** in the QGroundControl [Airframe Configuration](../config/airframe.md):

![QGC - select firmware for West Wing](../../assets/airframes/fw/wing_wing/qgc_firmware_flying_wing_west_wing.png)

### Actuator Mapping

Set up the [Actuator Configuration](../config/actuators.md) to match the wiring for the ailerons and throttle as [indicated above](#wiring).

![QGC - set the actuators](../../assets/airframes/fw/wing_wing/qgc_actuator_config.png)

### Other Configuration

Perform all the the other [Basic Configuration](../config/index.md), including [Autotuning](../config/autotune_fw.md).

Advanced tuning is optional - see [Fixed-wing Vehicle Configuration](../config_fw/index.md).
