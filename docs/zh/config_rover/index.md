# Rover Configuration/Tuning

This topic provides a step-by-step guide for setting up your rover.

Successive steps enable [drive modes](../flight_modes_rover/index.md) with more autopilot support and features:

| Step | 配置                                    | Unlocked PX4 Drive Mode                                                   |
| ---- | ------------------------------------- | ------------------------------------------------------------------------- |
| 1    | [Basic Setup](basic_setup.md)         | [Full manual mode](../flight_modes_rover/manual.md#manual-mode)           |
| 2    | [Rate Tuning](rate_tuning.md)         | [Manual acro mode](../flight_modes_rover/manual.md#acro-mode)             |
| 3    | [Attitude Tuning](attitude_tuning.md) | [Manual stabilized mode](../flight_modes_rover/manual.md#stabilized-mode) |
| 4    | [Velocity Tuning](velocity_tuning.md) | [Manual position mode](../flight_modes_rover/manual.md#manual-mode)       |
| 5    | [Position Tuning](position_tuning.md) | [Auto modes](../flight_modes_rover/auto.md)                               |

:::warning
A drive mode will only work properly if all the configuration for the preceding modes has been completed.
:::

## Flashing the Rover Build

Rovers use a custom build that must be flashed onto your flight controller instead of the default PX4 build:

1. First build the rover firmware for your flight controller from the `main` branch (there is no release build, so you can't just select this build from QGroundControl).

   To build for rover with the `make` command, replace the `_default` suffix with `_rover`.
   For example, to build rover for px4_fmu-v6x boards, you would use the command:

   ```sh
   make px4_fmu-v6x_rover
   ```

   ::: info
   You can also enable the modules in default builds by adding these lines to your [board configuration](../hardware/porting_guide_config.md) (e.g. for fmu-v6x you might add them to [`main/boards/px4/fmu-v6x/default.px4board`](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v6x/default.px4board)):

   ```sh
   CONFIG_MODULES_ROVER_ACKERMANN=y
   CONFIG_MODULES_ROVER_DIFFERENTIAL=y
   CONFIG_MODULES_ROVER_MECANUM=y
   ```

   Note that adding the rover modules may lead to flash overflow, in which case you will need to disable modules that you do not plan to use (such as those related to multicopter or fixed wing).

:::

2. Load the **custom firmware** that you just built onto your flight controller (see [Loading Firmware > Installing PX4 Main, Beta or Custom Firmware](../config/firmware.md#installing-px4-main-beta-or-custom-firmware)).
