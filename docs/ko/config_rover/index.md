# Rover Configuration/Tuning

This topic provides a step-by-step guide for setting up your rover.

Successive steps enable [drive modes](../flight_modes_rover/index.md) with more autopilot support and features:

| Step | 설정                                    | Unlocked PX4 Drive Mode                                                   |
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

Rover is built as a [firmware variant](../dev_setup/building_px4.md#px4-make-build-targets), and must be installed as "Custom Firmware" in QGC (other vehicles are present in the default variant).

The release versions of Rover firmware for different boards are attached to the associated GitHub release tag.
For example, you can find `px4_fmu-v5x_rover.px4` on [PX4-Autopilot/releases/tag/v1.16.1](https://github.com/PX4/PX4-Autopilot/releases/tag/v1.16.1).
For the `main` branch version of Rover you will need to [build the firmware](#building-rover).

Load the firmware onto your flight controller as "Custom Firmware" (see [Loading Firmware > Installing PX4 Main, Beta or Custom Firmware](../config/firmware.md#installing-px4-main-beta-or-custom-firmware)).

## Building Rover

Rover is built as the `rover` [firmware variant](../dev_setup/building_px4.md#px4-make-build-targets).
What this means is that when building the firmware with the `make` command, you replace the `_default` suffix in the configuration target with `_rover`.

For example, to build rover for `px4_fmu-v6x` boards, you would use the following command:

```sh
make px4_fmu-v6x_rover
```

Note that configuration targets are constructed with the format "VENDOR_MODEL_VARIANT".

The built firmware can be installed as custom firmware, as shown above in in [Flashing the Rover Build](#flashing-the-rover-build).

:::info
You can also enable the modules in default builds by adding these lines to your [board configuration](../hardware/porting_guide_config.md) (e.g. for fmu-v6x you might add them to [`main/boards/px4/fmu-v6x/default.px4board`](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v6x/default.px4board)):

```sh
CONFIG_MODULES_ROVER_ACKERMANN=y
CONFIG_MODULES_ROVER_DIFFERENTIAL=y
CONFIG_MODULES_ROVER_MECANUM=y
```

Adding the rover modules may lead to flash overflow, in which case you will need to disable modules that you do not plan to use (such as those related to multicopter or fixed wing).
:::
