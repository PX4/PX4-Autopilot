# VESC ESCs (DroneCAN)

The [VESC project](https://vesc-project.com/) is a fully open source hardware and software design for advanced FOC motor controllers.
While it can be controlled using traditional PWM input, it also supports being connected over CAN bus using [DroneCAN](../dronecan/index.md).

## Where to Buy

[Vesc Project > Hardware](https://vesc-project.com/Hardware)

## Hardware Setup

### Wiring

ESCs are connected to the CAN bus using the VESC CAN connector. Note that this is _not_ the Pixhawk standard 4 pin JST GH connector. For more information, refer to the [CAN Wiring](../can/index.md#wiring) instructions. ESC order does not matter.

## Firmware Setup

The preferred tool for motor enumeration is the [VESC tool](https://vesc-project.com/vesc_tool).
In addition to the normal motor configuration that you will have to setup in the VESC tool, you will also need to properly setup the app configuration.
The recommended app setup is as follows:

| Parameter               | Option                 |
| ----------------------- | ---------------------- |
| App to use              | `No App`               |
| VESC ID                 | `1,2,...`              |
| Can Status Message Mode | `CAN_STATUS_1_2_3_4_5` |
| CAN Baud Rate           | `CAN_BAUD_500K`        |
| CAN Mode                | `UAVCAN`               |
| UAVCAN ESC Index        | `0,1,...`              |

VESC ID should have the same motor numbering as in PX4 convention, starting at `1` for top-right motor, `2` for bottom-left motor etc.
However the `UAVCAN ESC Index` starts from `0`, and as such it is always one index lower than the `VESC ID`.
For example, in a quadcopter the bottom left motor will have `VESC ID = 2` and `UAVCAN ESC Index = 1`.

Finally the `CAN Baud Rate` must match the value set in [UAVCAN_BITRATE](../advanced_config/parameter_reference.md#UAVCAN_BITRATE).

## Flight Controller Setup

### Enable DroneCAN

Connect the ESCs to the Pixhawk CAN bus. Power up the entire vehicle using a battery or power supply (not just the flight controller over USB) and enable the DroneCAN driver by setting the parameter [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) to `3` to enable both dynamic node ID allocation and DroneCAN ESC output.

### PX4 Configuration

Assign motors to outputs using the [Acutator](../config/actuators.md#actuator-testing) configuration screen.

<!-- removed as there is no info for it in linked doc -->
<!--
## Troubleshooting

See DroneCAN Troubleshooting - (index.md#troubleshooting).
-->

## Further Information

- [VESC Project ESCs](https://vesc-project.com/)
- [Benjamin Vedder's blog](http://vedder.se) (project owner)
