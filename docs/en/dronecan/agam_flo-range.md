# Agam Flo Range Sensor

Agam Flo Range Sensor is a DroneCAN [optical flow](../sensor/optical_flow.md), [distance sensor](../sensor/rangefinders.md), and IMU module, designed for PX4 and ArduPilot compatible autonomous vehicles.

![Agam Flo Range Sensor](../../assets/hardware/sensors/optical_flow/agam_flo-range.png)

The sensor is suitable for multirotors, fixed-wing aircraft, VTOLs, rovers, boats, underwater vehicles, and other autonomous robotic platforms, and requires **PX4 v1.15 or later**.

The standard package includes:

- Agam Flo Range Sensor
- DroneCAN Cable
- Mounting Hardware
- Quick Start Guide

For enterprise, OEM, and bulk orders, contact Agam Robotics directly through their sales channel.

## Where to Buy

- [Agam Flo Range Sensor](https://www.agamrobotics.com/product-page/agam-florange-sensor)

## Hardware Specifications

- Sensors
  - PixArt PAA3905E1 Optical Flow Sensor
    <!-- check this: figures below are from the vendor product page; confirm before merge -->
    - Working distance 80 mm to infinity
    - 16-bit motion data output
  - Broadcom AFBR-S50LV85D Time-of-Flight Distance Sensor
    <!-- check this: figures below are from the vendor product page; confirm before merge -->
    - Typical range up to 30 m, up to 100 m in dual-frequency mode
    - Field of View 12.4° x 6.2° (32 pixels)
    - Update rate up to 3 kHz
    - Operates in up to 200k lux ambient light, 850 nm laser, Class 1 eye-safe
  - TDK InvenSense ICM-42688-P 6-axis IMU
- STM32F412RET6 MCU (ARM Cortex-M4, up to 100 MHz)
- DroneCAN interface, SWD debug interface, status LED
- Dimensions: 44.95 x 29.4 x 15.59 mm <!-- check this: from vendor product page -->
- Weight: 9.38 g <!-- check this: from vendor product page -->
- Operating voltage: 5 V <!-- check this: current draw / power consumption not published by vendor -->
- Additional hardware protection: reverse polarity, over-voltage, EMI filtering, ESD, brown-out protection, hardware watchdog

## Hardware Setup

### Wiring

The Agam Flo Range Sensor is connected to the CAN bus using a Pixhawk standard 4-pin JST-GH cable.
For more information, refer to the [CAN Wiring](../can/index.md#wiring) instructions.

### Mounting

Mount the sensor securely on the underside of the airframe with an unobstructed downward field of view. <!-- check this: confirm recommended orientation / add orientation image if vendor provides one -->

This corresponds to the default value (`0`) of [SENS_FLOW_ROT](../advanced_config/parameter_reference.md#SENS_FLOW_ROT).
Change the parameter appropriately if using a different orientation.

## Firmware Setup

The Agam Flo Range Sensor integrates with the PX4 DroneCAN framework and follows the standard DroneCAN peripheral architecture, supporting firmware update over the CAN bus and [dynamic node allocation](index.md#node-id-allocation).
<!-- check this: exact firmware/bootloader target names, and whether firmware update over CAN and dynamic
     node allocation are actually supported, are not confirmed on Agam's own site — verify with Agam
     Robotics and compare against ark_flow.md's "Firmware Setup" section before merge. -->

## Flight Controller Setup

::: info
Confirm whether the Agam Flo Range Sensor requires an SD card in the flight controller to boot, as [ARK Flow](ark_flow.md) does. <!-- check this -->
:::

### Enable DroneCAN

The steps are:

- In _QGroundControl_ set the parameter [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) to `2` for dynamic node allocation (or `3` if using [DroneCAN ESCs](escs.md)) and reboot (see [Finding/Updating Parameters](../advanced_config/parameters.md)).
- Connect the Agam Flo Range Sensor CAN to the flight controller CAN.

DroneCAN configuration in PX4 is explained in more detail in [DroneCAN > Enabling DroneCAN](index.md#enabling-dronecan).

### PX4 Configuration

First set the parameters to [Enable DroneCAN](#enable-dronecan) (as shown above).
Then set the EKF optical flow and rangefinder parameters to enable fusing the sensor data, and define offsets if the sensor is not centred within the vehicle.

Set the following parameters in _QGroundControl_:

- Enable optical flow fusion by setting [EKF2_OF_CTRL](../advanced_config/parameter_reference.md#EKF2_OF_CTRL).
- To optionally disable GPS aiding, set [EKF2_GPS_CTRL](../advanced_config/parameter_reference.md#EKF2_GPS_CTRL) to `0`.
- Enable [UAVCAN_SUB_FLOW](../advanced_config/parameter_reference.md#UAVCAN_SUB_FLOW).
- Enable [UAVCAN_SUB_RNG](../advanced_config/parameter_reference.md#UAVCAN_SUB_RNG).
- Set [EKF2_RNG_CTRL](../advanced_config/parameter_reference.md#EKF2_RNG_CTRL) to `1`.
- Set [EKF2_RNG_A_HMAX](../advanced_config/parameter_reference.md#EKF2_RNG_A_HMAX) to match the AFBR-S50LV85D's operating range (up to `30`, or `100` in dual-frequency mode). <!-- check this -->
- Set [EKF2_RNG_QLTY_T](../advanced_config/parameter_reference.md#EKF2_RNG_QLTY_T) to `0.2`.
- Set [UAVCAN_RNG_MIN](../advanced_config/parameter_reference.md#UAVCAN_RNG_MIN) and [UAVCAN_RNG_MAX](../advanced_config/parameter_reference.md#UAVCAN_RNG_MAX) to match the AFBR-S50LV85D's operating range. <!-- check this -->
- Set [SENS_FLOW_MINHGT](../advanced_config/parameter_reference.md#SENS_FLOW_MINHGT) to match the PAA3905E1's minimum working distance (`0.08`). <!-- check this -->
- Set [SENS_FLOW_MAXHGT](../advanced_config/parameter_reference.md#SENS_FLOW_MAXHGT) appropriately for the vehicle's typical operating height. <!-- check this -->
- Set [SENS_FLOW_MAXR](../advanced_config/parameter_reference.md#SENS_FLOW_MAXR) to match the PAA3905E1's maximum angular flow rate. <!-- check this: not published by vendor -->
- The parameters [EKF2_OF_POS_X](../advanced_config/parameter_reference.md#EKF2_OF_POS_X), [EKF2_OF_POS_Y](../advanced_config/parameter_reference.md#EKF2_OF_POS_Y) and [EKF2_OF_POS_Z](../advanced_config/parameter_reference.md#EKF2_OF_POS_Z) can be set to account for the offset of the sensor from the vehicle centre of gravity.

When optical flow is the only source of horizontal position/velocity, then lowering the gain for controller response to horizontal position error [MPC_XY_P](../advanced_config/parameter_reference.md#MPC_XY_P) (e.g. to 0.5) is recommended to reduce oscillations.

## Agam Flo Range Sensor Configuration

<!-- check this: table assumes standard PX4 CAN-node parameters (CANNODE_NODE_ID / CANNODE_TERM), per the
     "Firmware Setup" note above — confirm the sensor actually exposes these before merge -->

On the Agam Flo Range Sensor, you may need to configure the following parameters:

| Parameter                                                                                                | Description                                                                                                                           |
| -------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="CANNODE_NODE_ID"></a>[CANNODE_NODE_ID](../advanced_config/parameter_reference.md#CANNODE_NODE_ID) | CAN node ID (0 for dynamic allocation). If set to 0 (default), dynamic node allocation is used. Set to 1-125 to use a static node ID. |
| <a id="CANNODE_TERM"></a>[CANNODE_TERM](../advanced_config/parameter_reference.md#CANNODE_TERM)          | CAN built-in bus termination.                                                                                                         |

## Applications

Agam Flo Range Sensor is suited to applications including:

- Terrain Following
- Precision Landing / Autonomous Landing
- Optical Flow Navigation / Position Hold
- GPS-denied Navigation
- Indoor Navigation
- Low-altitude Flight
- Precision Agriculture, Surveying, Mapping, Infrastructure Inspection
- Warehouse Automation, Drone Delivery, Search and Rescue
- Ground Robots (UGVs), Surface Vehicles (USVs), Research Platforms

## See Also

- [Agam Flo Range Sensor](https://agamrobotics.gitbook.io/docs/sensors/agam-florange-sensor) (Agam Robotics Docs)
