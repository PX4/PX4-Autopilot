# ARK DIST SR

ARK DIST SR is a low range, open source [DroneCAN](index.md) [distance sensor](../sensor/rangefinders.md).
It has an approximate range of between 8cm to 30m.

![ARK DIST SR](../../assets/hardware/sensors/optical_flow/ark_dist.jpg)

## Where to Buy

Order this module from:

- [ARK Electronics](https://arkelectron.com/product/ark-dist-sr/) (US)

## Hardware Specifications

- [Open Source Schematic and BOM](https://github.com/ARK-Electronics/ARK_DIST)
- Sensors
  - [Broadcom AFBR-S50LV85D Time-of-Flight Distance Sensor](https://www.broadcom.com/products/optical-sensors/time-of-flight-3d-sensors/afbr-s50lv85d)
    - Typical distance range up to 30m
    - Integrated 850 nm laser light source
    - Field-of-View (FoV) of 12.4° x 6.2° with 32 pixels
    - Operation of up to 200k Lux ambient light
    - Reference Pixel for system health monitoring
    - Works well on all surface conditions
    - Transmitter beam of 2° x 2° to illuminate between 1 and 3 pixels
- Two Pixhawk Standard CAN Connectors (4 Pin JST GH)
- Pixhawk Standard UART Connector (6 Pin JST SH)
- Pixhawk Standard Debug Connector (6 Pin JST SH)
- Small Form Factor
  - 2.0cm x 2.8cm x 1.4cm
  - 4g
- LED Indicators
- USA Built
- NDAA Compliant
- Power Requirements
  - 5v
    - 84mA Average
    - 86mA Max

## Hardware Setup

### Wiring

The ARK DIST is connected to the CAN bus using a Pixhawk standard 4 pin JST GH cable.
For more information, refer to the [CAN Wiring](../can/index.md#wiring) instructions.

The ARK DIST can also be connected with UART and communicates over MAVLink sending the [DISTANCE_SENSOR](https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR) message.

## Firmware Setup

ARK DIST SR runs the [PX4 DroneCAN Firmware](px4_cannode_fw.md).
As such, it supports firmware update over the CAN bus and [dynamic node allocation](index.md#node-id-allocation).

## PX4 Configuration

### DroneCAN

#### Enable DroneCAN

The steps are:

- In _QGroundControl_ set the parameter [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) to `2` for dynamic node allocation (or `3` if using [DroneCAN ESCs](../dronecan/escs.md)) and reboot (see [Finding/Updating Parameters](../advanced_config/parameters.md)).
- Connect ARK DIST SR CAN to the Pixhawk CAN.

Once enabled, the module will be detected on boot.
Distance sensor data should arrive at 40Hz.

DroneCAN configuration in PX4 is explained in more detail in [DroneCAN > Enabling DroneCAN](../dronecan/index.md#enabling-dronecan).

#### CAN Configuration

First set the parameters to [Enable DroneCAN](#enable-dronecan) (as shown above).

Set the following parameters in _QGroundControl_:

- Enable [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) to 2 for dynamic node allocation.
- Enable [UAVCAN_SUB_RNG](../advanced_config/parameter_reference.md#UAVCAN_SUB_RNG).
- Set [EKF2_RNG_A_HMAX](../advanced_config/parameter_reference.md#EKF2_RNG_A_HMAX) to `30`.
- Set [EKF2_RNG_QLTY_T](../advanced_config/parameter_reference.md#EKF2_RNG_QLTY_T) to `0.2`.
- Set [UAVCAN_RNG_MIN](../advanced_config/parameter_reference.md#UAVCAN_RNG_MIN) to `0.08`.
- Set [UAVCAN_RNG_MAX](../advanced_config/parameter_reference.md#UAVCAN_RNG_MAX) to `30`.

See also [Distance Sensor/Range Finder in _DroneCAN > Subscriptions and Publications_](../dronecan/#distance-sensor-range-finder).

### UART/MAVLink Configuration

If connecting via a UART set the following parameters in _QGroundControl_:

- Set [MAV_X_CONFIG](../advanced_config/parameter_reference.md#MAV_0_CONFIG) to the port the sensor is connected to.
- Set [MAV_X_FORWARD](../advanced_config/parameter_reference.md#MAV_0_FORWARD) to `0` (off).
- Set [MAV_X_MODE](../advanced_config/parameter_reference.md#MAV_0_MODE) to `7` or `13` to (Minimal or Low Bandwidth) to reduce memory usage.
- Set `SER_XXX_BAUD` to `115200`, where `XXX` is specific to the port you are using (such as [SER_GPS2_BAUD](../advanced_config/parameter_reference.md#SER_GPS2_BAUD)).
- Set [EKF2_RNG_A_HMAX](../advanced_config/parameter_reference.md#EKF2_RNG_A_HMAX) to `30`.
- Set [EKF2_RNG_QLTY_T](../advanced_config/parameter_reference.md#EKF2_RNG_QLTY_T) to `0.2`.

## See Also

- [ARK DIST SR](https://docs.arkelectron.com/sensor/ark-dist) (ARK Docs)
