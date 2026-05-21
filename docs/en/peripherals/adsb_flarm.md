# ADS-B/FLARM/UTM Receivers: Air Traffic Avoidance

PX4 supports simple air traffic avoidance in [missions](../flying/missions.md) using [ADS-B](https://en.wikipedia.org/wiki/Automatic_dependent_surveillance_%E2%80%93_broadcast), [FLARM](https://en.wikipedia.org/wiki/FLARM), or [UTM](https://www.faa.gov/uas/advanced_operations/traffic_management) transponders that use the standard MAVLink interfaces.

If a potential collision is detected, PX4 can _warn_, immediately [land](../flight_modes_mc/land.md), or [return](../flight_modes_mc/return.md) (depending on the value of [NAV_TRAFF_AVOID](#NAV_TRAFF_AVOID)).

## Supported Hardware

PX4 traffic avoidance works with ADS-B or FLARM products that supply transponder data using the MAVLink [ADSB_VEHICLE](https://mavlink.io/en/messages/common.html#ADSB_VEHICLE) message, and UTM products that supply transponder data using the MAVLink [UTM_GLOBAL_POSITION](https://mavlink.io/en/messages/common.html#UTM_GLOBAL_POSITION) message.

It has been tested with the following devices:

- [PingRX ADS-B Receiver](https://uavionix.com/product/pingrx-pro/) (uAvionix)
- [FLARM](https://www.flarm.com/en/drones/)

## Hardware Setup

Any of the devices can be connected to any free/unused serial port on the flight controller.
Most commonly they are connected to `TELEM2` (if this is not being use for some other purpose).

### PingRX Pro

The PingRX MAVLink port uses a JST ZHR-4 mating connector with pinout as shown below.

| Pin     | Signal   | Volt         |
| ------- | -------- | ------------ |
| 1 (red) | RX (IN)  | +5V tolerant |
| 2 (blk) | TX (OUT) |
| 3 (blk) | Power    | +4 to 6V     |
| 4 (blk) | GND      | GND          |

The PingRX comes with connector cable that can be attached directly to the `TELEM2` port (DF13-6P) on an [mRo Pixhawk](../flight_controller/mro_pixhawk.md).
For other ports or boards, you will need to obtain your own cable.

The recommended port configuration for this receiver is:

| Parameter                                                                    | Recommended Value |
| ---------------------------------------------------------------------------- | ----------------- |
| [MAV_X_CONFIG](../advanced_config/parameter_reference.md#MAV_1_CONFIG)       | `TELEM 2`         |
| [MAV_X_MODE](../advanced_config/parameter_reference.md#MAV_1_MODE)           | uAvionix          |
| [MAV_X_RADIO_CTL](../advanced_config/parameter_reference.md#MAV_1_RADIO_CTL) | Disabled          |

## FLARM

FLARM has an on-board DF-13 6 Pin connector that has an identical pinout to the [mRo Pixhawk](../flight_controller/mro_pixhawk.md).

| Pin     | Signal   | Volt        |
| ------- | -------- | ----------- |
| 1 (red) | VCC      | +4V to +36V |
| 2 (blk) | TX (OUT) | +3.3V       |
| 3 (blk) | RX (IN)  | +3.3V       |
| 4 (blk) | -        | +3.3V       |
| 5 (blk) | -        | +3.3V       |
| 6 (blk) | GND      | GND         |

::: info
The TX and RX on the flight controller must be connected to the RX and TX on the FLARM, respectively.
:::

## PX4 Configuration

### Port Configuration

The receivers are configured in the same way as any other [MAVLink Peripheral](../peripherals/mavlink_peripherals.md).
The recommended configuration for most devices (unless they have device-specific configuration like PingRX) is to connect to `TELEM 2` and [set the parameters](../advanced_config/parameters.md) as shown:

| Parameter                                                                | Recommended Value                 |
| ------------------------------------------------------------------------ | --------------------------------- |
| [MAV_X_CONFIG](../advanced_config/parameter_reference.md#MAV_1_CONFIG)   | `TELEM 2`                         |
| [MAV_X_MODE](../advanced_config/parameter_reference.md#MAV_1_MODE)       | Normal                            |
| [MAV_X_RATE](../advanced_config/parameter_reference.md#MAV_1_RATE)       | 0 (default sending rate for port) |
| [MAV_X_FORWARD](../advanced_config/parameter_reference.md#MAV_1_FORWARD) | Enabled                           |

Then reboot the vehicle.

You will now find a new parameter called [SER_TEL2_BAUD](../advanced_config/parameter_reference.md#SER_TEL2_BAUD), which must be set to 57600.

### Configure Traffic Avoidance

Configure the action when there is a potential collision using the parameter below:

| Parameter                                                                                                   | Description                                                                                                                                                                       |
| ----------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="NAV_TRAFF_AVOID"></a>[NAV_TRAFF_AVOID](../advanced_config/parameter_reference.md#NAV_TRAFF_AVOID)    | Enable traffic avoidance mode specify avoidance response. 0: Disable, 1: Warn only, 2: Return mode, 3: Land mode.                                                                 |
| <a id="NAV_TRAFF_A_HOR"></a>[NAV_TRAFF_A_HOR](../advanced_config/parameter_reference.md#NAV_TRAFF_A_HOR)    | Horizontal radius of cylinder around the vehicle that defines its airspace (i.e. the airspace in the ground plane).                                                               |
| <a id="NAV_TRAFF_A_VER"></a>[NAV_TRAFF_A_VER](../advanced_config/parameter_reference.md#NAV_TRAFF_A_VER)    | Vertical height above and below vehicle of the cylinder that defines its airspace (also see [NAV_TRAFF_A_HOR](#NAV_TRAFF_A_HOR)).                                                 |
| <a id="NAV_TRAFF_COLL_T"></a>[NAV_TRAFF_COLL_T](../advanced_config/parameter_reference.md#NAV_TRAFF_COLL_T) | Collision time threshold. Avoidance will trigger if the estimated time until collision drops below this value (the estimated time is based on relative speed of traffic and UAV). |

### Arming Check

PX4 can be configured to check for the presence of a traffic avoidance system (ADSB or FLARM transponder) before arming.
This ensures that a traffic avoidance system is connected and functioning before flight.

The check is configured using the [COM_ARM_TRAFF](../advanced_config/parameter_reference.md#COM_ARM_TRAFF) parameter:

| Value | Description                                                                                                                |
| ----- | -------------------------------------------------------------------------------------------------------------------------- |
| 0     | Disabled (default). No check is performed.                                                                                 |
| 1     | Warning only. A warning is issued if no traffic avoidance system is detected, but arming is allowed.                       |
| 2     | Enforce for all modes. Arming is denied if no traffic avoidance system is detected, regardless of flight mode.             |
| 3     | Enforce for mission modes only. Arming is denied if no traffic avoidance system is detected and a mission mode is planned. |

When a traffic avoidance system is detected, the system tracks its presence with a 3-second timeout.
If the system is lost or regained, corresponding events are logged ("Traffic avoidance system lost" / "Traffic avoidance system regained").

## Implementation

### ADSB/FLARM

PX4 listens for valid transponder reports during missions.

If a valid transponder report is received, PX4 first uses the traffic transponder information to estimate whether the traffic heading and height indicates there will be an intersection with the airspace of the UAV.
The UAV airspace consists of a surrounding cylinder defined by the radius [NAV_TRAFF_A_HOR](#NAV_TRAFF_A_HOR) and height [NAV_TRAFF_A_VER](#NAV_TRAFF_A_VER), with the UAV at it's center.
The traffic detector then checks if the time until intersection with the UAV airspace is below the [NAV_TRAFF_COLL_T](#NAV_TRAFF_COLL_T) threshold based on the relative speed.
If the both checks are true, the [Traffic Avoidance Failsafe](../config/safety.md#traffic-avoidance-failsafe) action is started, and the vehicle will either warn, land, or return.

The code can be found in `Navigator::check_traffic` ([/src/modules/navigator/navigator_main.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/navigator/navigator_main.cpp)).

PX4 will also forward the transponder data to a GCS if this has been configured for the MAVLink instance (this is recommended).
The last 10 Digits of the GUID is displayed as Drone identification.

### UTM

PX4 listens for `UTM_GLOBAL_POSITION` MAVLink messages during missions.
When a valid message is received, its validity flags, position and heading are mapped into the same `transponder_report` UORB topic used for _ADS-B traffic avoidance_.

The implementation is otherwise _exactly_ as described in the section above.

::: info
[UTM_GLOBAL_POSITION](https://mavlink.io/en/messages/common.html#UTM_GLOBAL_POSITION) contains additional fields that are not provided by an ADSB transponder (see [ADSB_VEHICLE](https://mavlink.io/en/messages/common.html#ADSB_VEHICLE)).
The current implementation simply drops the additional fields (including information about the vehicle's planned next waypoint).
:::

## Further Information

- [MAVLink Peripherals](../peripherals/mavlink_peripherals.md)
- [Serial Port Configuration](../peripherals/serial_configuration.md)
