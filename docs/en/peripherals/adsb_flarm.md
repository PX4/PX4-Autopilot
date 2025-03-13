# ADS-B/FLARM/UTM Receivers: Air Traffic Avoidance

PX4 supports simple air traffic avoidance in [missions](../flying/missions.md) using [ADS-B](https://en.wikipedia.org/wiki/Automatic_dependent_surveillance_%E2%80%93_broadcast), [FLARM](https://en.wikipedia.org/wiki/FLARM), or [UTM](https://www.faa.gov/uas/research_development/traffic_management) transponders that use the standard MAVLink interfaces.

If a potential collision is detected, PX4 can _warn_, immediately [land](../flight_modes_mc/land.md), or [return](../flight_modes_mc/return.md) (depending on the value of [NAV_TRAFF_AVOID](#NAV_TRAFF_AVOID)).

## Supported Hardware

PX4 traffic avoidance works with ADS-B or FLARM products that supply transponder data using the MAVLink [ADSB_VEHICLE](https://mavlink.io/en/messages/common.html#ADSB_VEHICLE) message, and UTM products that supply transponder data using the MAVLink [UTM_GLOBAL_POSITION](https://mavlink.io/en/messages/common.html#UTM_GLOBAL_POSITION) message.

It has been tested with the following devices:

- [PingRX ADS-B Receiver](https://uavionix.com/product/pingrx-pro/) (uAvionix)
- [FLARM](https://flarm.com/products/uav/atom-uav-flarm-for-drones/) <!-- I think originally https://flarm.com/products/powerflarm/uav/ -->

## Hardware Setup

Any of the devices can be connected to any free/unused serial port on the flight controller.
Most commonly they are connected to `TELEM2` (if this is not being use for some other purpose).

### PingRX

The PingRX MAVLink port uses a JST ZHR-4 mating connector with pinout as shown below.

| Pin     | Signal   | Volt         |
| ------- | -------- | ------------ |
| 1 (red) | RX (IN)  | +5V tolerant |
| 2 (blk) | TX (OUT) |
| 3 (blk) | Power    | +4 to 6V     |
| 4 (blk) | GND      | GND          |

The PingRX comes with connector cable that can be attached directly to the TELEM2 port (DF13-6P) on an [mRo Pixhawk](../flight_controller/mro_pixhawk.md).
For other ports or boards, you will need to obtain your own cable.

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

## Software Configuration

### Port Configuration

The recievers are configured in the same way as any other [MAVLink Peripheral](../peripherals/mavlink_peripherals.md).
The only _specific_ setup is that the port baud rate must be set to 57600 and the a low-bandwidth profile (`MAV_X_MODE`).

Assuming you have connected the device to the TELEM2 port, [set the parameters](../advanced_config/parameters.md) as shown:

- [MAV_1_CONFIG](../advanced_config/parameter_reference.md#MAV_1_CONFIG) = TELEM 2
- [MAV_1_MODE](../advanced_config/parameter_reference.md#MAV_1_MODE) = Normal
- [MAV_1_RATE](../advanced_config/parameter_reference.md#MAV_1_RATE) = 0 (default sending rate for port).
- [MAV_1_FORWARD](../advanced_config/parameter_reference.md#MAV_1_FORWARD) = Enabled

Then reboot the vehicle.

You will now find a new parameter called [SER_TEL2_BAUD](../advanced_config/parameter_reference.md#SER_TEL2_BAUD), which must be set to 57600.

### Configure Traffic Avoidance

Configure the action when there is a potential collision using the parameter below:

| Parameter                                                                                                   | Description                                                                                                                                                                       |
| ----------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="NAV_TRAFF_AVOID"></a>[NAV_TRAFF_AVOID](../advanced_config/parameter_reference.md#NAV_TRAFF_AVOID)    | Enable traffic avoidance mode specify avoidance response. 0: Disable, 1: Warn only, 2: Return mode, 3: Land mode.                                                                 |
| <a id="NAV_TRAFF_A_HOR"></a>[NAV_TRAFF_A_HOR](../advanced_config/parameter_reference.md#NAV_TRAFF_A_HOR)    | Horizonal radius of cylinder around the vehicle that defines its airspace (i.e. the airspace in the ground plane).                                                                |
| <a id="NAV_TRAFF_A_VER"></a>[NAV_TRAFF_A_VER](../advanced_config/parameter_reference.md#NAV_TRAFF_A_VER)    | Vertical height above and below vehicle of the cylinder that defines its airspace (also see [NAV_TRAFF_A_HOR](#NAV_TRAFF_A_HOR)).                                                 |
| <a id="NAV_TRAFF_COLL_T"></a>[NAV_TRAFF_COLL_T](../advanced_config/parameter_reference.md#NAV_TRAFF_COLL_T) | Collision time threshold. Avoidance will trigger if the estimated time until collision drops below this value (the estimated time is based on relative speed of traffic and UAV). |

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

## Testing/Simulated ADSB Traffic

You can simulate ADS-B traffic for testing.
Note that this requires that you [Build PX4](../dev_setup/building_px4.md).

::: info
Simulated ADS-B traffic can trigger real failsafe actions.
Use with care in real flight!
:::

To enable this feature:

1. Uncomment the code in `AdsbConflict::run_fake_traffic()`([AdsbConflict.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/adsb/AdsbConflict.cpp#L342C1-L342C1)).
1. Rebuild and run PX4.
1. Execute the [`navigator fake_traffic` command](../modules/modules_controller.md#navigator) in the [QGroundControl MAVLink Shell](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/analyze_view/mavlink_console.html) (or some other [PX4 Console or MAVLink shell](../debug/consoles.md), such as the PX4 simulator terminal).

The code in `run_fake_traffic()` is then executed.
You should see ADS-B warnings in the Console/MAVLink shell, and QGC should also show an ADS-B traffic popup.

By default `run_fake_traffic()` publishes a number of traffic messages (it calls [`AdsbConflict::fake_traffic()`](#fake-traffic-method) to emit each report).
These simulate ADS-B traffic where there may be a conflict, where there won't be a conflict, as well as spamming the traffic buffer.

::: details Information about the test methods

The relevent methods are defined in [AdsbConflict.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/adsb/AdsbConflict.cpp#L342C1-L342C1).

#### `run_fake_traffic()` method

The `run_fake_traffic()` method is run when the `navigator fake_traffic` command is called.

The method calls the `fake_traffic()` method to generate simulated transponder messages around the current vehicle position.
It passes in the current vehicle position, and information about the simulated traffic, such as callsign, distances, directions, altitude differences, velocities, and emitter types.

The (commented out) code in `run_fake_traffic()` simulates a number of different scenarios, including conflicts and non-conflicts, as well as spamming the traffic buffer.

#### `fake_traffic()` method

`AdsbConflict::fake_traffic()` is called by the [`run_fake_traffic()`](#run-fake-traffic-method) to create individual ADS-B transponder reports.

This takes several parameters, which specify the characteristics of the fake traffic:

- `callsign`: Callsign of the fake transponder.
- `distance`: Horizontal distance to the fake vehicle from the current vehicle.
- `direction`: Direction in NED from this vehicle to the fake in radians.
- `traffic_heading`: Travel direction of the traffic in NED in radians.
- `altitude_diff`: Altitude difference of the fake traffic. Positive is up.
- `hor_velocity`: Horizontal velocity of fake traffic, in m/s.
- `ver_velocity`: Vertical velocity of fake traffic, in m/s.
- `emitter_type`: Type of fake vehicle, as an enumerated value.
- `icao_address`: ICAO address.
- `lat_uav`: Lat of this vehicle (used to position fake traffic around vehicle)
- `on_uav`: Lon of this vehicle (used to position fake traffic around vehicle)
- `alt_uav`: Altitude of the vehicle (as reference - used to position fake traffic around vehicle)

The method creates a simulated transponder message near the vehicle, using following steps:

- Calculates the latitude and longitude of the traffic based on the UAV's position, distance, and direction.
- Computes the new altitude by adding the altitude difference to the UAV's altitude.
- Populates a [TransponderReport](../msg_docs/TransponderReport.md) topic with the simulated traffic data.
- If the board supports a Universally Unique Identifier (UUID), the method retrieves the UUID using `board_get_px4_guid` and copies it to the `uas_id` field of the structure.
  Otherwise, it generates a simulated GUID.
- Publishes the simulated traffic message using `orb_publish`.

:::

<!-- See also implementation PR: https://github.com/PX4/PX4-Autopilot/pull/21283 -->
<!-- See also bug to make this work without uncommenting: https://github.com/PX4/PX4-Autopilot/issues/21810 -->

## Further Information

- [MAVLink Peripherals](../peripherals/mavlink_peripherals.md)
- [Serial Port Configuration](../peripherals/serial_configuration.md)
