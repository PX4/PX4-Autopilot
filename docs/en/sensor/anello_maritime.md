# ANELLO Maritime INS

The `anello` driver receives the ANELLO Maritime INS fused navigation solution over a dedicated MAVLink 1 or 2 serial connection.
In primary mode, the INS supplies PX4 vehicle attitude, local NED position, global position and velocity.
The navigation filter runs inside the INS; PX4 EKF2 is disabled.
Onboard accelerometers and gyroscopes remain enabled, including the onboard gyro used by rate control.

## Input Contract

Configure the INS to stream `ATTITUDE_QUATERNION` and/or `ATTITUDE` at 50 Hz or faster and `GLOBAL_POSITION_INT` at 5 Hz or faster.
Validate the required position update rate and latency for the intended controller; 5 Hz is a minimum input requirement, not a flight performance guarantee.
With `ANELLO_AUTOCFG=1` (the default), the driver requests 50 Hz `ATTITUDE_QUATERNION`, 50 Hz `ATTITUDE`, and 10 Hz `GLOBAL_POSITION_INT` using `MAV_CMD_SET_MESSAGE_INTERVAL` on the connected serial link.
Both TX and RX must be connected. Set `ANELLO_AUTOCFG=0` for receive-only operation and configure the INS rates separately.
Body axes must be forward/right/down and attitude must transform body into NED.
Configure mounting alignment in the INS; this driver applies no mounting rotation.

Quaternion messages take priority while fresh (250 ms), with Euler fallback after that deadline.
Global position and NED velocity expire after 1 second.
Altitude comes from `GLOBAL_POSITION_INT.alt` (AMSL millimetres), not `relative_alt`.
Velocity comes from `vx/vy/vz` (centimetres per second); positive `vz` remains down.
See the [MAVLink message definitions](https://mavlink.io/en/messages/common.html).
The first fresh attitude/position pair establishes the fixed local origin.
Earth-ellipsoid altitude, terrain height, acceleration and covariance are unavailable.

The port must carry only the INS output.
The first accepted navigation message or non-GCS heartbeat locks the driver to its system and component IDs until restart.
All navigation messages must use that same pair.
CRC failures, malformed rotations, non-finite attitude/rates and out-of-range coordinates are rejected.
MAVLink signing authentication is not configured; use a trusted physical link.

These messages do not report convergence, GNSS fix quality, covariance, reset counters or navigation health.
The INS must suppress invalid or unconverged output.
Freshness alone cannot detect a frozen solution that continues transmitting, delayed/replayed packets, or degraded dead reckoning.
Local receipt time is used; sender clocks are not synchronised.
An INS reset or solution discontinuity requires stopping operation and reinitialising the system; automatic reset compensation and EKF fallback are not provided.

## Automatic Stream Configuration

The driver discovers the INS system/component IDs from incoming traffic and addresses requests to that peer.
Requests use the board's `MAV_SYS_ID` and the peripheral component ID, with private MAVLink transmit state.
Each rate request waits for an acknowledgment, with at most three attempts.
Unsupported or unanswered requests appear as failures in `anello status`; they do not block receiving navigation.
An accepted acknowledgment confirms the command response, not the measured output rate or navigation quality.

Rates are requested again after more than three seconds without accepted peer traffic, or a detected backward navigation boot-clock jump greater than one second.
This reapplies runtime settings; it does not save configuration inside the INS.
If the INS sends no heartbeat or navigation messages, peer discovery cannot begin.
Reconfiguring streams after an INS restart does not compensate for navigation discontinuities.

## Build

The driver is included in these configurations:

- `cubepilot_cubeorangeplus_default`
- `px4_fmu-v6x_default`
- `px4_sitl_default`

Other boards can enable `CONFIG_DRIVERS_INS_ANELLO=y` with MAVLink support.

## Cube Orange Plus Setup

Use TELEM1 at 921600 baud.
TELEM1 maps to `/dev/ttyS0` on Cube Orange Plus; on FMUv6X it maps to `/dev/ttyS6`.
Check electrical levels against the INS interface documentation before connecting the UART.

This checkout sets the following Cube Orange Plus firmware defaults for the requested ANELLO test setup.
Other board targets retain their existing estimator defaults.

| Parameter | Firmware default |
| --- | --- |
| `MAV_0_CONFIG` | `0` |
| `SENS_ANELLO_CFG` | `101` (TELEM1) |
| `ANELLO_BAUD` | `921600` |
| `ANELLO_MODE` | `1` (primary) |
| `ANELLO_AUTOCFG` | `1` |
| `EKF2_EN` | `0` |
| `ATT_EN` | `0`, if present |
| `SENS_IMU_MODE` | `1` |
| `ANELLO_EPH` | `100` m |
| `ANELLO_EPV` | `100` m |
| `ANELLO_EV` | `100` m/s |
| `SYS_HAS_GPS` | `0` (no separate GNSS receiver) |

These are `param set-default` settings: saved user values take precedence after flashing.
Inspect the parameters and reset only the intended parameters to their defaults when needed; preserve sensor calibrations.
The three uncertainty values are user-selected test assumptions, not measured accuracy.
At these values, PX4 accuracy checks may reject navigation for position control.
No accuracy or arming checks are relaxed.

For a staged monitor-to-primary bench test, explicitly apply the following settings:

1. Move or disable every existing serial consumer assigned to TELEM1, including the default MAVLink instance (`MAV_0_CONFIG` when assigned to this port).
   USB telemetry can remain enabled.
2. Set `SENS_ANELLO_CFG=101` (TELEM1), `ANELLO_BAUD=921600`, and `ANELLO_MODE=0`.
3. Reboot and inspect `anello status`, `listener external_ins_attitude`, `listener external_ins_local_position`, and `listener external_ins_global_position`.
4. Configure `ANELLO_EPH`, `ANELLO_EPV`, and `ANELLO_EV` with validated conservative one-sigma bounds for horizontal position (m), vertical position (m), and velocity (m/s).
   The generic parameter default `-1` keeps position and velocity invalid rather than claiming unknown accuracy is perfect; the Cube defaults above override it.
   These fixed assumptions are not measured INS uncertainties and do not adapt to GNSS degradation.
5. To select the INS as primary, set `ANELLO_MODE=1`, `EKF2_EN=0`, `ATT_EN=0` (if this parameter is present), and `SENS_IMU_MODE=1`, then reboot.
   Disable any other external estimator publisher.
   The driver refuses primary startup if another vehicle estimate is already advertised.
6. Verify `listener vehicle_attitude`, `listener vehicle_local_position`, `listener vehicle_global_position`, `listener estimator_status`, and `listener vehicle_angular_velocity`.
   Confirm the latter still comes from an onboard gyro.

Keep onboard inertial sensors calibrated and enabled.
Do not select ANELLO as a raw GPS or IMU source; this protocol supplies fused estimates.
When no separate GNSS receiver or magnetometer is installed, configure `SYS_HAS_GPS` and `SYS_HAS_MAG` to match the actual installation.
Do not disable unrelated arming checks to make the integration pass.

The baud rate is owned by `ANELLO_BAUD`, not `SER_TEL1_BAUD`.
For a manual bench start, use `anello start -d /dev/ttyS0` after configuring parameters and rebooting.
Primary mode requires a reboot after stopping the driver because the previous uORB advertisements can remain present.
To restore onboard estimation, select monitor mode (or disable the ANELLO serial assignment), re-enable `EKF2_EN`, and reboot.

## Verification

After building SITL, run:

```sh
python3 src/drivers/ins/anello/test_parser.py
```

For the serial integration test, build the real-time SITL variant and use a Python environment with `pymavlink` installed:

```sh
make px4_sitl_nolockstep
python3 src/drivers/ins/anello/test_sitl.py
```

This test uses a pseudo-UART at 921600 baud and an isolated PX4 instance with temporary parameters.
It checks monitor isolation, primary-start guards, local/global conversions, independent attitude/position timeouts and recovery without moving the origin.
It also checks receive-only mode, request addressing and rates, acknowledgment retry, and stream reconfiguration after INS reboot and link interruption.
The real-time variant is used because the test sender runs on wall time rather than a physics simulator clock.

The decoder tests cover MAVLink 1/2 framing, fragmented packets, CRC rejection and recovery, quaternion normalisation and priority, Euler fallback, invalid values, source isolation, coordinate units and timeout boundaries.
Stream configuration tests cover private transmit framing, partial writes, acknowledgment filtering, bounded retries, rejection/progress responses, reconnect detection and sender-clock wrap.

Before operational use, bench-test the physical link and configured rates, axis signs, heading, altitude datum and local origin.
Disconnect the INS and stop each output stream separately to verify the expected attitude and navigation failsafes.
Attitude is explicitly invalidated after 250 ms; position/velocity are invalidated after 1 second or loss of attitude.
Test resumed streams and verify the fixed origin does not move.
Hardware control performance, vibration, delay, INS alignment and vehicle-specific failsafe behaviour require validation on the actual vehicle.
