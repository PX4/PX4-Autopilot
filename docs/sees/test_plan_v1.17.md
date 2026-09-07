# Verification Plan: Sees Fork (`v1.17.0-dev` branch)

* Verification plan for custom modifications applied to Sees fork of open-source [PX4 firmware](https://github.com/PX4/PX4-Autopilot).
* Branch v1.17.0-dev
* Companion to: `docs/sees/migration_plan_v1.17.md`


> NOTE: This document is work in progress until V1.17 migration is complete

---

## Methodology

A test defines the expected behavior, how to exercise it, and pass criteria. There are four execution lanes for verification, cheapest first:

| Lane | ID | What it covers | Tooling |
|------|----|-----------|---------|
| Unit | U | Pure logic | `make tests` |
| SITL | S | Module interaction, mode-transition logic, mavlink streams | see `SeesInterface2/Notes/Simulations.md` |
| HITL (bench) | B | Driver-level behavior needing real peripherals (CAN GPS, batmon, IMU, radios) | Bench rig with the specific sensor/radio, `nsh` console, log inspection (`ulog`) |
| Flight-Whitby | FW | End-to-end behavior only observable in flight; verifiable on development drone (Whitby) | Test flight per existing Sees flight-test procedure |
| Flight-Yealm | FY | End-to-end behavior only observable in flight; verifiable only on operational drone (Yealm) | Test flight per existing Sees flight-test procedure |

---

## Tests

### SITL Tests

**Test ID** | S-001
--|--
**PR** | [101](https://github.com/SEESAI/Firmware/pull/101)
**Behaviour** | No unexpected yaw on takeoff or hold with a small `NAV_ACC_RAD`
**Test lane** | SITL
**Result** (Pass/Fail) | Pass (SIH, 2026-10-13)

**Background**
- With `MPC_YAW_MODE` 0 (default), `FlightTaskAuto::_set_heading_from_mode()` points the nose at the target whenever horizontal distance to it exceeds the acceptance radius. During takeoff and hold the target is directly overhead, so a tiny radius makes the vehicle chase its own position jitter. The fix floors the radius used for this check at 1 m (`MIN_HEADING_UPDATE_RADIUS`).
- SITL reproduces the bug: SIH hover drift (~0.7-0.8 m) is far above 0.05 m. A flight test is not needed to verify the fix.

**Test Method**
- `make px4_sitl sihsim_quadx`, wait for the EKF to have a valid local position.
- `param set NAV_ACC_RAD 0.05` (the parameter's min).
- `commander takeoff`, then hold for ~40 s. Sample `vehicle_local_position` (`heading`, `x`, `y`) at 2 Hz, or review the ulog.
- Repeat with `param set NAV_ACC_RAD 2` (multicopter default).
- Negative control (optional, to confirm the test still detects the bug): set `MIN_HEADING_UPDATE_RADIUS` to `0.f`, rebuild, repeat at 0.05.
- **Pass** if, in both runs:
  - heading stays within ±10° of its value at takeoff;
  - `param show NAV_ACC_RAD` still reads back the value that was set (the fix doesn't rewrite the param).

**Results** (heading change vs takeoff, cumulative yaw travel, max horizontal drift)

Build | `NAV_ACC_RAD` | Heading range | Yaw travel | Max drift | Result
--|--|--|--|--|--
Fix | 0.05 | -2.1° to +2.4° | 29° | 0.82 m | Pass
Fix | 2 | -2.5° to +1.5° | 32° | 0.84 m | Pass
Negative control (floor = 0) | 0.05 | -146° to +112° | 761° | 0.73 m | Fails as expected

**Note:** SIH drift (~0.8 m) is close to the 1 m floor. Real wind/GNSS drift above 1 m would make the vehicle turn toward the takeoff point. That is the same behaviour as upstream with `NAV_ACC_RAD` 1, but it may justify a larger floor (e.g. 2 m, the MC default). Check in flight logs from Whitby.

------
------

**Test ID** | S-002
--|--
**PR** | [109](https://github.com/SEESAI/Firmware/pull/109)
**Behaviour** | SI2 receives STATUSTEXT notifications
**Test lane** | SITL
**Result** | --

**Test Method**
- **TODO**
- Connect to the onboard instance (-m onboard, port 14540/14580) without sending a GCS heartbeat. Trigger a message (e.g. `commander arm`) and confirm STATUSTEXT arrives. This is exactly the is_onboard_controller_connected() path the fix changes.

------
------

### HITL (Bench) Tests

Test ID | B-001
--|--
**PR** | [103](https://github.com/SEESAI/Firmware/pull/103)
**Behaviour** | Hygrometer (SHT3x) driver enabled by default
**Test lane** | Bench
**Result** | --

**Test Method**
- Boot the CubeOrange with the sensor attached
- Run `sht3x status`. Verify **TODO: specify expected output**
- Run `listener sensor_hygrometer`. **TODO: specify expected output**

------
------

**Test ID** | B-002
--|--
**PR** | [107](https://github.com/SEESAI/Firmware/pull/107)
**Behaviour** | Audible BVLOS compliant tone on kill-switch and entry into AUTO_RTL
**Test lane** | Bench
**Result** | --

**Test Method**
- That the tone is triggered.
- Run `listener tune_control` and look for tune_id = TUNE_ID_BATTERY_WARNING_FAST.
- RTL: `commander takeoff`, then `commander mode auto:rtl`. The topic should repeat about every 800 ms for as long as RTL lasts (navigator_main.cpp).
- Kill: set `RC_MAP_KILL_SW` and drive that channel with `RC_CHANNELS_OVERRIDE` from pymavlink.

------
------

**Test ID** | B-003
--|--
**PR** | [108](https://github.com/SEESAI/Firmware/pull/108)
**Behaviour** | Four MAVLink instances are available
**Test lane** | Bench
**Result** | --

**Test Method**
- **TODO**
- set MAV_0..3_CONFIG to four serial ports, reboot, and confirm `mavlink status` lists 4 instances. Check free RAM as well.

------
------

**Test ID** | B-004
--|--
**PR** | [104](https://github.com/SEESAI/Firmware/pull/104)
**Behaviour** | FSI2 indicates whether valid RTK corrections are being received
**Test lane** | Bench
**Result** | --

**Test Method**
- **TODO**
- The fix only turns on the GPS_RTCM_DATA stream for onboard instances. Send fake GPS_RTCM_DATA into the GCS port (14550) and check it comes back out on the onboard port, then stops when you stop sending. That covers what this change touches.
- Whether SI2's flag tracks the real RTK status needs a real RTK GPS and base station. The simulated GPS is always a plain 3D fix. That can be done outdoors on the bench, so it doesn't need to be a flight test.

------
------

### Flight tests

- TODO: Rich wants an additional flight test that in summary makes aggressive manual manoeuvres to make sure the Yealm never goes unstable.


### V1.17 DEFER_UNTIL_TEST ITEMS

ID | (#PR) Behaviour | Test lane: pass criteria | result (Pass/Fail)
---|-----------------|--------------------------|-------------------


## TODO

- Write verification statements for #102, #106 and all other DEFER_UNTIL_TEST items
