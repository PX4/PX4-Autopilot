---
pageClass: is-wide-page
---

# VehicleStatusV1 (UORB message)

Encodes the system state of the vehicle published by commander.

**TOPICS:** vehicle_statusv1

## Fields

| Name                         | Type     | Unit [Frame]   | Range/Enum                                    | Description                                                                                     |
| ---------------------------- | -------- | -------------- | --------------------------------------------- | ----------------------------------------------------------------------------------------------- |
| timestamp                    | `uint64` | us             |                                               | Time since system start                                                                         |
| armed_time                   | `uint64` | us             |                                               | Arming timestamp                                                                                |
| takeoff_time                 | `uint64` | us             |                                               | Takeoff timestamp                                                                               |
| arming_state                 | `uint8`  |                |                                               |
| latest_arming_reason         | `uint8`  |                |                                               |
| latest_disarming_reason      | `uint8`  |                |                                               |
| nav_state_timestamp          | `uint64` |                |                                               | Time when current nav_state activated                                                           |
| nav_state_user_intention     | `uint8`  |                |                                               | Mode that the user selected (might be different from nav_state in a failsafe situation)         |
| nav_state                    | `uint8`  |                | [NAVIGATION_STATE](#NAVIGATION_STATE)         | Currently active mode                                                                           |
| executor_in_charge           | `uint8`  |                |                                               | Current mode executor in charge (0=Autopilot)                                                   |
| valid_nav_states_mask        | `uint32` |                |                                               | Bitmask for all valid nav_state values                                                          |
| can_set_nav_states_mask      | `uint32` |                |                                               | Bitmask for all modes that a user can select                                                    |
| failure_detector_status      | `uint16` |                | [FAILURE](#FAILURE)                           |
| hil_state                    | `uint8`  | enum HIL_STATE |                                               |
| vehicle_type                 | `uint8`  |                | [VEHICLE_TYPE](#VEHICLE_TYPE)                 |
| failsafe                     | `bool`   |                |                                               | true if system is in failsafe state (e.g.:RTL, Hover, Terminate, ...)                           |
| failsafe_and_user_took_over  | `bool`   |                |                                               | true if system is in failsafe state but the user took over control                              |
| failsafe_defer_state         | `uint8`  |                | [FAILSAFE_DEFER_STATE](#FAILSAFE_DEFER_STATE) |
| gcs_connection_lost          | `bool`   |                |                                               | datalink to GCS lost                                                                            |
| gcs_connection_lost_counter  | `uint8`  |                |                                               | counts unique GCS connection lost events                                                        |
| high_latency_data_link_lost  | `bool`   |                |                                               | Set to true if the high latency data link (eg. RockBlock Iridium 9603 telemetry module) is lost |
| is_vtol                      | `bool`   |                |                                               | True if the system is VTOL capable                                                              |
| is_vtol_tailsitter           | `bool`   |                |                                               | True if the system performs a 90° pitch down rotation during transition from MC to FW           |
| in_transition_mode           | `bool`   |                |                                               | True if VTOL is doing a transition                                                              |
| in_transition_to_fw          | `bool`   |                |                                               | True if VTOL is doing a transition from MC to FW                                                |
| system_type                  | `uint8`  |                |                                               | system type, contains mavlink MAV_TYPE                                                          |
| system_id                    | `uint8`  |                |                                               | system id, contains MAVLink's system ID field                                                   |
| component_id                 | `uint8`  |                |                                               | subsystem / component id, contains MAVLink's component ID field                                 |
| safety_button_available      | `bool`   |                |                                               | Set to true if a safety button is connected                                                     |
| safety_off                   | `bool`   |                |                                               | Set to true if safety is off                                                                    |
| power_input_valid            | `bool`   |                |                                               | Set if input power is valid                                                                     |
| usb_connected                | `bool`   |                |                                               | Set to true (never cleared) once telemetry received from usb link                               |
| open_drone_id_system_present | `bool`   |                |                                               |
| open_drone_id_system_healthy | `bool`   |                |                                               |
| parachute_system_present     | `bool`   |                |                                               |
| parachute_system_healthy     | `bool`   |                |                                               |
| rc_calibration_in_progress   | `bool`   |                |                                               |
| calibration_enabled          | `bool`   |                |                                               |
| pre_flight_checks_pass       | `bool`   |                |                                               | true if all checks necessary to arm pass                                                        |

## Enums

### NAVIGATION_STATE {#NAVIGATION_STATE}

| Name                                                                                    | Type    | Value | Description                           |
| --------------------------------------------------------------------------------------- | ------- | ----- | ------------------------------------- |
| <a href="#NAVIGATION_STATE_MANUAL"></a> NAVIGATION_STATE_MANUAL                         | `uint8` | 0     | Manual mode                           |
| <a href="#NAVIGATION_STATE_ALTCTL"></a> NAVIGATION_STATE_ALTCTL                         | `uint8` | 1     | Altitude control mode                 |
| <a href="#NAVIGATION_STATE_POSCTL"></a> NAVIGATION_STATE_POSCTL                         | `uint8` | 2     | Position control mode                 |
| <a href="#NAVIGATION_STATE_AUTO_MISSION"></a> NAVIGATION_STATE_AUTO_MISSION             | `uint8` | 3     | Auto mission mode                     |
| <a href="#NAVIGATION_STATE_AUTO_LOITER"></a> NAVIGATION_STATE_AUTO_LOITER               | `uint8` | 4     | Auto loiter mode                      |
| <a href="#NAVIGATION_STATE_AUTO_RTL"></a> NAVIGATION_STATE_AUTO_RTL                     | `uint8` | 5     | Auto return to launch mode            |
| <a href="#NAVIGATION_STATE_POSITION_SLOW"></a> NAVIGATION_STATE_POSITION_SLOW           | `uint8` | 6     |
| <a href="#NAVIGATION_STATE_FREE5"></a> NAVIGATION_STATE_FREE5                           | `uint8` | 7     |
| <a href="#NAVIGATION_STATE_ALTITUDE_CRUISE"></a> NAVIGATION_STATE_ALTITUDE_CRUISE       | `uint8` | 8     | Altitude with Cruise mode             |
| <a href="#NAVIGATION_STATE_FREE3"></a> NAVIGATION_STATE_FREE3                           | `uint8` | 9     |
| <a href="#NAVIGATION_STATE_ACRO"></a> NAVIGATION_STATE_ACRO                             | `uint8` | 10    | Acro mode                             |
| <a href="#NAVIGATION_STATE_FREE2"></a> NAVIGATION_STATE_FREE2                           | `uint8` | 11    |
| <a href="#NAVIGATION_STATE_DESCEND"></a> NAVIGATION_STATE_DESCEND                       | `uint8` | 12    | Descend mode (no position control)    |
| <a href="#NAVIGATION_STATE_TERMINATION"></a> NAVIGATION_STATE_TERMINATION               | `uint8` | 13    | Termination mode                      |
| <a href="#NAVIGATION_STATE_OFFBOARD"></a> NAVIGATION_STATE_OFFBOARD                     | `uint8` | 14    |
| <a href="#NAVIGATION_STATE_STAB"></a> NAVIGATION_STATE_STAB                             | `uint8` | 15    | Stabilized mode                       |
| <a href="#NAVIGATION_STATE_FREE1"></a> NAVIGATION_STATE_FREE1                           | `uint8` | 16    |
| <a href="#NAVIGATION_STATE_AUTO_TAKEOFF"></a> NAVIGATION_STATE_AUTO_TAKEOFF             | `uint8` | 17    | Takeoff                               |
| <a href="#NAVIGATION_STATE_AUTO_LAND"></a> NAVIGATION_STATE_AUTO_LAND                   | `uint8` | 18    | Land                                  |
| <a href="#NAVIGATION_STATE_AUTO_FOLLOW_TARGET"></a> NAVIGATION_STATE_AUTO_FOLLOW_TARGET | `uint8` | 19    | Auto Follow                           |
| <a href="#NAVIGATION_STATE_AUTO_PRECLAND"></a> NAVIGATION_STATE_AUTO_PRECLAND           | `uint8` | 20    | Precision land with landing target    |
| <a href="#NAVIGATION_STATE_ORBIT"></a> NAVIGATION_STATE_ORBIT                           | `uint8` | 21    | Orbit in a circle                     |
| <a href="#NAVIGATION_STATE_AUTO_VTOL_TAKEOFF"></a> NAVIGATION_STATE_AUTO_VTOL_TAKEOFF   | `uint8` | 22    | Takeoff, transition, establish loiter |
| <a href="#NAVIGATION_STATE_EXTERNAL1"></a> NAVIGATION_STATE_EXTERNAL1                   | `uint8` | 23    |
| <a href="#NAVIGATION_STATE_EXTERNAL2"></a> NAVIGATION_STATE_EXTERNAL2                   | `uint8` | 24    |
| <a href="#NAVIGATION_STATE_EXTERNAL3"></a> NAVIGATION_STATE_EXTERNAL3                   | `uint8` | 25    |
| <a href="#NAVIGATION_STATE_EXTERNAL4"></a> NAVIGATION_STATE_EXTERNAL4                   | `uint8` | 26    |
| <a href="#NAVIGATION_STATE_EXTERNAL5"></a> NAVIGATION_STATE_EXTERNAL5                   | `uint8` | 27    |
| <a href="#NAVIGATION_STATE_EXTERNAL6"></a> NAVIGATION_STATE_EXTERNAL6                   | `uint8` | 28    |
| <a href="#NAVIGATION_STATE_EXTERNAL7"></a> NAVIGATION_STATE_EXTERNAL7                   | `uint8` | 29    |
| <a href="#NAVIGATION_STATE_EXTERNAL8"></a> NAVIGATION_STATE_EXTERNAL8                   | `uint8` | 30    |
| <a href="#NAVIGATION_STATE_MAX"></a> NAVIGATION_STATE_MAX                               | `uint8` | 31    |

### FAILURE {#FAILURE}

| Name                                                            | Type     | Value | Description |
| --------------------------------------------------------------- | -------- | ----- | ----------- |
| <a href="#FAILURE_NONE"></a> FAILURE_NONE                       | `uint16` | 0     |
| <a href="#FAILURE_ROLL"></a> FAILURE_ROLL                       | `uint16` | 1     | (1 << 0)    |
| <a href="#FAILURE_PITCH"></a> FAILURE_PITCH                     | `uint16` | 2     | (1 << 1)    |
| <a href="#FAILURE_ALT"></a> FAILURE_ALT                         | `uint16` | 4     | (1 << 2)    |
| <a href="#FAILURE_EXT"></a> FAILURE_EXT                         | `uint16` | 8     | (1 << 3)    |
| <a href="#FAILURE_ARM_ESC"></a> FAILURE_ARM_ESC                 | `uint16` | 16    | (1 << 4)    |
| <a href="#FAILURE_BATTERY"></a> FAILURE_BATTERY                 | `uint16` | 32    | (1 << 5)    |
| <a href="#FAILURE_IMBALANCED_PROP"></a> FAILURE_IMBALANCED_PROP | `uint16` | 64    | (1 << 6)    |
| <a href="#FAILURE_MOTOR"></a> FAILURE_MOTOR                     | `uint16` | 128   | (1 << 7)    |

### VEHICLE_TYPE {#VEHICLE_TYPE}

| Name                                                              | Type    | Value | Description |
| ----------------------------------------------------------------- | ------- | ----- | ----------- |
| <a href="#VEHICLE_TYPE_UNSPECIFIED"></a> VEHICLE_TYPE_UNSPECIFIED | `uint8` | 0     |
| <a href="#VEHICLE_TYPE_ROTARY_WING"></a> VEHICLE_TYPE_ROTARY_WING | `uint8` | 1     |
| <a href="#VEHICLE_TYPE_FIXED_WING"></a> VEHICLE_TYPE_FIXED_WING   | `uint8` | 2     |
| <a href="#VEHICLE_TYPE_ROVER"></a> VEHICLE_TYPE_ROVER             | `uint8` | 3     |

### FAILSAFE_DEFER_STATE {#FAILSAFE_DEFER_STATE}

| Name                                                                                    | Type    | Value | Description                                      |
| --------------------------------------------------------------------------------------- | ------- | ----- | ------------------------------------------------ |
| <a href="#FAILSAFE_DEFER_STATE_DISABLED"></a> FAILSAFE_DEFER_STATE_DISABLED             | `uint8` | 0     |
| <a href="#FAILSAFE_DEFER_STATE_ENABLED"></a> FAILSAFE_DEFER_STATE_ENABLED               | `uint8` | 1     |
| <a href="#FAILSAFE_DEFER_STATE_WOULD_FAILSAFE"></a> FAILSAFE_DEFER_STATE_WOULD_FAILSAFE | `uint8` | 2     | Failsafes deferred, but would trigger a failsafe |

## Constants

| Name                                                                                      | Type     | Value | Description |
| ----------------------------------------------------------------------------------------- | -------- | ----- | ----------- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION                                           | `uint32` | 1     |
| <a href="#ARMING_STATE_DISARMED"></a> ARMING_STATE_DISARMED                               | `uint8`  | 1     |
| <a href="#ARMING_STATE_ARMED"></a> ARMING_STATE_ARMED                                     | `uint8`  | 2     |
| <a href="#ARM_DISARM_REASON_STICK_GESTURE"></a> ARM_DISARM_REASON_STICK_GESTURE           | `uint8`  | 1     |
| <a href="#ARM_DISARM_REASON_RC_SWITCH"></a> ARM_DISARM_REASON_RC_SWITCH                   | `uint8`  | 2     |
| <a href="#ARM_DISARM_REASON_COMMAND_INTERNAL"></a> ARM_DISARM_REASON_COMMAND_INTERNAL     | `uint8`  | 3     |
| <a href="#ARM_DISARM_REASON_COMMAND_EXTERNAL"></a> ARM_DISARM_REASON_COMMAND_EXTERNAL     | `uint8`  | 4     |
| <a href="#ARM_DISARM_REASON_MISSION_START"></a> ARM_DISARM_REASON_MISSION_START           | `uint8`  | 5     |
| <a href="#ARM_DISARM_REASON_LANDING"></a> ARM_DISARM_REASON_LANDING                       | `uint8`  | 6     |
| <a href="#ARM_DISARM_REASON_PREFLIGHT_INACTION"></a> ARM_DISARM_REASON_PREFLIGHT_INACTION | `uint8`  | 7     |
| <a href="#ARM_DISARM_REASON_KILL_SWITCH"></a> ARM_DISARM_REASON_KILL_SWITCH               | `uint8`  | 8     |
| <a href="#ARM_DISARM_REASON_RC_BUTTON"></a> ARM_DISARM_REASON_RC_BUTTON                   | `uint8`  | 13    |
| <a href="#ARM_DISARM_REASON_FAILSAFE"></a> ARM_DISARM_REASON_FAILSAFE                     | `uint8`  | 14    |
| <a href="#HIL_STATE_OFF"></a> HIL_STATE_OFF                                               | `uint8`  | 0     |
| <a href="#HIL_STATE_ON"></a> HIL_STATE_ON                                                 | `uint8`  | 1     |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/px4_msgs_old/msg/VehicleStatusV1.msg)

::: details Click here to see original file

```c
# Encodes the system state of the vehicle published by commander

uint32 MESSAGE_VERSION = 1

uint64 timestamp # [us] Time since system start

uint64 armed_time # [us] Arming timestamp
uint64 takeoff_time # [us] Takeoff timestamp

uint8 arming_state
uint8 ARMING_STATE_DISARMED = 1
uint8 ARMING_STATE_ARMED = 2

uint8 latest_arming_reason
uint8 latest_disarming_reason
uint8 ARM_DISARM_REASON_STICK_GESTURE = 1
uint8 ARM_DISARM_REASON_RC_SWITCH = 2
uint8 ARM_DISARM_REASON_COMMAND_INTERNAL = 3
uint8 ARM_DISARM_REASON_COMMAND_EXTERNAL = 4
uint8 ARM_DISARM_REASON_MISSION_START = 5
uint8 ARM_DISARM_REASON_LANDING = 6
uint8 ARM_DISARM_REASON_PREFLIGHT_INACTION = 7
uint8 ARM_DISARM_REASON_KILL_SWITCH = 8
uint8 ARM_DISARM_REASON_RC_BUTTON = 13
uint8 ARM_DISARM_REASON_FAILSAFE = 14

uint64 nav_state_timestamp # Time when current nav_state activated

uint8 nav_state_user_intention # Mode that the user selected (might be different from nav_state in a failsafe situation)

uint8 nav_state # [@enum NAVIGATION_STATE] Currently active mode
uint8 NAVIGATION_STATE_MANUAL = 0 # Manual mode
uint8 NAVIGATION_STATE_ALTCTL = 1 # Altitude control mode
uint8 NAVIGATION_STATE_POSCTL = 2 # Position control mode
uint8 NAVIGATION_STATE_AUTO_MISSION = 3 # Auto mission mode
uint8 NAVIGATION_STATE_AUTO_LOITER = 4 # Auto loiter mode
uint8 NAVIGATION_STATE_AUTO_RTL = 5 # Auto return to launch mode
uint8 NAVIGATION_STATE_POSITION_SLOW = 6
uint8 NAVIGATION_STATE_FREE5 = 7
uint8 NAVIGATION_STATE_ALTITUDE_CRUISE = 8 # Altitude with Cruise mode
uint8 NAVIGATION_STATE_FREE3 = 9
uint8 NAVIGATION_STATE_ACRO = 10 # Acro mode
uint8 NAVIGATION_STATE_FREE2 = 11
uint8 NAVIGATION_STATE_DESCEND = 12 # Descend mode (no position control)
uint8 NAVIGATION_STATE_TERMINATION = 13 # Termination mode
uint8 NAVIGATION_STATE_OFFBOARD = 14
uint8 NAVIGATION_STATE_STAB = 15 # Stabilized mode
uint8 NAVIGATION_STATE_FREE1 = 16
uint8 NAVIGATION_STATE_AUTO_TAKEOFF = 17 # Takeoff
uint8 NAVIGATION_STATE_AUTO_LAND = 18 # Land
uint8 NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 19 # Auto Follow
uint8 NAVIGATION_STATE_AUTO_PRECLAND = 20 # Precision land with landing target
uint8 NAVIGATION_STATE_ORBIT = 21 # Orbit in a circle
uint8 NAVIGATION_STATE_AUTO_VTOL_TAKEOFF = 22 # Takeoff, transition, establish loiter
uint8 NAVIGATION_STATE_EXTERNAL1 = 23
uint8 NAVIGATION_STATE_EXTERNAL2 = 24
uint8 NAVIGATION_STATE_EXTERNAL3 = 25
uint8 NAVIGATION_STATE_EXTERNAL4 = 26
uint8 NAVIGATION_STATE_EXTERNAL5 = 27
uint8 NAVIGATION_STATE_EXTERNAL6 = 28
uint8 NAVIGATION_STATE_EXTERNAL7 = 29
uint8 NAVIGATION_STATE_EXTERNAL8 = 30
uint8 NAVIGATION_STATE_MAX = 31

uint8 executor_in_charge # [-] Current mode executor in charge (0=Autopilot)

uint32 valid_nav_states_mask # [-] Bitmask for all valid nav_state values
uint32 can_set_nav_states_mask # [-] Bitmask for all modes that a user can select

# Bitmask of detected failures
uint16 failure_detector_status # [@enum FAILURE]
uint16 FAILURE_NONE = 0
uint16 FAILURE_ROLL = 1 # (1 << 0)
uint16 FAILURE_PITCH = 2 # (1 << 1)
uint16 FAILURE_ALT = 4 # (1 << 2)
uint16 FAILURE_EXT = 8 # (1 << 3)
uint16 FAILURE_ARM_ESC = 16 # (1 << 4)
uint16 FAILURE_BATTERY = 32 # (1 << 5)
uint16 FAILURE_IMBALANCED_PROP = 64 # (1 << 6)
uint16 FAILURE_MOTOR = 128 # (1 << 7)

uint8 hil_state # [enum HIL_STATE]
uint8 HIL_STATE_OFF = 0
uint8 HIL_STATE_ON = 1

# Current vehicle locomotion method. A vehicle can have different methods (e.g. VTOL transitions from RW to FW method)
uint8 vehicle_type # [@enum VEHICLE_TYPE]
uint8 VEHICLE_TYPE_UNSPECIFIED = 0
uint8 VEHICLE_TYPE_ROTARY_WING = 1
uint8 VEHICLE_TYPE_FIXED_WING = 2
uint8 VEHICLE_TYPE_ROVER = 3

uint8 FAILSAFE_DEFER_STATE_DISABLED = 0
uint8 FAILSAFE_DEFER_STATE_ENABLED = 1
uint8 FAILSAFE_DEFER_STATE_WOULD_FAILSAFE = 2 # Failsafes deferred, but would trigger a failsafe

bool failsafe # true if system is in failsafe state (e.g.:RTL, Hover, Terminate, ...)
bool failsafe_and_user_took_over # true if system is in failsafe state but the user took over control
uint8 failsafe_defer_state # [@enum FAILSAFE_DEFER_STATE]

# Link loss
bool gcs_connection_lost # datalink to GCS lost
uint8 gcs_connection_lost_counter # counts unique GCS connection lost events
bool high_latency_data_link_lost # Set to true if the high latency data link (eg. RockBlock Iridium 9603 telemetry module) is lost

# VTOL flags
bool is_vtol # True if the system is VTOL capable
bool is_vtol_tailsitter # True if the system performs a 90° pitch down rotation during transition from MC to FW
bool in_transition_mode # True if VTOL is doing a transition
bool in_transition_to_fw # True if VTOL is doing a transition from MC to FW

# MAVLink identification
uint8 system_type # system type, contains mavlink MAV_TYPE
uint8 system_id # system id, contains MAVLink's system ID field
uint8 component_id # subsystem / component id, contains MAVLink's component ID field

bool safety_button_available # Set to true if a safety button is connected
bool safety_off # Set to true if safety is off

bool power_input_valid # Set if input power is valid
bool usb_connected # Set to true (never cleared) once telemetry received from usb link

bool open_drone_id_system_present
bool open_drone_id_system_healthy

bool parachute_system_present
bool parachute_system_healthy

bool rc_calibration_in_progress
bool calibration_enabled

bool pre_flight_checks_pass # true if all checks necessary to arm pass
```

:::
