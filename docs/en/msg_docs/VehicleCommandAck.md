---
pageClass: is-wide-page
---

# VehicleCommandAck (UORB message)

Vehicle Command Acknowledgement uORB message.

Used for acknowledging the vehicle command being received.
Follows the MAVLink COMMAND_ACK message definition

**TOPICS:** vehicle_command_ack

## Fields

| Name             | Type     | Unit [Frame]                | Range/Enum                                | Description                                                                                                                                 |
| ---------------- | -------- | --------------------------- | ----------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp        | `uint64` | us                          |                                           | time since system start                                                                                                                     |
| command          | `uint32` |                             |                                           | Command that is being acknowledged                                                                                                          |
| result           | `uint8`  |                             | [VEHICLE_CMD_RESULT](#VEHICLE_CMD_RESULT) | Command result                                                                                                                              |
| result_param1    | `uint8`  |                             |                                           | Can be set with the reason why the command was denied, or the progress percentage when result is MAV_RESULT_IN_PROGRESS (%)                 |
| result_param2    | `int32`  | enum ARM_AUTH_DENIED_REASON |                                           | Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it to be denied, or what ARM_AUTH_DENIED_REASON |
| target_system    | `uint8`  |                             |                                           | Target system                                                                                                                               |
| target_component | `uint16` |                             |                                           | Target component / mode executor                                                                                                            |
| from_external    | `bool`   |                             |                                           | Indicates if the command came from an external source                                                                                       |

## Enums

### VEHICLE_CMD_RESULT {#VEHICLE_CMD_RESULT}

| Name                                                                                              | Type    | Value | Description                                          |
| ------------------------------------------------------------------------------------------------- | ------- | ----- | ---------------------------------------------------- |
| <a href="#VEHICLE_CMD_RESULT_ACCEPTED"></a> VEHICLE_CMD_RESULT_ACCEPTED                           | `uint8` | 0     | Command ACCEPTED and EXECUTED                        |
| <a href="#VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED"></a> VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED   | `uint8` | 1     | Command TEMPORARY REJECTED/DENIED                    |
| <a href="#VEHICLE_CMD_RESULT_DENIED"></a> VEHICLE_CMD_RESULT_DENIED                               | `uint8` | 2     | Command PERMANENTLY DENIED                           |
| <a href="#VEHICLE_CMD_RESULT_UNSUPPORTED"></a> VEHICLE_CMD_RESULT_UNSUPPORTED                     | `uint8` | 3     | Command UNKNOWN/UNSUPPORTED                          |
| <a href="#VEHICLE_CMD_RESULT_FAILED"></a> VEHICLE_CMD_RESULT_FAILED                               | `uint8` | 4     | Command executed, but failed                         |
| <a href="#VEHICLE_CMD_RESULT_IN_PROGRESS"></a> VEHICLE_CMD_RESULT_IN_PROGRESS                     | `uint8` | 5     | Command being executed                               |
| <a href="#VEHICLE_CMD_RESULT_CANCELLED"></a> VEHICLE_CMD_RESULT_CANCELLED                         | `uint8` | 6     | Command Canceled                                     |
| <a href="#VEHICLE_CMD_RESULT_COMMAND_LONG_ONLY"></a> VEHICLE_CMD_RESULT_COMMAND_LONG_ONLY         | `uint8` | 7     | Command is only accepted when sent as a COMMAND_LONG |
| <a href="#VEHICLE_CMD_RESULT_COMMAND_INT_ONLY"></a> VEHICLE_CMD_RESULT_COMMAND_INT_ONLY           | `uint8` | 8     | Command is only accepted when sent as a COMMAND_INT  |
| <a href="#VEHICLE_CMD_RESULT_UNSUPPORTED_MAV_FRAME"></a> VEHICLE_CMD_RESULT_UNSUPPORTED_MAV_FRAME | `uint8` | 9     | Command does not support specified frame             |

## Constants

| Name                                                                                          | Type     | Value | Description |
| --------------------------------------------------------------------------------------------- | -------- | ----- | ----------- |
| <a id="#MESSAGE_VERSION"></a> MESSAGE_VERSION                                                 | `uint32` | 1     |
| <a id="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH                                               | `uint8`  | 8     |
| <a id="#ARM_AUTH_DENIED_REASON_GENERIC"></a> ARM_AUTH_DENIED_REASON_GENERIC                   | `uint16` | 0     |
| <a id="#ARM_AUTH_DENIED_REASON_NONE"></a> ARM_AUTH_DENIED_REASON_NONE                         | `uint16` | 1     |
| <a id="#ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT"></a> ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT | `uint16` | 2     |
| <a id="#ARM_AUTH_DENIED_REASON_TIMEOUT"></a> ARM_AUTH_DENIED_REASON_TIMEOUT                   | `uint16` | 3     |
| <a id="#ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE"></a> ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE   | `uint16` | 4     |
| <a id="#ARM_AUTH_DENIED_REASON_BAD_WEATHER"></a> ARM_AUTH_DENIED_REASON_BAD_WEATHER           | `uint16` | 5     |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/VehicleCommandAck.msg)

::: details Click here to see original file

```c
# Vehicle Command Acknowledgement uORB message.
#
# Used for acknowledging the vehicle command being received.
# Follows the MAVLink COMMAND_ACK message definition

uint32 MESSAGE_VERSION = 1

uint64 timestamp # [us] time since system start

uint8 ORB_QUEUE_LENGTH = 8

uint32 command # [-] Command that is being acknowledged

uint8 result # [@enum VEHICLE_CMD_RESULT] Command result
# VEHICLE_CMD_RESULT Result cases. Follows the MAVLink MAV_RESULT enum definition
uint8 VEHICLE_CMD_RESULT_ACCEPTED = 0 # Command ACCEPTED and EXECUTED
uint8 VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED = 1 # Command TEMPORARY REJECTED/DENIED
uint8 VEHICLE_CMD_RESULT_DENIED = 2 # Command PERMANENTLY DENIED
uint8 VEHICLE_CMD_RESULT_UNSUPPORTED = 3 # Command UNKNOWN/UNSUPPORTED
uint8 VEHICLE_CMD_RESULT_FAILED = 4 # Command executed, but failed
uint8 VEHICLE_CMD_RESULT_IN_PROGRESS = 5 # Command being executed
uint8 VEHICLE_CMD_RESULT_CANCELLED = 6 # Command Canceled
uint8 VEHICLE_CMD_RESULT_COMMAND_LONG_ONLY = 7 # Command is only accepted when sent as a COMMAND_LONG
uint8 VEHICLE_CMD_RESULT_COMMAND_INT_ONLY = 8 # Command is only accepted when sent as a COMMAND_INT
uint8 VEHICLE_CMD_RESULT_UNSUPPORTED_MAV_FRAME = 9 # Command does not support specified frame

uint8 result_param1 # [-] Can be set with the reason why the command was denied, or the progress percentage when result is MAV_RESULT_IN_PROGRESS (%)

int32 result_param2 # [enum ARM_AUTH_DENIED_REASON] Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it to be denied, or what ARM_AUTH_DENIED_REASON
# Arming denied specific cases
uint16 ARM_AUTH_DENIED_REASON_GENERIC = 0
uint16 ARM_AUTH_DENIED_REASON_NONE = 1
uint16 ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT = 2
uint16 ARM_AUTH_DENIED_REASON_TIMEOUT = 3
uint16 ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE = 4
uint16 ARM_AUTH_DENIED_REASON_BAD_WEATHER = 5

uint8 target_system # [-] Target system
uint16 target_component # Target component / mode executor

bool from_external # Indicates if the command came from an external source
```

:::
