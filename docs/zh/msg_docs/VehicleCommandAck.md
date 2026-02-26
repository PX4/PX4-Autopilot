---
pageClass: is-wide-page
---

# VehicleCommandAck (UORB message)

Vehicle Command Ackonwledgement uORB message. Used for acknowledging the vehicle command being received. Follows the MAVLink COMMAND_ACK message definition.

**TOPICS:** vehicle_commandack

## Fields

| 参数名                                   | 类型       | Unit [Frame] | Range/Enum | 描述                                                                                                                                                                                                                                                                      |
| ------------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                             | `uint64` |                                                                  |            | time since system start (microseconds)                                                                                                                                                                                                               |
| command                               | `uint32` |                                                                  |            | Command that is being acknowledged                                                                                                                                                                                                                                      |
| result                                | `uint8`  |                                                                  |            | Command result                                                                                                                                                                                                                                                          |
| result_param1    | `uint8`  |                                                                  |            | Also used as progress[%], it can be set with the reason why the command was denied, or the progress percentage when result is MAV_RESULT_IN_PROGRESS |
| result_param2    | `int32`  |                                                                  |            | Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it to be denied.                                                             |
| target_system    | `uint8`  |                                                                  |            |                                                                                                                                                                                                                                                                         |
| target_component | `uint16` |                                                                  |            | Target component / mode executor                                                                                                                                                                                                                                        |
| from_external    | `bool`   |                                                                  |            | Indicates if the command came from an external source                                                                                                                                                                                                                   |

## Constants

| 参数名                                                                                                                                                                                                                         | 类型       | 值 | 描述                                |
| --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------- | - | --------------------------------- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION                                                                                                                                                        | `uint32` | 0 |                                   |
| <a href="#VEHICLE_CMD_RESULT_ACCEPTED"></a> VEHICLE_CMD_RESULT_ACCEPTED                                                                                      | `uint8`  | 0 | Command ACCEPTED and EXECUTED     |
| <a href="#VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED"></a> VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED                                         | `uint8`  | 1 | Command TEMPORARY REJECTED/DENIED |
| <a href="#VEHICLE_CMD_RESULT_DENIED"></a> VEHICLE_CMD_RESULT_DENIED                                                                                          | `uint8`  | 2 | Command PERMANENTLY DENIED        |
| <a href="#VEHICLE_CMD_RESULT_UNSUPPORTED"></a> VEHICLE_CMD_RESULT_UNSUPPORTED                                                                                | `uint8`  | 3 | Command UNKNOWN/UNSUPPORTED       |
| <a href="#VEHICLE_CMD_RESULT_FAILED"></a> VEHICLE_CMD_RESULT_FAILED                                                                                          | `uint8`  | 4 | Command executed, but failed      |
| <a href="#VEHICLE_CMD_RESULT_IN_PROGRESS"></a> VEHICLE_CMD_RESULT_IN_PROGRESS                                                           | `uint8`  | 5 | Command being executed            |
| <a href="#VEHICLE_CMD_RESULT_CANCELLED"></a> VEHICLE_CMD_RESULT_CANCELLED                                                                                    | `uint8`  | 6 | Command Canceled                  |
| <a href="#ARM_AUTH_DENIED_REASON_GENERIC"></a> ARM_AUTH_DENIED_REASON_GENERIC                                                           | `uint16` | 0 |                                   |
| <a href="#ARM_AUTH_DENIED_REASON_NONE"></a> ARM_AUTH_DENIED_REASON_NONE                                                                 | `uint16` | 1 |                                   |
| <a href="#ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT"></a> ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT                    | `uint16` | 2 |                                   |
| <a href="#ARM_AUTH_DENIED_REASON_TIMEOUT"></a> ARM_AUTH_DENIED_REASON_TIMEOUT                                                           | `uint16` | 3 |                                   |
| <a href="#ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE"></a> ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE | `uint16` | 4 |                                   |
| <a href="#ARM_AUTH_DENIED_REASON_BAD_WEATHER"></a> ARM_AUTH_DENIED_REASON_BAD_WEATHER                              | `uint16` | 5 |                                   |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH                                                                                                                                 | `uint8`  | 8 |                                   |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/VehicleCommandAck.msg)

:::details
Click here to see original file

```c
# Vehicle Command Ackonwledgement uORB message.
# Used for acknowledging the vehicle command being received.
# Follows the MAVLink COMMAND_ACK message definition

uint32 MESSAGE_VERSION = 0

uint64 timestamp		# time since system start (microseconds)

# Result cases. This follows the MAVLink MAV_RESULT enum definition
uint8 VEHICLE_CMD_RESULT_ACCEPTED = 0			# Command ACCEPTED and EXECUTED |
uint8 VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED = 1	# Command TEMPORARY REJECTED/DENIED |
uint8 VEHICLE_CMD_RESULT_DENIED = 2			# Command PERMANENTLY DENIED |
uint8 VEHICLE_CMD_RESULT_UNSUPPORTED = 3		# Command UNKNOWN/UNSUPPORTED |
uint8 VEHICLE_CMD_RESULT_FAILED = 4			# Command executed, but failed |
uint8 VEHICLE_CMD_RESULT_IN_PROGRESS = 5		# Command being executed |
uint8 VEHICLE_CMD_RESULT_CANCELLED = 6			# Command Canceled

# Arming denied specific cases
uint16 ARM_AUTH_DENIED_REASON_GENERIC = 0
uint16 ARM_AUTH_DENIED_REASON_NONE = 1
uint16 ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT = 2
uint16 ARM_AUTH_DENIED_REASON_TIMEOUT = 3
uint16 ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE = 4
uint16 ARM_AUTH_DENIED_REASON_BAD_WEATHER = 5

uint8 ORB_QUEUE_LENGTH = 8

uint32 command						# Command that is being acknowledged
uint8 result						# Command result
uint8 result_param1					# Also used as progress[%], it can be set with the reason why the command was denied, or the progress percentage when result is MAV_RESULT_IN_PROGRESS
int32 result_param2					# Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it to be denied.
uint8 target_system
uint16 target_component 				# Target component / mode executor

bool from_external					# Indicates if the command came from an external source
```

:::
