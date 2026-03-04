---
pageClass: is-wide-page
---

# DronecanNodeStatus (UORB message)

**TOPICS:** dronecan_nodestatus

## Fields

| 参数名                                                                                        | 类型       | Unit [Frame] | Range/Enum | 描述                                                        |
| ------------------------------------------------------------------------------------------ | -------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                                                                                  | `uint64` |                                                                  |            | time since system start (microseconds) |
| node_id                                                               | `uint16` |                                                                  |            | The node ID which this data comes from                    |
| uptime_sec                                                            | `uint32` |                                                                  |            | Node uptime                                               |
| health                                                                                     | `uint8`  |                                                                  |            |                                                           |
| mode                                                                                       | `uint8`  |                                                                  |            |                                                           |
| sub_mode                                                              | `uint8`  |                                                                  |            |                                                           |
| vendor_specific_status_code | `uint16` |                                                                  |            |                                                           |

## Constants

| 参数名                                                                                                 | 类型      | 值 | 描述                                                                                                |
| --------------------------------------------------------------------------------------------------- | ------- | - | ------------------------------------------------------------------------------------------------- |
| <a href="#HEALTH_OK"></a> HEALTH_OK                                            | `uint8` | 0 | The node is functioning properly.                                                 |
| <a href="#HEALTH_WARNING"></a> HEALTH_WARNING                                  | `uint8` | 1 | A critical parameter went out of range or the node encountered a minor failure.   |
| <a href="#HEALTH_ERROR"></a> HEALTH_ERROR                                      | `uint8` | 2 | The node encountered a major failure.                                             |
| <a href="#HEALTH_CRITICAL"></a> HEALTH_CRITICAL                                | `uint8` | 3 | The node suffered a fatal malfunction.                                            |
| <a href="#MODE_OPERATIONAL"></a> MODE_OPERATIONAL                              | `uint8` | 0 | Normal operating mode.                                                            |
| <a href="#MODE_INITIALIZATION"></a> MODE_INITIALIZATION                        | `uint8` | 1 | Initialization is in progress; this mode is entered immediately after startup.    |
| <a href="#MODE_MAINTENANCE"></a> MODE_MAINTENANCE                              | `uint8` | 2 | E.g. calibration, the bootloader is running, etc. |
| <a href="#MODE_SOFTWARE_UPDATE"></a> MODE_SOFTWARE_UPDATE | `uint8` | 3 | New software/firmware is being loaded.                                            |
| <a href="#MODE_OFFLINE"></a> MODE_OFFLINE                                      | `uint8` | 7 | The node is no longer available.                                                  |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/DronecanNodeStatus.msg)

:::details
Click here to see original file

```c
uint64 timestamp					# time since system start (microseconds)

uint16 node_id						# The node ID which this data comes from

# From the uavcan.protocol.NodeStatus message
uint32 uptime_sec					# Node uptime

#
# Abstract node health.
#
uint8 HEALTH_OK         = 0     # The node is functioning properly.
uint8 HEALTH_WARNING    = 1     # A critical parameter went out of range or the node encountered a minor failure.
uint8 HEALTH_ERROR      = 2     # The node encountered a major failure.
uint8 HEALTH_CRITICAL   = 3     # The node suffered a fatal malfunction.
uint8 health

#
# Current mode.
#
# Mode OFFLINE can be actually reported by the node to explicitly inform other network
# participants that the sending node is about to shutdown. In this case other nodes will not
# have to wait OFFLINE_TIMEOUT_MS before they detect that the node is no longer available.
#
# Reserved values can be used in future revisions of the specification.
#
uint8 MODE_OPERATIONAL      = 0         # Normal operating mode.
uint8 MODE_INITIALIZATION   = 1         # Initialization is in progress; this mode is entered immediately after startup.
uint8 MODE_MAINTENANCE      = 2         # E.g. calibration, the bootloader is running, etc.
uint8 MODE_SOFTWARE_UPDATE  = 3         # New software/firmware is being loaded.
uint8 MODE_OFFLINE          = 7         # The node is no longer available.
uint8 mode

#
# Not used currently, keep zero when publishing, ignore when receiving.
#
uint8 sub_mode

#
# Optional, vendor-specific node status code, e.g. a fault code or a status bitmask.
#
uint16 vendor_specific_status_code
```

:::
