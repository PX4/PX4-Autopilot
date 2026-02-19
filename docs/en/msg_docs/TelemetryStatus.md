---
pageClass: is-wide-page
---

# TelemetryStatus (UORB message)

**TOPICS:** telemetry_status

## Fields

| Name                                | Type      | Unit [Frame] | Range/Enum | Description                               |
| ----------------------------------- | --------- | ------------ | ---------- | ----------------------------------------- |
| timestamp                           | `uint64`  |              |            | time since system start (microseconds)    |
| type                                | `uint8`   |              |            | type of the radio hardware (LINK*TYPE*\*) |
| mode                                | `uint8`   |              |            |
| flow_control                        | `bool`    |              |            |
| forwarding                          | `bool`    |              |            |
| mavlink_v2                          | `bool`    |              |            |
| ftp                                 | `bool`    |              |            |
| streams                             | `uint8`   |              |            |
| data_rate                           | `float32` |              |            | configured maximum data rate (Bytes/s)    |
| rate_multiplier                     | `float32` |              |            |
| tx_rate_avg                         | `float32` |              |            | transmit rate average (Bytes/s)           |
| tx_error_rate_avg                   | `float32` |              |            | transmit error rate average (Bytes/s)     |
| tx_message_count                    | `uint32`  |              |            | total message sent count                  |
| tx_buffer_overruns                  | `uint32`  |              |            | number of TX buffer overruns              |
| rx_rate_avg                         | `float32` |              |            | transmit rate average (Bytes/s)           |
| rx_message_count                    | `uint32`  |              |            | count of total messages received          |
| rx_message_lost_count               | `uint32`  |              |            |
| rx_buffer_overruns                  | `uint32`  |              |            | number of RX buffer overruns              |
| rx_parse_errors                     | `uint32`  |              |            | number of parse errors                    |
| rx_packet_drop_count                | `uint32`  |              |            | number of packet drops                    |
| rx_message_lost_rate                | `float32` |              |            |
| heartbeat_type_antenna_tracker      | `bool`    |              |            | MAV_TYPE_ANTENNA_TRACKER                  |
| heartbeat_type_gcs                  | `bool`    |              |            | MAV_TYPE_GCS                              |
| heartbeat_type_onboard_controller   | `bool`    |              |            | MAV_TYPE_ONBOARD_CONTROLLER               |
| heartbeat_type_gimbal               | `bool`    |              |            | MAV_TYPE_GIMBAL                           |
| heartbeat_type_adsb                 | `bool`    |              |            | MAV_TYPE_ADSB                             |
| heartbeat_type_flarm                | `bool`    |              |            | MAV_TYPE_FLARM                            |
| heartbeat_type_camera               | `bool`    |              |            | MAV_TYPE_CAMERA                           |
| heartbeat_type_parachute            | `bool`    |              |            | MAV_TYPE_PARACHUTE                        |
| heartbeat_type_open_drone_id        | `bool`    |              |            | MAV_TYPE_ODID                             |
| heartbeat_component_telemetry_radio | `bool`    |              |            | MAV_COMP_ID_TELEMETRY_RADIO               |
| heartbeat_component_log             | `bool`    |              |            | MAV_COMP_ID_LOG                           |
| heartbeat_component_osd             | `bool`    |              |            | MAV_COMP_ID_OSD                           |
| heartbeat_component_vio             | `bool`    |              |            | MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY      |
| heartbeat_component_pairing_manager | `bool`    |              |            | MAV_COMP_ID_PAIRING_MANAGER               |
| heartbeat_component_udp_bridge      | `bool`    |              |            | MAV_COMP_ID_UDP_BRIDGE                    |
| heartbeat_component_uart_bridge     | `bool`    |              |            | MAV_COMP_ID_UART_BRIDGE                   |
| open_drone_id_system_healthy        | `bool`    |              |            |
| parachute_system_healthy            | `bool`    |              |            |

## Constants

| Name                                                                | Type     | Value   | Description                                     |
| ------------------------------------------------------------------- | -------- | ------- | ----------------------------------------------- |
| <a href="#LINK_TYPE_GENERIC"></a> LINK_TYPE_GENERIC                 | `uint8`  | 0       |
| <a href="#LINK_TYPE_UBIQUITY_BULLET"></a> LINK_TYPE_UBIQUITY_BULLET | `uint8`  | 1       |
| <a href="#LINK_TYPE_WIRE"></a> LINK_TYPE_WIRE                       | `uint8`  | 2       |
| <a href="#LINK_TYPE_USB"></a> LINK_TYPE_USB                         | `uint8`  | 3       |
| <a href="#LINK_TYPE_IRIDIUM"></a> LINK_TYPE_IRIDIUM                 | `uint8`  | 4       |
| <a href="#HEARTBEAT_TIMEOUT_US"></a> HEARTBEAT_TIMEOUT_US           | `uint64` | 2500000 | Heartbeat timeout (tolerate missing 1 + jitter) |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/TelemetryStatus.msg)

::: details Click here to see original file

```c
uint8 LINK_TYPE_GENERIC = 0
uint8 LINK_TYPE_UBIQUITY_BULLET = 1
uint8 LINK_TYPE_WIRE = 2
uint8 LINK_TYPE_USB = 3
uint8 LINK_TYPE_IRIDIUM	= 4

uint64 timestamp			# time since system start (microseconds)

uint8 type				#  type of the radio hardware (LINK_TYPE_*)

uint8 mode

bool flow_control
bool forwarding
bool mavlink_v2
bool ftp

uint8 streams

float32 data_rate                       # configured maximum data rate (Bytes/s)

float32 rate_multiplier

float32 tx_rate_avg                     # transmit rate average (Bytes/s)
float32 tx_error_rate_avg               # transmit error rate average (Bytes/s)
uint32 tx_message_count                 # total message sent count
uint32 tx_buffer_overruns               # number of TX buffer overruns

float32 rx_rate_avg                     # transmit rate average (Bytes/s)
uint32 rx_message_count                 # count of total messages received
uint32 rx_message_lost_count
uint32 rx_buffer_overruns               # number of RX buffer overruns
uint32 rx_parse_errors                  # number of parse errors
uint32 rx_packet_drop_count             # number of packet drops
float32 rx_message_lost_rate


uint64 HEARTBEAT_TIMEOUT_US = 2500000       # Heartbeat timeout (tolerate missing 1 + jitter)

# Heartbeats per type
bool heartbeat_type_antenna_tracker         # MAV_TYPE_ANTENNA_TRACKER
bool heartbeat_type_gcs                     # MAV_TYPE_GCS
bool heartbeat_type_onboard_controller      # MAV_TYPE_ONBOARD_CONTROLLER
bool heartbeat_type_gimbal                  # MAV_TYPE_GIMBAL
bool heartbeat_type_adsb                    # MAV_TYPE_ADSB
bool heartbeat_type_flarm                   # MAV_TYPE_FLARM
bool heartbeat_type_camera                  # MAV_TYPE_CAMERA
bool heartbeat_type_parachute               # MAV_TYPE_PARACHUTE
bool heartbeat_type_open_drone_id           # MAV_TYPE_ODID

# Heartbeats per component
bool heartbeat_component_telemetry_radio    # MAV_COMP_ID_TELEMETRY_RADIO
bool heartbeat_component_log                # MAV_COMP_ID_LOG
bool heartbeat_component_osd                # MAV_COMP_ID_OSD
bool heartbeat_component_vio                # MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
bool heartbeat_component_pairing_manager    # MAV_COMP_ID_PAIRING_MANAGER
bool heartbeat_component_udp_bridge         # MAV_COMP_ID_UDP_BRIDGE
bool heartbeat_component_uart_bridge        # MAV_COMP_ID_UART_BRIDGE

# Misc component health
bool open_drone_id_system_healthy
bool parachute_system_healthy
```

:::
