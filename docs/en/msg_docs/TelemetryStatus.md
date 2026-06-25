---
pageClass: is-wide-page
---

# TelemetryStatus (UORB message)

**TOPICS:** telemetry_status

## Fields

| Name                                                                                    | Type      | Unit [Frame] | Range/Enum | Description                               |
| --------------------------------------------------------------------------------------- | --------- | ------------ | ---------- | ----------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                                                     | `uint64`  |              |            | time since system start (microseconds)    |
| <a id="fld_type"></a>type                                                               | `uint8`   |              |            | type of the radio hardware (LINK*TYPE*\*) |
| <a id="fld_mode"></a>mode                                                               | `uint8`   |              |            |
| <a id="fld_flow_control"></a>flow_control                                               | `bool`    |              |            |
| <a id="fld_forwarding"></a>forwarding                                                   | `bool`    |              |            |
| <a id="fld_mavlink_v2"></a>mavlink_v2                                                   | `bool`    |              |            |
| <a id="fld_ftp"></a>ftp                                                                 | `bool`    |              |            |
| <a id="fld_streams"></a>streams                                                         | `uint8`   |              |            |
| <a id="fld_data_rate"></a>data_rate                                                     | `float32` |              |            | configured maximum data rate (Bytes/s)    |
| <a id="fld_rate_multiplier"></a>rate_multiplier                                         | `float32` |              |            |
| <a id="fld_tx_rate_avg"></a>tx_rate_avg                                                 | `float32` |              |            | transmit rate average (Bytes/s)           |
| <a id="fld_tx_error_rate_avg"></a>tx_error_rate_avg                                     | `float32` |              |            | transmit error rate average (Bytes/s)     |
| <a id="fld_tx_message_count"></a>tx_message_count                                       | `uint32`  |              |            | total message sent count                  |
| <a id="fld_tx_buffer_overruns"></a>tx_buffer_overruns                                   | `uint32`  |              |            | number of TX buffer overruns              |
| <a id="fld_rx_rate_avg"></a>rx_rate_avg                                                 | `float32` |              |            | transmit rate average (Bytes/s)           |
| <a id="fld_rx_message_count"></a>rx_message_count                                       | `uint32`  |              |            | count of total messages received          |
| <a id="fld_rx_message_lost_count"></a>rx_message_lost_count                             | `uint32`  |              |            |
| <a id="fld_rx_buffer_overruns"></a>rx_buffer_overruns                                   | `uint32`  |              |            | number of RX buffer overruns              |
| <a id="fld_rx_parse_errors"></a>rx_parse_errors                                         | `uint32`  |              |            | number of parse errors                    |
| <a id="fld_rx_packet_drop_count"></a>rx_packet_drop_count                               | `uint32`  |              |            | number of packet drops                    |
| <a id="fld_rx_message_lost_rate"></a>rx_message_lost_rate                               | `float32` |              |            |
| <a id="fld_heartbeat_type_antenna_tracker"></a>heartbeat_type_antenna_tracker           | `bool`    |              |            | MAV_TYPE_ANTENNA_TRACKER                  |
| <a id="fld_heartbeat_type_gcs"></a>heartbeat_type_gcs                                   | `bool`    |              |            | MAV_TYPE_GCS                              |
| <a id="fld_heartbeat_type_onboard_controller"></a>heartbeat_type_onboard_controller     | `bool`    |              |            | MAV_TYPE_ONBOARD_CONTROLLER               |
| <a id="fld_heartbeat_type_gimbal"></a>heartbeat_type_gimbal                             | `bool`    |              |            | MAV_TYPE_GIMBAL                           |
| <a id="fld_heartbeat_type_adsb"></a>heartbeat_type_adsb                                 | `bool`    |              |            | MAV_TYPE_ADSB                             |
| <a id="fld_heartbeat_type_flarm"></a>heartbeat_type_flarm                               | `bool`    |              |            | MAV_TYPE_FLARM                            |
| <a id="fld_heartbeat_type_camera"></a>heartbeat_type_camera                             | `bool`    |              |            | MAV_TYPE_CAMERA                           |
| <a id="fld_heartbeat_type_parachute"></a>heartbeat_type_parachute                       | `bool`    |              |            | MAV_TYPE_PARACHUTE                        |
| <a id="fld_heartbeat_type_open_drone_id"></a>heartbeat_type_open_drone_id               | `bool`    |              |            | MAV_TYPE_ODID                             |
| <a id="fld_heartbeat_component_telemetry_radio"></a>heartbeat_component_telemetry_radio | `bool`    |              |            | MAV_COMP_ID_TELEMETRY_RADIO               |
| <a id="fld_heartbeat_component_log"></a>heartbeat_component_log                         | `bool`    |              |            | MAV_COMP_ID_LOG                           |
| <a id="fld_heartbeat_component_osd"></a>heartbeat_component_osd                         | `bool`    |              |            | MAV_COMP_ID_OSD                           |
| <a id="fld_heartbeat_component_vio"></a>heartbeat_component_vio                         | `bool`    |              |            | MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY      |
| <a id="fld_heartbeat_component_pairing_manager"></a>heartbeat_component_pairing_manager | `bool`    |              |            | MAV_COMP_ID_PAIRING_MANAGER               |
| <a id="fld_heartbeat_component_udp_bridge"></a>heartbeat_component_udp_bridge           | `bool`    |              |            | MAV_COMP_ID_UDP_BRIDGE                    |
| <a id="fld_heartbeat_component_uart_bridge"></a>heartbeat_component_uart_bridge         | `bool`    |              |            | MAV_COMP_ID_UART_BRIDGE                   |
| <a id="fld_open_drone_id_system_healthy"></a>open_drone_id_system_healthy               | `bool`    |              |            |
| <a id="fld_parachute_system_healthy"></a>parachute_system_healthy                       | `bool`    |              |            |

## Constants

| Name                                                              | Type     | Value   | Description                                     |
| ----------------------------------------------------------------- | -------- | ------- | ----------------------------------------------- |
| <a id="#LINK_TYPE_GENERIC"></a> LINK_TYPE_GENERIC                 | `uint8`  | 0       |
| <a id="#LINK_TYPE_UBIQUITY_BULLET"></a> LINK_TYPE_UBIQUITY_BULLET | `uint8`  | 1       |
| <a id="#LINK_TYPE_WIRE"></a> LINK_TYPE_WIRE                       | `uint8`  | 2       |
| <a id="#LINK_TYPE_USB"></a> LINK_TYPE_USB                         | `uint8`  | 3       |
| <a id="#LINK_TYPE_IRIDIUM"></a> LINK_TYPE_IRIDIUM                 | `uint8`  | 4       |
| <a id="#HEARTBEAT_TIMEOUT_US"></a> HEARTBEAT_TIMEOUT_US           | `uint64` | 2500000 | Heartbeat timeout (tolerate missing 1 + jitter) |

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
