---
pageClass: is-wide-page
---

# Ping (UORB message)

**TOPICS:** ping

## Fields

| Name                                            | Type      | Unit [Frame] | Range/Enum | Description                            |
| ----------------------------------------------- | --------- | ------------ | ---------- | -------------------------------------- |
| <a id="fld_timestamp"></a>timestamp             | `uint64`  |              |            | time since system start (microseconds) |
| <a id="fld_ping_time"></a>ping_time             | `uint64`  |              |            | Timestamp of the ping packet           |
| <a id="fld_ping_sequence"></a>ping_sequence     | `uint32`  |              |            | Sequence number of the ping packet     |
| <a id="fld_dropped_packets"></a>dropped_packets | `uint32`  |              |            | Number of dropped ping packets         |
| <a id="fld_rtt_ms"></a>rtt_ms                   | `float32` |              |            | Round trip time (in ms)                |
| <a id="fld_system_id"></a>system_id             | `uint8`   |              |            | System ID of the remote system         |
| <a id="fld_component_id"></a>component_id       | `uint8`   |              |            | Component ID of the remote system      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/Ping.msg)

::: details Click here to see original file

```c
uint64 timestamp			# time since system start (microseconds)
uint64 ping_time			# Timestamp of the ping packet
uint32 ping_sequence		# Sequence number of the ping packet
uint32 dropped_packets		# Number of dropped ping packets
float32 rtt_ms				# Round trip time (in ms)
uint8 system_id				# System ID of the remote system
uint8 component_id			# Component ID of the remote system
```

:::
