---
pageClass: is-wide-page
---

# PowerMonitor (UORB message)

power monitor message.

**TOPICS:** power_monitor

## Fields

| Name                                | Type      | Unit [Frame] | Range/Enum | Description                            |
| ----------------------------------- | --------- | ------------ | ---------- | -------------------------------------- |
| <a id="fld_timestamp"></a>timestamp | `uint64`  |              |            | Time since system start (microseconds) |
| <a id="fld_voltage_v"></a>voltage_v | `float32` |              |            | Voltage in volts, 0 if unknown         |
| <a id="fld_current_a"></a>current_a | `float32` |              |            | Current in amperes, -1 if unknown      |
| <a id="fld_power_w"></a>power_w     | `float32` |              |            | power in watts, -1 if unknown          |
| <a id="fld_rconf"></a>rconf         | `int16`   |              |            |
| <a id="fld_rsv"></a>rsv             | `int16`   |              |            |
| <a id="fld_rbv"></a>rbv             | `int16`   |              |            |
| <a id="fld_rp"></a>rp               | `int16`   |              |            |
| <a id="fld_rc"></a>rc               | `int16`   |              |            |
| <a id="fld_rcal"></a>rcal           | `int16`   |              |            |
| <a id="fld_me"></a>me               | `int16`   |              |            |
| <a id="fld_al"></a>al               | `int16`   |              |            |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/PowerMonitor.msg)

::: details Click here to see original file

```c
# power monitor message

uint64 timestamp			# Time since system start (microseconds)

float32 voltage_v			# Voltage in volts, 0 if unknown
float32 current_a		    # Current in amperes, -1 if unknown
float32 power_w		        # power in watts, -1 if unknown
int16 rconf
int16 rsv
int16 rbv
int16 rp
int16 rc
int16 rcal
int16 me
int16 al
```

:::
