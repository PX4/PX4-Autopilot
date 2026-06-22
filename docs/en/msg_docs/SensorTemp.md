---
pageClass: is-wide-page
---

# SensorTemp (UORB message)

**TOPICS:** sensor_temp

## Fields

| Name                                | Type      | Unit [Frame] | Range/Enum | Description                                                               |
| ----------------------------------- | --------- | ------------ | ---------- | ------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp | `uint64`  |              |            | time since system start (microseconds)                                    |
| <a id="fld_device_id"></a>device_id | `uint32`  |              |            | unique device ID for the sensor that does not change between power cycles |
| <a id="fld_"></a>                   | `float32` |              |            | Temperature provided by sensor (Celsius)                                  |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorTemp.msg)

::: details Click here to see original file

```c
uint64 timestamp          # time since system start (microseconds)

uint32 device_id          # unique device ID for the sensor that does not change between power cycles
float32  temperature      # Temperature provided by sensor (Celsius)
```

:::
