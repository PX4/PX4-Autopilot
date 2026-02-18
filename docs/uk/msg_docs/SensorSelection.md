---
pageClass: is-wide-page
---

# SensorSelection (UORB повідомлення)

Ідентифікатори датчиків для вибраних датчиків, виведених на темі sensor_combined. Will be updated on startup of the sensor module and when sensor selection changes.

**TOPICS:** sensor_selection

## Fields

| Назва                                                     | Тип      | Unit [Frame] | Range/Enum | Опис                                                      |
| --------------------------------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                                                 | `uint64` |                                                                  |            | time since system start (microseconds) |
| accel_device_id | `uint32` |                                                                  |            | unique device ID for the selected accelerometers          |
| gyro_device_id  | `uint32` |                                                                  |            | unique device ID for the selected rate gyros              |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorSelection.msg)

:::details
Click here to see original file

```c
#
# Sensor ID's for the voted sensors output on the sensor_combined topic.
# Will be updated on startup of the sensor module and when sensor selection changes
#
uint64 timestamp		# time since system start (microseconds)
uint32 accel_device_id		# unique device ID for the selected accelerometers
uint32 gyro_device_id		# unique device ID for the selected rate gyros
```

:::
