---
pageClass: is-wide-page
---

# Airspeed (UORB message)

Airspeed data from sensors.

This is published by airspeed sensor drivers, CAN airspeed sensors, simulators.
It is subscribed by the airspeed selector module, which validates the data from multiple sensors and passes on a single estimation to the EKF, controllers and telemetry providers.

**TOPICS:** airspeed

## Fields

| 参数名                                                                                   | 类型        | Unit [Frame] | Range/Enum                                                                  | 描述                               |
| ------------------------------------------------------------------------------------- | --------- | ---------------------------------------------------------------- | --------------------------------------------------------------------------- | -------------------------------- |
| timestamp                                                                             | `uint64`  | us                                                               |                                                                             | Time since system start          |
| timestamp_sample                                                 | `uint64`  | us                                                               |                                                                             | Timestamp of the raw data        |
| indicated_airspeed_m_s | `float32` | 米/秒                                                              |                                                                             | Indicated airspeed               |
| true_airspeed_m_s      | `float32` | 米/秒                                                              |                                                                             | True airspeed                    |
| confidence                                                                            | `float32` |                                                                  | [0 : 1] | Confidence value for this sensor |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/Airspeed.msg)

:::details
Click here to see original file

```c
# Airspeed data from sensors
#
# This is published by airspeed sensor drivers, CAN airspeed sensors, simulators.
# It is subscribed by the airspeed selector module, which validates the data from multiple sensors and passes on a single estimation to the EKF, controllers and telemetry providers.

uint64 timestamp                 # [us] Time since system start
uint64 timestamp_sample          # [us] Timestamp of the raw data
float32 indicated_airspeed_m_s   # [m/s] Indicated airspeed
float32 true_airspeed_m_s        # [m/s] True airspeed
float32 confidence               # [@range 0,1] Confidence value for this sensor
```

:::
