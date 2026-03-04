---
pageClass: is-wide-page
---

# EstimatorSelectorStatus (UORB message)

**TOPICS:** estimator_selectorstatus

## Fields

| 参数名                                                               | 类型           | Unit [Frame] | Range/Enum | 描述                                                        |
| ----------------------------------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                                                         | `uint64`     |                                                                  |            | time since system start (microseconds) |
| primary_instance                             | `uint8`      |                                                                  |            |                                                           |
| instances_available                          | `uint8`      |                                                                  |            |                                                           |
| instance_changed_count  | `uint32`     |                                                                  |            |                                                           |
| last_instance_change    | `uint64`     |                                                                  |            |                                                           |
| accel_device_id         | `uint32`     |                                                                  |            |                                                           |
| baro_device_id          | `uint32`     |                                                                  |            |                                                           |
| gyro_device_id          | `uint32`     |                                                                  |            |                                                           |
| mag_device_id           | `uint32`     |                                                                  |            |                                                           |
| combined_test_ratio     | `float32[9]` |                                                                  |            |                                                           |
| relative_test_ratio     | `float32[9]` |                                                                  |            |                                                           |
| healthy                                                           | `bool[9]`    |                                                                  |            |                                                           |
| accumulated_gyro_error  | `float32[4]` |                                                                  |            |                                                           |
| accumulated_accel_error | `float32[4]` |                                                                  |            |                                                           |
| gyro_fault_detected     | `bool`       |                                                                  |            |                                                           |
| accel_fault_detected    | `bool`       |                                                                  |            |                                                           |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/EstimatorSelectorStatus.msg)

:::details
Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)

uint8 primary_instance

uint8 instances_available

uint32 instance_changed_count
uint64 last_instance_change

uint32 accel_device_id
uint32 baro_device_id
uint32 gyro_device_id
uint32 mag_device_id

float32[9] combined_test_ratio
float32[9] relative_test_ratio
bool[9] healthy

float32[4] accumulated_gyro_error
float32[4] accumulated_accel_error
bool gyro_fault_detected
bool accel_fault_detected
```

:::
