---
pageClass: is-wide-page
---

# EstimatorEventFlags (UORB message)

**TOPICS:** estimator_eventflags

## Fields

| 명칭                                                                                                          | 형식       | Unit [Frame] | Range/Enum | 설명                                                                                                        |
| ----------------------------------------------------------------------------------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------------------------------------------------------- |
| timestamp                                                                                                   | `uint64` |                                                                  |            | time since system start (microseconds)                                                 |
| timestamp_sample                                                                       | `uint64` |                                                                  |            | the timestamp of the raw data (microseconds)                                           |
| information_event_changes                                         | `uint32` |                                                                  |            | number of information event changes                                                                       |
| gps_checks_passed                                                 | `bool`   |                                                                  |            | 0 - true when gps quality checks are passing passed                                                       |
| reset_vel_to_gps                             | `bool`   |                                                                  |            | 1 - true when the velocity states are reset to the gps measurement                                        |
| reset_vel_to_flow                            | `bool`   |                                                                  |            | 2 - true when the velocity states are reset using the optical flow measurement                            |
| reset_vel_to_vision                          | `bool`   |                                                                  |            | 3 - true when the velocity states are reset to the vision system measurement                              |
| reset_vel_to_zero                            | `bool`   |                                                                  |            | 4 - true when the velocity states are reset to zero                                                       |
| reset_pos_to_last_known | `bool`   |                                                                  |            | 5 - true when the position states are reset to the last known position                                    |
| reset_pos_to_gps                             | `bool`   |                                                                  |            | 6 - true when the position states are reset to the gps measurement                                        |
| reset_pos_to_vision                          | `bool`   |                                                                  |            | 7 - true when the position states are reset to the vision system measurement                              |
| starting_gps_fusion                                               | `bool`   |                                                                  |            | 8 - true when the filter starts using gps measurements to correct the state estimates                     |
| starting_vision_pos_fusion                   | `bool`   |                                                                  |            | 9 - true when the filter starts using vision system position measurements to correct the state estimates  |
| starting_vision_vel_fusion                   | `bool`   |                                                                  |            | 10 - true when the filter starts using vision system velocity measurements to correct the state estimates |
| starting_vision_yaw_fusion                   | `bool`   |                                                                  |            | 11 - true when the filter starts using vision system yaw measurements to correct the state estimates      |
| yaw_aligned_to_imu_gps  | `bool`   |                                                                  |            | 12 - true when the filter resets the yaw to an estimate derived from IMU and GPS data                     |
| reset_hgt_to_baro                            | `bool`   |                                                                  |            | 13 - true when the vertical position state is reset to the baro measurement                               |
| reset_hgt_to_gps                             | `bool`   |                                                                  |            | 14 - true when the vertical position state is reset to the gps measurement                                |
| reset_hgt_to_rng                             | `bool`   |                                                                  |            | 15 - true when the vertical position state is reset to the rng measurement                                |
| reset_hgt_to_ev                              | `bool`   |                                                                  |            | 16 - true when the vertical position state is reset to the ev measurement                                 |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/EstimatorEventFlags.msg)

:::details
Click here to see original file

```c
uint64 timestamp                        # time since system start (microseconds)
uint64 timestamp_sample                 # the timestamp of the raw data (microseconds)

# information events
uint32 information_event_changes        # number of information event changes
bool gps_checks_passed                  #  0 - true when gps quality checks are passing passed
bool reset_vel_to_gps                   #  1 - true when the velocity states are reset to the gps measurement
bool reset_vel_to_flow                  #  2 - true when the velocity states are reset using the optical flow measurement
bool reset_vel_to_vision                #  3 - true when the velocity states are reset to the vision system measurement
bool reset_vel_to_zero                  #  4 - true when the velocity states are reset to zero
bool reset_pos_to_last_known            #  5 - true when the position states are reset to the last known position
bool reset_pos_to_gps                   #  6 - true when the position states are reset to the gps measurement
bool reset_pos_to_vision                #  7 - true when the position states are reset to the vision system measurement
bool starting_gps_fusion                #  8 - true when the filter starts using gps measurements to correct the state estimates
bool starting_vision_pos_fusion         #  9 - true when the filter starts using vision system position measurements to correct the state estimates
bool starting_vision_vel_fusion         # 10 - true when the filter starts using vision system velocity measurements to correct the state estimates
bool starting_vision_yaw_fusion         # 11 - true when the filter starts using vision system yaw  measurements to correct the state estimates
bool yaw_aligned_to_imu_gps             # 12 - true when the filter resets the yaw to an estimate derived from IMU and GPS data
bool reset_hgt_to_baro                  # 13 - true when the vertical position state is reset to the baro measurement
bool reset_hgt_to_gps                   # 14 - true when the vertical position state is reset to the gps measurement
bool reset_hgt_to_rng                   # 15 - true when the vertical position state is reset to the rng measurement
bool reset_hgt_to_ev                    # 16 - true when the vertical position state is reset to the ev measurement
```

:::
