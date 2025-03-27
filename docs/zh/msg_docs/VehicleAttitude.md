# VehicleAttitude (UORB message)

This is similar to the mavlink message ATTITUDE_QUATERNION, but for onboard use
The quaternion uses the Hamilton convention, and the order is q(w, x, y, z)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/VehicleAttitude.msg)

```c
# This is similar to the mavlink message ATTITUDE_QUATERNION, but for onboard use
# The quaternion uses the Hamilton convention, and the order is q(w, x, y, z)

uint32 MESSAGE_VERSION = 0

uint64 timestamp                # time since system start (microseconds)

uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

float32[4] q                    # Quaternion rotation from the FRD body frame to the NED earth frame
float32[4] delta_q_reset        # Amount by which quaternion has changed during last reset
uint8 quat_reset_counter        # Quaternion reset counter

# TOPICS vehicle_attitude vehicle_attitude_groundtruth external_ins_attitude
# TOPICS estimator_attitude

```
