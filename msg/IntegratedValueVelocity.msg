# This is similar to the mavlink message ATTITUDE_QUATERNION, but for onboard use

uint64 timestamp                # time since system start (microseconds)

uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

float32[3] integrated_value_velocity                    # the integral of the accelerometer values
