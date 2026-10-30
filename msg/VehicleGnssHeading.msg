# GNSS heading from a dual-antenna or moving-baseline receiver
#
# Published by the sensors module from sensor_gnss_relative. Consumed by EKF2 independently from position and velocity.

uint64 timestamp              # time since system start (microseconds)
uint64 timestamp_sample       # time since system start (microseconds) - actual measurement time

uint32 device_id              # unique device ID for the sensor that does not change between power cycles

float32 heading               # body-frame heading rel to NED from dual antenna array (rad, [-PI, PI])
float32 heading_accuracy      # 1-sigma heading accuracy (rad)
float32 heading_offset        # yaw of the configured antenna baseline in the body frame; heading + heading_offset is the measured baseline heading (rad, [-PI, PI])
float32 baseline_length       # antenna baseline length reported by the receiver, NaN if it doesn't report one (m)

uint8 jamming_state           # jamming_state of the receiver providing the heading, values as in SensorGnss (0: Unknown, 1: OK, 2: Mitigated, 3: Detected)
uint8 spoofing_state          # spoofing_state of the receiver providing the heading, values as in SensorGnss (0: Unknown, 1: OK, 2: Mitigated, 3: Detected)
bool usable                   # heading may be used: its receiver reports no spoofing or jamming that the enabled checks reject
