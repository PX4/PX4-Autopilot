# Validated GNSS heading from dual-antenna / moving baseline receiver pair.
# Published by the sensors module after selecting and validating heading from sensor_gnss_relative
# or falling back to sensor_gps heading fields. Consumed by EKF2 independently from position/velocity.

uint64 timestamp              # time since system start (microseconds)
uint64 timestamp_sample       # time since system start (microseconds) - actual measurement time

uint32 device_id              # unique device ID for the sensor that does not change between power cycles

float32 heading               # body-frame heading rel to NED from dual antenna array (rad, [-PI, PI])
float32 heading_accuracy      # 1-sigma heading accuracy (rad)
float32 heading_offset        # yaw of the antenna baseline in the body frame, from SENS_GPSn_ROT; heading + heading_offset is the measured baseline heading (rad, [-PI, PI])
