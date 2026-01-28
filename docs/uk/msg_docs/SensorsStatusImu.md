# SensorsStatusImu (повідомлення UORB)

Метрики перевірки датчика. Це значення буде нульовим для датчика, який є первинним або незаповненим.

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorsStatusImu.msg)

```c
#
# Sensor check metrics. This will be zero for a sensor that's primary or unpopulated.
#
uint64 timestamp # time since system start (microseconds)

uint32 accel_device_id_primary       # current primary accel device id for reference

uint32[4] accel_device_ids
float32[4] accel_inconsistency_m_s_s # magnitude of acceleration difference between IMU instance and mean in m/s^2.
bool[4] accel_healthy
uint8[4] accel_priority

uint32 gyro_device_id_primary       # current primary gyro device id for reference

uint32[4] gyro_device_ids
float32[4] gyro_inconsistency_rad_s # magnitude of angular rate difference between IMU instance and mean in (rad/s).
bool[4] gyro_healthy
uint8[4] gyro_priority

```
