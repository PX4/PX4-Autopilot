# VehicleMagnetometer (UORB message)



[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleMagnetometer.msg)

```c

uint64 timestamp            # time since system start (microseconds)

uint64 timestamp_sample     # the timestamp of the raw data (microseconds)

uint32 device_id            # unique device ID for the selected magnetometer

float32[3] magnetometer_ga  # Magnetic field in the FRD body frame XYZ-axis in Gauss

uint8 calibration_count     # Calibration changed counter. Monotonically increases whenever calibration changes.

```
