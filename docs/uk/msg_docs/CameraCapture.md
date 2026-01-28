# CameraCapture (повідомлення UORB)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/CameraCapture.msg)

```c
uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_utc		# Capture time in UTC / GPS time
uint32 seq					# Image sequence number
float64 lat					# Latitude in degrees (WGS84)
float64 lon					# Longitude in degrees (WGS84)
float32 alt					# Altitude (AMSL)
float32 ground_distance			# Altitude above ground (meters)
float32[4] q					# Attitude of the camera relative to NED earth-fixed frame when using a gimbal, otherwise vehicle attitude
int8 result					# 1 for success, 0 for failure, -1 if camera does not provide feedback

```
