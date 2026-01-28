# VehicleRoi (повідомлення UORB)

Vehicle Region Of Interest (ROI)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleRoi.msg)

```c
# Vehicle Region Of Interest (ROI)

uint64 timestamp			# time since system start (microseconds)

uint8 ROI_NONE = 0			# No region of interest
uint8 ROI_WPNEXT = 1			# Point toward next MISSION with optional offset
uint8 ROI_WPINDEX = 2			# Point toward given MISSION
uint8 ROI_LOCATION = 3			# Point toward fixed location
uint8 ROI_TARGET = 4			# Point toward target
uint8 ROI_ENUM_END = 5

uint8 mode          # ROI mode (see above)

float64 lat			    # Latitude to point to
float64 lon			    # Longitude to point to
float32 alt			    # Altitude to point to

# additional angle offsets to next waypoint (only used with ROI_WPNEXT)
float32 roll_offset		# angle offset in rad
float32 pitch_offset		# angle offset in rad
float32 yaw_offset		# angle offset in rad

```
