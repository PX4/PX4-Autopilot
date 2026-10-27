# Vision target measurement from the THROUGH vision node (ROS 2 -> PX4)
#
# Angles are relative to the camera optical axis (camera frame: x forward, y right, z down).

uint64 timestamp            # time since system start (us)
uint64 timestamp_sample     # image capture time (us, PX4 clock)

float32 pan_error           # [rad] +: target is RIGHT of image center
float32 tilt_error          # [rad] +: target is BELOW image center
float32 target_size         # [-] enclosing-circle diameter / image width (0..~1.5)
float32 confidence          # [-] 0..1, detection quality (optional, 1.0 if unused)

bool uncage                 # human lock state (true = locked by operator)
bool target_valid           # target detected in this frame

uint32 seq                  # frame sequence number
