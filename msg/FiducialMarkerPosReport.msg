# Relative position of a precision-landing target detected by a vision pipeline (e.g. an ArUco marker).
#
# Published by: vision pipelines (on-board or off-board over MAVLink TARGET_RELATIVE), decoded in mavlink_receiver.
# Subscribed by: vision_target_estimator (VTEPosition).
#
# The measurement is expressed in an arbitrary sensor frame; the quaternion q rotates it into the NED earth frame.

uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] Timestamp of the raw observation

float32[3] rel_pos # [m] Target position relative to vehicle, expressed in the frame defined by q
float32[3] cov_rel_pos # [m^2] Target position variance, expressed in the frame defined by q

float32[4] q # [-] Quaternion rotation from the rel_pos frame to the NED earth frame
