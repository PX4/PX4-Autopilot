# Yaw of a precision-landing target relative to the NED (North, East, Down) frame, reported by a vision pipeline.
#
# Published by: vision pipelines (on-board or off-board over MAVLink TARGET_RELATIVE), decoded in mavlink_receiver.
# Subscribed by: vision_target_estimator (VTEOrientation).

uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] Timestamp of the raw observation

float32 yaw_ned # [rad] [@frame NED] Orientation of the target relative to the NED frame [-Pi ; Pi]
float32 yaw_var_ned # [rad^2] Orientation uncertainty
