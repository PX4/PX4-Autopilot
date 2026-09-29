# TEMPORARY DEBUG TOPIC (not meant to be committed): state of the attitude reference model in mc_att_control

uint64 timestamp        # time since system start (microseconds)

float32[4] q            # reference attitude quaternion (w, x, y, z) tracked by the attitude P controller
float32[3] rate         # reference angular rate [rad/s] in the reference body frame (error-driven correction + commanded yaw rate)
float32[3] acceleration # reference angular acceleration [rad/s^2] (trajectory acceleration on limited axes, analytical linear-model acceleration on the others)
