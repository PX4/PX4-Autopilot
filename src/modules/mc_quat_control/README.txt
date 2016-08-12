This controller is for the inner loop controller (Section 3) of "An Open-Source Implementation of a Unit Quaternion based Attitude and Trajectory Tracking for Quadrotors"

The gains to tune are 

MC_QUAT_ROLL_P
MC_QUAT_PITCH_P

MC_QUAT_ROLLRATE_P
MC_QUAT_PITCHRATE_P

MC_QUAT_ROLLRATE_D
MC_QUAT_PITCHRATE_D

MC_QUAT_YAW_P

MC_QUAT_YAWRATE_P

For symmetric quads, the above pairs should be the same. For manual control,

MC_QUAT_ROLLRATE_D
MC_QUAT_PITCHRATE_D
MC_QUAT_YAW_P

can be set to zero.
