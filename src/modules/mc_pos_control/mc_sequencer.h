#ifndef MC_SEQUENCER_H
#define MC_SEQUENCER_H

#include <px4_defines.h>
#include <math.h>
#include <drivers/drv_hrt.h>

#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/control_state.h>

#include <mathlib/mathlib.h>
#include <platforms/px4_defines.h>

/*
 * Perform a 360 degree roll or pitch flip starting and ending at current attitude
 *
 * Inputs:
 * ctrl_state: for current attitude quaternion
 * manual: sequence trigger switch
 *
 * Outputs:
 * att_sp: thrust and rotation angle setpoints
 * R_sp: attitude setpoint
 * rollRate, pitchRate, yawRate: rotation rate commands
 */
void flip_sequence(
	struct control_state_s &ctrl_state,
	struct vehicle_attitude_setpoint_s &att_sp,
	struct manual_control_setpoint_s &manual,
	math::Matrix<3, 3> &R_sp,
	float &rollRate, float &pitchRate, float &yawRate);

/*
 * Perform a programmed sequence with specified target attitudes
 *
 * Inputs:
 * sequence: array of seq_entry items
 * ctrl_state: for current attitude quaternion
 * manual: sequence trigger switch
 *
 * Outputs:
 * att_sp: thrust and rotation angle setpoints
 * R_sp: attitude setpoint
 * rollRate, pitchRate, yawRate: rotation rate commands
 */
void prog_sequence(
	struct control_state_s &ctrl_state,
	struct vehicle_attitude_setpoint_s &att_sp,
	struct manual_control_setpoint_s &manual,
	math::Matrix<3, 3> &R_sp,
	float &rollRate, float &pitchRate, float &yawRate);

enum Seq_state {
	IDLE, RATE, ATTITUDE, DELAY, NEXT_ENTRY
};
struct seq_entry_s {
	Seq_state type;
	float thrust;

	// rates are in radians/second (independent of parameter values)
	float rollRate;
	float pitchRate;
	float yawRate;

	float target_euler[3];

	float delay;
};

#endif
