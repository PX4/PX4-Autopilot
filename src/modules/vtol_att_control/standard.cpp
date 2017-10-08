/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file standard.cpp
 *
 * @author Simon Wilks		<simon@uaventure.com>
 * @author Roman Bapst		<bapstroman@gmail.com>
 * @author Andreas Antener	<andreas@uaventure.com>
 * @author Sander Smeets	<sander@droneslab.com>
 *
*/

#include "standard.h"

#include "vtol_att_control_main.h"

#include <cfloat>

Standard::Standard(VtolAttitudeControl *attc) :
	VtolType(attc),
	_param_pusher_trans(this, "TRANS_THR"),
	_param_front_trans_timeout(this, "TRANS_TIMEOUT"),
	_param_back_trans_throttle(this, "B_TRANS_THR"),
	_param_back_trans_ramp(this, "B_TRANS_RAMP"),
	_param_back_reverse_output(this, "B_REV_OUT"),
	_param_back_reverse_delay(this, "B_REV_DEL"),
	_param_down_pitch_max(this, "DWN_PITCH_MAX"),
	_param_forward_thrust_scale(this, "FWD_THRUST_SC"),
	_param_fw_pitch_setpoint_offset(this, "FW_PSP_OFF", false),
	_param_mc_xy_cruise_velocity(this, "MPC_XY_CRUISE", false)
{
	_vtol_schedule.flight_mode = MC_MODE;
	_vtol_schedule.transition_start = 0;

	_mc_throttle_weight = 1.0f;
}

void Standard::update_vtol_state()
{
	/* After flipping the switch the vehicle will start the pusher (or tractor) motor, picking up
	 * forward speed. After the vehicle has picked up enough speed the rotors shutdown.
	 * For the back transition the pusher motor is immediately stopped and rotors reactivated.
	 */

	if (!_attc->is_fixed_wing_requested()) {

		// the transition to fw mode switch is off

		if (_vtol_schedule.flight_mode == FW_MODE) {
			// transition to mc mode
			if (_attc->get_vtol_vehicle_status()->vtol_transition_failsafe) {
				// Failsafe event, engage mc motors immediately
				_vtol_schedule.flight_mode = MC_MODE;
				_flag_enable_mc_motors = true;

			} else {
				// Regular backtransition
				_vtol_schedule.transition_start = hrt_absolute_time();
				_vtol_schedule.flight_mode = TRANSITION_TO_MC;
				_flag_enable_mc_motors = true;
				_transition_achieved = false;
			}

		} else if (_vtol_schedule.flight_mode == TRANSITION_TO_FW) {
			// failsafe back to mc mode
			_vtol_schedule.flight_mode = MC_MODE;

		} else if (_vtol_schedule.flight_mode == TRANSITION_TO_MC) {
			// transition to MC mode if transition time has passed or forward velocity drops below MPC cruise speed
			const matrix::Dcmf R_to_body(matrix::Quatf(_v_att->q).inversed());
			const matrix::Vector3f vel = R_to_body * matrix::Vector3f(_local_pos->vx, _local_pos->vy, _local_pos->vz);

			const bool mc_speed_limit = _local_pos->v_xy_valid && (vel(0) <= _param_mc_xy_cruise_velocity.get());
			const bool back_trans_timeout = (hrt_elapsed_time(&_vtol_schedule.transition_start) >
							 (_param_back_trans_dur.get() * 1e6f));

			if (back_trans_timeout || mc_speed_limit || can_transition_on_ground()) {
				if (!_transition_achieved) {
					// first time transition achieved, begin transition timer
					_trans_finished_ts = hrt_absolute_time();
				}

				_transition_achieved = true;

			} else {
				_transition_achieved = false;
				_trans_finished_ts = 0;
			}

			// all conditions must be continuously satisfied for > 1s to complete transition
			if (_transition_achieved && (hrt_elapsed_time(&_trans_finished_ts) > 1e6f)) {
				_vtol_schedule.flight_mode = MC_MODE;
			}
		}

	} else {
		// the transition to fw mode switch is on
		if (_vtol_schedule.flight_mode == MC_MODE || _vtol_schedule.flight_mode == TRANSITION_TO_MC) {
			// start transition to fw mode

			_vtol_schedule.transition_start = hrt_absolute_time();
			_vtol_schedule.flight_mode = TRANSITION_TO_FW;
			_transition_achieved = false;

		} else if (_vtol_schedule.flight_mode == TRANSITION_TO_FW) {
			// continue the transition to FW mode while monitoring airspeed and MC throttle

			const bool airspeed_disabled = (_param_airspeed_mode.get() != 0);

			const bool airspeed = (_airspeed->indicated_airspeed_m_s >= _param_airspeed_trans.get()) || airspeed_disabled;
			const bool min_trans_time = (hrt_elapsed_time(&_vtol_schedule.transition_start) > (_param_front_trans_time_min.get() *
						     1e6f));

			if ((airspeed && min_trans_time) || can_transition_on_ground()) {
				if (!_transition_achieved) {
					// first time transition achieved, begin transition timer
					_trans_finished_ts = hrt_absolute_time();
				}

				_transition_achieved = true;

			} else {
				_transition_achieved = false;
				_trans_finished_ts = 0;
			}

			// all conditions must be continuously satisfied for > 1s to complete transition
			if (_transition_achieved && (hrt_elapsed_time(&_trans_finished_ts) > 1e6f)) {
				_vtol_schedule.flight_mode = FW_MODE;

				// we can turn off the multirotor motors now
				_flag_enable_mc_motors = false;

				// update final transition finished time
				_trans_finished_ts = hrt_absolute_time();
			}
		}
	}

	_vtol_mode = (mode)_vtol_schedule.flight_mode;
}

void Standard::update_transition_state()
{
	VtolType::update_transition_state();

	// copy virtual attitude setpoint to real attitude setpoint
	memcpy(_v_att_sp, _mc_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));

	const float trans_elapsed = hrt_elapsed_time(&_vtol_schedule.transition_start) / 1e6f;

	if (_vtol_schedule.flight_mode == TRANSITION_TO_FW) {
		if (_param_front_trans_dur.get() <= 0.0f) {
			// just set the final target throttle value
			_pusher_throttle = _param_pusher_trans.get();

		} else if (_pusher_throttle <= _param_pusher_trans.get()) {
			const float progress = constrain(trans_elapsed / _param_front_trans_dur.get(), 0.0f, 1.0f);

			// ramp up throttle to the target throttle value
			_pusher_throttle = _param_pusher_trans.get() * progress;
		}

		// disable airbrakes output
		_airbrakes_output = 0.0f;

		const float as_blend_region = _param_airspeed_trans.get() - _param_airspeed_blend_start.get();
		const float as_above_blend = _airspeed->indicated_airspeed_m_s - _param_airspeed_blend_start.get();

		const bool airspeed_enabled = (_param_airspeed_mode.get() == 0);

		// do blending of mc and fw controls if a blending airspeed has been provided and the minimum transition time has passed
		if (airspeed_enabled && (as_blend_region > 0.0f) && (as_above_blend > 0.0f) &&
		    (trans_elapsed > _param_front_trans_time_min.get())) {

			_mc_throttle_weight = math::constrain(1.0f - as_above_blend / as_blend_region, 0.0f, 1.0f);

		} else if (!airspeed_enabled &&
			   (trans_elapsed > (_param_front_trans_time_min.get() / 2.0f) &&
			    (trans_elapsed < _param_front_trans_time_min.get()))) {

			// time based blending when no airspeed sensor is set
			const float weight = 1.0f - (trans_elapsed / _param_front_trans_time_min.get());

			// weight is doubled to blend over the 2nd half of the interval
			_mc_throttle_weight = math::constrain(2.0f * weight, 0.0f, 1.0f);

		} else {
			// at low speeds give full weight to mc
			_mc_throttle_weight = 1.0f;
		}

		// ramp up FW_PSP_OFF
		_v_att_sp->pitch_body = _param_fw_pitch_setpoint_offset.get() * (1.0f - _mc_pitch_weight);
		matrix::Quatf q_sp(matrix::Eulerf(_v_att_sp->roll_body, _v_att_sp->pitch_body, _v_att_sp->yaw_body));
		q_sp.copyTo(_v_att_sp->q_d);
		_v_att_sp->q_d_valid = true;

		// check front transition timeout
		if (_param_front_trans_timeout.get() > FLT_EPSILON) {
			if (trans_elapsed > _param_front_trans_timeout.get()) {

				// transition timeout occurred, abort transition
				_attc->abort_front_transition("Transition timeout");
			}
		}

	} else if (_vtol_schedule.flight_mode == TRANSITION_TO_MC) {

		// maintain FW_PSP_OFF
		_v_att_sp->pitch_body = _param_fw_pitch_setpoint_offset.get();
		matrix::Quatf q_sp(matrix::Eulerf(_v_att_sp->roll_body, _v_att_sp->pitch_body, _v_att_sp->yaw_body));
		q_sp.copyTo(_v_att_sp->q_d);
		_v_att_sp->q_d_valid = true;

		// enable airbrakes output during back transition (if configured)
		_airbrakes_output = _param_back_reverse_output.get();

		// desired throttle during back transition (usually reverse throttle)
		const float btrans_elapsed = hrt_elapsed_time(&_vtol_schedule.transition_start) / 1e6f;

		if (btrans_elapsed >= _param_back_reverse_delay.get()) {
			const float thrscale = constrain(btrans_elapsed / _param_front_trans_dur.get(), -1.0f, 1.0f);
			_pusher_throttle = thrscale * _param_back_trans_throttle.get();

		} else {
			_pusher_throttle = 0.0f;
		}

		if (_param_back_trans_ramp.get() > FLT_EPSILON) {
			// continually increase control as we transition back to mc mode
			_mc_throttle_weight = constrain(trans_elapsed / _param_back_trans_ramp.get(), 0.0f, 1.0f);

		} else {
			_mc_throttle_weight = 1.0f;
		}

		// in back transition mode we need to start the MC motors again
		if (_flag_enable_mc_motors) {
			_flag_enable_mc_motors = !enable_mc_motors();
		}
	}
}

void Standard::update_mc_state()
{
	VtolType::update_mc_state();

	// disable pusher and airbrakes
	_pusher_throttle = 0.0f;
	_airbrakes_output = 0.0f;

	// full MC weight
	_mc_throttle_weight = 1.0f;

	// enable MC motors here in case we transitioned directly to MC mode
	if (_flag_enable_mc_motors) {
		_flag_enable_mc_motors = !enable_mc_motors();
	}

	// if the thrust scale param is zero or the drone is on manual mode,
	// then the pusher-for-pitch strategy is disabled and we can return
	if (_param_forward_thrust_scale.get() < FLT_EPSILON ||
	    !_v_control_mode->flag_control_position_enabled) {

		return;
	}

	// Do not engage pusher assist during a failsafe event
	// There could be a problem with the fixed wing drive
	if (_attc->get_vtol_vehicle_status()->vtol_transition_failsafe) {
		return;
	}

	// disable pusher assist during landing
	if (_attc->get_pos_sp_triplet()->current.valid &&
	    (_attc->get_pos_sp_triplet()->current.type == position_setpoint_s::SETPOINT_TYPE_LAND)) {

		return;
	}

	matrix::Dcmf R(matrix::Quatf(_v_att->q));
	matrix::Dcmf R_sp(matrix::Quatf(_v_att_sp->q_d));
	matrix::Eulerf euler(R);
	matrix::Eulerf euler_sp(R_sp);

	// direction of desired body z axis represented in earth frame
	matrix::Vector3f body_z_sp(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));

	// rotate desired body z axis into new frame which is rotated in z by the current
	// heading of the vehicle. we refer to this as the heading frame.
	matrix::Dcmf R_yaw = matrix::Eulerf(0.0f, 0.0f, -euler(2));
	body_z_sp = R_yaw * body_z_sp;
	body_z_sp.normalize();

	// calculate the desired pitch seen in the heading frame
	// this value corresponds to the amount the vehicle would try to pitch forward
	float pitch_forward = atan2f(body_z_sp(0), body_z_sp(2));

	// only allow pitching forward up to threshold, the rest of the desired
	// forward acceleration will be compensated by the pusher
	const float down_pitch_max_rad = math::radians(_param_down_pitch_max.get());

	if (pitch_forward < -down_pitch_max_rad) {
		// desired roll angle in heading frame stays the same
		float roll_new = -asinf(body_z_sp(1));

		_pusher_throttle = (sinf(-pitch_forward) - sinf(down_pitch_max_rad)) * _param_forward_thrust_scale.get();
		_pusher_throttle = constrain(_pusher_throttle, 0.0f, 1.0f);

		// return the vehicle to level position
		float pitch_new = 0.0f;

		// create corrected desired body z axis in heading frame
		matrix::Dcmf R_tmp = matrix::Eulerf(roll_new, pitch_new, 0.0f);
		matrix::Vector3f tilt_new(R_tmp(0, 2), R_tmp(1, 2), R_tmp(2, 2));

		// rotate the vector into a new frame which is rotated in z by the desired heading
		// with respect to the earth frame.
		float yaw_error = _wrap_pi(euler_sp(2) - euler(2));
		matrix::Dcmf R_yaw_correction = matrix::Eulerf(0.0f, 0.0f, -yaw_error);
		tilt_new = R_yaw_correction * tilt_new;

		// now extract roll and pitch setpoints
		_v_att_sp->pitch_body = atan2f(tilt_new(0), tilt_new(2));
		_v_att_sp->roll_body = -asinf(tilt_new(1));
		R_sp = matrix::Eulerf(_v_att_sp->roll_body, _v_att_sp->pitch_body, euler_sp(2));
		matrix::Quatf q_sp(R_sp);
		q_sp.copyTo(_v_att_sp->q_d);
	}
}

void Standard::update_fw_state()
{
	VtolType::update_fw_state();

	// zero pusher and airbrakes
	_pusher_throttle = 0.0f;
	_airbrakes_output = 0.0f;

	// MC off
	_mc_throttle_weight = 0.0f;

	// stop MC motors in FW mode
	if (!_flag_enable_mc_motors) {
		_flag_enable_mc_motors = disable_mc_motors();
	}
}

/**
 * Prepare message to actuators with data from mc and fw attitude controllers. An mc attitude weighting will determine
 * what proportion of control should be applied to each of the control groups (mc and fw).
 */
void Standard::fill_actuator_outputs()
{
	_actuators_out_0->timestamp = _actuators_mc_in->timestamp;
	_actuators_out_1->timestamp = _actuators_fw_in->timestamp;

	const auto &mc_in = _actuators_mc_in->control; // MC input: mc_att_control
	const auto &fw_in = _actuators_fw_in->control; // FW input: fw_att_control

	auto &mc_out = _actuators_out_0->control; // MC output: actuator controls 0
	auto &fw_out = _actuators_out_1->control; // FW output: actuator controls 1

	switch (_vtol_schedule.flight_mode) {
	case MC_MODE:
		mc_out[actuator_controls_s::INDEX_ROLL] = mc_in[actuator_controls_s::INDEX_ROLL];
		mc_out[actuator_controls_s::INDEX_PITCH] = mc_in[actuator_controls_s::INDEX_PITCH];
		mc_out[actuator_controls_s::INDEX_YAW] = mc_in[actuator_controls_s::INDEX_YAW];
		mc_out[actuator_controls_s::INDEX_THROTTLE] = mc_in[actuator_controls_s::INDEX_THROTTLE];

		if (_param_elevons_mc_lock.get()) {
			fw_out[actuator_controls_s::INDEX_ROLL] = 0.0f;
			fw_out[actuator_controls_s::INDEX_PITCH] = 0.0f;
			fw_out[actuator_controls_s::INDEX_YAW] = 0.0f;
			fw_out[actuator_controls_s::INDEX_THROTTLE] = _pusher_throttle;
			fw_out[actuator_controls_s::INDEX_AIRBRAKES] = _airbrakes_output;

		} else {
			// TODO: fix reversed roll
			fw_out[actuator_controls_s::INDEX_ROLL] = -fw_in[actuator_controls_s::INDEX_ROLL];
			fw_out[actuator_controls_s::INDEX_PITCH] = fw_in[actuator_controls_s::INDEX_PITCH];
			fw_out[actuator_controls_s::INDEX_YAW] = fw_in[actuator_controls_s::INDEX_YAW];
			fw_out[actuator_controls_s::INDEX_THROTTLE] = _pusher_throttle;
			fw_out[actuator_controls_s::INDEX_AIRBRAKES] = _airbrakes_output;
		}

		break;

	case TRANSITION_TO_FW:
	case TRANSITION_TO_MC:
		mc_out[actuator_controls_s::INDEX_ROLL] = mc_in[actuator_controls_s::INDEX_ROLL] * _mc_throttle_weight;
		mc_out[actuator_controls_s::INDEX_PITCH] = mc_in[actuator_controls_s::INDEX_PITCH] * _mc_throttle_weight;
		mc_out[actuator_controls_s::INDEX_YAW] = mc_in[actuator_controls_s::INDEX_YAW] * _mc_throttle_weight;
		mc_out[actuator_controls_s::INDEX_THROTTLE] = mc_in[actuator_controls_s::INDEX_THROTTLE] * _mc_throttle_weight;

		fw_out[actuator_controls_s::INDEX_ROLL] = -fw_in[actuator_controls_s::INDEX_ROLL];
		fw_out[actuator_controls_s::INDEX_PITCH] = fw_in[actuator_controls_s::INDEX_PITCH];
		fw_out[actuator_controls_s::INDEX_YAW] = fw_in[actuator_controls_s::INDEX_YAW];
		fw_out[actuator_controls_s::INDEX_THROTTLE] = _pusher_throttle;
		fw_out[actuator_controls_s::INDEX_AIRBRAKES] = _airbrakes_output;

		break;

	case FW_MODE:
		mc_out[actuator_controls_s::INDEX_ROLL] = 0.0f;
		mc_out[actuator_controls_s::INDEX_PITCH] = 0.0f;
		mc_out[actuator_controls_s::INDEX_YAW] = 0.0f;
		mc_out[actuator_controls_s::INDEX_THROTTLE] = 0.0f;

		// TODO: fix reversed roll
		fw_out[actuator_controls_s::INDEX_ROLL] = -fw_in[actuator_controls_s::INDEX_ROLL];
		fw_out[actuator_controls_s::INDEX_PITCH] = fw_in[actuator_controls_s::INDEX_PITCH];
		fw_out[actuator_controls_s::INDEX_YAW] = fw_in[actuator_controls_s::INDEX_YAW];
		fw_out[actuator_controls_s::INDEX_THROTTLE] = fw_in[actuator_controls_s::INDEX_THROTTLE];
		fw_out[actuator_controls_s::INDEX_AIRBRAKES] = fw_in[actuator_controls_s::INDEX_AIRBRAKES];

		break;
	}
}

void
Standard::waiting_on_tecs()
{
	// keep thrust from transition
	_v_att_sp->thrust = _pusher_throttle;
};
