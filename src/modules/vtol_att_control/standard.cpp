/****************************************************************************
 *
 *   Copyright (c) 2015-2022 PX4 Development Team. All rights reserved.
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

#include <float.h>

using namespace matrix;

Standard::Standard(VtolAttitudeControl *attc) :
	VtolType(attc)
{
}

void
Standard::parameters_update()
{
	VtolType::updateParams();

	// make sure that pusher ramp in backtransition is smaller than back transition (max) duration
	_param_vt_b_trans_ramp.set(math::min(_param_vt_b_trans_ramp.get(), _param_vt_b_trans_dur.get()));
}

void Standard::update_vtol_state()
{
	/* After flipping the switch the vehicle will start the pusher (or tractor) motor, picking up
	 * forward speed. After the vehicle has picked up enough speed the rotors shutdown.
	 * For the back transition the pusher motor is immediately stopped and rotors reactivated.
	 */

	float mc_weight = _mc_roll_weight;

	if (_vtol_vehicle_status->fixed_wing_system_failure) {
		// Failsafe event, engage mc motors immediately
		_vtol_mode = vtol_mode::MC_MODE;
		_pusher_throttle = 0.0f;

	} else if (!_attc->is_fixed_wing_requested()) {

		// the transition to fw mode switch is off
		if (_vtol_mode == vtol_mode::MC_MODE) {
			// in mc mode
			_vtol_mode = vtol_mode::MC_MODE;
			mc_weight = 1.0f;

		} else if (_vtol_mode == vtol_mode::FW_MODE) {
			// Regular backtransition
			resetTransitionStates();
			_vtol_mode = vtol_mode::TRANSITION_TO_MC;

		} else if (_vtol_mode == vtol_mode::TRANSITION_TO_FW) {
			// failsafe back to mc mode
			_vtol_mode = vtol_mode::MC_MODE;
			mc_weight = 1.0f;
			_pusher_throttle = 0.0f;

		} else if (_vtol_mode == vtol_mode::TRANSITION_TO_MC) {
			// speed exit condition: use ground if valid, otherwise airspeed
			bool exit_backtransition_speed_condition = false;

			if (_local_pos->v_xy_valid) {
				const Dcmf R_to_body(Quatf(_v_att->q).inversed());
				const Vector3f vel = R_to_body * Vector3f(_local_pos->vx, _local_pos->vy, _local_pos->vz);
				exit_backtransition_speed_condition = vel(0) < _param_mpc_xy_cruise.get();

			} else if (PX4_ISFINITE(_airspeed_validated->calibrated_airspeed_m_s)) {
				exit_backtransition_speed_condition = _airspeed_validated->calibrated_airspeed_m_s < _param_mpc_xy_cruise.get();
			}

			const bool exit_backtransition_time_condition = _time_since_trans_start > _param_vt_b_trans_dur.get();

			if (can_transition_on_ground() || exit_backtransition_speed_condition || exit_backtransition_time_condition) {
				_vtol_mode = vtol_mode::MC_MODE;
			}
		}

	} else {
		// the transition to fw mode switch is on
		if (_vtol_mode == vtol_mode::MC_MODE || _vtol_mode == vtol_mode::TRANSITION_TO_MC) {
			// start transition to fw mode
			/* NOTE: The failsafe transition to fixed-wing was removed because it can result in an
			 * unsafe flying state. */
			resetTransitionStates();
			_vtol_mode = vtol_mode::TRANSITION_TO_FW;

		} else if (_vtol_mode == vtol_mode::FW_MODE) {
			// in fw mode
			_vtol_mode = vtol_mode::FW_MODE;
			mc_weight = 0.0f;

		} else if (_vtol_mode == vtol_mode::TRANSITION_TO_FW) {

			if (isFrontTransitionCompleted()) {
				_vtol_mode = vtol_mode::FW_MODE;

				// don't set pusher throttle here as it's being ramped up elsewhere
				_trans_finished_ts = hrt_absolute_time();
			}
		}
	}

	_mc_roll_weight = mc_weight;
	_mc_pitch_weight = mc_weight;
	_mc_yaw_weight = mc_weight;
	_mc_throttle_weight = mc_weight;

	// map specific control phases to simple control modes
	switch (_vtol_mode) {
	case vtol_mode::MC_MODE:
		_common_vtol_mode = mode::ROTARY_WING;
		break;

	case vtol_mode::FW_MODE:
		_common_vtol_mode = mode::FIXED_WING;
		break;

	case vtol_mode::TRANSITION_TO_FW:
		_common_vtol_mode = mode::TRANSITION_TO_FW;
		break;

	case vtol_mode::TRANSITION_TO_MC:
		_common_vtol_mode = mode::TRANSITION_TO_MC;
		break;
	}
}

void Standard::update_transition_state()
{
	const hrt_abstime now = hrt_absolute_time();
	float mc_weight = 1.0f;

	VtolType::update_transition_state();

	// we get attitude setpoint from a multirotor flighttask if climbrate is controlled.
	// in any other case the fixed wing attitude controller publishes attitude setpoint from manual stick input.
	if (_v_control_mode->flag_control_climb_rate_enabled) {
		// we need the incoming (virtual) attitude setpoints (both mc and fw) to be recent, otherwise return (means the previous setpoint stays active)
		if (_mc_virtual_att_sp->timestamp < (now - 1_s) || _fw_virtual_att_sp->timestamp < (now - 1_s)) {
			return;
		}

		memcpy(_v_att_sp, _mc_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));
		_v_att_sp->roll_body = _fw_virtual_att_sp->roll_body;

	} else {
		// we need a recent incoming (fw virtual) attitude setpoint, otherwise return (means the previous setpoint stays active)
		if (_fw_virtual_att_sp->timestamp < (now - 1_s)) {
			return;
		}

		memcpy(_v_att_sp, _fw_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));
		_v_att_sp->thrust_body[2] = -_fw_virtual_att_sp->thrust_body[0];
	}

	if (_vtol_mode == vtol_mode::TRANSITION_TO_FW) {
		if (_param_vt_psher_slew.get() <= FLT_EPSILON) {
			// just set the final target throttle value
			_pusher_throttle = _param_vt_f_trans_thr.get();

		} else if (_pusher_throttle <= _param_vt_f_trans_thr.get()) {
			// ramp up throttle to the target throttle value
			const float dt = math::min((now - _last_time_pusher_transition_update) / 1e6f, 0.05f);
			_pusher_throttle = math::min(_pusher_throttle +
						     _param_vt_psher_slew.get() * dt, _param_vt_f_trans_thr.get());

			_last_time_pusher_transition_update = now;
		}

		_airspeed_trans_blend_margin = getTransitionAirspeed() - getBlendAirspeed();

		// do blending of mc and fw controls if a blending airspeed has been provided and the minimum transition time has passed
		if (_airspeed_trans_blend_margin > 0.0f &&
		    PX4_ISFINITE(_airspeed_validated->calibrated_airspeed_m_s) &&
		    _airspeed_validated->calibrated_airspeed_m_s > 0.0f &&
		    _airspeed_validated->calibrated_airspeed_m_s >= getBlendAirspeed() &&
		    _time_since_trans_start > getMinimumFrontTransitionTime()) {

			mc_weight = 1.0f - fabsf(_airspeed_validated->calibrated_airspeed_m_s - getBlendAirspeed()) /
				    _airspeed_trans_blend_margin;
			// time based blending when no airspeed sensor is set

		} else if (!_param_fw_use_airspd.get() || !PX4_ISFINITE(_airspeed_validated->calibrated_airspeed_m_s)) {
			mc_weight = 1.0f - _time_since_trans_start / getMinimumFrontTransitionTime();
			mc_weight = math::constrain(2.0f * mc_weight, 0.0f, 1.0f);

		}

		// ramp up FW_PSP_OFF
		_v_att_sp->pitch_body = math::radians(_param_fw_psp_off.get()) * (1.0f - mc_weight);

		_v_att_sp->thrust_body[0] = _pusher_throttle;

		const Quatf q_sp(Eulerf(_v_att_sp->roll_body, _v_att_sp->pitch_body, _v_att_sp->yaw_body));
		q_sp.copyTo(_v_att_sp->q_d);

	} else if (_vtol_mode == vtol_mode::TRANSITION_TO_MC) {

		if (_v_control_mode->flag_control_climb_rate_enabled) {
			// control backtransition deceleration using pitch.
			_v_att_sp->pitch_body = update_and_get_backtransition_pitch_sp();
		}

		const Quatf q_sp(Eulerf(_v_att_sp->roll_body, _v_att_sp->pitch_body, _v_att_sp->yaw_body));
		q_sp.copyTo(_v_att_sp->q_d);

		_pusher_throttle = 0.0f;

		// continually increase mc attitude control as we transition back to mc mode
		if (_param_vt_b_trans_ramp.get() > FLT_EPSILON) {
			mc_weight = _time_since_trans_start / _param_vt_b_trans_ramp.get();
		}
	}

	mc_weight = math::constrain(mc_weight, 0.0f, 1.0f);

	_mc_roll_weight = mc_weight;
	_mc_pitch_weight = mc_weight;
	_mc_yaw_weight = mc_weight;
	_mc_throttle_weight = mc_weight;
}

void Standard::update_mc_state()
{
	VtolType::update_mc_state();

	_pusher_throttle = VtolType::pusher_assist();
}

void Standard::update_fw_state()
{
	VtolType::update_fw_state();
}

/**
 * Prepare message to actuators with data from mc and fw attitude controllers. An mc attitude weighting will determine
 * what proportion of control should be applied to each of the control groups (mc and fw).
 */
void Standard::fill_actuator_outputs()
{
	_torque_setpoint_0->timestamp = hrt_absolute_time();
	_torque_setpoint_0->timestamp_sample = _vehicle_torque_setpoint_virtual_mc->timestamp_sample;
	_torque_setpoint_0->xyz[0] = 0.f;
	_torque_setpoint_0->xyz[1] = 0.f;
	_torque_setpoint_0->xyz[2] = 0.f;

	_torque_setpoint_1->timestamp = hrt_absolute_time();
	_torque_setpoint_1->timestamp_sample = _vehicle_torque_setpoint_virtual_fw->timestamp_sample;
	_torque_setpoint_1->xyz[0] = 0.f;
	_torque_setpoint_1->xyz[1] = 0.f;
	_torque_setpoint_1->xyz[2] = 0.f;

	_thrust_setpoint_0->timestamp = hrt_absolute_time();
	_thrust_setpoint_0->timestamp_sample = _vehicle_thrust_setpoint_virtual_mc->timestamp_sample;
	_thrust_setpoint_0->xyz[0] = 0.f;
	_thrust_setpoint_0->xyz[1] = 0.f;
	_thrust_setpoint_0->xyz[2] = 0.f;

	_thrust_setpoint_1->timestamp = hrt_absolute_time();
	_thrust_setpoint_1->timestamp_sample = _vehicle_thrust_setpoint_virtual_fw->timestamp_sample;
	_thrust_setpoint_1->xyz[0] = 0.f;
	_thrust_setpoint_1->xyz[1] = 0.f;
	_thrust_setpoint_1->xyz[2] = 0.f;

	switch (_vtol_mode) {
	case vtol_mode::MC_MODE:

		// MC actuators:
		_torque_setpoint_0->xyz[0] = _vehicle_torque_setpoint_virtual_mc->xyz[0];
		_torque_setpoint_0->xyz[1] = _vehicle_torque_setpoint_virtual_mc->xyz[1];
		_torque_setpoint_0->xyz[2] = _vehicle_torque_setpoint_virtual_mc->xyz[2];
		_thrust_setpoint_0->xyz[2] = _vehicle_thrust_setpoint_virtual_mc->xyz[2];

		// FW actuators:
		if (!_param_vt_elev_mc_lock.get()) {
			_torque_setpoint_1->xyz[0] = _vehicle_torque_setpoint_virtual_fw->xyz[0];
			_torque_setpoint_1->xyz[1] = _vehicle_torque_setpoint_virtual_fw->xyz[1];
		}

		_thrust_setpoint_0->xyz[0] = _pusher_throttle;
		break;

	case vtol_mode::TRANSITION_TO_FW:

	// FALLTHROUGH
	case vtol_mode::TRANSITION_TO_MC:
		// MC actuators:
		_torque_setpoint_0->xyz[0] = _vehicle_torque_setpoint_virtual_mc->xyz[0] * _mc_roll_weight;
		_torque_setpoint_0->xyz[1] = _vehicle_torque_setpoint_virtual_mc->xyz[1] * _mc_pitch_weight;
		_torque_setpoint_0->xyz[2] = _vehicle_torque_setpoint_virtual_mc->xyz[2] * _mc_yaw_weight;
		_thrust_setpoint_0->xyz[2] = _vehicle_thrust_setpoint_virtual_mc->xyz[2] * _mc_throttle_weight;

		// FW actuators
		_torque_setpoint_1->xyz[0] = _vehicle_torque_setpoint_virtual_fw->xyz[0] * (1.f - _mc_roll_weight);
		_torque_setpoint_1->xyz[1] = _vehicle_torque_setpoint_virtual_fw->xyz[1] * (1.f - _mc_pitch_weight);
		_torque_setpoint_1->xyz[2] = _vehicle_torque_setpoint_virtual_fw->xyz[2] * (1.f - _mc_yaw_weight);
		_thrust_setpoint_0->xyz[0] = _pusher_throttle;

		break;

	case vtol_mode::FW_MODE:

		// FW actuators
		_torque_setpoint_1->xyz[0] = _vehicle_torque_setpoint_virtual_fw->xyz[0];
		_torque_setpoint_1->xyz[1] = _vehicle_torque_setpoint_virtual_fw->xyz[1];
		_torque_setpoint_1->xyz[2] = _vehicle_torque_setpoint_virtual_fw->xyz[2];
		_thrust_setpoint_0->xyz[0] = _vehicle_thrust_setpoint_virtual_fw->xyz[0];
		break;
	}
}

void
Standard::waiting_on_tecs()
{
	// keep thrust from transition
	_v_att_sp->thrust_body[0] = _pusher_throttle;
};

void Standard::blendThrottleAfterFrontTransition(float scale)
{
	const float tecs_throttle = _v_att_sp->thrust_body[0];
	_v_att_sp->thrust_body[0] = scale * tecs_throttle + (1.0f - scale) * _pusher_throttle;
}
