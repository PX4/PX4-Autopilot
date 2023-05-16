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
 * @file tiltrotor.cpp
 *
 * @author Roman Bapst 		<bapstroman@gmail.com>
 * @author Andreas Antener 	<andreas@uaventure.com>
 *
*/

#include "tiltrotor.h"
#include "vtol_att_control_main.h"

using namespace matrix;
using namespace time_literals;

#define  FRONTTRANS_THR_MIN 0.25f
#define BACKTRANS_THROTTLE_DOWNRAMP_DUR_S 1.0f
#define BACKTRANS_THROTTLE_UPRAMP_DUR_S 1.0f;
#define BACKTRANS_MOTORS_UPTILT_DUR_S 1.0f;

Tiltrotor::Tiltrotor(VtolAttitudeControl *attc) :
	VtolType(attc)
{
}

void
Tiltrotor::parameters_update()
{
	VtolType::updateParams();
}

void Tiltrotor::update_vtol_state()
{
	/* simple logic using a two way switch to perform transitions.
	 * after flipping the switch the vehicle will start tilting rotors, picking up
	 * forward speed. After the vehicle has picked up enough speed the rotors are tilted
	 * forward completely. For the backtransition the motors simply rotate back.
	*/

	if (_vtol_vehicle_status->fixed_wing_system_failure) {
		// Failsafe event, switch to MC mode immediately
		_vtol_mode = vtol_mode::MC_MODE;

	} else 	if (!_attc->is_fixed_wing_requested()) {

		// plane is in multicopter mode
		switch (_vtol_mode) {
		case vtol_mode::MC_MODE:
			break;

		case vtol_mode::FW_MODE:
			resetTransitionStates();
			_vtol_mode = vtol_mode::TRANSITION_BACK;
			break;

		case vtol_mode::TRANSITION_FRONT_P1:
			// failsafe into multicopter mode
			_vtol_mode = vtol_mode::MC_MODE;
			break;

		case vtol_mode::TRANSITION_FRONT_P2:
			// failsafe into multicopter mode
			_vtol_mode = vtol_mode::MC_MODE;
			break;

		case vtol_mode::TRANSITION_BACK:
			const bool exit_backtransition_tilt_condition = _tilt_control <= (_param_vt_tilt_mc.get() + 0.01f);

			// speed exit condition: use ground if valid, otherwise airspeed
			bool exit_backtransition_speed_condition = false;

			if (_local_pos->v_xy_valid) {
				const Dcmf R_to_body(Quatf(_v_att->q).inversed());
				const Vector3f vel = R_to_body * Vector3f(_local_pos->vx, _local_pos->vy, _local_pos->vz);
				exit_backtransition_speed_condition = vel(0) < _param_mpc_xy_cruise.get() ;

			} else if (PX4_ISFINITE(_airspeed_validated->calibrated_airspeed_m_s)) {
				exit_backtransition_speed_condition = _airspeed_validated->calibrated_airspeed_m_s < _param_mpc_xy_cruise.get() ;
			}

			const bool exit_backtransition_time_condition = _time_since_trans_start > _param_vt_b_trans_dur.get() ;

			if (exit_backtransition_tilt_condition && (exit_backtransition_speed_condition || exit_backtransition_time_condition)) {
				_vtol_mode = vtol_mode::MC_MODE;
			}

			break;
		}

	} else {

		switch (_vtol_mode) {
		case vtol_mode::MC_MODE:
			// initialise a front transition
			resetTransitionStates();
			_vtol_mode = vtol_mode::TRANSITION_FRONT_P1;
			break;

		case vtol_mode::FW_MODE:
			break;

		case vtol_mode::TRANSITION_FRONT_P1: {
				if (isFrontTransitionCompleted()) {
					_vtol_mode = vtol_mode::TRANSITION_FRONT_P2;
					_trans_finished_ts = hrt_absolute_time();
					resetTransitionStates();
				}

				break;
			}

		case vtol_mode::TRANSITION_FRONT_P2:

			// if the rotors have been tilted completely we switch to fw mode
			if (_tilt_control >= _param_vt_tilt_fw.get()) {
				_vtol_mode = vtol_mode::FW_MODE;
				_tilt_control = _param_vt_tilt_fw.get();
			}

			break;

		case vtol_mode::TRANSITION_BACK:
			// failsafe into fixed wing mode
			_vtol_mode = vtol_mode::FW_MODE;
			break;
		}
	}

	// map tiltrotor specific control phases to simple control modes
	switch (_vtol_mode) {
	case vtol_mode::MC_MODE:
		_common_vtol_mode = mode::ROTARY_WING;
		break;

	case vtol_mode::FW_MODE:
		_common_vtol_mode = mode::FIXED_WING;
		break;

	case vtol_mode::TRANSITION_FRONT_P1:
	case vtol_mode::TRANSITION_FRONT_P2:
		_common_vtol_mode = mode::TRANSITION_TO_FW;
		break;

	case vtol_mode::TRANSITION_BACK:
		_common_vtol_mode = mode::TRANSITION_TO_MC;
		break;
	}
}

void Tiltrotor::update_mc_state()
{
	VtolType::update_mc_state();

	/*Motor spin up: define the first second after arming as motor spin up time, during which
	* the tilt is set to the value of VT_TILT_SPINUP. This allows the user to set a spin up
	* tilt angle in case the propellers don't spin up smoothly in full upright (MC mode) position.
	*/

	const int spin_up_duration_p1 = 1000_ms; // duration of 1st phase of spinup (at fixed tilt)
	const int spin_up_duration_p2 = 700_ms; // duration of 2nd phase of spinup (transition from spinup tilt to mc tilt)

	// reset this timestamp while disarmed
	if (!_v_control_mode->flag_armed) {
		_last_timestamp_disarmed = hrt_absolute_time();
		_tilt_motors_for_startup = _param_vt_tilt_spinup.get() > 0.01f; // spinup phase only required if spinup tilt > 0

	} else if (_tilt_motors_for_startup) {
		// leave motors tilted forward after arming to allow them to spin up easier
		if (hrt_absolute_time() - _last_timestamp_disarmed > (spin_up_duration_p1 + spin_up_duration_p2)) {
			_tilt_motors_for_startup = false;
		}
	}

	if (_tilt_motors_for_startup) {
		if (hrt_absolute_time() - _last_timestamp_disarmed < spin_up_duration_p1) {
			_tilt_control = _param_vt_tilt_spinup.get();

		} else {
			// duration phase 2: begin to adapt tilt to multicopter tilt
			float delta_tilt = (_param_vt_tilt_mc.get() - _param_vt_tilt_spinup.get());
			_tilt_control = _param_vt_tilt_spinup.get() + delta_tilt / spin_up_duration_p2 * (hrt_absolute_time() -
					(_last_timestamp_disarmed + spin_up_duration_p1));
		}

		_mc_yaw_weight = 0.0f; //disable yaw control during spinup

	} else {
		// normal operation
		_tilt_control = VtolType::pusher_assist() + _param_vt_tilt_mc.get();
		_mc_yaw_weight = 1.0f;
	}
}

void Tiltrotor::update_fw_state()
{
	VtolType::update_fw_state();

	// this is needed to avoid a race condition when entering backtransition when the mc rate controller publishes
	// a zero throttle value
	_v_att_sp->thrust_body[2] = -_v_att_sp->thrust_body[0];

	// make sure motors are tilted forward
	_tilt_control = _param_vt_tilt_fw.get();
}

void Tiltrotor::update_transition_state()
{
	VtolType::update_transition_state();

	const hrt_abstime now = hrt_absolute_time();

	// we get attitude setpoint from a multirotor flighttask if altitude is controlled.
	// in any other case the fixed wing attitude controller publishes attitude setpoint from manual stick input.
	if (_v_control_mode->flag_control_climb_rate_enabled) {
		// we need the incoming (virtual) attitude setpoints (both mc and fw) to be recent, otherwise return (means the previous setpoint stays active)
		if (_mc_virtual_att_sp->timestamp < (now - 1_s) || _fw_virtual_att_sp->timestamp < (now - 1_s)) {
			return;
		}

		memcpy(_v_att_sp, _mc_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));
		_v_att_sp->roll_body = _fw_virtual_att_sp->roll_body;
		_thrust_transition = -_mc_virtual_att_sp->thrust_body[2];

	} else {
		// we need a recent incoming (fw virtual) attitude setpoint, otherwise return (means the previous setpoint stays active)
		if (_fw_virtual_att_sp->timestamp < (now - 1_s)) {
			return;
		}

		memcpy(_v_att_sp, _fw_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));
		_thrust_transition = _fw_virtual_att_sp->thrust_body[0];
	}


	if (_vtol_mode == vtol_mode::TRANSITION_FRONT_P1) {
		// for the first part of the transition all rotors are enabled

		// tilt rotors forward up to certain angle
		if (_tilt_control <= _param_vt_tilt_trans.get()) {
			const float ramped_up_tilt = _param_vt_tilt_mc.get() +
						     fabsf(_param_vt_tilt_trans.get() - _param_vt_tilt_mc.get()) *
						     _time_since_trans_start / _param_vt_f_trans_dur.get() ;

			// only allow increasing tilt (tilt in hover can already be non-zero)
			_tilt_control = math::max(_tilt_control, ramped_up_tilt);
		}

		// at low speeds give full weight to MC
		_mc_roll_weight = 1.0f;
		_mc_yaw_weight = 1.0f;

		if (!_param_fw_arsp_mode.get()  && PX4_ISFINITE(_airspeed_validated->calibrated_airspeed_m_s) &&
		    _airspeed_validated->calibrated_airspeed_m_s >= _param_vt_arsp_blend.get()) {
			const float weight = 1.0f - (_airspeed_validated->calibrated_airspeed_m_s - _param_vt_arsp_blend.get()) /
					     (_param_vt_arsp_trans.get()  - _param_vt_arsp_blend.get());
			_mc_roll_weight = weight;
			_mc_yaw_weight = weight;
		}

		// without airspeed do timed weight changes
		if ((_param_fw_arsp_mode.get() || !PX4_ISFINITE(_airspeed_validated->calibrated_airspeed_m_s)) &&
		    _time_since_trans_start > getMinimumFrontTransitionTime()) {
			_mc_roll_weight = 1.0f - (_time_since_trans_start - getMinimumFrontTransitionTime()) /
					  (getOpenLoopFrontTransitionTime() - getMinimumFrontTransitionTime());
			_mc_yaw_weight = _mc_roll_weight;
		}

		// add minimum throttle for front transition
		_thrust_transition = math::max(_thrust_transition, FRONTTRANS_THR_MIN);

	} else if (_vtol_mode == vtol_mode::TRANSITION_FRONT_P2) {
		// the plane is ready to go into fixed wing mode, tilt the rotors forward completely
		_tilt_control = math::constrain(_param_vt_tilt_trans.get() +
						fabsf(_param_vt_tilt_fw.get() - _param_vt_tilt_trans.get()) * _time_since_trans_start /
						_param_vt_trans_p2_dur.get(), _param_vt_tilt_trans.get(), _param_vt_tilt_fw.get());

		_mc_roll_weight = 0.0f;
		_mc_yaw_weight = 0.0f;

		// add minimum throttle for front transition
		_thrust_transition = math::max(_thrust_transition, FRONTTRANS_THR_MIN);

		// this line is needed such that the fw rate controller is initialized with the current throttle value.
		// if this is not then then there is race condition where the fw rate controller still publishes a zero sample throttle after transition
		_v_att_sp->thrust_body[0] = _thrust_transition;

	} else if (_vtol_mode == vtol_mode::TRANSITION_BACK) {

		// tilt rotors back once motors are idle
		if (_time_since_trans_start > BACKTRANS_THROTTLE_DOWNRAMP_DUR_S) {

			float progress = (_time_since_trans_start - BACKTRANS_THROTTLE_DOWNRAMP_DUR_S) / BACKTRANS_MOTORS_UPTILT_DUR_S;
			progress = math::constrain(progress, 0.0f, 1.0f);
			_tilt_control = moveLinear(_param_vt_tilt_fw.get(), _param_vt_tilt_mc.get(), progress);
		}

		_mc_yaw_weight = 1.0f;

		// control backtransition deceleration using pitch.
		if (_v_control_mode->flag_control_climb_rate_enabled) {
			_v_att_sp->pitch_body = update_and_get_backtransition_pitch_sp();
		}

		if (_time_since_trans_start < BACKTRANS_THROTTLE_DOWNRAMP_DUR_S) {
			// blend throttle from FW value to 0
			_mc_throttle_weight = 1.0f;
			const float target_throttle = 0.0f;
			const float progress = _time_since_trans_start / BACKTRANS_THROTTLE_DOWNRAMP_DUR_S;
			blendThrottleDuringBacktransition(progress, target_throttle);

		} else if (_time_since_trans_start < timeUntilMotorsAreUp()) {
			// while we quickly rotate back the motors keep throttle at idle

			// turn on all MC motors
			_mc_throttle_weight = 0.0f;
			_mc_roll_weight = 0.0f;
			_mc_pitch_weight = 0.0f;

		} else {
			_mc_roll_weight = 1.0f;
			_mc_pitch_weight = 1.0f;
			// slowly ramp up throttle to avoid step inputs
			float progress = (_time_since_trans_start - timeUntilMotorsAreUp()) / BACKTRANS_THROTTLE_UPRAMP_DUR_S;
			progress = math::constrain(progress, 0.0f, 1.0f);
			_mc_throttle_weight = moveLinear(0.0f, 1.0f, progress);
		}
	}


	_v_att_sp->thrust_body[2] = -_thrust_transition;

	const Quatf q_sp(Eulerf(_v_att_sp->roll_body, _v_att_sp->pitch_body, _v_att_sp->yaw_body));
	q_sp.copyTo(_v_att_sp->q_d);

	_mc_roll_weight = math::constrain(_mc_roll_weight, 0.0f, 1.0f);
	_mc_yaw_weight = math::constrain(_mc_yaw_weight, 0.0f, 1.0f);
	_mc_throttle_weight = math::constrain(_mc_throttle_weight, 0.0f, 1.0f);
}

void Tiltrotor::waiting_on_tecs()
{
	// keep multicopter thrust until we get data from TECS
	_v_att_sp->thrust_body[0] = _thrust_transition;
}

void Tiltrotor::fill_actuator_outputs()
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

	// Multirotor output
	_torque_setpoint_0->xyz[0] = _vehicle_torque_setpoint_virtual_mc->xyz[0] * _mc_roll_weight;
	_torque_setpoint_0->xyz[1] = _vehicle_torque_setpoint_virtual_mc->xyz[1] * _mc_pitch_weight;
	_torque_setpoint_0->xyz[2] = _vehicle_torque_setpoint_virtual_mc->xyz[2] * _mc_yaw_weight;

	// Special case tiltrotor: instead of passing a 3D thrust vector (that would mostly have a x-component in FW, and z in MC),
	// pass the vector magnitude and collective tilt separately. MC also needs collective thrust on z.
	// Passing 3D thrust plus tilt is not feasible as they
	// can't be allocated independently, and with the current controller it's not possible to have collective tilt calculated
	// by the allocator directly.
	float collective_thrust_normalized_setpoint = 0.f;

	if (_vtol_mode == vtol_mode::FW_MODE) {

		collective_thrust_normalized_setpoint = _vehicle_thrust_setpoint_virtual_fw->xyz[0];
		_thrust_setpoint_0->xyz[2] = -collective_thrust_normalized_setpoint;

		/* allow differential thrust if enabled */
		if (_param_vt_fw_difthr_en.get() & static_cast<int32_t>(VtFwDifthrEnBits::YAW_BIT)) {
			_torque_setpoint_0->xyz[2] = _vehicle_torque_setpoint_virtual_fw->xyz[2] * _param_vt_fw_difthr_s_y.get() ;
		}

	} else {
		collective_thrust_normalized_setpoint = -_vehicle_thrust_setpoint_virtual_mc->xyz[2] * _mc_throttle_weight;
		_thrust_setpoint_0->xyz[2] = -collective_thrust_normalized_setpoint;
	}

	// Fixed wing output
	if (!_param_vt_elev_mc_lock.get() || _vtol_mode != vtol_mode::MC_MODE) {
		_torque_setpoint_1->xyz[0] = _vehicle_torque_setpoint_virtual_fw->xyz[0];
		_torque_setpoint_1->xyz[1] = _vehicle_torque_setpoint_virtual_fw->xyz[1];
		_torque_setpoint_1->xyz[2] = _vehicle_torque_setpoint_virtual_fw->xyz[2];
	}

	// publish tiltrotor extra controls
	tiltrotor_extra_controls_s tiltrotor_extra_controls = {};
	tiltrotor_extra_controls.collective_tilt_normalized_setpoint = _tilt_control;
	tiltrotor_extra_controls.collective_thrust_normalized_setpoint = collective_thrust_normalized_setpoint;
	tiltrotor_extra_controls.timestamp = hrt_absolute_time();
	_tiltrotor_extra_controls_pub.publish(tiltrotor_extra_controls);
}

void Tiltrotor::blendThrottleAfterFrontTransition(float scale)
{
	const float tecs_throttle = _v_att_sp->thrust_body[0];

	_v_att_sp->thrust_body[0] = scale * tecs_throttle + (1.0f - scale) * _thrust_transition;
}

void Tiltrotor::blendThrottleDuringBacktransition(float scale, float target_throttle)
{
	_thrust_transition = scale * target_throttle + (1.0f - scale) * _last_thr_in_fw_mode;
}

float Tiltrotor::timeUntilMotorsAreUp()
{
	return BACKTRANS_THROTTLE_DOWNRAMP_DUR_S + BACKTRANS_MOTORS_UPTILT_DUR_S;
}

float Tiltrotor::moveLinear(float start, float stop, float progress)
{
	return start + progress * (stop - start);
}

bool Tiltrotor::isFrontTransitionCompletedBase()
{
	return VtolType::isFrontTransitionCompletedBase() && _tilt_control >= _param_vt_tilt_trans.get();
}
