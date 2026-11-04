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

#define FRONTTRANS_THR_MIN 0.25f
#define BACKTRANS_THROTTLE_DOWNRAMP_DUR_S 0.5f
#define BACKTRANS_THROTTLE_UPRAMP_DUR_S 0.5f

Tiltrotor::Tiltrotor(VtolAttitudeControl *attc) :
	VtolType(attc)
{
}

void
Tiltrotor::parameters_update()
{
	VtolType::updateParams();
}

bool Tiltrotor::isFrontTransitionCompleted()
{
	// the rotors only get tilted beyond VT_TILT_TRANS once the common front transition conditions are met
	// (see update_transition_state), so fully tilted rotors mean the transition is complete
	return _tilt_control >= _param_vt_tilt_fw.get();
}

void Tiltrotor::onTransitionStart()
{
	// A front transition started from hover begins with the rotors at most at VT_TILT_TRANS: the tilt
	// being beyond it is what marks the second part of the transition (tilting forward completely).
	// Hover tilt assist can exceed VT_TILT_TRANS; previously this was snapped back when the second part
	// started. A restart from a back transition keeps the current tilt.
	if (_common_vtol_mode == mode::ROTARY_WING) {
		_tilt_control = math::min(_tilt_control, _param_vt_tilt_trans.get());
	}
}

bool Tiltrotor::isBackTransitionCompleted()
{
	const bool exit_backtransition_tilt_condition = _tilt_control <= (_param_vt_tilt_mc.get() + 0.01f);

	return exit_backtransition_tilt_condition && isBackTransitionSpeedOrTimeExitConditionMet();
}

void Tiltrotor::update_mc_state()
{
	VtolType::update_mc_state();

	_tilt_control = VtolType::pusher_assist() + _param_vt_tilt_mc.get();
	_mc_yaw_weight = 1.0f;
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
		_thrust_transition = -_mc_virtual_att_sp->thrust_body[2];

	} else {
		// we need a recent incoming (fw virtual) attitude setpoint, otherwise return (means the previous setpoint stays active)
		if (_fw_virtual_att_sp->timestamp < (now - 1_s)) {
			return;
		}

		memcpy(_v_att_sp, _fw_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));
		_thrust_transition = _fw_virtual_att_sp->thrust_body[0];
	}


	const Eulerf attitude_setpoint_euler(Quatf(_v_att_sp->q_d));
	float roll_body = attitude_setpoint_euler.phi();
	float pitch_body = attitude_setpoint_euler.theta();
	float yaw_body = attitude_setpoint_euler.psi();

	if (_v_control_mode->flag_control_climb_rate_enabled) {
		roll_body = Eulerf(Quatf(_fw_virtual_att_sp->q_d)).phi();
	}

	// Second part of the front transition: tilt the rotors forward completely once the common front transition
	// conditions (airspeed / time, or on ground) are met. The tilt beyond VT_TILT_TRANS is the memory that this
	// has started, so a later airspeed dip does not stop the rotors.
	const bool tilt_rotors_fully = _common_vtol_mode == mode::TRANSITION_TO_FW
				       && (_tilt_control > _param_vt_tilt_trans.get() || VtolType::isFrontTransitionCompleted());

	if (_common_vtol_mode == mode::TRANSITION_TO_FW && !tilt_rotors_fully) {
		// for the first part of the transition all rotors are enabled

		// tilt rotors forward up to VT_TILT_TRANS
		const float ramped_up_tilt = _param_vt_tilt_mc.get() +
					     fabsf(_param_vt_tilt_trans.get() - _param_vt_tilt_mc.get()) *
					     _time_since_trans_start / _param_vt_f_trans_dur.get() ;

		// only allow increasing tilt (tilt in hover can already be non-zero), never beyond VT_TILT_TRANS
		_tilt_control = math::min(math::max(_tilt_control, ramped_up_tilt), _param_vt_tilt_trans.get());

		// at low speeds give full weight to MC
		_mc_roll_weight = 1.0f;
		_mc_yaw_weight = 1.0f;

		const float airspeed_trans_blend_margin = getTransitionAirspeed() - getBlendAirspeed();

		if (airspeed_trans_blend_margin > FLT_EPSILON &&
		    PX4_ISFINITE(_attc->get_calibrated_airspeed()) &&
		    _attc->get_calibrated_airspeed() >= getBlendAirspeed()) {
			_mc_roll_weight = 1.0f - (_attc->get_calibrated_airspeed() - getBlendAirspeed()) /
					  airspeed_trans_blend_margin;
		}

		// without airspeed do timed weight changes
		const float time_trans_open_loop_margin = getOpenLoopFrontTransitionTime() - getMinimumFrontTransitionTime();

		if ((!PX4_ISFINITE(_attc->get_calibrated_airspeed())) &&
		    time_trans_open_loop_margin > FLT_EPSILON &&
		    _time_since_trans_start > getMinimumFrontTransitionTime()) {
			_mc_roll_weight = 1.0f - (_time_since_trans_start - getMinimumFrontTransitionTime()) /
					  time_trans_open_loop_margin;
		}

		// add minimum throttle for front transition
		_thrust_transition = math::max(_thrust_transition, FRONTTRANS_THR_MIN);

	} else if (tilt_rotors_fully) {
		// the plane is ready to go into fixed wing mode, tilt the rotors forward completely at the
		// rate given by VT_TRANS_P2_DUR for the VT_TILT_TRANS to VT_TILT_FW range
		const float tilt_rate = fabsf(_param_vt_tilt_fw.get() - _param_vt_tilt_trans.get()) /
					math::max(_param_vt_trans_p2_dur.get(), 0.1f);
		_tilt_control = math::min(_tilt_control + tilt_rate * _transition_dt, _param_vt_tilt_fw.get());

		_mc_roll_weight = 0.0f;
		_mc_yaw_weight = 0.0f;

		// add minimum throttle for front transition
		_thrust_transition = math::max(_thrust_transition, FRONTTRANS_THR_MIN);

		// this line is needed such that the fw rate controller is initialized with the current throttle value.
		// if this is not then then there is race condition where the fw rate controller still publishes a zero sample throttle after transition
		_v_att_sp->thrust_body[0] = _thrust_transition;

	} else if (_common_vtol_mode == mode::TRANSITION_TO_MC) {

		// tilt rotors back once motors are idle
		if (_time_since_trans_start > BACKTRANS_THROTTLE_DOWNRAMP_DUR_S) {

			float progress = (_time_since_trans_start - BACKTRANS_THROTTLE_DOWNRAMP_DUR_S) / math::max(_param_vt_bt_tilt_dur.get(),
					 0.1f);
			progress = math::constrain(progress, 0.0f, 1.0f);
			_tilt_control = moveLinear(_param_vt_tilt_fw.get(), _param_vt_tilt_mc.get(), progress);
		}

		_mc_yaw_weight = 1.0f;

		// control backtransition deceleration using pitch.
		if (_v_control_mode->flag_control_climb_rate_enabled) {
			pitch_body = Eulerf(Quatf(_mc_virtual_att_sp->q_d)).theta();
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

	const Quatf q_sp(Eulerf(roll_body, pitch_body, yaw_body));
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

	if (_common_vtol_mode == mode::FIXED_WING) {

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
	if (!_param_vt_elev_mc_lock.get() || _common_vtol_mode != mode::ROTARY_WING) {
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
	return BACKTRANS_THROTTLE_DOWNRAMP_DUR_S + _param_vt_bt_tilt_dur.get();
}

float Tiltrotor::moveLinear(float start, float stop, float progress)
{
	return start + progress * (stop - start);
}

bool Tiltrotor::isFrontTransitionCompletedBase()
{
	return VtolType::isFrontTransitionCompletedBase() && _tilt_control >= _param_vt_tilt_trans.get();
}
