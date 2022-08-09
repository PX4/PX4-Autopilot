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
* @file tailsitter.cpp
*
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author David Vorsin     <davidvorsin@gmail.com>
*
*/

#include "tailsitter.h"
#include "vtol_att_control_main.h"

#include <uORB/topics/landing_gear.h>

#define PITCH_TRANSITION_FRONT_P1 -0.5f	// pitch angle to switch to TRANSITION_P2 (28°)
#define PITCH_TRANSITION_BACK -0.25f	// pitch angle to switch to MC

using namespace matrix;

Tailsitter::Tailsitter(VtolAttitudeControl *attc) :
	VtolType(attc)
{
	_vtol_schedule.flight_mode = vtol_mode::MC_MODE;
	_vtol_schedule.transition_start = 0;

	_flag_was_in_trans_mode = false;
}

void
Tailsitter::parameters_update()
{
	VtolType::updateParams();

}

void Tailsitter::update_vtol_state()
{
	/* simple logic using a two way switch to perform transitions.
	 * after flipping the switch the vehicle will start tilting in MC control mode, picking up
	 * forward speed. After the vehicle has picked up enough and sufficient pitch angle the uav will go into FW mode.
	 * For the backtransition the pitch is controlled in MC mode again and switches to full MC control reaching the sufficient pitch angle.
	*/

	float pitch = Eulerf(Quatf(_v_att->q)).theta();

	if (_vtol_vehicle_status->vtol_transition_failsafe) {
		// Failsafe event, switch to MC mode immediately
		_vtol_schedule.flight_mode = vtol_mode::MC_MODE;

		//reset failsafe when FW is no longer requested
		if (!_attc->is_fixed_wing_requested()) {
			_vtol_vehicle_status->vtol_transition_failsafe = false;
		}

	} else if (!_attc->is_fixed_wing_requested()) {

		switch (_vtol_schedule.flight_mode) { // user switchig to MC mode
		case vtol_mode::MC_MODE:
			break;

		case vtol_mode::FW_MODE:
			_vtol_schedule.flight_mode = vtol_mode::TRANSITION_BACK;
			_vtol_schedule.transition_start = hrt_absolute_time();
			break;

		case vtol_mode::TRANSITION_FRONT_P1:
			// failsafe into multicopter mode
			_vtol_schedule.flight_mode = vtol_mode::MC_MODE;
			break;

		case vtol_mode::TRANSITION_BACK:
			float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.transition_start) * 1e-6f;

			// check if we have reached pitch angle to switch to MC mode
			if (pitch >= PITCH_TRANSITION_BACK || time_since_trans_start > _param_vt_b_trans_dur.get()) {
				_vtol_schedule.flight_mode = vtol_mode::MC_MODE;
			}

			break;
		}

	} else {  // user switchig to FW mode

		switch (_vtol_schedule.flight_mode) {
		case vtol_mode::MC_MODE:
			// initialise a front transition
			_vtol_schedule.flight_mode = vtol_mode::TRANSITION_FRONT_P1;
			_vtol_schedule.transition_start = hrt_absolute_time();
			break;

		case vtol_mode::FW_MODE:
			break;

		case vtol_mode::TRANSITION_FRONT_P1: {

				const float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.transition_start) * 1e-6f;


				const bool airspeed_triggers_transition = PX4_ISFINITE(_airspeed_validated->calibrated_airspeed_m_s)
						&& !_param_fw_arsp_mode.get() ;

				bool transition_to_fw = false;

				if (pitch <= PITCH_TRANSITION_FRONT_P1) {
					if (airspeed_triggers_transition) {
						transition_to_fw = _airspeed_validated->calibrated_airspeed_m_s >= _param_vt_arsp_trans.get() ;

					} else {
						transition_to_fw = true;
					}
				}

				transition_to_fw |= can_transition_on_ground();

				if (transition_to_fw) {
					_vtol_schedule.flight_mode = vtol_mode::FW_MODE;
				}

				// check front transition timeout
				if (_param_vt_trans_timeout.get()  > FLT_EPSILON) {
					if (time_since_trans_start > _param_vt_trans_timeout.get()) {
						// transition timeout occured, abort transition
						_attc->quadchute(VtolAttitudeControl::QuadchuteReason::TransitionTimeout);
					}
				}

				break;
			}

		case vtol_mode::TRANSITION_BACK:
			// failsafe into fixed wing mode
			_vtol_schedule.flight_mode = vtol_mode::FW_MODE;
			break;
		}
	}

	// map tailsitter specific control phases to simple control modes
	switch (_vtol_schedule.flight_mode) {
	case vtol_mode::MC_MODE:
		_vtol_mode = mode::ROTARY_WING;
		_flag_was_in_trans_mode = false;
		break;

	case vtol_mode::FW_MODE:
		_vtol_mode = mode::FIXED_WING;
		_flag_was_in_trans_mode = false;
		break;

	case vtol_mode::TRANSITION_FRONT_P1:
		_vtol_mode = mode::TRANSITION_TO_FW;
		break;

	case vtol_mode::TRANSITION_BACK:
		_vtol_mode = mode::TRANSITION_TO_MC;
		break;
	}
}

void Tailsitter::update_transition_state()
{
	const float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.transition_start) * 1e-6f;

	if (!_flag_was_in_trans_mode) {
		_flag_was_in_trans_mode = true;

		if (_vtol_schedule.flight_mode == vtol_mode::TRANSITION_BACK) {
			// calculate rotation axis for transition.
			_q_trans_start = Quatf(_v_att->q);
			Vector3f z = -_q_trans_start.dcm_z();
			_trans_rot_axis = z.cross(Vector3f(0, 0, -1));

			// as heading setpoint we choose the heading given by the direction the vehicle points
			float yaw_sp = atan2f(z(1), z(0));

			// the intial attitude setpoint for a backtransition is a combination of the current fw pitch setpoint,
			// the yaw setpoint and zero roll since we want wings level transition

			const bool run_mc_att_c_in_fw_tailsitter = false; // TODO: just for testing

			if (run_mc_att_c_in_fw_tailsitter) {
				_q_trans_start = Eulerf(0.0f, _mc_virtual_att_sp->pitch_body, yaw_sp);

			} else {
				_q_trans_start = Eulerf(0.0f, _fw_virtual_att_sp->pitch_body, yaw_sp);
			}

			// attitude during transitions are controlled by mc attitude control so rotate the desired attitude to the
			// multirotor frame
			_q_trans_start = _q_trans_start * Quatf(Eulerf(0, -M_PI_2_F, 0));

		} else if (_vtol_schedule.flight_mode == vtol_mode::TRANSITION_FRONT_P1) {
			// initial attitude setpoint for the transition should be with wings level
			_q_trans_start = Eulerf(0.0f, _mc_virtual_att_sp->pitch_body, _mc_virtual_att_sp->yaw_body);
			Vector3f x = Dcmf(Quatf(_v_att->q)) * Vector3f(1, 0, 0);
			_trans_rot_axis = -x.cross(Vector3f(0, 0, -1));
		}

		_q_trans_sp = _q_trans_start;
	}

	// ensure input quaternions are exactly normalized because acosf(1.00001) == NaN
	_q_trans_sp.normalize();

	// tilt angle (zero if vehicle nose points up (hover))
	float cos_tilt = _q_trans_sp(0) * _q_trans_sp(0) - _q_trans_sp(1) * _q_trans_sp(1) - _q_trans_sp(2) *
			 _q_trans_sp(2) + _q_trans_sp(3) * _q_trans_sp(3);
	cos_tilt = cos_tilt >  1.0f ?  1.0f : cos_tilt;
	cos_tilt = cos_tilt < -1.0f ? -1.0f : cos_tilt;
	const float tilt = acosf(cos_tilt);

	if (_vtol_schedule.flight_mode == vtol_mode::TRANSITION_FRONT_P1) {

		// calculate pitching rate - and constrain to at least 0.1s transition time
		const float trans_pitch_rate = M_PI_2_F / math::max(_param_vt_f_trans_dur.get(), 0.1f);

		if (tilt < M_PI_2_F - math::radians(_param_fw_psp_off.get())) {
			_q_trans_sp = Quatf(AxisAnglef(_trans_rot_axis,
						       time_since_trans_start * trans_pitch_rate)) * _q_trans_start;
		}

		// check front transition timeout
		if (_param_vt_trans_timeout.get()  > FLT_EPSILON) {
			if (time_since_trans_start > _param_vt_trans_timeout.get()) {
				// transition timeout occured, abort transition
				_attc->quadchute(VtolAttitudeControl::QuadchuteReason::TransitionTimeout);
			}
		}

	} else if (_vtol_schedule.flight_mode == vtol_mode::TRANSITION_BACK) {

		// calculate pitching rate - and constrain to at least 0.1s transition time
		const float trans_pitch_rate = M_PI_2_F / math::max(_param_vt_b_trans_dur.get(), 0.1f);

		if (!_flag_idle_mc) {
			_flag_idle_mc = set_idle_mc();
		}

		if (tilt > 0.01f) {
			_q_trans_sp = Quatf(AxisAnglef(_trans_rot_axis,
						       time_since_trans_start * trans_pitch_rate)) * _q_trans_start;
		}
	}

	_v_att_sp->thrust_body[2] = _mc_virtual_att_sp->thrust_body[2];

	_v_att_sp->timestamp = hrt_absolute_time();

	const Eulerf euler_sp(_q_trans_sp);
	_v_att_sp->roll_body = euler_sp.phi();
	_v_att_sp->pitch_body = euler_sp.theta();
	_v_att_sp->yaw_body = euler_sp.psi();

	_q_trans_sp.copyTo(_v_att_sp->q_d);
}

void Tailsitter::waiting_on_tecs()
{
	// copy the last trust value from the front transition
	_v_att_sp->thrust_body[0] = _thrust_transition;
}

void Tailsitter::update_fw_state()
{
	if (_flag_idle_mc) {
		_flag_idle_mc = !set_idle_fw();
	}

	const bool run_mc_att_c_in_fw_tailsitter = false; // TODO: just for testing

	// copy virtual attitude setpoint to real attitude setpoint
	if (run_mc_att_c_in_fw_tailsitter) {
		memcpy(_v_att_sp, _mc_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));

	} else {
		memcpy(_v_att_sp, _fw_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));
	}

	check_quadchute_condition();

	// printf("""""""""""""\n");
	// printf("roll sp: %f\n", (double)_v_att_sp->roll_body);
	// printf("pitch sp: (raw) %f\n", (double)_v_att_sp->pitch_body);
	// printf("yaw_sp: %f\n", (double)_v_att_sp->yaw_body);

	// // rotate setpoint 90° down
	// _v_att_sp->roll_body = _v_att_sp->roll_body;
	// _v_att_sp->pitch_body = _v_att_sp->pitch_body - M_PI_2_F;
	// _v_att_sp->yaw_body = _v_att_sp->yaw_body;
	// Quatf q(Eulerf(_v_att_sp->roll_body, _v_att_sp->pitch_body, _v_att_sp->yaw_body));
	// q.copyTo(_v_att_sp->q_d);

	// // calculate rotation axis for transition.
	// _q_trans_start = Quatf(_v_att->q);
	// Vector3f z = -_q_trans_start.dcm_z();
	// _trans_rot_axis = z.cross(Vector3f(0, 0, -1));

	// _q_trans_sp = Quatf(AxisAnglef(_trans_rot_axis, M_PI_2_F));

	// // as heading setpoint we choose the heading given by the direction the vehicle points
	// float yaw_sp = atan2f(z(1), z(0));

	// // the intial attitude setpoint for a backtransition is a combination of the current fw pitch setpoint,
	// // the yaw setpoint and zero roll since we want wings level transition
	// _q_trans_start = Eulerf(0.0f, _fw_virtual_att_sp->pitch_body, yaw_sp);


	// const Eulerf euler_sp(_q_trans_sp);
	// _v_att_sp->roll_body = euler_sp.phi();
	// _v_att_sp->pitch_body = euler_sp.theta();
	// _v_att_sp->yaw_body = euler_sp.psi();


	// printf("roll sp: %f\n", (double)_v_att_sp->roll_body);
	// printf("pitch sp: (rotated) %f\n", (double)_v_att_sp->pitch_body);
	// printf("yaw_sp: %f\n", (double)_v_att_sp->yaw_body);

	// _q_trans_sp.copyTo(_v_att_sp->q_d);

}

/**
* Write data to actuator output topic.
*/
void Tailsitter::fill_actuator_outputs()
{
	auto &mc_in = _actuators_mc_in->control;

	auto &mc_out = _actuators_out_0->control;

	_torque_setpoint_0->timestamp = hrt_absolute_time();
	_torque_setpoint_0->timestamp_sample = _actuators_mc_in->timestamp_sample;
	_torque_setpoint_0->xyz[0] = 0.f;
	_torque_setpoint_0->xyz[1] = 0.f;
	_torque_setpoint_0->xyz[2] = 0.f;

	_thrust_setpoint_0->timestamp = hrt_absolute_time();
	_thrust_setpoint_0->timestamp_sample = _actuators_mc_in->timestamp_sample;
	_thrust_setpoint_0->xyz[0] = 0.f;
	_thrust_setpoint_0->xyz[1] = 0.f;
	_thrust_setpoint_0->xyz[2] = 0.f;

	if (_vtol_schedule.flight_mode == vtol_mode::FW_MODE) {
		_torque_setpoint_0->xyz[2] = mc_in[actuator_controls_s::INDEX_YAW] * _param_vt_fw_difthr_s_r.get();
		_torque_setpoint_0->xyz[1] = mc_in[actuator_controls_s::INDEX_PITCH] * _param_vt_fw_difthr_s_p.get() ;
		_torque_setpoint_0->xyz[0] = mc_in[actuator_controls_s::INDEX_ROLL] * _param_vt_fw_difthr_s_y.get();

	} else {
		_torque_setpoint_0->xyz[0] = mc_in[actuator_controls_s::INDEX_ROLL];
		_torque_setpoint_0->xyz[1] = mc_in[actuator_controls_s::INDEX_PITCH];
		_torque_setpoint_0->xyz[2] = mc_in[actuator_controls_s::INDEX_YAW];
	}

	_thrust_setpoint_0->xyz[2] = -mc_in[actuator_controls_s::INDEX_THROTTLE];

	// Landing Gear
	if (_vtol_schedule.flight_mode == vtol_mode::MC_MODE) {
		mc_out[actuator_controls_s::INDEX_LANDING_GEAR] = landing_gear_s::GEAR_DOWN;

	} else {
		mc_out[actuator_controls_s::INDEX_LANDING_GEAR] = landing_gear_s::GEAR_UP;
	}

	_actuators_out_0->timestamp_sample = _actuators_mc_in->timestamp_sample;
	_actuators_out_0->timestamp = _actuators_out_1->timestamp = hrt_absolute_time();
}
