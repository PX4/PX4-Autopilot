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
* @file vtol_type.cpp
*
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author Andreas Antener	<andreas@uaventure.com>
*
*/

#include "vtol_type.h"
#include "vtol_att_control_main.h"

#include <float.h>
#include <px4_platform_common/defines.h>
#include <matrix/math.hpp>

using namespace matrix;

#define THROTTLE_BLENDING_DUR_S 1.0f


VtolType::VtolType(VtolAttitudeControl *att_controller) :
	ModuleParams(nullptr),
	_attc(att_controller),
	_common_vtol_mode(mode::ROTARY_WING)
{
	_v_att = _attc->get_att();
	_v_att_sp = _attc->get_att_sp();
	_mc_virtual_att_sp = _attc->get_mc_virtual_att_sp();
	_fw_virtual_att_sp = _attc->get_fw_virtual_att_sp();
	_v_control_mode = _attc->get_control_mode();
	_vtol_vehicle_status = _attc->get_vtol_vehicle_status();
	_vehicle_torque_setpoint_virtual_mc = _attc->get_vehicle_torque_setpoint_virtual_mc();
	_vehicle_torque_setpoint_virtual_fw = _attc->get_vehicle_torque_setpoint_virtual_fw();
	_vehicle_thrust_setpoint_virtual_mc = _attc->get_vehicle_thrust_setpoint_virtual_mc();
	_vehicle_thrust_setpoint_virtual_fw = _attc->get_vehicle_thrust_setpoint_virtual_fw();
	_torque_setpoint_0 = _attc->get_torque_setpoint_0();
	_torque_setpoint_1 = _attc->get_torque_setpoint_1();
	_thrust_setpoint_0 = _attc->get_thrust_setpoint_0();
	_thrust_setpoint_1 = _attc->get_thrust_setpoint_1();
	_local_pos = _attc->get_local_pos();
	_local_pos_sp = _attc->get_local_pos_sp();
	_airspeed_validated = _attc->get_airspeed();
	_tecs_status = _attc->get_tecs_status();
	_land_detected = _attc->get_land_detected();
}

bool VtolType::init()
{
	return true;
}

void VtolType::parameters_update()
{
	updateParams();

	// make sure that transition speed is above blending speed
	_param_vt_arsp_trans.set(math::max(_param_vt_arsp_trans.get(), _param_vt_arsp_blend.get()));
	// make sure that openloop transition time is above minimum time
	_param_vt_f_tr_ol_tm.set(math::max(_param_vt_f_tr_ol_tm.get(), _param_vt_trans_min_tm.get()));
}

void VtolType::update_mc_state()
{
	resetAccelToPitchPitchIntegrator();

	// copy virtual attitude setpoint to real attitude setpoint
	memcpy(_v_att_sp, _mc_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));

	_mc_roll_weight = 1.0f;
	_mc_pitch_weight = 1.0f;
	_mc_yaw_weight = 1.0f;
	_mc_throttle_weight = 1.0f;
}

void VtolType::update_fw_state()
{
	resetAccelToPitchPitchIntegrator();
	_last_thr_in_fw_mode =  _vehicle_thrust_setpoint_virtual_fw->xyz[0];

	// copy virtual attitude setpoint to real attitude setpoint
	memcpy(_v_att_sp, _fw_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));
	_mc_roll_weight = 0.0f;
	_mc_pitch_weight = 0.0f;
	_mc_yaw_weight = 0.0f;

	// tecs didn't publish an update yet after the transition
	if (_tecs_status->timestamp < _trans_finished_ts) {
		_tecs_running = false;

	} else if (!_tecs_running) {
		_tecs_running = true;
		_tecs_running_ts = hrt_absolute_time();
	}

	// TECS didn't publish yet or the position controller didn't publish yet AFTER tecs
	// only wait on TECS we're in a mode where it is actually running
	if ((!_tecs_running || (_tecs_running && _fw_virtual_att_sp->timestamp <= _tecs_running_ts))
	    && _v_control_mode->flag_control_altitude_enabled) {

		waiting_on_tecs();
		_throttle_blend_start_ts = hrt_absolute_time();

	} else if (shouldBlendThrottleAfterFrontTransition()) {
		const float progress = (float)(hrt_absolute_time() - _throttle_blend_start_ts) * 1e-6f / THROTTLE_BLENDING_DUR_S;

		if (progress >= 1.0f) {
			stopBlendingThrottleAfterFrontTransition();

		} else {
			blendThrottleAfterFrontTransition(progress);
		}
	}

	check_quadchute_condition();
}

void VtolType::update_transition_state()
{
	hrt_abstime t_now = hrt_absolute_time();
	_transition_dt = (float)(t_now - _last_loop_ts) / 1e6f;
	_transition_dt = math::constrain(_transition_dt, 0.0001f, 0.02f);
	_last_loop_ts = t_now;
	_throttle_blend_start_ts = t_now;

	_time_since_trans_start = (float)(t_now - _transition_start_timestamp) * 1e-6f;

	check_quadchute_condition();
}

float VtolType::update_and_get_backtransition_pitch_sp()
{
	// maximum up or down pitch the controller is allowed to demand
	const float pitch_lim = 0.3f;
	const Eulerf euler(Quatf(_v_att->q));

	const float track = atan2f(_local_pos->vy, _local_pos->vx);
	const float accel_body_forward = cosf(track) * _local_pos->ax + sinf(track) * _local_pos->ay;

	// increase the target deceleration setpoint provided to the controller by 20%
	// to make overshooting the transition waypoint less likely in the presence of tracking errors
	const float dec_sp = _param_vt_b_dec_mss.get() * 1.2f;

	// get accel error, positive means decelerating too slow, need to pitch up (must reverse dec_max, as it is a positive number)
	const float accel_error_forward = dec_sp + accel_body_forward;

	const float pitch_sp_new = _accel_to_pitch_integ;

	float integrator_input = _param_vt_b_dec_i.get() * accel_error_forward;

	if ((pitch_sp_new >= pitch_lim && accel_error_forward > 0.0f) ||
	    (pitch_sp_new <= 0.f && accel_error_forward < 0.0f)) {
		integrator_input = 0.0f;
	}

	_accel_to_pitch_integ += integrator_input * _transition_dt;

	// only allow positive (pitch up) pitch setpoint
	return math::constrain(pitch_sp_new, 0.f, pitch_lim);
}

bool VtolType::isFrontTransitionCompleted()
{
	bool ret = isFrontTransitionCompletedBase();

	return ret || can_transition_on_ground();
}

bool VtolType::isFrontTransitionCompletedBase()
{
	// continue the transition to fw mode while monitoring airspeed for a final switch to fw mode
	const bool airspeed_triggers_transition = PX4_ISFINITE(_airspeed_validated->calibrated_airspeed_m_s)
			&& !_param_fw_arsp_mode.get();
	const bool minimum_trans_time_elapsed = _time_since_trans_start > getMinimumFrontTransitionTime();
	const bool openloop_trans_time_elapsed = _time_since_trans_start > getOpenLoopFrontTransitionTime();

	bool transition_to_fw = false;

	if (airspeed_triggers_transition) {
		transition_to_fw = minimum_trans_time_elapsed
				   && _airspeed_validated->calibrated_airspeed_m_s >= _param_vt_arsp_trans.get();

	} else {
		transition_to_fw = openloop_trans_time_elapsed;
	}

	return transition_to_fw;

}

bool VtolType::can_transition_on_ground()
{
	return !_v_control_mode->flag_armed || _land_detected->landed;
}

void VtolType::resetTransitionStates()
{
	_transition_start_timestamp = hrt_absolute_time();
	_time_since_trans_start = 0.f;
	_local_position_z_start_of_transition = _local_pos->z;
}

bool VtolType::isQuadchuteEnabled()
{
	float dist_to_ground = 0.f;
	const float home_position_z = _attc->get_home_position_z();

	if (_local_pos->dist_bottom_valid) {
		dist_to_ground = _local_pos->dist_bottom;

	} else if (PX4_ISFINITE(home_position_z)) {
		dist_to_ground = -(_local_pos->z - home_position_z);

	} else {
		dist_to_ground = -_local_pos->z;

	}

	const bool above_quadchute_altitude_limit = _param_quadchute_max_height.get() > 0
			&& dist_to_ground > (float)_param_quadchute_max_height.get();

	return _v_control_mode->flag_armed &&
	       !_land_detected->landed && !above_quadchute_altitude_limit;

}

bool VtolType::isMinAltBreached()
{
	// fixed-wing minimum altitude
	if (_param_vt_fw_min_alt.get() > FLT_EPSILON) {

		if (-(_local_pos->z) < _param_vt_fw_min_alt.get()) {
			return true;

		}
	}

	return false;
}

bool VtolType::isUncommandedDescent()
{
	const float current_altitude = -_local_pos->z + _local_pos->ref_alt;

	if (_param_vt_qc_alt_loss.get() > FLT_EPSILON && _local_pos->z_valid && _local_pos->z_global
	    && _v_control_mode->flag_control_altitude_enabled
	    && PX4_ISFINITE(_tecs_status->altitude_reference)
	    && (current_altitude < _tecs_status->altitude_reference)
	    && hrt_elapsed_time(&_tecs_status->timestamp) < 1_s) {

		_quadchute_ref_alt = math::min(math::max(_quadchute_ref_alt, current_altitude),
					       _tecs_status->altitude_reference);

		return (_quadchute_ref_alt - current_altitude) > _param_vt_qc_alt_loss.get();

	} else {
		_quadchute_ref_alt = -MAXFLOAT;
	}

	return false;
}

bool VtolType::isFrontTransitionAltitudeLoss()
{
	bool result = false;

	// only run if param set, altitude valid and controlled, and in transition to FW or within 5s of finishing it.
	if (_param_vt_qc_t_alt_loss.get() > FLT_EPSILON && _local_pos->z_valid && _v_control_mode->flag_control_altitude_enabled
	    && (_common_vtol_mode == mode::TRANSITION_TO_FW || hrt_elapsed_time(&_trans_finished_ts) < 5_s)) {

		result = _local_pos->z - _local_position_z_start_of_transition > _param_vt_qc_t_alt_loss.get();
	}

	return result;
}

bool VtolType::isPitchExceeded()
{
	// fixed-wing maximum pitch angle
	if (_param_vt_fw_qc_p.get() > 0) {
		Eulerf euler = Quatf(_v_att->q);

		if (fabsf(euler.theta()) > fabsf(math::radians(static_cast<float>(_param_vt_fw_qc_p.get())))) {
			return true;
		}
	}

	return false;
}

bool VtolType::isRollExceeded()
{
	// fixed-wing maximum roll angle
	if (_param_vt_fw_qc_r.get() > 0) {
		Eulerf euler = Quatf(_v_att->q);

		if (fabsf(euler.phi()) > fabsf(math::radians(static_cast<float>(_param_vt_fw_qc_r.get())))) {
			return true;
		}
	}

	return false;
}

bool VtolType::isFrontTransitionTimeout()
{
	// check front transition timeout
	if (getFrontTransitionTimeout()  > FLT_EPSILON && _common_vtol_mode == mode::TRANSITION_TO_FW) {

		if (_time_since_trans_start > getFrontTransitionTimeout()) {
			// transition timeout occured, abort transition
			return true;
		}
	}

	return false;
}

QuadchuteReason VtolType::getQuadchuteReason()
{
	if (isMinAltBreached()) {
		return QuadchuteReason::MinimumAltBreached;
	}

	if (isUncommandedDescent()) {
		return QuadchuteReason::UncommandedDescent;
	}

	if (isFrontTransitionAltitudeLoss()) {
		return QuadchuteReason::TransitionAltitudeLoss;
	}

	if (isPitchExceeded()) {
		return QuadchuteReason::MaximumPitchExceeded;
	}

	if (isRollExceeded()) {
		return QuadchuteReason::MaximumRollExceeded;
	}

	if (isFrontTransitionTimeout()) {
		return QuadchuteReason::TransitionTimeout;
	}

	return QuadchuteReason::None;
}

void VtolType::handleSpecialExternalCommandQuadchute()
{
	if (_attc->get_transition_command() == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC && _attc->get_immediate_transition()
	    && !_quadchute_command_treated) {
		_attc->quadchute(QuadchuteReason::ExternalCommand);
		_quadchute_command_treated = true;
		_attc->reset_immediate_transition();

	} else {
		_quadchute_command_treated = false;
	}
}

void VtolType::check_quadchute_condition()
{
	handleSpecialExternalCommandQuadchute();

	if (isQuadchuteEnabled()) {
		QuadchuteReason reason = getQuadchuteReason();

		if (reason != QuadchuteReason::None) {
			_attc->quadchute(reason);
		}
	}
}

float VtolType::pusher_assist()
{
	// Altitude above ground is local z-position or altitude above home or distance sensor altitude depending on what's available
	float dist_to_ground = 0.f;
	const float home_position_z = _attc->get_home_position_z();

	if (_local_pos->dist_bottom_valid) {
		dist_to_ground = _local_pos->dist_bottom;

	} else if (PX4_ISFINITE(home_position_z)) {
		dist_to_ground = -(_local_pos->z - home_position_z);

	} else {
		dist_to_ground = -_local_pos->z;

	}

	// disable pusher assist depending on setting of forward_thrust_enable_mode:
	switch (_param_vt_fwd_thrust_en.get()) {
	case DISABLE: // disable in all modes
		return 0.0f;
		break;

	case ENABLE_WITHOUT_LAND: // disable in land mode
		if (_attc->get_pos_sp_triplet()->current.valid
		    && _attc->get_pos_sp_triplet()->current.type == position_setpoint_s::SETPOINT_TYPE_LAND
		    && _v_control_mode->flag_control_auto_enabled) {
			return 0.0f;
		}

		break;

	case ENABLE_ABOVE_MPC_LAND_ALT1: // disable if below MPC_LAND_ALT1
		if (!PX4_ISFINITE(dist_to_ground) || (dist_to_ground < _param_mpc_land_alt1.get())) {
			return 0.0f;
		}

		break;

	case ENABLE_ABOVE_MPC_LAND_ALT2: // disable if below MPC_LAND_ALT2
		if (!PX4_ISFINITE(dist_to_ground) || (dist_to_ground < _param_mpc_land_alt2.get())) {
			return 0.0f;
		}

		break;

	case ENABLE_ABOVE_MPC_LAND_ALT1_WITHOUT_LAND: // disable if below MPC_LAND_ALT1 or in land mode
		if ((_attc->get_pos_sp_triplet()->current.valid
		     && _attc->get_pos_sp_triplet()->current.type == position_setpoint_s::SETPOINT_TYPE_LAND
		     && _v_control_mode->flag_control_auto_enabled) ||
		    (!PX4_ISFINITE(dist_to_ground) || (dist_to_ground < _param_mpc_land_alt1.get()))) {
			return 0.0f;
		}

		break;

	case ENABLE_ABOVE_MPC_LAND_ALT2_WITHOUT_LAND: // disable if below MPC_LAND_ALT2 or in land mode
		if ((_attc->get_pos_sp_triplet()->current.valid
		     && _attc->get_pos_sp_triplet()->current.type == position_setpoint_s::SETPOINT_TYPE_LAND
		     && _v_control_mode->flag_control_auto_enabled) ||
		    (!PX4_ISFINITE(dist_to_ground) || (dist_to_ground < _param_mpc_land_alt2.get()))) {
			return 0.0f;
		}

		break;
	}

	// if the thrust scale param is zero or the drone is not in some position or altitude control mode,
	// then the pusher-for-pitch strategy is disabled and we can return
	if (_param_vt_fwd_thrust_sc.get() < FLT_EPSILON || !(_v_control_mode->flag_control_position_enabled
			|| _v_control_mode->flag_control_altitude_enabled)) {
		return 0.0f;
	}

	// Do not engage pusher assist during a failsafe event (could be a problem with the fixed wing drive)
	if (_attc->get_vtol_vehicle_status()->fixed_wing_system_failure) {
		return 0.0f;
	}

	const Dcmf R(Quatf(_v_att->q));
	const Dcmf R_sp(Quatf(_v_att_sp->q_d));
	const Eulerf euler(R);
	const Eulerf euler_sp(R_sp);

	// direction of desired body z axis represented in earth frame
	Vector3f body_z_sp(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));

	// rotate desired body z axis into new frame which is rotated in z by the current
	// heading of the vehicle. we refer to this as the heading frame.
	Dcmf R_yaw = Eulerf(0.0f, 0.0f, -euler(2));
	body_z_sp = R_yaw * body_z_sp;
	body_z_sp.normalize();

	// calculate the desired pitch seen in the heading frame
	// this value corresponds to the amount the vehicle would try to pitch down
	const float pitch_setpoint = atan2f(body_z_sp(0), body_z_sp(2));

	// normalized pusher support throttle (standard VTOL) or tilt (tiltrotor), initialize to 0
	float forward_thrust = 0.0f;

	float pitch_setpoint_min = math::radians(_param_vt_pitch_min.get());

	if (_attc->get_pos_sp_triplet()->current.valid
	    && _attc->get_pos_sp_triplet()->current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
		pitch_setpoint_min = math::radians(
					     _param_vt_lnd_pitch_min.get()); // set min pitch during LAND (usually lower to generate less lift)
	}

	// only allow pitching down up to threshold, the rest of the desired
	// forward acceleration will be compensated by the pusher/tilt

	if (pitch_setpoint < pitch_setpoint_min) {
		// desired roll angle in heading frame stays the same
		const float roll_new = -asinf(body_z_sp(1));

		forward_thrust = (sinf(pitch_setpoint_min) - sinf(pitch_setpoint)) * _param_vt_fwd_thrust_sc.get();
		// limit forward actuation to [0, 0.9]
		forward_thrust = math::constrain(forward_thrust, 0.0f, 0.9f);

		// Set the pitch to 0 if the pitch limit is negative (pitch down), but allow a positive (pitch up) pitch.
		// This can be used for tiltrotor to make them hover with a positive angle of attack
		const float pitch_new = pitch_setpoint_min > 0.f ? pitch_setpoint_min : 0.f;

		// create corrected desired body z axis in heading frame
		const Dcmf R_tmp = Eulerf(roll_new, pitch_new, 0.0f);
		Vector3f tilt_new(R_tmp(0, 2), R_tmp(1, 2), R_tmp(2, 2));

		// rotate the vector into a new frame which is rotated in z by the desired heading
		// with respect to the earh frame.
		const float yaw_error = wrap_pi(euler_sp(2) - euler(2));
		const Dcmf R_yaw_correction = Eulerf(0.0f, 0.0f, -yaw_error);
		tilt_new = R_yaw_correction * tilt_new;

		// now extract roll and pitch setpoints
		_v_att_sp->pitch_body = atan2f(tilt_new(0), tilt_new(2));
		_v_att_sp->roll_body = -asinf(tilt_new(1));

		const Quatf q_sp(Eulerf(_v_att_sp->roll_body, _v_att_sp->pitch_body, euler_sp(2)));
		q_sp.copyTo(_v_att_sp->q_d);
	}

	return forward_thrust;

}

float VtolType::getFrontTransitionTimeFactor() const
{
	// assumptions: transition_time = transition_true_airspeed / average_acceleration (thrust)
	// transition_true_airspeed ~ sqrt(rho0 / rh0)
	// average_acceleration ~ rho / rho0
	// transition_time ~ sqrt(rho0/rh0) * rho0 / rho

	// low value: hot day at 4000m AMSL with some margin
	// high value: cold day at 0m AMSL with some margin
	const float rho = math::constrain(_attc->getAirDensity(), 0.7f, 1.5f);

	if (PX4_ISFINITE(rho)) {
		float rho0_over_rho = CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C / rho;
		return sqrtf(rho0_over_rho) * rho0_over_rho;
	}

	return 1.0f;
}

float VtolType::getMinimumFrontTransitionTime() const
{
	return getFrontTransitionTimeFactor() * _param_vt_trans_min_tm.get();
}

float VtolType::getFrontTransitionTimeout() const
{
	return getFrontTransitionTimeFactor() * _param_vt_trans_timeout.get();
}

float VtolType::getOpenLoopFrontTransitionTime() const
{
	return getFrontTransitionTimeFactor() * _param_vt_f_tr_ol_tm.get();
}
