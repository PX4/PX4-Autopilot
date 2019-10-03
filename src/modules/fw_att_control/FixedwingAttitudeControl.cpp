/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include "FixedwingAttitudeControl.hpp"

#include <vtol_att_control/vtol_type.h>

using namespace time_literals;

/**
 * Fixedwing attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int fw_att_control_main(int argc, char *argv[]);

FixedwingAttitudeControl::FixedwingAttitudeControl() :
	WorkItem(MODULE_NAME, px4::wq_configurations::att_pos_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, "fw_att_control: cycle"))
{
	// check if VTOL first
	vehicle_status_poll();

	_parameter_handles.p_tc = param_find("FW_P_TC");
	_parameter_handles.p_p = param_find("FW_PR_P");
	_parameter_handles.p_i = param_find("FW_PR_I");
	_parameter_handles.p_ff = param_find("FW_PR_FF");
	_parameter_handles.p_rmax_pos = param_find("FW_P_RMAX_POS");
	_parameter_handles.p_rmax_neg = param_find("FW_P_RMAX_NEG");
	_parameter_handles.p_integrator_max = param_find("FW_PR_IMAX");

	_parameter_handles.r_tc = param_find("FW_R_TC");
	_parameter_handles.r_p = param_find("FW_RR_P");
	_parameter_handles.r_i = param_find("FW_RR_I");
	_parameter_handles.r_ff = param_find("FW_RR_FF");
	_parameter_handles.r_integrator_max = param_find("FW_RR_IMAX");
	_parameter_handles.r_rmax = param_find("FW_R_RMAX");

	_parameter_handles.y_p = param_find("FW_YR_P");
	_parameter_handles.y_i = param_find("FW_YR_I");
	_parameter_handles.y_ff = param_find("FW_YR_FF");
	_parameter_handles.y_integrator_max = param_find("FW_YR_IMAX");
	_parameter_handles.y_rmax = param_find("FW_Y_RMAX");
	_parameter_handles.roll_to_yaw_ff = param_find("FW_RLL_TO_YAW_FF");

	_parameter_handles.w_en = param_find("FW_W_EN");
	_parameter_handles.w_p = param_find("FW_WR_P");
	_parameter_handles.w_i = param_find("FW_WR_I");
	_parameter_handles.w_ff = param_find("FW_WR_FF");
	_parameter_handles.w_integrator_max = param_find("FW_WR_IMAX");
	_parameter_handles.w_rmax = param_find("FW_W_RMAX");

	_parameter_handles.airspeed_min = param_find("FW_AIRSPD_MIN");
	_parameter_handles.airspeed_trim = param_find("FW_AIRSPD_TRIM");
	_parameter_handles.airspeed_max = param_find("FW_AIRSPD_MAX");

	_parameter_handles.trim_roll = param_find("TRIM_ROLL");
	_parameter_handles.trim_pitch = param_find("TRIM_PITCH");
	_parameter_handles.trim_yaw = param_find("TRIM_YAW");
	_parameter_handles.dtrim_roll_vmin = param_find("FW_DTRIM_R_VMIN");
	_parameter_handles.dtrim_pitch_vmin = param_find("FW_DTRIM_P_VMIN");
	_parameter_handles.dtrim_yaw_vmin = param_find("FW_DTRIM_Y_VMIN");
	_parameter_handles.dtrim_roll_vmax = param_find("FW_DTRIM_R_VMAX");
	_parameter_handles.dtrim_pitch_vmax = param_find("FW_DTRIM_P_VMAX");
	_parameter_handles.dtrim_yaw_vmax = param_find("FW_DTRIM_Y_VMAX");
	_parameter_handles.dtrim_roll_flaps = param_find("FW_DTRIM_R_FLPS");
	_parameter_handles.dtrim_pitch_flaps = param_find("FW_DTRIM_P_FLPS");
	_parameter_handles.rollsp_offset_deg = param_find("FW_RSP_OFF");
	_parameter_handles.pitchsp_offset_deg = param_find("FW_PSP_OFF");

	_parameter_handles.man_roll_max = param_find("FW_MAN_R_MAX");
	_parameter_handles.man_pitch_max = param_find("FW_MAN_P_MAX");
	_parameter_handles.man_roll_scale = param_find("FW_MAN_R_SC");
	_parameter_handles.man_pitch_scale = param_find("FW_MAN_P_SC");
	_parameter_handles.man_yaw_scale = param_find("FW_MAN_Y_SC");

	_parameter_handles.acro_max_x_rate = param_find("FW_ACRO_X_MAX");
	_parameter_handles.acro_max_y_rate = param_find("FW_ACRO_Y_MAX");
	_parameter_handles.acro_max_z_rate = param_find("FW_ACRO_Z_MAX");

	_parameter_handles.flaps_scale = param_find("FW_FLAPS_SCL");
	_parameter_handles.flaps_takeoff_scale = param_find("FW_FLAPS_TO_SCL");
	_parameter_handles.flaps_land_scale = param_find("FW_FLAPS_LND_SCL");
	_parameter_handles.flaperon_scale = param_find("FW_FLAPERON_SCL");

	_parameter_handles.rattitude_thres = param_find("FW_RATT_TH");

	_parameter_handles.bat_scale_en = param_find("FW_BAT_SCALE_EN");
	_parameter_handles.airspeed_mode = param_find("FW_ARSP_MODE");

	/* fetch initial parameter values */
	parameters_update();

	// set initial maximum body rate setpoints
	_roll_ctrl.set_max_rate(_parameters.acro_max_x_rate_rad);
	_pitch_ctrl.set_max_rate_pos(_parameters.acro_max_y_rate_rad);
	_pitch_ctrl.set_max_rate_neg(_parameters.acro_max_y_rate_rad);
	_yaw_ctrl.set_max_rate(_parameters.acro_max_z_rate_rad);
}

FixedwingAttitudeControl::~FixedwingAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
FixedwingAttitudeControl::init()
{
	if (!_att_sub.registerCallback()) {
		PX4_ERR("vehicle attitude callback registration failed!");
		return false;
	}

	return true;
}

int
FixedwingAttitudeControl::parameters_update()
{
	int32_t tmp = 0;
	param_get(_parameter_handles.p_tc, &(_parameters.p_tc));
	param_get(_parameter_handles.p_p, &(_parameters.p_p));
	param_get(_parameter_handles.p_i, &(_parameters.p_i));
	param_get(_parameter_handles.p_ff, &(_parameters.p_ff));
	param_get(_parameter_handles.p_rmax_pos, &(_parameters.p_rmax_pos));
	param_get(_parameter_handles.p_rmax_neg, &(_parameters.p_rmax_neg));
	param_get(_parameter_handles.p_integrator_max, &(_parameters.p_integrator_max));

	param_get(_parameter_handles.r_tc, &(_parameters.r_tc));
	param_get(_parameter_handles.r_p, &(_parameters.r_p));
	param_get(_parameter_handles.r_i, &(_parameters.r_i));
	param_get(_parameter_handles.r_ff, &(_parameters.r_ff));

	param_get(_parameter_handles.r_integrator_max, &(_parameters.r_integrator_max));
	param_get(_parameter_handles.r_rmax, &(_parameters.r_rmax));

	param_get(_parameter_handles.y_p, &(_parameters.y_p));
	param_get(_parameter_handles.y_i, &(_parameters.y_i));
	param_get(_parameter_handles.y_ff, &(_parameters.y_ff));
	param_get(_parameter_handles.y_integrator_max, &(_parameters.y_integrator_max));
	param_get(_parameter_handles.y_rmax, &(_parameters.y_rmax));
	param_get(_parameter_handles.roll_to_yaw_ff, &(_parameters.roll_to_yaw_ff));

	param_get(_parameter_handles.w_en, &tmp);
	_parameters.w_en = (tmp == 1);

	param_get(_parameter_handles.w_p, &(_parameters.w_p));
	param_get(_parameter_handles.w_i, &(_parameters.w_i));
	param_get(_parameter_handles.w_ff, &(_parameters.w_ff));
	param_get(_parameter_handles.w_integrator_max, &(_parameters.w_integrator_max));
	param_get(_parameter_handles.w_rmax, &(_parameters.w_rmax));

	param_get(_parameter_handles.airspeed_min, &(_parameters.airspeed_min));
	param_get(_parameter_handles.airspeed_trim, &(_parameters.airspeed_trim));
	param_get(_parameter_handles.airspeed_max, &(_parameters.airspeed_max));

	param_get(_parameter_handles.trim_roll, &(_parameters.trim_roll));
	param_get(_parameter_handles.trim_pitch, &(_parameters.trim_pitch));
	param_get(_parameter_handles.trim_yaw, &(_parameters.trim_yaw));
	param_get(_parameter_handles.dtrim_roll_vmin, &(_parameters.dtrim_roll_vmin));
	param_get(_parameter_handles.dtrim_roll_vmax, &(_parameters.dtrim_roll_vmax));
	param_get(_parameter_handles.dtrim_pitch_vmin, &(_parameters.dtrim_pitch_vmin));
	param_get(_parameter_handles.dtrim_pitch_vmax, &(_parameters.dtrim_pitch_vmax));
	param_get(_parameter_handles.dtrim_yaw_vmin, &(_parameters.dtrim_yaw_vmin));
	param_get(_parameter_handles.dtrim_yaw_vmax, &(_parameters.dtrim_yaw_vmax));

	param_get(_parameter_handles.dtrim_roll_flaps, &(_parameters.dtrim_roll_flaps));
	param_get(_parameter_handles.dtrim_pitch_flaps, &(_parameters.dtrim_pitch_flaps));

	param_get(_parameter_handles.rollsp_offset_deg, &(_parameters.rollsp_offset_deg));
	param_get(_parameter_handles.pitchsp_offset_deg, &(_parameters.pitchsp_offset_deg));
	_parameters.rollsp_offset_rad = math::radians(_parameters.rollsp_offset_deg);
	_parameters.pitchsp_offset_rad = math::radians(_parameters.pitchsp_offset_deg);
	param_get(_parameter_handles.man_roll_max, &(_parameters.man_roll_max));
	param_get(_parameter_handles.man_pitch_max, &(_parameters.man_pitch_max));
	_parameters.man_roll_max = math::radians(_parameters.man_roll_max);
	_parameters.man_pitch_max = math::radians(_parameters.man_pitch_max);
	param_get(_parameter_handles.man_roll_scale, &(_parameters.man_roll_scale));
	param_get(_parameter_handles.man_pitch_scale, &(_parameters.man_pitch_scale));
	param_get(_parameter_handles.man_yaw_scale, &(_parameters.man_yaw_scale));

	param_get(_parameter_handles.acro_max_x_rate, &(_parameters.acro_max_x_rate_rad));
	param_get(_parameter_handles.acro_max_y_rate, &(_parameters.acro_max_y_rate_rad));
	param_get(_parameter_handles.acro_max_z_rate, &(_parameters.acro_max_z_rate_rad));
	_parameters.acro_max_x_rate_rad = math::radians(_parameters.acro_max_x_rate_rad);
	_parameters.acro_max_y_rate_rad = math::radians(_parameters.acro_max_y_rate_rad);
	_parameters.acro_max_z_rate_rad = math::radians(_parameters.acro_max_z_rate_rad);

	param_get(_parameter_handles.flaps_scale, &_parameters.flaps_scale);
	param_get(_parameter_handles.flaps_takeoff_scale, &_parameters.flaps_takeoff_scale);
	param_get(_parameter_handles.flaps_land_scale, &_parameters.flaps_land_scale);
	param_get(_parameter_handles.flaperon_scale, &_parameters.flaperon_scale);

	param_get(_parameter_handles.rattitude_thres, &_parameters.rattitude_thres);

	param_get(_parameter_handles.bat_scale_en, &_parameters.bat_scale_en);

	param_get(_parameter_handles.airspeed_mode, &tmp);
	_parameters.airspeed_disabled = (tmp == 1);

	/* pitch control parameters */
	_pitch_ctrl.set_time_constant(_parameters.p_tc);
	_pitch_ctrl.set_k_p(_parameters.p_p);
	_pitch_ctrl.set_k_i(_parameters.p_i);
	_pitch_ctrl.set_k_ff(_parameters.p_ff);
	_pitch_ctrl.set_integrator_max(_parameters.p_integrator_max);

	/* roll control parameters */
	_roll_ctrl.set_time_constant(_parameters.r_tc);
	_roll_ctrl.set_k_p(_parameters.r_p);
	_roll_ctrl.set_k_i(_parameters.r_i);
	_roll_ctrl.set_k_ff(_parameters.r_ff);
	_roll_ctrl.set_integrator_max(_parameters.r_integrator_max);

	/* yaw control parameters */
	_yaw_ctrl.set_k_p(_parameters.y_p);
	_yaw_ctrl.set_k_i(_parameters.y_i);
	_yaw_ctrl.set_k_ff(_parameters.y_ff);
	_yaw_ctrl.set_integrator_max(_parameters.y_integrator_max);

	/* wheel control parameters */
	_wheel_ctrl.set_k_p(_parameters.w_p);
	_wheel_ctrl.set_k_i(_parameters.w_i);
	_wheel_ctrl.set_k_ff(_parameters.w_ff);
	_wheel_ctrl.set_integrator_max(_parameters.w_integrator_max);
	_wheel_ctrl.set_max_rate(math::radians(_parameters.w_rmax));

	return PX4_OK;
}

void
FixedwingAttitudeControl::vehicle_control_mode_poll()
{
	_vcontrol_mode_sub.update(&_vcontrol_mode);

	if (_vehicle_status.is_vtol) {
		const bool is_hovering = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
					 && !_vehicle_status.in_transition_mode;
		const bool is_tailsitter_transition = _vehicle_status.in_transition_mode && _is_tailsitter;

		if (is_hovering || is_tailsitter_transition) {
			_vcontrol_mode.flag_control_attitude_enabled = false;
			_vcontrol_mode.flag_control_manual_enabled = false;
		}
	}
}

void
FixedwingAttitudeControl::vehicle_manual_poll()
{
	const bool is_tailsitter_transition = _is_tailsitter && _vehicle_status.in_transition_mode;
	const bool is_fixed_wing = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

	if (_vcontrol_mode.flag_control_manual_enabled && (!is_tailsitter_transition || is_fixed_wing)) {

		// Always copy the new manual setpoint, even if it wasn't updated, to fill the _actuators with valid values
		if (_manual_sub.copy(&_manual)) {

			// Check if we are in rattitude mode and the pilot is above the threshold on pitch
			if (_vcontrol_mode.flag_control_rattitude_enabled) {
				if (fabsf(_manual.y) > _parameters.rattitude_thres || fabsf(_manual.x) > _parameters.rattitude_thres) {
					_vcontrol_mode.flag_control_attitude_enabled = false;
				}
			}

			if (!_vcontrol_mode.flag_control_climb_rate_enabled &&
			    !_vcontrol_mode.flag_control_offboard_enabled) {

				if (_vcontrol_mode.flag_control_attitude_enabled) {
					// STABILIZED mode generate the attitude setpoint from manual user inputs
					_att_sp.timestamp = hrt_absolute_time();
					_att_sp.roll_body = _manual.y * _parameters.man_roll_max + _parameters.rollsp_offset_rad;
					_att_sp.roll_body = math::constrain(_att_sp.roll_body, -_parameters.man_roll_max, _parameters.man_roll_max);
					_att_sp.pitch_body = -_manual.x * _parameters.man_pitch_max + _parameters.pitchsp_offset_rad;
					_att_sp.pitch_body = math::constrain(_att_sp.pitch_body, -_parameters.man_pitch_max, _parameters.man_pitch_max);
					_att_sp.yaw_body = 0.0f;
					_att_sp.thrust_body[0] = _manual.z;

					Quatf q(Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body));
					q.copyTo(_att_sp.q_d);
					_att_sp.q_d_valid = true;

					if (_attitude_sp_pub != nullptr) {
						/* publish the attitude rates setpoint */
						orb_publish(_attitude_setpoint_id, _attitude_sp_pub, &_att_sp);

					} else if (_attitude_setpoint_id) {
						/* advertise the attitude rates setpoint */
						_attitude_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
					}

				} else if (_vcontrol_mode.flag_control_rates_enabled &&
					   !_vcontrol_mode.flag_control_attitude_enabled) {

					// RATE mode we need to generate the rate setpoint from manual user inputs
					_rates_sp.timestamp = hrt_absolute_time();
					_rates_sp.roll = _manual.y * _parameters.acro_max_x_rate_rad;
					_rates_sp.pitch = -_manual.x * _parameters.acro_max_y_rate_rad;
					_rates_sp.yaw = _manual.r * _parameters.acro_max_z_rate_rad;
					_rates_sp.thrust_body[0] = _manual.z;

					_rate_sp_pub.publish(_rates_sp);

				} else {
					/* manual/direct control */
					_actuators.control[actuator_controls_s::INDEX_ROLL] = _manual.y * _parameters.man_roll_scale + _parameters.trim_roll;
					_actuators.control[actuator_controls_s::INDEX_PITCH] = -_manual.x * _parameters.man_pitch_scale +
							_parameters.trim_pitch;
					_actuators.control[actuator_controls_s::INDEX_YAW] = _manual.r * _parameters.man_yaw_scale + _parameters.trim_yaw;
					_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _manual.z;
				}
			}
		}
	}
}

void
FixedwingAttitudeControl::vehicle_attitude_setpoint_poll()
{
	if (_att_sp_sub.update(&_att_sp)) {
		_rates_sp.thrust_body[0] = _att_sp.thrust_body[0];
		_rates_sp.thrust_body[1] = _att_sp.thrust_body[1];
		_rates_sp.thrust_body[2] = _att_sp.thrust_body[2];
	}
}

void
FixedwingAttitudeControl::vehicle_rates_setpoint_poll()
{
	if (_rates_sp_sub.update(&_rates_sp)) {
		if (_is_tailsitter) {
			float tmp = _rates_sp.roll;
			_rates_sp.roll = -_rates_sp.yaw;
			_rates_sp.yaw = tmp;
		}
	}
}

void
FixedwingAttitudeControl::vehicle_status_poll()
{
	if (_vehicle_status_sub.update(&_vehicle_status)) {
		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_actuators_id) {
			if (_vehicle_status.is_vtol) {
				_actuators_id = ORB_ID(actuator_controls_virtual_fw);
				_attitude_setpoint_id = ORB_ID(fw_virtual_attitude_setpoint);

				int32_t vt_type = -1;

				if (param_get(param_find("VT_TYPE"), &vt_type) == PX4_OK) {
					_is_tailsitter = (static_cast<vtol_type>(vt_type) == vtol_type::TAILSITTER);
				}

			} else {
				_actuators_id = ORB_ID(actuator_controls_0);
				_attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
			}
		}
	}
}

void
FixedwingAttitudeControl::vehicle_land_detected_poll()
{
	if (_vehicle_land_detected_sub.updated()) {
		vehicle_land_detected_s vehicle_land_detected {};

		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
			_landed = vehicle_land_detected.landed;
		}
	}
}

float FixedwingAttitudeControl::get_airspeed_and_update_scaling()
{
	_airspeed_sub.update();
	const bool airspeed_valid = PX4_ISFINITE(_airspeed_sub.get().indicated_airspeed_m_s)
				    && (hrt_elapsed_time(&_airspeed_sub.get().timestamp) < 1_s)
				    && !_vehicle_status.aspd_use_inhibit;

	// if no airspeed measurement is available out best guess is to use the trim airspeed
	float airspeed = _parameters.airspeed_trim;

	if (!_parameters.airspeed_disabled && airspeed_valid) {
		/* prevent numerical drama by requiring 0.5 m/s minimal speed */
		airspeed = math::max(0.5f, _airspeed_sub.get().indicated_airspeed_m_s);

	} else {
		// VTOL: if we have no airspeed available and we are in hover mode then assume the lowest airspeed possible
		// this assumption is good as long as the vehicle is not hovering in a headwind which is much larger
		// than the minimum airspeed
		if (_vehicle_status.is_vtol && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		    && !_vehicle_status.in_transition_mode) {
			airspeed = _parameters.airspeed_min;
		}
	}

	/*
	 * For scaling our actuators using anything less than the min (close to stall)
	 * speed doesn't make any sense - its the strongest reasonable deflection we
	 * want to do in flight and its the baseline a human pilot would choose.
	 *
	 * Forcing the scaling to this value allows reasonable handheld tests.
	 */
	const float airspeed_constrained = math::constrain(airspeed, _parameters.airspeed_min, _parameters.airspeed_max);
	_airspeed_scaling = _parameters.airspeed_trim / airspeed_constrained;

	return airspeed;
}

void FixedwingAttitudeControl::Run()
{
	if (should_exit()) {
		_att_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	if (_att_sub.update(&_att)) {

		// only update parameters if they changed
		bool params_updated = _parameter_update_sub.updated();

		// check for parameter updates
		if (params_updated) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			parameters_update();
		}

		/* only run controller if attitude changed */
		static uint64_t last_run = 0;
		float deltaT = math::constrain((hrt_elapsed_time(&last_run) / 1e6f), 0.01f, 0.1f);
		last_run = hrt_absolute_time();

		/* get current rotation matrix and euler angles from control state quaternions */
		matrix::Dcmf R = matrix::Quatf(_att.q);

		vehicle_angular_velocity_s angular_velocity{};
		_vehicle_rates_sub.copy(&angular_velocity);
		float rollspeed = angular_velocity.xyz[0];
		float pitchspeed = angular_velocity.xyz[1];
		float yawspeed = angular_velocity.xyz[2];

		if (_is_tailsitter) {
			/* vehicle is a tailsitter, we need to modify the estimated attitude for fw mode
			 *
			 * Since the VTOL airframe is initialized as a multicopter we need to
			 * modify the estimated attitude for the fixed wing operation.
			 * Since the neutral position of the vehicle in fixed wing mode is -90 degrees rotated around
			 * the pitch axis compared to the neutral position of the vehicle in multicopter mode
			 * we need to swap the roll and the yaw axis (1st and 3rd column) in the rotation matrix.
			 * Additionally, in order to get the correct sign of the pitch, we need to multiply
			 * the new x axis of the rotation matrix with -1
			 *
			 * original:			modified:
			 *
			 * Rxx  Ryx  Rzx		-Rzx  Ryx  Rxx
			 * Rxy	Ryy  Rzy		-Rzy  Ryy  Rxy
			 * Rxz	Ryz  Rzz		-Rzz  Ryz  Rxz
			 * */
			matrix::Dcmf R_adapted = R;		//modified rotation matrix

			/* move z to x */
			R_adapted(0, 0) = R(0, 2);
			R_adapted(1, 0) = R(1, 2);
			R_adapted(2, 0) = R(2, 2);

			/* move x to z */
			R_adapted(0, 2) = R(0, 0);
			R_adapted(1, 2) = R(1, 0);
			R_adapted(2, 2) = R(2, 0);

			/* change direction of pitch (convert to right handed system) */
			R_adapted(0, 0) = -R_adapted(0, 0);
			R_adapted(1, 0) = -R_adapted(1, 0);
			R_adapted(2, 0) = -R_adapted(2, 0);

			/* fill in new attitude data */
			R = R_adapted;

			/* lastly, roll- and yawspeed have to be swaped */
			float helper = rollspeed;
			rollspeed = -yawspeed;
			yawspeed = helper;
		}

		const matrix::Eulerf euler_angles(R);

		vehicle_attitude_setpoint_poll();
		vehicle_status_poll(); // this poll has to be before the control_mode_poll, otherwise rate sp are not published during whole transition
		vehicle_control_mode_poll();
		vehicle_manual_poll();
		_global_pos_sub.update(&_global_pos);
		vehicle_land_detected_poll();

		// the position controller will not emit attitude setpoints in some modes
		// we need to make sure that this flag is reset
		_att_sp.fw_control_yaw = _att_sp.fw_control_yaw && _vcontrol_mode.flag_control_auto_enabled;

		/* lock integrator until control is started */
		bool lock_integrator = !_vcontrol_mode.flag_control_rates_enabled
				       || (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && ! _vehicle_status.in_transition_mode);

		/* Simple handling of failsafe: deploy parachute if failsafe is on */
		if (_vcontrol_mode.flag_control_termination_enabled) {
			_actuators_airframe.control[7] = 1.0f;

		} else {
			_actuators_airframe.control[7] = 0.0f;
		}

		/* if we are in rotary wing mode, do nothing */
		if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !_vehicle_status.is_vtol) {
			perf_end(_loop_perf);
			return;
		}

		control_flaps(deltaT);

		/* decide if in stabilized or full manual control */
		if (_vcontrol_mode.flag_control_rates_enabled) {

			const float airspeed = get_airspeed_and_update_scaling();

			/* Use min airspeed to calculate ground speed scaling region.
			 * Don't scale below gspd_scaling_trim
			 */
			float groundspeed = sqrtf(_global_pos.vel_n * _global_pos.vel_n +
						  _global_pos.vel_e * _global_pos.vel_e);
			float gspd_scaling_trim = (_parameters.airspeed_min * 0.6f);
			float groundspeed_scaler = gspd_scaling_trim / ((groundspeed < gspd_scaling_trim) ? gspd_scaling_trim : groundspeed);

			/* reset integrals where needed */
			if (_att_sp.roll_reset_integral) {
				_roll_ctrl.reset_integrator();
			}

			if (_att_sp.pitch_reset_integral) {
				_pitch_ctrl.reset_integrator();
			}

			if (_att_sp.yaw_reset_integral) {
				_yaw_ctrl.reset_integrator();
				_wheel_ctrl.reset_integrator();
			}

			/* Reset integrators if the aircraft is on ground
			 * or a multicopter (but not transitioning VTOL)
			 */
			if (_landed
			    || (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
				&& !_vehicle_status.in_transition_mode)) {

				_roll_ctrl.reset_integrator();
				_pitch_ctrl.reset_integrator();
				_yaw_ctrl.reset_integrator();
				_wheel_ctrl.reset_integrator();
			}

			/* Prepare data for attitude controllers */
			struct ECL_ControlData control_input = {};
			control_input.roll = euler_angles.phi();
			control_input.pitch = euler_angles.theta();
			control_input.yaw = euler_angles.psi();
			control_input.body_x_rate = rollspeed;
			control_input.body_y_rate = pitchspeed;
			control_input.body_z_rate = yawspeed;
			control_input.roll_setpoint = _att_sp.roll_body;
			control_input.pitch_setpoint = _att_sp.pitch_body;
			control_input.yaw_setpoint = _att_sp.yaw_body;
			control_input.airspeed_min = _parameters.airspeed_min;
			control_input.airspeed_max = _parameters.airspeed_max;
			control_input.airspeed = airspeed;
			control_input.scaler = _airspeed_scaling;
			control_input.lock_integrator = lock_integrator;
			control_input.groundspeed = groundspeed;
			control_input.groundspeed_scaler = groundspeed_scaler;

			/* reset body angular rate limits on mode change */
			if ((_vcontrol_mode.flag_control_attitude_enabled != _flag_control_attitude_enabled_last) || params_updated) {
				if (_vcontrol_mode.flag_control_attitude_enabled
				    || _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
					_roll_ctrl.set_max_rate(math::radians(_parameters.r_rmax));
					_pitch_ctrl.set_max_rate_pos(math::radians(_parameters.p_rmax_pos));
					_pitch_ctrl.set_max_rate_neg(math::radians(_parameters.p_rmax_neg));
					_yaw_ctrl.set_max_rate(math::radians(_parameters.y_rmax));

				} else {
					_roll_ctrl.set_max_rate(_parameters.acro_max_x_rate_rad);
					_pitch_ctrl.set_max_rate_pos(_parameters.acro_max_y_rate_rad);
					_pitch_ctrl.set_max_rate_neg(_parameters.acro_max_y_rate_rad);
					_yaw_ctrl.set_max_rate(_parameters.acro_max_z_rate_rad);
				}
			}

			_flag_control_attitude_enabled_last = _vcontrol_mode.flag_control_attitude_enabled;

			/* bi-linear interpolation over airspeed for actuator trim scheduling */
			float trim_roll = _parameters.trim_roll;
			float trim_pitch = _parameters.trim_pitch;
			float trim_yaw = _parameters.trim_yaw;

			if (airspeed < _parameters.airspeed_trim) {
				trim_roll += math::gradual(airspeed, _parameters.airspeed_min, _parameters.airspeed_trim, _parameters.dtrim_roll_vmin,
							   0.0f);
				trim_pitch += math::gradual(airspeed, _parameters.airspeed_min, _parameters.airspeed_trim, _parameters.dtrim_pitch_vmin,
							    0.0f);
				trim_yaw += math::gradual(airspeed, _parameters.airspeed_min, _parameters.airspeed_trim, _parameters.dtrim_yaw_vmin,
							  0.0f);

			} else {
				trim_roll += math::gradual(airspeed, _parameters.airspeed_trim, _parameters.airspeed_max, 0.0f,
							   _parameters.dtrim_roll_vmax);
				trim_pitch += math::gradual(airspeed, _parameters.airspeed_trim, _parameters.airspeed_max, 0.0f,
							    _parameters.dtrim_pitch_vmax);
				trim_yaw += math::gradual(airspeed, _parameters.airspeed_trim, _parameters.airspeed_max, 0.0f,
							  _parameters.dtrim_yaw_vmax);
			}

			/* add trim increment if flaps are deployed  */
			trim_roll += _flaps_applied * _parameters.dtrim_roll_flaps;
			trim_pitch += _flaps_applied * _parameters.dtrim_pitch_flaps;

			/* Run attitude controllers */
			if (_vcontrol_mode.flag_control_attitude_enabled) {
				if (PX4_ISFINITE(_att_sp.roll_body) && PX4_ISFINITE(_att_sp.pitch_body)) {
					_roll_ctrl.control_attitude(control_input);
					_pitch_ctrl.control_attitude(control_input);
					_yaw_ctrl.control_attitude(control_input); //runs last, because is depending on output of roll and pitch attitude
					_wheel_ctrl.control_attitude(control_input);

					/* Update input data for rate controllers */
					control_input.roll_rate_setpoint = _roll_ctrl.get_desired_rate();
					control_input.pitch_rate_setpoint = _pitch_ctrl.get_desired_rate();
					control_input.yaw_rate_setpoint = _yaw_ctrl.get_desired_rate();

					/* Run attitude RATE controllers which need the desired attitudes from above, add trim */
					float roll_u = _roll_ctrl.control_euler_rate(control_input);
					_actuators.control[actuator_controls_s::INDEX_ROLL] = (PX4_ISFINITE(roll_u)) ? roll_u + trim_roll : trim_roll;

					if (!PX4_ISFINITE(roll_u)) {
						_roll_ctrl.reset_integrator();
					}

					float pitch_u = _pitch_ctrl.control_euler_rate(control_input);
					_actuators.control[actuator_controls_s::INDEX_PITCH] = (PX4_ISFINITE(pitch_u)) ? pitch_u + trim_pitch : trim_pitch;

					if (!PX4_ISFINITE(pitch_u)) {
						_pitch_ctrl.reset_integrator();
					}

					float yaw_u = 0.0f;

					if (_parameters.w_en && _att_sp.fw_control_yaw) {
						yaw_u = _wheel_ctrl.control_bodyrate(control_input);

					} else {
						yaw_u = _yaw_ctrl.control_euler_rate(control_input);
					}

					_actuators.control[actuator_controls_s::INDEX_YAW] = (PX4_ISFINITE(yaw_u)) ? yaw_u + trim_yaw : trim_yaw;

					/* add in manual rudder control in manual modes */
					if (_vcontrol_mode.flag_control_manual_enabled) {
						_actuators.control[actuator_controls_s::INDEX_YAW] += _manual.r;
					}

					if (!PX4_ISFINITE(yaw_u)) {
						_yaw_ctrl.reset_integrator();
						_wheel_ctrl.reset_integrator();
					}

					/* throttle passed through if it is finite and if no engine failure was detected */
					_actuators.control[actuator_controls_s::INDEX_THROTTLE] = (PX4_ISFINITE(_att_sp.thrust_body[0])
							&& !_vehicle_status.engine_failure) ? _att_sp.thrust_body[0] : 0.0f;

					/* scale effort by battery status */
					if (_parameters.bat_scale_en &&
					    _actuators.control[actuator_controls_s::INDEX_THROTTLE] > 0.1f) {

						if (_battery_status_sub.updated()) {
							battery_status_s battery_status{};

							if (_battery_status_sub.copy(&battery_status)) {
								if (battery_status.scale > 0.0f) {
									_battery_scale = battery_status.scale;
								}
							}
						}

						_actuators.control[actuator_controls_s::INDEX_THROTTLE] *= _battery_scale;
					}
				}

				/*
				 * Lazily publish the rate setpoint (for analysis, the actuators are published below)
				 * only once available
				 */
				_rates_sp.roll = _roll_ctrl.get_desired_bodyrate();
				_rates_sp.pitch = _pitch_ctrl.get_desired_bodyrate();
				_rates_sp.yaw = _yaw_ctrl.get_desired_bodyrate();

				_rates_sp.timestamp = hrt_absolute_time();

				_rate_sp_pub.publish(_rates_sp);

			} else {
				vehicle_rates_setpoint_poll();

				_roll_ctrl.set_bodyrate_setpoint(_rates_sp.roll);
				_yaw_ctrl.set_bodyrate_setpoint(_rates_sp.yaw);
				_pitch_ctrl.set_bodyrate_setpoint(_rates_sp.pitch);

				float roll_u = _roll_ctrl.control_bodyrate(control_input);
				_actuators.control[actuator_controls_s::INDEX_ROLL] = (PX4_ISFINITE(roll_u)) ? roll_u + trim_roll : trim_roll;

				float pitch_u = _pitch_ctrl.control_bodyrate(control_input);
				_actuators.control[actuator_controls_s::INDEX_PITCH] = (PX4_ISFINITE(pitch_u)) ? pitch_u + trim_pitch : trim_pitch;

				float yaw_u = _yaw_ctrl.control_bodyrate(control_input);
				_actuators.control[actuator_controls_s::INDEX_YAW] = (PX4_ISFINITE(yaw_u)) ? yaw_u + trim_yaw : trim_yaw;

				_actuators.control[actuator_controls_s::INDEX_THROTTLE] = PX4_ISFINITE(_rates_sp.thrust_body[0]) ?
						_rates_sp.thrust_body[0] : 0.0f;
			}

			rate_ctrl_status_s rate_ctrl_status;
			rate_ctrl_status.timestamp = hrt_absolute_time();
			rate_ctrl_status.rollspeed_integ = _roll_ctrl.get_integrator();
			rate_ctrl_status.pitchspeed_integ = _pitch_ctrl.get_integrator();
			rate_ctrl_status.yawspeed_integ = _yaw_ctrl.get_integrator();
			rate_ctrl_status.additional_integ1 = _wheel_ctrl.get_integrator();

			_rate_ctrl_status_pub.publish(rate_ctrl_status);
		}

		// Add feed-forward from roll control output to yaw control output
		// This can be used to counteract the adverse yaw effect when rolling the plane
		_actuators.control[actuator_controls_s::INDEX_YAW] += _parameters.roll_to_yaw_ff * math::constrain(
					_actuators.control[actuator_controls_s::INDEX_ROLL], -1.0f, 1.0f);

		_actuators.control[actuator_controls_s::INDEX_FLAPS] = _flaps_applied;
		_actuators.control[5] = _manual.aux1;
		_actuators.control[actuator_controls_s::INDEX_AIRBRAKES] = _flaperons_applied;
		// FIXME: this should use _vcontrol_mode.landing_gear_pos in the future
		_actuators.control[7] = _manual.aux3;

		/* lazily publish the setpoint only once available */
		_actuators.timestamp = hrt_absolute_time();
		_actuators.timestamp_sample = _att.timestamp;
		_actuators_airframe.timestamp = hrt_absolute_time();
		_actuators_airframe.timestamp_sample = _att.timestamp;

		/* Only publish if any of the proper modes are enabled */
		if (_vcontrol_mode.flag_control_rates_enabled ||
		    _vcontrol_mode.flag_control_attitude_enabled ||
		    _vcontrol_mode.flag_control_manual_enabled) {
			/* publish the actuator controls */
			if (_actuators_0_pub != nullptr) {
				orb_publish(_actuators_id, _actuators_0_pub, &_actuators);

			} else if (_actuators_id) {
				_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
			}

			_actuators_2_pub.publish(_actuators_airframe);
		}
	}

	perf_end(_loop_perf);
}

void FixedwingAttitudeControl::control_flaps(const float dt)
{
	/* default flaps to center */
	float flap_control = 0.0f;

	/* map flaps by default to manual if valid */
	if (PX4_ISFINITE(_manual.flaps) && _vcontrol_mode.flag_control_manual_enabled
	    && fabsf(_parameters.flaps_scale) > 0.01f) {
		flap_control = 0.5f * (_manual.flaps + 1.0f) * _parameters.flaps_scale;

	} else if (_vcontrol_mode.flag_control_auto_enabled
		   && fabsf(_parameters.flaps_scale) > 0.01f) {
		switch (_att_sp.apply_flaps) {
		case vehicle_attitude_setpoint_s::FLAPS_OFF : flap_control = 0.0f; // no flaps
			break;

		case vehicle_attitude_setpoint_s::FLAPS_LAND : flap_control = 1.0f * _parameters.flaps_scale *
					_parameters.flaps_land_scale; // landing flaps
			break;

		case vehicle_attitude_setpoint_s::FLAPS_TAKEOFF : flap_control = 1.0f * _parameters.flaps_scale *
					_parameters.flaps_takeoff_scale; // take-off flaps
			break;
		}
	}

	// move the actual control value continuous with time, full flap travel in 1sec
	if (fabsf(_flaps_applied - flap_control) > 0.01f) {
		_flaps_applied += (_flaps_applied - flap_control) < 0 ? dt : -dt;

	} else {
		_flaps_applied = flap_control;
	}

	/* default flaperon to center */
	float flaperon_control = 0.0f;

	/* map flaperons by default to manual if valid */
	if (PX4_ISFINITE(_manual.aux2) && _vcontrol_mode.flag_control_manual_enabled
	    && fabsf(_parameters.flaperon_scale) > 0.01f) {
		flaperon_control = 0.5f * (_manual.aux2 + 1.0f) * _parameters.flaperon_scale;

	} else if (_vcontrol_mode.flag_control_auto_enabled
		   && fabsf(_parameters.flaperon_scale) > 0.01f) {
		flaperon_control = (_att_sp.apply_flaps == vehicle_attitude_setpoint_s::FLAPS_LAND) ? 1.0f *
				   _parameters.flaperon_scale : 0.0f;
	}

	// move the actual control value continuous with time, full flap travel in 1sec
	if (fabsf(_flaperons_applied - flaperon_control) > 0.01f) {
		_flaperons_applied += (_flaperons_applied - flaperon_control) < 0 ? dt : -dt;

	} else {
		_flaperons_applied = flaperon_control;
	}
}

int FixedwingAttitudeControl::task_spawn(int argc, char *argv[])
{
	FixedwingAttitudeControl *instance = new FixedwingAttitudeControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int FixedwingAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FixedwingAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_att_control is the fixed wing attitude controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_COMMAND("start");

	PRINT_MODULE_USAGE_NAME("fw_att_control", "controller");

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int FixedwingAttitudeControl::print_status()
{
	PX4_INFO("Running");

	perf_print_counter(_loop_perf);

	return 0;
}

int fw_att_control_main(int argc, char *argv[])
{
	return FixedwingAttitudeControl::main(argc, argv);
}
