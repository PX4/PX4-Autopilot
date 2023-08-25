/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
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

#include "FixedwingPositionControl.hpp"

#include <px4_platform_common/events.h>

using math::constrain;
using math::max;
using math::min;
using math::radians;

using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector2d;
using matrix::Vector3f;
using matrix::wrap_pi;

FixedwingPositionControl::FixedwingPositionControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_attitude_sp_pub(vtol ? ORB_ID(fw_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_launchDetector(this),
	_runway_takeoff(this)
{
	if (vtol) {
		_param_handle_airspeed_trans = param_find("VT_ARSP_TRANS");
	}

	// limit to 50 Hz
	_local_pos_sub.set_interval_ms(20);

	_pos_ctrl_status_pub.advertise();
	_pos_ctrl_landing_status_pub.advertise();
	_tecs_status_pub.advertise();
	_launch_detection_status_pub.advertise();
	_landing_gear_pub.advertise();

	_flaps_setpoint_pub.advertise();
	_spoilers_setpoint_pub.advertise();

	_airspeed_slew_rate_controller.setSlewRate(ASPD_SP_SLEW_RATE);

	/* fetch initial parameter values */
	parameters_update();
}

FixedwingPositionControl::~FixedwingPositionControl()
{
	perf_free(_loop_perf);
}

bool
FixedwingPositionControl::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

int
FixedwingPositionControl::parameters_update()
{
	updateParams();

	// VTOL parameter VT_ARSP_TRANS
	if (_param_handle_airspeed_trans != PARAM_INVALID) {
		param_get(_param_handle_airspeed_trans, &_param_airspeed_trans);
	}

	// NPFG parameters
	_npfg.setPeriod(_param_npfg_period.get());
	_npfg.setDamping(_param_npfg_damping.get());
	_npfg.enablePeriodLB(_param_npfg_en_period_lb.get());
	_npfg.enablePeriodUB(_param_npfg_en_period_ub.get());
	_npfg.enableMinGroundSpeed(_param_npfg_en_min_gsp.get());
	_npfg.enableTrackKeeping(_param_npfg_en_track_keeping.get());
	_npfg.enableWindExcessRegulation(_param_npfg_en_wind_reg.get());
	_npfg.setMinGroundSpeed(_param_fw_gnd_spd_min.get());
	_npfg.setMaxTrackKeepingMinGroundSpeed(_param_npfg_track_keeping_gsp_max.get());
	_npfg.setRollTimeConst(_param_npfg_roll_time_const.get());
	_npfg.setSwitchDistanceMultiplier(_param_npfg_switch_distance_multiplier.get());
	_npfg.setRollLimit(radians(_param_fw_r_lim.get()));
	_npfg.setRollSlewRate(radians(_param_fw_pn_r_slew_max.get()));
	_npfg.setPeriodSafetyFactor(_param_npfg_period_safety_factor.get());

	// TECS parameters
	_tecs.set_max_climb_rate(_param_fw_t_clmb_max.get());
	_tecs.set_max_sink_rate(_param_fw_t_sink_max.get());
	_tecs.set_min_sink_rate(_param_fw_t_sink_min.get());
	_tecs.set_speed_weight(_param_fw_t_spdweight.get());
	_tecs.set_equivalent_airspeed_trim(_param_fw_airspd_trim.get());
	_tecs.set_equivalent_airspeed_min(_param_fw_airspd_min.get());
	_tecs.set_equivalent_airspeed_max(_param_fw_airspd_max.get());
	_tecs.set_throttle_damp(_param_fw_t_thr_damp.get());
	_tecs.set_integrator_gain_throttle(_param_fw_t_I_gain_thr.get());
	_tecs.set_integrator_gain_pitch(_param_fw_t_I_gain_pit.get());
	_tecs.set_throttle_slewrate(_param_fw_thr_slew_max.get());
	_tecs.set_vertical_accel_limit(_param_fw_t_vert_acc.get());
	_tecs.set_roll_throttle_compensation(_param_fw_t_rll2thr.get());
	_tecs.set_pitch_damping(_param_fw_t_ptch_damp.get());
	_tecs.set_altitude_error_time_constant(_param_fw_t_h_error_tc.get());
	_tecs.set_altitude_rate_ff(_param_fw_t_hrate_ff.get());
	_tecs.set_airspeed_error_time_constant(_param_fw_t_tas_error_tc.get());
	_tecs.set_ste_rate_time_const(_param_ste_rate_time_const.get());
	_tecs.set_seb_rate_ff_gain(_param_seb_rate_ff.get());
	_tecs.set_airspeed_measurement_std_dev(_param_speed_standard_dev.get());
	_tecs.set_airspeed_rate_measurement_std_dev(_param_speed_rate_standard_dev.get());
	_tecs.set_airspeed_filter_process_std_dev(_param_process_noise_standard_dev.get());

	int check_ret = PX4_OK;

	// sanity check parameters
	if (_param_fw_airspd_max.get() < _param_fw_airspd_min.get()) {
		/* EVENT
		 * @description
		 * - <param>FW_AIRSPD_MAX</param>: {1:.1}
		 * - <param>FW_AIRSPD_MIN</param>: {2:.1}
		 */
		events::send<float, float>(events::ID("fixedwing_position_control_conf_invalid_airspeed"), events::Log::Error,
					   "Invalid configuration: Airspeed max smaller than min",
					   _param_fw_airspd_max.get(), _param_fw_airspd_min.get());
		check_ret = PX4_ERROR;
	}

	if (_param_fw_airspd_max.get() < 5.0f || _param_fw_airspd_min.get() > 100.0f) {
		/* EVENT
		 * @description
		 * - <param>FW_AIRSPD_MAX</param>: {1:.1}
		 * - <param>FW_AIRSPD_MIN</param>: {2:.1}
		 */
		events::send<float, float>(events::ID("fixedwing_position_control_conf_invalid_airspeed_bounds"), events::Log::Error,
					   "Invalid configuration: Airspeed max \\< 5 m/s or min \\> 100 m/s",
					   _param_fw_airspd_max.get(), _param_fw_airspd_min.get());
		check_ret = PX4_ERROR;
	}

	if (_param_fw_airspd_trim.get() < _param_fw_airspd_min.get() ||
	    _param_fw_airspd_trim.get() > _param_fw_airspd_max.get()) {
		/* EVENT
		 * @description
		 * - <param>FW_AIRSPD_MAX</param>: {1:.1}
		 * - <param>FW_AIRSPD_MIN</param>: {2:.1}
		 * - <param>FW_AIRSPD_TRIM</param>: {3:.1}
		 */
		events::send<float, float, float>(events::ID("fixedwing_position_control_conf_invalid_trim_bounds"),
						  events::Log::Error,
						  "Invalid configuration: Airspeed trim out of min or max bounds",
						  _param_fw_airspd_max.get(), _param_fw_airspd_min.get(), _param_fw_airspd_trim.get());
		check_ret = PX4_ERROR;
	}

	if (_param_fw_airspd_stall.get() > _param_fw_airspd_min.get()) {
		/* EVENT
		 * @description
		 * - <param>FW_AIRSPD_MIN</param>: {1:.1}
		 * - <param>FW_AIRSPD_STALL</param>: {2:.1}
		 */
		events::send<float, float>(events::ID("fixedwing_position_control_conf_invalid_stall"), events::Log::Error,
					   "Invalid configuration: FW_AIRSPD_STALL higher FW_AIRSPD_MIN",
					   _param_fw_airspd_min.get(), _param_fw_airspd_stall.get());
		check_ret = PX4_ERROR;
	}

	return check_ret;
}

void
FixedwingPositionControl::vehicle_control_mode_poll()
{
	if (_control_mode_sub.updated()) {
		const bool was_armed = _control_mode.flag_armed;

		if (_control_mode_sub.copy(&_control_mode)) {

			// reset state when arming
			if (!was_armed && _control_mode.flag_armed) {
				reset_takeoff_state();
				reset_landing_state();
			}
		}
	}
}

void
FixedwingPositionControl::vehicle_command_poll()
{
	vehicle_command_s vehicle_command;

	while (_vehicle_command_sub.update(&vehicle_command)) {
		if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_GO_AROUND) {
			// only abort landing before point of no return (horizontal and vertical)
			if (_control_mode.flag_control_auto_enabled &&
			    _position_setpoint_current_valid &&
			    (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND)) {

				updateLandingAbortStatus(position_controller_landing_status_s::ABORTED_BY_OPERATOR);
			}

		} else if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_CHANGE_SPEED) {

			if ((static_cast<uint8_t>(vehicle_command.param1 + .5f) == vehicle_command_s::SPEED_TYPE_AIRSPEED)) {
				if (vehicle_command.param2 > FLT_EPSILON) {	// param2 is an equivalent airspeed setpoint
					if (_control_mode_current == FW_POSCTRL_MODE_AUTO) {
						_pos_sp_triplet.current.cruising_speed = vehicle_command.param2;

					} else if (_control_mode_current == FW_POSCTRL_MODE_MANUAL_ALTITUDE
						   || _control_mode_current == FW_POSCTRL_MODE_MANUAL_POSITION) {
						_commanded_manual_airspeed_setpoint = vehicle_command.param2;
					}

				}
			}

		}
	}
}

void
FixedwingPositionControl::airspeed_poll()
{
	bool airspeed_valid = _airspeed_valid;
	airspeed_validated_s airspeed_validated;

	if ((_param_fw_arsp_mode.get() == 0) && _airspeed_validated_sub.update(&airspeed_validated)) {

		_eas2tas = 1.0f; //this is the default value, taken in case of invalid airspeed

		if (PX4_ISFINITE(airspeed_validated.calibrated_airspeed_m_s)
		    && PX4_ISFINITE(airspeed_validated.true_airspeed_m_s)
		    && (airspeed_validated.calibrated_airspeed_m_s > 0.0f)) {

			airspeed_valid = true;

			_time_airspeed_last_valid = airspeed_validated.timestamp;
			_airspeed = airspeed_validated.calibrated_airspeed_m_s;

			_eas2tas = constrain(airspeed_validated.true_airspeed_m_s / airspeed_validated.calibrated_airspeed_m_s, 0.9f, 2.0f);
		}

	} else {
		// no airspeed updates for one second
		if (airspeed_valid && (hrt_elapsed_time(&_time_airspeed_last_valid) > 1_s)) {
			airspeed_valid = false;
		}
	}

	// update TECS if validity changed
	if (airspeed_valid != _airspeed_valid) {
		_tecs.enable_airspeed(airspeed_valid);
		_airspeed_valid = airspeed_valid;
	}
}

void
FixedwingPositionControl::wind_poll()
{
	if (_wind_sub.updated()) {
		wind_s wind;
		_wind_sub.update(&wind);

		// assumes wind is valid if finite
		_wind_valid = PX4_ISFINITE(wind.windspeed_north)
			      && PX4_ISFINITE(wind.windspeed_east);

		_time_wind_last_received = hrt_absolute_time();

		_wind_vel(0) = wind.windspeed_north;
		_wind_vel(1) = wind.windspeed_east;

	} else {
		// invalidate wind estimate usage (and correspondingly NPFG, if enabled) after subscription timeout
		_wind_valid = _wind_valid && (hrt_absolute_time() - _time_wind_last_received) < WIND_EST_TIMEOUT;
	}

	if (!_wind_valid) {
		_wind_vel(0) = 0.f;
		_wind_vel(1) = 0.f;
	}
}

void
FixedwingPositionControl::manual_control_setpoint_poll()
{
	_manual_control_setpoint_sub.update(&_manual_control_setpoint);

	_manual_control_setpoint_for_height_rate = _manual_control_setpoint.pitch;
	_manual_control_setpoint_for_airspeed = _manual_control_setpoint.throttle;

	if (_param_fw_pos_stk_conf.get() & STICK_CONFIG_SWAP_STICKS_BIT) {
		/* Alternate stick allocation (similar concept as for multirotor systems:
		 * demanding up/down with the throttle stick, and move faster/break with the pitch one.
		 */
		_manual_control_setpoint_for_height_rate = -_manual_control_setpoint.throttle;
		_manual_control_setpoint_for_airspeed = _manual_control_setpoint.pitch;
	}

	// send neutral setpoints if no update for 1 s
	if (hrt_elapsed_time(&_manual_control_setpoint.timestamp) > 1_s) {
		_manual_control_setpoint_for_height_rate = 0.f;
		_manual_control_setpoint_for_airspeed = 0.f;
	}
}


void
FixedwingPositionControl::vehicle_attitude_poll()
{
	vehicle_attitude_s att;

	if (_vehicle_attitude_sub.update(&att)) {
		vehicle_angular_velocity_s angular_velocity{};
		_vehicle_angular_velocity_sub.copy(&angular_velocity);
		const Vector3f rates{angular_velocity.xyz};

		Dcmf R{Quatf(att.q)};

		// if the vehicle is a tailsitter we have to rotate the attitude by the pitch offset
		// between multirotor and fixed wing flight
		if (_vehicle_status.is_vtol_tailsitter) {
			const Dcmf R_offset{Eulerf{0.f, M_PI_2_F, 0.f}};
			R = R * R_offset;

			_yawrate = rates(0);

		} else {
			_yawrate = rates(2);
		}

		const Eulerf euler_angles(R);
		_pitch = euler_angles(1);
		_yaw = euler_angles(2);

		Vector3f body_acceleration = R.transpose() * Vector3f{_local_pos.ax, _local_pos.ay, _local_pos.az};
		_body_acceleration_x = body_acceleration(0);

		Vector3f body_velocity = R.transpose() * Vector3f{_local_pos.vx, _local_pos.vy, _local_pos.vz};
		_body_velocity_x = body_velocity(0);

		// load factor due to banking
		const float load_factor = 1.f / cosf(euler_angles(0));
		_tecs.set_load_factor(load_factor);
	}
}

float
FixedwingPositionControl::get_manual_airspeed_setpoint()
{
	float altctrl_airspeed = _param_fw_airspd_trim.get();

	if (_param_fw_pos_stk_conf.get() & STICK_CONFIG_ENABLE_AIRSPEED_SP_MANUAL_BIT) {
		// neutral throttle corresponds to trim airspeed
		return math::interpolateNXY(_manual_control_setpoint_for_airspeed,
		{-1.f, 0.f, 1.f},
		{_param_fw_airspd_min.get(), _param_fw_airspd_trim.get(), _param_fw_airspd_max.get()});

	} else if (PX4_ISFINITE(_commanded_manual_airspeed_setpoint)) {
		altctrl_airspeed = _commanded_manual_airspeed_setpoint;
	}

	return altctrl_airspeed;
}

float
FixedwingPositionControl::adapt_airspeed_setpoint(const float control_interval, float calibrated_airspeed_setpoint,
		float calibrated_min_airspeed, const Vector2f &ground_speed)
{
	if (!PX4_ISFINITE(calibrated_airspeed_setpoint) || calibrated_airspeed_setpoint <= FLT_EPSILON) {
		calibrated_airspeed_setpoint = _param_fw_airspd_trim.get();
	}

	// Adapt cruise airspeed when otherwise the min groundspeed couldn't be maintained
	if (!_wind_valid) {
		/*
		 * This error value ensures that a plane (as long as its throttle capability is
		 * not exceeded) travels towards a waypoint (and is not pushed more and more away
		 * by wind). Not countering this would lead to a fly-away. Only non-zero in presence
		 * of sufficient wind. "minimum ground speed undershoot".
		 */
		const float ground_speed_body = _body_velocity_x;

		if (ground_speed_body < _param_fw_gnd_spd_min.get()) {
			calibrated_airspeed_setpoint += _param_fw_gnd_spd_min.get() - ground_speed_body;
		}
	}

	float load_factor_from_bank_angle = 1.0f;

	if (PX4_ISFINITE(_att_sp.roll_body)) {
		load_factor_from_bank_angle = 1.0f / cosf(_att_sp.roll_body);
	}

	float weight_ratio = 1.0f;

	if (_param_weight_base.get() > FLT_EPSILON && _param_weight_gross.get() > FLT_EPSILON) {
		weight_ratio = math::constrain(_param_weight_gross.get() / _param_weight_base.get(), MIN_WEIGHT_RATIO,
					       MAX_WEIGHT_RATIO);
	}

	// Here we make sure that the set minimum airspeed is automatically adapted to the current load factor.
	// The minimum airspeed is the controller limit (FW_AIRSPD_MIN, unless either in takeoff or landing) that should
	// resemble the vehicles stall speed (CAS) with a 1g load plus some safety margin (as no controller tracks perfectly).
	// Stall speed increases with the square root of the load factor: V_stall ~ sqrt(load_factor).
	// The load_factor is composed of a term from the bank angle and a term from the weight ratio.
	calibrated_min_airspeed *= sqrtf(load_factor_from_bank_angle * weight_ratio);

	// Aditional option to increase the min airspeed setpoint based on wind estimate for more stability in higher winds.
	if (_airspeed_valid && _wind_valid && _param_fw_wind_arsp_sc.get() > FLT_EPSILON) {
		calibrated_min_airspeed = math::min(calibrated_min_airspeed + _param_fw_wind_arsp_sc.get() *
						    _wind_vel.length(), _param_fw_airspd_max.get());
	}

	calibrated_airspeed_setpoint = constrain(calibrated_airspeed_setpoint, calibrated_min_airspeed,
				       _param_fw_airspd_max.get());

	// initialize to current airspeed setpoint, also if previous setpoint is out of bounds to not apply slew rate in that case
	const bool slewed_airspeed_outside_of_limits = _airspeed_slew_rate_controller.getState() < calibrated_min_airspeed
			|| _airspeed_slew_rate_controller.getState() > _param_fw_airspd_max.get();

	if (!PX4_ISFINITE(_airspeed_slew_rate_controller.getState()) || slewed_airspeed_outside_of_limits) {
		_airspeed_slew_rate_controller.setForcedValue(calibrated_airspeed_setpoint);

	} else if (control_interval > FLT_EPSILON) {
		// constrain airspeed setpoint changes with slew rate of ASPD_SP_SLEW_RATE m/s/s
		calibrated_airspeed_setpoint = _airspeed_slew_rate_controller.update(calibrated_airspeed_setpoint, control_interval);
	}

	return calibrated_airspeed_setpoint;
}

void
FixedwingPositionControl::tecs_status_publish(float alt_sp, float equivalent_airspeed_sp,
		float true_airspeed_derivative_raw, float throttle_trim)
{
	tecs_status_s tecs_status{};

	const TECS::DebugOutput &debug_output{_tecs.getStatus()};

	switch (_tecs.tecs_mode()) {
	case TECS::ECL_TECS_MODE_NORMAL:
		tecs_status.mode = tecs_status_s::TECS_MODE_NORMAL;
		break;

	case TECS::ECL_TECS_MODE_UNDERSPEED:
		tecs_status.mode = tecs_status_s::TECS_MODE_UNDERSPEED;
		break;
	}

	tecs_status.altitude_sp = alt_sp;
	tecs_status.altitude_reference = debug_output.altitude_reference;
	tecs_status.height_rate_reference = debug_output.height_rate_reference;
	tecs_status.height_rate_direct = debug_output.height_rate_direct;
	tecs_status.height_rate_setpoint = debug_output.control.altitude_rate_control;
	tecs_status.height_rate = -_local_pos.vz;
	tecs_status.equivalent_airspeed_sp = equivalent_airspeed_sp;
	tecs_status.true_airspeed_sp = _eas2tas * equivalent_airspeed_sp;
	tecs_status.true_airspeed_filtered = debug_output.true_airspeed_filtered;
	tecs_status.true_airspeed_derivative_sp = debug_output.control.true_airspeed_derivative_control;
	tecs_status.true_airspeed_derivative = debug_output.true_airspeed_derivative;
	tecs_status.true_airspeed_derivative_raw = true_airspeed_derivative_raw;
	tecs_status.total_energy_rate = debug_output.control.total_energy_rate_estimate;
	tecs_status.total_energy_balance_rate = debug_output.control.energy_balance_rate_estimate;
	tecs_status.total_energy_rate_sp = debug_output.control.total_energy_rate_sp;
	tecs_status.total_energy_balance_rate_sp = debug_output.control.energy_balance_rate_sp;
	tecs_status.throttle_integ = debug_output.control.throttle_integrator;
	tecs_status.pitch_integ = debug_output.control.pitch_integrator;
	tecs_status.throttle_sp = _tecs.get_throttle_setpoint();
	tecs_status.pitch_sp_rad = _tecs.get_pitch_setpoint();
	tecs_status.throttle_trim = throttle_trim;

	tecs_status.timestamp = hrt_absolute_time();

	_tecs_status_pub.publish(tecs_status);
}

void
FixedwingPositionControl::status_publish()
{
	position_controller_status_s pos_ctrl_status = {};

	pos_ctrl_status.nav_roll = _att_sp.roll_body;
	pos_ctrl_status.nav_pitch = _att_sp.pitch_body;

	npfg_status_s npfg_status = {};

	npfg_status.wind_est_valid = _wind_valid;

	const float bearing = _npfg.getBearing(); // dont repeat atan2 calc

	pos_ctrl_status.nav_bearing = bearing;
	pos_ctrl_status.target_bearing = _target_bearing;
	pos_ctrl_status.xtrack_error = _npfg.getTrackError();
	pos_ctrl_status.acceptance_radius = _npfg.switchDistance(500.0f);

	npfg_status.lat_accel = _npfg.getLateralAccel();
	npfg_status.lat_accel_ff = _npfg.getLateralAccelFF();
	npfg_status.heading_ref = _npfg.getHeadingRef();
	npfg_status.bearing = bearing;
	npfg_status.bearing_feas = _npfg.getBearingFeas();
	npfg_status.bearing_feas_on_track = _npfg.getOnTrackBearingFeas();
	npfg_status.signed_track_error = _npfg.getTrackError();
	npfg_status.track_error_bound = _npfg.getTrackErrorBound();
	npfg_status.airspeed_ref = _npfg.getAirspeedRef();
	npfg_status.min_ground_speed_ref = _npfg.getMinGroundSpeedRef();
	npfg_status.adapted_period = _npfg.getAdaptedPeriod();
	npfg_status.p_gain = _npfg.getPGain();
	npfg_status.time_const = _npfg.getTimeConst();
	npfg_status.timestamp = hrt_absolute_time();

	_npfg_status_pub.publish(npfg_status);

	pos_ctrl_status.wp_dist = get_distance_to_next_waypoint(_current_latitude, _current_longitude,
				  _pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon);

	pos_ctrl_status.yaw_acceptance = NAN;

	pos_ctrl_status.timestamp = hrt_absolute_time();

	pos_ctrl_status.type = _position_sp_type;

	_pos_ctrl_status_pub.publish(pos_ctrl_status);
}

void
FixedwingPositionControl::landing_status_publish()
{
	position_controller_landing_status_s pos_ctrl_landing_status = {};

	pos_ctrl_landing_status.lateral_touchdown_offset = _lateral_touchdown_position_offset;
	pos_ctrl_landing_status.flaring = _flare_states.flaring;
	pos_ctrl_landing_status.abort_status = _landing_abort_status;
	pos_ctrl_landing_status.timestamp = hrt_absolute_time();

	_pos_ctrl_landing_status_pub.publish(pos_ctrl_landing_status);
}

void
FixedwingPositionControl::updateLandingAbortStatus(const uint8_t new_abort_status)
{
	// prevent automatic aborts if already flaring, but allow manual aborts
	if (!_flare_states.flaring || new_abort_status == position_controller_landing_status_s::ABORTED_BY_OPERATOR) {

		// only announce changes
		if (new_abort_status > 0 && _landing_abort_status != new_abort_status) {

			switch (new_abort_status) {
			case (position_controller_landing_status_s::ABORTED_BY_OPERATOR): {
					events::send(events::ID("fixedwing_position_control_landing_abort_status_operator_abort"), events::Log::Critical,
						     "Landing aborted by operator");
					break;
				}

			case (position_controller_landing_status_s::TERRAIN_NOT_FOUND): {
					events::send(events::ID("fixedwing_position_control_landing_abort_status_terrain_not_found"), events::Log::Critical,
						     "Landing aborted: terrain measurement not found");
					break;
				}

			case (position_controller_landing_status_s::TERRAIN_TIMEOUT): {
					events::send(events::ID("fixedwing_position_control_landing_abort_status_terrain_timeout"), events::Log::Critical,
						     "Landing aborted: terrain estimate timed out");
					break;
				}

			default: {
					events::send(events::ID("fixedwing_position_control_landing_abort_status_unknown_criterion"), events::Log::Critical,
						     "Landing aborted: unknown criterion");
				}
			}
		}

		_landing_abort_status = (new_abort_status >= position_controller_landing_status_s::UNKNOWN_ABORT_CRITERION) ?
					position_controller_landing_status_s::UNKNOWN_ABORT_CRITERION : new_abort_status;
		landing_status_publish();
	}
}

void
FixedwingPositionControl::get_waypoint_heading_distance(float heading, position_setpoint_s &waypoint_prev,
		position_setpoint_s &waypoint_next, bool flag_init)
{
	position_setpoint_s temp_prev = waypoint_prev;
	position_setpoint_s temp_next = waypoint_next;

	if (flag_init) {
		// previous waypoint: HDG_HOLD_SET_BACK_DIST meters behind us
		waypoint_from_heading_and_distance(_current_latitude, _current_longitude, heading + radians(180.0f),
						   HDG_HOLD_SET_BACK_DIST, &temp_prev.lat, &temp_prev.lon);

		// next waypoint: HDG_HOLD_DIST_NEXT meters in front of us
		waypoint_from_heading_and_distance(_current_latitude, _current_longitude, heading,
						   HDG_HOLD_DIST_NEXT, &temp_next.lat, &temp_next.lon);

	} else {
		// use the existing flight path from prev to next

		// previous waypoint: shifted HDG_HOLD_REACHED_DIST + HDG_HOLD_SET_BACK_DIST
		create_waypoint_from_line_and_dist(waypoint_next.lat, waypoint_next.lon, waypoint_prev.lat, waypoint_prev.lon,
						   HDG_HOLD_REACHED_DIST + HDG_HOLD_SET_BACK_DIST, &temp_prev.lat, &temp_prev.lon);

		// next waypoint: shifted -(HDG_HOLD_DIST_NEXT + HDG_HOLD_REACHED_DIST)
		create_waypoint_from_line_and_dist(waypoint_next.lat, waypoint_next.lon, waypoint_prev.lat, waypoint_prev.lon,
						   -(HDG_HOLD_REACHED_DIST + HDG_HOLD_DIST_NEXT), &temp_next.lat, &temp_next.lon);
	}

	waypoint_prev = temp_prev;
	waypoint_prev.alt = _current_altitude;

	waypoint_next = temp_next;
	waypoint_next.alt = _current_altitude;
}

float
FixedwingPositionControl::getManualHeightRateSetpoint()
{
	/*
	 * The complete range is -1..+1, so this is 6%
	 * of the up or down range or 3% of the total range.
	 */
	const float deadBand = 0.06f;

	/*
	 * The correct scaling of the complete range needs
	 * to account for the missing part of the slope
	 * due to the deadband
	 */
	const float factor = 1.0f - deadBand;

	float height_rate_setpoint = 0.0f;

	/*
	 * Manual control has as convention the rotation around
	 * an axis. Positive X means to rotate positively around
	 * the X axis in NED frame, which is pitching down
	 */
	if (_manual_control_setpoint_for_height_rate > deadBand) {
		/* pitching down */
		float pitch = -(_manual_control_setpoint_for_height_rate - deadBand) / factor;
		height_rate_setpoint = pitch * _param_sinkrate_target.get();

	} else if (_manual_control_setpoint_for_height_rate < - deadBand) {
		/* pitching up */
		float pitch = -(_manual_control_setpoint_for_height_rate + deadBand) / factor;
		const float climb_rate_target = _param_climbrate_target.get();

		height_rate_setpoint = pitch * climb_rate_target;

	}

	return height_rate_setpoint;
}

void
FixedwingPositionControl::updateManualTakeoffStatus()
{
	if (!_completed_manual_takeoff) {
		const bool at_controllable_airspeed = _airspeed > _param_fw_airspd_min.get()
						      || !_airspeed_valid;
		const bool is_hovering = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
					 && _control_mode.flag_armed;
		_completed_manual_takeoff = (!_landed && at_controllable_airspeed) || is_hovering;
	}
}

void
FixedwingPositionControl::set_control_mode_current(const hrt_abstime &now)
{
	/* only run position controller in fixed-wing mode and during transitions for VTOL */
	if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !_vehicle_status.in_transition_mode) {
		_control_mode_current = FW_POSCTRL_MODE_OTHER;
		return; // do not publish the setpoint
	}

	FW_POSCTRL_MODE commanded_position_control_mode = _control_mode_current;

	_skipping_takeoff_detection = false;

	if (((_control_mode.flag_control_auto_enabled && _control_mode.flag_control_position_enabled) ||
	     _control_mode.flag_control_offboard_enabled) && _position_setpoint_current_valid) {

		if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {

			if (_vehicle_status.is_vtol && _vehicle_status.in_transition_mode) {
				_control_mode_current = FW_POSCTRL_MODE_AUTO;

				// in this case we want the waypoint handled as a position setpoint -- a submode in control_auto()
				_pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;

			} else {
				_control_mode_current = FW_POSCTRL_MODE_AUTO_TAKEOFF;

				if (commanded_position_control_mode != FW_POSCTRL_MODE_AUTO_TAKEOFF && !_landed) {
					// skip takeoff detection when switching from any other mode, auto or manual,
					// while already in air.
					// TODO: find a better place for / way of doing this
					_skipping_takeoff_detection = true;
				}
			}

		} else if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {

			if (!_vehicle_status.in_transition_mode) {

				// Use _position_setpoint_previous_valid to determine if landing should be straight or circular.
				// Straight landings are currently only possible in Missions, and there the previous WP
				// is valid, and circular ones are used outside of Missions, as the land mode sets prev_valid=false.
				if (_position_setpoint_previous_valid) {
					_control_mode_current = FW_POSCTRL_MODE_AUTO_LANDING_STRAIGHT;

				} else {
					_control_mode_current = FW_POSCTRL_MODE_AUTO_LANDING_CIRCULAR;
				}

			} else {
				// in this case we want the waypoint handled as a position setpoint -- a submode in control_auto()
				_pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
			}

		} else {
			_control_mode_current = FW_POSCTRL_MODE_AUTO;
		}

	} else if (_control_mode.flag_control_auto_enabled
		   && _control_mode.flag_control_climb_rate_enabled
		   && _control_mode.flag_armed // only enter this modes if armed, as pure failsafe modes
		   && !_control_mode.flag_control_position_enabled) {

		// failsafe modes engaged if position estimate is invalidated

		if (commanded_position_control_mode != FW_POSCTRL_MODE_AUTO_ALTITUDE
		    && commanded_position_control_mode != FW_POSCTRL_MODE_AUTO_CLIMBRATE) {
			// reset timer the first time we switch into this mode
			_time_in_fixed_bank_loiter = now;
		}

		if (hrt_elapsed_time(&_time_in_fixed_bank_loiter) < (_param_nav_gpsf_lt.get() * 1_s)
		    && !_vehicle_status.in_transition_mode) {
			if (commanded_position_control_mode != FW_POSCTRL_MODE_AUTO_ALTITUDE) {
				// Need to init because last loop iteration was in a different mode
				events::send(events::ID("fixedwing_position_control_fb_loiter"), events::Log::Critical,
					     "Start loiter with fixed bank angle");
			}

			_control_mode_current = FW_POSCTRL_MODE_AUTO_ALTITUDE;

		} else {
			if (commanded_position_control_mode != FW_POSCTRL_MODE_AUTO_CLIMBRATE && !_vehicle_status.in_transition_mode) {
				events::send(events::ID("fixedwing_position_control_descend"), events::Log::Critical, "Start descending");
			}

			_control_mode_current = FW_POSCTRL_MODE_AUTO_CLIMBRATE;
		}


	} else if (_control_mode.flag_control_manual_enabled && _control_mode.flag_control_position_enabled) {
		if (commanded_position_control_mode != FW_POSCTRL_MODE_MANUAL_POSITION) {
			/* Need to init because last loop iteration was in a different mode */
			_hdg_hold_yaw = _yaw; // yaw is not controlled, so set setpoint to current yaw
			_hdg_hold_enabled = false; // this makes sure the waypoints are reset below
			_yaw_lock_engaged = false;

			/* reset setpoints from other modes (auto) otherwise we won't
			 * level out without new manual input */
			_att_sp.roll_body = _manual_control_setpoint.roll * radians(_param_fw_r_lim.get());
			_att_sp.yaw_body = _yaw; // yaw is not controlled, so set setpoint to current yaw
		}

		_control_mode_current = FW_POSCTRL_MODE_MANUAL_POSITION;

	} else if (_control_mode.flag_control_manual_enabled && _control_mode.flag_control_altitude_enabled) {

		_control_mode_current = FW_POSCTRL_MODE_MANUAL_ALTITUDE;

	} else {
		_control_mode_current = FW_POSCTRL_MODE_OTHER;
	}
}

void
FixedwingPositionControl::update_in_air_states(const hrt_abstime now)
{
	/* save time when airplane is in air */
	if (!_was_in_air && !_landed) {
		_was_in_air = true;
		_time_went_in_air = now;

		_tecs.initialize(_current_altitude, -_local_pos.vz, _airspeed, _eas2tas);
	}

	/* reset flag when airplane landed */
	if (_landed) {
		_was_in_air = false;
		_completed_manual_takeoff = false;

		_tecs.initialize(_current_altitude, -_local_pos.vz, _airspeed, _eas2tas);
	}
}

void
FixedwingPositionControl::move_position_setpoint_for_vtol_transition(position_setpoint_s &current_sp)
{
	// TODO: velocity, altitude, or just a heading hold position mode should be used for this, not position
	// shifting hacks

	if (_vehicle_status.in_transition_to_fw) {

		if (!PX4_ISFINITE(_transition_waypoint(0))) {
			double lat_transition, lon_transition;

			// Create a virtual waypoint HDG_HOLD_DIST_NEXT meters in front of the vehicle which the path navigation controller can track
			// during the transition. Use the current yaw setpoint to determine the transition heading, as that one in turn
			// is set to the transition heading by Navigator, or current yaw if setpoint is not valid.
			const float transition_heading = PX4_ISFINITE(current_sp.yaw) ? current_sp.yaw : _yaw;
			waypoint_from_heading_and_distance(_current_latitude, _current_longitude, transition_heading, HDG_HOLD_DIST_NEXT,
							   &lat_transition,
							   &lon_transition);

			_transition_waypoint(0) = lat_transition;
			_transition_waypoint(1) = lon_transition;
		}


		current_sp.lat = _transition_waypoint(0);
		current_sp.lon = _transition_waypoint(1);

	} else {
		/* reset transition waypoint, will be set upon entering front transition */
		_transition_waypoint(0) = static_cast<double>(NAN);
		_transition_waypoint(1) = static_cast<double>(NAN);
	}
}

void
FixedwingPositionControl::control_auto(const float control_interval, const Vector2d &curr_pos,
				       const Vector2f &ground_speed, const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr,
				       const position_setpoint_s &pos_sp_next)
{
	position_setpoint_s current_sp = pos_sp_curr;
	move_position_setpoint_for_vtol_transition(current_sp);

	const uint8_t position_sp_type = handle_setpoint_type(current_sp);

	_position_sp_type = position_sp_type;

	if (position_sp_type == position_setpoint_s::SETPOINT_TYPE_LOITER
	    || current_sp.type == position_setpoint_s::SETPOINT_TYPE_LOITER) {
		publishOrbitStatus(current_sp);
	}

	switch (position_sp_type) {
	case position_setpoint_s::SETPOINT_TYPE_IDLE:
		_att_sp.thrust_body[0] = 0.0f;
		_att_sp.roll_body = 0.0f;
		_att_sp.pitch_body = radians(_param_fw_psp_off.get());
		break;

	case position_setpoint_s::SETPOINT_TYPE_POSITION:
		control_auto_position(control_interval, curr_pos, ground_speed, pos_sp_prev, current_sp);
		break;

	case position_setpoint_s::SETPOINT_TYPE_VELOCITY:
		control_auto_velocity(control_interval, curr_pos, ground_speed, current_sp);
		break;

	case position_setpoint_s::SETPOINT_TYPE_LOITER:
		control_auto_loiter(control_interval, curr_pos, ground_speed, pos_sp_prev, current_sp, pos_sp_next);
		break;
	}

	/* Copy thrust output for publication, handle special cases */
	if (position_sp_type == position_setpoint_s::SETPOINT_TYPE_IDLE) {

		_att_sp.thrust_body[0] = 0.0f;

	} else {
		// when we are landed state we want the motor to spin at idle speed
		_att_sp.thrust_body[0] = (_landed) ? min(_param_fw_thr_idle.get(), 1.f) : get_tecs_thrust();
	}

	/* Copy thrust and pitch values from tecs */
	_att_sp.pitch_body = get_tecs_pitch();

	if (!_vehicle_status.in_transition_to_fw) {
		publishLocalPositionSetpoint(current_sp);
	}
}

void
FixedwingPositionControl::control_auto_fixed_bank_alt_hold(const float control_interval)
{
	// only control altitude and airspeed ("fixed-bank loiter")

	tecs_update_pitch_throttle(control_interval,
				   _current_altitude,
				   _param_fw_airspd_trim.get(),
				   radians(_param_fw_p_lim_min.get()),
				   radians(_param_fw_p_lim_max.get()),
				   _param_fw_thr_min.get(),
				   _param_fw_thr_max.get(),
				   _param_sinkrate_target.get(),
				   _param_climbrate_target.get());

	_att_sp.roll_body = math::radians(_param_nav_gpsf_r.get()); // open loop loiter bank angle
	_att_sp.yaw_body = 0.f;

	if (_landed) {
		_att_sp.thrust_body[0] = _param_fw_thr_min.get();

	} else {
		_att_sp.thrust_body[0] = min(get_tecs_thrust(), _param_fw_thr_max.get());
	}

	_att_sp.pitch_body = get_tecs_pitch();

}

void
FixedwingPositionControl::control_auto_descend(const float control_interval)
{
	// Hard-code descend rate to 0.5m/s. This is a compromise to give the system to recover,
	// but not letting it drift too far away.
	const float descend_rate = -0.5f;

	tecs_update_pitch_throttle(control_interval,
				   _current_altitude,
				   _param_fw_airspd_trim.get(),
				   radians(_param_fw_p_lim_min.get()),
				   radians(_param_fw_p_lim_max.get()),
				   _param_fw_thr_min.get(),
				   _param_fw_thr_max.get(),
				   _param_sinkrate_target.get(),
				   _param_climbrate_target.get(),
				   false,
				   descend_rate);

	_att_sp.roll_body = math::radians(_param_nav_gpsf_r.get()); // open loop loiter bank angle
	_att_sp.yaw_body = 0.f;

	_att_sp.thrust_body[0] = (_landed) ? _param_fw_thr_min.get() : min(get_tecs_thrust(), _param_fw_thr_max.get());
	_att_sp.pitch_body = get_tecs_pitch();
}

uint8_t
FixedwingPositionControl::handle_setpoint_type(const position_setpoint_s &pos_sp_curr)
{
	uint8_t position_sp_type = pos_sp_curr.type;

	if (!_control_mode.flag_control_position_enabled && _control_mode.flag_control_velocity_enabled) {
		return position_setpoint_s::SETPOINT_TYPE_VELOCITY;
	}

	Vector2d curr_wp{0, 0};

	/* current waypoint (the one currently heading for) */
	curr_wp = Vector2d(pos_sp_curr.lat, pos_sp_curr.lon);

	const float acc_rad = _npfg.switchDistance(500.0f);

	if (pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_POSITION
	    || pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_LOITER) {

		float dist_xy = -1.f;
		float dist_z = -1.f;

		const float dist = get_distance_to_point_global_wgs84(
					   (double)curr_wp(0), (double)curr_wp(1), pos_sp_curr.alt,
					   _current_latitude, _current_longitude, _current_altitude,
					   &dist_xy, &dist_z);

		if (pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
			// Achieve position setpoint altitude via loiter when laterally close to WP.
			// Detect if system has switchted into a Loiter before (check _position_sp_type), and in that
			// case remove the dist_xy check (not switch out of Loiter until altitude is reached).
			if ((!_vehicle_status.in_transition_mode) && (dist >= 0.f)
			    && (dist_z > _param_nav_fw_alt_rad.get())
			    && (dist_xy < acc_rad || _position_sp_type == position_setpoint_s::SETPOINT_TYPE_LOITER)) {

				// SETPOINT_TYPE_POSITION -> SETPOINT_TYPE_LOITER
				position_sp_type = position_setpoint_s::SETPOINT_TYPE_LOITER;
			}
		}
	}

	return position_sp_type;
}

void
FixedwingPositionControl::control_auto_position(const float control_interval, const Vector2d &curr_pos,
		const Vector2f &ground_speed, const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr)
{
	const float acc_rad = _npfg.switchDistance(500.0f);
	Vector2d curr_wp{0, 0};
	Vector2d prev_wp{0, 0};

	/* current waypoint (the one currently heading for) */
	curr_wp = Vector2d(pos_sp_curr.lat, pos_sp_curr.lon);

	if (_position_setpoint_previous_valid && pos_sp_prev.type != position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {
		prev_wp(0) = pos_sp_prev.lat;
		prev_wp(1) = pos_sp_prev.lon;

	} else {
		// No valid previous waypoint, go along the line between aircraft and current waypoint
		prev_wp = curr_pos;
	}

	float tecs_fw_thr_min;
	float tecs_fw_thr_max;

	if (pos_sp_curr.gliding_enabled) {
		/* enable gliding with this waypoint */
		_tecs.set_speed_weight(2.0f);
		tecs_fw_thr_min = 0.0;
		tecs_fw_thr_max = 0.0;

	} else {
		tecs_fw_thr_min = _param_fw_thr_min.get();
		tecs_fw_thr_max = _param_fw_thr_max.get();
	}

	// waypoint is a plain navigation waypoint
	float position_sp_alt = pos_sp_curr.alt;

	// Altitude first order hold (FOH)
	if (_position_setpoint_previous_valid &&
	    ((pos_sp_prev.type == position_setpoint_s::SETPOINT_TYPE_POSITION) ||
	     (pos_sp_prev.type == position_setpoint_s::SETPOINT_TYPE_LOITER))
	   ) {
		const float d_curr_prev = get_distance_to_next_waypoint((double)curr_wp(0), (double)curr_wp(1),
					  pos_sp_prev.lat, pos_sp_prev.lon);

		// Do not try to find a solution if the last waypoint is inside the acceptance radius of the current one
		if (d_curr_prev > math::max(acc_rad, fabsf(pos_sp_curr.loiter_radius))) {
			// Calculate distance to current waypoint
			const float d_curr = get_distance_to_next_waypoint((double)curr_wp(0), (double)curr_wp(1),
					     _current_latitude, _current_longitude);

			// Save distance to waypoint if it is the smallest ever achieved, however make sure that
			// _min_current_sp_distance_xy is never larger than the distance between the current and the previous wp
			_min_current_sp_distance_xy = math::min(d_curr, _min_current_sp_distance_xy, d_curr_prev);

			// if the minimal distance is smaller than the acceptance radius, we should be at waypoint alt
			// navigator will soon switch to the next waypoint item (if there is one) as soon as we reach this altitude
			if (_min_current_sp_distance_xy > math::max(acc_rad, fabsf(pos_sp_curr.loiter_radius))) {
				// The setpoint is set linearly and such that the system reaches the current altitude at the acceptance
				// radius around the current waypoint
				const float delta_alt = (pos_sp_curr.alt - pos_sp_prev.alt);
				const float grad = -delta_alt / (d_curr_prev - math::max(acc_rad, fabsf(pos_sp_curr.loiter_radius)));
				const float a = pos_sp_prev.alt - grad * d_curr_prev;

				position_sp_alt = a + grad * _min_current_sp_distance_xy;
			}
		}
	}

	float target_airspeed = adapt_airspeed_setpoint(control_interval, pos_sp_curr.cruising_speed,
				_param_fw_airspd_min.get(), ground_speed);

	Vector2f curr_pos_local{_local_pos.x, _local_pos.y};
	Vector2f curr_wp_local = _global_local_proj_ref.project(curr_wp(0), curr_wp(1));
	Vector2f prev_wp_local = _global_local_proj_ref.project(prev_wp(0), prev_wp(1));

	_npfg.setAirspeedNom(target_airspeed * _eas2tas);
	_npfg.setAirspeedMax(_param_fw_airspd_max.get() * _eas2tas);

	if (_control_mode.flag_control_offboard_enabled && PX4_ISFINITE(pos_sp_curr.vx) && PX4_ISFINITE(pos_sp_curr.vy)) {
		// Navigate directly on position setpoint and path tangent
		matrix::Vector2f velocity_2d(pos_sp_curr.vx, pos_sp_curr.vy);
		float curvature = PX4_ISFINITE(_pos_sp_triplet.current.loiter_radius) ? 1 / _pos_sp_triplet.current.loiter_radius :
				  0.0f;
		navigatePathTangent(curr_pos_local, curr_wp_local, velocity_2d.normalized(), ground_speed,
				    _wind_vel, curvature);

	} else {
		navigateWaypoints(prev_wp_local, curr_wp_local, curr_pos_local, ground_speed, _wind_vel);
	}

	_att_sp.roll_body = _npfg.getRollSetpoint();
	target_airspeed = _npfg.getAirspeedRef() / _eas2tas;

	_att_sp.yaw_body = _yaw; // yaw is not controlled, so set setpoint to current yaw

	tecs_update_pitch_throttle(control_interval,
				   position_sp_alt,
				   target_airspeed,
				   radians(_param_fw_p_lim_min.get()),
				   radians(_param_fw_p_lim_max.get()),
				   tecs_fw_thr_min,
				   tecs_fw_thr_max,
				   _param_sinkrate_target.get(),
				   _param_climbrate_target.get());
}

void
FixedwingPositionControl::control_auto_velocity(const float control_interval, const Vector2d &curr_pos,
		const Vector2f &ground_speed, const position_setpoint_s &pos_sp_curr)
{
	float tecs_fw_thr_min;
	float tecs_fw_thr_max;

	if (pos_sp_curr.gliding_enabled) {
		/* enable gliding with this waypoint */
		_tecs.set_speed_weight(2.0f);
		tecs_fw_thr_min = 0.0;
		tecs_fw_thr_max = 0.0;

	} else {
		tecs_fw_thr_min = _param_fw_thr_min.get();
		tecs_fw_thr_max = _param_fw_thr_max.get();
	}

	// waypoint is a plain navigation waypoint
	float position_sp_alt = pos_sp_curr.alt;


	//Offboard velocity control
	Vector2f target_velocity{pos_sp_curr.vx, pos_sp_curr.vy};
	_target_bearing = wrap_pi(atan2f(target_velocity(1), target_velocity(0)));

	float target_airspeed = adapt_airspeed_setpoint(control_interval, pos_sp_curr.cruising_speed,
				_param_fw_airspd_min.get(), ground_speed);

	Vector2f curr_pos_local{_local_pos.x, _local_pos.y};
	_npfg.setAirspeedNom(target_airspeed * _eas2tas);
	_npfg.setAirspeedMax(_param_fw_airspd_max.get() * _eas2tas);
	navigateBearing(curr_pos_local, _target_bearing, ground_speed, _wind_vel);
	_att_sp.roll_body = _npfg.getRollSetpoint();
	target_airspeed = _npfg.getAirspeedRef() / _eas2tas;

	_att_sp.yaw_body = _yaw;

	tecs_update_pitch_throttle(control_interval,
				   position_sp_alt,
				   target_airspeed,
				   radians(_param_fw_p_lim_min.get()),
				   radians(_param_fw_p_lim_max.get()),
				   tecs_fw_thr_min,
				   tecs_fw_thr_max,
				   _param_sinkrate_target.get(),
				   _param_climbrate_target.get(),
				   tecs_status_s::TECS_MODE_NORMAL,
				   pos_sp_curr.vz);
}

void
FixedwingPositionControl::control_auto_loiter(const float control_interval, const Vector2d &curr_pos,
		const Vector2f &ground_speed, const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr,
		const position_setpoint_s &pos_sp_next)
{
	Vector2d curr_wp{0, 0};
	Vector2d prev_wp{0, 0};

	/* current waypoint (the one currently heading for) */
	curr_wp = Vector2d(pos_sp_curr.lat, pos_sp_curr.lon);

	if (_position_setpoint_previous_valid && pos_sp_prev.type != position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {
		prev_wp(0) = pos_sp_prev.lat;
		prev_wp(1) = pos_sp_prev.lon;

	} else {
		// No valid previous waypoint, go along the line between aircraft and current waypoint
		prev_wp = curr_pos;
	}

	float airspeed_sp = -1.f;

	if (PX4_ISFINITE(pos_sp_curr.cruising_speed) &&
	    pos_sp_curr.cruising_speed > FLT_EPSILON) {

		airspeed_sp = pos_sp_curr.cruising_speed;
	}

	float tecs_fw_thr_min;
	float tecs_fw_thr_max;

	if (pos_sp_curr.gliding_enabled) {
		/* enable gliding with this waypoint */
		_tecs.set_speed_weight(2.0f);
		tecs_fw_thr_min = 0.0;
		tecs_fw_thr_max = 0.0;

	} else {
		tecs_fw_thr_min = _param_fw_thr_min.get();
		tecs_fw_thr_max = _param_fw_thr_max.get();
	}

	/* waypoint is a loiter waypoint */
	float loiter_radius = pos_sp_curr.loiter_radius;

	if (fabsf(pos_sp_curr.loiter_radius) < FLT_EPSILON) {
		loiter_radius = _param_nav_loiter_rad.get();
	}

	Vector2f curr_pos_local{_local_pos.x, _local_pos.y};
	Vector2f curr_wp_local{_global_local_proj_ref.project(curr_wp(0), curr_wp(1))};
	Vector2f vehicle_to_loiter_center{curr_wp_local - curr_pos_local};

	const bool close_to_circle = vehicle_to_loiter_center.norm() < loiter_radius + _npfg.switchDistance(500);

	if (pos_sp_next.type == position_setpoint_s::SETPOINT_TYPE_LAND && _position_setpoint_next_valid
	    && close_to_circle && _param_fw_lnd_earlycfg.get()) {
		// We're in a loiter directly before a landing WP. Enable our landing configuration (flaps,
		// landing airspeed and potentially tighter altitude control) already such that we don't
		// have to do this switch (which can cause significant altitude errors) close to the ground.
		_tecs.set_altitude_error_time_constant(_param_fw_thrtc_sc.get() * _param_fw_t_h_error_tc.get());
		airspeed_sp = (_param_fw_lnd_airspd.get() > FLT_EPSILON) ? _param_fw_lnd_airspd.get() : _param_fw_airspd_min.get();
		_flaps_setpoint = _param_fw_flaps_lnd_scl.get();
		_spoilers_setpoint = _param_fw_spoilers_lnd.get();
		_new_landing_gear_position = landing_gear_s::GEAR_DOWN;
	}

	float target_airspeed = adapt_airspeed_setpoint(control_interval, airspeed_sp, _param_fw_airspd_min.get(),
				ground_speed);

	_npfg.setAirspeedNom(target_airspeed * _eas2tas);
	_npfg.setAirspeedMax(_param_fw_airspd_max.get() * _eas2tas);
	navigateLoiter(curr_wp_local, curr_pos_local, loiter_radius, pos_sp_curr.loiter_direction_counter_clockwise,
		       ground_speed,
		       _wind_vel);
	_att_sp.roll_body = _npfg.getRollSetpoint();
	target_airspeed = _npfg.getAirspeedRef() / _eas2tas;

	_att_sp.yaw_body = _yaw; // yaw is not controlled, so set setpoint to current yaw

	float alt_sp = pos_sp_curr.alt;

	if (_landing_abort_status) {
		if (pos_sp_curr.alt - _current_altitude  < kClearanceAltitudeBuffer) {
			// aborted landing complete, normal loiter over landing point
			updateLandingAbortStatus(position_controller_landing_status_s::NOT_ABORTED);

		} else {
			// continue straight until vehicle has sufficient altitude
			_att_sp.roll_body = 0.0f;
		}

		_tecs.set_altitude_error_time_constant(_param_fw_thrtc_sc.get() * _param_fw_t_h_error_tc.get());
	}

	tecs_update_pitch_throttle(control_interval,
				   alt_sp,
				   target_airspeed,
				   radians(_param_fw_p_lim_min.get()),
				   radians(_param_fw_p_lim_max.get()),
				   tecs_fw_thr_min,
				   tecs_fw_thr_max,
				   _param_sinkrate_target.get(),
				   _param_climbrate_target.get());
}

void
FixedwingPositionControl::control_auto_takeoff(const hrt_abstime &now, const float control_interval,
		const Vector2d &global_position, const Vector2f &ground_speed, const position_setpoint_s &pos_sp_curr)
{
	if (!_control_mode.flag_armed) {
		reset_takeoff_state();
	}

	// for now taking current position setpoint altitude as clearance altitude. this is the altitude we need to
	// clear all occlusions in the takeoff path
	const float clearance_altitude_amsl = pos_sp_curr.alt;

	// set the altitude to something above the clearance altitude to ensure the vehicle climbs past the value
	// (navigator will accept the takeoff as complete once crossing the clearance altitude)
	const float altitude_setpoint_amsl = clearance_altitude_amsl + kClearanceAltitudeBuffer;

	Vector2f local_2D_position{_local_pos.x, _local_pos.y};

	const float takeoff_airspeed = (_param_fw_tko_airspd.get() > FLT_EPSILON) ? _param_fw_tko_airspd.get() :
				       _param_fw_airspd_min.get();

	float adjusted_min_airspeed = _param_fw_airspd_min.get();

	if (takeoff_airspeed < _param_fw_airspd_min.get()) {
		// adjust underspeed detection bounds for takeoff airspeed
		_tecs.set_equivalent_airspeed_min(takeoff_airspeed);
		adjusted_min_airspeed = takeoff_airspeed;
	}

	float target_airspeed = adapt_airspeed_setpoint(control_interval, takeoff_airspeed, adjusted_min_airspeed,
				ground_speed);

	if (_runway_takeoff.runwayTakeoffEnabled()) {
		if (!_runway_takeoff.isInitialized()) {
			_runway_takeoff.init(now, _yaw, global_position);

			_takeoff_ground_alt = _current_altitude;

			_launch_current_yaw = _yaw;

			events::send(events::ID("fixedwing_position_control_takeoff"), events::Log::Info, "Takeoff on runway");
		}

		if (_skipping_takeoff_detection) {
			_runway_takeoff.forceSetFlyState();
		}

		_runway_takeoff.update(now, takeoff_airspeed, _airspeed, _current_altitude - _takeoff_ground_alt,
				       clearance_altitude_amsl - _takeoff_ground_alt);

		// yaw control is disabled once in "taking off" state
		_att_sp.fw_control_yaw_wheel = _runway_takeoff.controlYaw();

		// XXX: hacky way to pass through manual nose-wheel incrementing. need to clean this interface.
		if (_param_rwto_nudge.get()) {
			_att_sp.yaw_sp_move_rate = _manual_control_setpoint.yaw;
		}

		// tune up the lateral position control guidance when on the ground
		if (_runway_takeoff.controlYaw()) {
			_npfg.setPeriod(_param_rwto_npfg_period.get());

		}

		const Vector2f start_pos_local = _global_local_proj_ref.project(_runway_takeoff.getStartPosition()(0),
						 _runway_takeoff.getStartPosition()(1));
		const Vector2f takeoff_waypoint_local = _global_local_proj_ref.project(pos_sp_curr.lat, pos_sp_curr.lon);

		// by default set the takeoff bearing to the takeoff yaw, but override in a mission takeoff with bearing to takeoff WP
		Vector2f takeoff_bearing_vector = {cosf(_launch_current_yaw), sinf(_launch_current_yaw)};

		if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {
			// the bearing from runway start to the takeoff waypoint is followed until the clearance altitude is exceeded
			takeoff_bearing_vector = calculateTakeoffBearingVector(start_pos_local, takeoff_waypoint_local);
		}

		_npfg.setAirspeedNom(target_airspeed * _eas2tas);
		_npfg.setAirspeedMax(_param_fw_airspd_max.get() * _eas2tas);
		navigatePathTangent(local_2D_position, start_pos_local, takeoff_bearing_vector, ground_speed,
				    _wind_vel, 0.0f);

		_att_sp.roll_body = _runway_takeoff.getRoll(_npfg.getRollSetpoint());

		target_airspeed = _npfg.getAirspeedRef() / _eas2tas;

		// use npfg's bearing to commanded course, controlled via yaw angle while on runway
		const float bearing = _npfg.getBearing();

		// heading hold mode will override this bearing setpoint
		_att_sp.yaw_body = _runway_takeoff.getYaw(bearing);

		// update tecs
		const float pitch_max = _runway_takeoff.getMaxPitch(math::radians(_param_fw_p_lim_max.get()));
		const float pitch_min = _runway_takeoff.getMinPitch(math::radians(_takeoff_pitch_min.get()),
					math::radians(_param_fw_p_lim_min.get()));

		if (_runway_takeoff.resetIntegrators()) {
			// reset integrals except yaw (which also counts for the wheel controller)
			_att_sp.reset_integral = true;

			// throttle is open loop anyway during ground roll, no need to wind up the integrator
			_tecs.resetIntegrals();
		}

		tecs_update_pitch_throttle(control_interval,
					   altitude_setpoint_amsl,
					   target_airspeed,
					   pitch_min,
					   pitch_max,
					   _param_fw_thr_min.get(),
					   _param_fw_thr_max.get(),
					   _param_sinkrate_target.get(),
					   _param_fw_t_clmb_max.get());

		_tecs.set_equivalent_airspeed_min(_param_fw_airspd_min.get()); // reset after TECS calculation

		_att_sp.pitch_body = _runway_takeoff.getPitch(get_tecs_pitch());
		_att_sp.thrust_body[0] = _runway_takeoff.getThrottle(_param_fw_thr_idle.get(), get_tecs_thrust());

		_flaps_setpoint = _param_fw_flaps_to_scl.get();

		// retract ladning gear once passed the climbout state
		if (_runway_takeoff.getState() > RunwayTakeoffState::CLIMBOUT) {
			_new_landing_gear_position = landing_gear_s::GEAR_UP;
		}

	} else {
		/* Perform launch detection */
		if (!_skipping_takeoff_detection && _param_fw_laun_detcn_on.get() &&
		    _launchDetector.getLaunchDetected() < launch_detection_status_s::STATE_FLYING) {

			if (_control_mode.flag_armed) {
				/* Perform launch detection */

				/* Detect launch using body X (forward) acceleration */
				_launchDetector.update(control_interval, _body_acceleration_x);
			}

		} else	{
			/* no takeoff detection --> fly */
			_launchDetector.forceSetFlyState();
		}

		if (!_launch_detected && _launchDetector.getLaunchDetected() > launch_detection_status_s::STATE_WAITING_FOR_LAUNCH
		    && _param_fw_laun_detcn_on.get()) {
			_launch_detected = true;
			_launch_global_position = global_position;
			_takeoff_ground_alt = _current_altitude;
			_launch_current_yaw = _yaw;
		}

		const Vector2f launch_local_position = _global_local_proj_ref.project(_launch_global_position(0),
						       _launch_global_position(1));
		const Vector2f takeoff_waypoint_local = _global_local_proj_ref.project(pos_sp_curr.lat, pos_sp_curr.lon);

		// by default set the takeoff bearing to the takeoff yaw, but override in a mission takeoff with bearing to takeoff WP
		Vector2f takeoff_bearing_vector = {cosf(_launch_current_yaw), sinf(_launch_current_yaw)};

		if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {
			// the bearing from launch to the takeoff waypoint is followed until the clearance altitude is exceeded
			takeoff_bearing_vector = calculateTakeoffBearingVector(launch_local_position, takeoff_waypoint_local);
		}

		/* Set control values depending on the detection state */
		if (_launchDetector.getLaunchDetected() > launch_detection_status_s::STATE_WAITING_FOR_LAUNCH
		    && _param_fw_laun_detcn_on.get()) {
			/* Launch has been detected, hence we have to control the plane. */

			_npfg.setAirspeedNom(target_airspeed * _eas2tas);
			_npfg.setAirspeedMax(_param_fw_airspd_max.get() * _eas2tas);
			navigatePathTangent(local_2D_position, launch_local_position, takeoff_bearing_vector, ground_speed, _wind_vel,
					    0.0f);
			_att_sp.roll_body = _npfg.getRollSetpoint();
			target_airspeed = _npfg.getAirspeedRef() / _eas2tas;


			const float max_takeoff_throttle = (_launchDetector.getLaunchDetected() < launch_detection_status_s::STATE_FLYING) ?
							   _param_fw_thr_idle.get() : _param_fw_thr_max.get();

			tecs_update_pitch_throttle(control_interval,
						   altitude_setpoint_amsl,
						   target_airspeed,
						   radians(_takeoff_pitch_min.get()),
						   radians(_param_fw_p_lim_max.get()),
						   _param_fw_thr_min.get(),
						   max_takeoff_throttle,
						   _param_sinkrate_target.get(),
						   _param_fw_t_clmb_max.get());

			if (_launchDetector.getLaunchDetected() < launch_detection_status_s::STATE_FLYING) {
				// explicitly set idle throttle until motors are enabled
				_att_sp.thrust_body[0] = _param_fw_thr_idle.get();

			} else {
				_att_sp.thrust_body[0] = (_landed) ? min(_param_fw_thr_idle.get(), 1.f) : get_tecs_thrust();
			}

			_att_sp.pitch_body = get_tecs_pitch();
			_att_sp.yaw_body = _yaw; // yaw is not controlled, so set setpoint to current yaw

		} else {
			/* Tell the attitude controller to stop integrating while we are waiting for the launch */
			_att_sp.reset_integral = true;

			/* Set default roll and pitch setpoints during detection phase */
			_att_sp.roll_body = 0.0f;
			_att_sp.thrust_body[0] = _param_fw_thr_idle.get();
			_att_sp.pitch_body = radians(_takeoff_pitch_min.get());
		}

		launch_detection_status_s launch_detection_status;
		launch_detection_status.timestamp = now;
		launch_detection_status.launch_detection_state = _launchDetector.getLaunchDetected();
		_launch_detection_status_pub.publish(launch_detection_status);
	}

	_att_sp.roll_body = constrainRollNearGround(_att_sp.roll_body, _current_altitude, _takeoff_ground_alt);

	if (!_vehicle_status.in_transition_to_fw) {
		publishLocalPositionSetpoint(pos_sp_curr);
	}
}

void
FixedwingPositionControl::control_auto_landing_straight(const hrt_abstime &now, const float control_interval,
		const Vector2f &ground_speed, const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr)
{
	// first handle non-position things like airspeed and tecs settings
	const float airspeed_land = (_param_fw_lnd_airspd.get() > FLT_EPSILON) ? _param_fw_lnd_airspd.get() :
				    _param_fw_airspd_min.get();
	float adjusted_min_airspeed = _param_fw_airspd_min.get();

	if (airspeed_land < _param_fw_airspd_min.get()) {
		// adjust underspeed detection bounds for landing airspeed
		_tecs.set_equivalent_airspeed_min(airspeed_land);
		adjusted_min_airspeed = airspeed_land;
	}

	float target_airspeed = adapt_airspeed_setpoint(control_interval, airspeed_land, adjusted_min_airspeed,
				ground_speed);
	// Enable tighter altitude control for landings
	_tecs.set_altitude_error_time_constant(_param_fw_thrtc_sc.get() * _param_fw_t_h_error_tc.get());

	// now handle position
	const Vector2f local_position{_local_pos.x, _local_pos.y};
	Vector2f local_land_point = _global_local_proj_ref.project(pos_sp_curr.lat, pos_sp_curr.lon);

	initializeAutoLanding(now, pos_sp_prev, pos_sp_curr.alt, local_position, local_land_point);

	// touchdown may get nudged by manual inputs
	local_land_point = calculateTouchdownPosition(control_interval, local_land_point);
	const Vector2f landing_approach_vector = calculateLandingApproachVector();

	// calculate the altitude setpoint based on the landing glide slope
	const float along_track_dist_to_touchdown = -landing_approach_vector.unit_or_zero().dot(
				local_position - local_land_point);
	const float glide_slope = _landing_approach_entrance_rel_alt / _landing_approach_entrance_offset_vector.norm();

	// NOTE: this relative altitude can go below zero, this is intentional. in the case the vehicle is tracking the glide
	// slope at an offset above the track, making the altitude setpoint constant on intersection with terrain causes
	// an increase in throttle (to slow down and smoothly intersect the flattened altitude setpoint), which is undesirable
	// directly before the flare. instead, we keep the steady state behavior, and let the flare get triggered once at
	// the desired altitude
	const float glide_slope_rel_alt = math::min(along_track_dist_to_touchdown * glide_slope,
					  _landing_approach_entrance_rel_alt);

	const bool abort_on_terrain_measurement_timeout = checkLandingAbortBitMask(_param_fw_lnd_abort.get(),
			position_controller_landing_status_s::TERRAIN_NOT_FOUND);
	const bool abort_on_terrain_timeout = checkLandingAbortBitMask(_param_fw_lnd_abort.get(),
					      position_controller_landing_status_s::TERRAIN_TIMEOUT);
	const float terrain_alt = getLandingTerrainAltitudeEstimate(now, pos_sp_curr.alt, abort_on_terrain_measurement_timeout,
				  abort_on_terrain_timeout);
	const float glide_slope_reference_alt = (_param_fw_lnd_useter.get() ==
						TerrainEstimateUseOnLanding::kFollowTerrainRelativeLandingGlideSlope) ? terrain_alt : pos_sp_curr.alt;

	float altitude_setpoint;

	if (_current_altitude > glide_slope_reference_alt + glide_slope_rel_alt) {
		// descend to the glide slope
		altitude_setpoint = glide_slope_reference_alt + glide_slope_rel_alt;

	} else {
		// continue horizontally
		altitude_setpoint = _current_altitude;
	}

	// flare at the maximum of the altitude determined by the time before touchdown and a minimum flare altitude
	const float flare_rel_alt = math::max(_param_fw_lnd_fl_time.get() * _local_pos.vz, _param_fw_lnd_flalt.get());

	// the terrain estimate (if enabled) is always used to determine the flaring altitude
	if ((_current_altitude < terrain_alt + flare_rel_alt) || _flare_states.flaring) {
		// flare and land with minimal speed

		// flaring is a "point of no return"
		if (!_flare_states.flaring) {
			_flare_states.flaring = true;
			_flare_states.start_time = now;
			_flare_states.initial_height_rate_setpoint = -_local_pos.vz;
			_flare_states.initial_throttle_setpoint = _att_sp.thrust_body[0];
			events::send(events::ID("fixedwing_position_control_landing_flaring"), events::Log::Info,
				     "Landing, flaring");
		}

		// ramp in flare limits and setpoints with the flare time, command a soft touchdown
		const float seconds_since_flare_start = hrt_elapsed_time(&_flare_states.start_time) * 1.e-6f;
		const float flare_ramp_interpolator = math::constrain(seconds_since_flare_start / _param_fw_lnd_fl_time.get(), 0.0f,
						      1.0f);

		/* lateral guidance first, because npfg will adjust the airspeed setpoint if necessary */

		// tune up the lateral position control guidance when on the ground
		const float ground_roll_npfg_period = flare_ramp_interpolator * _param_rwto_npfg_period.get() +
						      (1.0f - flare_ramp_interpolator) * _param_npfg_period.get();
		_npfg.setPeriod(ground_roll_npfg_period);

		const Vector2f local_approach_entrance = local_land_point - landing_approach_vector;

		_npfg.setAirspeedNom(target_airspeed * _eas2tas);
		_npfg.setAirspeedMax(_param_fw_airspd_max.get() * _eas2tas);
		navigateWaypoints(local_approach_entrance, local_land_point, local_position, ground_speed, _wind_vel);
		target_airspeed = _npfg.getAirspeedRef() / _eas2tas;
		_att_sp.roll_body = _npfg.getRollSetpoint();

		// use npfg's bearing to commanded course, controlled via yaw angle while on runway
		_att_sp.yaw_body = _npfg.getBearing();

		/* longitudinal guidance */

		const float flare_ramp_interpolator_sqrt = sqrtf(flare_ramp_interpolator);

		const float height_rate_setpoint = flare_ramp_interpolator_sqrt * (-_param_fw_lnd_fl_sink.get()) +
						   (1.0f - flare_ramp_interpolator_sqrt) * _flare_states.initial_height_rate_setpoint;

		float pitch_min_rad = flare_ramp_interpolator_sqrt * radians(_param_fw_lnd_fl_pmin.get()) +
				      (1.0f - flare_ramp_interpolator_sqrt) * radians(_param_fw_p_lim_min.get());
		float pitch_max_rad = flare_ramp_interpolator_sqrt * radians(_param_fw_lnd_fl_pmax.get()) +
				      (1.0f - flare_ramp_interpolator_sqrt) * radians(_param_fw_p_lim_max.get());

		if (_param_fw_lnd_td_time.get() > FLT_EPSILON) {
			const float touchdown_time = math::max(_param_fw_lnd_td_time.get(), _param_fw_lnd_fl_time.get());

			const float touchdown_interpolator = math::constrain((seconds_since_flare_start - touchdown_time) /
							     POST_TOUCHDOWN_CLAMP_TIME, 0.0f,
							     1.0f);

			pitch_max_rad = touchdown_interpolator * _param_rwto_psp.get() + (1.0f - touchdown_interpolator) * pitch_max_rad;
			pitch_min_rad = touchdown_interpolator * _param_rwto_psp.get() + (1.0f - touchdown_interpolator) * pitch_min_rad;
		}

		// idle throttle may be >0 for internal combustion engines
		// normally set to zero for electric motors
		const float throttle_max = flare_ramp_interpolator_sqrt * _param_fw_thr_idle.get() +
					   (1.0f - flare_ramp_interpolator_sqrt) *
					   _param_fw_thr_max.get();

		tecs_update_pitch_throttle(control_interval,
					   altitude_setpoint,
					   target_airspeed,
					   pitch_min_rad,
					   pitch_max_rad,
					   _param_fw_thr_idle.get(),
					   throttle_max,
					   _param_sinkrate_target.get(),
					   _param_climbrate_target.get(),
					   true,
					   height_rate_setpoint);

		/* set the attitude and throttle commands */

		// TECS has authority (though constrained) over pitch during flare, throttle is hard set to idle
		_att_sp.pitch_body = get_tecs_pitch();

		// enable direct yaw control using rudder/wheel
		_att_sp.fw_control_yaw_wheel = true;

		// XXX: hacky way to pass through manual nose-wheel incrementing. need to clean this interface.
		if (_param_fw_lnd_nudge.get() > LandingNudgingOption::kNudgingDisabled) {
			_att_sp.yaw_sp_move_rate = _manual_control_setpoint.yaw;
		}

		// blend the height rate controlled throttle setpoints with initial throttle setting over the flare
		// ramp time period to maintain throttle command continuity when switching from altitude to height rate
		// control
		const float blended_throttle = flare_ramp_interpolator * get_tecs_thrust() + (1.0f - flare_ramp_interpolator) *
					       _flare_states.initial_throttle_setpoint;

		_att_sp.thrust_body[0] = blended_throttle;

	} else {

		// follow the glide slope

		/* lateral guidance */

		const Vector2f local_approach_entrance = local_land_point - landing_approach_vector;

		_npfg.setAirspeedNom(target_airspeed * _eas2tas);
		_npfg.setAirspeedMax(_param_fw_airspd_max.get() * _eas2tas);
		navigateWaypoints(local_approach_entrance, local_land_point, local_position, ground_speed, _wind_vel);
		target_airspeed = _npfg.getAirspeedRef() / _eas2tas;
		_att_sp.roll_body = _npfg.getRollSetpoint();

		/* longitudinal guidance */

		// open the desired max sink rate to encompass the glide slope if within the aircraft's performance limits
		// x/sqrt(x^2+1) = sin(arctan(x))
		const float glide_slope_sink_rate = airspeed_land * glide_slope / sqrtf(glide_slope * glide_slope + 1.0f);
		const float desired_max_sinkrate = math::min(math::max(glide_slope_sink_rate, _param_sinkrate_target.get()),
						   _param_fw_t_sink_max.get());

		tecs_update_pitch_throttle(control_interval,
					   altitude_setpoint,
					   target_airspeed,
					   radians(_param_fw_p_lim_min.get()),
					   radians(_param_fw_p_lim_max.get()),
					   _param_fw_thr_min.get(),
					   _param_fw_thr_max.get(),
					   desired_max_sinkrate,
					   _param_climbrate_target.get());

		/* set the attitude and throttle commands */

		_att_sp.pitch_body = get_tecs_pitch();

		// yaw is not controlled in nominal flight
		_att_sp.yaw_body = _yaw;

		// enable direct yaw control using rudder/wheel
		_att_sp.fw_control_yaw_wheel = false;

		_att_sp.thrust_body[0] = (_landed) ? _param_fw_thr_idle.get() : get_tecs_thrust();
	}

	_tecs.set_equivalent_airspeed_min(_param_fw_airspd_min.get()); // reset after TECS calculation

	_att_sp.roll_body = constrainRollNearGround(_att_sp.roll_body, _current_altitude, terrain_alt);

	_flaps_setpoint = _param_fw_flaps_lnd_scl.get();
	_spoilers_setpoint = _param_fw_spoilers_lnd.get();

	// deploy gear as soon as we're in land mode, if not already done before
	_new_landing_gear_position = landing_gear_s::GEAR_DOWN;

	if (!_vehicle_status.in_transition_to_fw) {
		publishLocalPositionSetpoint(pos_sp_curr);
	}

	landing_status_publish();
}

void
FixedwingPositionControl::control_auto_landing_circular(const hrt_abstime &now, const float control_interval,
		const Vector2f &ground_speed, const position_setpoint_s &pos_sp_curr)
{
	// first handle non-position things like airspeed and tecs settings
	const float airspeed_land = (_param_fw_lnd_airspd.get() > FLT_EPSILON) ? _param_fw_lnd_airspd.get() :
				    _param_fw_airspd_min.get();
	float adjusted_min_airspeed = _param_fw_airspd_min.get();

	if (airspeed_land < _param_fw_airspd_min.get()) {
		// adjust underspeed detection bounds for landing airspeed
		_tecs.set_equivalent_airspeed_min(airspeed_land);
		adjusted_min_airspeed = airspeed_land;
	}

	float target_airspeed = adapt_airspeed_setpoint(control_interval, airspeed_land, adjusted_min_airspeed,
				ground_speed);

	// Enable tighter altitude control for landings
	_tecs.set_altitude_error_time_constant(_param_fw_thrtc_sc.get() * _param_fw_t_h_error_tc.get());

	const Vector2f local_position{_local_pos.x, _local_pos.y};
	Vector2f local_landing_orbit_center = _global_local_proj_ref.project(pos_sp_curr.lat, pos_sp_curr.lon);

	if (_time_started_landing == 0) {
		// save time at which we started landing and reset landing abort status
		reset_landing_state();
		_time_started_landing = now;
	}

	const bool abort_on_terrain_timeout = checkLandingAbortBitMask(_param_fw_lnd_abort.get(),
					      position_controller_landing_status_s::TERRAIN_TIMEOUT);
	const float terrain_alt = getLandingTerrainAltitudeEstimate(now, pos_sp_curr.alt, false, abort_on_terrain_timeout);

	// flare at the maximum of the altitude determined by the time before touchdown and a minimum flare altitude
	const float flare_rel_alt = math::max(_param_fw_lnd_fl_time.get() * _local_pos.vz, _param_fw_lnd_flalt.get());

	float loiter_radius = pos_sp_curr.loiter_radius;

	if (fabsf(pos_sp_curr.loiter_radius) < FLT_EPSILON) {
		loiter_radius = _param_nav_loiter_rad.get();
	}

	// the terrain estimate (if enabled) is always used to determine the flaring altitude
	if ((_current_altitude < terrain_alt + flare_rel_alt) || _flare_states.flaring) {
		// flare and land with minimal speed

		// flaring is a "point of no return"
		if (!_flare_states.flaring) {
			_flare_states.flaring = true;
			_flare_states.start_time = now;
			_flare_states.initial_height_rate_setpoint = -_local_pos.vz;
			_flare_states.initial_throttle_setpoint = _att_sp.thrust_body[0];
			events::send(events::ID("fixedwing_position_control_landing_circle_flaring"), events::Log::Info,
				     "Landing, flaring");
		}

		// ramp in flare limits and setpoints with the flare time, command a soft touchdown
		const float seconds_since_flare_start = hrt_elapsed_time(&_flare_states.start_time) * 1.e-6f;
		const float flare_ramp_interpolator = math::constrain(seconds_since_flare_start / _param_fw_lnd_fl_time.get(), 0.0f,
						      1.0f);

		/* lateral guidance first, because npfg will adjust the airspeed setpoint if necessary */

		// tune up the lateral position control guidance when on the ground
		const float ground_roll_npfg_period = flare_ramp_interpolator * _param_rwto_npfg_period.get() +
						      (1.0f - flare_ramp_interpolator) * _param_npfg_period.get();

		_npfg.setPeriod(ground_roll_npfg_period);
		_npfg.setAirspeedNom(target_airspeed * _eas2tas);
		_npfg.setAirspeedMax(_param_fw_airspd_max.get() * _eas2tas);

		navigateLoiter(local_landing_orbit_center, local_position, loiter_radius,
			       pos_sp_curr.loiter_direction_counter_clockwise,
			       ground_speed, _wind_vel);
		target_airspeed = _npfg.getAirspeedRef() / _eas2tas;
		_att_sp.roll_body = _npfg.getRollSetpoint();

		_att_sp.yaw_body = _yaw; // yaw is not controlled, so set setpoint to current yaw

		/* longitudinal guidance */

		const float flare_ramp_interpolator_sqrt = sqrtf(flare_ramp_interpolator);

		const float height_rate_setpoint = flare_ramp_interpolator_sqrt * (-_param_fw_lnd_fl_sink.get()) +
						   (1.0f - flare_ramp_interpolator_sqrt) * _flare_states.initial_height_rate_setpoint;

		float pitch_min_rad = flare_ramp_interpolator_sqrt * radians(_param_fw_lnd_fl_pmin.get()) +
				      (1.0f - flare_ramp_interpolator_sqrt) * radians(_param_fw_p_lim_min.get());
		float pitch_max_rad = flare_ramp_interpolator_sqrt * radians(_param_fw_lnd_fl_pmax.get()) +
				      (1.0f - flare_ramp_interpolator_sqrt) * radians(_param_fw_p_lim_max.get());

		if (_param_fw_lnd_td_time.get() > FLT_EPSILON) {
			const float touchdown_time = math::max(_param_fw_lnd_td_time.get(), _param_fw_lnd_fl_time.get());

			const float touchdown_interpolator = math::constrain((seconds_since_flare_start - touchdown_time) /
							     POST_TOUCHDOWN_CLAMP_TIME, 0.0f, 1.0f);

			pitch_max_rad = touchdown_interpolator * _param_rwto_psp.get() + (1.0f - touchdown_interpolator) * pitch_max_rad;
			pitch_min_rad = touchdown_interpolator * _param_rwto_psp.get() + (1.0f - touchdown_interpolator) * pitch_min_rad;
		}

		// idle throttle may be >0 for internal combustion engines
		// normally set to zero for electric motors
		const float throttle_max = flare_ramp_interpolator_sqrt * _param_fw_thr_idle.get() +
					   (1.0f - flare_ramp_interpolator_sqrt) *
					   _param_fw_thr_max.get();

		tecs_update_pitch_throttle(control_interval,
					   _current_altitude, // is not controlled, control descend rate
					   target_airspeed,
					   pitch_min_rad,
					   pitch_max_rad,
					   _param_fw_thr_idle.get(),
					   throttle_max,
					   _param_sinkrate_target.get(),
					   _param_climbrate_target.get(),
					   true,
					   height_rate_setpoint);

		/* set the attitude and throttle commands */

		// TECS has authority (though constrained) over pitch during flare, throttle is hard set to idle
		_att_sp.pitch_body = get_tecs_pitch();

		// enable direct yaw control using rudder/wheel
		_att_sp.fw_control_yaw_wheel = true;

		// XXX: hacky way to pass through manual nose-wheel incrementing. need to clean this interface.
		if (_param_fw_lnd_nudge.get() > LandingNudgingOption::kNudgingDisabled) {
			_att_sp.yaw_sp_move_rate = _manual_control_setpoint.yaw;
		}

		// blend the height rate controlled throttle setpoints with initial throttle setting over the flare
		// ramp time period to maintain throttle command continuity when switching from altitude to height rate
		// control
		const float blended_throttle = flare_ramp_interpolator * get_tecs_thrust() + (1.0f - flare_ramp_interpolator) *
					       _flare_states.initial_throttle_setpoint;

		_att_sp.thrust_body[0] = blended_throttle;

	} else {

		// follow the glide slope

		/* lateral guidance */
		_npfg.setAirspeedNom(target_airspeed * _eas2tas);
		_npfg.setAirspeedMax(_param_fw_airspd_max.get() * _eas2tas);

		navigateLoiter(local_landing_orbit_center, local_position, loiter_radius,
			       pos_sp_curr.loiter_direction_counter_clockwise,
			       ground_speed, _wind_vel);
		target_airspeed = _npfg.getAirspeedRef() / _eas2tas;
		_att_sp.roll_body = _npfg.getRollSetpoint();

		/* longitudinal guidance */

		// open the desired max sink rate to encompass the glide slope if within the aircraft's performance limits
		// x/sqrt(x^2+1) = sin(arctan(x))
		const float glide_slope = math::radians(_param_fw_lnd_ang.get());
		const float glide_slope_sink_rate = airspeed_land * glide_slope / sqrtf(glide_slope * glide_slope + 1.0f);
		const float desired_max_sinkrate = math::min(math::max(glide_slope_sink_rate, _param_sinkrate_target.get()),
						   _param_fw_t_sink_max.get());
		tecs_update_pitch_throttle(control_interval,
					   _current_altitude, // is not controlled, control descend rate
					   target_airspeed,
					   radians(_param_fw_p_lim_min.get()),
					   radians(_param_fw_p_lim_max.get()),
					   _param_fw_thr_min.get(),
					   _param_fw_thr_max.get(),
					   desired_max_sinkrate,
					   _param_climbrate_target.get(),
					   false,
					   -glide_slope_sink_rate); // heightrate = -sinkrate

		/* set the attitude and throttle commands */

		_att_sp.pitch_body = get_tecs_pitch();

		// yaw is not controlled in nominal flight
		_att_sp.yaw_body = _yaw;

		// enable direct yaw control using rudder/wheel
		_att_sp.fw_control_yaw_wheel = false;

		_att_sp.thrust_body[0] = (_landed) ? _param_fw_thr_idle.get() : get_tecs_thrust();
	}

	_tecs.set_equivalent_airspeed_min(_param_fw_airspd_min.get()); // reset after TECS calculation

	_att_sp.roll_body = constrainRollNearGround(_att_sp.roll_body, _current_altitude, terrain_alt);

	_flaps_setpoint = _param_fw_flaps_lnd_scl.get();
	_spoilers_setpoint = _param_fw_spoilers_lnd.get();

	if (!_vehicle_status.in_transition_to_fw) {
		publishLocalPositionSetpoint(pos_sp_curr);
	}

	landing_status_publish();
	publishOrbitStatus(pos_sp_curr);
}

void
FixedwingPositionControl::control_manual_altitude(const float control_interval, const Vector2d &curr_pos,
		const Vector2f &ground_speed)
{
	updateManualTakeoffStatus();

	const float calibrated_airspeed_sp = adapt_airspeed_setpoint(control_interval, get_manual_airspeed_setpoint(),
					     _param_fw_airspd_min.get(), ground_speed);
	const float height_rate_sp = getManualHeightRateSetpoint();

	// TECS may try to pitch down to gain airspeed if we underspeed, constrain the pitch when underspeeding if we are
	// just passed launch
	const float min_pitch = (_completed_manual_takeoff) ? radians(_param_fw_p_lim_min.get()) :
				MIN_PITCH_DURING_MANUAL_TAKEOFF;

	float throttle_max = _param_fw_thr_max.get();

	// enable the operator to kill the throttle on ground
	if (_landed && (_manual_control_setpoint_for_airspeed < THROTTLE_THRESH)) {
		throttle_max = 0.0f;
	}

	tecs_update_pitch_throttle(control_interval,
				   _current_altitude,
				   calibrated_airspeed_sp,
				   min_pitch,
				   radians(_param_fw_p_lim_max.get()),
				   _param_fw_thr_min.get(),
				   throttle_max,
				   _param_sinkrate_target.get(),
				   _param_climbrate_target.get(),
				   false,
				   height_rate_sp);

	_att_sp.roll_body = _manual_control_setpoint.roll * radians(_param_fw_r_lim.get());
	_att_sp.yaw_body = _yaw; // yaw is not controlled, so set setpoint to current yaw

	_att_sp.thrust_body[0] = min(get_tecs_thrust(), throttle_max);
	_att_sp.pitch_body = get_tecs_pitch();
}

void
FixedwingPositionControl::control_manual_position(const float control_interval, const Vector2d &curr_pos,
		const Vector2f &ground_speed)
{
	updateManualTakeoffStatus();

	float calibrated_airspeed_sp = adapt_airspeed_setpoint(control_interval, get_manual_airspeed_setpoint(),
				       _param_fw_airspd_min.get(), ground_speed);
	const float height_rate_sp = getManualHeightRateSetpoint();

	// TECS may try to pitch down to gain airspeed if we underspeed, constrain the pitch when underspeeding if we are
	// just passed launch
	const float min_pitch = (_completed_manual_takeoff) ? radians(_param_fw_p_lim_min.get()) :
				MIN_PITCH_DURING_MANUAL_TAKEOFF;

	float throttle_max = _param_fw_thr_max.get();

	// enable the operator to kill the throttle on ground
	if (_landed && (_manual_control_setpoint_for_airspeed < THROTTLE_THRESH)) {
		throttle_max = 0.0f;
	}

	/* heading control */
	if (fabsf(_manual_control_setpoint.roll) < HDG_HOLD_MAN_INPUT_THRESH &&
	    fabsf(_manual_control_setpoint.yaw) < HDG_HOLD_MAN_INPUT_THRESH) {

		/* heading / roll is zero, lock onto current heading */
		if (fabsf(_yawrate) < HDG_HOLD_YAWRATE_THRESH && !_yaw_lock_engaged) {
			// little yaw movement, lock to current heading
			_yaw_lock_engaged = true;
		}

		/* user tries to do a takeoff in heading hold mode, reset the yaw setpoint on every iteration
			to make sure the plane does not start rolling
		*/
		if (!_completed_manual_takeoff) {
			_hdg_hold_enabled = false;
			_yaw_lock_engaged = true;
		}

		if (_yaw_lock_engaged) {

			/* just switched back from non heading-hold to heading hold */
			if (!_hdg_hold_enabled) {
				_hdg_hold_enabled = true;
				_hdg_hold_yaw = _yaw;

				get_waypoint_heading_distance(_hdg_hold_yaw, _hdg_hold_prev_wp, _hdg_hold_curr_wp, true);
			}

			/* we have a valid heading hold position, are we too close? */
			const float dist = get_distance_to_next_waypoint(_current_latitude, _current_longitude, _hdg_hold_curr_wp.lat,
					   _hdg_hold_curr_wp.lon);

			if (dist < HDG_HOLD_REACHED_DIST) {
				get_waypoint_heading_distance(_hdg_hold_yaw, _hdg_hold_prev_wp, _hdg_hold_curr_wp, false);
			}

			Vector2d prev_wp{_hdg_hold_prev_wp.lat, _hdg_hold_prev_wp.lon};
			Vector2d curr_wp{_hdg_hold_curr_wp.lat, _hdg_hold_curr_wp.lon};

			Vector2f curr_pos_local{_local_pos.x, _local_pos.y};
			Vector2f curr_wp_local = _global_local_proj_ref.project(curr_wp(0), curr_wp(1));
			Vector2f prev_wp_local = _global_local_proj_ref.project(prev_wp(0),
						 prev_wp(1));

			_npfg.setAirspeedNom(calibrated_airspeed_sp * _eas2tas);
			_npfg.setAirspeedMax(_param_fw_airspd_max.get() * _eas2tas);
			navigateWaypoints(prev_wp_local, curr_wp_local, curr_pos_local, ground_speed, _wind_vel);
			_att_sp.roll_body = _npfg.getRollSetpoint();
			calibrated_airspeed_sp = _npfg.getAirspeedRef() / _eas2tas;

			_att_sp.yaw_body = _yaw; // yaw is not controlled, so set setpoint to current yaw
		}
	}

	tecs_update_pitch_throttle(control_interval,
				   _current_altitude,
				   calibrated_airspeed_sp,
				   min_pitch,
				   radians(_param_fw_p_lim_max.get()),
				   _param_fw_thr_min.get(),
				   throttle_max,
				   _param_sinkrate_target.get(),
				   _param_climbrate_target.get(),
				   false,
				   height_rate_sp);

	if (!_yaw_lock_engaged || fabsf(_manual_control_setpoint.roll) >= HDG_HOLD_MAN_INPUT_THRESH ||
	    fabsf(_manual_control_setpoint.yaw) >= HDG_HOLD_MAN_INPUT_THRESH) {

		_hdg_hold_enabled = false;
		_yaw_lock_engaged = false;

		// do slew rate limiting on roll if enabled
		float roll_sp_new = _manual_control_setpoint.roll * radians(_param_fw_r_lim.get());
		const float roll_rate_slew_rad = radians(_param_fw_pn_r_slew_max.get());

		if (control_interval > 0.f && roll_rate_slew_rad > 0.f) {
			roll_sp_new = constrain(roll_sp_new, _att_sp.roll_body - roll_rate_slew_rad * control_interval,
						_att_sp.roll_body + roll_rate_slew_rad * control_interval);
		}

		_att_sp.roll_body = roll_sp_new;
		_att_sp.yaw_body = _yaw; // yaw is not controlled, so set setpoint to current yaw
	}

	_att_sp.thrust_body[0] = min(get_tecs_thrust(), throttle_max);
	_att_sp.pitch_body = get_tecs_pitch();
}

float
FixedwingPositionControl::get_tecs_pitch()
{
	if (_tecs_is_running) {
		return _tecs.get_pitch_setpoint() + radians(_param_fw_psp_off.get());
	}

	// return level flight pitch offset to prevent stale tecs state when it's not running
	return radians(_param_fw_psp_off.get());
}

float
FixedwingPositionControl::get_tecs_thrust()
{
	if (_tecs_is_running) {
		return min(_tecs.get_throttle_setpoint(), 1.f);
	}

	// return 0 to prevent stale tecs state when it's not running
	return 0.0f;
}

void
FixedwingPositionControl::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	/* only run controller if position changed */

	if (_local_pos_sub.update(&_local_pos)) {

		const float control_interval = math::constrain((_local_pos.timestamp - _last_time_position_control_called) * 1e-6f,
					       MIN_AUTO_TIMESTEP, MAX_AUTO_TIMESTEP);
		_last_time_position_control_called = _local_pos.timestamp;

		// check for parameter updates
		if (_parameter_update_sub.updated()) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			parameters_update();
		}

		vehicle_global_position_s gpos;

		if (_global_pos_sub.update(&gpos)) {
			_current_latitude = gpos.lat;
			_current_longitude = gpos.lon;
		}

		_current_altitude = -_local_pos.z + _local_pos.ref_alt; // Altitude AMSL in meters

		// handle estimator reset events. we only adjust setpoins for manual modes
		if (_control_mode.flag_control_manual_enabled) {
			if (_control_mode.flag_control_altitude_enabled && _local_pos.vz_reset_counter != _alt_reset_counter) {
				// make TECS accept step in altitude and demanded altitude
				_tecs.handle_alt_step(_current_altitude, -_local_pos.vz);
			}

			// adjust navigation waypoints in position control mode
			if (_control_mode.flag_control_altitude_enabled && _control_mode.flag_control_velocity_enabled
			    && _local_pos.vxy_reset_counter != _pos_reset_counter) {

				// reset heading hold flag, which will re-initialise position control
				_hdg_hold_enabled = false;
			}
		}

		// update the reset counters in any case
		_alt_reset_counter = _local_pos.vz_reset_counter;
		_pos_reset_counter = _local_pos.vxy_reset_counter;

		// Convert Local setpoints to global setpoints
		if (!_global_local_proj_ref.isInitialized()
		    || (_global_local_proj_ref.getProjectionReferenceTimestamp() != _local_pos.ref_timestamp)
		    || (_local_pos.vxy_reset_counter != _pos_reset_counter)) {

			_global_local_proj_ref.initReference(_local_pos.ref_lat, _local_pos.ref_lon,
							     _local_pos.ref_timestamp);
			_global_local_alt0 = _local_pos.ref_alt;
		}

		if (_control_mode.flag_control_offboard_enabled) {
			trajectory_setpoint_s trajectory_setpoint;

			if (_trajectory_setpoint_sub.update(&trajectory_setpoint)) {
				bool valid_setpoint = false;
				_pos_sp_triplet = {}; // clear any existing
				_pos_sp_triplet.timestamp = trajectory_setpoint.timestamp;
				_pos_sp_triplet.current.timestamp = trajectory_setpoint.timestamp;
				_pos_sp_triplet.current.cruising_speed = NAN; // ignored
				_pos_sp_triplet.current.cruising_throttle = NAN; // ignored
				_pos_sp_triplet.current.vx = NAN;
				_pos_sp_triplet.current.vy = NAN;
				_pos_sp_triplet.current.vz = NAN;
				_pos_sp_triplet.current.lat = static_cast<double>(NAN);
				_pos_sp_triplet.current.lon = static_cast<double>(NAN);
				_pos_sp_triplet.current.alt = NAN;

				if (Vector3f(trajectory_setpoint.position).isAllFinite()) {
					if (_global_local_proj_ref.isInitialized()) {
						double lat;
						double lon;
						_global_local_proj_ref.reproject(trajectory_setpoint.position[0], trajectory_setpoint.position[1], lat, lon);
						valid_setpoint = true;
						_pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
						_pos_sp_triplet.current.lat = lat;
						_pos_sp_triplet.current.lon = lon;
						_pos_sp_triplet.current.alt = _global_local_alt0 - trajectory_setpoint.position[2];
					}

				}

				if (Vector3f(trajectory_setpoint.velocity).isAllFinite()) {
					valid_setpoint = true;
					_pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
					_pos_sp_triplet.current.vx = trajectory_setpoint.velocity[0];
					_pos_sp_triplet.current.vy = trajectory_setpoint.velocity[1];
					_pos_sp_triplet.current.vz = trajectory_setpoint.velocity[2];

					if (Vector3f(trajectory_setpoint.acceleration).isAllFinite()) {
						Vector2f velocity_sp_2d(trajectory_setpoint.velocity[0], trajectory_setpoint.velocity[1]);
						Vector2f normalized_velocity_sp_2d = velocity_sp_2d.normalized();
						Vector2f acceleration_sp_2d(trajectory_setpoint.acceleration[0], trajectory_setpoint.acceleration[1]);
						Vector2f acceleration_normal = acceleration_sp_2d - acceleration_sp_2d.dot(normalized_velocity_sp_2d) *
									       normalized_velocity_sp_2d;
						float direction = -normalized_velocity_sp_2d.cross(acceleration_normal.normalized());
						_pos_sp_triplet.current.loiter_radius = direction * velocity_sp_2d.norm() * velocity_sp_2d.norm() /
											acceleration_normal.norm();

					} else {
						_pos_sp_triplet.current.loiter_radius = NAN;
					}
				}

				_position_setpoint_current_valid = valid_setpoint;
			}

		} else {
			if (_pos_sp_triplet_sub.update(&_pos_sp_triplet)) {

				_position_setpoint_previous_valid = PX4_ISFINITE(_pos_sp_triplet.previous.lat)
								    && PX4_ISFINITE(_pos_sp_triplet.previous.lon)
								    && PX4_ISFINITE(_pos_sp_triplet.previous.alt);

				_position_setpoint_current_valid = PX4_ISFINITE(_pos_sp_triplet.current.lat)
								   && PX4_ISFINITE(_pos_sp_triplet.current.lon)
								   && PX4_ISFINITE(_pos_sp_triplet.current.alt);

				_position_setpoint_next_valid = PX4_ISFINITE(_pos_sp_triplet.next.lat)
								&& PX4_ISFINITE(_pos_sp_triplet.next.lon)
								&& PX4_ISFINITE(_pos_sp_triplet.next.alt);

				// reset the altitude foh (first order hold) logic
				_min_current_sp_distance_xy = FLT_MAX;
			}
		}

		airspeed_poll();
		manual_control_setpoint_poll();
		vehicle_attitude_poll();
		vehicle_command_poll();
		vehicle_control_mode_poll();
		wind_poll();

		vehicle_air_data_s air_data;

		if (_vehicle_air_data_sub.update(&air_data)) {
			_air_density = PX4_ISFINITE(air_data.rho) ? air_data.rho : _air_density;
		}

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.update(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
			}
		}

		_vehicle_status_sub.update(&_vehicle_status);

		Vector2d curr_pos(_current_latitude, _current_longitude);
		Vector2f ground_speed(_local_pos.vx, _local_pos.vy);

		set_control_mode_current(_local_pos.timestamp);

		update_in_air_states(_local_pos.timestamp);

		// update lateral guidance timesteps for slewrates
		_npfg.setDt(control_interval);

		// restore nominal TECS parameters in case changed intermittently (e.g. in landing handling)
		_tecs.set_speed_weight(_param_fw_t_spdweight.get());
		_tecs.set_altitude_error_time_constant(_param_fw_t_h_error_tc.get());

		// restore lateral-directional guidance parameters (changed in takeoff mode)
		_npfg.setPeriod(_param_npfg_period.get());

		_att_sp.reset_integral = false;

		// by default no flaps/spoilers, is overwritten below in certain modes
		_flaps_setpoint = 0.f;
		_spoilers_setpoint = 0.f;

		// by default we don't want yaw to be contoller directly with rudder
		_att_sp.fw_control_yaw_wheel = false;

		// default to zero - is used (IN A HACKY WAY) to pass direct nose wheel steering via yaw stick to the actuators during auto takeoff
		_att_sp.yaw_sp_move_rate = 0.0f;

		if (_control_mode_current != FW_POSCTRL_MODE_AUTO_LANDING_STRAIGHT
		    && _control_mode_current != FW_POSCTRL_MODE_AUTO_LANDING_CIRCULAR) {
			reset_landing_state();
		}

		if (_control_mode_current != FW_POSCTRL_MODE_AUTO_TAKEOFF) {
			reset_takeoff_state();
		}

		int8_t old_landing_gear_position = _new_landing_gear_position;
		_new_landing_gear_position = landing_gear_s::GEAR_KEEP; // is overwritten in Takeoff and Land

		switch (_control_mode_current) {
		case FW_POSCTRL_MODE_AUTO: {
				control_auto(control_interval, curr_pos, ground_speed, _pos_sp_triplet.previous, _pos_sp_triplet.current,
					     _pos_sp_triplet.next);
				break;
			}

		case FW_POSCTRL_MODE_AUTO_ALTITUDE: {
				control_auto_fixed_bank_alt_hold(control_interval);
				break;
			}

		case FW_POSCTRL_MODE_AUTO_CLIMBRATE: {
				control_auto_descend(control_interval);
				break;
			}

		case FW_POSCTRL_MODE_AUTO_LANDING_STRAIGHT: {
				control_auto_landing_straight(_local_pos.timestamp, control_interval, ground_speed, _pos_sp_triplet.previous,
							      _pos_sp_triplet.current);
				break;
			}

		case FW_POSCTRL_MODE_AUTO_LANDING_CIRCULAR: {
				control_auto_landing_circular(_local_pos.timestamp, control_interval, ground_speed, _pos_sp_triplet.current);
				break;
			}

		case FW_POSCTRL_MODE_AUTO_TAKEOFF: {
				control_auto_takeoff(_local_pos.timestamp, control_interval, curr_pos, ground_speed, _pos_sp_triplet.current);
				break;
			}

		case FW_POSCTRL_MODE_MANUAL_POSITION: {
				control_manual_position(control_interval, curr_pos, ground_speed);
				break;
			}

		case FW_POSCTRL_MODE_MANUAL_ALTITUDE: {
				control_manual_altitude(control_interval, curr_pos, ground_speed);
				break;
			}

		case FW_POSCTRL_MODE_OTHER: {
				_att_sp.thrust_body[0] = min(_att_sp.thrust_body[0], _param_fw_thr_max.get());

				_tecs.initialize(_current_altitude, -_local_pos.vz, _airspeed, _eas2tas);

				break;
			}

		}

		if (_control_mode_current != FW_POSCTRL_MODE_OTHER) {

			if (_control_mode.flag_control_manual_enabled) {
				_att_sp.roll_body = constrain(_att_sp.roll_body, -radians(_param_fw_r_lim.get()),
							      radians(_param_fw_r_lim.get()));
				_att_sp.pitch_body = constrain(_att_sp.pitch_body, radians(_param_fw_p_lim_min.get()),
							       radians(_param_fw_p_lim_max.get()));
			}

			if (_control_mode.flag_control_position_enabled ||
			    _control_mode.flag_control_velocity_enabled ||
			    _control_mode.flag_control_acceleration_enabled ||
			    _control_mode.flag_control_altitude_enabled ||
			    _control_mode.flag_control_climb_rate_enabled) {

				const Quatf q(Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body));
				q.copyTo(_att_sp.q_d);

				_att_sp.timestamp = hrt_absolute_time();
				_attitude_sp_pub.publish(_att_sp);

				// only publish status in full FW mode
				if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
				    || _vehicle_status.in_transition_mode) {
					status_publish();

				}
			}
		}

		// if there's any change in landing gear setpoint publish it
		if (_new_landing_gear_position != old_landing_gear_position
		    && _new_landing_gear_position != landing_gear_s::GEAR_KEEP) {

			landing_gear_s landing_gear = {};
			landing_gear.landing_gear = _new_landing_gear_position;
			landing_gear.timestamp = hrt_absolute_time();
			_landing_gear_pub.publish(landing_gear);
		}

		// In Manual modes flaps and spoilers are directly controlled in the Attitude controller and not published here
		if (_control_mode.flag_control_auto_enabled
		    && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
			normalized_unsigned_setpoint_s flaps_setpoint;
			flaps_setpoint.normalized_setpoint = _flaps_setpoint;
			flaps_setpoint.timestamp = hrt_absolute_time();
			_flaps_setpoint_pub.publish(flaps_setpoint);

			normalized_unsigned_setpoint_s spoilers_setpoint;
			spoilers_setpoint.normalized_setpoint = _spoilers_setpoint;
			spoilers_setpoint.timestamp = hrt_absolute_time();
			_spoilers_setpoint_pub.publish(spoilers_setpoint);
		}

		perf_end(_loop_perf);
	}
}

void
FixedwingPositionControl::reset_takeoff_state()
{
	_runway_takeoff.reset();

	_launchDetector.reset();

	_launch_detected = false;

	_takeoff_ground_alt = _current_altitude;
}

void
FixedwingPositionControl::reset_landing_state()
{
	_time_started_landing = 0;

	_flare_states = FlareStates{};

	_lateral_touchdown_position_offset = 0.0f;

	_last_time_terrain_alt_was_valid = 0;

	// reset abort land, unless loitering after an abort
	if (_landing_abort_status && (_pos_sp_triplet.current.type != position_setpoint_s::SETPOINT_TYPE_LOITER)) {

		updateLandingAbortStatus(position_controller_landing_status_s::NOT_ABORTED);
	}
}

float FixedwingPositionControl::calculateTrimThrottle(float throttle_min,
		float throttle_max, float airspeed_sp)
{
	float throttle_trim =
		_param_fw_thr_trim.get(); // throttle required for level flight at trim airspeed, at sea level (standard atmosphere)

	// Drag modelling (parasite drag): calculate mapping airspeed-->throttle, assuming a linear relation with different gradients
	// above and below trim. This is tunable thorugh FW_THR_ASPD_MIN and FW_THR_ASPD_MAX.
	const float slope_below_trim = (_param_fw_thr_trim.get() - _param_fw_thr_aspd_min.get()) /
				       (_param_fw_airspd_trim.get() - _param_fw_airspd_min.get());
	const float slope_above_trim = (_param_fw_thr_aspd_max.get() - _param_fw_thr_trim.get()) /
				       (_param_fw_airspd_max.get() - _param_fw_airspd_trim.get());

	if (PX4_ISFINITE(airspeed_sp) && PX4_ISFINITE(slope_below_trim) && _param_fw_thr_aspd_min.get() > FLT_EPSILON
	    && airspeed_sp < _param_fw_airspd_trim.get()) {
		throttle_trim = _param_fw_thr_trim.get() - slope_below_trim * (_param_fw_airspd_trim.get() - airspeed_sp);

	} else if (PX4_ISFINITE(airspeed_sp) && PX4_ISFINITE(slope_above_trim) && _param_fw_thr_aspd_max.get() > FLT_EPSILON
		   && airspeed_sp > _param_fw_airspd_trim.get()) {
		throttle_trim = _param_fw_thr_trim.get() + slope_above_trim * (airspeed_sp - _param_fw_airspd_trim.get());
	}

	float weight_ratio = 1.0f;

	if (_param_weight_base.get() > FLT_EPSILON && _param_weight_gross.get() > FLT_EPSILON) {
		weight_ratio = math::constrain(_param_weight_gross.get() / _param_weight_base.get(), MIN_WEIGHT_RATIO,
					       MAX_WEIGHT_RATIO);
	}

	float air_density_throttle_scale = 1.0f;

	if (PX4_ISFINITE(_air_density)) {
		// scale throttle as a function of sqrt(rho0/rho)
		const float eas2tas = sqrtf(CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C / _air_density);
		const float eas2tas_at_5000m_amsl = sqrtf(CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C / AIR_DENSITY_STANDARD_ATMOS_5000_AMSL);
		air_density_throttle_scale = constrain(eas2tas, 1.f, eas2tas_at_5000m_amsl);
	}

	// compensate trim throttle for both weight and air density
	return math::constrain(throttle_trim * sqrtf(weight_ratio) * air_density_throttle_scale, throttle_min, throttle_max);
}

void
FixedwingPositionControl::tecs_update_pitch_throttle(const float control_interval, float alt_sp, float airspeed_sp,
		float pitch_min_rad, float pitch_max_rad, float throttle_min, float throttle_max,
		const float desired_max_sinkrate, const float desired_max_climbrate,
		bool disable_underspeed_detection, float hgt_rate_sp)
{
	_tecs_is_running = true;

	// do not run TECS if vehicle is a VTOL and we are in rotary wing mode or in transition
	// (it should also not run during VTOL blending because airspeed is too low still)
	if (_vehicle_status.is_vtol) {
		if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING || _vehicle_status.in_transition_mode) {
			_tecs_is_running = false;
		}

		if (_vehicle_status.in_transition_mode) {
			// we're in transition
			_was_in_transition = true;

			// set this to transition airspeed to init tecs correctly
			if (_param_fw_arsp_mode.get() == 1 && PX4_ISFINITE(_param_airspeed_trans)) {
				// some vtols fly without airspeed sensor
				_airspeed_after_transition = _param_airspeed_trans;

			} else {
				_airspeed_after_transition = _airspeed;
			}

			_airspeed_after_transition = constrain(_airspeed_after_transition, _param_fw_airspd_min.get(),
							       _param_fw_airspd_max.get());

		} else if (_was_in_transition) {
			// after transition we ramp up desired airspeed from the speed we had coming out of the transition
			_airspeed_after_transition += control_interval * 2.0f; // increase 2m/s

			if (_airspeed_after_transition < airspeed_sp && _airspeed < airspeed_sp) {
				airspeed_sp = max(_airspeed_after_transition, _airspeed);

			} else {
				_was_in_transition = false;
				_airspeed_after_transition = 0.0f;
			}
		}
	}

	if (!_tecs_is_running) {
		// next time we run TECS we should reinitialize states
		_reinitialize_tecs = true;
		return;
	}

	// We need an altitude lock to calculate the TECS control
	if (_local_pos.timestamp == 0) {
		_reinitialize_tecs = true;
	}

	if (_reinitialize_tecs) {
		_tecs.initialize(_current_altitude, -_local_pos.vz, _airspeed, _eas2tas);
		_reinitialize_tecs = false;
	}

	/* No underspeed protection in landing mode */
	_tecs.set_detect_underspeed_enabled(!disable_underspeed_detection);

	if (_landed) {
		_tecs.initialize(_current_altitude, -_local_pos.vz, _airspeed, _eas2tas);
	}

	/* update TECS vehicle state estimates */
	const float throttle_trim_adjusted = calculateTrimThrottle(throttle_min,
					     throttle_max, airspeed_sp);

	// HOTFIX: the airspeed rate estimate using acceleration in body-forward direction has shown to lead to high biases
	// when flying tight turns. It's in this case much safer to just set the estimated airspeed rate to 0.
	const float airspeed_rate_estimate = 0.f;

	_tecs.update(_pitch - radians(_param_fw_psp_off.get()),
		     _current_altitude,
		     alt_sp,
		     airspeed_sp,
		     _airspeed,
		     _eas2tas,
		     throttle_min,
		     throttle_max,
		     _param_fw_thr_trim.get(),
		     throttle_trim_adjusted,
		     pitch_min_rad - radians(_param_fw_psp_off.get()),
		     pitch_max_rad - radians(_param_fw_psp_off.get()),
		     desired_max_climbrate,
		     desired_max_sinkrate,
		     airspeed_rate_estimate,
		     -_local_pos.vz,
		     hgt_rate_sp);

	tecs_status_publish(alt_sp, airspeed_sp, airspeed_rate_estimate, throttle_trim_adjusted);
}

float
FixedwingPositionControl::constrainRollNearGround(const float roll_setpoint, const float altitude,
		const float terrain_altitude) const
{
	// we want the wings level when at the wing height above ground
	const float height_above_ground = math::max(altitude - (terrain_altitude + _param_fw_wing_height.get()), 0.0f);

	// this is a conservative (linear) approximation of the roll angle that would cause wing tip strike
	// roll strike = arcsin( 2 * height / span )
	// d(roll strike)/d(height) = 2 / span / cos(2 * height / span)
	// d(roll strike)/d(height) (@height=0) = 2 / span
	// roll strike ~= 2 * height / span
	const float roll_wingtip_strike = 2.0f * height_above_ground / _param_fw_wing_span.get();

	return math::constrain(roll_setpoint, -roll_wingtip_strike, roll_wingtip_strike);
}

Vector2f
FixedwingPositionControl::calculateTakeoffBearingVector(const Vector2f &launch_position,
		const Vector2f &takeoff_waypoint) const
{
	Vector2f takeoff_bearing_vector = takeoff_waypoint - launch_position;

	if (takeoff_bearing_vector.norm_squared() > FLT_EPSILON) {
		takeoff_bearing_vector.normalize();

	} else {
		// TODO: a new bearing only based fixed-wing takeoff command / mission item will get rid of the need
		// for this check

		// takeoff in the direction of the airframe
		takeoff_bearing_vector(0) = cosf(_yaw);
		takeoff_bearing_vector(1) = sinf(_yaw);
	}

	return takeoff_bearing_vector;
}

void
FixedwingPositionControl::initializeAutoLanding(const hrt_abstime &now, const position_setpoint_s &pos_sp_prev,
		const float land_point_altitude, const Vector2f &local_position, const Vector2f &local_land_point)
{
	if (_time_started_landing == 0) {

		float height_above_land_point;
		Vector2f local_approach_entrance;

		// set the landing approach entrance location when we have just started the landing and store it
		// NOTE: the landing approach vector is relative to the land point. ekf resets may cause a local frame
		// jump, so we reference to the land point, which is globally referenced and will update
		if (_position_setpoint_previous_valid) {
			height_above_land_point = pos_sp_prev.alt - land_point_altitude;
			local_approach_entrance = _global_local_proj_ref.project(pos_sp_prev.lat, pos_sp_prev.lon);

		} else {
			// no valid previous waypoint, construct one from the glide slope and direction from current
			// position to land point

			// NOTE: this is not really a supported use case at the moment, this is just bandaiding any
			// ill-advised usage of the current implementation

			// TODO: proper handling of on-the-fly landing points would need to involve some more sophisticated
			// landing pattern generation and corresponding logic

			height_above_land_point = _current_altitude - land_point_altitude;
			local_approach_entrance = local_position;
		}

		_landing_approach_entrance_rel_alt = math::max(height_above_land_point, FLT_EPSILON);

		const Vector2f landing_approach_vector = local_land_point - local_approach_entrance;
		float landing_approach_distance = landing_approach_vector.norm();

		const float max_glide_slope = tanf(math::radians(_param_fw_lnd_ang.get()));
		const float glide_slope = _landing_approach_entrance_rel_alt / landing_approach_distance;

		if (glide_slope > max_glide_slope) {
			// rescale the landing distance - this will have the same effect as dropping down the approach
			// entrance altitude on the vehicle's behavior. if we reach here.. it means the navigator checks
			// didn't work, or something is using the control_auto_landing_straight() method inappropriately
			landing_approach_distance = _landing_approach_entrance_rel_alt / max_glide_slope;
		}

		if (landing_approach_vector.norm_squared() > FLT_EPSILON) {
			_landing_approach_entrance_offset_vector = -landing_approach_vector.unit_or_zero() * landing_approach_distance;

		} else {
			// land in direction of airframe
			_landing_approach_entrance_offset_vector = Vector2f({cosf(_yaw), sinf(_yaw)}) * landing_approach_distance;
		}

		// save time at which we started landing and reset landing abort status
		reset_landing_state();
		_time_started_landing = now;
	}
}

Vector2f
FixedwingPositionControl::calculateTouchdownPosition(const float control_interval, const Vector2f &local_land_position)
{
	if (fabsf(_manual_control_setpoint.yaw) > MANUAL_TOUCHDOWN_NUDGE_INPUT_DEADZONE
	    && _param_fw_lnd_nudge.get() > LandingNudgingOption::kNudgingDisabled
	    && !_flare_states.flaring) {
		// laterally nudge touchdown location with yaw stick
		// positive is defined in the direction of a right hand turn starting from the approach vector direction
		const float signed_deadzone_threshold = MANUAL_TOUCHDOWN_NUDGE_INPUT_DEADZONE * math::signNoZero(
				_manual_control_setpoint.yaw);
		_lateral_touchdown_position_offset += (_manual_control_setpoint.yaw - signed_deadzone_threshold) *
						      MAX_TOUCHDOWN_POSITION_NUDGE_RATE * control_interval;
		_lateral_touchdown_position_offset =  math::constrain(_lateral_touchdown_position_offset, -_param_fw_lnd_td_off.get(),
						      _param_fw_lnd_td_off.get());
	}

	const Vector2f approach_unit_vector = -_landing_approach_entrance_offset_vector.unit_or_zero();
	const Vector2f approach_unit_normal_vector{-approach_unit_vector(1), approach_unit_vector(0)};

	return local_land_position + approach_unit_normal_vector * _lateral_touchdown_position_offset;
}

Vector2f
FixedwingPositionControl::calculateLandingApproachVector() const
{
	Vector2f landing_approach_vector = -_landing_approach_entrance_offset_vector;
	const Vector2f approach_unit_vector = landing_approach_vector.unit_or_zero();
	const Vector2f approach_unit_normal_vector{-approach_unit_vector(1), approach_unit_vector(0)};

	if (_param_fw_lnd_nudge.get() == LandingNudgingOption::kNudgeApproachAngle) {
		// nudge the approach angle -- i.e. we adjust the approach vector to reach from the original approach
		// entrance position to the newly nudged touchdown point
		// NOTE: this lengthens the landing distance.. which will adjust the glideslope height slightly
		landing_approach_vector += approach_unit_normal_vector * _lateral_touchdown_position_offset;
	}

	// if _param_fw_lnd_nudge.get() == LandingNudgingOption::kNudgingDisabled, no nudging

	// if _param_fw_lnd_nudge.get() == LandingNudgingOption::kNudgeApproachPath, the full path (including approach
	// entrance point) is nudged with the touchdown point, which does not require any additions to the approach vector

	return landing_approach_vector;
}

float
FixedwingPositionControl::getLandingTerrainAltitudeEstimate(const hrt_abstime &now, const float land_point_altitude,
		const bool abort_on_terrain_measurement_timeout, const bool abort_on_terrain_timeout)
{
	if (_param_fw_lnd_useter.get() > TerrainEstimateUseOnLanding::kDisableTerrainEstimation) {

		if (_local_pos.dist_bottom_valid) {

			const float terrain_estimate = _local_pos.ref_alt + -_local_pos.z - _local_pos.dist_bottom;
			_last_valid_terrain_alt_estimate = terrain_estimate;
			_last_time_terrain_alt_was_valid = now;

			return terrain_estimate;
		}

		if (_last_time_terrain_alt_was_valid == 0) {

			const bool terrain_first_measurement_timed_out = (now - _time_started_landing) > TERRAIN_ALT_FIRST_MEASUREMENT_TIMEOUT;

			if (terrain_first_measurement_timed_out && abort_on_terrain_measurement_timeout) {
				updateLandingAbortStatus(position_controller_landing_status_s::TERRAIN_NOT_FOUND);
			}

			return land_point_altitude;
		}

		if (!_local_pos.dist_bottom_valid) {

			const bool terrain_timed_out = (now - _last_time_terrain_alt_was_valid) > TERRAIN_ALT_TIMEOUT;

			if (terrain_timed_out && abort_on_terrain_timeout) {
				updateLandingAbortStatus(position_controller_landing_status_s::TERRAIN_TIMEOUT);
			}

			return _last_valid_terrain_alt_estimate;
		}
	}

	return land_point_altitude;
}

bool FixedwingPositionControl::checkLandingAbortBitMask(const uint8_t automatic_abort_criteria_bitmask,
		uint8_t landing_abort_criterion)
{
	// landing abort status contains a manual criterion at abort_status==1, need to subtract 2 to directly compare
	// to automatic criteria bits from the parameter FW_LND_ABORT
	if (landing_abort_criterion <= 1) {
		return false;
	}

	landing_abort_criterion -= 2;

	return ((1 << landing_abort_criterion) & automatic_abort_criteria_bitmask) == (1 << landing_abort_criterion);
}

void FixedwingPositionControl::publishLocalPositionSetpoint(const position_setpoint_s &current_waypoint)
{
	vehicle_local_position_setpoint_s local_position_setpoint{};
	local_position_setpoint.timestamp = hrt_absolute_time();

	Vector2f current_setpoint;

	current_setpoint = _closest_point_on_path;

	local_position_setpoint.x = current_setpoint(0);
	local_position_setpoint.y = current_setpoint(1);
	local_position_setpoint.z = _global_local_alt0 - current_waypoint.alt;
	local_position_setpoint.yaw = NAN;
	local_position_setpoint.yawspeed = NAN;
	local_position_setpoint.vx = NAN;
	local_position_setpoint.vy = NAN;
	local_position_setpoint.vz = NAN;
	local_position_setpoint.acceleration[0] = NAN;
	local_position_setpoint.acceleration[1] = NAN;
	local_position_setpoint.acceleration[2] = NAN;
	local_position_setpoint.thrust[0] = _att_sp.thrust_body[0];
	local_position_setpoint.thrust[1] = _att_sp.thrust_body[1];
	local_position_setpoint.thrust[2] = _att_sp.thrust_body[2];
	_local_pos_sp_pub.publish(local_position_setpoint);
}

void FixedwingPositionControl::publishOrbitStatus(const position_setpoint_s pos_sp)
{
	orbit_status_s orbit_status{};
	orbit_status.timestamp = hrt_absolute_time();
	float loiter_radius = pos_sp.loiter_radius * (pos_sp.loiter_direction_counter_clockwise ? -1.f : 1.f);

	if (fabsf(loiter_radius) < FLT_EPSILON) {
		loiter_radius = _param_nav_loiter_rad.get();
	}

	orbit_status.radius = loiter_radius;
	orbit_status.frame = 0; // MAV_FRAME::MAV_FRAME_GLOBAL
	orbit_status.x = static_cast<double>(pos_sp.lat);
	orbit_status.y = static_cast<double>(pos_sp.lon);
	orbit_status.z = pos_sp.alt;
	orbit_status.yaw_behaviour = orbit_status_s::ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE;
	_orbit_status_pub.publish(orbit_status);
}

void FixedwingPositionControl::navigateWaypoints(const Vector2f &waypoint_A, const Vector2f &waypoint_B,
		const Vector2f &vehicle_pos, const Vector2f &ground_vel, const Vector2f &wind_vel)
{
	// similar to logic found in ECL_L1_Pos_Controller method of same name
	// BUT no arbitrary max approach angle, approach entirely determined by generated
	// bearing vectors

	const Vector2f vector_A_to_B = waypoint_B - waypoint_A;
	const Vector2f vector_B_to_A = -vector_A_to_B;
	const Vector2f vector_A_to_vehicle = vehicle_pos - waypoint_A;
	const Vector2f vector_B_to_vehicle = vehicle_pos - waypoint_B;
	Vector2f desired_path = vector_A_to_B; // this is the default path tangent, but is overridden below

	if (vector_A_to_B.norm() < FLT_EPSILON) {
		// the waypoints are on top of each other and should be considered as a
		// single waypoint, fly directly to it
		if (vector_A_to_vehicle.norm() > FLT_EPSILON) {
			desired_path = -vector_A_to_vehicle;

		} else {
			// Fly to a point and on it. Stay to the current control. Do not update the npfg library to get last output.
			return;
		}

	} else if ((vector_A_to_B.dot(vector_A_to_vehicle) < -FLT_EPSILON)) {
		// we are in front of waypoint A, fly directly to it until we are within switch distance.
		if (vector_A_to_vehicle.norm() > _npfg.switchDistance(500.0f)) {
			desired_path = -vector_A_to_vehicle;
		}

	} else if (vector_B_to_A.dot(vector_B_to_vehicle) < -FLT_EPSILON) {
		// we are behind waypoint B, fly directly to it
		desired_path = -vector_B_to_vehicle;
	}

	// track the line segment
	Vector2f unit_path_tangent{desired_path.normalized()};
	_target_bearing = atan2f(unit_path_tangent(1), unit_path_tangent(0));
	_closest_point_on_path = waypoint_A + vector_A_to_vehicle.dot(unit_path_tangent) * unit_path_tangent;
	_npfg.guideToPath(vehicle_pos, ground_vel, wind_vel, unit_path_tangent, waypoint_A, 0.0f);
} // navigateWaypoints

void FixedwingPositionControl::navigateLoiter(const Vector2f &loiter_center, const Vector2f &vehicle_pos,
		float radius, bool loiter_direction_counter_clockwise, const Vector2f &ground_vel, const Vector2f &wind_vel)
{
	const float loiter_direction_multiplier = loiter_direction_counter_clockwise ? -1.f : 1.f;

	Vector2f vector_center_to_vehicle = vehicle_pos - loiter_center;
	const float dist_to_center = vector_center_to_vehicle.norm();

	// find the direction from the circle center to the closest point on its perimeter
	// from the vehicle position
	Vector2f unit_vec_center_to_closest_pt;

	if (dist_to_center < 0.1f) {
		// the logic breaks down at the circle center, employ some mitigation strategies
		// until we exit this region
		if (ground_vel.norm() < 0.1f) {
			// arbitrarily set the point in the northern top of the circle
			unit_vec_center_to_closest_pt = Vector2f{1.0f, 0.0f};

		} else {
			// set the point in the direction we are moving
			unit_vec_center_to_closest_pt = ground_vel.normalized();
		}

	} else {
		// set the point in the direction of the aircraft
		unit_vec_center_to_closest_pt = vector_center_to_vehicle.normalized();
	}

	// 90 deg clockwise rotation * loiter direction
	const Vector2f unit_path_tangent = loiter_direction_multiplier * Vector2f{-unit_vec_center_to_closest_pt(1), unit_vec_center_to_closest_pt(0)};

	float path_curvature = loiter_direction_multiplier / radius;
	_target_bearing = atan2f(unit_path_tangent(1), unit_path_tangent(0));
	_closest_point_on_path = unit_vec_center_to_closest_pt * radius + loiter_center;
	_npfg.guideToPath(vehicle_pos, ground_vel, wind_vel, unit_path_tangent,
			  loiter_center + unit_vec_center_to_closest_pt * radius, path_curvature);
} // navigateLoiter


void FixedwingPositionControl::navigatePathTangent(const matrix::Vector2f &vehicle_pos,
		const matrix::Vector2f &position_setpoint,
		const matrix::Vector2f &tangent_setpoint,
		const matrix::Vector2f &ground_vel, const matrix::Vector2f &wind_vel, const float &curvature)
{
	const Vector2f unit_path_tangent{tangent_setpoint.normalized()};
	_target_bearing = atan2f(unit_path_tangent(1), unit_path_tangent(0));
	_closest_point_on_path = position_setpoint;
	_npfg.guideToPath(vehicle_pos, ground_vel, wind_vel, tangent_setpoint.normalized(), position_setpoint, curvature);
} // navigatePathTangent

void FixedwingPositionControl::navigateBearing(const matrix::Vector2f &vehicle_pos, float bearing,
		const Vector2f &ground_vel, const Vector2f &wind_vel)
{

	const Vector2f unit_path_tangent = Vector2f{cosf(bearing), sinf(bearing)};
	_target_bearing = atan2f(unit_path_tangent(1), unit_path_tangent(0));
	_closest_point_on_path = vehicle_pos;
	_npfg.guideToPath(vehicle_pos, ground_vel, wind_vel, unit_path_tangent, vehicle_pos, 0.0f);
} // navigateBearing

int FixedwingPositionControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	FixedwingPositionControl *instance = new FixedwingPositionControl(vtol);

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

int FixedwingPositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FixedwingPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_pos_control is the fixed-wing position controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_pos_control_main(int argc, char *argv[])
{
	return FixedwingPositionControl::main(argc, argv);
}
