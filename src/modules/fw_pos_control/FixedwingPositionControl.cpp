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
#ifdef CONFIG_FIGURE_OF_EIGHT
	, _figure_eight(_npfg, _wind_vel, _eas2tas)
#endif // CONFIG_FIGURE_OF_EIGHT
{

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

	_roll_slew_rate.setSlewRate(radians(_param_fw_pn_r_slew_max.get()));
	_roll_slew_rate.setForcedValue(0.f);

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

void
FixedwingPositionControl::parameters_update()
{
	updateParams();

	_performance_model.updateParameters();

	_roll_slew_rate.setSlewRate(radians(_param_fw_pn_r_slew_max.get()));

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
	_npfg.setPeriodSafetyFactor(_param_npfg_period_safety_factor.get());

	// TECS parameters
	_tecs.set_max_climb_rate(_performance_model.getMaximumClimbRate(_air_density));
	_tecs.set_max_sink_rate(_param_fw_t_sink_max.get());
	_tecs.set_min_sink_rate(_performance_model.getMinimumSinkRate(_air_density));
	_tecs.set_speed_weight(_param_fw_t_spdweight.get());
	_tecs.set_equivalent_airspeed_trim(_performance_model.getCalibratedTrimAirspeed());
	_tecs.set_equivalent_airspeed_min(_performance_model.getMinimumCalibratedAirspeed());
	_tecs.set_equivalent_airspeed_max(_performance_model.getMaximumCalibratedAirspeed());
	_tecs.set_throttle_damp(_param_fw_t_thr_damping.get());
	_tecs.set_integrator_gain_throttle(_param_fw_t_thr_integ.get());
	_tecs.set_integrator_gain_pitch(_param_fw_t_I_gain_pit.get());
	_tecs.set_throttle_slewrate(_param_fw_thr_slew_max.get());
	_tecs.set_vertical_accel_limit(_param_fw_t_vert_acc.get());
	_tecs.set_roll_throttle_compensation(_param_fw_t_rll2thr.get());
	_tecs.set_pitch_damping(_param_fw_t_ptch_damp.get());
	_tecs.set_altitude_error_time_constant(_param_fw_t_h_error_tc.get());
	_tecs.set_fast_descend_altitude_error(_param_fw_t_fast_alt_err.get());
	_tecs.set_altitude_rate_ff(_param_fw_t_hrate_ff.get());
	_tecs.set_airspeed_error_time_constant(_param_fw_t_tas_error_tc.get());
	_tecs.set_ste_rate_time_const(_param_ste_rate_time_const.get());
	_tecs.set_seb_rate_ff_gain(_param_seb_rate_ff.get());
	_tecs.set_airspeed_measurement_std_dev(_param_speed_standard_dev.get());
	_tecs.set_airspeed_rate_measurement_std_dev(_param_speed_rate_standard_dev.get());
	_tecs.set_airspeed_filter_process_std_dev(_param_process_noise_standard_dev.get());

	_performance_model.runSanityChecks();
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

	if (_param_fw_use_airspd.get() && _airspeed_validated_sub.update(&airspeed_validated)) {

		_eas2tas = 1.0f; //this is the default value, taken in case of invalid airspeed

		if (PX4_ISFINITE(airspeed_validated.calibrated_airspeed_m_s)
		    && PX4_ISFINITE(airspeed_validated.true_airspeed_m_s)
		    && (airspeed_validated.calibrated_airspeed_m_s > FLT_EPSILON)) {

			airspeed_valid = true;

			_time_airspeed_last_valid = airspeed_validated.timestamp;
			_airspeed_eas = airspeed_validated.calibrated_airspeed_m_s;

			_eas2tas = constrain(airspeed_validated.true_airspeed_m_s / airspeed_validated.calibrated_airspeed_m_s, 0.9f, 2.0f);

		} else {
			airspeed_valid = false;
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
		_tecs.set_load_factor(getLoadFactor());
	}
}

float
FixedwingPositionControl::get_manual_airspeed_setpoint()
{

	float altctrl_airspeed = _performance_model.getCalibratedTrimAirspeed();

	if (_param_fw_pos_stk_conf.get() & STICK_CONFIG_ENABLE_AIRSPEED_SP_MANUAL_BIT) {
		// neutral throttle corresponds to trim airspeed
		return math::interpolateNXY(_manual_control_setpoint_for_airspeed,
		{-1.f, 0.f, 1.f},
		{_performance_model.getMinimumCalibratedAirspeed(getLoadFactor()), altctrl_airspeed, _performance_model.getMaximumCalibratedAirspeed()});

	} else if (PX4_ISFINITE(_commanded_manual_airspeed_setpoint)) {
		altctrl_airspeed = constrain(_commanded_manual_airspeed_setpoint,
					     _performance_model.getMinimumCalibratedAirspeed(getLoadFactor()),
					     _performance_model.getMaximumCalibratedAirspeed());
	}

	return altctrl_airspeed;
}

float
FixedwingPositionControl::adapt_airspeed_setpoint(const float control_interval, float calibrated_airspeed_setpoint,
		float calibrated_min_airspeed, const Vector2f &ground_speed, bool in_takeoff_situation)
{
	// --- airspeed *constraint adjustments ---

	// Aditional option to increase the min airspeed setpoint based on wind estimate for more stability in higher winds.
	if (!in_takeoff_situation && _airspeed_valid && _wind_valid && _param_fw_wind_arsp_sc.get() > FLT_EPSILON) {
		calibrated_min_airspeed = math::min(calibrated_min_airspeed + _param_fw_wind_arsp_sc.get() *
						    _wind_vel.length(), _performance_model.getMaximumCalibratedAirspeed());
	}

	// --- airspeed *setpoint adjustments ---

	if (!PX4_ISFINITE(calibrated_airspeed_setpoint) || calibrated_airspeed_setpoint <= FLT_EPSILON) {
		calibrated_airspeed_setpoint = _performance_model.getCalibratedTrimAirspeed();
	}

	// Adapt cruise airspeed when otherwise the min groundspeed couldn't be maintained
	if (!_wind_valid && !in_takeoff_situation) {
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

	calibrated_airspeed_setpoint = constrain(calibrated_airspeed_setpoint, calibrated_min_airspeed,
				       _performance_model.getMaximumCalibratedAirspeed());

	// initialize the airspeed setpoint to the max(current airsped, min airspeed)
	if (!PX4_ISFINITE(_airspeed_slew_rate_controller.getState())
	    || !_tecs_is_running) {
		_airspeed_slew_rate_controller.setForcedValue(math::max(calibrated_min_airspeed, _airspeed_eas));
	}

	// reset the airspeed setpoint to the min airspeed if the min airspeed changes while in operation
	if (_airspeed_slew_rate_controller.getState() < calibrated_min_airspeed) {
		_airspeed_slew_rate_controller.setForcedValue(calibrated_min_airspeed);
	}

	if (control_interval > FLT_EPSILON) {
		// constrain airspeed setpoint changes with slew rate of ASPD_SP_SLEW_RATE m/s/s
		_airspeed_slew_rate_controller.update(calibrated_airspeed_setpoint, control_interval);
	}

	if (_airspeed_slew_rate_controller.getState() > _performance_model.getMaximumCalibratedAirspeed()) {
		_airspeed_slew_rate_controller.setForcedValue(_performance_model.getMaximumCalibratedAirspeed());
	}

	return _airspeed_slew_rate_controller.getState();
}

void
FixedwingPositionControl::tecs_status_publish(float alt_sp, float equivalent_airspeed_sp,
		float true_airspeed_derivative_raw, float throttle_trim)
{
	tecs_status_s tecs_status{};

	const TECS::DebugOutput &debug_output{_tecs.getStatus()};

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
	tecs_status.underspeed_ratio = _tecs.get_underspeed_ratio();

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
	npfg_status.can_run_factor = _npfg.canRun(_local_pos, _wind_valid);
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

float FixedwingPositionControl::getCorrectedNpfgRollSetpoint()
{
	// Scale the npfg output to zero if npfg is not certain for correct output
	float new_roll_setpoint(_npfg.getRollSetpoint());
	const float can_run_factor(constrain(_npfg.canRun(_local_pos, _wind_valid), 0.f, 1.f));

	hrt_abstime now{hrt_absolute_time()};

	// Warn the user when the scale is less than 90% for at least 2 seconds (disable in transition)

	// If the npfg was not running before, reset the user warning variables.
	if ((now - _time_since_last_npfg_call) > ROLL_WARNING_TIMEOUT) {
		_need_report_npfg_uncertain_condition = true;
		_time_since_first_reduced_roll = 0U;
	}

	if (_vehicle_status.in_transition_mode || can_run_factor > ROLL_WARNING_CAN_RUN_THRESHOLD || _landed) {
		// NPFG reports a good condition or we are in transition, reset the user warning variables.
		_need_report_npfg_uncertain_condition = true;
		_time_since_first_reduced_roll = 0U;

	} else if (_need_report_npfg_uncertain_condition) {
		if (_time_since_first_reduced_roll == 0U) {
			_time_since_first_reduced_roll = now;
		}

		if ((now - _time_since_first_reduced_roll) > ROLL_WARNING_TIMEOUT) {
			_need_report_npfg_uncertain_condition = false;
			events::send(events::ID("npfg_roll_command_uncertain"), events::Log::Warning,
				     "Roll command reduced due to uncertain velocity/wind estimates!");
		}

	} else {
		// Nothing to do, already reported.
	}

	_time_since_last_npfg_call = now;

	return can_run_factor * (new_roll_setpoint);
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
		const bool at_controllable_airspeed = _airspeed_eas > _performance_model.getMinimumCalibratedAirspeed(getLoadFactor())
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

	if (_control_mode.flag_control_offboard_enabled && _position_setpoint_current_valid
	    && _control_mode.flag_control_position_enabled) {
		if (PX4_ISFINITE(_pos_sp_triplet.current.vx) && PX4_ISFINITE(_pos_sp_triplet.current.vy)
		    && PX4_ISFINITE(_pos_sp_triplet.current.vz)) {
			// Offboard position with velocity setpoints
			_control_mode_current = FW_POSCTRL_MODE_AUTO_PATH;
			return;

		} else {
			// Offboard position setpoint only
			_control_mode_current = FW_POSCTRL_MODE_AUTO;
			return;
		}

	} else if ((_control_mode.flag_control_auto_enabled && _control_mode.flag_control_position_enabled)
		   && (_position_setpoint_current_valid
		       || _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE)) {

		// Enter this mode only if the current waypoint has valid 3D position setpoints or is of type IDLE.
		// A setpoint of type IDLE can be published by Navigator without a valid position, and is handled here in FW_POSCTRL_MODE_AUTO.
		const bool doing_backtransition = _vehicle_status.in_transition_mode && !_vehicle_status.in_transition_to_fw;

		if (doing_backtransition) {
			_control_mode_current = FW_POSCTRL_MODE_TRANSITON;

		} else if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {

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

			// Use _position_setpoint_previous_valid to determine if landing should be straight or circular.
			// Straight landings are currently only possible in Missions, and there the previous WP
			// is valid, and circular ones are used outside of Missions, as the land mode sets prev_valid=false.
			if (_position_setpoint_previous_valid) {
				_control_mode_current = FW_POSCTRL_MODE_AUTO_LANDING_STRAIGHT;

			} else {
				_control_mode_current = FW_POSCTRL_MODE_AUTO_LANDING_CIRCULAR;
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
	/* reset flag when airplane landed */
	if (_landed) {
		_completed_manual_takeoff = false;
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

	const uint8_t position_sp_type = handle_setpoint_type(current_sp, pos_sp_next);

	_position_sp_type = position_sp_type;

	if (position_sp_type == position_setpoint_s::SETPOINT_TYPE_LOITER
	    || current_sp.type == position_setpoint_s::SETPOINT_TYPE_LOITER) {
#ifdef CONFIG_FIGURE_OF_EIGHT

		if (current_sp.loiter_pattern == position_setpoint_s::LOITER_TYPE_FIGUREEIGHT) {
			publishFigureEightStatus(current_sp);

		} else
#endif // CONFIG_FIGURE_OF_EIGHT
		{
			publishOrbitStatus(current_sp);
		}
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
#ifdef CONFIG_FIGURE_OF_EIGHT
		if (current_sp.loiter_pattern == position_setpoint_s::LOITER_TYPE_FIGUREEIGHT) {
			controlAutoFigureEight(control_interval, curr_pos, ground_speed, pos_sp_prev, current_sp);

		} else
#endif // CONFIG_FIGURE_OF_EIGHT
		{
			control_auto_loiter(control_interval, curr_pos, ground_speed, pos_sp_prev, current_sp, pos_sp_next);

		}

		break;
	}

#ifdef CONFIG_FIGURE_OF_EIGHT

	/* reset loiter state */
	if ((position_sp_type != position_setpoint_s::SETPOINT_TYPE_LOITER) ||
	    ((position_sp_type == position_setpoint_s::SETPOINT_TYPE_LOITER) &&
	     (current_sp.loiter_pattern != position_setpoint_s::LOITER_TYPE_FIGUREEIGHT))) {
		_figure_eight.resetPattern();
	}

#endif // CONFIG_FIGURE_OF_EIGHT

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
				   _performance_model.getCalibratedTrimAirspeed(),
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
				   _performance_model.getCalibratedTrimAirspeed(),
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
FixedwingPositionControl::handle_setpoint_type(const position_setpoint_s &pos_sp_curr,
		const position_setpoint_s &pos_sp_next)
{
	uint8_t position_sp_type = pos_sp_curr.type;

	if (!_control_mode.flag_control_position_enabled && _control_mode.flag_control_velocity_enabled) {
		return position_setpoint_s::SETPOINT_TYPE_VELOCITY;
	}

	Vector2d curr_wp{0, 0};

	/* current waypoint (the one currently heading for) */
	curr_wp = Vector2d(pos_sp_curr.lat, pos_sp_curr.lon);

	const float acc_rad = _npfg.switchDistance(500.0f);

	const bool approaching_vtol_backtransition = _vehicle_status.is_vtol
			&& pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_POSITION && _position_setpoint_current_valid
			&& pos_sp_next.type == position_setpoint_s::SETPOINT_TYPE_LAND && _position_setpoint_next_valid;


	// check if we should switch to loiter but only if we are not expecting a backtransition to happen
	if (pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_POSITION && !approaching_vtol_backtransition) {

		float dist_xy = -1.f;
		float dist_z = -1.f;

		const float dist = get_distance_to_point_global_wgs84(
					   (double)curr_wp(0), (double)curr_wp(1), pos_sp_curr.alt,
					   _current_latitude, _current_longitude, _current_altitude,
					   &dist_xy, &dist_z);

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

	return position_sp_type;
}

void
FixedwingPositionControl::control_auto_position(const float control_interval, const Vector2d &curr_pos,
		const Vector2f &ground_speed, const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr)
{
	const float acc_rad = _npfg.switchDistance(500.0f);
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
		const float d_curr_prev = get_distance_to_next_waypoint(pos_sp_curr.lat, pos_sp_curr.lon, pos_sp_prev.lat,
					  pos_sp_prev.lon);

		// Do not try to find a solution if the last waypoint is inside the acceptance radius of the current one
		if (d_curr_prev > math::max(acc_rad, fabsf(pos_sp_curr.loiter_radius))) {
			// Calculate distance to current waypoint
			const float d_curr = get_distance_to_next_waypoint(pos_sp_curr.lat, pos_sp_curr.lon, _current_latitude,
					     _current_longitude);

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
				_performance_model.getMinimumCalibratedAirspeed(getLoadFactor()), ground_speed);

	Vector2f curr_pos_local{_local_pos.x, _local_pos.y};
	Vector2f curr_wp_local = _global_local_proj_ref.project(pos_sp_curr.lat, pos_sp_curr.lon);

	_npfg.setAirspeedNom(target_airspeed * _eas2tas);
	_npfg.setAirspeedMax(_performance_model.getMaximumCalibratedAirspeed() * _eas2tas);

	if (_position_setpoint_previous_valid && pos_sp_prev.type != position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {
		Vector2f prev_wp_local = _global_local_proj_ref.project(pos_sp_prev.lat, pos_sp_prev.lon);
		navigateWaypoints(prev_wp_local, curr_wp_local, curr_pos_local, ground_speed, _wind_vel);

	} else {
		navigateWaypoint(curr_wp_local, curr_pos_local, ground_speed, _wind_vel);
	}

	_att_sp.roll_body = getCorrectedNpfgRollSetpoint();
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
				_performance_model.getMinimumCalibratedAirspeed(getLoadFactor()), ground_speed);

	Vector2f curr_pos_local{_local_pos.x, _local_pos.y};
	_npfg.setAirspeedNom(target_airspeed * _eas2tas);
	_npfg.setAirspeedMax(_performance_model.getMaximumCalibratedAirspeed() * _eas2tas);
	navigateBearing(curr_pos_local, _target_bearing, ground_speed, _wind_vel);
	_att_sp.roll_body = getCorrectedNpfgRollSetpoint();
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
				   false,
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
		airspeed_sp = (_param_fw_lnd_airspd.get() > FLT_EPSILON) ? _param_fw_lnd_airspd.get() :
			      _performance_model.getMinimumCalibratedAirspeed(getLoadFactor());
		_flaps_setpoint = _param_fw_flaps_lnd_scl.get();
		_spoilers_setpoint = _param_fw_spoilers_lnd.get();
		_new_landing_gear_position = landing_gear_s::GEAR_DOWN;
	}

	float target_airspeed = adapt_airspeed_setpoint(control_interval, airspeed_sp,
				_performance_model.getMinimumCalibratedAirspeed(getLoadFactor()),
				ground_speed);

	_npfg.setAirspeedNom(target_airspeed * _eas2tas);
	_npfg.setAirspeedMax(_performance_model.getMaximumCalibratedAirspeed() * _eas2tas);
	navigateLoiter(curr_wp_local, curr_pos_local, loiter_radius, pos_sp_curr.loiter_direction_counter_clockwise,
		       ground_speed,
		       _wind_vel);
	_att_sp.roll_body = getCorrectedNpfgRollSetpoint();
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

#ifdef CONFIG_FIGURE_OF_EIGHT
void
FixedwingPositionControl::controlAutoFigureEight(const float control_interval, const Vector2d &curr_pos,
		const Vector2f &ground_speed, const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr)
{
	// airspeed settings
	float target_airspeed = adapt_airspeed_setpoint(control_interval, pos_sp_curr.cruising_speed,
				_performance_model.getMinimumCalibratedAirspeed(), ground_speed);

	// Lateral Control

	Vector2f curr_pos_local{_local_pos.x, _local_pos.y};

	FigureEight::FigureEightPatternParameters params;
	params.center_pos_local = _global_local_proj_ref.project(pos_sp_curr.lat, pos_sp_curr.lon);
	params.loiter_direction_counter_clockwise = pos_sp_curr.loiter_direction_counter_clockwise;
	params.loiter_minor_radius = pos_sp_curr.loiter_minor_radius;
	params.loiter_orientation = pos_sp_curr.loiter_orientation;
	params.loiter_radius = pos_sp_curr.loiter_radius;

	// Apply control
	_figure_eight.updateSetpoint(curr_pos_local, ground_speed, params, target_airspeed);
	_att_sp.roll_body = _figure_eight.getRollSetpoint();
	target_airspeed = _figure_eight.getAirspeedSetpoint();
	_target_bearing = _figure_eight.getTargetBearing();
	_closest_point_on_path = _figure_eight.getClosestPoint();

	// TECS
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

	tecs_update_pitch_throttle(control_interval,
				   pos_sp_curr.alt,
				   target_airspeed,
				   radians(_param_fw_p_lim_min.get()),
				   radians(_param_fw_p_lim_max.get()),
				   tecs_fw_thr_min,
				   tecs_fw_thr_max,
				   _param_sinkrate_target.get(),
				   _param_climbrate_target.get());

	// Yaw
	_att_sp.yaw_body = _yaw; // yaw is not controlled, so set setpoint to current yaw
}

void FixedwingPositionControl::publishFigureEightStatus(const position_setpoint_s pos_sp)
{
	figure_eight_status_s figure_eight_status{};
	figure_eight_status.timestamp = hrt_absolute_time();
	figure_eight_status.major_radius = pos_sp.loiter_radius * (pos_sp.loiter_direction_counter_clockwise ? -1.f : 1.f);
	figure_eight_status.minor_radius = pos_sp.loiter_minor_radius;
	figure_eight_status.orientation = pos_sp.loiter_orientation;
	figure_eight_status.frame = 5; //MAV_FRAME_GLOBAL_INT
	figure_eight_status.x = static_cast<int32_t>(pos_sp.lat * 1e7);
	figure_eight_status.y = static_cast<int32_t>(pos_sp.lon * 1e7);
	figure_eight_status.z = pos_sp.alt;

	_figure_eight_status_pub.publish(figure_eight_status);
}
#endif // CONFIG_FIGURE_OF_EIGHT

void
FixedwingPositionControl::control_auto_path(const float control_interval, const Vector2d &curr_pos,
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
	float target_airspeed = adapt_airspeed_setpoint(control_interval, pos_sp_curr.cruising_speed,
				_performance_model.getMinimumCalibratedAirspeed(getLoadFactor()), ground_speed);

	Vector2f curr_pos_local{_local_pos.x, _local_pos.y};
	Vector2f curr_wp_local = _global_local_proj_ref.project(pos_sp_curr.lat, pos_sp_curr.lon);

	_npfg.setAirspeedNom(target_airspeed * _eas2tas);
	_npfg.setAirspeedMax(_performance_model.getMaximumCalibratedAirspeed() * _eas2tas);

	// Navigate directly on position setpoint and path tangent
	matrix::Vector2f velocity_2d(pos_sp_curr.vx, pos_sp_curr.vy);
	const float curvature = PX4_ISFINITE(_pos_sp_triplet.current.loiter_radius) ? 1 /
				_pos_sp_triplet.current.loiter_radius :
				0.0f;
	navigatePathTangent(curr_pos_local, curr_wp_local, velocity_2d.normalized(), ground_speed, _wind_vel, curvature);

	_att_sp.roll_body = getCorrectedNpfgRollSetpoint();
	target_airspeed = _npfg.getAirspeedRef() / _eas2tas;

	_att_sp.yaw_body = _yaw; // yaw is not controlled, so set setpoint to current yaw

	tecs_update_pitch_throttle(control_interval,
				   pos_sp_curr.alt,
				   target_airspeed,
				   radians(_param_fw_p_lim_min.get()),
				   radians(_param_fw_p_lim_max.get()),
				   tecs_fw_thr_min,
				   tecs_fw_thr_max,
				   _param_sinkrate_target.get(),
				   _param_climbrate_target.get());

	_att_sp.thrust_body[0] = min(get_tecs_thrust(), tecs_fw_thr_max);
	_att_sp.pitch_body = get_tecs_pitch();
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
				       _performance_model.getMinimumCalibratedAirspeed(getLoadFactor());

	float adjusted_min_airspeed = _performance_model.getMinimumCalibratedAirspeed(getLoadFactor());

	if (takeoff_airspeed < adjusted_min_airspeed) {
		// adjust underspeed detection bounds for takeoff airspeed
		_tecs.set_equivalent_airspeed_min(takeoff_airspeed);
		adjusted_min_airspeed = takeoff_airspeed;
	}

	if (_runway_takeoff.runwayTakeoffEnabled()) {
		if (!_runway_takeoff.isInitialized()) {
			_runway_takeoff.init(now, _yaw, global_position);
			_takeoff_ground_alt = _current_altitude;
			_launch_current_yaw = _yaw;
			_airspeed_slew_rate_controller.setForcedValue(takeoff_airspeed);

			events::send(events::ID("fixedwing_position_control_takeoff"), events::Log::Info, "Takeoff on runway");
		}

		if (_skipping_takeoff_detection) {
			_runway_takeoff.forceSetFlyState();
		}

		_runway_takeoff.update(now, takeoff_airspeed, _airspeed_eas, _current_altitude - _takeoff_ground_alt,
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
		float takeoff_bearing = _launch_current_yaw;

		if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {
			// the bearing from runway start to the takeoff waypoint is followed until the clearance altitude is exceeded
			const Vector2f takeoff_bearing_vector = takeoff_waypoint_local - start_pos_local;

			if (takeoff_bearing_vector.norm() > FLT_EPSILON) {
				takeoff_bearing = atan2f(takeoff_bearing_vector(1), takeoff_bearing_vector(0));
			}
		}

		float target_airspeed = adapt_airspeed_setpoint(control_interval, takeoff_airspeed, adjusted_min_airspeed, ground_speed,
					true);

		_npfg.setAirspeedNom(target_airspeed * _eas2tas);
		_npfg.setAirspeedMax(_performance_model.getMaximumCalibratedAirspeed() * _eas2tas);
		navigateLine(start_pos_local, takeoff_bearing, local_2D_position, ground_speed, _wind_vel);

		_att_sp.roll_body = _runway_takeoff.getRoll(getCorrectedNpfgRollSetpoint());

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
					   _performance_model.getMaximumClimbRate(_air_density));

		_tecs.set_equivalent_airspeed_min(_performance_model.getMinimumCalibratedAirspeed()); // reset after TECS calculation

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

		if (!_launch_detected && _launchDetector.getLaunchDetected() > launch_detection_status_s::STATE_WAITING_FOR_LAUNCH) {
			_launch_detected = true;
			_launch_global_position = global_position;
			_takeoff_ground_alt = _current_altitude;
			_launch_current_yaw = _yaw;
			_airspeed_slew_rate_controller.setForcedValue(takeoff_airspeed);
		}

		const Vector2f launch_local_position = _global_local_proj_ref.project(_launch_global_position(0),
						       _launch_global_position(1));
		const Vector2f takeoff_waypoint_local = _global_local_proj_ref.project(pos_sp_curr.lat, pos_sp_curr.lon);

		// by default set the takeoff bearing to the takeoff yaw, but override in a mission takeoff with bearing to takeoff WP
		float takeoff_bearing = _launch_current_yaw;

		if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {
			// the bearing from launch to the takeoff waypoint is followed until the clearance altitude is exceeded
			const Vector2f takeoff_bearing_vector = takeoff_waypoint_local - launch_local_position;

			if (takeoff_bearing_vector.norm() > FLT_EPSILON) {
				takeoff_bearing = atan2f(takeoff_bearing_vector(1), takeoff_bearing_vector(0));
			}
		}

		/* Set control values depending on the detection state */
		if (_launchDetector.getLaunchDetected() > launch_detection_status_s::STATE_WAITING_FOR_LAUNCH) {
			/* Launch has been detected, hence we have to control the plane. */

			float target_airspeed = adapt_airspeed_setpoint(control_interval, takeoff_airspeed, adjusted_min_airspeed, ground_speed,
						true);

			_npfg.setAirspeedNom(target_airspeed * _eas2tas);
			_npfg.setAirspeedMax(_performance_model.getMaximumCalibratedAirspeed() * _eas2tas);
			navigateLine(launch_local_position, takeoff_bearing, local_2D_position, ground_speed, _wind_vel);
			_att_sp.roll_body = getCorrectedNpfgRollSetpoint();
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
						   _performance_model.getMaximumClimbRate(_air_density));

			if (_launchDetector.getLaunchDetected() < launch_detection_status_s::STATE_FLYING) {
				// explicitly set idle throttle until motors are enabled
				_att_sp.thrust_body[0] = _param_fw_thr_idle.get();

			} else {
				_att_sp.thrust_body[0] = get_tecs_thrust();
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
				    _performance_model.getMinimumCalibratedAirspeed(getLoadFactor());
	float adjusted_min_airspeed = _performance_model.getMinimumCalibratedAirspeed(getLoadFactor());

	if (airspeed_land < adjusted_min_airspeed) {
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
		_npfg.setAirspeedMax(_performance_model.getMaximumCalibratedAirspeed() * _eas2tas);
		navigateLine(local_approach_entrance, local_land_point, local_position, ground_speed, _wind_vel);
		target_airspeed = _npfg.getAirspeedRef() / _eas2tas;
		_att_sp.roll_body = getCorrectedNpfgRollSetpoint();

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
		_npfg.setAirspeedMax(_performance_model.getMaximumCalibratedAirspeed() * _eas2tas);
		navigateLine(local_approach_entrance, local_land_point, local_position, ground_speed, _wind_vel);
		target_airspeed = _npfg.getAirspeedRef() / _eas2tas;
		_att_sp.roll_body = getCorrectedNpfgRollSetpoint();

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

	_tecs.set_equivalent_airspeed_min(_performance_model.getMinimumCalibratedAirspeed()); // reset after TECS calculation

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
				    _performance_model.getMinimumCalibratedAirspeed(getLoadFactor());
	float adjusted_min_airspeed = _performance_model.getMinimumCalibratedAirspeed(getLoadFactor());

	if (airspeed_land < adjusted_min_airspeed) {
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
		_npfg.setAirspeedMax(_performance_model.getMaximumCalibratedAirspeed() * _eas2tas);

		navigateLoiter(local_landing_orbit_center, local_position, loiter_radius,
			       pos_sp_curr.loiter_direction_counter_clockwise,
			       ground_speed, _wind_vel);
		target_airspeed = _npfg.getAirspeedRef() / _eas2tas;
		_att_sp.roll_body = getCorrectedNpfgRollSetpoint();

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
		_npfg.setAirspeedMax(_performance_model.getMaximumCalibratedAirspeed() * _eas2tas);

		navigateLoiter(local_landing_orbit_center, local_position, loiter_radius,
			       pos_sp_curr.loiter_direction_counter_clockwise,
			       ground_speed, _wind_vel);
		target_airspeed = _npfg.getAirspeedRef() / _eas2tas;
		_att_sp.roll_body = getCorrectedNpfgRollSetpoint();

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

	_tecs.set_equivalent_airspeed_min(_performance_model.getMinimumCalibratedAirspeed()); // reset after TECS calculation

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
					     _performance_model.getMinimumCalibratedAirspeed(getLoadFactor()), ground_speed, !_completed_manual_takeoff);
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
				       _performance_model.getMinimumCalibratedAirspeed(getLoadFactor()), ground_speed, !_completed_manual_takeoff);
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
	// TODO: either make it course hold (easier) or a real heading hold (minus all the complexity here)
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

				_hdg_hold_position.lat = _current_latitude;
				_hdg_hold_position.lon = _current_longitude;
			}

			Vector2f curr_pos_local{_local_pos.x, _local_pos.y};
			Vector2f curr_wp_local = _global_local_proj_ref.project(_hdg_hold_position.lat, _hdg_hold_position.lon);

			_npfg.setAirspeedNom(calibrated_airspeed_sp * _eas2tas);
			_npfg.setAirspeedMax(_performance_model.getMaximumCalibratedAirspeed() * _eas2tas);
			navigateLine(curr_wp_local, _hdg_hold_yaw, curr_pos_local, ground_speed, _wind_vel);
			_att_sp.roll_body = getCorrectedNpfgRollSetpoint();
			calibrated_airspeed_sp = _npfg.getAirspeedRef() / _eas2tas;

			_att_sp.yaw_body = _yaw; // yaw is not controlled, so set setpoint to current yaw
		}
	}

	tecs_update_pitch_throttle(control_interval,
				   _current_altitude, // TODO: check if this is really what we want.. or if we want to lock the altitude.
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

		_att_sp.roll_body = _manual_control_setpoint.roll * radians(_param_fw_r_lim.get());
		_att_sp.yaw_body = _yaw; // yaw is not controlled, so set setpoint to current yaw
	}

	_att_sp.thrust_body[0] = min(get_tecs_thrust(), throttle_max);
	_att_sp.pitch_body = get_tecs_pitch();
}

void FixedwingPositionControl::control_backtransition(const float control_interval, const Vector2f &ground_speed,
		const position_setpoint_s &pos_sp_curr)
{
	float target_airspeed = adapt_airspeed_setpoint(control_interval, pos_sp_curr.cruising_speed,
				_performance_model.getMinimumCalibratedAirspeed(getLoadFactor()), ground_speed);

	Vector2f curr_pos_local{_local_pos.x, _local_pos.y};
	Vector2f curr_wp_local = _global_local_proj_ref.project(pos_sp_curr.lat, pos_sp_curr.lon);

	_npfg.setAirspeedNom(target_airspeed * _eas2tas);
	_npfg.setAirspeedMax(_performance_model.getMaximumCalibratedAirspeed() * _eas2tas);

	// Set the position where the backtransition started the first ime we pass through here.
	// Will get reset if not in transition anymore.
	if (!_lpos_where_backtrans_started.isAllFinite()) {
		_lpos_where_backtrans_started = curr_pos_local;
	}

	navigateLine(_lpos_where_backtrans_started, curr_wp_local, curr_pos_local, ground_speed, _wind_vel);

	_att_sp.roll_body = getCorrectedNpfgRollSetpoint();
	target_airspeed = _npfg.getAirspeedRef() / _eas2tas;

	_att_sp.yaw_body = _yaw; // yaw is not controlled, so set setpoint to current yaw

	tecs_update_pitch_throttle(control_interval,
				   pos_sp_curr.alt,
				   target_airspeed,
				   radians(_param_fw_p_lim_min.get()),
				   radians(_param_fw_p_lim_max.get()),
				   _param_fw_thr_min.get(),
				   _param_fw_thr_max.get(),
				   _param_sinkrate_target.get(),
				   _param_climbrate_target.get());

	_att_sp.thrust_body[0] = (_landed) ? _param_fw_thr_min.get() : min(get_tecs_thrust(), _param_fw_thr_max.get());
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

		if (_local_pos.z_global && PX4_ISFINITE(_local_pos.ref_alt)) {
			_reference_altitude = _local_pos.ref_alt;

		} else {
			_reference_altitude = 0.f;
		}

		_current_altitude = -_local_pos.z + _reference_altitude; // Altitude AMSL in meters

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

			double reference_latitude = 0.;
			double reference_longitude = 0.;

			if (_local_pos.xy_global && PX4_ISFINITE(_local_pos.ref_lat) && PX4_ISFINITE(_local_pos.ref_lon)) {
				reference_latitude = _local_pos.ref_lat;
				reference_longitude = _local_pos.ref_lon;
			}

			_global_local_proj_ref.initReference(reference_latitude, reference_longitude,
							     _local_pos.ref_timestamp);
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
						_pos_sp_triplet.current.alt = _reference_altitude - trajectory_setpoint.position[2];
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
			_tecs.set_max_climb_rate(_performance_model.getMaximumClimbRate(_air_density));
			_tecs.set_min_sink_rate(_performance_model.getMinimumSinkRate(_air_density));
		}

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.update(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
			}
		}

		if (_vehicle_status_sub.update(&_vehicle_status)) {
			if (!_vehicle_status.in_transition_mode) {
				// reset position of backtransition start if not in transition
				_lpos_where_backtrans_started = Vector2f(NAN, NAN);
			}
		}

		Vector2d curr_pos(_current_latitude, _current_longitude);
		Vector2f ground_speed(_local_pos.vx, _local_pos.vy);

		set_control_mode_current(_local_pos.timestamp);

		update_in_air_states(_local_pos.timestamp);

		// restore nominal TECS parameters in case changed intermittently (e.g. in landing handling)
		_tecs.set_speed_weight(_param_fw_t_spdweight.get());
		_tecs.set_altitude_error_time_constant(_param_fw_t_h_error_tc.get());

		// restore lateral-directional guidance parameters (changed in takeoff mode)
		_npfg.setPeriod(_param_npfg_period.get());

		_att_sp.reset_integral = false;

		// by default no flaps/spoilers, is overwritten below in certain modes
		_flaps_setpoint = 0.f;
		_spoilers_setpoint = 0.f;

		// reset flight phase estimate
		_flight_phase_estimation_pub.get().flight_phase = flight_phase_estimation_s::FLIGHT_PHASE_UNKNOWN;

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

		case FW_POSCTRL_MODE_AUTO_PATH: {
				control_auto_path(control_interval, curr_pos, ground_speed, _pos_sp_triplet.current);
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
				break;
			}

		case FW_POSCTRL_MODE_TRANSITON: {
				control_backtransition(control_interval, ground_speed, _pos_sp_triplet.current);
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

				// roll slew rate
				_att_sp.roll_body = _roll_slew_rate.update(_att_sp.roll_body, control_interval);

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

		} else {
			_roll_slew_rate.setForcedValue(_roll);
		}

		// Publish estimate of level flight
		_flight_phase_estimation_pub.get().timestamp = hrt_absolute_time();
		_flight_phase_estimation_pub.update();

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

void
FixedwingPositionControl::tecs_update_pitch_throttle(const float control_interval, float alt_sp, float airspeed_sp,
		float pitch_min_rad, float pitch_max_rad, float throttle_min, float throttle_max,
		const float desired_max_sinkrate, const float desired_max_climbrate,
		bool disable_underspeed_detection, float hgt_rate_sp)
{
	// do not run TECS if vehicle is a VTOL and we are in rotary wing mode or in transition
	if (_vehicle_status.is_vtol && (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
					|| _vehicle_status.in_transition_mode)) {
		_tecs_is_running = false;
		return;

	} else {
		_tecs_is_running = true;
	}

	/* update TECS vehicle state estimates */
	const float throttle_trim_compensated = _performance_model.getTrimThrottle(throttle_min,
						throttle_max, airspeed_sp, _air_density);

	/* No underspeed protection in landing mode */
	_tecs.set_detect_underspeed_enabled(!disable_underspeed_detection);

	// HOTFIX: the airspeed rate estimate using acceleration in body-forward direction has shown to lead to high biases
	// when flying tight turns. It's in this case much safer to just set the estimated airspeed rate to 0.
	const float airspeed_rate_estimate = 0.f;

	_tecs.update(_pitch - radians(_param_fw_psp_off.get()),
		     _current_altitude,
		     alt_sp,
		     airspeed_sp,
		     _airspeed_eas,
		     _eas2tas,
		     throttle_min,
		     throttle_max,
		     throttle_trim_compensated,
		     pitch_min_rad - radians(_param_fw_psp_off.get()),
		     pitch_max_rad - radians(_param_fw_psp_off.get()),
		     desired_max_climbrate,
		     desired_max_sinkrate,
		     airspeed_rate_estimate,
		     -_local_pos.vz,
		     hgt_rate_sp);

	tecs_status_publish(alt_sp, airspeed_sp, airspeed_rate_estimate, throttle_trim_compensated);

	if (_tecs_is_running && !_vehicle_status.in_transition_mode
	    && (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING)) {
		const TECS::DebugOutput &tecs_output{_tecs.getStatus()};

		// Check level flight: the height rate setpoint is not set or set to 0 and we are close to the target altitude and target altitude is not moving
		if ((fabsf(tecs_output.height_rate_reference) < MAX_ALT_REF_RATE_FOR_LEVEL_FLIGHT) &&
		    fabsf(_current_altitude - tecs_output.altitude_reference) < _param_nav_fw_alt_rad.get()) {
			_flight_phase_estimation_pub.get().flight_phase = flight_phase_estimation_s::FLIGHT_PHASE_LEVEL;

		} else if (((tecs_output.altitude_reference - _current_altitude) >= _param_nav_fw_alt_rad.get()) ||
			   (tecs_output.height_rate_reference >= MAX_ALT_REF_RATE_FOR_LEVEL_FLIGHT)) {
			_flight_phase_estimation_pub.get().flight_phase = flight_phase_estimation_s::FLIGHT_PHASE_CLIMB;

		} else if (((_current_altitude - tecs_output.altitude_reference) >= _param_nav_fw_alt_rad.get()) ||
			   (tecs_output.height_rate_reference <= -MAX_ALT_REF_RATE_FOR_LEVEL_FLIGHT)) {
			_flight_phase_estimation_pub.get().flight_phase = flight_phase_estimation_s::FLIGHT_PHASE_DESCEND;

		} else {
			//We can't infer the flight phase , do nothing, estimation is reset at each step
		}
	}
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
	local_position_setpoint.z = _reference_altitude - current_waypoint.alt;
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

void FixedwingPositionControl::navigateWaypoints(const Vector2f &start_waypoint, const Vector2f &end_waypoint,
		const Vector2f &vehicle_pos, const Vector2f &ground_vel, const Vector2f &wind_vel)
{
	const Vector2f start_waypoint_to_end_waypoint = end_waypoint - start_waypoint;
	const Vector2f start_waypoint_to_vehicle = vehicle_pos - start_waypoint;
	const Vector2f end_waypoint_to_vehicle = vehicle_pos - end_waypoint;

	if (start_waypoint_to_end_waypoint.norm() < FLT_EPSILON) {
		// degenerate case: the waypoints are on top of each other, this should only happen when someone uses this
		// method incorrectly. just as a safe guard, call the singular waypoint navigation method.
		navigateWaypoint(end_waypoint, vehicle_pos, ground_vel, wind_vel);
		return;
	}

	if ((start_waypoint_to_end_waypoint.dot(start_waypoint_to_vehicle) < -FLT_EPSILON)
	    && (start_waypoint_to_vehicle.norm() > _npfg.switchDistance(500.0f))) {
		// we are in front of the start waypoint, fly directly to it until we are within switch distance
		navigateWaypoint(start_waypoint, vehicle_pos, ground_vel, wind_vel);
		return;
	}

	if (start_waypoint_to_end_waypoint.dot(end_waypoint_to_vehicle) > FLT_EPSILON) {
		// we are beyond the end waypoint, fly back to it
		// NOTE: this logic ideally never gets executed, as a waypoint switch should happen before passing the
		// end waypoint. however this included here as a safety precaution if any navigator (module) switch condition
		// is missed for any reason. in the future this logic should all be handled in one place in a dedicated
		// flight mode state machine.
		navigateWaypoint(end_waypoint, vehicle_pos, ground_vel, wind_vel);
		return;
	}

	// follow the line segment between the start and end waypoints
	navigateLine(start_waypoint, end_waypoint, vehicle_pos, ground_vel, wind_vel);
}

void FixedwingPositionControl::navigateWaypoint(const Vector2f &waypoint_pos, const Vector2f &vehicle_pos,
		const Vector2f &ground_vel, const Vector2f &wind_vel)
{
	const Vector2f vehicle_to_waypoint = waypoint_pos - vehicle_pos;

	if (vehicle_to_waypoint.norm() < FLT_EPSILON) {
		// degenerate case: the vehicle is on top of the single waypoint. (can happen). maintain the last npfg command.
		return;
	}

	const Vector2f unit_path_tangent = vehicle_to_waypoint.normalized();
	_closest_point_on_path = waypoint_pos;

	const float path_curvature = 0.f;
	_npfg.guideToPath(vehicle_pos, ground_vel, wind_vel, unit_path_tangent, _closest_point_on_path, path_curvature);

	// for logging - note we are abusing path tangent vs bearing definitions here. npfg interfaces need to be refined.
	_target_bearing = atan2f(unit_path_tangent(1), unit_path_tangent(0));
}

void FixedwingPositionControl::navigateLine(const Vector2f &point_on_line_1, const Vector2f &point_on_line_2,
		const Vector2f &vehicle_pos, const Vector2f &ground_vel, const Vector2f &wind_vel)
{
	const Vector2f line_segment = point_on_line_2 - point_on_line_1;

	if (line_segment.norm() <= FLT_EPSILON) {
		// degenerate case: line segment has zero length. maintain the last npfg command.
		return;
	}

	const Vector2f unit_path_tangent = line_segment.normalized();

	const Vector2f point_1_to_vehicle = vehicle_pos - point_on_line_1;
	_closest_point_on_path = point_on_line_1 + point_1_to_vehicle.dot(unit_path_tangent) * unit_path_tangent;

	const float path_curvature = 0.f;
	_npfg.guideToPath(vehicle_pos, ground_vel, wind_vel, unit_path_tangent, _closest_point_on_path, path_curvature);

	// for logging - note we are abusing path tangent vs bearing definitions here. npfg interfaces need to be refined.
	_target_bearing = atan2f(unit_path_tangent(1), unit_path_tangent(0));
}

void FixedwingPositionControl::navigateLine(const Vector2f &point_on_line, const float line_bearing,
		const Vector2f &vehicle_pos, const Vector2f &ground_vel, const Vector2f &wind_vel)
{
	const Vector2f unit_path_tangent{cosf(line_bearing), sinf(line_bearing)};

	const Vector2f point_on_line_to_vehicle = vehicle_pos - point_on_line;
	_closest_point_on_path = point_on_line + point_on_line_to_vehicle.dot(unit_path_tangent) * unit_path_tangent;

	const float path_curvature = 0.f;
	_npfg.guideToPath(vehicle_pos, ground_vel, wind_vel, unit_path_tangent, _closest_point_on_path, path_curvature);

	// for logging - note we are abusing path tangent vs bearing definitions here. npfg interfaces need to be refined.
	_target_bearing = line_bearing;
}

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
}

void FixedwingPositionControl::navigatePathTangent(const matrix::Vector2f &vehicle_pos,
		const matrix::Vector2f &position_setpoint,
		const matrix::Vector2f &tangent_setpoint,
		const matrix::Vector2f &ground_vel, const matrix::Vector2f &wind_vel, const float &curvature)
{
	if (tangent_setpoint.norm() <= FLT_EPSILON) {
		// degenerate case: no direction. maintain the last npfg command.
		return;
	}

	const Vector2f unit_path_tangent{tangent_setpoint.normalized()};
	_target_bearing = atan2f(unit_path_tangent(1), unit_path_tangent(0));
	_closest_point_on_path = position_setpoint;
	_npfg.guideToPath(vehicle_pos, ground_vel, wind_vel, tangent_setpoint.normalized(), position_setpoint, curvature);
}

void FixedwingPositionControl::navigateBearing(const matrix::Vector2f &vehicle_pos, float bearing,
		const Vector2f &ground_vel, const Vector2f &wind_vel)
{

	const Vector2f unit_path_tangent = Vector2f{cosf(bearing), sinf(bearing)};
	_target_bearing = atan2f(unit_path_tangent(1), unit_path_tangent(0));
	_closest_point_on_path = vehicle_pos;
	_npfg.guideToPath(vehicle_pos, ground_vel, wind_vel, unit_path_tangent, vehicle_pos, 0.0f);
}

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
float FixedwingPositionControl::getLoadFactor()
{
	float load_factor_from_bank_angle = 1.0f;

	if (PX4_ISFINITE(_att_sp.roll_body)) {
		load_factor_from_bank_angle = 1.0f / math::max(cosf(_att_sp.roll_body), FLT_EPSILON);
	}

	return load_factor_from_bank_angle;

}


extern "C" __EXPORT int fw_pos_control_main(int argc, char *argv[])
{
	return FixedwingPositionControl::main(argc, argv);
}
