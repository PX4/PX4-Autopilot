/****************************************************************************
 *
 * Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include "FwLateralLongitudinalControl.hpp"
#include <px4_platform_common/events.h>

using math::constrain;
using math::max;
using math::radians;

using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector2f;

// [m/s] maximum reference altitude rate threshhold
static constexpr float MAX_ALT_REF_RATE_FOR_LEVEL_FLIGHT = 0.1f;
// [us] time after which the wind estimate is disabled if no longer updating
static constexpr hrt_abstime WIND_EST_TIMEOUT = 10_s;
// [s] slew rate with which we change altitude time constant
static constexpr float TECS_ALT_TIME_CONST_SLEW_RATE = 1.0f;

static constexpr float HORIZONTAL_EVH_FACTOR_COURSE_VALID{3.f}; ///< Factor of velocity standard deviation above which course calculation is considered good enough
static constexpr float HORIZONTAL_EVH_FACTOR_COURSE_INVALID{2.f}; ///< Factor of velocity standard deviation below which course calculation is considered unsafe
static constexpr float COS_HEADING_TRACK_ANGLE_NOT_PUSHED_BACK{0.09f}; ///< Cos of Heading to track angle below which it is assumed that the vehicle is not pushed back by the wind ~cos(85°)
static constexpr float COS_HEADING_TRACK_ANGLE_PUSHED_BACK{0.f}; ///< Cos of Heading to track angle above which it is assumed that the vehicle is pushed back by the wind

// [s] Timeout that has to pass in roll-constraining failsafe before warning is triggered
static constexpr uint64_t ROLL_WARNING_TIMEOUT = 2_s;

// [-] Can-run threshold needed to trigger the roll-constraining failsafe warning
static constexpr float ROLL_WARNING_CAN_RUN_THRESHOLD = 0.9f;

// [m/s/s] slew rate limit for airspeed setpoint changes
static constexpr float ASPD_SP_SLEW_RATE = 1.f;

FwLateralLongitudinalControl::FwLateralLongitudinalControl(bool is_vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_attitude_sp_pub(is_vtol ? ORB_ID(fw_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_tecs_status_pub.advertise();
	_flight_phase_estimation_pub.advertise();
	_fixed_wing_lateral_status_pub.advertise();
	parameters_update();
	_airspeed_slew_rate_controller.setSlewRate(ASPD_SP_SLEW_RATE);
}

FwLateralLongitudinalControl::~FwLateralLongitudinalControl()
{
	perf_free(_loop_perf);
}
void
FwLateralLongitudinalControl::parameters_update()
{
	updateParams();
	_performance_model.updateParameters();
	_performance_model.runSanityChecks();

	_tecs.set_max_climb_rate(_performance_model.getMaximumClimbRate(_air_density));
	_tecs.set_max_sink_rate(_param_fw_t_sink_max.get());
	_tecs.set_min_sink_rate(_performance_model.getMinimumSinkRate(_air_density));
	_tecs.set_equivalent_airspeed_trim(_performance_model.getCalibratedTrimAirspeed());
	_tecs.set_equivalent_airspeed_min(_performance_model.getMinimumCalibratedAirspeed(getLoadFactor(), _flaps_setpoint));
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

	_roll_slew_rate.setSlewRate(radians(_param_fw_pn_r_slew_max.get()));

	_tecs_alt_time_const_slew_rate.setSlewRate(TECS_ALT_TIME_CONST_SLEW_RATE);
	_tecs_alt_time_const_slew_rate.setForcedValue(_param_fw_t_h_error_tc.get() * _param_fw_thrtc_sc.get());
}

void FwLateralLongitudinalControl::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	/* only run controller if position changed */
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		parameters_update();
	}

	if (_local_pos_sub.update(&_local_pos)) {

		const float control_interval = math::constrain((_local_pos.timestamp - _last_time_loop_ran) * 1e-6f,
					       0.001f, 0.1f);
		_last_time_loop_ran = _local_pos.timestamp;

		updateControllerConfiguration();

		_tecs.set_speed_weight(_long_configuration.speed_weight);
		updateTECSAltitudeTimeConstant(checkLowHeightConditions()
					       || _long_configuration.enforce_low_height_condition, control_interval);
		_tecs.set_altitude_error_time_constant(_tecs_alt_time_const_slew_rate.getState());

		if (_vehicle_air_data_sub.updated()) {
			_vehicle_air_data_sub.update();
			_air_density = PX4_ISFINITE(_vehicle_air_data_sub.get().rho) ? _vehicle_air_data_sub.get().rho : _air_density;
			_tecs.set_max_climb_rate(_performance_model.getMaximumClimbRate(_air_density));
			_tecs.set_min_sink_rate(_performance_model.getMinimumSinkRate(_air_density));
		}

		if (_vehicle_landed_sub.updated()) {
			vehicle_land_detected_s landed{};
			_vehicle_landed_sub.copy(&landed);
			_landed = landed.landed;
		}

		_flight_phase_estimation_pub.get().flight_phase = flight_phase_estimation_s::FLIGHT_PHASE_UNKNOWN;

		_vehicle_status_sub.update();
		_control_mode_sub.update();

		if (_flaps_setpoint_sub.updated()) {
			normalized_unsigned_setpoint_s flaps_setpoint{};
			_flaps_setpoint_sub.copy(&flaps_setpoint);
			_flaps_setpoint = flaps_setpoint.normalized_setpoint;
		}

		update_control_state();

		if (_control_mode_sub.get().flag_control_manual_enabled && _control_mode_sub.get().flag_control_altitude_enabled
		    && _local_pos.z_reset_counter != _z_reset_counter) {
			if (_control_mode_sub.get().flag_control_altitude_enabled && _local_pos.z_reset_counter != _z_reset_counter) {
				// make TECS accept step in altitude and demanded altitude
				_tecs.handle_alt_step(_long_control_state.altitude_msl, _long_control_state.height_rate);
			}
		}

		const bool should_run = (_control_mode_sub.get().flag_control_position_enabled ||
					 _control_mode_sub.get().flag_control_velocity_enabled ||
					 _control_mode_sub.get().flag_control_acceleration_enabled ||
					 _control_mode_sub.get().flag_control_altitude_enabled ||
					 _control_mode_sub.get().flag_control_climb_rate_enabled) &&
					(_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
					 || _vehicle_status_sub.get().in_transition_mode);

		if (should_run) {

			// ----- Longitudinal ------
			float pitch_sp{NAN};
			float throttle_sp{NAN};

			if (_fw_longitudinal_ctrl_sub.updated()) {
				_fw_longitudinal_ctrl_sub.copy(&_long_control_sp);
			}

			const float airspeed_sp_eas = adapt_airspeed_setpoint(control_interval, _long_control_sp.equivalent_airspeed,
						      _min_airspeed_from_guidance, _lateral_control_state.wind_speed.length());

			// If the both altitude and height rate are set, set altitude setpoint to NAN
			const float altitude_sp = PX4_ISFINITE(_long_control_sp.height_rate) ? NAN : _long_control_sp.altitude;

			tecs_update_pitch_throttle(control_interval, altitude_sp,
						   airspeed_sp_eas,
						   _long_configuration.pitch_min,
						   _long_configuration.pitch_max,
						   _long_configuration.throttle_min,
						   _long_configuration.throttle_max,
						   _long_configuration.sink_rate_target,
						   _long_configuration.climb_rate_target,
						   _long_configuration.disable_underspeed_protection,
						   _long_control_sp.height_rate
						  );

			pitch_sp = PX4_ISFINITE(_long_control_sp.pitch_direct) ? _long_control_sp.pitch_direct : _tecs.get_pitch_setpoint();
			throttle_sp = PX4_ISFINITE(_long_control_sp.throttle_direct) ? _long_control_sp.throttle_direct :
				      _tecs.get_throttle_setpoint();

			// ----- Lateral ------
			float roll_sp {NAN};

			if (_fw_lateral_ctrl_sub.updated()) {
				// We store the update of _fw_lateral_ctrl_sub in a member variable instead of only local such that we can run
				// the controllers also without new setpoints.
				_fw_lateral_ctrl_sub.copy(&_lat_control_sp);
			}

			float airspeed_direction_sp{NAN};
			float lateral_accel_sp {NAN};
			const Vector2f airspeed_vector = _lateral_control_state.ground_speed - _lateral_control_state.wind_speed;

			if (PX4_ISFINITE(_lat_control_sp.course) && !PX4_ISFINITE(_lat_control_sp.airspeed_direction)) {
				// only use the course setpoint if it's finite but airspeed_direction is not

				airspeed_direction_sp = _course_to_airspeed.mapCourseSetpointToHeadingSetpoint(
								_lat_control_sp.course, _lateral_control_state.wind_speed,
								airspeed_sp_eas);

				// Note: the here updated _min_airspeed_from_guidance is only used in the next iteration
				// in the longitudinal controller.
				const float max_true_airspeed = _performance_model.getMaximumCalibratedAirspeed() * _long_control_state.eas2tas;
				_min_airspeed_from_guidance = _course_to_airspeed.getMinAirspeedForCurrentBearing(
								      _lat_control_sp.course, _lateral_control_state.wind_speed,
								      max_true_airspeed, _param_fw_gnd_spd_min.get())
							      / _long_control_state.eas2tas;

			} else if (PX4_ISFINITE(_lat_control_sp.airspeed_direction)) {
				// If the airspeed_direction is finite we use that instead of the course.

				airspeed_direction_sp = _lat_control_sp.airspeed_direction;
				_min_airspeed_from_guidance = 0.f; // reset if no longer in course control

			} else {
				_min_airspeed_from_guidance = 0.f; // reset if no longer in course control
			}

			if (PX4_ISFINITE(airspeed_direction_sp)) {
				const float heading = atan2f(airspeed_vector(1), airspeed_vector(0));
				lateral_accel_sp = _airspeed_direction_control.controlHeading(airspeed_direction_sp, heading,
						   airspeed_vector.norm());
			}

			if (PX4_ISFINITE(_lat_control_sp.lateral_acceleration)) {
				lateral_accel_sp = PX4_ISFINITE(lateral_accel_sp) ? lateral_accel_sp + _lat_control_sp.lateral_acceleration :
						   _lat_control_sp.lateral_acceleration;
			}

			if (!PX4_ISFINITE(lateral_accel_sp)) {
				lateral_accel_sp = 0.f; // mitigation if no valid setpoint is received: 0 lateral acceleration
			}

			lateral_accel_sp = getCorrectedLateralAccelSetpoint(lateral_accel_sp);
			lateral_accel_sp = math::constrain(lateral_accel_sp, -_lateral_configuration.lateral_accel_max,
							   _lateral_configuration.lateral_accel_max);
			roll_sp = mapLateralAccelerationToRollAngle(lateral_accel_sp);

			fixed_wing_lateral_status_s fixed_wing_lateral_status{};
			fixed_wing_lateral_status.timestamp = hrt_absolute_time();
			fixed_wing_lateral_status.lateral_acceleration_setpoint = lateral_accel_sp;
			fixed_wing_lateral_status.can_run_factor = _can_run_factor;

			_fixed_wing_lateral_status_pub.publish(fixed_wing_lateral_status);

			// additional is_finite checks that should not be necessary, but are kept for safety
			float roll_body = PX4_ISFINITE(roll_sp) ? roll_sp : 0.0f;
			float pitch_body = PX4_ISFINITE(pitch_sp) ? pitch_sp : 0.0f;
			const float yaw_body = _yaw; // yaw is not controlled in fixed wing, need to set it though for quaternion generation
			const float thrust_body_x = PX4_ISFINITE(throttle_sp) ? throttle_sp : 0.0f;

			if (_control_mode_sub.get().flag_control_manual_enabled) {
				roll_body = constrain(roll_body, -radians(_param_fw_r_lim.get()),
						      radians(_param_fw_r_lim.get()));
				pitch_body = constrain(pitch_body, radians(_param_fw_p_lim_min.get()),
						       radians(_param_fw_p_lim_max.get()));
			}

			// roll slew rate
			roll_body = _roll_slew_rate.update(roll_body, control_interval);

			_att_sp.timestamp = hrt_absolute_time();
			const Quatf q(Eulerf(roll_body, pitch_body, yaw_body));
			q.copyTo(_att_sp.q_d);

			_att_sp.thrust_body[0] = thrust_body_x;

			_attitude_sp_pub.publish(_att_sp);

		}

		_z_reset_counter = _local_pos.z_reset_counter;
	}

	perf_end(_loop_perf);
}

void FwLateralLongitudinalControl::updateControllerConfiguration()
{
	if (_lateral_configuration.timestamp == 0) {
		_lateral_configuration.timestamp = _local_pos.timestamp;
		_lateral_configuration.lateral_accel_max = tanf(radians(_param_fw_r_lim.get())) * CONSTANTS_ONE_G;

	}

	if (_long_configuration.timestamp == 0) {
		setDefaultLongitudinalControlConfiguration();
	}

	if (_long_control_configuration_sub.updated() || _parameter_update_sub.updated()) {
		longitudinal_control_configuration_s configuration_in{};
		_long_control_configuration_sub.copy(&configuration_in);
		updateLongitudinalControlConfiguration(configuration_in);
	}

	if (_lateral_control_configuration_sub.updated() || _parameter_update_sub.updated()) {
		lateral_control_configuration_s configuration_in{};
		_lateral_control_configuration_sub.copy(&configuration_in);
		_lateral_configuration.timestamp = configuration_in.timestamp;

		if (PX4_ISFINITE(configuration_in.lateral_accel_max)) {
			_lateral_configuration.lateral_accel_max = min(configuration_in.lateral_accel_max, tanf(radians(
						_param_fw_r_lim.get())) * CONSTANTS_ONE_G);

		} else {
			_lateral_configuration.lateral_accel_max = tanf(radians(_param_fw_r_lim.get())) * CONSTANTS_ONE_G;
		}
	}
}

void
FwLateralLongitudinalControl::tecs_update_pitch_throttle(const float control_interval, float alt_sp, float airspeed_sp,
		float pitch_min_rad, float pitch_max_rad, float throttle_min,
		float throttle_max, const float desired_max_sinkrate,
		const float desired_max_climbrate,
		bool disable_underspeed_detection, float hgt_rate_sp)
{
	bool tecs_is_running = true;

	// do not run TECS if vehicle is a VTOL and we are in rotary wing mode or in transition
	if (_vehicle_status_sub.get().is_vtol
	    && (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		|| _vehicle_status_sub.get().in_transition_mode)) {
		tecs_is_running = false;
		return;

	}

	const float throttle_trim_compensated = _performance_model.getTrimThrottle(throttle_min,
						throttle_max, airspeed_sp, _air_density);

	_tecs.set_detect_underspeed_enabled(!disable_underspeed_detection);

	// HOTFIX: the airspeed rate estimate using acceleration in body-forward direction has shown to lead to high biases
	// when flying tight turns. It's in this case much safer to just set the estimated airspeed rate to 0.
	const float airspeed_rate_estimate = 0.f;

	_tecs.update(_long_control_state.pitch_rad - radians(_param_fw_psp_off.get()),
		     _long_control_state.altitude_msl,
		     alt_sp,
		     airspeed_sp,
		     _long_control_state.airspeed_eas,
		     _long_control_state.eas2tas,
		     throttle_min,
		     throttle_max,
		     throttle_trim_compensated,
		     pitch_min_rad - radians(_param_fw_psp_off.get()),
		     pitch_max_rad - radians(_param_fw_psp_off.get()),
		     desired_max_climbrate,
		     desired_max_sinkrate,
		     airspeed_rate_estimate,
		     _long_control_state.height_rate,
		     hgt_rate_sp);

	tecs_status_publish(alt_sp, airspeed_sp, airspeed_rate_estimate, throttle_trim_compensated);

	if (tecs_is_running && !_vehicle_status_sub.get().in_transition_mode
	    && (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING)) {
		const TECS::DebugOutput &tecs_output{_tecs.getStatus()};

		// Check level flight: the height rate setpoint is not set or set to 0 and we are close to the target altitude and target altitude is not moving
		if ((fabsf(tecs_output.height_rate_reference) < MAX_ALT_REF_RATE_FOR_LEVEL_FLIGHT) &&
		    fabsf(_long_control_state.altitude_msl - tecs_output.altitude_reference) < _param_nav_fw_alt_rad.get()) {
			_flight_phase_estimation_pub.get().flight_phase = flight_phase_estimation_s::FLIGHT_PHASE_LEVEL;

		} else if (((tecs_output.altitude_reference - _long_control_state.altitude_msl) >= _param_nav_fw_alt_rad.get()) ||
			   (tecs_output.height_rate_reference >= MAX_ALT_REF_RATE_FOR_LEVEL_FLIGHT)) {
			_flight_phase_estimation_pub.get().flight_phase = flight_phase_estimation_s::FLIGHT_PHASE_CLIMB;

		} else if (((_long_control_state.altitude_msl - tecs_output.altitude_reference) >= _param_nav_fw_alt_rad.get()) ||
			   (tecs_output.height_rate_reference <= -MAX_ALT_REF_RATE_FOR_LEVEL_FLIGHT)) {
			_flight_phase_estimation_pub.get().flight_phase = flight_phase_estimation_s::FLIGHT_PHASE_DESCEND;

		} else {
			//We can't infer the flight phase , do nothing, estimation is reset at each step
		}
	}
}

void
FwLateralLongitudinalControl::tecs_status_publish(float alt_sp, float equivalent_airspeed_sp,
		float true_airspeed_derivative_raw, float throttle_trim)
{
	tecs_status_s tecs_status{};

	const TECS::DebugOutput &debug_output{_tecs.getStatus()};

	tecs_status.altitude_sp = alt_sp;
	tecs_status.altitude_reference = debug_output.altitude_reference;
	tecs_status.altitude_time_constant = _tecs.get_altitude_error_time_constant();
	tecs_status.height_rate_reference = debug_output.height_rate_reference;
	tecs_status.height_rate_direct = debug_output.height_rate_direct;
	tecs_status.height_rate_setpoint = debug_output.control.altitude_rate_control;
	tecs_status.height_rate = -_local_pos.vz;
	tecs_status.equivalent_airspeed_sp = equivalent_airspeed_sp;
	tecs_status.true_airspeed_sp = debug_output.true_airspeed_sp;
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
	tecs_status.fast_descend_ratio = debug_output.fast_descend;

	tecs_status.timestamp = hrt_absolute_time();

	_tecs_status_pub.publish(tecs_status);
}

int FwLateralLongitudinalControl::task_spawn(int argc, char *argv[])
{
	bool is_vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			is_vtol = true;
		}
	}

	FwLateralLongitudinalControl *instance = new FwLateralLongitudinalControl(is_vtol);

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

bool
FwLateralLongitudinalControl::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

int FwLateralLongitudinalControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FwLateralLongitudinalControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_lat_lon_control computes attitude and throttle setpoints from lateral and longitudinal control setpoints.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_lat_lon_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

void FwLateralLongitudinalControl::update_control_state() {
	updateAltitudeAndHeightRate();
	updateAirspeed();
	updateAttitude();
	updateWind();

	_lateral_control_state.ground_speed = Vector2f(_local_pos.vx, _local_pos.vy);
}

void FwLateralLongitudinalControl::updateWind() {
	if (_wind_sub.updated()) {
		wind_s wind{};
		_wind_sub.update(&wind);

		// assumes wind is valid if finite
		_wind_valid = PX4_ISFINITE(wind.windspeed_north)
			      && PX4_ISFINITE(wind.windspeed_east);

		_time_wind_last_received = hrt_absolute_time();

		_lateral_control_state.wind_speed(0) = wind.windspeed_north;
		_lateral_control_state.wind_speed(1) = wind.windspeed_east;

	} else {
		// invalidate wind estimate usage (and correspondingly NPFG, if enabled) after subscription timeout
		_wind_valid = _wind_valid && (hrt_absolute_time() - _time_wind_last_received) < WIND_EST_TIMEOUT;
	}

	if (!_wind_valid) {
		_lateral_control_state.wind_speed.setZero();
	}
}

void FwLateralLongitudinalControl::updateAltitudeAndHeightRate() {
	float ref_alt{0.f};
	if (_local_pos.z_global && PX4_ISFINITE(_local_pos.ref_alt)) {
		ref_alt = _local_pos.ref_alt;
	}

	_long_control_state.altitude_msl = -_local_pos.z + ref_alt; // Altitude AMSL in meters
	_long_control_state.height_rate = -_local_pos.vz;

}

void FwLateralLongitudinalControl::updateAttitude() {
	vehicle_attitude_s att;

	if (_vehicle_attitude_sub.update(&att)) {

		Dcmf R{Quatf(att.q)};

		// if the vehicle is a tailsitter we have to rotate the attitude by the pitch offset
		// between multirotor and fixed wing flight
		if (_vehicle_status_sub.get().is_vtol_tailsitter) {
			const Dcmf R_offset{Eulerf{0.f, M_PI_2_F, 0.f}};
			R = R * R_offset;
		}

		const Eulerf euler_angles(R);
		_long_control_state.pitch_rad = euler_angles.theta();
		_yaw = euler_angles.psi();

		// load factor due to banking
		const float load_factor_from_bank_angle = 1.0f / max(cosf(euler_angles.phi()), FLT_EPSILON);
		_tecs.set_load_factor(load_factor_from_bank_angle);
	}
}

void FwLateralLongitudinalControl::updateAirspeed() {

	airspeed_validated_s airspeed_validated;

	if (_param_fw_use_airspd.get() && _airspeed_validated_sub.update(&airspeed_validated)) {

		// do not use synthetic airspeed as this would create a thrust loop
		if (PX4_ISFINITE(airspeed_validated.calibrated_airspeed_m_s)
		    && PX4_ISFINITE(airspeed_validated.true_airspeed_m_s)
		    && airspeed_validated.airspeed_source != airspeed_validated_s::SOURCE_SYNTHETIC) {

			_time_airspeed_last_valid = airspeed_validated.timestamp;
			_long_control_state.airspeed_eas = airspeed_validated.calibrated_airspeed_m_s;
			_long_control_state.eas2tas = constrain(airspeed_validated.true_airspeed_m_s / airspeed_validated.calibrated_airspeed_m_s, 0.9f, 2.0f);
		}
	}

	// no airspeed updates for one second --> declare invalid
	const bool airspeed_valid = hrt_elapsed_time(&_time_airspeed_last_valid) < 1_s;

	if (!airspeed_valid) {
		_long_control_state.eas2tas = 1.f;
	}

	_tecs.enable_airspeed(airspeed_valid);
}

float
FwLateralLongitudinalControl::adapt_airspeed_setpoint(const float control_interval, float calibrated_airspeed_setpoint,
		float calibrated_min_airspeed_guidance, float wind_speed)
{
	float system_min_airspeed = _performance_model.getMinimumCalibratedAirspeed(getLoadFactor(), _flaps_setpoint);

	const float system_max_airspeed = _performance_model.getMaximumCalibratedAirspeed();

	// airspeed setpoint adjustments
	if (!PX4_ISFINITE(calibrated_airspeed_setpoint) || calibrated_airspeed_setpoint <= FLT_EPSILON) {
		calibrated_airspeed_setpoint = _performance_model.getCalibratedTrimAirspeed();

		// Aditional option to increase the min airspeed setpoint based on wind estimate for more stability in higher winds.
		if (_wind_valid && _param_fw_wind_arsp_sc.get() > FLT_EPSILON) {
			system_min_airspeed = math::min(system_min_airspeed + _param_fw_wind_arsp_sc.get() *
			wind_speed, system_max_airspeed);
		}
	}

	// increase setpoint to at what's at least required for the lateral guidance
	calibrated_airspeed_setpoint = math::max(calibrated_airspeed_setpoint, calibrated_min_airspeed_guidance);

	// constrain airspeed to feasible range
	calibrated_airspeed_setpoint = math::constrain(calibrated_airspeed_setpoint, system_min_airspeed, system_max_airspeed);

	if (!PX4_ISFINITE(_airspeed_slew_rate_controller.getState())) {

		// initialize the airspeed setpoint
		if (PX4_ISFINITE(_long_control_state.airspeed_eas) && _long_control_state.airspeed_eas < system_min_airspeed) {
			// current airpseed is below minimum - init with minimum
			_airspeed_slew_rate_controller.setForcedValue(system_min_airspeed);

		} else if (PX4_ISFINITE(_long_control_state.airspeed_eas) && _long_control_state.airspeed_eas > system_max_airspeed) {
			// current airpseed is above maximum - init with maximum
			_airspeed_slew_rate_controller.setForcedValue(system_max_airspeed);

		} else if (PX4_ISFINITE(_long_control_state.airspeed_eas)) {
			// current airpseed is between min and max - init with current
			_airspeed_slew_rate_controller.setForcedValue(_long_control_state.airspeed_eas);

		} else {
			// current airpseed is invalid - init with setpoint
			_airspeed_slew_rate_controller.setForcedValue(calibrated_airspeed_setpoint);
		}
	} else {
		// update slew rate state
		if (_airspeed_slew_rate_controller.getState() < system_min_airspeed) {
			// current airpseed setpoint is below minimum - reset to minimum
			_airspeed_slew_rate_controller.setForcedValue(system_min_airspeed);

		} else if (_airspeed_slew_rate_controller.getState() > system_max_airspeed) {
			// current airpseed setpoint is above maximum - reset to maximum
			_airspeed_slew_rate_controller.setForcedValue(system_max_airspeed);

		} else if (PX4_ISFINITE(_long_control_state.airspeed_eas)) {
			// current airpseed setpoint is between min and max - update
			_airspeed_slew_rate_controller.update(calibrated_airspeed_setpoint, control_interval);

		}
	}

	return _airspeed_slew_rate_controller.getState();
}

bool FwLateralLongitudinalControl::checkLowHeightConditions() const
{
	// Are conditions for low-height
	return _param_fw_t_thr_low_hgt.get() >= 0.f && _local_pos.dist_bottom_valid
	       && _local_pos.dist_bottom < _param_fw_t_thr_low_hgt.get();
}

void FwLateralLongitudinalControl::updateTECSAltitudeTimeConstant(const bool is_low_height, const float dt)
{
	// Target time constant for the TECS altitude tracker
	float alt_tracking_tc = _param_fw_t_h_error_tc.get();

	if (is_low_height) {
		// If low-height conditions satisfied, compute target time constant for altitude tracking
		alt_tracking_tc *= _param_fw_thrtc_sc.get();
	}

	_tecs_alt_time_const_slew_rate.update(alt_tracking_tc, dt);
}

float FwLateralLongitudinalControl::getGuidanceQualityFactor(const vehicle_local_position_s &local_pos, const bool is_wind_valid) const
{
	if (is_wind_valid) {
		// If we have a valid wind estimate, npfg is able to handle all degenerated cases
		return 1.f;
	}

	// NPFG can run without wind information as long as the system is not flying backwards and has a minimal ground speed
	// Check the minimal ground speed. if it is greater than twice the standard deviation, we assume that we can infer a valid track angle
	const Vector2f ground_vel(local_pos.vx, local_pos.vy);
	const float ground_speed(ground_vel.norm());
	const float low_ground_speed_factor(math::constrain((ground_speed - HORIZONTAL_EVH_FACTOR_COURSE_INVALID *
									    local_pos.evh) / ((HORIZONTAL_EVH_FACTOR_COURSE_VALID - HORIZONTAL_EVH_FACTOR_COURSE_INVALID)*local_pos.evh),
							    0.f, 1.f));

	// Check that the angle between heading and track is not off too much. if it is greater than 90° we will be pushed back from the wind and the npfg will propably give a roll command in the wrong direction.
	const Vector2f heading_vector(matrix::Dcm2f(local_pos.heading)*Vector2f({1.f, 0.f}));
	const Vector2f ground_vel_norm(ground_vel.normalized());
	const float flying_forward_factor(math::constrain((heading_vector.dot(ground_vel_norm) -
							   COS_HEADING_TRACK_ANGLE_PUSHED_BACK) / ((COS_HEADING_TRACK_ANGLE_NOT_PUSHED_BACK -
												    COS_HEADING_TRACK_ANGLE_PUSHED_BACK)),
							  0.f, 1.f));

	return flying_forward_factor * low_ground_speed_factor;
}
float FwLateralLongitudinalControl::getCorrectedLateralAccelSetpoint(float lateral_accel_sp)
{
	// Scale the npfg output to zero if npfg is not certain for correct output
	_can_run_factor = math::constrain(getGuidanceQualityFactor(_local_pos, _wind_valid), 0.f, 1.f);

	hrt_abstime now{hrt_absolute_time()};

	// Warn the user when the scale is less than 90% for at least 2 seconds (disable in transition)

	// If the npfg was not running before, reset the user warning variables.
	if ((now - _time_since_last_npfg_call) > ROLL_WARNING_TIMEOUT) {
		_need_report_npfg_uncertain_condition = true;
		_time_since_first_reduced_roll = 0U;
	}

	if (_vehicle_status_sub.get().in_transition_mode || _can_run_factor > ROLL_WARNING_CAN_RUN_THRESHOLD || _landed) {
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

	return _can_run_factor * (lateral_accel_sp);
}
float FwLateralLongitudinalControl::mapLateralAccelerationToRollAngle(float lateral_acceleration_sp) const {
	return  atanf(lateral_acceleration_sp / CONSTANTS_ONE_G);
}

void FwLateralLongitudinalControl::setDefaultLongitudinalControlConfiguration() {
	_long_configuration.timestamp = hrt_absolute_time();
	_long_configuration.pitch_min = radians(_param_fw_p_lim_min.get());
	_long_configuration.pitch_max = radians(_param_fw_p_lim_max.get());
	_long_configuration.throttle_min = _param_fw_thr_min.get();
	_long_configuration.throttle_max = _param_fw_thr_max.get();
	_long_configuration.climb_rate_target = _param_climbrate_target.get();
	_long_configuration.sink_rate_target = _param_sinkrate_target.get();
	_long_configuration.disable_underspeed_protection = false;
	_long_configuration.enforce_low_height_condition = false;
}

void FwLateralLongitudinalControl::updateLongitudinalControlConfiguration(const longitudinal_control_configuration_s &configuration_in) {
	_long_configuration.timestamp = configuration_in.timestamp;

	if (PX4_ISFINITE(configuration_in.pitch_min)) {
		_long_configuration.pitch_min = math::constrain(configuration_in.pitch_min, radians(_param_fw_p_lim_min.get()), radians(_param_fw_p_lim_max.get()));
	} else {
		_long_configuration.pitch_min = radians(_param_fw_p_lim_min.get());
	}

	if (PX4_ISFINITE(configuration_in.pitch_max)) {
		_long_configuration.pitch_max = math::constrain(configuration_in.pitch_max, _long_configuration.pitch_min, radians(_param_fw_p_lim_max.get()));
	} else {
		_long_configuration.pitch_max = radians(_param_fw_p_lim_max.get());
	}

	if (PX4_ISFINITE(configuration_in.throttle_min)) {
		_long_configuration.throttle_min = math::constrain(configuration_in.throttle_min, _param_fw_thr_min.get(), _param_fw_thr_max.get());
	} else {
		_long_configuration.throttle_min = _param_fw_thr_min.get();
	}

	if (PX4_ISFINITE(configuration_in.throttle_max)) {
		_long_configuration.throttle_max = math::constrain(configuration_in.throttle_max, _long_configuration.throttle_min, _param_fw_thr_max.get());
	} else {
		_long_configuration.throttle_max = _param_fw_thr_max.get();
	}

	if (PX4_ISFINITE(configuration_in.climb_rate_target)) {
		_long_configuration.climb_rate_target = math::max(0.0f, configuration_in.climb_rate_target);
	} else {
		_long_configuration.climb_rate_target = _param_climbrate_target.get();
	}

	if (PX4_ISFINITE(configuration_in.sink_rate_target)) {
		_long_configuration.sink_rate_target = math::max(0.0f, configuration_in.sink_rate_target);
	} else {
		_long_configuration.sink_rate_target = _param_sinkrate_target.get();
	}

	if (PX4_ISFINITE(configuration_in.speed_weight)) {
		_long_configuration.speed_weight = math::constrain(configuration_in.speed_weight, 0.f, 2.f);
	} else {
		_long_configuration.speed_weight = _param_t_spdweight.get();
	}
}

float FwLateralLongitudinalControl::getLoadFactor() const
{
	float load_factor_from_bank_angle = 1.f;

	const float roll_body = Eulerf(Quatf(_att_sp.q_d)).phi();

	if (PX4_ISFINITE(roll_body)) {
		load_factor_from_bank_angle = 1.f / math::max(cosf(roll_body), FLT_EPSILON);
	}

	return load_factor_from_bank_angle;
}

extern "C" __EXPORT int fw_lat_lon_control_main(int argc, char *argv[])
{
	return FwLateralLongitudinalControl::main(argc, argv);
}
