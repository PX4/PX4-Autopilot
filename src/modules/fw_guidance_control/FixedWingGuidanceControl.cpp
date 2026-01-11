/****************************************************************************
 *
 *   Copyright (c) 2013-2025 PX4 Development Team. All rights reserved.
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

#include "FixedWingGuidanceControl.hpp"

#include <px4_platform_common/events.h>
#include <uORB/topics/longitudinal_control_configuration.h>

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

const fixed_wing_lateral_setpoint_s empty_lateral_control_setpoint = {.timestamp = 0, .course = NAN, .airspeed_direction = NAN, .lateral_acceleration = NAN};
const fixed_wing_longitudinal_setpoint_s empty_longitudinal_control_setpoint = {.timestamp = 0, .altitude = NAN, .height_rate = NAN, .equivalent_airspeed = NAN, .pitch_direct = NAN, .throttle_direct = NAN};

FixedWingGuidanceControl::FixedWingGuidanceControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	// limit to 50 Hz
	_local_pos_sub.set_interval_ms(20);

	_launch_detection_status_pub.advertise();
	_fixed_wing_lateral_guidance_status_pub.advertise();

	parameters_update();
}

FixedWingGuidanceControl::~FixedWingGuidanceControl()
{
	perf_free(_loop_perf);
}

bool
FixedWingGuidanceControl::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
FixedWingGuidanceControl::parameters_update()
{
	updateParams();

	_directional_guidance.setPeriod(_param_npfg_period.get());
	_directional_guidance.setDamping(_param_npfg_damping.get());
	_directional_guidance.enablePeriodLB(_param_npfg_en_period_lb.get());
	_directional_guidance.enablePeriodUB(_param_npfg_en_period_ub.get());
	_directional_guidance.setRollTimeConst(_param_npfg_roll_time_const.get());
	_directional_guidance.setSwitchDistanceMultiplier(_param_npfg_switch_distance_multiplier.get());
	_directional_guidance.setPeriodSafetyFactor(_param_npfg_period_safety_factor.get());
}

void
FixedWingGuidanceControl::vehicle_control_mode_poll()
{
}

void
FixedWingGuidanceControl::airspeed_poll()
{
	airspeed_validated_s airspeed_validated;

	if (_param_fw_use_airspd.get() && _airspeed_validated_sub.update(&airspeed_validated)) {

		// do not use synthetic airspeed as it's for the use here not reliable enough
		if (PX4_ISFINITE(airspeed_validated.calibrated_airspeed_m_s)
		    && airspeed_validated.airspeed_source != airspeed_validated_s::SOURCE_SYNTHETIC) {

			_airspeed_eas = airspeed_validated.calibrated_airspeed_m_s;
		}
	}

	// no airspeed updates for one second --> declare invalid
	// this flag is used for some logic like: exiting takeoff, flaps retraction
	_airspeed_valid = hrt_elapsed_time(&_time_airspeed_last_valid) < 1_s;
}

void
FixedWingGuidanceControl::wind_poll(const hrt_abstime now)
{
	if (_wind_sub.updated()) {
		wind_s wind;
		_wind_sub.update(&wind);

		// assumes wind is valid if finite
		_wind_valid = PX4_ISFINITE(wind.windspeed_north)
			      && PX4_ISFINITE(wind.windspeed_east);

		_time_wind_last_received = now;

		_wind_vel(0) = wind.windspeed_north;
		_wind_vel(1) = wind.windspeed_east;

	} else {
		// invalidate wind estimate usage (and correspondingly NPFG, if enabled) after subscription timeout
		_wind_valid = _wind_valid && (now - _time_wind_last_received) < WIND_EST_TIMEOUT;
	}

	if (!_wind_valid) {
		_wind_vel(0) = 0.f;
		_wind_vel(1) = 0.f;
	}
}

void
FixedWingGuidanceControl::manual_control_setpoint_poll()
{
	_sticks.checkAndUpdateStickInputs();

	_manual_control_setpoint_for_height_rate = _sticks.getPitch();
	_manual_control_setpoint_for_airspeed = _sticks.getThrottleZeroCentered();

	if (_param_fw_pos_stk_conf.get() & STICK_CONFIG_SWAP_STICKS_BIT) {
		/* Alternate stick allocation (similar concept as for multirotor systems:
		 * demanding up/down with the throttle stick, and move faster/break with the pitch one.
		 */

		_manual_control_setpoint_for_height_rate = -_sticks.getThrottleZeroCentered();
		_manual_control_setpoint_for_airspeed = _sticks.getPitch();
	}
}

void
FixedWingGuidanceControl::vehicle_attitude_poll()
{
	vehicle_attitude_s vehicle_attitude;

	if (_vehicle_attitude_sub.update(&vehicle_attitude)) {
		vehicle_angular_velocity_s angular_velocity{};
		_vehicle_angular_velocity_sub.copy(&angular_velocity);
		const Vector3f rates{angular_velocity.xyz};

		Dcmf R{Quatf(vehicle_attitude.q)};

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
		_yaw = euler_angles(2);

		const Vector3f body_acceleration = R.transpose() * Vector3f{_local_pos.ax, _local_pos.ay, _local_pos.az};
		_body_acceleration_x = body_acceleration(0);

		const Vector3f body_velocity = R.transpose() * Vector3f{_local_pos.vx, _local_pos.vy, _local_pos.vz};
		_body_velocity_x = body_velocity(0);
	}
}

void
FixedWingGuidanceControl::update_in_air_states(const hrt_abstime now)
{
	/* reset flag when airplane landed */
	if (_landed) {
		_completed_manual_takeoff = false;
	}
}

void
FixedWingGuidanceControl::control_auto_path(const float control_interval, const Vector2d &curr_pos,
		const Vector2f &ground_speed, const position_setpoint_s &pos_sp_curr)
{
	const float target_airspeed = pos_sp_curr.cruising_speed > FLT_EPSILON ? pos_sp_curr.cruising_speed : NAN;

	Vector2f curr_pos_local{_local_pos.x, _local_pos.y};
	Vector2f curr_wp_local = _global_local_proj_ref.project(pos_sp_curr.lat, pos_sp_curr.lon);

	// Navigate directly on position setpoint and path tangent
	const matrix::Vector2f velocity_2d(pos_sp_curr.vx, pos_sp_curr.vy);
	const float curvature = PX4_ISFINITE(_pos_sp_triplet.current.loiter_radius) ? 1 /
				_pos_sp_triplet.current.loiter_radius :
				0.0f;
	const DirectionalGuidanceOutput sp = navigatePathTangent(curr_pos_local, curr_wp_local, velocity_2d.normalized(),
					     ground_speed, _wind_vel, curvature);

	fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
	fw_lateral_ctrl_sp.timestamp = hrt_absolute_time();
	fw_lateral_ctrl_sp.course = sp.course_setpoint;
	fw_lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;
	_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);

	const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
		.timestamp = hrt_absolute_time(),
		.altitude = pos_sp_curr.alt,
		.height_rate = NAN,
		.equivalent_airspeed = target_airspeed,
		.pitch_direct = NAN,
		.throttle_direct = NAN
	};

	_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

	if (pos_sp_curr.gliding_enabled) {
		_ctrl_configuration_handler.setThrottleMin(0.0f);
		_ctrl_configuration_handler.setThrottleMax(0.0f);
		_ctrl_configuration_handler.setSpeedWeight(2.0f);
	}
}

void
FixedWingGuidanceControl::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	_vehicle_status_sub.update(&_vehicle_status);

	/* only run controller if position changed and we are not running an external mode*/

	const bool is_external_nav_state = ((_vehicle_status.nav_state >= vehicle_status_s::NAVIGATION_STATE_EXTERNAL1)
					    && (_vehicle_status.nav_state <= vehicle_status_s::NAVIGATION_STATE_EXTERNAL8))
					   || _vehicle_status.nav_state <= vehicle_status_s::NAVIGATION_STATE_OFFBOARD;

	if (!is_external_nav_state) {
		// this will cause the configuration handler to publish immediately the next time an internal flight
		// mode is active
		_ctrl_configuration_handler.resetLastPublishTime();

	} else if (_local_pos_sub.update(&_local_pos)) {

		const hrt_abstime now = _local_pos.timestamp;

		const float control_interval = math::constrain((now - _last_time_position_control_called) * 1e-6f,
					       MIN_AUTO_TIMESTEP, MAX_AUTO_TIMESTEP);
		_last_time_position_control_called = now;

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
			// adjust navigation waypoints in position control mode
			if (_control_mode.flag_control_altitude_enabled && _control_mode.flag_control_velocity_enabled
			    && _local_pos.xy_reset_counter != _xy_reset_counter) {

				// reset heading hold flag, which will re-initialise position control
				_hdg_hold_enabled = false;
			}
		}

		// Convert Local setpoints to global setpoints
		if (!_global_local_proj_ref.isInitialized()
		    || (_global_local_proj_ref.getProjectionReferenceTimestamp() != _local_pos.ref_timestamp)
		    || (_local_pos.xy_reset_counter != _xy_reset_counter)) {

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
			global_trajectory_setpoint_s global_trajectory_setpoint;

			if (_global_trajectory_setpoint_sub.update(&global_trajectory_setpoint)) {
				bool valid_setpoint = false;
				_pos_sp_triplet = {}; // clear any existing
				_pos_sp_triplet.timestamp = global_trajectory_setpoint.timestamp;
				_pos_sp_triplet.current.timestamp = global_trajectory_setpoint.timestamp;
				_pos_sp_triplet.current.cruising_speed = NAN; // ignored
				_pos_sp_triplet.current.cruising_throttle = NAN; // ignored
				_pos_sp_triplet.current.vx = NAN;
				_pos_sp_triplet.current.vy = NAN;
				_pos_sp_triplet.current.vz = NAN;
				_pos_sp_triplet.current.lat = static_cast<double>(NAN);
				_pos_sp_triplet.current.lon = static_cast<double>(NAN);
				_pos_sp_triplet.current.alt = NAN;

				if (PX4_ISFINITE(global_trajectory_setpoint.lat) && PX4_ISFINITE(global_trajectory_setpoint.lon)
				    && PX4_ISFINITE(global_trajectory_setpoint.alt)) {
					valid_setpoint = true;
					_pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
					_pos_sp_triplet.current.lat = global_trajectory_setpoint.lat;
					_pos_sp_triplet.current.lon = global_trajectory_setpoint.lon;
					_pos_sp_triplet.current.alt = global_trajectory_setpoint.alt;
				}

				if (Vector3f(global_trajectory_setpoint.velocity).isAllFinite()) {
					valid_setpoint = true;
					_pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
					_pos_sp_triplet.current.vx = global_trajectory_setpoint.velocity[0];
					_pos_sp_triplet.current.vy = global_trajectory_setpoint.velocity[1];
					_pos_sp_triplet.current.vz = global_trajectory_setpoint.velocity[2];

					if (Vector3f(global_trajectory_setpoint.acceleration).isAllFinite()) {
						Vector2f velocity_sp_2d(global_trajectory_setpoint.velocity[0], global_trajectory_setpoint.velocity[1]);
						Vector2f normalized_velocity_sp_2d = velocity_sp_2d.normalized();
						Vector2f acceleration_sp_2d(global_trajectory_setpoint.acceleration[0], global_trajectory_setpoint.acceleration[1]);
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
		}

		airspeed_poll();
		manual_control_setpoint_poll();
		vehicle_attitude_poll();
		vehicle_control_mode_poll();
		wind_poll(now);

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.update(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
			}
		}

		if (!_vehicle_status.in_transition_mode) {
			// reset position of backtransition start if not in transition
			_lpos_where_backtrans_started = Vector2f(NAN, NAN);
			_backtrans_heading = NAN;
		}


		Vector2d curr_pos(_current_latitude, _current_longitude);
		Vector2f ground_speed(_local_pos.vx, _local_pos.vy);

		update_in_air_states(now);

		// restore nominal TECS parameters in case changed intermittently (e.g. in landing handling)

		// restore lateral-directional guidance parameters (changed in takeoff mode)
		_directional_guidance.setPeriod(_param_npfg_period.get());

		// by default set speed weight to the param value, can be overwritten inside the methods below
		_ctrl_configuration_handler.setSpeedWeight(_param_t_spdweight.get());


		control_auto_path(control_interval, curr_pos, ground_speed, _pos_sp_triplet.current);

		_ctrl_configuration_handler.update(now);

		// only publish status in full FW mode
		if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
		    || _vehicle_status.in_transition_mode) {
			publish_lateral_guidance_status(now);

		}

		_xy_reset_counter = _local_pos.xy_reset_counter;

		perf_end(_loop_perf);
	}
}


DirectionalGuidanceOutput FixedWingGuidanceControl::navigatePathTangent(const matrix::Vector2f &vehicle_pos,
		const matrix::Vector2f &position_setpoint,
		const matrix::Vector2f &tangent_setpoint,
		const matrix::Vector2f &ground_vel, const matrix::Vector2f &wind_vel, const float &curvature)
{
	if (tangent_setpoint.norm() <= FLT_EPSILON) {
		// degenerate case: no direction. maintain the last npfg command.
		return DirectionalGuidanceOutput{};
	}

	const Vector2f unit_path_tangent{tangent_setpoint.normalized()};
	_closest_point_on_path = position_setpoint;
	return _directional_guidance.guideToPath(vehicle_pos, ground_vel, wind_vel, tangent_setpoint.normalized(),
			position_setpoint,
			curvature);
}

void FixedWingGuidanceControl::publish_lateral_guidance_status(const hrt_abstime now)
{
	fixed_wing_lateral_guidance_status_s fixed_wing_lateral_guidance_status{};

	fixed_wing_lateral_guidance_status.timestamp = now;
	fixed_wing_lateral_guidance_status.course_setpoint = _directional_guidance.getCourseSetpoint();
	fixed_wing_lateral_guidance_status.lateral_acceleration_ff = _directional_guidance.getLateralAccelerationSetpoint();
	fixed_wing_lateral_guidance_status.bearing_feas = _directional_guidance.getBearingFeasibility();
	fixed_wing_lateral_guidance_status.bearing_feas_on_track = _directional_guidance.getBearingFeasibilityOnTrack();
	fixed_wing_lateral_guidance_status.signed_track_error = _directional_guidance.getSignedTrackError();
	fixed_wing_lateral_guidance_status.track_error_bound = _directional_guidance.getTrackErrorBound();
	fixed_wing_lateral_guidance_status.adapted_period = _directional_guidance.getAdaptedPeriod();
	fixed_wing_lateral_guidance_status.wind_est_valid = _wind_valid;

	_fixed_wing_lateral_guidance_status_pub.publish(fixed_wing_lateral_guidance_status);
}

int FixedWingGuidanceControl::task_spawn(int argc, char *argv[])
{
	FixedWingGuidanceControl *instance = new FixedWingGuidanceControl();

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

int FixedWingGuidanceControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FixedWingGuidanceControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the setpoint generation for all PX4-internal fixed-wing modes, height-rate control and higher.
It takes the current mode state of the vehicle as input and outputs setpoints consumed by the fixed-wing
lateral-longitudinal controller and and controllers below that (attitude, rate).

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_mode_manager", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_guidance_control_main(int argc, char *argv[])
{
	return FixedWingGuidanceControl::main(argc, argv);
}
