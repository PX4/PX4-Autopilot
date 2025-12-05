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

	_pos_ctrl_landing_status_pub.advertise();
	_launch_detection_status_pub.advertise();
	_landing_gear_pub.advertise();
	_flaps_setpoint_pub.advertise();
	_spoilers_setpoint_pub.advertise();
	_fixed_wing_lateral_guidance_status_pub.advertise();
	_fixed_wing_runway_control_pub.advertise();

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
FixedWingGuidanceControl::vehicle_command_poll()
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

float
FixedWingGuidanceControl::get_manual_airspeed_setpoint()
{
	float manual_airspeed_setpoint = NAN;

	if (_param_fw_pos_stk_conf.get() & STICK_CONFIG_ENABLE_AIRSPEED_SP_MANUAL_BIT) {
		// neutral throttle corresponds to trim airspeed
		manual_airspeed_setpoint = math::interpolateNXY(_manual_control_setpoint_for_airspeed,
		{-1.f, 0.f, 1.f},
		{_param_fw_airspd_min.get(), _param_fw_airspd_trim.get(), _param_fw_airspd_max.get()});

	} else if (PX4_ISFINITE(_commanded_manual_airspeed_setpoint)) {
		// override stick by commanded airspeed
		manual_airspeed_setpoint = _commanded_manual_airspeed_setpoint;
	}

	return manual_airspeed_setpoint;
}

void
FixedWingGuidanceControl::landing_status_publish()
{
	position_controller_landing_status_s pos_ctrl_landing_status = {};

	pos_ctrl_landing_status.lateral_touchdown_offset = _lateral_touchdown_position_offset;
	pos_ctrl_landing_status.flaring = _flare_states.flaring;
	pos_ctrl_landing_status.abort_status = _landing_abort_status;
	pos_ctrl_landing_status.timestamp = hrt_absolute_time();

	_pos_ctrl_landing_status_pub.publish(pos_ctrl_landing_status);
}

void
FixedWingGuidanceControl::updateLandingAbortStatus(const uint8_t new_abort_status)
{
	// prevent automatic aborts if already flaring, but allow manual aborts
	if (!_flare_states.flaring || new_abort_status == position_controller_landing_status_s::ABORTED_BY_OPERATOR) {

		// only announce changes
		// if (new_abort_status > 0 && _landing_abort_status != new_abort_status) {

		// 	switch (new_abort_status) {
		// 	case (position_controller_landing_status_s::ABORTED_BY_OPERATOR): {
		// 			events::send(events::ID("fixedwing_position_control_landing_abort_status_operator_abort"), events::Log::Critical,
		// 				     "Landing aborted by operator");
		// 			break;
		// 		}

		// 	case (position_controller_landing_status_s::TERRAIN_NOT_FOUND): {
		// 			events::send(events::ID("fixedwing_position_control_landing_abort_status_terrain_not_found"), events::Log::Critical,
		// 				     "Landing aborted: terrain measurement not found");
		// 			break;
		// 		}

		// 	case (position_controller_landing_status_s::TERRAIN_TIMEOUT): {
		// 			events::send(events::ID("fixedwing_position_control_landing_abort_status_terrain_timeout"), events::Log::Critical,
		// 				     "Landing aborted: terrain estimate timed out");
		// 			break;
		// 		}

		// 	default: {
		// 			events::send(events::ID("fixedwing_position_control_landing_abort_status_unknown_criterion"), events::Log::Critical,
		// 				     "Landing aborted: unknown criterion");
		// 		}
		// 	}
		// }

		_landing_abort_status = (new_abort_status >= position_controller_landing_status_s::UNKNOWN_ABORT_CRITERION) ?
					position_controller_landing_status_s::UNKNOWN_ABORT_CRITERION : new_abort_status;
		landing_status_publish();
	}
}

float
FixedWingGuidanceControl::getManualHeightRateSetpoint()
{
	float height_rate_setpoint = 0.f;

	if (_manual_control_setpoint_for_height_rate >= FLT_EPSILON) {
		height_rate_setpoint = math::interpolate<float>(math::deadzone(_manual_control_setpoint_for_height_rate,
				       kStickDeadBand), 0, 1.f, 0.f, -_param_sinkrate_target.get());

	} else {
		height_rate_setpoint = math::interpolate<float>(math::deadzone(_manual_control_setpoint_for_height_rate,
				       kStickDeadBand), -1., 0.f, _param_climbrate_target.get(), 0.f);
	}

	return height_rate_setpoint;
}

void
FixedWingGuidanceControl::updateManualTakeoffStatus()
{
	if (!_completed_manual_takeoff) {
		const bool at_controllable_airspeed = _airspeed_eas > _param_fw_airspd_min.get()
						      || !_airspeed_valid;
		const bool is_hovering = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
					 && _control_mode.flag_armed;
		_completed_manual_takeoff = (!_landed && at_controllable_airspeed) || is_hovering;
	}
}

// void
// FixedWingGuidanceControl::set_control_mode_current(const hrt_abstime &now)
// {
// 	/* only run position controller in fixed-wing mode and during transitions for VTOL */
// 	if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !_vehicle_status.in_transition_mode) {
// 		_control_mode_current = FW_POSCTRL_MODE_OTHER;
// 		return; // do not publish the setpoint
// 	}

// 	const FW_POSCTRL_MODE previous_position_control_mode = _control_mode_current;

// 	_skipping_takeoff_detection = false;
// 	const bool doing_backtransition = _vehicle_status.in_transition_mode && !_vehicle_status.in_transition_to_fw;

// 	if (_control_mode.flag_control_offboard_enabled && _position_setpoint_current_valid
// 	    && _control_mode.flag_control_position_enabled) {
// 		if (PX4_ISFINITE(_pos_sp_triplet.current.vx) && PX4_ISFINITE(_pos_sp_triplet.current.vy)
// 		    && PX4_ISFINITE(_pos_sp_triplet.current.vz)) {
// 			// Offboard position with velocity setpoints
// 			_control_mode_current = FW_POSCTRL_MODE_AUTO_PATH;
// 			return;

// 		} else {
// 			// Offboard position setpoint only
// 			_control_mode_current = FW_POSCTRL_MODE_AUTO;
// 			return;
// 		}

// 	} else if ((_control_mode.flag_control_auto_enabled && _control_mode.flag_control_position_enabled)
// 		   && (_position_setpoint_current_valid
// 		       || _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE)) {

// 		// Enter this mode only if the current waypoint has valid 3D position setpoints or is of type IDLE.
// 		// A setpoint of type IDLE can be published by Navigator without a valid position, and is handled here in FW_POSCTRL_MODE_AUTO.

// 		if (doing_backtransition) {
// 			_control_mode_current = FW_POSCTRL_MODE_TRANSITION_TO_HOVER_LINE_FOLLOW;

// 		} else if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {

// 			if (_vehicle_status.is_vtol && _vehicle_status.in_transition_mode) {
// 				_control_mode_current = FW_POSCTRL_MODE_AUTO;

// 				// in this case we want the waypoint handled as a position setpoint -- a submode in control_auto()
// 				_pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;

// 			} else {
// 				_control_mode_current = _local_pos.xy_valid ? FW_POSCTRL_MODE_AUTO_TAKEOFF : FW_POSCTRL_MODE_AUTO_TAKEOFF_NO_NAV;

// 				if (previous_position_control_mode != FW_POSCTRL_MODE_AUTO_TAKEOFF_NO_NAV
// 				    && previous_position_control_mode != FW_POSCTRL_MODE_AUTO_TAKEOFF && !_landed) {
// 					// skip takeoff detection when switching from any other mode, auto or manual,
// 					// while already in air.
// 					// TODO: find a better place for / way of doing this
// 					_skipping_takeoff_detection = true;
// 				}
// 			}

// 		} else if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {

// 			// Use _position_setpoint_previous_valid to determine if landing should be straight or circular.
// 			// Straight landings are currently only possible in Missions, and there the previous WP
// 			// is valid, and circular ones are used outside of Missions, as the land mode sets prev_valid=false.
// 			if (_position_setpoint_previous_valid) {
// 				_control_mode_current = FW_POSCTRL_MODE_AUTO_LANDING_STRAIGHT;

// 			} else {
// 				_control_mode_current = FW_POSCTRL_MODE_AUTO_LANDING_CIRCULAR;
// 			}

// 		} else {
// 			_control_mode_current = FW_POSCTRL_MODE_AUTO;
// 		}

// 	} else if (_control_mode.flag_control_auto_enabled
// 		   && _control_mode.flag_control_climb_rate_enabled
// 		   && _control_mode.flag_armed // only enter this modes if armed, as pure failsafe modes
// 		   && !_control_mode.flag_control_position_enabled) {

// 		// failsafe modes engaged if position estimate is invalidated

// 		if (previous_position_control_mode != FW_POSCTRL_MODE_AUTO_ALTITUDE
// 		    && previous_position_control_mode != FW_POSCTRL_MODE_AUTO_CLIMBRATE) {
// 			// reset timer the first time we switch into this mode
// 			_time_in_fixed_bank_loiter = now;
// 		}

// 		if (doing_backtransition) {
// 			// we handle loss of position control during backtransition as a special case
// 			_control_mode_current = FW_POSCTRL_MODE_TRANSITION_TO_HOVER_HEADING_HOLD;

// 		} else if (hrt_elapsed_time(&_time_in_fixed_bank_loiter) < (_param_nav_gpsf_lt.get() * 1_s)
// 			   && !_vehicle_status.in_transition_mode) {
// 			if (previous_position_control_mode != FW_POSCTRL_MODE_AUTO_ALTITUDE) {
// 				// Need to init because last loop iteration was in a different mode
// 				events::send(events::ID("fixedwing_position_control_fb_loiter"), events::Log::Critical,
// 					     "Start loiter with fixed bank angle");
// 			}

// 			_control_mode_current = FW_POSCTRL_MODE_AUTO_ALTITUDE;

// 		} else {
// 			if (previous_position_control_mode != FW_POSCTRL_MODE_AUTO_CLIMBRATE && !_vehicle_status.in_transition_mode) {
// 				events::send(events::ID("fixedwing_position_control_descend"), events::Log::Critical, "Start descending");
// 			}

// 			_control_mode_current = FW_POSCTRL_MODE_AUTO_CLIMBRATE;
// 		}


// 	} else if (_control_mode.flag_control_manual_enabled && _control_mode.flag_control_position_enabled) {
// 		if (previous_position_control_mode != FW_POSCTRL_MODE_MANUAL_POSITION) {
// 			/* Need to init because last loop iteration was in a different mode */
// 			_hdg_hold_yaw = _yaw; // yaw is not controlled, so set setpoint to current yaw
// 			_hdg_hold_enabled = false; // this makes sure the waypoints are reset below
// 			_yaw_lock_engaged = false;
// 		}

// 		_control_mode_current = FW_POSCTRL_MODE_MANUAL_POSITION;

// 	} else if (_control_mode.flag_control_manual_enabled && _control_mode.flag_control_altitude_enabled) {

// 		_control_mode_current = FW_POSCTRL_MODE_MANUAL_ALTITUDE;

// 	} else {
// 		_control_mode_current = FW_POSCTRL_MODE_OTHER;
// 	}
// }

void
FixedWingGuidanceControl::update_in_air_states(const hrt_abstime now)
{
	/* reset flag when airplane landed */
	if (_landed) {
		_completed_manual_takeoff = false;
	}
}

void
FixedWingGuidanceControl::move_position_setpoint_for_vtol_transition(position_setpoint_s &current_sp)
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

void FixedWingGuidanceControl::control_idle()
{
	const hrt_abstime  now = hrt_absolute_time();
	fixed_wing_lateral_setpoint_s lateral_ctrl_sp {empty_lateral_control_setpoint};
	lateral_ctrl_sp.timestamp = now;
	lateral_ctrl_sp.lateral_acceleration = 0.0f;
	_lateral_ctrl_sp_pub.publish(lateral_ctrl_sp);

	fixed_wing_longitudinal_setpoint_s long_contrl_sp {empty_longitudinal_control_setpoint};
	long_contrl_sp.timestamp = now;
	long_contrl_sp.pitch_direct = 0.f;
	long_contrl_sp.throttle_direct = 0.0f;
	_longitudinal_ctrl_sp_pub.publish(long_contrl_sp);

	_ctrl_configuration_handler.setThrottleMax(0.0f);
	_ctrl_configuration_handler.setThrottleMin(0.0f);
}

uint8_t
FixedWingGuidanceControl::handle_setpoint_type(const position_setpoint_s &pos_sp_curr,
		const position_setpoint_s &pos_sp_next)
{
	uint8_t position_sp_type = pos_sp_curr.type;

	if (!_control_mode.flag_control_position_enabled && _control_mode.flag_control_velocity_enabled) {
		return position_setpoint_s::SETPOINT_TYPE_VELOCITY;
	}

	Vector2d curr_wp{0, 0};

	/* current waypoint (the one currently heading for) */
	curr_wp = Vector2d(pos_sp_curr.lat, pos_sp_curr.lon);

	const float acc_rad = _directional_guidance.switchDistance(500.0f);

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

		const float acc_rad_z = (PX4_ISFINITE(pos_sp_curr.alt_acceptance_radius)
					 && pos_sp_curr.alt_acceptance_radius > FLT_EPSILON) ? pos_sp_curr.alt_acceptance_radius :
					_param_nav_fw_alt_rad.get();

		// Achieve position setpoint altitude via loiter when laterally close to WP.
		// Detect if system has switchted into a Loiter before (check _position_sp_type), and in that
		// case remove the dist_xy check (not switch out of Loiter until altitude is reached).
		if ((!_vehicle_status.in_transition_mode) && (dist >= 0.f)
		    && (dist_z > acc_rad_z)
		    && (dist_xy < acc_rad || _position_sp_type == position_setpoint_s::SETPOINT_TYPE_LOITER)) {

			// SETPOINT_TYPE_POSITION -> SETPOINT_TYPE_LOITER
			position_sp_type = position_setpoint_s::SETPOINT_TYPE_LOITER;
		}
	}

	return position_sp_type;
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

float FixedWingGuidanceControl::rollAngleToLateralAccel(float roll_body) const
{
	return tanf(roll_body) * CONSTANTS_ONE_G;
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

	const bool is_external_nav_state = (_vehicle_status.nav_state >= vehicle_status_s::NAVIGATION_STATE_EXTERNAL1)
					   && (_vehicle_status.nav_state <= vehicle_status_s::NAVIGATION_STATE_EXTERNAL8);

	if (is_external_nav_state) {
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

		}

		airspeed_poll();
		manual_control_setpoint_poll();
		vehicle_attitude_poll();
		vehicle_command_poll();
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

		// set_control_mode_current(now);

		update_in_air_states(now);

		// restore nominal TECS parameters in case changed intermittently (e.g. in landing handling)

		// restore lateral-directional guidance parameters (changed in takeoff mode)
		_directional_guidance.setPeriod(_param_npfg_period.get());

		// by default no flaps/spoilers, is overwritten below in certain modes
		_flaps_setpoint = 0.f;
		_spoilers_setpoint = 0.f;

		// by default set speed weight to the param value, can be overwritten inside the methods below
		_ctrl_configuration_handler.setSpeedWeight(_param_t_spdweight.get());

		if (_control_mode_current != FW_POSCTRL_MODE_AUTO_LANDING_STRAIGHT
		    && _control_mode_current != FW_POSCTRL_MODE_AUTO_LANDING_CIRCULAR) {
			reset_landing_state();
		}

		if (_control_mode_current != FW_POSCTRL_MODE_AUTO_TAKEOFF
		    && _control_mode_current != FW_POSCTRL_MODE_AUTO_TAKEOFF_NO_NAV) {
			reset_takeoff_state();
		}

		int8_t old_landing_gear_position = _new_landing_gear_position;
		_new_landing_gear_position = landing_gear_s::GEAR_KEEP; // is overwritten in Takeoff and Land

		if (_control_mode_current == FW_POSCTRL_MODE_AUTO_PATH) {
				control_auto_path(control_interval, curr_pos, ground_speed, _pos_sp_triplet.current);
		}

		if (_control_mode_current != FW_POSCTRL_MODE_OTHER) {
			_ctrl_configuration_handler.update(now);
		}

		// only publish status in full FW mode
		if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
		    || _vehicle_status.in_transition_mode) {
			publish_lateral_guidance_status(now);

		}

		// if there's any change in landing gear setpoint publish it
		if (_new_landing_gear_position != old_landing_gear_position
		    && _new_landing_gear_position != landing_gear_s::GEAR_KEEP) {

			landing_gear_s landing_gear = {};
			landing_gear.landing_gear = _new_landing_gear_position;
			landing_gear.timestamp = now;
			_landing_gear_pub.publish(landing_gear);
		}

		// In Manual modes flaps and spoilers are directly controlled in the Attitude controller and not published here
		if (_control_mode.flag_control_auto_enabled
		    && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
			normalized_unsigned_setpoint_s flaps_setpoint;
			flaps_setpoint.normalized_setpoint = _flaps_setpoint;
			flaps_setpoint.timestamp = now;
			_flaps_setpoint_pub.publish(flaps_setpoint);

			normalized_unsigned_setpoint_s spoilers_setpoint;
			spoilers_setpoint.normalized_setpoint = _spoilers_setpoint;
			spoilers_setpoint.timestamp = now;
			_spoilers_setpoint_pub.publish(spoilers_setpoint);
		}

		_xy_reset_counter = _local_pos.xy_reset_counter;

		perf_end(_loop_perf);
	}
}

void
FixedWingGuidanceControl::reset_takeoff_state()
{
	_launch_detected = false;

	_takeoff_ground_alt = _current_altitude;
}

void
FixedWingGuidanceControl::reset_landing_state()
{
	_time_started_landing = 0;

	_flare_states = FlareStates{};

	_lateral_touchdown_position_offset = 0.0f;

	_last_time_terrain_alt_was_valid = 0;

	// reset abort land, unless loitering after an abort
	if ((_landing_abort_status && (_pos_sp_triplet.current.type != position_setpoint_s::SETPOINT_TYPE_LOITER)) ||
	    (_landing_abort_status && _param_fw_lnd_abort.get() == 0)) {

		updateLandingAbortStatus(position_controller_landing_status_s::NOT_ABORTED);
	}
}

float FixedWingGuidanceControl::getMaxRollAngleNearGround(const float altitude, const float terrain_altitude) const
{
	// we want the wings level when at the wing height above ground
	const float height_above_ground = math::max(altitude - (terrain_altitude + _param_fw_wing_height.get()), 0.0f);

	// this is a conservative (linear) approximation of the roll angle that would cause wing tip strike
	// roll strike = arcsin( 2 * height / span )
	// d(roll strike)/d(height) = 2 / span / cos(2 * height / span)
	// d(roll strike)/d(height) (@height=0) = 2 / span
	// roll strike ~= 2 * height / span

	return  math::constrain(2.f * height_above_ground / _param_fw_wing_span.get(), 0.f,
				math::radians(_param_fw_r_lim.get()));
}


void
FixedWingGuidanceControl::initializeAutoLanding(const hrt_abstime &now, const position_setpoint_s &pos_sp_prev,
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
FixedWingGuidanceControl::calculateTouchdownPosition(const float control_interval, const Vector2f &local_land_position)
{
	if (fabsf(_sticks.getYaw()) > MANUAL_TOUCHDOWN_NUDGE_INPUT_DEADZONE
	    && _param_fw_lnd_nudge.get() > LandingNudgingOption::kNudgingDisabled
	    && !_flare_states.flaring) {
		// laterally nudge touchdown location with yaw stick
		// positive is defined in the direction of a right hand turn starting from the approach vector direction
		const float signed_deadzone_threshold = MANUAL_TOUCHDOWN_NUDGE_INPUT_DEADZONE * math::signNoZero(
				_sticks.getYaw());
		_lateral_touchdown_position_offset += (_sticks.getYaw() - signed_deadzone_threshold) *
						      MAX_TOUCHDOWN_POSITION_NUDGE_RATE * control_interval;
		_lateral_touchdown_position_offset = math::constrain(_lateral_touchdown_position_offset, -_param_fw_lnd_td_off.get(),
						     _param_fw_lnd_td_off.get());
	}

	const Vector2f approach_unit_vector = -_landing_approach_entrance_offset_vector.unit_or_zero();
	const Vector2f approach_unit_normal_vector{-approach_unit_vector(1), approach_unit_vector(0)};

	return local_land_position + approach_unit_normal_vector * _lateral_touchdown_position_offset;
}

Vector2f
FixedWingGuidanceControl::calculateLandingApproachVector() const
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
FixedWingGuidanceControl::getLandingTerrainAltitudeEstimate(const hrt_abstime &now, const float land_point_altitude,
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

bool FixedWingGuidanceControl::checkLandingAbortBitMask(const uint8_t automatic_abort_criteria_bitmask,
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

void FixedWingGuidanceControl::publishLocalPositionSetpoint(const position_setpoint_s &current_waypoint)
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
	_local_pos_sp_pub.publish(local_position_setpoint);
}

void FixedWingGuidanceControl::publishOrbitStatus(const position_setpoint_s pos_sp)
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

DirectionalGuidanceOutput FixedWingGuidanceControl::navigateWaypoints(const Vector2f &start_waypoint,
		const Vector2f &end_waypoint,
		const Vector2f &vehicle_pos, const Vector2f &ground_vel, const Vector2f &wind_vel)
{
	const Vector2f start_waypoint_to_end_waypoint = end_waypoint - start_waypoint;
	const Vector2f start_waypoint_to_vehicle = vehicle_pos - start_waypoint;
	const Vector2f end_waypoint_to_vehicle = vehicle_pos - end_waypoint;

	if (start_waypoint_to_end_waypoint.norm() < FLT_EPSILON) {
		// degenerate case: the waypoints are on top of each other, this should only happen when someone uses this
		// method incorrectly. just as a safe guard, call the singular waypoint navigation method.
		return navigateWaypoint(end_waypoint, vehicle_pos, ground_vel, wind_vel);
	}

	if ((start_waypoint_to_end_waypoint.dot(start_waypoint_to_vehicle) < -FLT_EPSILON)
	    && (start_waypoint_to_vehicle.norm() > _directional_guidance.switchDistance(500.0f))) {
		// we are in front of the start waypoint, fly directly to it until we are within switch distance
		return navigateWaypoint(start_waypoint, vehicle_pos, ground_vel, wind_vel);
	}

	if (start_waypoint_to_end_waypoint.dot(end_waypoint_to_vehicle) > FLT_EPSILON) {
		// we are beyond the end waypoint, fly back to it
		// NOTE: this logic ideally never gets executed, as a waypoint switch should happen before passing the
		// end waypoint. however this included here as a safety precaution if any navigator (module) switch condition
		// is missed for any reason. in the future this logic should all be handled in one place in a dedicated
		// flight mode state machine.
		return navigateWaypoint(end_waypoint, vehicle_pos, ground_vel, wind_vel);
	}

	// follow the line segment between the start and end waypoints
	return navigateLine(start_waypoint, end_waypoint, vehicle_pos, ground_vel, wind_vel);
}

DirectionalGuidanceOutput FixedWingGuidanceControl::navigateWaypoint(const Vector2f &waypoint_pos,
		const Vector2f &vehicle_pos,
		const Vector2f &ground_vel, const Vector2f &wind_vel)
{
	const Vector2f vehicle_to_waypoint = waypoint_pos - vehicle_pos;

	if (vehicle_to_waypoint.norm() < FLT_EPSILON) {
		// degenerate case: the vehicle is on top of the single waypoint. (can happen). maintain the last npfg command.
		return DirectionalGuidanceOutput{};
	}

	const Vector2f unit_path_tangent = vehicle_to_waypoint.normalized();
	_closest_point_on_path = waypoint_pos;

	const float path_curvature = 0.f;
	DirectionalGuidanceOutput sp = _directional_guidance.guideToPath(vehicle_pos, ground_vel, wind_vel, unit_path_tangent,
				       _closest_point_on_path, path_curvature);

	return sp;
}

DirectionalGuidanceOutput FixedWingGuidanceControl::navigateLine(const Vector2f &point_on_line_1,
		const Vector2f &point_on_line_2,
		const Vector2f &vehicle_pos, const Vector2f &ground_vel, const Vector2f &wind_vel)
{
	const Vector2f line_segment = point_on_line_2 - point_on_line_1;

	if (line_segment.norm() <= FLT_EPSILON) {
		// degenerate case: line segment has zero length. maintain the last npfg command.
		return DirectionalGuidanceOutput{};
	}

	const Vector2f unit_path_tangent = line_segment.normalized();

	const Vector2f point_1_to_vehicle = vehicle_pos - point_on_line_1;
	_closest_point_on_path = point_on_line_1 + point_1_to_vehicle.dot(unit_path_tangent) * unit_path_tangent;

	const float path_curvature = 0.f;
	const DirectionalGuidanceOutput sp = _directional_guidance.guideToPath(vehicle_pos, ground_vel, wind_vel,
					     unit_path_tangent,
					     _closest_point_on_path, path_curvature);

	return sp;
}

DirectionalGuidanceOutput FixedWingGuidanceControl::navigateLine(const Vector2f &point_on_line,
		const float line_bearing,
		const Vector2f &vehicle_pos, const Vector2f &ground_vel, const Vector2f &wind_vel)
{
	const Vector2f unit_path_tangent{cosf(line_bearing), sinf(line_bearing)};

	const Vector2f point_on_line_to_vehicle = vehicle_pos - point_on_line;
	_closest_point_on_path = point_on_line + point_on_line_to_vehicle.dot(unit_path_tangent) * unit_path_tangent;

	const float path_curvature = 0.f;
	const DirectionalGuidanceOutput sp = _directional_guidance.guideToPath(vehicle_pos, ground_vel, wind_vel,
					     unit_path_tangent,
					     _closest_point_on_path, path_curvature);

	return sp;
}

DirectionalGuidanceOutput FixedWingGuidanceControl::navigateLoiter(const Vector2f &loiter_center,
		const Vector2f &vehicle_pos,
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

	const float path_curvature = loiter_direction_multiplier / radius;
	_closest_point_on_path = unit_vec_center_to_closest_pt * radius + loiter_center;
	return _directional_guidance.guideToPath(vehicle_pos, ground_vel, wind_vel, unit_path_tangent,
			loiter_center + unit_vec_center_to_closest_pt * radius, path_curvature);
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

DirectionalGuidanceOutput FixedWingGuidanceControl::navigateBearing(const matrix::Vector2f &vehicle_pos, float bearing,
		const Vector2f &ground_vel, const Vector2f &wind_vel)
{
	const Vector2f unit_path_tangent = Vector2f{cosf(bearing), sinf(bearing)};
	_closest_point_on_path = vehicle_pos;
	return _directional_guidance.guideToPath(vehicle_pos, ground_vel, wind_vel, unit_path_tangent, vehicle_pos, 0.0f);
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
