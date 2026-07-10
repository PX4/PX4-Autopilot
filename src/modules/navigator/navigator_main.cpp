/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file navigator_main.cpp
 *
 * Handles mission items, geo fencing and failsafe navigation behavior.
 * Published the position setpoint triplet for the position controller.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Jean Cyr <jean.m.cyr@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * and many more...
 */

#include "navigator.h"

#include <float.h>
#include <sys/stat.h>

#include <dataman_client/DatamanClient.hpp>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/events.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <systemlib/mavlink_log.h>

using namespace time_literals;

namespace navigator
{
Navigator *g_navigator;
}

#if CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC
namespace
{
int print_fake_traffic_usage(const char *reason = nullptr)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PX4_INFO_RAW("Usage:\n");
	PX4_INFO_RAW("  navigator fake_traffic [mode]\n");
	PX4_INFO_RAW("  navigator fake_traffic stop\n");
	PX4_INFO_RAW("  navigator fake_traffic help\n");
	PX4_INFO_RAW("\n");
	PX4_INFO_RAW("Modes:\n");
	PX4_INFO_RAW("  unique_ids   identifier fallback checks\n");
	PX4_INFO_RAW("  escalation   one target getting closer\n");
	PX4_INFO_RAW("  spam_same    repeated updates for one target\n");
	PX4_INFO_RAW("  spam_new     many new targets\n");
	PX4_INFO_RAW("  flags        invalid velocity-flag case\n");
	PX4_INFO_RAW("  queue_fill   3 batches of ORB_QUEUE_LENGTH\n");
	PX4_INFO_RAW("\n");
	PX4_INFO_RAW("Warning: fake traffic runs through the normal DAA pipeline and can trigger configured actions.\n");
	return 0;
}

bool parse_fake_traffic_mode(const char *mode_name, DetectAndAvoid::FakeTraffMode &mode)
{
	if (!strcmp(mode_name, "unique_ids")) {
		mode = DetectAndAvoid::FakeTraffMode::kUniqueIds;
		return true;
	}

	if (!strcmp(mode_name, "escalation")) {
		mode = DetectAndAvoid::FakeTraffMode::kEscalation;
		return true;
	}

	if (!strcmp(mode_name, "spam_same")) {
		mode = DetectAndAvoid::FakeTraffMode::kSpamSame;
		return true;
	}

	if (!strcmp(mode_name, "spam_new")) {
		mode = DetectAndAvoid::FakeTraffMode::kSpamNew;
		return true;
	}

	if (!strcmp(mode_name, "flags")) {
		mode = DetectAndAvoid::FakeTraffMode::kFlags;
		return true;
	}

	if (!strcmp(mode_name, "queue_fill")) {
		mode = DetectAndAvoid::FakeTraffMode::kQueueFill;
		return true;
	}

	return false;
}
} // namespace
#endif // CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC

Navigator::Navigator() :
	ModuleParams(nullptr),
	_loop_perf(perf_alloc(PC_ELAPSED, "navigator")),
	_geofence(this),
	_mission(this),
	_loiter(this),
	_takeoff(this),
#if CONFIG_MODE_NAVIGATOR_VTOL_TAKEOFF
	_vtol_takeoff(this),
#endif //CONFIG_MODE_NAVIGATOR_VTOL_TAKEOFF
	_land(this),
	_precland(this),
	_rtl(this),
	_course(this)
#if CONFIG_NAVIGATOR_ADSB
	, _detect_and_avoid(this)
#endif // CONFIG_NAVIGATOR_ADSB
{
	/* Create a list of our possible navigation types */
	_navigation_mode_array[0] = &_mission;
	_navigation_mode_array[1] = &_loiter;
	_navigation_mode_array[2] = &_rtl;
	_navigation_mode_array[3] = &_takeoff;
	_navigation_mode_array[4] = &_land;
	_navigation_mode_array[5] = &_precland;
#if CONFIG_MODE_NAVIGATOR_VTOL_TAKEOFF
	_navigation_mode_array[6] = &_vtol_takeoff;
#endif //CONFIG_MODE_NAVIGATOR_VTOL_TAKEOFF
	_navigation_mode_array[7] = &_course;

	/* iterate through navigation modes and initialize _mission_item for each */
	for (unsigned int i = 0; i < NAVIGATOR_MODE_ARRAY_SIZE; i++) {
		if (_navigation_mode_array[i]) {
			_navigation_mode_array[i]->initialize();
		}
	}

	_handle_back_trans_dec_mss = param_find("VT_B_DEC_MSS");

	_handle_mpc_jerk_auto = param_find("MPC_JERK_AUTO");
	_handle_mpc_acc_hor = param_find("MPC_ACC_HOR");

	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_mission_sub = orb_subscribe(ORB_ID(mission));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	_distance_sensor_mode_change_request_pub.advertise();
	_distance_sensor_mode_change_request_pub.get().timestamp = hrt_absolute_time();
	_distance_sensor_mode_change_request_pub.get().request_on_off = distance_sensor_mode_change_request_s::REQUEST_OFF;
	_distance_sensor_mode_change_request_pub.update();

#if CONFIG_NAVIGATOR_ADSB
	_detect_and_avoid.on_activation();
#endif // CONFIG_NAVIGATOR_ADSB

	reset_triplets();
}

Navigator::~Navigator()
{
	perf_free(_loop_perf);
	orb_unsubscribe(_local_pos_sub);
	orb_unsubscribe(_mission_sub);
	orb_unsubscribe(_vehicle_status_sub);
#if CONFIG_NAVIGATOR_ADSB
	_detect_and_avoid.on_inactivation();
#endif // CONFIG_NAVIGATOR_ADSB
}

void Navigator::params_update()
{
	updateParams();

	if (_handle_back_trans_dec_mss != PARAM_INVALID) {
		param_get(_handle_back_trans_dec_mss, &_param_back_trans_dec_mss);
	}

	if (_handle_mpc_jerk_auto != PARAM_INVALID) {
		param_get(_handle_mpc_jerk_auto, &_param_mpc_jerk_auto);
	}

	if (_handle_mpc_acc_hor != PARAM_INVALID) {
		param_get(_handle_mpc_acc_hor, &_param_mpc_acc_hor);
	}

	_mission.set_command_timeout(_param_mis_command_tout.get());
}

void Navigator::run()
{

	/* Try to load the geofence file if it exists. */
	struct stat buffer;

	if (stat(GEOFENCE_FILENAME, &buffer) == 0) {
		PX4_INFO("Loading geofence from %s", GEOFENCE_FILENAME);
		_geofence.loadFromFile(GEOFENCE_FILENAME);
	}

	params_update();

	/* wakeup source(s) */
	px4_pollfd_struct_t fds[3] {};

	/* Setup of loop */
	fds[0].fd = _local_pos_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _vehicle_status_sub;
	fds[1].events = POLLIN;
	fds[2].fd = _mission_sub;
	fds[2].events = POLLIN;

	uint32_t geofence_id{0};
	uint32_t safe_points_id{0};

	/* rate-limit position subscription to 20 Hz / 50 ms */
	orb_set_interval(_local_pos_sub, 50);

	while (!should_exit()) {

		/* wait for up to 1000ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			/* Let the loop run anyway, don't do `continue` here. */

		} else if (pret < 0) {
			/* this is undesirable but not much we can do - might want to flag unhappy status */
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(10000);
			continue;
		}

		perf_begin(_loop_perf);

		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vstatus);

		if (fds[2].revents & POLLIN) {
			mission_s mission;
			orb_copy(ORB_ID(mission), _mission_sub, &mission);

			if (mission.geofence_id != geofence_id) {
				geofence_id = mission.geofence_id;
				_geofence.updateFence();
			}

			if (mission.safe_points_id != safe_points_id) {
				safe_points_id = mission.safe_points_id;
				_rtl.updateSafePoints(safe_points_id);
			}
		}

		/* gps updated */
		if (_gps_pos_sub.updated()) {
			_gps_pos_sub.copy(&_gps_pos);
		}

		/* global position updated */
		if (_global_pos_sub.updated()) {
			_global_pos_sub.copy(&_global_pos);
		}

		/* update last known position with GCS heartbeat */
		for (auto &telemetry_sub : _telemetry_status_subs) {
			telemetry_status_s telemetry;

			if (telemetry_sub.update(&telemetry) && telemetry.heartbeat_type_gcs) {
				_last_pos_with_gcs_heartbeat.lat = _global_pos.lat;
				_last_pos_with_gcs_heartbeat.lon = _global_pos.lon;
				_last_pos_with_gcs_heartbeat.alt = _global_pos.alt;
				_last_pos_with_gcs_heartbeat.yaw = _local_pos.heading;
				break;
			}
		}

		/* check for parameter updates */
		if (_parameter_update_sub.updated()) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			params_update();
		}

		_land_detected_sub.update(&_land_detected);
		_position_controller_status_sub.update();
		_fw_lateral_guidance_status_sub.update();
		_takeoff_status_sub.update();
		_home_pos_sub.update(&_home_pos);

		// Handle Vehicle commands
		int vehicle_command_updates = 0;

		while (_wait_for_vehicle_status_timestamp == 0 && _vehicle_command_sub.updated()
		       && (vehicle_command_updates < vehicle_command_s::ORB_QUEUE_LENGTH)) {
			vehicle_command_updates++;
			const unsigned last_generation = _vehicle_command_sub.get_last_generation();

			vehicle_command_s cmd{};
			_vehicle_command_sub.copy(&cmd);

			if (_vehicle_command_sub.get_last_generation() != last_generation + 1) {
				PX4_ERR("vehicle_command lost, generation %d -> %d", last_generation, _vehicle_command_sub.get_last_generation());
			}

			if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_GO_AROUND) {

				// DO_GO_AROUND is currently handled by the position controller (unacknowledged)
				// TODO: move DO_GO_AROUND handling to navigator
				publish_vehicle_command_ack(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_REPOSITION
				   && _vstatus.arming_state == vehicle_status_s::ARMING_STATE_ARMED
				   && (((uint32_t)cmd.param2 & 1) != 0
				       || _vstatus.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER)) {
				// Only apply the reposition setpoint when armed and either a mode switch into Hold was requested
				// (CHANGE_MODE flag) or we're already in Hold. Otherwise a later switch into Hold could execute a
				// stale setpoint (loiter.cpp applies it within a 500ms window).

				// Wait for vehicle_status before handling the next command, otherwise the setpoint could be overwritten
				_wait_for_vehicle_status_timestamp = hrt_absolute_time();

				vehicle_global_position_s position_setpoint{};

				if (PX4_ISFINITE(cmd.param5) && PX4_ISFINITE(cmd.param6)) {
					position_setpoint.lat = cmd.param5;
					position_setpoint.lon = cmd.param6;

				} else {
					position_setpoint.lat = get_global_position()->lat;
					position_setpoint.lon = get_global_position()->lon;
				}

				position_setpoint.alt = PX4_ISFINITE(cmd.param7) ? cmd.param7 : get_global_position()->alt;

				if (geofence_allows_position(position_setpoint)) {
					position_setpoint_triplet_s *rep = get_reposition_triplet();
					position_setpoint_triplet_s *curr = get_position_setpoint_triplet();

					// store current position as previous position and goal as next
					rep->previous.yaw = get_local_position()->heading;
					rep->previous.lat = get_global_position()->lat;
					rep->previous.lon = get_global_position()->lon;
					rep->previous.alt = get_global_position()->alt;


					rep->current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;

					bool only_alt_change_requested = false;

					// If no argument for ground speed, use default value.
					if (cmd.param1 <= 0 || !PX4_ISFINITE(cmd.param1)) {
						// on entering Loiter mode, reset speed setpoint to default
						if (_navigation_mode != &_loiter) {
							rep->current.cruising_speed = -1.f;

						} else {
							rep->current.cruising_speed = get_cruising_speed();
						}

					} else {
						rep->current.cruising_speed = cmd.param1;
					}

					rep->current.cruising_throttle = get_cruising_throttle();
					rep->current.acceptance_radius = get_acceptance_radius();

					// Go on and check which changes had been requested
					if (PX4_ISFINITE(cmd.param4)) {
						rep->current.yaw = cmd.param4;

					} else {
						rep->current.yaw = NAN;
					}

					if (PX4_ISFINITE(cmd.param5) && PX4_ISFINITE(cmd.param6)) {
						// Position change with optional altitude change
						rep->current.lat = cmd.param5;
						rep->current.lon = cmd.param6;

						if (PX4_ISFINITE(cmd.param7)) {
							rep->current.alt = cmd.param7;

						} else {
							rep->current.alt = get_global_position()->alt;
						}

					} else if (PX4_ISFINITE(cmd.param7) || PX4_ISFINITE(cmd.param4)) {
						// Position is not changing, thus we keep the setpoint
						rep->current.lat = PX4_ISFINITE(curr->current.lat) ? curr->current.lat : get_global_position()->lat;
						rep->current.lon = PX4_ISFINITE(curr->current.lon) ? curr->current.lon : get_global_position()->lon;

						if (PX4_ISFINITE(cmd.param7)) {
							rep->current.alt = cmd.param7;
							only_alt_change_requested = true;

						} else {
							rep->current.alt = get_global_position()->alt;
						}

					} else {
						// All three set to NaN - pause vehicle
						rep->current.alt = get_global_position()->alt;

						if (_vstatus.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
						    && (get_position_setpoint_triplet()->current.type != position_setpoint_s::SETPOINT_TYPE_TAKEOFF)) {

							preproject_stop_point(rep->current.lat, rep->current.lon);

						} else {
							// For fixedwings we can use the current vehicle's position to define the loiter point
							rep->current.lat = get_global_position()->lat;
							rep->current.lon = get_global_position()->lon;
						}
					}

					if (only_alt_change_requested) {
						if (PX4_ISFINITE(curr->current.loiter_radius) && curr->current.loiter_radius > FLT_EPSILON) {
							rep->current.loiter_radius = curr->current.loiter_radius;


						} else {
							rep->current.loiter_radius = get_default_loiter_rad();
						}

						if (PX4_ISFINITE(curr->current.loiter_minor_radius) && fabsf(curr->current.loiter_minor_radius) > FLT_EPSILON) {
							rep->current.loiter_minor_radius = curr->current.loiter_minor_radius;

						} else {
							rep->current.loiter_minor_radius = NAN;
						}

						if (PX4_ISFINITE(curr->current.loiter_orientation) && fabsf(curr->current.loiter_minor_radius) > FLT_EPSILON) {
							rep->current.loiter_orientation = curr->current.loiter_orientation;

						} else {
							rep->current.loiter_orientation = 0.0f;
						}

						if (curr->current.loiter_pattern > 0) {
							rep->current.loiter_pattern = curr->current.loiter_pattern;

						} else {
							rep->current.loiter_pattern = position_setpoint_s::LOITER_TYPE_ORBIT;
						}

						rep->current.loiter_direction_counter_clockwise = curr->current.loiter_direction_counter_clockwise;
					}

					rep->previous.timestamp = hrt_absolute_time();

					rep->current.valid = true;
					rep->current.timestamp = hrt_absolute_time();

					rep->next.valid = false;

					_time_loitering_after_gf_breach = 0; // a manual reposition unlatches the post-breach loiter state

				} else {
					mavlink_log_critical(&_mavlink_log_pub, "Reposition is outside geofence\t");
					events::send(events::ID("navigator_reposition_outside_geofence"), {events::Log::Error, events::LogInternal::Info},
						     "Reposition is outside geofence");
				}

				// CMD_DO_REPOSITION is acknowledged by commander

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_CHANGE_ALTITUDE
				   && _vstatus.arming_state == vehicle_status_s::ARMING_STATE_ARMED
				   && (_navigation_mode == &_course
				       || _vstatus.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER)) {
				// Only update the setpoint if armed and already in a mode that consumes it. Otherwise a later
				// switch into Hold could execute a stale setpoint (loiter.cpp applies it within a 500ms window).

				if (_navigation_mode == &_course) {
					// In course mode, update altitude directly (after geofence check)
					float new_alt = PX4_ISFINITE(cmd.param1) ? cmd.param1 : get_global_position()->alt;

					vehicle_global_position_s position_setpoint{};
					position_setpoint.lat = get_global_position()->lat;
					position_setpoint.lon = get_global_position()->lon;
					position_setpoint.alt = new_alt;

					if (geofence_allows_position(position_setpoint)) {
						_course.set_altitude(new_alt);

					} else {
						mavlink_log_critical(&_mavlink_log_pub, "Altitude change is outside geofence\t");
						events::send(events::ID("navigator_course_change_altitude_outside_geofence"), {events::Log::Error, events::LogInternal::Info},
							     "Altitude change is outside geofence");
					}

					// DO_CHANGE_ALTITUDE is acknowledged by commander

				} else {

					// A VEHICLE_CMD_DO_CHANGE_ALTITUDE has the exact same effect as a VEHICLE_CMD_DO_REPOSITION with only the altitude
					// field populated, this logic is copied from above.

					// only supports MAV_FRAME_GLOBAL and MAV_FRAMEs with absolute altitude amsl

					vehicle_global_position_s position_setpoint{};
					position_setpoint.lat = get_global_position()->lat;
					position_setpoint.lon = get_global_position()->lon;
					position_setpoint.alt = PX4_ISFINITE(cmd.param1) ? cmd.param1 : get_global_position()->alt;

					// Wait for vehicle_status before handling the next command, otherwise the setpoint could be overwritten
					_wait_for_vehicle_status_timestamp = hrt_absolute_time();

					if (geofence_allows_position(position_setpoint)) {
						position_setpoint_triplet_s *rep = get_reposition_triplet();
						position_setpoint_triplet_s *curr = get_position_setpoint_triplet();

						// store current position as previous position and goal as next
						rep->previous.yaw = get_local_position()->heading;
						rep->previous.lat = get_global_position()->lat;
						rep->previous.lon = get_global_position()->lon;
						rep->previous.alt = get_global_position()->alt;

						rep->current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;

						// on entering Loiter mode, reset speed setpoint to default
						if (_navigation_mode != &_loiter) {
							rep->current.cruising_speed = -1.f;

						} else {
							rep->current.cruising_speed = get_cruising_speed();
						}

						rep->current.cruising_throttle = get_cruising_throttle();
						rep->current.acceptance_radius = get_acceptance_radius();
						rep->current.yaw = NAN;

						// Position is not changing, thus we keep the setpoint
						rep->current.lat = PX4_ISFINITE(curr->current.lat) ? curr->current.lat : get_global_position()->lat;
						rep->current.lon = PX4_ISFINITE(curr->current.lon) ? curr->current.lon : get_global_position()->lon;

						// set the altitude corresponding to command
						rep->current.alt = PX4_ISFINITE(cmd.param1) ? cmd.param1 : get_global_position()->alt;

						if (PX4_ISFINITE(curr->current.loiter_radius) && curr->current.loiter_radius > FLT_EPSILON) {
							rep->current.loiter_radius = curr->current.loiter_radius;

						} else {
							rep->current.loiter_radius = get_default_loiter_rad();
						}

						rep->current.loiter_direction_counter_clockwise = curr->current.loiter_direction_counter_clockwise;

						rep->previous.timestamp = hrt_absolute_time();

						rep->current.valid = true;
						rep->current.timestamp = hrt_absolute_time();

						rep->next.valid = false;

						_time_loitering_after_gf_breach = 0; // a manual reposition unlatches the post-breach loiter state

					} else {
						mavlink_log_critical(&_mavlink_log_pub, "Altitude change is outside geofence\t");
						events::send(events::ID("navigator_change_altitude_outside_geofence"), {events::Log::Error, events::LogInternal::Info},
							     "Altitude change is outside geofence");
					}

					// DO_CHANGE_ALTITUDE is acknowledged by commander
				} // else (not course hold)

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_GUIDED_CHANGE_HEADING
				   && _vstatus.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {

				uint8_t result{vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED};

				// param1: heading type (0 = HEADING_TYPE_COURSE_OVER_GROUND)
				// param2: target bearing [deg, 0=north]
				const bool control_course = (lroundf(cmd.param1) == 0);

				if (_vstatus.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
				    && _navigation_mode == &_course && control_course && PX4_ISFINITE(cmd.param2)) {
					float course_rad = cmd.param2 * M_DEG_TO_RAD_F;

					if (_course.set_course(course_rad)) {
						result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
					}

					// DENIED if not FW, not in course mode, not correct heading type, or no positioning available
				}

				publish_vehicle_command_ack(cmd, result);

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_ORBIT &&
				   get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {

				// for multicopters the orbit command is directly executed by the orbit flighttask

				vehicle_global_position_s position_setpoint{};
				position_setpoint.lat = PX4_ISFINITE(cmd.param5) ? cmd.param5 : get_global_position()->lat;
				position_setpoint.lon = PX4_ISFINITE(cmd.param6) ? cmd.param6 : get_global_position()->lon;
				position_setpoint.alt = PX4_ISFINITE(cmd.param7) ? cmd.param7 : get_global_position()->alt;

				// Wait for vehicle_status before handling the next command, otherwise the setpoint could be overwritten
				_wait_for_vehicle_status_timestamp = hrt_absolute_time();

				if (geofence_allows_position(position_setpoint)) {
					position_setpoint_triplet_s *rep = get_reposition_triplet();
					rep->current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;
					rep->current.loiter_radius = get_default_loiter_rad();
					rep->current.loiter_direction_counter_clockwise = get_default_loiter_CCW();
					rep->current.loiter_orientation = 0.0f;
					rep->current.loiter_pattern = position_setpoint_s::LOITER_TYPE_ORBIT;
					rep->current.cruising_throttle = get_cruising_throttle();

					// on entering Loiter mode, reset speed setpoint to default
					if (_navigation_mode != &_loiter) {
						rep->current.cruising_speed = -1.f;

					} else {
						rep->current.cruising_speed = get_cruising_speed();
					}

					if (PX4_ISFINITE(cmd.param1)) {
						rep->current.loiter_radius = fabsf(cmd.param1);
						rep->current.loiter_direction_counter_clockwise = cmd.param1 < -FLT_EPSILON;
					}

					rep->current.lat = position_setpoint.lat;
					rep->current.lon = position_setpoint.lon;
					rep->current.alt = position_setpoint.alt;

					rep->current.valid = true;
					rep->current.timestamp = hrt_absolute_time();

					_time_loitering_after_gf_breach = 0; // a manual reposition unlatches the post-breach loiter state

				} else {
					mavlink_log_critical(&_mavlink_log_pub, "Orbit is outside geofence");
				}

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_FIGUREEIGHT &&
				   get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
#ifdef CONFIG_FIGURE_OF_EIGHT
				// Only valid for fixed wing mode

				vehicle_global_position_s position_setpoint{};
				position_setpoint.lat = PX4_ISFINITE(cmd.param5) ? cmd.param5 : get_global_position()->lat;
				position_setpoint.lon = PX4_ISFINITE(cmd.param6) ? cmd.param6 : get_global_position()->lon;
				position_setpoint.alt = PX4_ISFINITE(cmd.param7) ? cmd.param7 : get_global_position()->alt;

				// Wait for vehicle_status before handling the next command, otherwise the setpoint could be overwritten
				_wait_for_vehicle_status_timestamp = hrt_absolute_time();

				if (geofence_allows_position(position_setpoint)) {
					position_setpoint_triplet_s *rep = get_reposition_triplet();
					rep->current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;
					rep->current.loiter_minor_radius = fabsf(get_default_loiter_rad());
					rep->current.loiter_direction_counter_clockwise = get_default_loiter_CCW();
					rep->current.loiter_orientation = 0.0f;
					rep->current.loiter_pattern = position_setpoint_s::LOITER_TYPE_FIGUREEIGHT;
					rep->current.cruising_speed = get_cruising_speed();

					if (PX4_ISFINITE(cmd.param2) && fabsf(cmd.param2) > FLT_EPSILON) {
						rep->current.loiter_minor_radius = fabsf(cmd.param2);
					}

					rep->current.loiter_radius = 2.5f * rep->current.loiter_minor_radius;

					if (PX4_ISFINITE(cmd.param1)) {
						rep->current.loiter_radius = fabsf(cmd.param1);
						rep->current.loiter_direction_counter_clockwise = cmd.param1 < -FLT_EPSILON;
					}

					rep->current.loiter_radius = math::max(rep->current.loiter_radius, 2.0f * rep->current.loiter_minor_radius);

					if (PX4_ISFINITE(cmd.param4)) {
						rep->current.loiter_orientation = cmd.param4;
					}

					rep->current.lat = position_setpoint.lat;
					rep->current.lon = position_setpoint.lon;
					rep->current.alt = position_setpoint.alt;

					rep->current.valid = true;
					rep->current.timestamp = hrt_absolute_time();

					_time_loitering_after_gf_breach = 0; // a manual reposition unlatches the post-breach loiter state

				} else {
					mavlink_log_critical(&_mavlink_log_pub, "Figure 8 is outside geofence");
				}

#endif // CONFIG_FIGURE_OF_EIGHT

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF) {
				position_setpoint_triplet_s *rep = get_takeoff_triplet();

				// store current position as previous position and goal as next
				rep->previous.yaw = get_local_position()->heading;
				rep->previous.lat = get_global_position()->lat;
				rep->previous.lon = get_global_position()->lon;
				rep->previous.alt = get_global_position()->alt;

				rep->current.loiter_radius = get_default_loiter_rad();
				rep->current.loiter_direction_counter_clockwise = get_default_loiter_CCW();
				rep->current.type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;
				rep->current.cruising_speed = -1.f; // reset to default

				if (home_global_position_valid()) {

					rep->previous.valid = true;
					rep->previous.timestamp = hrt_absolute_time();

				} else {
					rep->previous.valid = false;
				}

				// Don't set a yaw setpoint for takeoff, as Navigator doesn't handle the yaw reset.
				// The yaw setpoint generation is handled by FlightTaskAuto.
				rep->current.yaw = NAN;

				if (PX4_ISFINITE(cmd.param5) && PX4_ISFINITE(cmd.param6)) {
					rep->current.lat = cmd.param5;
					rep->current.lon = cmd.param6;

				} else {
					// If one of them is non-finite set the current global position as target
					rep->current.lat = get_global_position()->lat;
					rep->current.lon = get_global_position()->lon;

				}

				rep->current.alt = cmd.param7;

				rep->current.valid = true;
				rep->current.timestamp = hrt_absolute_time();

				rep->next.valid = false;

				// Fixed-wing: vehicle will takeoff towards these coordinates and establish on a loiter there
				_takeoff.setLoiterPosition(matrix::Vector2d(cmd.param5, cmd.param6));
				_takeoff.setLoiterAltitudeAmsl(cmd.param7);

				// CMD_NAV_TAKEOFF is acknowledged by commander

#if CONFIG_MODE_NAVIGATOR_VTOL_TAKEOFF

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_NAV_VTOL_TAKEOFF) {

				_vtol_takeoff.setTransitionAltitudeAbsolute(cmd.param7);

				if (std::fabs(cmd.param2 - 3.0f) < FLT_EPSILON) { // Specified transition direction
					_vtol_takeoff.setTransitionDirection(cmd.param4);
				}

				// after the transition the vehicle will establish on a loiter at this position
				_vtol_takeoff.setLoiterLocation(matrix::Vector2d(cmd.param5, cmd.param6));

				// loiter height is the height above takeoff altitude at which the vehicle will establish on a loiter circle
				_vtol_takeoff.setLoiterHeight(cmd.param1);
#endif //CONFIG_MODE_NAVIGATOR_VTOL_TAKEOFF

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_LAND_START) {

				// find NAV_CMD_DO_LAND_START in the mission and
				// use MAV_CMD_MISSION_START to start the mission from the next item containing a position setpoint
				uint8_t result{vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED};

				if (_mission.get_land_start_available()) {
					vehicle_command_s vehicle_command{};
					vehicle_command.command = vehicle_command_s::VEHICLE_CMD_MISSION_START;
					vehicle_command.param1 = _mission.get_land_start_index();
					publish_vehicle_command(vehicle_command);

				} else {
					PX4_WARN("planned mission landing not available");
					result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_CANCELLED;
				}

				publish_vehicle_command_ack(cmd, result);

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_MISSION_START) {
				if (_mission_result.valid && PX4_ISFINITE(cmd.param1) && (cmd.param1 >= 0)) {
					if (!_mission.set_current_mission_index(cmd.param1)) {
						PX4_WARN("CMD_MISSION_START failed");
					}
				}

				// CMD_MISSION_START is acknowledged by commander

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_CHANGE_SPEED) {
				if (cmd.param2 > FLT_EPSILON) {
					// XXX not differentiating ground and airspeed yet
					set_cruising_speed(cmd.param2);

				} else {
					reset_cruising_speed();

					/* if no speed target was given try to set throttle */
					if (cmd.param3 > FLT_EPSILON) {
						set_cruising_throttle(cmd.param3 / 100);

					} else {
						set_cruising_throttle();
					}
				}

				// TODO: handle responses for supported DO_CHANGE_SPEED options?
				publish_vehicle_command_ack(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_SET_ROI
				   || cmd.command == vehicle_command_s::VEHICLE_CMD_NAV_ROI
				   || cmd.command == vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_LOCATION
				   || cmd.command == vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_WPNEXT_OFFSET
				   || cmd.command == vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_NONE) {
				_vroi = {};

				switch (cmd.command) {
				case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI:
				case vehicle_command_s::VEHICLE_CMD_NAV_ROI:
					_vroi.mode = cmd.param1;
					break;

				case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_LOCATION:
					_vroi.mode = vehicle_command_s::VEHICLE_ROI_LOCATION;
					_vroi.lat = cmd.param5;
					_vroi.lon = cmd.param6;
					_vroi.alt = cmd.param7;
					break;

				case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_WPNEXT_OFFSET:
					_vroi.mode = vehicle_command_s::VEHICLE_ROI_WPNEXT;
					_vroi.pitch_offset = (float)cmd.param5 * M_DEG_TO_RAD_F;
					_vroi.roll_offset = (float)cmd.param6 * M_DEG_TO_RAD_F;
					_vroi.yaw_offset = (float)cmd.param7 * M_DEG_TO_RAD_F;
					break;

				case vehicle_command_s::VEHICLE_CMD_DO_SET_ROI_NONE:
					_vroi.mode = vehicle_command_s::VEHICLE_ROI_NONE;
					break;

				default:
					_vroi.mode = vehicle_command_s::VEHICLE_ROI_NONE;
					break;
				}

				_vroi.timestamp = hrt_absolute_time();

				_vehicle_roi_pub.publish(_vroi);

				publish_vehicle_command_ack(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION
				   && get_vstatus()->nav_state != vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF) {
				// reset cruise speed and throttle to default when transitioning (VTOL Takeoff handles it separately)
				reset_cruising_speed();
				set_cruising_throttle();
			}

			else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_AUTOTUNE_ENABLE) {


				if (fabsf(cmd.param1 - 1.f) > FLT_EPSILON) {
					// only support enabling autotune (consistent with autotune module)
					events::send(events::ID("navigator_autotune_unsupported_input"), {events::Log::Warning, events::LogInternal::Warning},
						     "Provided autotune command is not supported. To enable autotune in mission, set param1 to 1");

				} else if (fabsf(cmd.param2) > FLT_EPSILON) {
					// warn user about axis selection
					events::send(events::ID("navigator_autotune_unsupported_ax"), {events::Log::Warning, events::LogInternal::Info},
						     "Autotune axis selection not supported through Mission. Use FW_AT_AXES to set axes for fixed-wing vehicles");
				}
			}
		}

#if CONFIG_NAVIGATOR_ADSB
		/* Check for traffic */
		check_traffic();
#endif // CONFIG_NAVIGATOR_ADSB

		/* Check geofence violation */
		geofence_breach_check();

		/* Do stuff according to navigation state set by commander */
		NavigatorMode *navigation_mode_new{nullptr};

		switch (_vstatus.nav_state) {
		case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
			_pos_sp_triplet_published_invalid_once = false;

			navigation_mode_new = &_mission;

			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
			_pos_sp_triplet_published_invalid_once = false;
			navigation_mode_new = &_loiter;
			break;

		case vehicle_status_s::NAVIGATION_STATE_GUIDED_COURSE:
			_pos_sp_triplet_published_invalid_once = false;
			navigation_mode_new = &_course;
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:

			// If we are already in mission landing, do not switch.
			if (_navigation_mode == &_mission && _mission.isLanding()) {
				navigation_mode_new = &_mission;
				break;

			} else {
				_pos_sp_triplet_published_invalid_once = false;
			}

			navigation_mode_new = &_rtl;

			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
			_pos_sp_triplet_published_invalid_once = false;
			navigation_mode_new = &_takeoff;
			break;

#if CONFIG_MODE_NAVIGATOR_VTOL_TAKEOFF

		case vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF:
			_pos_sp_triplet_published_invalid_once = false;
			navigation_mode_new = &_vtol_takeoff;
			break;
#endif //CONFIG_MODE_NAVIGATOR_VTOL_TAKEOFF

		case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
			_pos_sp_triplet_published_invalid_once = false;
			navigation_mode_new = &_land;
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
			_pos_sp_triplet_published_invalid_once = false;
			navigation_mode_new = &_precland;
			_precland.set_mode(PrecLandMode::Required);
			break;

		case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		case vehicle_status_s::NAVIGATION_STATE_ACRO:
		case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		case vehicle_status_s::NAVIGATION_STATE_ALTITUDE_CRUISE:
		case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		case vehicle_status_s::NAVIGATION_STATE_DESCEND:
		case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
		case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		case vehicle_status_s::NAVIGATION_STATE_STAB:
		default:
			navigation_mode_new = nullptr;
			break;
		}

		// Do not execute any state machine while we are disarmed
		if (_vstatus.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
			navigation_mode_new = nullptr;
		}

		/* we have a new navigation mode: reset triplet */
		if (_navigation_mode != navigation_mode_new) {
			// We don't reset the triplet in the following two cases:
			// 1)  if we just did an auto-takeoff and are now
			// going to loiter. Otherwise, we lose the takeoff altitude and end up lower
			// than where we wanted to go.
			// 2) We switch to loiter and the current position setpoint already has a valid loiter point.
			// In that case we can assume that the vehicle has already established a loiter and we don't need to set a new
			// loiter position.
			//
			// FIXME: a better solution would be to add reset where they are needed and remove
			//        this general reset here.

			const bool current_mode_is_takeoff = _navigation_mode == &_takeoff;
			const bool new_mode_is_loiter = navigation_mode_new == &_loiter;
			const bool valid_loiter_setpoint = (_pos_sp_triplet.current.valid
							    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER);

			const bool did_not_switch_takeoff_to_loiter = !(current_mode_is_takeoff && new_mode_is_loiter);
			const bool did_not_switch_to_loiter_with_valid_loiter_setpoint = !(new_mode_is_loiter && valid_loiter_setpoint);

			if (did_not_switch_takeoff_to_loiter && did_not_switch_to_loiter_with_valid_loiter_setpoint) {
				reset_triplets();
			}
		}

		// VTOL: transition to hover in Descend mode if force_vtol() is true
		if (_vstatus.nav_state == vehicle_status_s::NAVIGATION_STATE_DESCEND &&
		    _vstatus.is_vtol && _vstatus.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING &&
		    force_vtol()) {
			vehicle_command_s vehicle_command{};
			vehicle_command.command = NAV_CMD_DO_VTOL_TRANSITION;
			vehicle_command.param1 = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
			publish_vehicle_command(vehicle_command);
			mavlink_log_info(&_mavlink_log_pub, "Transition to hover mode and descend.\t");
			events::send(events::ID("navigator_transition_descend"), events::Log::Critical,
				     "Transition to hover mode and descend");
		}

		_navigation_mode = navigation_mode_new;

		if (_wait_for_vehicle_status_timestamp != 0 && _vstatus.timestamp > _wait_for_vehicle_status_timestamp) {
			_wait_for_vehicle_status_timestamp = 0;
		}

		/* iterate through navigation modes and set active/inactive for each */
		for (unsigned int i = 0; i < NAVIGATOR_MODE_ARRAY_SIZE; i++) {
			if (_navigation_mode_array[i]) {
				_navigation_mode_array[i]->run(_navigation_mode == _navigation_mode_array[i]);
			}
		}

		/* if nothing is running, set position setpoint triplet invalid once */
		if (_navigation_mode == nullptr && !_pos_sp_triplet_published_invalid_once) {
			_pos_sp_triplet_published_invalid_once = true;
			reset_triplets();
		}

		if (_pos_sp_triplet_updated) {
			publish_position_setpoint_triplet();
		}

		if (_mission_result_updated) {
			publish_mission_result();
		}

		neutralize_gimbal_if_control_activated();

		publish_navigator_status();

		publish_distance_sensor_mode_request();

		_geofence.run();

#if CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE

		const bool fence_updated = _geofence.consumeFenceUpdated();
		const float margin = geofence_avoidance_margin();

		// Margin is baked into polygons, so a margin change requires rebuilding them.
		const bool margin_changed = fabsf(margin - _last_geofence_avoidance_margin) > FLT_EPSILON;


		using PlannerStatus = GeofenceAvoidancePlanner::Status;

		if (fence_updated || margin_changed) {
			_geofence_avoidance_planner.updateGraphFromGeofence(_geofence, margin);
			const PlannerStatus planner_status = _geofence_avoidance_planner.status();
			_last_geofence_avoidance_margin = margin;

			// Add granularity with more status values / user messages if needed.

			switch (planner_status) {
			// Failure in building fence graph / path. Collapse to one generic user message. Add granularity if needed.
			case PlannerStatus::BudgetExceeded: // TODO make this more specific now that it is more likely
			case PlannerStatus::OutOfRange:
			case PlannerStatus::Degenerate:
			case PlannerStatus::DijkstraFailed:
				mavlink_log_warning(&_mavlink_log_pub, "Geofence data invalid (code %d), RTL will fly directly\t",
						    (int) planner_status);
				events::send<uint8_t>(events::ID("rtl_avoidance_build_failed"), {events::Log::Warning, events::LogInternal::Info},
						      "Geofence data invalid (code {1}), RTL will fly directly", (uint8_t)planner_status);
				break;

			default:
				// Not an error, or reported elsewhere
				break;
			}

		} else {

			// Signal status values not related to updating geofence data. Reset status to not spam.
			const PlannerStatus planner_status = _geofence_avoidance_planner.status();

			if (planner_status == PlannerStatus::DestinationInvalid) {
				mavlink_log_warning(&_mavlink_log_pub, "RTL destination invalid, not updating\t");
				events::send(
					events::ID("rtl_destination_invalid"),
					events::LogLevels(events::Log::Warning, events::LogInternal::Info),
					"RTL destination invalid, not updating"
				);
				_geofence_avoidance_planner.resetStatus();
			}

			if (planner_status == PlannerStatus::DestinationBreachesGeofence) {
				mavlink_log_warning(&_mavlink_log_pub, "RTL destination breaches geofence, will fly directly\t");
				events::send(
					events::ID("rtl_destination_breaches"),
					events::LogLevels(events::Log::Warning, events::LogInternal::Info),
					"RTL destination breaches geofence, will fly directly"
				);
				_geofence_avoidance_planner.resetStatus();
			}
		}


#endif // CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE

		perf_end(_loop_perf);
	}
}

void Navigator::geofence_breach_check()
{
	static constexpr hrt_abstime GEOFENCE_CHECK_INTERVAL_US = 200_ms;

	// reset the post-breach loiter latch if we left AUTO_LOITER (and 100ms after it was triggered)
	if (_vstatus.nav_state != vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER
	    && hrt_elapsed_time(&_time_loitering_after_gf_breach) > 100_ms) {
		_time_loitering_after_gf_breach = 0;
	}

	if (hrt_elapsed_time(&_last_geofence_check) > GEOFENCE_CHECK_INTERVAL_US) {

		double current_latitude = _global_pos.lat;
		double current_longitude = _global_pos.lon;
		float current_altitude = _global_pos.alt;
		bool position_valid = _global_pos.timestamp > 0;

		if (_geofence.getSource() == Geofence::GF_SOURCE_GPS) {
			current_latitude = _gps_pos.latitude_deg;
			current_longitude = _gps_pos.longitude_deg;
			current_altitude = _gps_pos.altitude_msl_m;
			position_valid = _global_pos.timestamp > 0;
		}

		if (!position_valid) {
			// we don't have a valid position yet, so we can't check for geofence violations
			return;
		}

		if (_time_loitering_after_gf_breach > 0) {
			// while loitering after a breach, only allow new triggers to be set, never cleared.
			// Prevents the reposition center from walking outward when the vehicle hovers at the fence boundary.
			_geofence_result.geofence_max_dist_triggered |= !_geofence.isCloserThanMaxDistToHome(current_latitude,
					current_longitude, current_altitude);
			_geofence_result.geofence_max_alt_triggered |= !_geofence.isBelowMaxAltitude(current_altitude);
			_geofence_result.geofence_custom_fence_triggered |= !_geofence.isInsidePolygonOrCircle(current_latitude,
					current_longitude, current_altitude);

		} else {
			_geofence_result.geofence_max_dist_triggered = !_geofence.isCloserThanMaxDistToHome(current_latitude,
					current_longitude, current_altitude);
			_geofence_result.geofence_max_alt_triggered = !_geofence.isBelowMaxAltitude(current_altitude);
			_geofence_result.geofence_custom_fence_triggered = !_geofence.isInsidePolygonOrCircle(current_latitude,
					current_longitude, current_altitude);
		}

		const auto now = hrt_absolute_time();

		_last_geofence_check = now;

		_geofence_result.timestamp = now;
		_geofence_result.geofence_action = _geofence.getGeofenceAction();

		const bool breach = _geofence_result.geofence_max_dist_triggered
				    || _geofence_result.geofence_max_alt_triggered
				    || _geofence_result.geofence_custom_fence_triggered;

		if (breach) {
			if (!_geofence_reposition_sent && _vstatus.arming_state == vehicle_status_s::ARMING_STATE_ARMED
			    && _geofence.getGeofenceAction() == geofence_result_s::GF_ACTION_LOITER) {

				// loiter at the current position; we no longer predict ahead of the vehicle
				position_setpoint_triplet_s *rep = get_reposition_triplet();
				rep->current.timestamp = now;
				rep->current.yaw = NAN;
				rep->current.lat = current_latitude;
				rep->current.lon = current_longitude;
				rep->current.alt = current_altitude;
				rep->current.valid = true;
				rep->current.loiter_radius = get_default_loiter_rad();
				rep->current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;
				rep->current.cruising_throttle = get_cruising_throttle();
				rep->current.acceptance_radius = get_acceptance_radius();
				rep->current.cruising_speed = get_cruising_speed();

				_geofence_reposition_sent = true;
				_time_loitering_after_gf_breach = now;
			}

		} else {
			_geofence_reposition_sent = false;
		}

		_geofence_result_pub.publish(_geofence_result);
	}
}

int Navigator::run_trampoline(int argc, char *argv[])
{
	return ModuleBase::run_trampoline_impl(desc, [](int ac, char *av[]) -> ModuleBase * {
		return Navigator::instantiate(ac, av);
	}, argc, argv);
}

int Navigator::task_spawn(int argc, char *argv[])
{
	desc.task_id = px4_task_spawn_cmd("navigator",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_NAVIGATION,
					  PX4_STACK_ADJUSTED(2230),
					  (px4_main_t)&run_trampoline,
					  (char *const *)argv);

	if (desc.task_id < 0) {
		desc.task_id = -1;
		return -errno;
	}

	return 0;
}

Navigator *Navigator::instantiate(int argc, char *argv[])
{
	Navigator *instance = new Navigator();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

int Navigator::print_status()
{
	PX4_INFO("Running");

	_geofence.printStatus();
#if CONFIG_NAVIGATOR_ADSB && !defined(CONSTRAINED_FLASH) && !defined(__PX4_NUTTX)
	_detect_and_avoid.print_status();
#endif // CONFIG_NAVIGATOR_ADSB && !CONSTRAINED_FLASH && !__PX4_NUTTX
	return 0;
}

void Navigator::publish_position_setpoint_triplet()
{
	_pos_sp_triplet.timestamp = hrt_absolute_time();
	_pos_sp_triplet_pub.publish(_pos_sp_triplet);
	_pos_sp_triplet_updated = false;
}

float Navigator::get_default_acceptance_radius() const
{
	return _param_nav_acc_rad.get();
}

float Navigator::get_altitude_acceptance_radius()
{
	if (get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {

		const position_setpoint_s &curr_sp = get_position_setpoint_triplet()->current;
		const position_setpoint_s &next_sp = get_position_setpoint_triplet()->next;

		if ((PX4_ISFINITE(curr_sp.alt_acceptance_radius) && curr_sp.alt_acceptance_radius > FLT_EPSILON)) {
			return curr_sp.alt_acceptance_radius;

		} else if (!force_vtol() && next_sp.type == position_setpoint_s::SETPOINT_TYPE_LAND && next_sp.valid) {
			// Use separate (tighter) altitude acceptance for clean altitude starting point before FW landing
			return _param_nav_fw_altl_rad.get();

		} else {
			return _param_nav_fw_alt_rad.get();
		}

	} else if (get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROVER) {
		return INFINITY;

	} else {
		float alt_acceptance_radius = _param_nav_mc_alt_rad.get();
		const position_setpoint_s &curr_sp = get_position_setpoint_triplet()->current;

		if (PX4_ISFINITE(curr_sp.alt_acceptance_radius) && curr_sp.alt_acceptance_radius > FLT_EPSILON) {
			alt_acceptance_radius = curr_sp.alt_acceptance_radius;

		}

		return alt_acceptance_radius;
	}
}

void Navigator::reset_triplets()
{
	reset_position_setpoint(_pos_sp_triplet.previous);
	reset_position_setpoint(_pos_sp_triplet.current);
	reset_position_setpoint(_pos_sp_triplet.next);

	_pos_sp_triplet_updated = true;
}

void Navigator::reset_position_setpoint(position_setpoint_s &sp)
{
	sp = position_setpoint_s{};
	sp.timestamp = hrt_absolute_time();
	sp.lat = static_cast<double>(NAN);
	sp.lon = static_cast<double>(NAN);
	sp.yaw = NAN;
	sp.course = NAN;
	sp.loiter_radius = get_default_loiter_rad();
	sp.acceptance_radius = get_default_acceptance_radius();
	sp.cruising_speed = get_cruising_speed();
	sp.cruising_throttle = get_cruising_throttle();
	sp.valid = false;
	sp.type = position_setpoint_s::SETPOINT_TYPE_IDLE;
	sp.loiter_direction_counter_clockwise = false;
}

float Navigator::get_cruising_throttle()
{
	/* Return the mission-requested cruise speed, or default FW_THR_TRIM value */
	if (_mission_throttle > FLT_EPSILON) {
		return _mission_throttle;

	} else {
		return NAN;
	}
}

float Navigator::get_acceptance_radius()
{
	float acceptance_radius = get_default_acceptance_radius(); // the value specified in the parameter NAV_ACC_RAD

	// for fixed-wing and rover, return the max of NAV_ACC_RAD and the controller acceptance radius (e.g. navigation switch distance)
	if (_vstatus.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {

		const fixed_wing_lateral_guidance_status_s &fw_guidance_status = _fw_lateral_guidance_status_sub.get();

		if (PX4_ISFINITE(fw_guidance_status.switch_distance) && fw_guidance_status.timestamp != 0) {
			acceptance_radius = math::max(acceptance_radius, fw_guidance_status.switch_distance);
		}

	} else if (_vstatus.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROVER) {

		const position_controller_status_s &pos_ctrl_status = _position_controller_status_sub.get();

		if (PX4_ISFINITE(pos_ctrl_status.acceptance_radius) && pos_ctrl_status.timestamp != 0) {
			acceptance_radius = math::max(acceptance_radius, pos_ctrl_status.acceptance_radius);
		}
	}

	return acceptance_radius;
}

bool Navigator::get_yaw_to_be_accepted(float mission_item_yaw)
{
	float yaw = mission_item_yaw;

	return PX4_ISFINITE(yaw);
}

void Navigator::load_fence_from_file(const char *filename)
{
	_geofence.loadFromFile(filename);
}

#if CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC
void Navigator::run_fake_traffic(DetectAndAvoid::FakeTraffMode mode)
{
	_detect_and_avoid.run_fake_traffic(mode, get_global_position()->lat, get_global_position()->lon,
					   get_global_position()->alt);
}

void Navigator::stop_fake_traffic()
{
	_detect_and_avoid.stop_fake_traffic();
}
#endif // CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC

#if CONFIG_NAVIGATOR_ADSB
void Navigator::check_traffic()
{
	_detect_and_avoid.on_active();
}
#endif // CONFIG_NAVIGATOR_ADSB

bool Navigator::abort_landing()
{
	// only abort if currently landing and position controller status updated
	bool should_abort = false;

	if (_pos_sp_triplet.current.valid
	    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {

		if (_pos_ctrl_landing_status_sub.updated()) {
			position_controller_landing_status_s landing_status{};

			// landing status from position controller must be newer than navigator's last position setpoint
			if (_pos_ctrl_landing_status_sub.copy(&landing_status)) {
				if (landing_status.timestamp > _pos_sp_triplet.timestamp) {
					should_abort = (landing_status.abort_status > 0);
				}
			}
		}
	}

	return should_abort;
}

bool Navigator::force_vtol()
{
	return _vstatus.is_vtol &&
	       (_vstatus.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING || _vstatus.in_transition_to_fw)
	       && _param_nav_force_vt.get();
}

int Navigator::custom_command(int argc, char *argv[])
{
	if (!is_running(desc)) {
		print_usage("not running");
		return 1;
	}

	if (!strcmp(argv[0], "fencefile")) {
		get_instance<Navigator>(desc)->load_fence_from_file(GEOFENCE_FILENAME);
		return 0;
	}

#if CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC

	if (!strcmp(argv[0], "fake_traffic")) {
		DetectAndAvoid::FakeTraffMode mode = DetectAndAvoid::FakeTraffMode::kUniqueIds;

		if (argc > 1) {
			const char *const argument = argv[1];

			if (!strcmp(argument, "help")) {
				return print_fake_traffic_usage();
			}

			if (!strcmp(argument, "stop")) {
				if (argc > 2) {
					return print_fake_traffic_usage("fake_traffic stop takes no extra arguments");
				}

				get_instance<Navigator>(desc)->stop_fake_traffic();
				PX4_INFO("DAA: fake traffic stopped");
				return 0;
			}

			if (!parse_fake_traffic_mode(argument, mode)) {
				return print_fake_traffic_usage("unknown fake_traffic mode");
			}
		}

		get_instance<Navigator>(desc)->run_fake_traffic(mode);

		return 0;
	}

#endif // CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC

	return print_usage("unknown command");
}

void Navigator::publish_mission_result()
{
	_mission_result.timestamp = hrt_absolute_time();

	/* lazily publish the mission result only once available */
	_mission_result_pub.publish(_mission_result);

	/* reset some of the flags */
	_mission_result.item_do_jump_changed = false;
	_mission_result.item_changed_index = 0;
	_mission_result.item_do_jump_remaining = 0;

	_mission_result_updated = false;
}

void Navigator::set_mission_failure_heading_timeout()
{
	if (!_mission_result.failure) {
		_mission_result.failure = true;
		set_mission_result_updated();
		mavlink_log_critical(&_mavlink_log_pub, "unable to reach heading within timeout\t");
		events::send(events::ID("navigator_mission_failure_heading"), events::Log::Critical,
			     "Mission failure: unable to reach heading within timeout");
	}
}

void Navigator::trigger_hagl_failsafe(const uint8_t nav_state)
{
	if ((_navigator_status.failure != navigator_status_s::FAILURE_HAGL) || _navigator_status.nav_state != nav_state) {
		_navigator_status.failure = navigator_status_s::FAILURE_HAGL;
		_navigator_status.nav_state = nav_state;

		_navigator_status_updated = true;
	}
}

void Navigator::publish_navigator_status()
{
	uint8_t current_nav_state = _vstatus.nav_state;

	if (_navigation_mode != nullptr) {
		current_nav_state = _navigation_mode->getNavigatorStateId();
	}

	if (_navigator_status.nav_state != current_nav_state) {
		_navigator_status.nav_state = current_nav_state;
		_navigator_status.failure = navigator_status_s::FAILURE_NONE;
		_navigator_status_updated = true;
	}

	if (_navigator_status_updated
	    || (hrt_elapsed_time(&_last_navigator_status_publication) > 500_ms)) {
		_navigator_status.timestamp = hrt_absolute_time();
		_navigator_status_pub.publish(_navigator_status);

		_navigator_status_updated = false;
		_last_navigator_status_publication = hrt_absolute_time();
	}
}

void Navigator::publish_vehicle_command(vehicle_command_s &vehicle_command)
{
	vehicle_command.timestamp = hrt_absolute_time();
	vehicle_command.source_system = _vstatus.system_id;
	vehicle_command.source_component = _vstatus.component_id;
	vehicle_command.target_system = _vstatus.system_id;
	vehicle_command.confirmation = false;
	vehicle_command.from_external = false;

	int target_camera_component_id;

	// The camera commands are not processed on the autopilot but will be
	// sent to the mavlink links to other components.
	switch (vehicle_command.command) {
	case NAV_CMD_IMAGE_START_CAPTURE:

		if (static_cast<int>(vehicle_command.param3) == 1) {
			// When sending a single capture we need to include the sequence number, thus camera_trigger needs to handle this command
			vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_DIGICAM_CONTROL;
			vehicle_command.param1 = 0.f; // Session control hide lens
			vehicle_command.param2 = 0.f; // Zoom absolute position
			vehicle_command.param3 = 0.f; // Zoom step
			vehicle_command.param4 = 0.f; // Focus lock
			vehicle_command.param5 = 1.; // Shoot command
			vehicle_command.param6 = 0.; // Command identity
			vehicle_command.param7 = 0.f; // Shot identifier

		} else {
			// We are only capturing multiple if param3 is 0 or > 1.
			// For multiple pictures the sequence number does not need to be included, thus there is no need to go through camera_trigger
			_is_capturing_images = true;
		}

		target_camera_component_id = static_cast<int>(vehicle_command.param1); // Target id from param 1

		if (target_camera_component_id > 0 && target_camera_component_id < 256) {
			vehicle_command.target_component = target_camera_component_id;

		} else {
			vehicle_command.target_component = 100; // MAV_COMP_ID_CAMERA
		}

		break;

	case NAV_CMD_IMAGE_STOP_CAPTURE:
		_is_capturing_images = false;
		target_camera_component_id = static_cast<int>(vehicle_command.param1); // Target id from param 1

		if (target_camera_component_id > 0 && target_camera_component_id < 256) {
			vehicle_command.target_component = target_camera_component_id;

		} else {
			vehicle_command.target_component = 100; // MAV_COMP_ID_CAMERA
		}

		break;

	case NAV_CMD_SET_CAMERA_MODE:
		target_camera_component_id = static_cast<int>(vehicle_command.param1); // Target id from param 1

		if (target_camera_component_id > 0 && target_camera_component_id < 256) {
			vehicle_command.target_component = target_camera_component_id;

		} else {
			vehicle_command.target_component = 100; // MAV_COMP_ID_CAMERA
		}

		break;

	case NAV_CMD_SET_CAMERA_SOURCE:
		target_camera_component_id = static_cast<int>(vehicle_command.param1); // Target id from param 1

		if (target_camera_component_id > 0 && target_camera_component_id < 256) {
			vehicle_command.target_component = target_camera_component_id;

		} else {
			vehicle_command.target_component = 100; // MAV_COMP_ID_CAMERA
		}

		break;

	case NAV_CMD_VIDEO_START_CAPTURE:
	case NAV_CMD_VIDEO_STOP_CAPTURE:
		vehicle_command.target_component = 100; // MAV_COMP_ID_CAMERA
		break;

	default:
		vehicle_command.target_component = 0;
		break;
	}

	_vehicle_cmd_pub.publish(vehicle_command);
}

void Navigator::publish_distance_sensor_mode_request()
{
	// Send request to enable distance sensor when in the landing phase of a mission or RTL
	if (((_navigation_mode == &_rtl) && _rtl.isLanding()) || ((_navigation_mode == &_mission) && _mission.isLanding())) {

		if (_distance_sensor_mode_change_request_pub.get().request_on_off !=
		    distance_sensor_mode_change_request_s::REQUEST_ON) {

			_distance_sensor_mode_change_request_pub.get().timestamp = hrt_absolute_time();
			_distance_sensor_mode_change_request_pub.get().request_on_off =
				distance_sensor_mode_change_request_s::REQUEST_ON;
			_distance_sensor_mode_change_request_pub.update();
		}

	} else if (_distance_sensor_mode_change_request_pub.get().request_on_off !=
		   distance_sensor_mode_change_request_s::REQUEST_OFF) {

		_distance_sensor_mode_change_request_pub.get().timestamp = hrt_absolute_time();
		_distance_sensor_mode_change_request_pub.get().request_on_off =
			distance_sensor_mode_change_request_s::REQUEST_OFF;
		_distance_sensor_mode_change_request_pub.update();
	}
}

void Navigator::publish_vehicle_command_ack(const vehicle_command_s &cmd, uint8_t result)
{
	vehicle_command_ack_s command_ack = {};

	command_ack.timestamp = hrt_absolute_time();
	command_ack.command = cmd.command;
	command_ack.target_system = cmd.source_system;
	command_ack.target_component = cmd.source_component;
	command_ack.from_external = false;

	command_ack.result = result;
	command_ack.result_param1 = 0;
	command_ack.result_param2 = 0;

	_vehicle_cmd_ack_pub.publish(command_ack);
}

void Navigator::acquire_gimbal_control()
{
	vehicle_command_s vehicle_command{};
	vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE;
	vehicle_command.param1 = _vstatus.system_id; // Take primary control
	vehicle_command.param2 = _vstatus.component_id;
	vehicle_command.param3 = -1.f; // Leave secondary control unchanged
	vehicle_command.param4 = -1.f;
	publish_vehicle_command(vehicle_command);
}

void Navigator::release_gimbal_control()
{
	vehicle_command_s vehicle_command{};
	vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE;
	vehicle_command.param1 = -3.f; // Remove primary control if it was taken
	vehicle_command.param2 = -3.f;
	vehicle_command.param3 = -1.f; // Leave secondary control unchanged
	vehicle_command.param4 = -1.f;
	publish_vehicle_command(vehicle_command);
}


void
Navigator::stop_capturing_images()
{
	if (_is_capturing_images) {
		vehicle_command_s vehicle_command{};
		vehicle_command.command = NAV_CMD_IMAGE_STOP_CAPTURE;
		vehicle_command.param1 = 0.f;
		publish_vehicle_command(vehicle_command);

		// _is_capturing_images is reset inside publish_vehicle_command.
	}
}

bool Navigator::geofence_allows_position(const vehicle_global_position_s &pos)
{
	if ((_geofence.getGeofenceAction() != geofence_result_s::GF_ACTION_NONE) &&
	    (_geofence.getGeofenceAction() != geofence_result_s::GF_ACTION_WARN)) {

		if (PX4_ISFINITE(pos.lat) && PX4_ISFINITE(pos.lon) && PX4_ISFINITE(pos.alt)) {
			return _geofence.checkPointAgainstAllGeofences(pos.lat, pos.lon, pos.alt);
		}
	}

	return true;
}

#if CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE
float Navigator::geofence_avoidance_margin() const
{
	// These margins should be above the horizontal tracking error that can be expected for each vehicle type.
	// If re-using the enlarged polygons for a predictive geofence failsafe feature, additionally ensure
	// that the margin is large enough to turn around / stop / carry out the desired failsafe action in time.

	if (_vstatus.is_vtol || _vstatus.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {

		// Use FW loiter radius even for VTOL in MC -- changing the
		// margin on transition is confusing and RTLing in MC as a VTOL
		// is a very rare edge case
		return get_default_loiter_rad();
	}

	// MC, rover, unspecified
	return 2.0f * get_default_acceptance_radius();
}
#endif // CONFIG_NAVIGATOR_GEOFENCE_AVOIDANCE

void Navigator::preproject_stop_point(double &lat, double &lon)
{
	// For multirotors we need to account for the braking distance, otherwise the vehicle will overshoot and go back
	const float course_over_ground = atan2f(_local_pos.vy, _local_pos.vx);

	// predict braking distance

	const float velocity_hor_abs = sqrtf(_local_pos.vx * _local_pos.vx + _local_pos.vy * _local_pos.vy);

	const float multirotor_braking_distance = math::trajectory::computeBrakingDistanceFromVelocity(velocity_hor_abs,
			_param_mpc_jerk_auto, _param_mpc_acc_hor, 0.6f * _param_mpc_jerk_auto);

	waypoint_from_heading_and_distance(get_global_position()->lat, get_global_position()->lon, course_over_ground,
					   multirotor_braking_distance, &lat, &lon);
}

void Navigator::mode_completed(uint8_t nav_state, uint8_t result)
{
	mode_completed_s mode_completed{};
	mode_completed.timestamp = hrt_absolute_time();
	mode_completed.result = result;
	mode_completed.nav_state = nav_state;
	_mode_completed_pub.publish(mode_completed);
}


void Navigator::disable_camera_trigger()
{
	// Disable camera trigger
	vehicle_command_s vehicle_command{};
	vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL;
	// Pause trigger
	vehicle_command.param1 = -1.f;
	vehicle_command.param3 = 1.f;
	publish_vehicle_command(vehicle_command);
}

void Navigator::set_gimbal_neutral()
{
	vehicle_command_s vehicle_command{};
	vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW;
	vehicle_command.param1 = NAN; // Don't set any angles
	vehicle_command.param2 = NAN;
	vehicle_command.param3 = NAN; // Don't set any angular velocities
	vehicle_command.param4 = NAN;
	vehicle_command.param5 = gimbal_manager_set_attitude_s::GIMBAL_MANAGER_FLAGS_NEUTRAL;
	publish_vehicle_command(vehicle_command);
}

void Navigator::activate_set_gimbal_neutral_timer(const hrt_abstime timestamp)
{
	if (_gimbal_neutral_activation_time == UINT64_MAX) {
		_gimbal_neutral_activation_time = timestamp;
	}
}

void Navigator::neutralize_gimbal_if_control_activated()
{
	const hrt_abstime now{hrt_absolute_time()};

	// The time delay must be sufficiently long to allow flight tasks to complete its
	// destruction and release gimbal control before the navigator takes control of the gimbal.
	if (_gimbal_neutral_activation_time != UINT64_MAX && now > _gimbal_neutral_activation_time + 250_ms) {
		acquire_gimbal_control();
		set_gimbal_neutral();
		release_gimbal_control();
		_gimbal_neutral_activation_time = UINT64_MAX;
	}
}

void Navigator::sendWarningDescentStoppedDueToTerrain()
{
	mavlink_log_critical(&_mavlink_log_pub, "Terrain collision risk, descent is stopped\t");
	events::send(events::ID("navigator_terrain_collision_risk"), events::Log::Critical,
		     "Terrain collision risk, descent is stopped");
}

int Navigator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Module that is responsible for autonomous flight modes. This includes missions (read from dataman),
takeoff and RTL.
It is also responsible for geofence violation checking.

### Implementation
The different internal modes are implemented as separate classes that inherit from a common base class `NavigatorMode`.
The member `_navigation_mode` contains the current active mode.

Navigator publishes position setpoint triplets (`position_setpoint_triplet_s`), which are then used by the position
controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("navigator", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("fencefile", "load a geofence file from SD card, stored at etc/geofence.txt");
#if CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC
	PRINT_MODULE_USAGE_COMMAND_DESCR("fake_traffic", "run synthetic DAA traffic; use 'navigator fake_traffic help'");
#endif // CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

/**
 * navigator app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int navigator_main(int argc, char *argv[])
{
	return ModuleBase::main(Navigator::desc, argc, argv);
}
