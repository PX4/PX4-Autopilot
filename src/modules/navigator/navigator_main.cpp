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
 */

#include "navigator.h"

#include <float.h>
#include <sys/stat.h>

#include <dataman/dataman.h>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/adsb/AdsbConflict.h>
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

Navigator::Navigator() :
	ModuleParams(nullptr),
	_loop_perf(perf_alloc(PC_ELAPSED, "navigator")),
	_geofence(this),
	_gf_breach_avoidance(this),
	_mission(this),
	_loiter(this),
	_takeoff(this),
	_vtol_takeoff(this),
	_land(this),
	_precland(this),
	_rtl(this)
{
	/* Create a list of our possible navigation types */
	_navigation_mode_array[0] = &_mission;
	_navigation_mode_array[1] = &_loiter;
	_navigation_mode_array[2] = &_rtl;
	_navigation_mode_array[3] = &_takeoff;
	_navigation_mode_array[4] = &_land;
	_navigation_mode_array[5] = &_precland;
	_navigation_mode_array[6] = &_vtol_takeoff;

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

	// Update the timeout used in mission_block (which can't hold it's own parameters)
	_mission.set_payload_deployment_timeout(_param_mis_payload_delivery_timeout.get());

	_adsb_conflict.set_conflict_detection_params(_param_nav_traff_a_hor_ct.get(),
			_param_nav_traff_a_ver.get(),
			_param_nav_traff_collision_time.get(), _param_nav_traff_avoid.get());

	reset_triplets();
}

Navigator::~Navigator()
{
	perf_free(_loop_perf);
	orb_unsubscribe(_local_pos_sub);
	orb_unsubscribe(_mission_sub);
	orb_unsubscribe(_vehicle_status_sub);
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

	_mission.set_payload_deployment_timeout(_param_mis_payload_delivery_timeout.get());
}

void Navigator::run()
{
	bool have_geofence_position_data = false;

	/* Try to load the geofence:
	 * if /fs/microsd/etc/geofence.txt load from this file */
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
			// copy mission to clear any update
			mission_s mission;
			orb_copy(ORB_ID(mission), _mission_sub, &mission);
		}

		/* gps updated */
		if (_gps_pos_sub.updated()) {
			_gps_pos_sub.copy(&_gps_pos);

			if (_geofence.getSource() == Geofence::GF_SOURCE_GPS) {
				have_geofence_position_data = true;
			}
		}

		/* global position updated */
		if (_global_pos_sub.updated()) {
			_global_pos_sub.copy(&_global_pos);

			if (_geofence.getSource() == Geofence::GF_SOURCE_GLOBALPOS) {
				have_geofence_position_data = true;
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
		_home_pos_sub.update(&_home_pos);

		// Handle Vehicle commands
		int vehicle_command_updates = 0;

		while (_vehicle_command_sub.updated() && (vehicle_command_updates < vehicle_command_s::ORB_QUEUE_LENGTH)) {
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
				   && _vstatus.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
				// only update the reposition setpoint if armed, as it otherwise won't get executed until the vehicle switches to loiter,
				// which can lead to dangerous and unexpected behaviors (see loiter.cpp, there is an if(armed) in there too)

				bool reposition_valid = true;

				vehicle_global_position_s position_setpoint{};
				position_setpoint.lat = cmd.param5;
				position_setpoint.lon = cmd.param6;
				position_setpoint.alt = PX4_ISFINITE(cmd.param7) ? cmd.param7 : get_global_position()->alt;

				if (have_geofence_position_data) {
					reposition_valid = geofence_allows_position(position_setpoint);
				}

				if (reposition_valid) {
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
						rep->current.yaw_valid = true;

					} else {
						rep->current.yaw = NAN;
						rep->current.yaw_valid = false;
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

							calculate_breaking_stop(rep->current.lat, rep->current.lon, rep->current.yaw);
							rep->current.yaw_valid = true;

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
							rep->current.loiter_radius = get_loiter_radius();
						}

						rep->current.loiter_direction_counter_clockwise = curr->current.loiter_direction_counter_clockwise;
					}

					rep->previous.timestamp = hrt_absolute_time();

					rep->current.valid = true;
					rep->current.timestamp = hrt_absolute_time();

					rep->next.valid = false;

				} else {
					mavlink_log_critical(&_mavlink_log_pub, "Reposition is outside geofence\t");
					events::send(events::ID("navigator_reposition_outside_geofence"), {events::Log::Error, events::LogInternal::Info},
						     "Reposition is outside geofence");
				}

				// CMD_DO_REPOSITION is acknowledged by commander

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_CHANGE_ALTITUDE
				   && _vstatus.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
				// only update the setpoint if armed, as it otherwise won't get executed until the vehicle switches to loiter,
				// which can lead to dangerous and unexpected behaviors (see loiter.cpp, there is an if(armed) in there too)

				// A VEHICLE_CMD_DO_CHANGE_ALTITUDE has the exact same effect as a VEHICLE_CMD_DO_REPOSITION with only the altitude
				// field populated, this logic is copied from above.

				// only supports MAV_FRAME_GLOBAL and MAV_FRAMEs with absolute altitude amsl

				bool change_altitude_valid = true;

				vehicle_global_position_s position_setpoint{};
				position_setpoint.lat = get_global_position()->lat;
				position_setpoint.lon = get_global_position()->lon;
				position_setpoint.alt = PX4_ISFINITE(cmd.param1) ? cmd.param1 : get_global_position()->alt;

				if (have_geofence_position_data) {
					change_altitude_valid = geofence_allows_position(position_setpoint);
				}

				if (change_altitude_valid) {
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
					rep->current.yaw_valid = false;

					// Position is not changing, thus we keep the setpoint
					rep->current.lat = PX4_ISFINITE(curr->current.lat) ? curr->current.lat : get_global_position()->lat;
					rep->current.lon = PX4_ISFINITE(curr->current.lon) ? curr->current.lon : get_global_position()->lon;

					// set the altitude corresponding to command
					rep->current.alt = PX4_ISFINITE(cmd.param1) ? cmd.param1 : get_global_position()->alt;

					if (_vstatus.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
					    && (get_position_setpoint_triplet()->current.type != position_setpoint_s::SETPOINT_TYPE_TAKEOFF)) {

						calculate_breaking_stop(rep->current.lat, rep->current.lon, rep->current.yaw);
						rep->current.yaw_valid = true;
					}

					if (PX4_ISFINITE(curr->current.loiter_radius) && curr->current.loiter_radius > FLT_EPSILON) {
						rep->current.loiter_radius = curr->current.loiter_radius;

					} else {
						rep->current.loiter_radius = get_loiter_radius();
					}

					rep->current.loiter_direction_counter_clockwise = curr->current.loiter_direction_counter_clockwise;

					rep->previous.timestamp = hrt_absolute_time();

					rep->current.valid = true;
					rep->current.timestamp = hrt_absolute_time();

					rep->next.valid = false;

				} else {
					mavlink_log_critical(&_mavlink_log_pub, "Altitude change is outside geofence\t");
					events::send(events::ID("navigator_change_altitude_outside_geofence"), {events::Log::Error, events::LogInternal::Info},
						     "Altitude change is outside geofence");
				}

				// DO_CHANGE_ALTITUDE is acknowledged by commander

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_ORBIT &&
				   get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {

				// for multicopters the orbit command is directly executed by the orbit flighttask

				bool orbit_location_valid = true;

				vehicle_global_position_s position_setpoint{};
				position_setpoint.lat = PX4_ISFINITE(cmd.param5) ? cmd.param5 : get_global_position()->lat;
				position_setpoint.lon = PX4_ISFINITE(cmd.param6) ? cmd.param6 : get_global_position()->lon;
				position_setpoint.alt = PX4_ISFINITE(cmd.param7) ? cmd.param7 : get_global_position()->alt;

				if (have_geofence_position_data) {
					orbit_location_valid = geofence_allows_position(position_setpoint);
				}

				if (orbit_location_valid) {
					position_setpoint_triplet_s *rep = get_reposition_triplet();
					rep->current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;
					rep->current.loiter_radius = get_loiter_radius();
					rep->current.loiter_direction_counter_clockwise = false;
					rep->current.cruising_throttle = get_cruising_throttle();

					// on entering Loiter mode, reset speed setpoint to default
					if (_navigation_mode != &_loiter) {
						rep->current.cruising_speed = -1.f;

					} else {
						rep->current.cruising_speed = get_cruising_speed();
					}

					if (PX4_ISFINITE(cmd.param1)) {
						rep->current.loiter_radius = fabsf(cmd.param1);
						rep->current.loiter_direction_counter_clockwise = cmd.param1 < 0;
					}

					rep->current.lat = position_setpoint.lat;
					rep->current.lon = position_setpoint.lon;
					rep->current.alt = position_setpoint.alt;

					rep->current.valid = true;
					rep->current.timestamp = hrt_absolute_time();

				} else {
					mavlink_log_critical(&_mavlink_log_pub, "Orbit is outside geofence");
				}

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF) {
				position_setpoint_triplet_s *rep = get_takeoff_triplet();

				// store current position as previous position and goal as next
				rep->previous.yaw = get_local_position()->heading;
				rep->previous.lat = get_global_position()->lat;
				rep->previous.lon = get_global_position()->lon;
				rep->previous.alt = get_global_position()->alt;

				rep->current.loiter_radius = get_loiter_radius();
				rep->current.loiter_direction_counter_clockwise = false;
				rep->current.type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;
				rep->current.cruising_speed = -1.f; // reset to default

				if (home_global_position_valid()) {
					// Only set yaw if we know the true heading
					// We assume that the heading is valid when the global position is valid because true heading
					// is required to fuse NE (e.g.: GNSS) data. // TODO: we should be more explicit here
					rep->current.yaw = cmd.param4;

					rep->previous.valid = true;
					rep->previous.timestamp = hrt_absolute_time();

				} else {
					rep->current.yaw = get_local_position()->heading;
					rep->previous.valid = false;
				}

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

				// CMD_NAV_TAKEOFF is acknowledged by commander

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_NAV_VTOL_TAKEOFF) {

				_vtol_takeoff.setTransitionAltitudeAbsolute(cmd.param7);

				// after the transition the vehicle will establish on a loiter at this position
				_vtol_takeoff.setLoiterLocation(matrix::Vector2d(cmd.param5, cmd.param6));

				// loiter height is the height above takeoff altitude at which the vehicle will establish on a loiter circle
				_vtol_takeoff.setLoiterHeight(cmd.param1);

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_LAND_START) {

				// find NAV_CMD_DO_LAND_START in the mission and
				// use MAV_CMD_MISSION_START to start the mission from the next item containing a position setpoint

				if (_mission.land_start()) {
					vehicle_command_s vcmd = {};
					vcmd.command = vehicle_command_s::VEHICLE_CMD_MISSION_START;
					vcmd.param1 = _mission.get_land_start_index();
					publish_vehicle_cmd(&vcmd);

				} else {
					PX4_WARN("planned mission landing not available");
				}

				publish_vehicle_command_ack(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);

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
					set_cruising_speed();

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
		}

		/* Check for traffic */
		check_traffic();

		/* Check geofence violation */
		geofence_breach_check(have_geofence_position_data);

		/* Do stuff according to navigation state set by commander */
		NavigatorMode *navigation_mode_new{nullptr};

		switch (_vstatus.nav_state) {
		case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
			_pos_sp_triplet_published_invalid_once = false;

			_mission.set_execution_mode(mission_result_s::MISSION_EXECUTION_MODE_NORMAL);
			navigation_mode_new = &_mission;

			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
			_pos_sp_triplet_published_invalid_once = false;
			navigation_mode_new = &_loiter;
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL: {
				_pos_sp_triplet_published_invalid_once = false;
				const bool rtl_activated_now = !_rtl_activated;

				switch (_rtl.get_rtl_type()) {
				case RTL::RTL_TYPE_MISSION_LANDING:
				case RTL::RTL_TYPE_CLOSEST: {
						// If a mission landing is desired we should only execute mission navigation mode if we currently are in fw mode.
						// In multirotor mode no landing pattern is required so we can just navigate to the land point directly and don't need to run mission.
						if (rtl_activated_now) {
							_shouldEngageMissionForLanding = _rtl.getRTLDestinationTypeMission()
											 && _vstatus.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
						}

						if (_shouldEngageMissionForLanding && (on_mission_landing() || _rtl.getRTLState() > RTL::RTL_STATE_CLIMB)) {

							// already in a mission landing, we just need to inform the user and stay in mission
							if (rtl_activated_now) {
								mavlink_log_info(get_mavlink_log_pub(), "RTL to Mission landing, continue landing\t");
								events::send(events::ID("rtl_land_at_mission_continue_landing"), events::Log::Info,
									     "RTL to Mission landing, continue landing");
							}

							if (_navigation_mode != &_mission) {
								// the first time we're here start the mission landig
								start_mission_landing();
							}

							_mission.set_execution_mode(mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD);
							navigation_mode_new = &_mission;

						} else {
							navigation_mode_new = &_rtl;
						}

						break;
					}

				case RTL::RTL_TYPE_MISSION_LANDING_REVERSED:
					if (_mission.get_land_start_available() && !get_land_detected()->landed) {
						// the mission contains a landing spot
						_mission.set_execution_mode(mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD);

						if (_navigation_mode != &_mission) {
							if (_navigation_mode == nullptr) {
								// switching from an manual mode, go to landing if not already landing
								if (!on_mission_landing()) {
									start_mission_landing();
								}

							} else {
								// switching from an auto mode, continue the mission from the closest item
								_mission.set_closest_item_as_current();
							}
						}

						if (rtl_activated_now) {
							mavlink_log_info(get_mavlink_log_pub(), "RTL Mission activated, continue mission\t");
							events::send(events::ID("navigator_rtl_mission_activated"), events::Log::Info,
								     "RTL Mission activated, continue mission");
						}

						navigation_mode_new = &_mission;

					} else {
						// fly the mission in reverse if switching from a non-manual mode
						_mission.set_execution_mode(mission_result_s::MISSION_EXECUTION_MODE_REVERSE);

						if ((_navigation_mode != nullptr && (_navigation_mode != &_rtl || _mission.get_mission_changed())) &&
						    (! _mission.get_mission_finished()) &&
						    (!get_land_detected()->landed)) {
							// determine the closest mission item if switching from a non-mission mode, and we are either not already
							// mission mode or the mission waypoints changed.
							// The seconds condition is required so that when no mission was uploaded and one is available the closest
							// mission item is determined and also that if the user changes the active mission index while rtl is active
							// always that waypoint is tracked first.
							if ((_navigation_mode != &_mission) && (rtl_activated_now || _mission.get_mission_waypoints_changed())) {
								_mission.set_closest_item_as_current();
							}

							if (rtl_activated_now) {
								mavlink_log_info(get_mavlink_log_pub(), "RTL Mission activated, fly mission in reverse\t");
								events::send(events::ID("navigator_rtl_mission_activated_rev"), events::Log::Info,
									     "RTL Mission activated, fly mission in reverse");
							}

							navigation_mode_new = &_mission;

						} else {
							if (rtl_activated_now) {
								mavlink_log_info(get_mavlink_log_pub(), "RTL Mission activated, fly to home\t");
								events::send(events::ID("navigator_rtl_mission_activated_home"), events::Log::Info,
									     "RTL Mission activated, fly to home");
							}

							navigation_mode_new = &_rtl;
						}
					}

					break;

				default:
					if (rtl_activated_now) {
						mavlink_log_info(get_mavlink_log_pub(), "RTL HOME activated\t");
						events::send(events::ID("navigator_rtl_home_activated"), events::Log::Info, "RTL activated");
					}

					navigation_mode_new = &_rtl;
					break;

				}

				_rtl_activated = true;
				break;
			}

		case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
			_pos_sp_triplet_published_invalid_once = false;
			navigation_mode_new = &_takeoff;
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF:
			_pos_sp_triplet_published_invalid_once = false;
			navigation_mode_new = &_vtol_takeoff;
			break;

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
		case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		case vehicle_status_s::NAVIGATION_STATE_DESCEND:
		case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
		case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		case vehicle_status_s::NAVIGATION_STATE_STAB:
		default:
			navigation_mode_new = nullptr;
			_can_loiter_at_sp = false;
			break;
		}

		if (_vstatus.nav_state != vehicle_status_s::NAVIGATION_STATE_AUTO_RTL) {
			_rtl_activated = false;
			_rtl.resetRtlState();
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


			// transition to hover in Descend mode
			if (_vstatus.nav_state == vehicle_status_s::NAVIGATION_STATE_DESCEND &&
			    _vstatus.is_vtol && _vstatus.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING &&
			    force_vtol()) {
				vehicle_command_s vcmd = {};
				vcmd.command = NAV_CMD_DO_VTOL_TRANSITION;
				vcmd.param1 = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
				publish_vehicle_cmd(&vcmd);
				mavlink_log_info(&_mavlink_log_pub, "Transition to hover mode and descend.\t");
				events::send(events::ID("navigator_transition_descend"), events::Log::Critical,
					     "Transition to hover mode and descend");
			}

		}

		_navigation_mode = navigation_mode_new;

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

		perf_end(_loop_perf);
	}
}

void Navigator::geofence_breach_check(bool &have_geofence_position_data)
{
	if (have_geofence_position_data &&
	    (_geofence.getGeofenceAction() != geofence_result_s::GF_ACTION_NONE) &&
	    (hrt_elapsed_time(&_last_geofence_check) > GEOFENCE_CHECK_INTERVAL_US)) {

		const position_controller_status_s &pos_ctrl_status = _position_controller_status_sub.get();

		matrix::Vector2<double> fence_violation_test_point;
		geofence_violation_type_u gf_violation_type{};
		float test_point_bearing;
		float test_point_distance;
		float vertical_test_point_distance;

		if (_vstatus.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
			test_point_bearing = atan2f(_local_pos.vy, _local_pos.vx);
			const float velocity_hor_abs = sqrtf(_local_pos.vx * _local_pos.vx + _local_pos.vy * _local_pos.vy);
			_gf_breach_avoidance.setHorizontalVelocity(velocity_hor_abs);
			_gf_breach_avoidance.setClimbRate(-_local_pos.vz);
			test_point_distance = _gf_breach_avoidance.computeBrakingDistanceMultirotor();
			vertical_test_point_distance = _gf_breach_avoidance.computeVerticalBrakingDistanceMultirotor();

		} else {
			test_point_distance = 2.0f * get_loiter_radius();
			vertical_test_point_distance = 5.0f;

			if (hrt_absolute_time() - pos_ctrl_status.timestamp < 100000 && PX4_ISFINITE(pos_ctrl_status.nav_bearing)) {
				test_point_bearing = pos_ctrl_status.nav_bearing;

			} else {
				test_point_bearing = atan2f(_local_pos.vy, _local_pos.vx);
			}
		}

		_gf_breach_avoidance.setHorizontalTestPointDistance(test_point_distance);
		_gf_breach_avoidance.setVerticalTestPointDistance(vertical_test_point_distance);
		_gf_breach_avoidance.setTestPointBearing(test_point_bearing);
		_gf_breach_avoidance.setCurrentPosition(_global_pos.lat, _global_pos.lon, _global_pos.alt);
		_gf_breach_avoidance.setMaxHorDistHome(_geofence.getMaxHorDistanceHome());
		_gf_breach_avoidance.setMaxVerDistHome(_geofence.getMaxVerDistanceHome());

		if (home_global_position_valid()) {
			_gf_breach_avoidance.setHomePosition(_home_pos.lat, _home_pos.lon, _home_pos.alt);
		}

		if (_geofence.getPredict()) {
			fence_violation_test_point = _gf_breach_avoidance.getFenceViolationTestPoint();

		} else {
			fence_violation_test_point = matrix::Vector2d(_global_pos.lat, _global_pos.lon);
			vertical_test_point_distance = 0;
		}

		gf_violation_type.flags.dist_to_home_exceeded = !_geofence.isCloserThanMaxDistToHome(fence_violation_test_point(0),
				fence_violation_test_point(1),
				_global_pos.alt);

		gf_violation_type.flags.max_altitude_exceeded = !_geofence.isBelowMaxAltitude(_global_pos.alt +
				vertical_test_point_distance);

		gf_violation_type.flags.fence_violation = !_geofence.isInsidePolygonOrCircle(fence_violation_test_point(0),
				fence_violation_test_point(1),
				_global_pos.alt);

		_last_geofence_check = hrt_absolute_time();
		have_geofence_position_data = false;

		_geofence_result.timestamp = hrt_absolute_time();
		_geofence_result.primary_geofence_action = _geofence.getGeofenceAction();
		_geofence_result.home_required = _geofence.isHomeRequired();

		if (gf_violation_type.value) {
			/* inform other apps via the mission result */
			_geofence_result.primary_geofence_breached = true;

			using geofence_violation_reason_t = events::px4::enums::geofence_violation_reason_t;

			if (gf_violation_type.flags.fence_violation) {
				_geofence_result.geofence_violation_reason = (uint8_t)geofence_violation_reason_t::fence_violation;

			} else if (gf_violation_type.flags.max_altitude_exceeded) {
				_geofence_result.geofence_violation_reason = (uint8_t)geofence_violation_reason_t::max_altitude_exceeded;

			} else if (gf_violation_type.flags.dist_to_home_exceeded) {
				_geofence_result.geofence_violation_reason = (uint8_t)geofence_violation_reason_t::dist_to_home_exceeded;

			}

			/* Issue a warning about the geofence violation once and only if we are armed */
			if (!_geofence_violation_warning_sent && _vstatus.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {

				// we have predicted a geofence violation and if the action is to loiter then
				// demand a reposition to a location which is inside the geofence
				if (_geofence.getGeofenceAction() == geofence_result_s::GF_ACTION_LOITER) {
					position_setpoint_triplet_s *rep = get_reposition_triplet();

					matrix::Vector2<double> loiter_center_lat_lon;
					matrix::Vector2<double> current_pos_lat_lon(_global_pos.lat, _global_pos.lon);
					float loiter_altitude_amsl = _global_pos.alt;


					if (_vstatus.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
						// the computation of the braking distance does not match the actual braking distance. Until we have a better model
						// we set the loiter point to the current position, that will make sure that the vehicle will loiter inside the fence
						loiter_center_lat_lon =  _gf_breach_avoidance.generateLoiterPointForMultirotor(gf_violation_type,
									 &_geofence);

						loiter_altitude_amsl = _gf_breach_avoidance.generateLoiterAltitudeForMulticopter(gf_violation_type);

					} else {

						loiter_center_lat_lon = _gf_breach_avoidance.generateLoiterPointForFixedWing(gf_violation_type, &_geofence);
						loiter_altitude_amsl = _gf_breach_avoidance.generateLoiterAltitudeForFixedWing(gf_violation_type);
					}

					rep->current.timestamp = hrt_absolute_time();
					rep->current.yaw = get_local_position()->heading;
					rep->current.yaw_valid = true;
					rep->current.lat = loiter_center_lat_lon(0);
					rep->current.lon = loiter_center_lat_lon(1);
					rep->current.alt = loiter_altitude_amsl;
					rep->current.valid = true;
					rep->current.loiter_radius = get_loiter_radius();
					rep->current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;
					rep->current.cruising_throttle = get_cruising_throttle();
					rep->current.acceptance_radius = get_acceptance_radius();
					rep->current.cruising_speed = get_cruising_speed();

				}

				_geofence_violation_warning_sent = true;
			}

		} else {
			/* inform other apps via the mission result */
			_geofence_result.primary_geofence_breached = false;

			/* Reset the _geofence_violation_warning_sent field */
			_geofence_violation_warning_sent = false;
		}

		_geofence_result_pub.publish(_geofence_result);
	}
}

int Navigator::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("navigator",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_NAVIGATION,
				      PX4_STACK_ADJUSTED(1952),
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
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
	return 0;
}

void Navigator::publish_position_setpoint_triplet()
{
	_pos_sp_triplet.timestamp = hrt_absolute_time();
	_pos_sp_triplet_pub.publish(_pos_sp_triplet);
	_pos_sp_triplet_updated = false;
}

float Navigator::get_default_acceptance_radius()
{
	return _param_nav_acc_rad.get();
}

float Navigator::get_altitude_acceptance_radius()
{
	if (get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
		const position_setpoint_s &next_sp = get_position_setpoint_triplet()->next;

		if (!force_vtol() && next_sp.type == position_setpoint_s::SETPOINT_TYPE_LAND && next_sp.valid) {
			// Use separate (tighter) altitude acceptance for clean altitude starting point before FW landing
			return _param_nav_fw_altl_rad.get();

		} else {
			return _param_nav_fw_alt_rad.get();
		}

	} else if (get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROVER) {
		return INFINITY;

	} else {
		float alt_acceptance_radius = _param_nav_mc_alt_rad.get();

		const position_controller_status_s &pos_ctrl_status = _position_controller_status_sub.get();

		if ((pos_ctrl_status.timestamp > _pos_sp_triplet.timestamp)
		    && pos_ctrl_status.altitude_acceptance > alt_acceptance_radius) {
			alt_acceptance_radius = pos_ctrl_status.altitude_acceptance;
		}

		return alt_acceptance_radius;
	}
}

float Navigator::get_cruising_speed()
{
	/* there are three options: The mission-requested cruise speed, or the current hover / plane speed */
	if (_vstatus.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		if (_mission_cruising_speed_mc > 0.0f) {
			return _mission_cruising_speed_mc;

		} else {
			return -1.0f;
		}

	} else {
		if (_mission_cruising_speed_fw > 0.0f) {
			return _mission_cruising_speed_fw;

		} else {
			return -1.0f;
		}
	}
}

void Navigator::set_cruising_speed(float speed)
{
	if (_vstatus.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		_mission_cruising_speed_mc = speed;

	} else {
		_mission_cruising_speed_fw = speed;
	}
}

void Navigator::reset_cruising_speed()
{
	_mission_cruising_speed_mc = -1.0f;
	_mission_cruising_speed_fw = -1.0f;
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
	sp.loiter_radius = get_loiter_radius();
	sp.acceptance_radius = get_default_acceptance_radius();
	sp.cruising_speed = get_cruising_speed();
	sp.cruising_throttle = get_cruising_throttle();
	sp.valid = false;
	sp.type = position_setpoint_s::SETPOINT_TYPE_IDLE;
	sp.disable_weather_vane = false;
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
	const position_controller_status_s &pos_ctrl_status = _position_controller_status_sub.get();

	// for fixed-wing and rover, return the max of NAV_ACC_RAD and the controller acceptance radius (e.g. navigation switch distance)
	if (_vstatus.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
	    && PX4_ISFINITE(pos_ctrl_status.acceptance_radius) && pos_ctrl_status.timestamp != 0) {

		acceptance_radius = math::max(acceptance_radius, pos_ctrl_status.acceptance_radius);
	}

	return acceptance_radius;
}

float Navigator::get_yaw_acceptance(float mission_item_yaw)
{
	float yaw = mission_item_yaw;

	const position_controller_status_s &pos_ctrl_status = _position_controller_status_sub.get();

	// if yaw_acceptance from position controller is NaN overwrite the mission item yaw such that
	// the waypoint can be reached from any direction
	if ((pos_ctrl_status.timestamp > _pos_sp_triplet.timestamp) && !PX4_ISFINITE(pos_ctrl_status.yaw_acceptance)) {
		yaw = pos_ctrl_status.yaw_acceptance;
	}

	return yaw;
}

void Navigator::load_fence_from_file(const char *filename)
{
	_geofence.loadFromFile(filename);
}

void Navigator::take_traffic_conflict_action()
{

	vehicle_command_s vcmd = {};

	switch (_adsb_conflict._conflict_detection_params.traffic_avoidance_mode) {

	case 2: {
			_rtl.set_return_alt_min(true);
			vcmd.command = vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH;
			publish_vehicle_cmd(&vcmd);
			break;
		}

	case 3: {
			vcmd.command = vehicle_command_s::VEHICLE_CMD_NAV_LAND;
			publish_vehicle_cmd(&vcmd);
			break;

		}

	case 4: {

			vcmd.command = vehicle_command_s::VEHICLE_CMD_NAV_LOITER_UNLIM;
			publish_vehicle_cmd(&vcmd);
			break;

		}
	}

}

void Navigator::run_fake_traffic()
{

	_adsb_conflict.run_fake_traffic(get_global_position()->lat, get_global_position()->lon,
					get_global_position()->alt);
}

void Navigator::check_traffic()
{

	if (_traffic_sub.updated()) {

		_traffic_sub.copy(&_adsb_conflict._transponder_report);

		uint16_t required_flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS |
					  transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING |
					  transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY | transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE;

		if ((_adsb_conflict._transponder_report.flags & required_flags) == required_flags) {

			_adsb_conflict.detect_traffic_conflict(get_global_position()->lat, get_global_position()->lon,
							       get_global_position()->alt, _local_pos.vx, _local_pos.vy, _local_pos.vz);

			if (_adsb_conflict.handle_traffic_conflict()) {
				take_traffic_conflict_action();
			}
		}
	}
}

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
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	if (!strcmp(argv[0], "fencefile")) {
		get_instance()->load_fence_from_file(GEOFENCE_FILENAME);
		return 0;

	} else if (!strcmp(argv[0], "fake_traffic")) {

		get_instance()->run_fake_traffic();

		return 0;
	}

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

void Navigator::publish_vehicle_cmd(vehicle_command_s *vcmd)
{
	vcmd->timestamp = hrt_absolute_time();
	vcmd->source_system = _vstatus.system_id;
	vcmd->source_component = _vstatus.component_id;
	vcmd->target_system = _vstatus.system_id;
	vcmd->confirmation = false;
	vcmd->from_external = false;

	// The camera commands are not processed on the autopilot but will be
	// sent to the mavlink links to other components.
	switch (vcmd->command) {
	case NAV_CMD_IMAGE_START_CAPTURE:

		if (static_cast<int>(vcmd->param3) == 1) {
			// When sending a single capture we need to include the sequence number, thus camera_trigger needs to handle this cmd
			vcmd->param1 = 0.0f;
			vcmd->param2 = 0.0f;
			vcmd->param3 = 0.0f;
			vcmd->param4 = 0.0f;
			vcmd->param5 = 1.0;
			vcmd->param6 = 0.0;
			vcmd->param7 = 0.0f;
			vcmd->command = vehicle_command_s::VEHICLE_CMD_DO_DIGICAM_CONTROL;

		} else {
			// We are only capturing multiple if param3 is 0 or > 1.
			// For multiple pictures the sequence number does not need to be included, thus there is no need to go through camera_trigger
			_is_capturing_images = true;
		}

		vcmd->target_component = 100; // MAV_COMP_ID_CAMERA
		break;

	case NAV_CMD_IMAGE_STOP_CAPTURE:
		_is_capturing_images = false;
		vcmd->target_component = 100; // MAV_COMP_ID_CAMERA
		break;

	case NAV_CMD_VIDEO_START_CAPTURE:
	case NAV_CMD_VIDEO_STOP_CAPTURE:
		vcmd->target_component = 100; // MAV_COMP_ID_CAMERA
		break;

	default:
		vcmd->target_component = _vstatus.component_id;
		break;
	}

	_vehicle_cmd_pub.publish(*vcmd);
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
	vehicle_command_s vcmd = {};
	vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE;
	vcmd.param1 = _vstatus.system_id;
	vcmd.param2 = _vstatus.component_id;
	vcmd.param3 = -1.0f; // Leave unchanged.
	vcmd.param4 = -1.0f; // Leave unchanged.
	publish_vehicle_cmd(&vcmd);
}

void Navigator::release_gimbal_control()
{
	vehicle_command_s vcmd = {};
	vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE;
	vcmd.param1 = -3.0f; // Remove control if it had it.
	vcmd.param2 = -3.0f; // Remove control if it had it.
	vcmd.param3 = -1.0f; // Leave unchanged.
	vcmd.param4 = -1.0f; // Leave unchanged.
	publish_vehicle_cmd(&vcmd);
}


void
Navigator::stop_capturing_images()
{
	if (_is_capturing_images) {
		vehicle_command_s vcmd = {};
		vcmd.command = NAV_CMD_IMAGE_STOP_CAPTURE;
		vcmd.param1 = 0.0f;
		publish_vehicle_cmd(&vcmd);

		// _is_capturing_images is reset inside publish_vehicle_cmd.
	}
}

bool Navigator::geofence_allows_position(const vehicle_global_position_s &pos)
{
	if ((_geofence.getGeofenceAction() != geofence_result_s::GF_ACTION_NONE) &&
	    (_geofence.getGeofenceAction() != geofence_result_s::GF_ACTION_WARN)) {

		if (PX4_ISFINITE(pos.lat) && PX4_ISFINITE(pos.lon)) {
			return _geofence.check(pos, _gps_pos);
		}
	}

	return true;
}

void Navigator::calculate_breaking_stop(double &lat, double &lon, float &yaw)
{
	// For multirotors we need to account for the braking distance, otherwise the vehicle will overshoot and go back
	float course_over_ground = atan2f(_local_pos.vy, _local_pos.vx);

	// predict braking distance

	const float velocity_hor_abs = sqrtf(_local_pos.vx * _local_pos.vx + _local_pos.vy * _local_pos.vy);

	float multirotor_braking_distance = math::trajectory::computeBrakingDistanceFromVelocity(velocity_hor_abs,
					    _param_mpc_jerk_auto, _param_mpc_acc_hor, 0.6f * _param_mpc_jerk_auto);

	waypoint_from_heading_and_distance(get_global_position()->lat, get_global_position()->lon, course_over_ground,
					   multirotor_braking_distance, &lat, &lon);
	yaw = get_local_position()->heading;
}

void Navigator::mode_completed(uint8_t nav_state, uint8_t result)
{
	mode_completed_s mode_completed{};
	mode_completed.timestamp = hrt_absolute_time();
	mode_completed.result = result;
	mode_completed.nav_state = nav_state;
	_mode_completed_pub.publish(mode_completed);
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
	PRINT_MODULE_USAGE_COMMAND_DESCR("fake_traffic", "publishes 4 fake transponder_report_s uORB messages");
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
	return Navigator::main(argc, argv);
}
