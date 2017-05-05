/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Jean Cyr <jean.m.cyr@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#include "navigator.h"

#include <cfloat>

#include <dataman/dataman.h>
#include <drivers/drv_hrt.h>
#include <geo/geo.h>
#include <mathlib/mathlib.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <systemlib/mavlink_log.h>
#include <systemlib/systemlib.h>
#include <uORB/topics/fence.h>
#include <uORB/topics/fw_pos_ctrl_status.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/uORB.h>

/**
 * navigator app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int navigator_main(int argc, char *argv[]);

#define GEOFENCE_CHECK_INTERVAL 200000

namespace navigator
{
Navigator	*g_navigator;
}

Navigator::Navigator() :
	SuperBlock(nullptr, "NAV"),
	_loop_perf(perf_alloc(PC_ELAPSED, "navigator")),
	_geofence(this),
	_mission(this, "MIS"),
	_loiter(this, "LOI"),
	_takeoff(this, "TKF"),
	_land(this, "LND"),
	_rtl(this, "RTL"),
	_rcLoss(this, "RCL"),
	_dataLinkLoss(this, "DLL"),
	_engineFailure(this, "EF"),
	_gpsFailure(this, "GPSF"),
	_follow_target(this, "TAR"),
	_param_loiter_radius(this, "LOITER_RAD"),
	_param_acceptance_radius(this, "ACC_RAD"),
	_param_fw_alt_acceptance_radius(this, "FW_ALT_RAD"),
	_param_mc_alt_acceptance_radius(this, "MC_ALT_RAD")
{
	/* Create a list of our possible navigation types */
	_navigation_mode_array[0] = &_mission;
	_navigation_mode_array[1] = &_loiter;
	_navigation_mode_array[2] = &_rtl;
	_navigation_mode_array[3] = &_dataLinkLoss;
	_navigation_mode_array[4] = &_engineFailure;
	_navigation_mode_array[5] = &_gpsFailure;
	_navigation_mode_array[6] = &_rcLoss;
	_navigation_mode_array[7] = &_takeoff;
	_navigation_mode_array[8] = &_land;
	_navigation_mode_array[9] = &_follow_target;

	updateParams();
}

Navigator::~Navigator()
{
	if (_navigator_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_navigator_task);
				break;
			}
		} while (_navigator_task != -1);
	}

	navigator::g_navigator = nullptr;
}

void
Navigator::global_position_update()
{
	orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
}

void
Navigator::local_position_update()
{
	orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
}

void
Navigator::gps_position_update()
{
	orb_copy(ORB_ID(vehicle_gps_position), _gps_pos_sub, &_gps_pos);
}

void
Navigator::sensor_combined_update()
{
	orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor_combined);
}

void
Navigator::home_position_update(bool force)
{
	bool updated = false;
	orb_check(_home_pos_sub, &updated);

	if (updated || force) {
		orb_copy(ORB_ID(home_position), _home_pos_sub, &_home_pos);
	}
}

void
Navigator::fw_pos_ctrl_status_update(bool force)
{
	bool updated = false;
	orb_check(_fw_pos_ctrl_status_sub, &updated);

	if (updated || force) {
		orb_copy(ORB_ID(fw_pos_ctrl_status), _fw_pos_ctrl_status_sub, &_fw_pos_ctrl_status);
	}
}

void
Navigator::vehicle_status_update()
{
	if (orb_copy(ORB_ID(vehicle_status), _vstatus_sub, &_vstatus) != OK) {
		/* in case the commander is not be running */
		_vstatus.arming_state = vehicle_status_s::ARMING_STATE_STANDBY;
	}
}

void
Navigator::vehicle_land_detected_update()
{
	orb_copy(ORB_ID(vehicle_land_detected), _land_detected_sub, &_land_detected);
}

void
Navigator::params_update()
{
	parameter_update_s param_update;
	orb_copy(ORB_ID(parameter_update), _param_update_sub, &param_update);
	updateParams();

	if (_navigation_mode) {
		_navigation_mode->updateParams();
	}
}

void
Navigator::task_main_trampoline(int argc, char *argv[])
{
	navigator::g_navigator->task_main();
}

void
Navigator::task_main()
{
	bool have_geofence_position_data = false;

	/* Try to load the geofence:
	 * if /fs/microsd/etc/geofence.txt load from this file
	 * else clear geofence data in datamanager */
	struct stat buffer;

	if (stat(GEOFENCE_FILENAME, &buffer) == 0) {
		PX4_INFO("Try to load geofence.txt");
		_geofence.loadFromFile(GEOFENCE_FILENAME);

	} else {
		if (_geofence.clearDm() != OK) {
			mavlink_log_critical(&_mavlink_log_pub, "failed clearing geofence");
		}
	}

	/* do subscriptions */
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_gps_pos_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	_fw_pos_ctrl_status_sub = orb_subscribe(ORB_ID(fw_pos_ctrl_status));
	_vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));
	_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_home_pos_sub = orb_subscribe(ORB_ID(home_position));
	_onboard_mission_sub = orb_subscribe(ORB_ID(onboard_mission));
	_offboard_mission_sub = orb_subscribe(ORB_ID(offboard_mission));
	_param_update_sub = orb_subscribe(ORB_ID(parameter_update));
	_vehicle_command_sub = orb_subscribe(ORB_ID(vehicle_command));

	/* copy all topics first time */
	vehicle_status_update();
	vehicle_land_detected_update();
	global_position_update();
	local_position_update();
	gps_position_update();
	sensor_combined_update();
	home_position_update(true);
	fw_pos_ctrl_status_update(true);
	params_update();

	/* wakeup source(s) */
	px4_pollfd_struct_t fds[1] = {};

	/* Setup of loop */
	fds[0].fd = _global_pos_sub;
	fds[0].events = POLLIN;

	bool global_pos_available_once = false;

	/* rate-limit global pos subscription to 20 Hz / 50 ms */
	orb_set_interval(_global_pos_sub, 49);

	while (!_task_should_exit) {

		/* wait for up to 1000ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			/* timed out - periodic check for _task_should_exit, etc. */
			if (global_pos_available_once) {
				global_pos_available_once = false;
				PX4_WARN("global position timeout");
			}

			/* Let the loop run anyway, don't do `continue` here. */

		} else if (pret < 0) {
			/* this is undesirable but not much we can do - might want to flag unhappy status */
			PX4_ERR("nav: poll error %d, %d", pret, errno);
			usleep(10000);
			continue;

		} else {

			if (fds[0].revents & POLLIN) {
				/* success, global pos is available */
				global_position_update();

				if (_geofence.getSource() == Geofence::GF_SOURCE_GLOBALPOS) {
					have_geofence_position_data = true;
				}

				global_pos_available_once = true;
			}
		}

		perf_begin(_loop_perf);

		bool updated;

		/* gps updated */
		orb_check(_gps_pos_sub, &updated);

		if (updated) {
			gps_position_update();

			if (_geofence.getSource() == Geofence::GF_SOURCE_GPS) {
				have_geofence_position_data = true;
			}
		}

		/* local position updated */
		orb_check(_local_pos_sub, &updated);

		if (updated) {
			local_position_update();
		}

		/* sensors combined updated */
		orb_check(_sensor_combined_sub, &updated);

		if (updated) {
			sensor_combined_update();
		}

		/* parameters updated */
		orb_check(_param_update_sub, &updated);

		if (updated) {
			params_update();
		}

		/* vehicle status updated */
		orb_check(_vstatus_sub, &updated);

		if (updated) {
			vehicle_status_update();
		}

		/* vehicle land detected updated */
		orb_check(_land_detected_sub, &updated);

		if (updated) {
			vehicle_land_detected_update();
		}

		/* navigation capabilities updated */
		orb_check(_fw_pos_ctrl_status_sub, &updated);

		if (updated) {
			fw_pos_ctrl_status_update();
		}

		/* home position updated */
		orb_check(_home_pos_sub, &updated);

		if (updated) {
			home_position_update();
		}

		orb_check(_vehicle_command_sub, &updated);

		if (updated) {
			vehicle_command_s cmd = {};
			orb_copy(ORB_ID(vehicle_command), _vehicle_command_sub, &cmd);

			if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_REPOSITION) {

				struct position_setpoint_triplet_s *rep = get_reposition_triplet();

				// store current position as previous position and goal as next
				rep->previous.yaw = get_global_position()->yaw;
				rep->previous.lat = get_global_position()->lat;
				rep->previous.lon = get_global_position()->lon;
				rep->previous.alt = get_global_position()->alt;

				rep->current.loiter_radius = get_loiter_radius();
				rep->current.loiter_direction = 1;
				rep->current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;

				// Go on and check which changes had been requested
				if (PX4_ISFINITE(cmd.param4)) {
					rep->current.yaw = cmd.param4;

				} else {
					rep->current.yaw = NAN;
				}

				if (PX4_ISFINITE(cmd.param5) && PX4_ISFINITE(cmd.param6)) {
					rep->current.lat = (cmd.param5 < 1000) ? cmd.param5 : cmd.param5 / (double)1e7;
					rep->current.lon = (cmd.param6 < 1000) ? cmd.param6 : cmd.param6 / (double)1e7;

				} else {
					rep->current.lat = get_global_position()->lat;
					rep->current.lon = get_global_position()->lon;
				}

				if (PX4_ISFINITE(cmd.param7)) {
					rep->current.alt = cmd.param7;

				} else {
					rep->current.alt = get_global_position()->alt;
				}

				rep->previous.valid = true;
				rep->current.valid = true;
				rep->next.valid = false;

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF) {
				struct position_setpoint_triplet_s *rep = get_takeoff_triplet();

				// store current position as previous position and goal as next
				rep->previous.yaw = get_global_position()->yaw;
				rep->previous.lat = get_global_position()->lat;
				rep->previous.lon = get_global_position()->lon;
				rep->previous.alt = get_global_position()->alt;

				rep->current.loiter_radius = get_loiter_radius();
				rep->current.loiter_direction = 1;
				rep->current.type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;

				if (home_position_valid()) {
					rep->current.yaw = cmd.param4;
					rep->previous.valid = true;

				} else {
					rep->current.yaw = get_local_position()->yaw;
					rep->previous.valid = false;
				}

				if (PX4_ISFINITE(cmd.param5) && PX4_ISFINITE(cmd.param6)) {
					rep->current.lat = (cmd.param5 < 1000) ? cmd.param5 : cmd.param5 / (double)1e7;
					rep->current.lon = (cmd.param6 < 1000) ? cmd.param6 : cmd.param6 / (double)1e7;

				} else {
					// If one of them is non-finite, reset both
					rep->current.lat = NAN;
					rep->current.lon = NAN;
				}

				rep->current.alt = cmd.param7;

				rep->current.valid = true;
				rep->next.valid = false;

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_LAND_START) {

				/* find NAV_CMD_DO_LAND_START in the mission and
				 * use MAV_CMD_MISSION_START to start the mission there
				 */
				int land_start = _mission.find_offboard_land_start();

				if (land_start != -1) {
					vehicle_command_s vcmd = {};
					vcmd.target_system = get_vstatus()->system_id;
					vcmd.target_component = get_vstatus()->component_id;
					vcmd.command = vehicle_command_s::VEHICLE_CMD_MISSION_START;
					vcmd.param1 = land_start;
					vcmd.param2 = 0;

					publish_vehicle_cmd(vcmd);

				} else {
					PX4_WARN("planned landing not available");
				}

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH) {

				vehicle_command_s vcmd = {};
				vcmd.target_system = get_vstatus()->system_id;
				vcmd.target_component = get_vstatus()->component_id;
				vcmd.command = vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH;

				orb_advert_t pub = orb_advertise_queue(ORB_ID(vehicle_command), &vcmd, vehicle_command_s::ORB_QUEUE_LENGTH);
				(void)orb_unadvertise(pub);

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_MISSION_START) {

				if (get_mission_result()->valid &&
				    PX4_ISFINITE(cmd.param1) && (cmd.param1 >= 0) && (cmd.param1 < _mission_result.seq_total)) {

					_mission.set_current_offboard_mission_index(cmd.param1);
				}

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_PAUSE_CONTINUE) {
				warnx("navigator: got pause/continue command");

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
			}
		}

		/* Check geofence violation */
		static hrt_abstime last_geofence_check = 0;

		if (have_geofence_position_data &&
		    (_geofence.getGeofenceAction() != geofence_result_s::GF_ACTION_NONE) &&
		    (hrt_elapsed_time(&last_geofence_check) > GEOFENCE_CHECK_INTERVAL)) {

			bool inside = _geofence.inside(_global_pos, _gps_pos, _sensor_combined.baro_alt_meter);
			last_geofence_check = hrt_absolute_time();
			have_geofence_position_data = false;

			_geofence_result.timestamp = hrt_absolute_time();
			_geofence_result.geofence_action = _geofence.getGeofenceAction();
			_geofence_result.home_required = _geofence.isHomeRequired();

			if (!inside) {
				/* inform other apps via the mission result */
				_geofence_result.geofence_violated = true;

				/* Issue a warning about the geofence violation once */
				if (!_geofence_violation_warning_sent) {
					mavlink_log_critical(&_mavlink_log_pub, "Geofence violation");
					_geofence_violation_warning_sent = true;
				}

			} else {
				/* inform other apps via the mission result */
				_geofence_result.geofence_violated = false;

				/* Reset the _geofence_violation_warning_sent field */
				_geofence_violation_warning_sent = false;
			}

			publish_geofence_result();
		}

		/* Do stuff according to navigation state set by commander */
		switch (_vstatus.nav_state) {
		case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
			_pos_sp_triplet_published_invalid_once = false;
			_navigation_mode = &_mission;
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
			_pos_sp_triplet_published_invalid_once = false;
			_navigation_mode = &_loiter;
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_RCRECOVER:
			_pos_sp_triplet_published_invalid_once = false;
			_navigation_mode = &_rcLoss;
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
			_pos_sp_triplet_published_invalid_once = false;
			_navigation_mode = &_rtl;
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
			_pos_sp_triplet_published_invalid_once = false;
			_navigation_mode = &_takeoff;
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
			_pos_sp_triplet_published_invalid_once = false;
			_navigation_mode = &_land;
			break;

		case vehicle_status_s::NAVIGATION_STATE_DESCEND:
			_pos_sp_triplet_published_invalid_once = false;
			_navigation_mode = &_land;
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_RTGS:
			_pos_sp_triplet_published_invalid_once = false;
			_navigation_mode = &_dataLinkLoss;
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL:
			_pos_sp_triplet_published_invalid_once = false;
			_navigation_mode = &_engineFailure;
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL:
			_pos_sp_triplet_published_invalid_once = false;
			_navigation_mode = &_gpsFailure;
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
			_pos_sp_triplet_published_invalid_once = false;
			_navigation_mode = &_follow_target;
			break;

		case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		case vehicle_status_s::NAVIGATION_STATE_ACRO:
		case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
		case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		case vehicle_status_s::NAVIGATION_STATE_STAB:
		default:
			_pos_sp_triplet_published_invalid_once = false;
			_navigation_mode = nullptr;
			_can_loiter_at_sp = false;
			break;
		}

		/* iterate through navigation modes and set active/inactive for each */
		for (unsigned int i = 0; i < NAVIGATOR_MODE_ARRAY_SIZE; i++) {
			_navigation_mode_array[i]->run(_navigation_mode == _navigation_mode_array[i]);
		}

		/* if we landed and have not received takeoff setpoint then stay in idle */
		if (_land_detected.landed &&
		    !((_vstatus.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF)
		      || (_vstatus.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION))) {

			_pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_IDLE;
			_pos_sp_triplet.current.valid = true;
			_pos_sp_triplet.previous.valid = false;
			_pos_sp_triplet.next.valid = false;

		}

		/* if nothing is running, set position setpoint triplet invalid once */
		if (_navigation_mode == nullptr && !_pos_sp_triplet_published_invalid_once) {
			_pos_sp_triplet_published_invalid_once = true;
			_pos_sp_triplet.previous.valid = false;
			_pos_sp_triplet.current.valid = false;
			_pos_sp_triplet.next.valid = false;
			_pos_sp_triplet_updated = true;
		}

		if (_pos_sp_triplet_updated) {
			_pos_sp_triplet.timestamp = hrt_absolute_time();
			publish_position_setpoint_triplet();
			_pos_sp_triplet_updated = false;
		}

		if (_mission_result_updated) {
			publish_mission_result();
			_mission_result_updated = false;
		}

		perf_end(_loop_perf);
	}

	PX4_INFO("exiting");

	_navigator_task = -1;
}

int
Navigator::start()
{
	ASSERT(_navigator_task == -1);

	/* start the task */
	_navigator_task = px4_task_spawn_cmd("navigator",
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_DEFAULT + 5,
					     1800,
					     (px4_main_t)&Navigator::task_main_trampoline,
					     nullptr);

	if (_navigator_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

void
Navigator::status()
{
	if (_geofence.valid()) {
		PX4_INFO("Geofence is valid");

	} else {
		PX4_INFO("Geofence not set (no /etc/geofence.txt on microsd) or not valid");
	}
}

void
Navigator::publish_position_setpoint_triplet()
{
	/* do not publish an empty triplet */
	if (!_pos_sp_triplet.current.valid) {
		return;
	}

	/* lazily publish the position setpoint triplet only once available */
	if (_pos_sp_triplet_pub != nullptr) {
		orb_publish(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_pub, &_pos_sp_triplet);

	} else {
		_pos_sp_triplet_pub = orb_advertise(ORB_ID(position_setpoint_triplet), &_pos_sp_triplet);
	}
}

float
Navigator::get_default_acceptance_radius()
{
	return _param_acceptance_radius.get();
}

float
Navigator::get_acceptance_radius()
{
	return get_acceptance_radius(_param_acceptance_radius.get());
}

float
Navigator::get_altitude_acceptance_radius()
{
	if (!get_vstatus()->is_rotary_wing) {
		return _param_fw_alt_acceptance_radius.get();

	} else {
		return _param_mc_alt_acceptance_radius.get();
	}
}

float
Navigator::get_cruising_speed()
{
	/* there are three options: The mission-requested cruise speed, or the current hover / plane speed */
	if (_vstatus.is_rotary_wing) {
		if (is_planned_mission() && _mission_cruising_speed_mc > 0.0f) {
			return _mission_cruising_speed_mc;

		} else {
			return -1.0f;
		}

	} else {
		if (is_planned_mission() && _mission_cruising_speed_fw > 0.0f) {
			return _mission_cruising_speed_fw;

		} else {
			return -1.0f;
		}
	}
}

void
Navigator::set_cruising_speed(float speed)
{
	if (_vstatus.is_rotary_wing) {
		_mission_cruising_speed_mc = speed;

	} else {
		_mission_cruising_speed_fw = speed;
	}
}

void
Navigator::reset_cruising_speed()
{
	_mission_cruising_speed_mc = -1.0f;
	_mission_cruising_speed_fw = -1.0f;
}

float
Navigator::get_cruising_throttle()
{
	/* Return the mission-requested cruise speed, or default FW_THR_CRUISE value */
	if (_mission_throttle > FLT_EPSILON) {
		return _mission_throttle;

	} else {
		return -1.0f;
	}
}

float
Navigator::get_acceptance_radius(float mission_item_radius)
{
	float radius = mission_item_radius;

	// XXX only use navigation capabilities for now
	// when in fixed wing mode
	// this might need locking against a commanded transition
	// so that a stale _vstatus doesn't trigger an accepted mission item.
	if (!_vstatus.is_rotary_wing && !_vstatus.in_transition_mode) {
		if ((hrt_elapsed_time(&_fw_pos_ctrl_status.timestamp) < 5000000) && (_fw_pos_ctrl_status.turn_distance > radius)) {
			radius = _fw_pos_ctrl_status.turn_distance;
		}
	}

	return radius;
}

void
Navigator::add_fence_point(int argc, char *argv[])
{
	_geofence.addPoint(argc, argv);
}

void
Navigator::load_fence_from_file(const char *filename)
{
	_geofence.loadFromFile(filename);
}

bool
Navigator::abort_landing()
{
	bool should_abort = false;

	if (!_vstatus.is_rotary_wing && !_vstatus.in_transition_mode) {
		if (hrt_elapsed_time(&_fw_pos_ctrl_status.timestamp) < 1000000) {

			if (get_position_setpoint_triplet()->current.valid
			    && get_position_setpoint_triplet()->current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {

				should_abort = _fw_pos_ctrl_status.abort_landing;
			}
		}
	}

	return should_abort;
}

static void usage()
{
	PX4_INFO("usage: navigator {start|stop|status|fence|fencefile}");
}

int navigator_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (navigator::g_navigator != nullptr) {
			PX4_WARN("already running");
			return 1;
		}

		navigator::g_navigator = new Navigator;

		if (navigator::g_navigator == nullptr) {
			PX4_ERR("alloc failed");
			return 1;
		}

		if (OK != navigator::g_navigator->start()) {
			delete navigator::g_navigator;
			navigator::g_navigator = nullptr;
			PX4_ERR("start failed");
			return 1;
		}

		return 0;
	}

	if (navigator::g_navigator == nullptr) {
		PX4_INFO("not running");
		return 1;
	}

	if (!strcmp(argv[1], "stop")) {
		delete navigator::g_navigator;
		navigator::g_navigator = nullptr;

	} else if (!strcmp(argv[1], "status")) {
		navigator::g_navigator->status();

	} else if (!strcmp(argv[1], "fence")) {
		navigator::g_navigator->add_fence_point(argc - 2, argv + 2);

	} else if (!strcmp(argv[1], "fencefile")) {
		navigator::g_navigator->load_fence_from_file(GEOFENCE_FILENAME);

	} else {
		usage();
		return 1;
	}

	return 0;
}

void
Navigator::publish_mission_result()
{
	_mission_result.timestamp = hrt_absolute_time();
	_mission_result.instance_count = _mission_instance_count;

	/* lazily publish the mission result only once available */
	if (_mission_result_pub != nullptr) {
		/* publish mission result */
		orb_publish(ORB_ID(mission_result), _mission_result_pub, &_mission_result);

	} else {
		/* advertise and publish */
		_mission_result_pub = orb_advertise(ORB_ID(mission_result), &_mission_result);
	}

	// Don't reset current waypoint because it won't be updated e.g. if not in mission mode
	// however, the current is still valid and therefore helpful for ground stations.
	//_mission_result.seq_current = 0;

	/* reset some of the flags */
	_mission_result.reached = false;
	_mission_result.item_do_jump_changed = false;
	_mission_result.item_changed_index = 0;
	_mission_result.item_do_jump_remaining = 0;
	_mission_result.valid = true;
}

void
Navigator::publish_geofence_result()
{
	/* lazily publish the geofence result only once available */
	if (_geofence_result_pub != nullptr) {
		/* publish mission result */
		orb_publish(ORB_ID(geofence_result), _geofence_result_pub, &_geofence_result);

	} else {
		/* advertise and publish */
		_geofence_result_pub = orb_advertise(ORB_ID(geofence_result), &_geofence_result);
	}
}

void
Navigator::publish_att_sp()
{
	/* lazily publish the attitude sp only once available */
	if (_att_sp_pub != nullptr) {
		/* publish att sp*/
		orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &_att_sp);

	} else {
		/* advertise and publish */
		_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_att_sp);
	}
}

void
Navigator::publish_vehicle_cmd(const struct vehicle_command_s &vcmd)
{
	if (_vehicle_cmd_pub != nullptr) {
		orb_publish(ORB_ID(vehicle_command), _vehicle_cmd_pub, &vcmd);

	} else {
		_vehicle_cmd_pub = orb_advertise_queue(ORB_ID(vehicle_command), &vcmd, vehicle_command_s::ORB_QUEUE_LENGTH);
	}
}

void
Navigator::set_mission_failure(const char *reason)
{
	if (!_mission_result.failure) {
		_mission_result.failure = true;
		set_mission_result_updated();
		mavlink_log_critical(&_mavlink_log_pub, "%s", reason);
	}
}
