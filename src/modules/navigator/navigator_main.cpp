/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <uORB/uORB.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/fence.h>
#include <uORB/topics/navigation_capabilities.h>
#include <uORB/topics/vehicle_command.h>
#include <drivers/drv_baro.h>

#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <geo/geo.h>
#include <dataman/dataman.h>
#include <mathlib/mathlib.h>
#include <systemlib/mavlink_log.h>

#include "navigator.h"

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
	SuperBlock(NULL, "NAV"),
	_task_should_exit(false),
	_navigator_task(-1),
	_mavlink_log_pub(nullptr),
	_global_pos_sub(-1),
	_gps_pos_sub(-1),
	_home_pos_sub(-1),
	_vstatus_sub(-1),
	_land_detected_sub(-1),
	_capabilities_sub(-1),
	_control_mode_sub(-1),
	_onboard_mission_sub(-1),
	_offboard_mission_sub(-1),
	_param_update_sub(-1),
	_vehicle_command_sub(-1),
	_pos_sp_triplet_pub(nullptr),
	_mission_result_pub(nullptr),
	_geofence_result_pub(nullptr),
	_att_sp_pub(nullptr),
	_vstatus{},
	_land_detected{},
	_control_mode{},
	_global_pos{},
	_gps_pos{},
	_sensor_combined{},
	_home_pos{},
	_mission_item{},
	_nav_caps{},
	_pos_sp_triplet{},
	_mission_result{},
	_att_sp{},
	_mission_item_valid(false),
	_mission_instance_count(0),
	_loop_perf(perf_alloc(PC_ELAPSED, "navigator")),
	_geofence(this),
	_geofence_violation_warning_sent(false),
	_inside_fence(true),
	_can_loiter_at_sp(false),
	_pos_sp_triplet_updated(false),
	_pos_sp_triplet_published_invalid_once(false),
	_mission_result_updated(false),
	_navigation_mode(nullptr),
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
	_param_datalinkloss_obc(this, "DLL_OBC"),
	_param_rcloss_obc(this, "RCL_OBC"),
	_param_cruising_speed_hover(this, "MPC_XY_CRUISE", false),
	_param_cruising_speed_plane(this, "FW_AIRSPD_TRIM", false),
	_mission_cruising_speed(-1.0f)
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
Navigator::navigation_capabilities_update()
{
	orb_copy(ORB_ID(navigation_capabilities), _capabilities_sub, &_nav_caps);
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
Navigator::vehicle_control_mode_update()
{
	if (orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode) != OK) {
		/* in case the commander is not be running */
		_control_mode.flag_control_auto_enabled = false;
		_control_mode.flag_armed = false;
	}
}

void
Navigator::params_update()
{
	parameter_update_s param_update;
	orb_copy(ORB_ID(parameter_update), _param_update_sub, &param_update);
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
		warnx("Try to load geofence.txt");
		_geofence.loadFromFile(GEOFENCE_FILENAME);

	} else {
		if (_geofence.clearDm() != OK) {
			mavlink_log_critical(&_mavlink_log_pub, "failed clearing geofence");
		}
	}

	/* do subscriptions */
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_gps_pos_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	_capabilities_sub = orb_subscribe(ORB_ID(navigation_capabilities));
	_vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));
	_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_home_pos_sub = orb_subscribe(ORB_ID(home_position));
	_onboard_mission_sub = orb_subscribe(ORB_ID(onboard_mission));
	_offboard_mission_sub = orb_subscribe(ORB_ID(offboard_mission));
	_param_update_sub = orb_subscribe(ORB_ID(parameter_update));
	_vehicle_command_sub = orb_subscribe(ORB_ID(vehicle_command));

	/* copy all topics first time */
	vehicle_status_update();
	vehicle_land_detected_update();
	vehicle_control_mode_update();
	global_position_update();
	gps_position_update();
	sensor_combined_update();
	home_position_update(true);
	navigation_capabilities_update();
	params_update();

	/* wakeup source(s) */
	px4_pollfd_struct_t fds[1] = {};

	/* Setup of loop */
	fds[0].fd = _global_pos_sub;
	fds[0].events = POLLIN;

	bool global_pos_available_once = false;

	while (!_task_should_exit) {

		/* wait for up to 200ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			/* timed out - periodic check for _task_should_exit, etc. */
			if (global_pos_available_once) {
				PX4_WARN("navigator timed out");
			}
			continue;

		} else if (pret < 0) {
			/* this is undesirable but not much we can do - might want to flag unhappy status */
			PX4_WARN("nav: poll error %d, %d", pret, errno);
			continue;
		}

		global_pos_available_once = true;

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

		/* sensors combined updated */
		orb_check(_sensor_combined_sub, &updated);
		if (updated) {
			sensor_combined_update();
		}

		/* parameters updated */
		orb_check(_param_update_sub, &updated);
		if (updated) {
			params_update();
			updateParams();
		}

		/* vehicle control mode updated */
		orb_check(_control_mode_sub, &updated);
		if (updated) {
			vehicle_control_mode_update();
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
		orb_check(_capabilities_sub, &updated);
		if (updated) {
			navigation_capabilities_update();
		}

		/* home position updated */
		orb_check(_home_pos_sub, &updated);
		if (updated) {
			home_position_update();
		}

		orb_check(_vehicle_command_sub, &updated);
		if (updated) {
			vehicle_command_s cmd;
			orb_copy(ORB_ID(vehicle_command), _vehicle_command_sub, &cmd);

			if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_REPOSITION) {
				warnx("navigator: got reposition command");
			}

			if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_PAUSE_CONTINUE) {
				warnx("navigator: got pause/continue command");
			}
		}

		/* global position updated */
		if (fds[0].revents & POLLIN) {
			global_position_update();
			if (_geofence.getSource() == Geofence::GF_SOURCE_GLOBALPOS) {
				have_geofence_position_data = true;
			}
		}

		/* Check geofence violation */
		static hrt_abstime last_geofence_check = 0;
		if (have_geofence_position_data &&
			(_geofence.getGeofenceAction() != geofence_result_s::GF_ACTION_NONE) &&
			(hrt_elapsed_time(&last_geofence_check) > GEOFENCE_CHECK_INTERVAL)) {
			bool inside = _geofence.inside(_global_pos, _gps_pos, _sensor_combined.baro_alt_meter[0], _home_pos, home_position_valid());
			last_geofence_check = hrt_absolute_time();
			have_geofence_position_data = false;

			_geofence_result.geofence_action = _geofence.getGeofenceAction();
			if (!inside) {
				/* inform other apps via the mission result */
				_geofence_result.geofence_violated = true;
				publish_geofence_result();

				/* Issue a warning about the geofence violation once */
				if (!_geofence_violation_warning_sent) {
					mavlink_log_critical(&_mavlink_log_pub, "Geofence violation");
					_geofence_violation_warning_sent = true;
				}
			} else {
				/* inform other apps via the mission result */
				_geofence_result.geofence_violated = false;
				publish_geofence_result();
				/* Reset the _geofence_violation_warning_sent field */
				_geofence_violation_warning_sent = false;
			}
		}

		/* Do stuff according to navigation state set by commander */
		switch (_vstatus.nav_state) {
			case vehicle_status_s::NAVIGATION_STATE_MANUAL:
			case vehicle_status_s::NAVIGATION_STATE_ACRO:
			case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
			case vehicle_status_s::NAVIGATION_STATE_POSCTL:
			case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
			case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
				_navigation_mode = nullptr;
				_can_loiter_at_sp = false;
				break;
			case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
				if (_nav_caps.abort_landing) {
					// pos controller aborted landing, requests loiter
					// above landing waypoint
					_navigation_mode = &_loiter;
					_pos_sp_triplet_published_invalid_once = false;
				} else {
					_pos_sp_triplet_published_invalid_once = false;
					_navigation_mode = &_mission;
				}
				break;
			case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
				_pos_sp_triplet_published_invalid_once = false;
				_navigation_mode = &_loiter;
				break;
			case vehicle_status_s::NAVIGATION_STATE_AUTO_RCRECOVER:
				_pos_sp_triplet_published_invalid_once = false;
				if (_param_rcloss_obc.get() != 0) {
					_navigation_mode = &_rcLoss;
				} else {
					_navigation_mode = &_rtl;
				}
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
				/* Use complex data link loss mode only when enabled via param
				* otherwise use rtl */
				_pos_sp_triplet_published_invalid_once = false;
				if (_param_datalinkloss_obc.get() != 0) {
					_navigation_mode = &_dataLinkLoss;
				} else {
					_navigation_mode = &_rtl;
				}
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
			default:
				_navigation_mode = nullptr;
				_can_loiter_at_sp = false;
				break;
		}

		/* iterate through navigation modes and set active/inactive for each */
		for (unsigned int i = 0; i < NAVIGATOR_MODE_ARRAY_SIZE; i++) {
			_navigation_mode_array[i]->run(_navigation_mode == _navigation_mode_array[i]);
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
			publish_position_setpoint_triplet();
			_pos_sp_triplet_updated = false;
		}

		if (_mission_result_updated) {
			publish_mission_result();
			_mission_result_updated = false;
		}

		perf_end(_loop_perf);
	}
	warnx("exiting.");

	_navigator_task = -1;
	return;
}

int
Navigator::start()
{
	ASSERT(_navigator_task == -1);

	/* start the task */
	_navigator_task = px4_task_spawn_cmd("navigator",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_DEFAULT + 5,
					 1500,
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
	/* TODO: add this again */
	// warnx("Global position is %svalid", _global_pos_valid ? "" : "in");

	// if (_global_pos.global_valid) {
	// 	warnx("Longitude %5.5f degrees, latitude %5.5f degrees", _global_pos.lon, _global_pos.lat);
	// 	warnx("Altitude %5.5f meters, altitude above home %5.5f meters",
	// 	      (double)_global_pos.alt, (double)(_global_pos.alt - _home_pos.alt));
	// 	warnx("Ground velocity in m/s, N %5.5f, E %5.5f, D %5.5f",
	// 	      (double)_global_pos.vel_n, (double)_global_pos.vel_e, (double)_global_pos.vel_d);
	// 	warnx("Compass heading in degrees %5.5f", (double)(_global_pos.yaw * M_RAD_TO_DEG_F));
	// }

	if (_geofence.valid()) {
		warnx("Geofence is valid");
		/* TODO: needed? */
//		warnx("Vertex longitude latitude");
//		for (unsigned i = 0; i < _fence.count; i++)
//		warnx("%6u %9.5f %8.5f", i, (double)_fence.vertices[i].lon, (double)_fence.vertices[i].lat);

	} else {
		warnx("Geofence not set (no /etc/geofence.txt on microsd) or not valid");
	}
}

void
Navigator::publish_position_setpoint_triplet()
{
	/* update navigation state */
	_pos_sp_triplet.nav_state = _vstatus.nav_state;

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
Navigator::get_cruising_speed()
{
	/* there are three options: The mission-requested cruise speed, or the current hover / plane speed */
	if (_mission_cruising_speed > 0.0f) {
		return _mission_cruising_speed;
	} else if (_vstatus.is_rotary_wing) {
		return _param_cruising_speed_hover.get();
	} else {
		return _param_cruising_speed_plane.get();
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
	if (!_vstatus.is_rotary_wing && !_vstatus.in_transition_mode && hrt_elapsed_time(&_nav_caps.timestamp) < 5000000) {
		if (_nav_caps.turn_distance > radius) {
			radius = _nav_caps.turn_distance;
		}
	}

	return radius;
}

void Navigator::add_fence_point(int argc, char *argv[])
{
	_geofence.addPoint(argc, argv);
}

void Navigator::load_fence_from_file(const char *filename)
{
	_geofence.loadFromFile(filename);
}


static void usage()
{
	warnx("usage: navigator {start|stop|status|fence|fencefile}");
}

int navigator_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (navigator::g_navigator != nullptr) {
			warnx("already running");
			return 1;
		}

		navigator::g_navigator = new Navigator;

		if (navigator::g_navigator == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != navigator::g_navigator->start()) {
			delete navigator::g_navigator;
			navigator::g_navigator = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (navigator::g_navigator == nullptr) {
		warnx("not running");
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
	_mission_result.instance_count = _mission_instance_count;

	/* lazily publish the mission result only once available */
	if (_mission_result_pub != nullptr) {
		/* publish mission result */
		orb_publish(ORB_ID(mission_result), _mission_result_pub, &_mission_result);

	} else {
		/* advertise and publish */
		_mission_result_pub = orb_advertise(ORB_ID(mission_result), &_mission_result);
	}

	/* reset some of the flags */
	_mission_result.seq_reached = false;
	_mission_result.seq_current = 0;
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
Navigator::set_mission_failure(const char* reason)
{
	if (!_mission_result.mission_failure) {
		_mission_result.mission_failure = true;
		set_mission_result_updated();
		mavlink_log_critical(&_mavlink_log_pub, "%s", reason);
	}
}
