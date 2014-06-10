/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 */

#include <nuttx/config.h>

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

#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <geo/geo.h>
#include <dataman/dataman.h>
#include <mathlib/mathlib.h>
#include <mavlink/mavlink_log.h>

#include "navigator.h"

/**
 * navigator app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int navigator_main(int argc, char *argv[]);


namespace navigator
{

Navigator	*g_navigator;
}

Navigator::Navigator() :
	SuperBlock(NULL, "NAV"),
	_task_should_exit(false),
	_navigator_task(-1),
	_mavlink_fd(-1),
	_global_pos_sub(-1),
	_home_pos_sub(-1),
	_vstatus_sub(-1),
	_capabilities_sub(-1),
	_control_mode_sub(-1),
	_onboard_mission_sub(-1),
	_offboard_mission_sub(-1),
	_pos_sp_triplet_pub(-1),
	_vstatus({}),
	_control_mode({}),
	_global_pos({}),
	_home_pos({}),
	_mission_item({}),
	_nav_caps({}),
	_pos_sp_triplet({}),
	_mission_item_valid(false),
	_loop_perf(perf_alloc(PC_ELAPSED, "navigator")),
	_geofence({}),
	_geofence_violation_warning_sent(false),
	_fence_valid(false),
	_inside_fence(true),
	_navigation_mode(nullptr),
	_mission(this, "MIS"),
	_loiter(this, "LOI"),
	_rtl(this, "RTL"),
	_update_triplet(false),
	_param_loiter_radius(this, "LOITER_RAD")
{
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
				task_delete(_navigator_task);
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
Navigator::home_position_update()
{
	orb_copy(ORB_ID(home_position), _home_pos_sub, &_home_pos);
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
		_vstatus.arming_state = ARMING_STATE_STANDBY;
	}
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
	/* inform about start */
	warnx("Initializing..");

	_mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

	/* Try to load the geofence:
	 * if /fs/microsd/etc/geofence.txt load from this file
	 * else clear geofence data in datamanager */
	struct stat buffer;

	if (stat(GEOFENCE_FILENAME, &buffer) == 0) {
		warnx("Try to load geofence.txt");
		_geofence.loadFromFile(GEOFENCE_FILENAME);

	} else {
		if (_geofence.clearDm() > 0)
			warnx("Geofence cleared");
		else
			warnx("Could not clear geofence");
	}

	/* do subscriptions */
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_capabilities_sub = orb_subscribe(ORB_ID(navigation_capabilities));
	_vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_home_pos_sub = orb_subscribe(ORB_ID(home_position));
	_onboard_mission_sub = orb_subscribe(ORB_ID(onboard_mission));
	_offboard_mission_sub = orb_subscribe(ORB_ID(offboard_mission));
	_param_update_sub = orb_subscribe(ORB_ID(parameter_update));

	/* copy all topics first time */
	vehicle_status_update();
	vehicle_control_mode_update();
	global_position_update();
	home_position_update();
	navigation_capabilities_update();
	params_update();

	/* rate limit position updates to 50 Hz */
	orb_set_interval(_global_pos_sub, 20);

	hrt_abstime mavlink_open_time = 0;
	const hrt_abstime mavlink_open_interval = 500000;

	/* wakeup source(s) */
	struct pollfd fds[6];

	/* Setup of loop */
	fds[0].fd = _global_pos_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _home_pos_sub;
	fds[1].events = POLLIN;
	fds[2].fd = _capabilities_sub;
	fds[2].events = POLLIN;
	fds[3].fd = _vstatus_sub;
	fds[3].events = POLLIN;
	fds[4].fd = _control_mode_sub;
	fds[4].events = POLLIN;
	fds[5].fd = _param_update_sub;
	fds[5].events = POLLIN;

	while (!_task_should_exit) {

		/* wait for up to 100ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		if (pret == 0) {
			/* timed out - periodic check for _task_should_exit, etc. */
			continue;

		} else if (pret < 0) {
			/* this is undesirable but not much we can do - might want to flag unhappy status */
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		perf_begin(_loop_perf);

		if (_mavlink_fd < 0 && hrt_absolute_time() > mavlink_open_time) {
			/* try to reopen the mavlink log device with specified interval */
			mavlink_open_time = hrt_abstime() + mavlink_open_interval;
			_mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
		}

		/* parameters updated */
		if (fds[5].revents & POLLIN) {
			params_update();
			updateParams();
		}

		/* vehicle control mode updated */
		if (fds[4].revents & POLLIN) {
			vehicle_control_mode_update();
		}

		/* vehicle status updated */
		if (fds[3].revents & POLLIN) {
			vehicle_status_update();
		}

		/* navigation capabilities updated */
		if (fds[2].revents & POLLIN) {
			navigation_capabilities_update();
		}

		/* home position updated */
		if (fds[1].revents & POLLIN) {
			home_position_update();
		}

		/* global position updated */
		if (fds[0].revents & POLLIN) {
			global_position_update();

			/* Check geofence violation */
			if (!_geofence.inside(&_global_pos)) {

				/* Issue a warning about the geofence violation once */
				if (!_geofence_violation_warning_sent) {
					mavlink_log_critical(_mavlink_fd, "#audio: Geofence violation");
					_geofence_violation_warning_sent = true;
				}
			} else {
				/* Reset the _geofence_violation_warning_sent field */
				_geofence_violation_warning_sent = false;
			}
		}

		/* Do stuff according to navigation state set by commander */
		switch (_vstatus.set_nav_state) {
			case NAVIGATION_STATE_MANUAL:
			case NAVIGATION_STATE_ACRO:
			case NAVIGATION_STATE_ALTCTL:
			case NAVIGATION_STATE_POSCTL:
				_navigation_mode = nullptr;
				_is_in_loiter = false;
				break;
			case NAVIGATION_STATE_AUTO_MISSION:
				_navigation_mode = &_mission;
				break;
			case NAVIGATION_STATE_AUTO_LOITER:
				_navigation_mode = &_loiter;
				break;
			case NAVIGATION_STATE_AUTO_RTL:
			case NAVIGATION_STATE_AUTO_RTL_RC:
			case NAVIGATION_STATE_AUTO_RTL_DL:
				_navigation_mode = &_rtl;
				break;
			case NAVIGATION_STATE_LAND:
			case NAVIGATION_STATE_TERMINATION:
			default:
				_navigation_mode = nullptr;
				_is_in_loiter = false;
				break;
		}

		/* TODO: make list of modes and loop through it */
		if (_navigation_mode == &_mission) {
			_update_triplet = _mission.update(&_pos_sp_triplet);
		} else {
			_mission.reset();
		}

		if (_navigation_mode == &_rtl) {
			_update_triplet = _rtl.update(&_pos_sp_triplet);
		} else {
			_rtl.reset();
		}

		if (_navigation_mode == &_loiter) {
			_update_triplet = _loiter.update(&_pos_sp_triplet);
		} else {
			_loiter.reset();
		}

		/* if nothing is running, set position setpoint triplet invalid */
		if (_navigation_mode == nullptr) {
			_pos_sp_triplet.previous.valid = false;
			_pos_sp_triplet.current.valid = false;
			_pos_sp_triplet.next.valid = false;
			_update_triplet = true;
		}

		if (_update_triplet ) {
			publish_position_setpoint_triplet();
			_update_triplet = false;
		}

		perf_end(_loop_perf);
	}
	warnx("exiting.");

	_navigator_task = -1;
	_exit(0);
}

int
Navigator::start()
{
	ASSERT(_navigator_task == -1);

	/* start the task */
	_navigator_task = task_spawn_cmd("navigator",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 5,
					 2000,
					 (main_t)&Navigator::task_main_trampoline,
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

	if (_fence_valid) {
		warnx("Geofence is valid");
		/* TODO: needed? */
//		warnx("Vertex longitude latitude");
//		for (unsigned i = 0; i < _fence.count; i++)
//		warnx("%6u %9.5f %8.5f", i, (double)_fence.vertices[i].lon, (double)_fence.vertices[i].lat);

	} else {
		warnx("Geofence not set");
	}
}
#if 0
bool
Navigator::start_none_on_ground()
{
	reset_reached();

	_pos_sp_triplet.previous.valid = false;
	_pos_sp_triplet.current.valid = false;
	_pos_sp_triplet.next.valid = false;

	_update_triplet = true;
	return true;
}

bool
Navigator::start_none_in_air()
{
	reset_reached();

	_pos_sp_triplet.previous.valid = false;
	_pos_sp_triplet.current.valid = false;
	_pos_sp_triplet.next.valid = false;

	_update_triplet = true;
	return true;
}

bool
Navigator::start_auto_on_ground()
{
	reset_reached();

	_pos_sp_triplet.previous.valid = false;
	_pos_sp_triplet.current.valid = true;
	_pos_sp_triplet.next.valid = false;

	_pos_sp_triplet.current.type = SETPOINT_TYPE_IDLE;

	_update_triplet = true;
	return true;
}

bool
Navigator::start_loiter()
{
	/* if no existing item available, use current position */
	if (!(_pos_sp_triplet.current.valid && _waypoint_position_reached)) {

		_pos_sp_triplet.current.lat = _global_pos.lat;
		_pos_sp_triplet.current.lon = _global_pos.lon;
		_pos_sp_triplet.current.yaw = NAN;	// NAN means to use current yaw
		_pos_sp_triplet.current.alt = _global_pos.alt;
	}
	_pos_sp_triplet.current.type = SETPOINT_TYPE_LOITER;
	_pos_sp_triplet.current.loiter_radius = _parameters.loiter_radius;
	_pos_sp_triplet.current.loiter_direction = 1;
	_pos_sp_triplet.previous.valid = false;
	_pos_sp_triplet.current.valid = true;
	_pos_sp_triplet.next.valid = false;

	mavlink_log_info(_mavlink_fd, "#audio: loiter at current altitude");

	_update_triplet = true;
	return true;
}

bool
Navigator::start_mission()
{
	/* start fresh */
	_pos_sp_triplet.previous.valid = false;
	_pos_sp_triplet.current.valid = false;
	_pos_sp_triplet.next.valid = false;

	return set_mission_items();
}

bool
Navigator::advance_mission()
{
	/* tell mission to move by one */
	_mission.move_to_next();

	/* now try to set the new mission items, if it fails, it will dispatch loiter */
	return set_mission_items();
}

bool
Navigator::set_mission_items()
{
	if (_pos_sp_triplet.current.valid) {
		memcpy(&_pos_sp_triplet.previous, &_pos_sp_triplet.current, sizeof(position_setpoint_s));
		_pos_sp_triplet.previous.valid = true;
	}

	bool onboard;
	int index;

	/* if we fail to set the current mission, continue to loiter */
	if (!_mission.get_current_mission_item(&_mission_item, &onboard, &index)) {

		return false;
	}

	/* if we got an RTL mission item, switch to RTL mode and give up */
	if (_mission_item.nav_cmd == NAV_CMD_RETURN_TO_LAUNCH) {
		return false;
	}

	_mission_item_valid = true;

	/* convert the current mission item and set it valid */
	mission_item_to_position_setpoint(&_mission_item, &_pos_sp_triplet.current);
	_pos_sp_triplet.current.valid = true;


	mission_item_s next_mission_item;

	bool last_wp = false;
	/* now try to set the next mission item as well, if there is no more next
	 * this means we're heading to the last waypoint */
	if (_mission.get_next_mission_item(&next_mission_item)) {
		/* convert the next mission item and set it valid */
		mission_item_to_position_setpoint(&next_mission_item, &_pos_sp_triplet.next);
		_pos_sp_triplet.next.valid = true;
	} else {
		last_wp = true;
	}

	/* notify user about what happened */
	mavlink_log_info(_mavlink_fd, "#audio: heading to %s%swaypoint %d",
		(last_wp ? "last " : "" ), (onboard ? "onboard " : ""), index);

	_update_triplet = true;

	reset_reached();

	return true;
}

bool
Navigator::start_rtl()
{
	if (_rtl.get_current_rtl_item(&_global_pos, &_mission_item)) {

		_mission_item_valid = true;

		mission_item_to_position_setpoint(&_mission_item, &_pos_sp_triplet.current);
		_pos_sp_triplet.current.valid = true;

		reset_reached();

		_update_triplet = true;
		return true;
	}

	/* if RTL doesn't work, fallback to loiter */
	return false;
}

bool
Navigator::advance_rtl()
{
	/* tell mission to move by one */
	_rtl.move_to_next();

	/* now try to set the new mission items, if it fails, it will dispatch loiter */
	if (_rtl.get_current_rtl_item(&_global_pos, &_mission_item)) {

		_mission_item_valid = true;

		mission_item_to_position_setpoint(&_mission_item, &_pos_sp_triplet.current);
		_pos_sp_triplet.current.valid = true;

		reset_reached();

		_update_triplet = true;
		return true;
	}

	return false;
}

bool
Navigator::start_land()
{
	/* TODO: verify/test */

	reset_reached();

	/* this state can be requested by commander even if no global position available,
	 * in his case controller must perform landing without position control */

	memcpy(&_pos_sp_triplet.previous, &_pos_sp_triplet.current, sizeof(position_setpoint_s));

	_mission_item.lat = _global_pos.lat;
	_mission_item.lon = _global_pos.lon;
	_mission_item.altitude_is_relative = false;
	_mission_item.altitude = _global_pos.alt;
	_mission_item.yaw = NAN;
	_mission_item.loiter_radius = _parameters.loiter_radius;
	_mission_item.loiter_direction = 1;
	_mission_item.nav_cmd = NAV_CMD_LAND;
	_mission_item.acceptance_radius = _parameters.acceptance_radius;
	_mission_item.time_inside = 0.0f;
	_mission_item.pitch_min = 0.0f;
	_mission_item.autocontinue = true;
	_mission_item.origin = ORIGIN_ONBOARD;

	_mission_item_valid = true;

	mission_item_to_position_setpoint(&_mission_item, &_pos_sp_triplet.current);

	_pos_sp_triplet.next.valid = false;

	_update_triplet = true;
	return true;
}
bool
Navigator::check_mission_item_reached()
{
	/* only check if there is actually a mission item to check */
	if (!_mission_item_valid) {
		return false;
	}

	if (_mission_item.nav_cmd == NAV_CMD_LAND) {
		return _vstatus.condition_landed;
	}

	/* XXX TODO count turns */
	if ((_mission_item.nav_cmd == NAV_CMD_LOITER_TURN_COUNT ||
	     _mission_item.nav_cmd == NAV_CMD_LOITER_UNLIMITED) &&
	    _mission_item.loiter_radius > 0.01f) {

	// 	return false;
	// }

	uint64_t now = hrt_absolute_time();

	if (!_waypoint_position_reached) {
		float acceptance_radius;

		if (_mission_item.nav_cmd == NAV_CMD_WAYPOINT && _mission_item.acceptance_radius > 0.01f) {
			acceptance_radius = _mission_item.acceptance_radius;

		} else {
			acceptance_radius = _parameters.acceptance_radius;
		}

		if (_do_takeoff) {
			/* require only altitude for takeoff */
			if (_global_pos.alt > _pos_sp_triplet.current.alt - acceptance_radius) {
				_waypoint_position_reached = true;
			}
		} else {
			float dist = -1.0f;
			float dist_xy = -1.0f;
			float dist_z = -1.0f;

			/* calculate AMSL altitude for this waypoint */
			float wp_alt_amsl = _mission_item.altitude;

			if (_mission_item.altitude_is_relative)
				wp_alt_amsl += _home_pos.alt;

			dist = get_distance_to_point_global_wgs84(_mission_item.lat, _mission_item.lon, wp_alt_amsl,
					(double)_global_pos.lat, (double)_global_pos.lon, _global_pos.alt,
					&dist_xy, &dist_z);

			if (dist >= 0.0f && dist <= acceptance_radius) {
				_waypoint_position_reached = true;
			}
		}
	}

	if (_waypoint_position_reached && !_waypoint_yaw_reached) {

		/* TODO: removed takeoff, why? */
		if (_vstatus.is_rotary_wing && isfinite(_mission_item.yaw)) {

			/* check yaw if defined only for rotary wing except takeoff */
			float yaw_err = _wrap_pi(_mission_item.yaw - _global_pos.yaw);

			if (fabsf(yaw_err) < 0.2f) { /* TODO: get rid of magic number */
				_waypoint_yaw_reached = true;
			}

		} else {
			_waypoint_yaw_reached = true;
		}
	}

	/* check if the current waypoint was reached */
	if (_waypoint_position_reached && _waypoint_yaw_reached) {

		if (_time_first_inside_orbit == 0) {
			_time_first_inside_orbit = now;

			if (_mission_item.time_inside > 0.01f) {
				mavlink_log_info(_mavlink_fd, "#audio: waypoint reached, wait for %.1fs",
					(double)_mission_item.time_inside);
			}
		}

		/* check if the MAV was long enough inside the waypoint orbit */
		if ((now - _time_first_inside_orbit >= (uint64_t)_mission_item.time_inside * 1e6)
		    || _mission_item.nav_cmd == NAV_CMD_TAKEOFF) {
			return true;
		}
	}
	return false;
}

void
Navigator::reset_reached()
{
	_time_first_inside_orbit = 0;
	_waypoint_position_reached = false;
	_waypoint_yaw_reached = false;

}
#endif
void
Navigator::publish_position_setpoint_triplet()
{
	/* update navigation state */
	/* TODO: set nav_state */

	/* lazily publish the position setpoint triplet only once available */
	if (_pos_sp_triplet_pub > 0) {
		orb_publish(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_pub, &_pos_sp_triplet);

	} else {
		_pos_sp_triplet_pub = orb_advertise(ORB_ID(position_setpoint_triplet), &_pos_sp_triplet);
	}
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
	errx(1, "usage: navigator {start|stop|status|fence|fencefile}");
}

int navigator_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
	}

	if (!strcmp(argv[1], "start")) {

		if (navigator::g_navigator != nullptr) {
			errx(1, "already running");
		}

		navigator::g_navigator = new Navigator;

		if (navigator::g_navigator == nullptr) {
			errx(1, "alloc failed");
		}

		if (OK != navigator::g_navigator->start()) {
			delete navigator::g_navigator;
			navigator::g_navigator = nullptr;
			err(1, "start failed");
		}

		return 0;
	}

	if (navigator::g_navigator == nullptr)
		errx(1, "not running");

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
	}

	return 0;
}
