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
 * Implementation of the main navigation state machine.
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
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/fence.h>
#include <uORB/topics/navigation_capabilities.h>

#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/state_table.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <geo/geo.h>
#include <dataman/dataman.h>
#include <mathlib/mathlib.h>
#include <mavlink/mavlink_log.h>

#include "mission.h"
#include "rtl.h"
#include "mission_feasibility_checker.h"
#include "geofence.h"


/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

/**
 * navigator app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int navigator_main(int argc, char *argv[]);

class Navigator : public StateTable
{
public:
	/**
	 * Constructor
	 */
	Navigator();

	/**
	 * Destructor, also kills the navigators task.
	 */
	~Navigator();

	/**
	* Start the navigator task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	/**
	 * Display the navigator status.
	 */
	void		status();

	/**
	 * Add point to geofence
	 */
	void		add_fence_point(int argc, char *argv[]);

	/**
	 * Load fence from file
	 */
	void		load_fence_from_file(const char *filename);

private:

	bool		_task_should_exit;		/**< if true, sensor task should exit */
	int		_navigator_task;		/**< task handle for sensor task */

	int		_mavlink_fd;			/**< the file descriptor to send messages over mavlink */

	int		_global_pos_sub;		/**< global position subscription */
	int		_home_pos_sub;			/**< home position subscription */
	int		_vstatus_sub;			/**< vehicle status subscription */
	int		_params_sub;			/**< notification of parameter updates */
	int		_offboard_mission_sub;		/**< notification of offboard mission updates */
	int 		_onboard_mission_sub;		/**< notification of onboard mission updates */
	int		_capabilities_sub;		/**< notification of vehicle capabilities updates */
	int		_control_mode_sub;		/**< vehicle control mode subscription */

	orb_advert_t	_pos_sp_triplet_pub;		/**< publish position setpoint triplet */

	vehicle_status_s				_vstatus;		/**< vehicle status */
	vehicle_control_mode_s				_control_mode;		/**< vehicle control mode */
	vehicle_global_position_s			_global_pos;		/**< global vehicle position */
	home_position_s					_home_pos;		/**< home position for RTL */
	mission_item_s 					_mission_item;		/**< current mission item */
	navigation_capabilities_s			_nav_caps;		/**< navigation capabilities */
	position_setpoint_triplet_s			_pos_sp_triplet;	/**< triplet of position setpoints */

	bool 		_mission_item_valid;		/**< flags if the current mission item is valid */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	Geofence	_geofence;			/**< class that handles the geofence */
	bool		_geofence_violation_warning_sent; /**< prevents spaming to mavlink */

	bool		_fence_valid;			/**< flag if fence is valid */
	bool		_inside_fence;			/**< vehicle is inside fence */


	Mission		_mission;			/**< class that handles the missions */

	RTL 		_rtl;				/**< class that handles RTL */

	bool		_waypoint_position_reached;	/**< flags if the position of the mission item has been reached */
	bool		_waypoint_yaw_reached;		/**< flags if the yaw position of the mission item has been reached */
	uint64_t	_time_first_inside_orbit;	/**< the time when we were first in the acceptance radius of the mission item */

	MissionFeasibilityChecker			missionFeasiblityChecker; /**< class that checks if a mission is feasible */

	bool		_update_triplet;		/**< flags if position setpoint triplet needs to be published */

	struct {
		float acceptance_radius;
		float loiter_radius;
		int onboard_mission_enabled;
		float takeoff_alt;
	}		_parameters;			/**< local copies of parameters */

	struct {
		param_t acceptance_radius;
		param_t loiter_radius;
		param_t onboard_mission_enabled;
		param_t takeoff_alt;
	}		_parameter_handles;		/**< handles for parameters */

	enum Event {
		EVENT_NONE_REQUESTED,
		EVENT_LOITER_REQUESTED,
		EVENT_MISSION_REQUESTED,
		EVENT_RTL_REQUESTED,
		EVENT_TAKEN_OFF,
		EVENT_LANDED,
		EVENT_MISSION_CHANGED,
		EVENT_HOME_POSITION_CHANGED,
		EVENT_MISSION_ITEM_REACHED,
		MAX_EVENT
	};						/**< possible events that can be thrown at state machine */

	/**
	 * State machine transition table
	 */
	static StateTable::Tran const myTable[NAV_STATE_MAX][MAX_EVENT];

	/**
	 * Update our local parameter cache.
	 */
	void		parameters_update();

	/**
	 * Retrieve global position
	 */
	void		global_position_update();

	/**
	 * Retrieve home position
	 */
	void		home_position_update();

	/**
	 * Retreive navigation capabilities
	 */
	void		navigation_capabilities_update();

	/**
	 * Retrieve offboard mission.
	 */
	void		offboard_mission_update();

	/**
	 * Retrieve onboard mission.
	 */
	void		onboard_mission_update();

	/**
	 * Retrieve vehicle status
	 */
	void		vehicle_status_update();

	/**
	 * Retrieve vehicle control mode
	 */
	void		vehicle_control_mode_update();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main task.
	 */
	void		task_main();

	/**
	 * Functions that are triggered when a new state is entered.
	 */
	bool		start_none_on_ground();
	bool		start_none_in_air();
	bool		start_auto_on_ground();
	bool		start_loiter();
	bool		start_mission();
	bool		advance_mission();
	bool		start_rtl();
	bool		advance_rtl();
	bool		start_land();

	/**
	 * Reset all "mission item reached" flags.
	 */
	void		reset_reached();

	/**
	 * Check if current mission item has been reached.
	 */
	bool		check_mission_item_reached();

	/**
	 * Set mission items by accessing the mission class.
	 * If failing, tell the state machine to loiter.
	 */
	bool		set_mission_items();

	/**
	 * Set a RTL item by accessing the RTL class.
	 * If failing, tell the state machine to loiter.
	 */
	void		set_rtl_item();

	/**
	 * Translate mission item to a position setpoint.
	 */
	void		mission_item_to_position_setpoint(const mission_item_s *item, position_setpoint_s *sp);

	/**
	 * Publish a new position setpoint triplet for position controllers
	 */
	void		publish_position_setpoint_triplet();
};

namespace navigator
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

Navigator	*g_navigator;
}

Navigator::Navigator() :

	/* state machine transition table */
	StateTable(&myTable[0][0], NAV_STATE_MAX, MAX_EVENT),
	_task_should_exit(false),
	_navigator_task(-1),
	_mavlink_fd(-1),
	_global_pos_sub(-1),
	_home_pos_sub(-1),
	_vstatus_sub(-1),
	_params_sub(-1),
	_offboard_mission_sub(-1),
	_onboard_mission_sub(-1),
	_capabilities_sub(-1),
	_control_mode_sub(-1),
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
	_mission({}),
	_rtl({}),
	_waypoint_position_reached(false),
	_waypoint_yaw_reached(false),
	_time_first_inside_orbit(0),
	_update_triplet(false),
	_parameters({}),
	_parameter_handles({})
{
	_parameter_handles.acceptance_radius = param_find("NAV_ACCEPT_RAD");
	_parameter_handles.loiter_radius = param_find("NAV_LOITER_RAD");
	_parameter_handles.onboard_mission_enabled = param_find("NAV_ONB_MIS_EN");
	_parameter_handles.takeoff_alt = param_find("NAV_TAKEOFF_ALT");

	/* Initialize state machine */
	myState = NAV_STATE_NONE_ON_GROUND;

	start_none_on_ground();
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
Navigator::parameters_update()
{
	/* read from param to clear updated flag */
	struct parameter_update_s update;
	orb_copy(ORB_ID(parameter_update), _params_sub, &update);

	param_get(_parameter_handles.acceptance_radius, &(_parameters.acceptance_radius));
	param_get(_parameter_handles.loiter_radius, &(_parameters.loiter_radius));
	param_get(_parameter_handles.onboard_mission_enabled, &(_parameters.onboard_mission_enabled));
	param_get(_parameter_handles.takeoff_alt, &(_parameters.takeoff_alt));

	_mission.set_onboard_mission_allowed((bool)_parameters.onboard_mission_enabled);

	_geofence.updateParams();
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

	_rtl.set_home_position(&_home_pos);
}

void
Navigator::navigation_capabilities_update()
{
	orb_copy(ORB_ID(navigation_capabilities), _capabilities_sub, &_nav_caps);
}


void
Navigator::offboard_mission_update()
{
	struct mission_s offboard_mission;

	if (orb_copy(ORB_ID(offboard_mission), _offboard_mission_sub, &offboard_mission) == OK) {

		/* Check mission feasibility, for now do not handle the return value,
		 * however warnings are issued to the gcs via mavlink from inside the MissionFeasiblityChecker */
		dm_item_t dm_current;

		if (offboard_mission.dataman_id == 0) {
			dm_current = DM_KEY_WAYPOINTS_OFFBOARD_0;

		} else {
			dm_current = DM_KEY_WAYPOINTS_OFFBOARD_1;
		}

		missionFeasiblityChecker.checkMissionFeasible(_vstatus.is_rotary_wing, dm_current, (size_t)offboard_mission.count, _geofence);

		_mission.set_offboard_dataman_id(offboard_mission.dataman_id);

		_mission.set_offboard_mission_count(offboard_mission.count);
		_mission.command_current_offboard_mission_index(offboard_mission.current_index);

	} else {
		_mission.set_offboard_mission_count(0);
		_mission.command_current_offboard_mission_index(0);
	}
}

void
Navigator::onboard_mission_update()
{
	struct mission_s onboard_mission;

	if (orb_copy(ORB_ID(onboard_mission), _onboard_mission_sub, &onboard_mission) == OK) {

		_mission.set_onboard_mission_count(onboard_mission.count);
		_mission.command_current_onboard_mission_index(onboard_mission.current_index);

	} else {
		_mission.set_onboard_mission_count(0);
		_mission.command_current_onboard_mission_index(0);
	}
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
	 * else clear geofence data in datamanager
	 */
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

	/*
	 * do subscriptions
	 */
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_offboard_mission_sub = orb_subscribe(ORB_ID(offboard_mission));
	_onboard_mission_sub = orb_subscribe(ORB_ID(onboard_mission));
	_capabilities_sub = orb_subscribe(ORB_ID(navigation_capabilities));
	_vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_home_pos_sub = orb_subscribe(ORB_ID(home_position));

	/* copy all topics first time */
	vehicle_status_update();
	vehicle_control_mode_update();
	parameters_update();
	global_position_update();
	home_position_update();
	navigation_capabilities_update();
	offboard_mission_update();
	onboard_mission_update();

	/* rate limit position updates to 50 Hz */
	orb_set_interval(_global_pos_sub, 20);

	hrt_abstime mavlink_open_time = 0;
	const hrt_abstime mavlink_open_interval = 500000;

	/* wakeup source(s) */
	struct pollfd fds[8];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _global_pos_sub;
	fds[1].events = POLLIN;
	fds[2].fd = _home_pos_sub;
	fds[2].events = POLLIN;
	fds[3].fd = _capabilities_sub;
	fds[3].events = POLLIN;
	fds[4].fd = _offboard_mission_sub;
	fds[4].events = POLLIN;
	fds[5].fd = _onboard_mission_sub;
	fds[5].events = POLLIN;
	fds[6].fd = _vstatus_sub;
	fds[6].events = POLLIN;
	fds[7].fd = _control_mode_sub;
	fds[7].events = POLLIN;

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

		/* vehicle control mode updated */
		if (fds[7].revents & POLLIN) {
			vehicle_control_mode_update();
		}

		/* vehicle status updated */
		if (fds[6].revents & POLLIN) {
			vehicle_status_update();

			/* commander requested new navigation mode, forward it to state machine */
			switch (_vstatus.set_nav_state) {
			case NAVIGATION_STATE_NONE:
				dispatch(EVENT_NONE_REQUESTED);
				break;

			case NAVIGATION_STATE_LOITER:
				dispatch(EVENT_LOITER_REQUESTED);
				break;

			case NAVIGATION_STATE_MISSION:
				dispatch(EVENT_MISSION_REQUESTED);
				break;

			case NAVIGATION_STATE_RTL:
				dispatch(EVENT_RTL_REQUESTED);
				break;

			case NAVIGATION_STATE_LAND:
				/* TODO: add and test this */
				// dispatch(EVENT_LAND_REQUESTED);
				break;

			default:
				warnx("ERROR: Requested navigation state not supported");
				break;
			}

			/* commander sets in-air/on-ground flag as well */
			if (_vstatus.condition_landed) {
				dispatch(EVENT_LANDED);
			} else {
				dispatch(EVENT_TAKEN_OFF);
			}
		}

		/* parameters updated */
		if (fds[0].revents & POLLIN) {
			parameters_update();
			/* note that these new parameters won't be in effect until a mission triplet is published again */
		}

		/* navigation capabilities updated */
		if (fds[3].revents & POLLIN) {
			navigation_capabilities_update();
		}

		/* offboard mission updated */
		if (fds[4].revents & POLLIN) {
			offboard_mission_update();
			/* TODO: check if mission really changed */
			dispatch(EVENT_MISSION_CHANGED);
		}

		/* onboard mission updated */
		if (fds[5].revents & POLLIN) {
			onboard_mission_update();
			/* TODO: check if mission really changed */
			dispatch(EVENT_MISSION_CHANGED);
		}

		/* home position updated */
		if (fds[2].revents & POLLIN) {
			home_position_update();
			/* TODO check if home position really changed */
			dispatch(EVENT_HOME_POSITION_CHANGED);
		}

		/* global position updated */
		if (fds[1].revents & POLLIN) {
			global_position_update();
			if (check_mission_item_reached()) {
				dispatch(EVENT_MISSION_ITEM_REACHED);
			}

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

	switch (myState) {
	case NAV_STATE_NONE_ON_GROUND:
		warnx("State: None on ground");
		break;

	case NAV_STATE_NONE_IN_AIR:
		warnx("State: None in air");
		break;

	case NAV_STATE_LOITER:
		warnx("State: Loiter");
		break;

	case NAV_STATE_MISSION:
		warnx("State: Mission");
		break;

	case NAV_STATE_RTL:
		warnx("State: RTL");
		break;

	case NAV_STATE_LAND:
		warnx("State: Land");
		break;

	default:
		warnx("State: Unknown");
		break;
	}
}

StateTable::Tran const Navigator::myTable[NAV_STATE_MAX][MAX_EVENT] = {
	{
		/* NAV_STATE_NONE_ON_GROUND */
		/* EVENT_NONE_REQUESTED */		{NO_ACTION, NAV_STATE_NONE_ON_GROUND},
		/* EVENT_LOITER_REQUESTED */		{NO_ACTION, NAV_STATE_NONE_ON_GROUND},
		/* EVENT_MISSION_REQUESTED */		{ACTION(&Navigator::start_auto_on_ground), NAV_STATE_AUTO_ON_GROUND},
		/* EVENT_RTL_REQUESTED */		{NO_ACTION, NAV_STATE_NONE_ON_GROUND},
		/* EVENT_TAKEN_OFF */			{ACTION(&Navigator::start_none_in_air), NAV_STATE_NONE_IN_AIR},
		/* EVENT_LANDED */			{NO_ACTION, NAV_STATE_NONE_ON_GROUND},
		/* EVENT_MISSION_CHANGED */		{NO_ACTION, NAV_STATE_NONE_ON_GROUND},
		/* EVENT_HOME_POSITION_CHANGED */	{NO_ACTION, NAV_STATE_NONE_ON_GROUND},
		/* EVENT_MISSION_ITEM_REACHED */	{NO_ACTION, NAV_STATE_NONE_ON_GROUND},
	},
	{
		/* NAV_STATE_NONE_IN_AIR */
		/* EVENT_NONE_REQUESTED */		{NO_ACTION, NAV_STATE_NONE_IN_AIR},
		/* EVENT_LOITER_REQUESTED */		{ACTION(&Navigator::start_loiter), NAV_STATE_LOITER},
		/* EVENT_MISSION_REQUESTED */		{ACTION(&Navigator::start_mission), NAV_STATE_MISSION},
		/* EVENT_RTL_REQUESTED */		{ACTION(&Navigator::start_rtl), NAV_STATE_RTL},
		/* EVENT_TAKEN_OFF */			{NO_ACTION, NAV_STATE_NONE_IN_AIR},
		/* EVENT_LANDED */			{ACTION(&Navigator::start_none_on_ground), NAV_STATE_NONE_ON_GROUND},
		/* EVENT_MISSION_CHANGED */		{NO_ACTION, NAV_STATE_NONE_IN_AIR},
		/* EVENT_HOME_POSITION_CHANGED */	{NO_ACTION, NAV_STATE_NONE_IN_AIR},
		/* EVENT_MISSION_ITEM_REACHED */	{NO_ACTION, NAV_STATE_NONE_IN_AIR},
	},
	{
		/* NAV_STATE_AUTO_ON_GROUND */
		/* EVENT_NONE_REQUESTED */		{NO_ACTION, NAV_STATE_AUTO_ON_GROUND},
		/* EVENT_LOITER_REQUESTED */		{NO_ACTION, NAV_STATE_AUTO_ON_GROUND},
		/* EVENT_MISSION_REQUESTED */		{NO_ACTION, NAV_STATE_AUTO_ON_GROUND},
		/* EVENT_RTL_REQUESTED */		{NO_ACTION, NAV_STATE_AUTO_ON_GROUND},
		/* EVENT_TAKEN_OFF */			{ACTION(&Navigator::start_mission), NAV_STATE_MISSION},
		/* EVENT_LANDED */			{NO_ACTION, NAV_STATE_AUTO_ON_GROUND},
		/* EVENT_MISSION_CHANGED */		{NO_ACTION, NAV_STATE_AUTO_ON_GROUND},
		/* EVENT_HOME_POSITION_CHANGED */	{NO_ACTION, NAV_STATE_AUTO_ON_GROUND},
		/* EVENT_MISSION_ITEM_REACHED */	{NO_ACTION, NAV_STATE_AUTO_ON_GROUND},
	},
	{
		/* NAV_STATE_LOITER */
		/* EVENT_NONE_REQUESTED */		{ACTION(&Navigator::start_none_in_air), NAV_STATE_NONE_IN_AIR},
		/* EVENT_LOITER_REQUESTED */		{NO_ACTION, NAV_STATE_LOITER},
		/* EVENT_MISSION_REQUESTED */		{ACTION(&Navigator::start_mission), NAV_STATE_MISSION},
		/* EVENT_RTL_REQUESTED */		{ACTION(&Navigator::start_rtl), NAV_STATE_RTL},
		/* EVENT_TAKEN_OFF */			{NO_ACTION, NAV_STATE_LOITER},
		/* EVENT_LANDED */			{NO_ACTION, NAV_STATE_LOITER},
		/* EVENT_MISSION_CHANGED */		{NO_ACTION, NAV_STATE_LOITER},
		/* EVENT_HOME_POSITION_CHANGED */	{NO_ACTION, NAV_STATE_LOITER},
		/* EVENT_MISSION_ITEM_REACHED */	{NO_ACTION, NAV_STATE_LOITER},
	},
	{
		/* NAV_STATE_MISSION */
		/* EVENT_NONE_REQUESTED */		{ACTION(&Navigator::start_none_in_air), NAV_STATE_NONE_IN_AIR},
		/* EVENT_LOITER_REQUESTED */		{ACTION(&Navigator::start_loiter), NAV_STATE_LOITER},
		/* EVENT_MISSION_REQUESTED */		{NO_ACTION, NAV_STATE_MISSION},
		/* EVENT_RTL_REQUESTED */		{ACTION(&Navigator::start_rtl), NAV_STATE_RTL},
		/* EVENT_TAKEN_OFF */			{NO_ACTION, NAV_STATE_MISSION},
		/* EVENT_LANDED */			{NO_ACTION, NAV_STATE_MISSION},
		/* EVENT_MISSION_CHANGED */		{ACTION(&Navigator::start_mission), NAV_STATE_MISSION},
		/* EVENT_HOME_POSITION_CHANGED */	{NO_ACTION, NAV_STATE_MISSION},
		/* EVENT_MISSION_ITEM_REACHED */	{ACTION(&Navigator::advance_mission), NAV_STATE_MISSION},
	},
	{
		/* NAV_STATE_RTL */
		/* EVENT_NONE_REQUESTED */		{ACTION(&Navigator::start_none_in_air), NAV_STATE_NONE_IN_AIR},
		/* EVENT_LOITER_REQUESTED */		{ACTION(&Navigator::start_loiter), NAV_STATE_LOITER},
		/* EVENT_MISSION_REQUESTED */		{ACTION(&Navigator::start_mission), NAV_STATE_MISSION},
		/* EVENT_RTL_REQUESTED */		{NO_ACTION, NAV_STATE_RTL},
		/* EVENT_TAKEN_OFF */			{NO_ACTION, NAV_STATE_RTL},
		/* EVENT_LANDED */			{NO_ACTION, NAV_STATE_RTL},
		/* EVENT_MISSION_CHANGED */		{NO_ACTION, NAV_STATE_RTL},
		/* EVENT_HOME_POSITION_CHANGED */	{ACTION(&Navigator::start_rtl), NAV_STATE_RTL},
		/* EVENT_MISSION_ITEM_REACHED */	{ACTION(&Navigator::advance_rtl), NAV_STATE_RTL},
	},
	{
		/* NAV_STATE_LAND */
		/* EVENT_NONE_REQUESTED */		{ACTION(&Navigator::start_none_in_air), NAV_STATE_NONE_IN_AIR},
		/* EVENT_LOITER_REQUESTED */		{ACTION(&Navigator::start_loiter), NAV_STATE_LOITER},
		/* EVENT_MISSION_REQUESTED */		{ACTION(&Navigator::start_mission), NAV_STATE_MISSION},
		/* EVENT_RTL_REQUESTED */		{ACTION(&Navigator::start_rtl), NAV_STATE_RTL},
		/* EVENT_TAKEN_OFF */			{NO_ACTION, NAV_STATE_LAND},
		/* EVENT_LANDED */			{ACTION(&Navigator::start_auto_on_ground), NAV_STATE_AUTO_ON_GROUND},
		/* EVENT_MISSION_CHANGED */		{NO_ACTION, NAV_STATE_LAND},
		/* EVENT_HOME_POSITION_CHANGED */	{NO_ACTION, NAV_STATE_LAND},
		/* EVENT_MISSION_ITEM_REACHED */	{NO_ACTION, NAV_STATE_LAND},
	},
};

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

		dispatch(EVENT_LOITER_REQUESTED);
		return false;
	}

	/* if we got an RTL mission item, switch to RTL mode and give up */
	if (_mission_item.nav_cmd == NAV_CMD_RETURN_TO_LAUNCH) {
		dispatch(EVENT_RTL_REQUESTED);
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
	dispatch(EVENT_LOITER_REQUESTED);
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

	dispatch(EVENT_LOITER_REQUESTED);
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

void
Navigator::mission_item_to_position_setpoint(const mission_item_s *item, position_setpoint_s *sp)
{
	sp->valid = true;

	if (item->nav_cmd == NAV_CMD_RETURN_TO_LAUNCH) {

		if (_pos_sp_triplet.previous.valid) {
			/* if previous setpoint is valid then use it to calculate heading to home */
			sp->yaw = get_bearing_to_next_waypoint(_pos_sp_triplet.previous.lat, _pos_sp_triplet.previous.lon, sp->lat, sp->lon);

		} else {
			/* else use current position */
			sp->yaw = get_bearing_to_next_waypoint(_global_pos.lat, _global_pos.lon, sp->lat, sp->lon);
		}

	} else {
		sp->lat = item->lat;
		sp->lon = item->lon;
		sp->alt = item->altitude_is_relative ? item->altitude + _home_pos.alt : item->altitude;
		sp->yaw = item->yaw;
		sp->loiter_radius = item->loiter_radius;
		sp->loiter_direction = item->loiter_direction;
		sp->pitch_min = item->pitch_min;
	}

	if (item->nav_cmd == NAV_CMD_TAKEOFF) {
		sp->type = SETPOINT_TYPE_TAKEOFF;

	} else if (item->nav_cmd == NAV_CMD_LAND) {
		sp->type = SETPOINT_TYPE_LAND;

	} else if (item->nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
		   item->nav_cmd == NAV_CMD_LOITER_TURN_COUNT ||
		   item->nav_cmd == NAV_CMD_LOITER_UNLIMITED) {
		sp->type = SETPOINT_TYPE_LOITER;

	} else {
		sp->type = SETPOINT_TYPE_NORMAL;
	}
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

	/* TODO: count turns */
	// if ((_mission_item.nav_cmd == NAV_CMD_LOITER_TURN_COUNT ||
	//      _mission_item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
	//      _mission_item.nav_cmd == NAV_CMD_LOITER_UNLIMITED) &&
	//     _mission_item.loiter_radius > 0.01f) {

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

		float dist = -1.0f;
		float dist_xy = -1.0f;
		float dist_z = -1.0f;

		float altitude_amsl = _mission_item.altitude_is_relative
					? _mission_item.altitude + _home_pos.alt : _mission_item.altitude;

		dist = get_distance_to_point_global_wgs84(_mission_item.lat, _mission_item.lon, altitude_amsl,
				_global_pos.lat, _global_pos.lon, _global_pos.alt,
				&dist_xy, &dist_z);

		if (_mission_item.nav_cmd == NAV_CMD_TAKEOFF) {

			/* require only altitude for takeoff */
			if (_global_pos.alt > altitude_amsl - acceptance_radius) {
				_waypoint_position_reached = true;
			}
		} else {
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

void
Navigator::publish_position_setpoint_triplet()
{
	/* update navigation state */
	_pos_sp_triplet.nav_state = static_cast<nav_state_t>(myState);

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
