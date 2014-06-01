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
 * Handles missions, geo fencing and failsafe navigation behavior.
 * Published the mission item triplet for the position controller.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Jean Cyr <jean.m.cyr@gmail.com>
 * @author Julian Oes <joes@student.ethz.ch>
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
#include <mathlib/mathlib.h>
#include <dataman/dataman.h>
#include <mavlink/mavlink_log.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "navigator_state.h"
#include "navigator_mission.h"
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
	void add_fence_point(int argc, char *argv[]);

	/**
	 * Load fence from file
	 */
	void load_fence_from_file(const char *filename);

private:

	bool		_task_should_exit;		/**< if true, sensor task should exit */
	int		_navigator_task;		/**< task handle for sensor task */

	int		_mavlink_fd;

	int		_global_pos_sub;		/**< global position subscription */
	int		_home_pos_sub;			/**< home position subscription */
	int		_vstatus_sub;			/**< vehicle status subscription */
	int		_params_sub;			/**< notification of parameter updates */
	int		_offboard_mission_sub;		/**< notification of offboard mission updates */
	int 		_onboard_mission_sub;		/**< notification of onboard mission updates */
	int		_capabilities_sub;		/**< notification of vehicle capabilities updates */
	int		_control_mode_sub;		/**< vehicle control mode subscription */

	orb_advert_t	_pos_sp_triplet_pub;		/**< publish position setpoint triplet */
	orb_advert_t	_mission_result_pub;		/**< publish mission result topic */

	struct vehicle_status_s				_vstatus;		/**< vehicle status */
	struct vehicle_control_mode_s			_control_mode;		/**< vehicle control mode */
	struct vehicle_global_position_s		_global_pos;		/**< global vehicle position */
	struct home_position_s				_home_pos;		/**< home position for RTL */
	struct position_setpoint_triplet_s		_pos_sp_triplet;	/**< triplet of position setpoints */
	struct mission_result_s				_mission_result;	/**< mission result for commander/mavlink */
	struct mission_item_s				_mission_item;		/**< current mission item */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	Geofence					_geofence;
	bool						_geofence_violation_warning_sent;

	bool						_fence_valid;		/**< flag if fence is valid */
	bool						_inside_fence;		/**< vehicle is inside fence */

	struct navigation_capabilities_s		_nav_caps;

	class 		Mission				_mission;

	bool		_mission_item_valid;		/**< current mission item valid */
	bool		_global_pos_valid;		/**< track changes of global_position */
	bool		_reset_loiter_pos;		/**< if true then loiter position should be set to current position */
	bool		_waypoint_position_reached;
	bool		_waypoint_yaw_reached;
	uint64_t	_time_first_inside_orbit;
	bool		_need_takeoff;			/**< if need to perform vertical takeoff before going to waypoint (only for MISSION mode and VTOL vehicles) */
	bool		_do_takeoff;			/**< vertical takeoff state, current mission item is generated by navigator (only for MISSION mode and VTOL vehicles) */

	MissionFeasibilityChecker missionFeasiblityChecker;

	uint64_t	_set_nav_state_timestamp;	/**< timestamp of last handled navigation state request */

	bool		_pos_sp_triplet_updated;

	const char *nav_states_str[NAV_STATE_MAX];

	struct {
		float min_altitude;
		float acceptance_radius;
		float loiter_radius;
		int onboard_mission_enabled;
		float takeoff_alt;
		float land_alt;
		float rtl_alt;
		float rtl_land_delay;
	}		_parameters;			/**< local copies of parameters */

	struct {
		param_t min_altitude;
		param_t acceptance_radius;
		param_t loiter_radius;
		param_t onboard_mission_enabled;
		param_t takeoff_alt;
		param_t land_alt;
		param_t rtl_alt;
		param_t rtl_land_delay;
	}		_parameter_handles;		/**< handles for parameters */

	enum Event {
		EVENT_NONE_REQUESTED,
		EVENT_READY_REQUESTED,
		EVENT_LOITER_REQUESTED,
		EVENT_MISSION_REQUESTED,
		EVENT_RTL_REQUESTED,
		EVENT_LAND_REQUESTED,
		EVENT_MISSION_CHANGED,
		EVENT_HOME_POSITION_CHANGED,
		MAX_EVENT
	};

	/**
	 * State machine transition table
	 */
	static StateTable::Tran const myTable[NAV_STATE_MAX][MAX_EVENT];

	enum RTLState {
		RTL_STATE_NONE = 0,
		RTL_STATE_CLIMB,
		RTL_STATE_RETURN,
		RTL_STATE_DESCEND
	};

	enum RTLState _rtl_state;

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
	void		offboard_mission_update(bool isrotaryWing);

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

	void		publish_safepoints(unsigned points);

	/**
	 * Functions that are triggered when a new state is entered.
	 */
	void		start_none();
	void		start_ready();
	void		start_loiter();
	void		start_mission();
	void		start_rtl();
	void		start_land();
	void		start_land_home();

	/**
	 * Fork for state transitions
	 */
	void		request_loiter_or_ready();
	void		request_mission_if_available();

	/**
	 * Guards offboard mission
	 */
	bool		offboard_mission_available(unsigned relative_index);

	/**
	 * Guards onboard mission
	 */
	bool		onboard_mission_available(unsigned relative_index);

	/**
	 * Reset all "reached" flags.
	 */
	void		reset_reached();

	/**
	 * Check if current mission item has been reached.
	 */
	bool		check_mission_item_reached();

	/**
	 * Perform actions when current mission item reached.
	 */
	void		on_mission_item_reached();

	/**
	 * Move to next waypoint
	 */
	void		set_mission_item();

	/**
	 * Switch to next RTL state
	 */
	void		set_rtl_item();

	/**
	 * Set position setpoint for mission item
	 */
	void		position_setpoint_from_mission_item(position_setpoint_s *sp, mission_item_s *item);

	/**
	 * Helper function to get a takeoff item
	 */
	void		get_takeoff_setpoint(position_setpoint_s *pos_sp);

	/**
	 * Publish a new mission item triplet for position controller
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

/* subscriptions */
	_global_pos_sub(-1),
	_home_pos_sub(-1),
	_vstatus_sub(-1),
	_params_sub(-1),
	_offboard_mission_sub(-1),
	_onboard_mission_sub(-1),
	_capabilities_sub(-1),
	_control_mode_sub(-1),

/* publications */
	_pos_sp_triplet_pub(-1),

/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "navigator")),

	_geofence_violation_warning_sent(false),
	_fence_valid(false),
	_inside_fence(true),
	_mission(),
	_mission_item_valid(false),
	_global_pos_valid(false),
	_reset_loiter_pos(true),
	_waypoint_position_reached(false),
	_waypoint_yaw_reached(false),
	_time_first_inside_orbit(0),
	_need_takeoff(true),
	_do_takeoff(false),
	_set_nav_state_timestamp(0),
	_pos_sp_triplet_updated(false),
/* states */
	_rtl_state(RTL_STATE_NONE)
{
	_parameter_handles.min_altitude = param_find("NAV_MIN_ALT");
	_parameter_handles.acceptance_radius = param_find("NAV_ACCEPT_RAD");
	_parameter_handles.loiter_radius = param_find("NAV_LOITER_RAD");
	_parameter_handles.onboard_mission_enabled = param_find("NAV_ONB_MIS_EN");
	_parameter_handles.takeoff_alt = param_find("NAV_TAKEOFF_ALT");
	_parameter_handles.land_alt = param_find("NAV_LAND_ALT");
	_parameter_handles.rtl_alt = param_find("NAV_RTL_ALT");
	_parameter_handles.rtl_land_delay = param_find("NAV_RTL_LAND_T");

	memset(&_pos_sp_triplet, 0, sizeof(struct position_setpoint_triplet_s));
	memset(&_mission_item, 0, sizeof(struct mission_item_s));

	memset(&nav_states_str, 0, sizeof(nav_states_str));
	nav_states_str[0] = "NONE";
	nav_states_str[1] = "READY";
	nav_states_str[2] = "LOITER";
	nav_states_str[3] = "MISSION";
	nav_states_str[4] = "RTL";
	nav_states_str[5] = "LAND";

	/* Initialize state machine */
	myState = NAV_STATE_NONE;
	start_none();
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

	param_get(_parameter_handles.min_altitude, &(_parameters.min_altitude));
	param_get(_parameter_handles.acceptance_radius, &(_parameters.acceptance_radius));
	param_get(_parameter_handles.loiter_radius, &(_parameters.loiter_radius));
	param_get(_parameter_handles.onboard_mission_enabled, &(_parameters.onboard_mission_enabled));
	param_get(_parameter_handles.takeoff_alt, &(_parameters.takeoff_alt));
	param_get(_parameter_handles.land_alt, &(_parameters.land_alt));
	param_get(_parameter_handles.rtl_alt, &(_parameters.rtl_alt));
	param_get(_parameter_handles.rtl_land_delay, &(_parameters.rtl_land_delay));

	_mission.set_onboard_mission_allowed((bool)_parameter_handles.onboard_mission_enabled);

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
}

void
Navigator::navigation_capabilities_update()
{
	orb_copy(ORB_ID(navigation_capabilities), _capabilities_sub, &_nav_caps);
}


void
Navigator::offboard_mission_update(bool isrotaryWing)
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

		missionFeasiblityChecker.checkMissionFeasible(isrotaryWing, dm_current, (size_t)offboard_mission.count, _geofence, _home_pos.alt);

		_mission.set_offboard_dataman_id(offboard_mission.dataman_id);

		_mission.set_offboard_mission_count(offboard_mission.count);
		_mission.set_current_offboard_mission_index(offboard_mission.current_index);

	} else {
		_mission.set_offboard_mission_count(0);
		_mission.set_current_offboard_mission_index(0);
	}

	_mission.publish_mission_result();
}

void
Navigator::onboard_mission_update()
{
	struct mission_s onboard_mission;

	if (orb_copy(ORB_ID(onboard_mission), _onboard_mission_sub, &onboard_mission) == OK) {

		_mission.set_onboard_mission_count(onboard_mission.count);
		_mission.set_current_onboard_mission_index(onboard_mission.current_index);

	} else {
		_mission.set_onboard_mission_count(0);
		_mission.set_current_onboard_mission_index(0);
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
	fflush(stdout);

	_mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

	mavlink_log_info(_mavlink_fd, "[navigator] started");

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
	offboard_mission_update(_vstatus.is_rotary_wing);
	onboard_mission_update();

	/* rate limit position updates to 50 Hz */
	orb_set_interval(_global_pos_sub, 20);

	unsigned prevState = NAV_STATE_NONE;
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

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
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

			/* evaluate state requested by commander */
			if (_control_mode.flag_armed && _control_mode.flag_control_auto_enabled) {
				/* publish position setpoint triplet on each status update if navigator active */
				_pos_sp_triplet_updated = true;

				if (_vstatus.set_nav_state_timestamp != _set_nav_state_timestamp) {
					/* commander requested new navigation mode, try to set it */
					switch (_vstatus.set_nav_state) {
					case NAV_STATE_NONE:
						/* nothing to do */
						break;

					case NAV_STATE_LOITER:
						request_loiter_or_ready();
						break;

					case NAV_STATE_MISSION:
						request_mission_if_available();
						break;

					case NAV_STATE_RTL:
						if (!(_rtl_state == RTL_STATE_DESCEND &&
							  (myState == NAV_STATE_LAND || myState == NAV_STATE_LOITER)) &&
							_vstatus.condition_home_position_valid) {
							dispatch(EVENT_RTL_REQUESTED);
						}

						break;

					case NAV_STATE_LAND:
						dispatch(EVENT_LAND_REQUESTED);

						break;

					default:
						warnx("ERROR: Requested navigation state not supported");
						break;
					}

				} else {
					/* on first switch to AUTO try mission by default, if none is available fallback to loiter */
					if (myState == NAV_STATE_NONE) {
						request_mission_if_available();
					}
				}

				/* check if waypoint has been reached in MISSION, RTL and LAND modes */
				if (myState == NAV_STATE_MISSION || myState == NAV_STATE_RTL || myState == NAV_STATE_LAND) {
					if (check_mission_item_reached()) {
						on_mission_item_reached();
					}
				}

			} else {
				/* navigator shouldn't act */
				dispatch(EVENT_NONE_REQUESTED);
			}

			_set_nav_state_timestamp = _vstatus.set_nav_state_timestamp;
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
			offboard_mission_update(_vstatus.is_rotary_wing);
			// XXX check if mission really changed
			dispatch(EVENT_MISSION_CHANGED);
		}

		/* onboard mission updated */
		if (fds[5].revents & POLLIN) {
			onboard_mission_update();
			// XXX check if mission really changed
			dispatch(EVENT_MISSION_CHANGED);
		}

		/* home position updated */
		if (fds[2].revents & POLLIN) {
			home_position_update();
			// XXX check if home position really changed
			dispatch(EVENT_HOME_POSITION_CHANGED);
		}

		/* global position updated */
		if (fds[1].revents & POLLIN) {
			global_position_update();

			if (_control_mode.flag_armed && _control_mode.flag_control_auto_enabled) {
				/* publish position setpoint triplet on each position update if navigator active */
				_pos_sp_triplet_updated = true;

				if (myState == NAV_STATE_LAND && !_global_pos_valid) {
					/* got global position when landing, update setpoint */
					start_land();
				}

				/* check if waypoint has been reached in MISSION, RTL and LAND modes */
				if (myState == NAV_STATE_MISSION || myState == NAV_STATE_RTL || myState == NAV_STATE_LAND) {
					if (check_mission_item_reached()) {
						on_mission_item_reached();
					}
				}
			}

			/* Check geofence violation */
			if (!_geofence.inside(&_global_pos)) {
				//xxx: publish geofence violation here (or change local flag depending on which app handles the flight termination)

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

		_global_pos_valid = _vstatus.condition_global_position_valid;

		/* publish position setpoint triplet if updated */
		if (_pos_sp_triplet_updated) {
			_pos_sp_triplet_updated = false;
			publish_position_setpoint_triplet();
		}

		/* notify user about state changes */
		if (myState != prevState) {
			mavlink_log_info(_mavlink_fd, "#audio: navigation state: %s", nav_states_str[myState]);
			prevState = myState;
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
	warnx("Global position: %svalid", _global_pos_valid ? "" : "in");

	if (_global_pos_valid) {
		warnx("Longitude %5.5f degrees, latitude %5.5f degrees", _global_pos.lon, _global_pos.lat);
		warnx("Altitude %5.5f meters, altitude above home %5.5f meters",
		      (double)_global_pos.alt, (double)(_global_pos.alt - _home_pos.alt));
		warnx("Ground velocity in m/s, N %5.5f, E %5.5f, D %5.5f",
		      (double)_global_pos.vel_n, (double)_global_pos.vel_e, (double)_global_pos.vel_d);
		warnx("Compass heading in degrees %5.5f", (double)(_global_pos.yaw * M_RAD_TO_DEG_F));
	}

	if (_fence_valid) {
		warnx("Geofence is valid");
//		warnx("Vertex longitude latitude");
//		for (unsigned i = 0; i < _fence.count; i++)
//		warnx("%6u %9.5f %8.5f", i, (double)_fence.vertices[i].lon, (double)_fence.vertices[i].lat);

	} else {
		warnx("Geofence not set");
	}

	switch (myState) {
	case NAV_STATE_NONE:
		warnx("State: None");
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

	default:
		warnx("State: Unknown");
		break;
	}
}

StateTable::Tran const Navigator::myTable[NAV_STATE_MAX][MAX_EVENT] = {
	{
		/* NAV_STATE_NONE */
		/* EVENT_NONE_REQUESTED */		{NO_ACTION, NAV_STATE_NONE},
		/* EVENT_READY_REQUESTED */		{ACTION(&Navigator::start_ready), NAV_STATE_READY},
		/* EVENT_LOITER_REQUESTED */		{ACTION(&Navigator::start_loiter), NAV_STATE_LOITER},
		/* EVENT_MISSION_REQUESTED */		{ACTION(&Navigator::start_mission), NAV_STATE_MISSION},
		/* EVENT_RTL_REQUESTED */		{ACTION(&Navigator::start_rtl), NAV_STATE_RTL},
		/* EVENT_LAND_REQUESTED */		{ACTION(&Navigator::start_land), NAV_STATE_LAND},
		/* EVENT_MISSION_CHANGED */		{NO_ACTION, NAV_STATE_NONE},
		/* EVENT_HOME_POSITION_CHANGED */	{NO_ACTION, NAV_STATE_NONE},
	},
	{
		/* NAV_STATE_READY */
		/* EVENT_NONE_REQUESTED */		{ACTION(&Navigator::start_none), NAV_STATE_NONE},
		/* EVENT_READY_REQUESTED */		{NO_ACTION, NAV_STATE_READY},
		/* EVENT_LOITER_REQUESTED */		{NO_ACTION, NAV_STATE_READY},
		/* EVENT_MISSION_REQUESTED */		{ACTION(&Navigator::start_mission), NAV_STATE_MISSION},
		/* EVENT_RTL_REQUESTED */		{NO_ACTION, NAV_STATE_READY},
		/* EVENT_LAND_REQUESTED */		{NO_ACTION, NAV_STATE_READY},
		/* EVENT_MISSION_CHANGED */		{NO_ACTION, NAV_STATE_READY},
		/* EVENT_HOME_POSITION_CHANGED */	{NO_ACTION, NAV_STATE_READY},
	},
	{
		/* NAV_STATE_LOITER */
		/* EVENT_NONE_REQUESTED */		{ACTION(&Navigator::start_none), NAV_STATE_NONE},
		/* EVENT_READY_REQUESTED */		{NO_ACTION, NAV_STATE_LOITER},
		/* EVENT_LOITER_REQUESTED */		{NO_ACTION, NAV_STATE_LOITER},
		/* EVENT_MISSION_REQUESTED */		{ACTION(&Navigator::start_mission), NAV_STATE_MISSION},
		/* EVENT_RTL_REQUESTED */		{ACTION(&Navigator::start_rtl), NAV_STATE_RTL},
		/* EVENT_LAND_REQUESTED */		{ACTION(&Navigator::start_land), NAV_STATE_LAND},
		/* EVENT_MISSION_CHANGED */		{NO_ACTION, NAV_STATE_LOITER},
		/* EVENT_HOME_POSITION_CHANGED */	{NO_ACTION, NAV_STATE_LOITER},
	},
	{
		/* NAV_STATE_MISSION */
		/* EVENT_NONE_REQUESTED */		{ACTION(&Navigator::start_none), NAV_STATE_NONE},
		/* EVENT_READY_REQUESTED */		{ACTION(&Navigator::start_ready), NAV_STATE_READY},
		/* EVENT_LOITER_REQUESTED */		{ACTION(&Navigator::start_loiter), NAV_STATE_LOITER},
		/* EVENT_MISSION_REQUESTED */		{NO_ACTION, NAV_STATE_MISSION},
		/* EVENT_RTL_REQUESTED */		{ACTION(&Navigator::start_rtl), NAV_STATE_RTL},
		/* EVENT_LAND_REQUESTED */		{ACTION(&Navigator::start_land), NAV_STATE_LAND},
		/* EVENT_MISSION_CHANGED */		{ACTION(&Navigator::start_mission), NAV_STATE_MISSION},
		/* EVENT_HOME_POSITION_CHANGED */	{NO_ACTION, NAV_STATE_MISSION},
	},
	{
		/* NAV_STATE_RTL */
		/* EVENT_NONE_REQUESTED */		{ACTION(&Navigator::start_none), NAV_STATE_NONE},
		/* EVENT_READY_REQUESTED */		{ACTION(&Navigator::start_ready), NAV_STATE_READY},
		/* EVENT_LOITER_REQUESTED */		{ACTION(&Navigator::start_loiter), NAV_STATE_LOITER},
		/* EVENT_MISSION_REQUESTED */		{ACTION(&Navigator::start_mission), NAV_STATE_MISSION},
		/* EVENT_RTL_REQUESTED */		{NO_ACTION, NAV_STATE_RTL},
		/* EVENT_LAND_REQUESTED */		{ACTION(&Navigator::start_land_home), NAV_STATE_LAND},
		/* EVENT_MISSION_CHANGED */		{NO_ACTION, NAV_STATE_RTL},
		/* EVENT_HOME_POSITION_CHANGED */	{ACTION(&Navigator::start_rtl), NAV_STATE_RTL},	// TODO need to reset rtl_state
	},
	{
		/* NAV_STATE_LAND */
		/* EVENT_NONE_REQUESTED */		{ACTION(&Navigator::start_none), NAV_STATE_NONE},
		/* EVENT_READY_REQUESTED */		{ACTION(&Navigator::start_ready), NAV_STATE_READY},
		/* EVENT_LOITER_REQUESTED */		{ACTION(&Navigator::start_loiter), NAV_STATE_LOITER},
		/* EVENT_MISSION_REQUESTED */		{ACTION(&Navigator::start_mission), NAV_STATE_MISSION},
		/* EVENT_RTL_REQUESTED */		{ACTION(&Navigator::start_rtl), NAV_STATE_RTL},
		/* EVENT_LAND_REQUESTED */		{NO_ACTION, NAV_STATE_LAND},
		/* EVENT_MISSION_CHANGED */		{NO_ACTION, NAV_STATE_LAND},
		/* EVENT_HOME_POSITION_CHANGED */	{NO_ACTION, NAV_STATE_LAND},
	},
};

void
Navigator::start_none()
{
	reset_reached();

	_pos_sp_triplet.previous.valid = false;
	_pos_sp_triplet.current.valid = false;
	_pos_sp_triplet.next.valid = false;
	_mission_item_valid = false;

	_reset_loiter_pos = true;
	_do_takeoff = false;
	_rtl_state = RTL_STATE_NONE;

	_pos_sp_triplet_updated = true;
}

void
Navigator::start_ready()
{
	reset_reached();

	_pos_sp_triplet.previous.valid = false;
	_pos_sp_triplet.current.valid = true;
	_pos_sp_triplet.next.valid = false;

	_pos_sp_triplet.current.type = SETPOINT_TYPE_IDLE;

	_mission_item_valid = false;

	_reset_loiter_pos = true;
	_do_takeoff = false;

	if (_rtl_state != RTL_STATE_DESCEND) {
		/* reset RTL state if landed not at home */
		_rtl_state = RTL_STATE_NONE;
	}

	_pos_sp_triplet_updated = true;
}

void
Navigator::start_loiter()
{
	reset_reached();

	_do_takeoff = false;

	/* set loiter position if needed */
	if (_reset_loiter_pos || !_pos_sp_triplet.current.valid) {
		_reset_loiter_pos = false;

		_pos_sp_triplet.current.lat = _global_pos.lat;
		_pos_sp_triplet.current.lon = _global_pos.lon;
		_pos_sp_triplet.current.yaw = NAN;	// NAN means to use current yaw

		float min_alt_amsl = _parameters.min_altitude + _home_pos.alt;

		/* use current altitude if above min altitude set by parameter */
		if (_global_pos.alt < min_alt_amsl && !_vstatus.is_rotary_wing) {
			_pos_sp_triplet.current.alt = min_alt_amsl;
			mavlink_log_info(_mavlink_fd, "#audio: loiter %.1fm higher", (double)(min_alt_amsl - _global_pos.alt));

		} else {
			_pos_sp_triplet.current.alt = _global_pos.alt;
			mavlink_log_info(_mavlink_fd, "#audio: loiter at current altitude");
		}

	}
	_pos_sp_triplet.current.type = SETPOINT_TYPE_LOITER;
	_pos_sp_triplet.current.loiter_radius = _parameters.loiter_radius;
	_pos_sp_triplet.current.loiter_direction = 1;
	_pos_sp_triplet.previous.valid = false;
	_pos_sp_triplet.current.valid = true;
	_pos_sp_triplet.next.valid = false;
	_mission_item_valid = false;

	_pos_sp_triplet_updated = true;
}

void
Navigator::start_mission()
{
	_need_takeoff = true;

	set_mission_item();
}

void
Navigator::set_mission_item()
{
	reset_reached();

	/* copy current mission to previous item */
	memcpy(&_pos_sp_triplet.previous, &_pos_sp_triplet.current, sizeof(position_setpoint_s));

	_reset_loiter_pos = true;
	_do_takeoff = false;

	int ret;
	bool onboard;
	unsigned index;

	ret = _mission.get_current_mission_item(&_mission_item, &onboard, &index);

	if (ret == OK) {
		_mission.report_current_offboard_mission_item();

		_mission_item_valid = true;
		position_setpoint_from_mission_item(&_pos_sp_triplet.current, &_mission_item);

		if (_mission_item.nav_cmd != NAV_CMD_RETURN_TO_LAUNCH &&
		    _mission_item.nav_cmd != NAV_CMD_LOITER_TIME_LIMIT &&
		    _mission_item.nav_cmd != NAV_CMD_LOITER_TURN_COUNT &&
		    _mission_item.nav_cmd != NAV_CMD_LOITER_UNLIMITED) {
			/* don't reset RTL state on RTL or LOITER items */
			_rtl_state = RTL_STATE_NONE;
		}

		if (_vstatus.is_rotary_wing) {
			if (_need_takeoff && (
				    _mission_item.nav_cmd == NAV_CMD_TAKEOFF ||
				    _mission_item.nav_cmd == NAV_CMD_WAYPOINT ||
				    _mission_item.nav_cmd == NAV_CMD_RETURN_TO_LAUNCH ||
				    _mission_item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
				    _mission_item.nav_cmd == NAV_CMD_LOITER_TURN_COUNT ||
				    _mission_item.nav_cmd == NAV_CMD_LOITER_UNLIMITED
			    )) {
				/* do special TAKEOFF handling for VTOL */
				_need_takeoff = false;

				/* calculate desired takeoff altitude AMSL */
				float takeoff_alt_amsl = _pos_sp_triplet.current.alt;

				if (_vstatus.condition_landed) {
					/* takeoff to at least NAV_TAKEOFF_ALT from ground if landed */
					takeoff_alt_amsl = fmaxf(takeoff_alt_amsl, _global_pos.alt + _parameters.takeoff_alt);
				}

				/* check if we really need takeoff */
				if (_vstatus.condition_landed || _global_pos.alt < takeoff_alt_amsl - _mission_item.acceptance_radius) {
					/* force TAKEOFF if landed or waypoint altitude is more than current */
					_do_takeoff = true;

					/* move current position setpoint to next */
					memcpy(&_pos_sp_triplet.next, &_pos_sp_triplet.current, sizeof(position_setpoint_s));

					/* set current setpoint to takeoff */

					_pos_sp_triplet.current.lat = _global_pos.lat;
					_pos_sp_triplet.current.lon = _global_pos.lon;
					_pos_sp_triplet.current.alt = takeoff_alt_amsl;
					_pos_sp_triplet.current.yaw = NAN;
					_pos_sp_triplet.current.type = SETPOINT_TYPE_TAKEOFF;
				}

			} else if (_mission_item.nav_cmd == NAV_CMD_LAND) {
				/* will need takeoff after landing */
				_need_takeoff = true;
			}
		}

		if (_do_takeoff) {
			mavlink_log_info(_mavlink_fd, "#audio: takeoff to %.1fm above home", (double)(_pos_sp_triplet.current.alt - _home_pos.alt));

		} else {
			if (onboard) {
				mavlink_log_info(_mavlink_fd, "#audio: heading to onboard WP %d", index);

			} else {
				mavlink_log_info(_mavlink_fd, "#audio: heading to offboard WP %d", index);
			}
		}

	} else {
		/* since a mission is not advanced without WPs available, this is not supposed to happen */
		_mission_item_valid = false;
		_pos_sp_triplet.current.valid = false;
		warnx("ERROR: current WP can't be set");
	}

	if (!_do_takeoff) {
		mission_item_s item_next;
		ret = _mission.get_next_mission_item(&item_next);

		if (ret == OK) {
			position_setpoint_from_mission_item(&_pos_sp_triplet.next, &item_next);

		} else {
			/* this will fail for the last WP */
			_pos_sp_triplet.next.valid = false;
		}
	}

	_pos_sp_triplet_updated = true;
}

void
Navigator::start_rtl()
{
	_do_takeoff = false;

	/* decide if we need climb */
	if (_rtl_state == RTL_STATE_NONE) {
		if (_global_pos.alt < _home_pos.alt + _parameters.rtl_alt) {
			_rtl_state = RTL_STATE_CLIMB;

		} else {
			_rtl_state = RTL_STATE_RETURN;
		}
	}

	/* if switching directly to return state, reset altitude setpoint */
	if (_rtl_state == RTL_STATE_RETURN) {
		_mission_item.altitude_is_relative = false;
		_mission_item.altitude = _global_pos.alt;
	}

	_reset_loiter_pos = true;
	set_rtl_item();
}

void
Navigator::start_land()
{
	reset_reached();

	/* this state can be requested by commander even if no global position available,
	 * in his case controller must perform landing without position control */
	_do_takeoff = false;
	_reset_loiter_pos = true;

	memcpy(&_pos_sp_triplet.previous, &_pos_sp_triplet.current, sizeof(position_setpoint_s));

	_mission_item_valid = true;

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

	position_setpoint_from_mission_item(&_pos_sp_triplet.current, &_mission_item);

	_pos_sp_triplet.next.valid = false;
}

void
Navigator::start_land_home()
{
	reset_reached();

	/* land to home position, should be called when hovering above home, from RTL state */
	_do_takeoff = false;
	_reset_loiter_pos = true;

	memcpy(&_pos_sp_triplet.previous, &_pos_sp_triplet.current, sizeof(position_setpoint_s));

	_mission_item_valid = true;

	_mission_item.lat = _home_pos.lat;
	_mission_item.lon = _home_pos.lon;
	_mission_item.altitude_is_relative = false;
	_mission_item.altitude = _home_pos.alt;
	_mission_item.yaw = NAN;
	_mission_item.loiter_radius = _parameters.loiter_radius;
	_mission_item.loiter_direction = 1;
	_mission_item.nav_cmd = NAV_CMD_LAND;
	_mission_item.acceptance_radius = _parameters.acceptance_radius;
	_mission_item.time_inside = 0.0f;
	_mission_item.pitch_min = 0.0f;
	_mission_item.autocontinue = true;
	_mission_item.origin = ORIGIN_ONBOARD;

	position_setpoint_from_mission_item(&_pos_sp_triplet.current, &_mission_item);

	_pos_sp_triplet.next.valid = false;
}

void
Navigator::set_rtl_item()
{
	reset_reached();

	switch (_rtl_state) {
	case RTL_STATE_CLIMB: {
			memcpy(&_pos_sp_triplet.previous, &_pos_sp_triplet.current, sizeof(position_setpoint_s));

			float climb_alt = _home_pos.alt + _parameters.rtl_alt;

			if (_vstatus.condition_landed) {
				climb_alt = fmaxf(climb_alt, _global_pos.alt + _parameters.rtl_alt);
			}

			_mission_item_valid = true;

			_mission_item.lat = _global_pos.lat;
			_mission_item.lon = _global_pos.lon;
			_mission_item.altitude_is_relative = false;
			_mission_item.altitude = climb_alt;
			_mission_item.yaw = NAN;
			_mission_item.loiter_radius = _parameters.loiter_radius;
			_mission_item.loiter_direction = 1;
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.acceptance_radius = _parameters.acceptance_radius;
			_mission_item.time_inside = 0.0f;
			_mission_item.pitch_min = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			position_setpoint_from_mission_item(&_pos_sp_triplet.current, &_mission_item);

			_pos_sp_triplet.next.valid = false;

			mavlink_log_info(_mavlink_fd, "#audio: RTL: climb to %.1fm above home", (double)(climb_alt - _home_pos.alt));
			break;
		}

	case RTL_STATE_RETURN: {
			memcpy(&_pos_sp_triplet.previous, &_pos_sp_triplet.current, sizeof(position_setpoint_s));

			_mission_item_valid = true;

			_mission_item.lat = _home_pos.lat;
			_mission_item.lon = _home_pos.lon;
			// don't change altitude
			if (_pos_sp_triplet.previous.valid) {
				/* if previous setpoint is valid then use it to calculate heading to home */
				_mission_item.yaw = get_bearing_to_next_waypoint(_pos_sp_triplet.previous.lat, _pos_sp_triplet.previous.lon, _mission_item.lat, _mission_item.lon);

			} else {
				/* else use current position */
				_mission_item.yaw = get_bearing_to_next_waypoint(_global_pos.lat, _global_pos.lon, _mission_item.lat, _mission_item.lon);
			}
			_mission_item.loiter_radius = _parameters.loiter_radius;
			_mission_item.loiter_direction = 1;
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.acceptance_radius = _parameters.acceptance_radius;
			_mission_item.time_inside = 0.0f;
			_mission_item.pitch_min = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			position_setpoint_from_mission_item(&_pos_sp_triplet.current, &_mission_item);

			_pos_sp_triplet.next.valid = false;

			mavlink_log_info(_mavlink_fd, "#audio: RTL: return at %.1fm above home", (double)(_mission_item.altitude - _home_pos.alt));
			break;
		}

	case RTL_STATE_DESCEND: {
			memcpy(&_pos_sp_triplet.previous, &_pos_sp_triplet.current, sizeof(position_setpoint_s));

			_mission_item_valid = true;

			_mission_item.lat = _home_pos.lat;
			_mission_item.lon = _home_pos.lon;
			_mission_item.altitude_is_relative = false;
			_mission_item.altitude = _home_pos.alt + _parameters.land_alt;
			_mission_item.yaw = NAN;
			_mission_item.loiter_radius = _parameters.loiter_radius;
			_mission_item.loiter_direction = 1;
			_mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
			_mission_item.acceptance_radius = _parameters.acceptance_radius;
			_mission_item.time_inside = _parameters.rtl_land_delay < 0.0f ? 0.0f : _parameters.rtl_land_delay;
			_mission_item.pitch_min = 0.0f;
			_mission_item.autocontinue = _parameters.rtl_land_delay > -0.001f;
			_mission_item.origin = ORIGIN_ONBOARD;

			position_setpoint_from_mission_item(&_pos_sp_triplet.current, &_mission_item);

			_pos_sp_triplet.next.valid = false;

			mavlink_log_info(_mavlink_fd, "#audio: RTL: descend to %.1fm above home", (double)(_mission_item.altitude - _home_pos.alt));
			break;
		}

	default: {
			mavlink_log_critical(_mavlink_fd, "#audio: [navigator] error: unknown RTL state: %d", _rtl_state);
			start_loiter();
			break;
		}
	}

	_pos_sp_triplet_updated = true;
}

void
Navigator::request_loiter_or_ready()
{
	/* XXX workaround: no landing detector for fixedwing yet */
	if (_vstatus.condition_landed && _vstatus.is_rotary_wing) {
		dispatch(EVENT_READY_REQUESTED);

	} else {
		dispatch(EVENT_LOITER_REQUESTED);
	}
}

void
Navigator::request_mission_if_available()
{
	if (_mission.current_mission_available()) {
		dispatch(EVENT_MISSION_REQUESTED);

	} else {
		request_loiter_or_ready();
	}
}

void
Navigator::position_setpoint_from_mission_item(position_setpoint_s *sp, mission_item_s *item)
{
	sp->valid = true;

	if (item->nav_cmd == NAV_CMD_RETURN_TO_LAUNCH) {
		/* set home position for RTL item */
		sp->lat = _home_pos.lat;
		sp->lon = _home_pos.lon;
		sp->alt = _home_pos.alt + _parameters.rtl_alt;

		if (_pos_sp_triplet.previous.valid) {
			/* if previous setpoint is valid then use it to calculate heading to home */
			sp->yaw = get_bearing_to_next_waypoint(_pos_sp_triplet.previous.lat, _pos_sp_triplet.previous.lon, sp->lat, sp->lon);

		} else {
			/* else use current position */
			sp->yaw = get_bearing_to_next_waypoint(_global_pos.lat, _global_pos.lon, sp->lat, sp->lon);
		}
		sp->loiter_radius = _parameters.loiter_radius;
		sp->loiter_direction = 1;
		sp->pitch_min = 0.0f;

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

	/* XXX TODO count turns */
	if ((_mission_item.nav_cmd == NAV_CMD_LOITER_TURN_COUNT ||
	     _mission_item.nav_cmd == NAV_CMD_LOITER_UNLIMITED) &&
	    _mission_item.loiter_radius > 0.01f) {

		return false;
	}

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
		if (_vstatus.is_rotary_wing && !_do_takeoff && isfinite(_mission_item.yaw)) {
			/* check yaw if defined only for rotary wing except takeoff */
			float yaw_err = _wrap_pi(_mission_item.yaw - _global_pos.yaw);

			if (fabsf(yaw_err) < 0.2f) { /* XXX get rid of magic number */
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
				mavlink_log_info(_mavlink_fd, "#audio: waypoint reached, wait for %.1fs", (double)_mission_item.time_inside);
			}
		}

		/* check if the MAV was long enough inside the waypoint orbit */
		if ((now - _time_first_inside_orbit >= (uint64_t)_mission_item.time_inside * 1e6)
		    || _mission_item.nav_cmd == NAV_CMD_TAKEOFF) {
			reset_reached();
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
Navigator::on_mission_item_reached()
{
	if (myState == NAV_STATE_MISSION) {

		_mission.report_mission_item_reached();

		if (_do_takeoff) {
			/* takeoff completed */
			_do_takeoff = false;
			mavlink_log_info(_mavlink_fd, "#audio: takeoff completed");

		} else {
			/* advance by one mission item */
			_mission.move_to_next();
		}

		if (_mission.current_mission_available()) {
			if (_mission_item.autocontinue) {
				/* continue mission */
				set_mission_item();

			} else {
				/* autocontinue disabled for this item */
				request_loiter_or_ready();
			}

		} else {
			/* if no more mission items available then finish mission */
			/* loiter at last waypoint */
			_reset_loiter_pos = false;
			mavlink_log_info(_mavlink_fd, "[navigator] mission completed");
			request_loiter_or_ready();
		}

	} else if (myState == NAV_STATE_RTL) {
		/* RTL completed */
		if (_rtl_state == RTL_STATE_DESCEND) {
			/* hovering above home position, land if needed or loiter */
			mavlink_log_info(_mavlink_fd, "[navigator] RTL completed");

			if (_mission_item.autocontinue) {
				dispatch(EVENT_LAND_REQUESTED);

			} else {
				_reset_loiter_pos = false;
				dispatch(EVENT_LOITER_REQUESTED);
			}

		} else {
			/* next RTL step */
			_rtl_state = (RTLState)(_rtl_state + 1);
			set_rtl_item();
		}

	} else if (myState == NAV_STATE_LAND) {
		/* landing completed */
		mavlink_log_info(_mavlink_fd, "[navigator] landing completed");
		dispatch(EVENT_READY_REQUESTED);
	}
	_mission.publish_mission_result();
}

void
Navigator::publish_position_setpoint_triplet()
{
	/* update navigation state */
	_pos_sp_triplet.nav_state = static_cast<nav_state_t>(myState);

	/* lazily publish the position setpoint triplet only once available */
	if (_pos_sp_triplet_pub > 0) {
		/* publish the position setpoint triplet */
		orb_publish(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_pub, &_pos_sp_triplet);

	} else {
		/* advertise and publish */
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
