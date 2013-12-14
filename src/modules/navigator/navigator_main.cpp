/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: @author Lorenz Meier <lm@inf.ethz.ch>
 *           @author Jean Cyr <jean.m.cyr@gmail.com>
 *           @author Julian Oes <joes@student.ethz.ch>
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
 * @file navigator_main.c
 * Implementation of the main navigation state machine.
 *
 * Handles missions, geo fencing and failsafe navigation behavior.
 * Published the mission item triplet for the position controller.
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
#include <uORB/topics/mission_item_triplet.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/vehicle_status.h>
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
	 * Destructor, also kills the sensors task.
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
	 * Load fence parameters.
	 */
	bool		load_fence(unsigned vertices);

	/**
	 * Specify fence vertex position.
	 */
	void		fence_point(int argc, char *argv[]);

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

	orb_advert_t	_triplet_pub;			/**< publish position setpoint triplet */
	orb_advert_t	_fence_pub;			/**< publish fence topic */ 
	orb_advert_t	_mission_result_pub;		/**< publish mission result topic */

	struct vehicle_status_s				_vstatus;		/**< vehicle status */
	struct vehicle_global_position_s		_global_pos;		/**< global vehicle position */
	struct home_position_s				_home_pos;		/**< home position for RTL */
	struct mission_item_triplet_s			_mission_item_triplet;	/**< triplet of mission items */
	struct mission_result_s				_mission_result;	/**< mission result for commander/mavlink */

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	
	struct fence_s 					_fence;			/**< storage for fence vertices */
	bool						_fence_valid;		/**< flag if fence is valid */
        bool						_inside_fence;		/**< vehicle is inside fence */

	struct navigation_capabilities_s		_nav_caps;

	unsigned	_current_offboard_mission_index;
	unsigned	_current_onboard_mission_index;
	unsigned	_offboard_mission_item_count;	/** number of offboard mission items copied */
	unsigned	_onboard_mission_item_count;	/** number of onboard mission items copied */

	enum {
		MISSION_TYPE_NONE,
		MISSION_TYPE_ONBOARD,
		MISSION_TYPE_OFFBOARD,
	} _mission_type;

	bool		_waypoint_position_reached;
	bool		_waypoint_yaw_reached;
	uint64_t	_time_first_inside_orbit;

	struct {
		float min_altitude;
		float loiter_radius;
		int onboard_mission_enabled;
	}		_parameters;			/**< local copies of parameters */

	struct {
		param_t min_altitude;
		param_t loiter_radius;
		param_t onboard_mission_enabled;

	}		_parameter_handles;		/**< handles for parameters */

	enum Event {
		EVENT_NONE_REQUESTED,
		EVENT_LOITER_REQUESTED,
		EVENT_MISSION_REQUESTED,
		EVENT_RTL_REQUESTED,
		EVENT_MISSION_FINISHED,
		EVENT_MISSION_CHANGED,
		EVENT_HOME_POSITION_CHANGED,
		MAX_EVENT
	};

	enum State {
		STATE_INIT,
		STATE_NONE,
		STATE_LOITER,
		STATE_MISSION,
		STATE_MISSION_LOITER,
		STATE_RTL,
		STATE_RTL_LOITER,
		MAX_STATE
	};

	/**
	 * State machine transition table
	 */
	static StateTable::Tran const myTable[MAX_STATE][MAX_EVENT];

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
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main() __attribute__((noreturn));

	void		publish_fence(unsigned vertices);

	void		publish_safepoints(unsigned points);

	bool		fence_valid(const struct fence_s &fence);

	/**
	 * Functions that are triggered when a new state is entered.
	 */
	void		start_none();
	void		start_loiter();
	void		start_mission();
	void		start_mission_loiter();
	void		start_rtl();
	void		start_rtl_loiter();

	/**
	 * Guards offboard mission
	 */
	bool		offboard_mission_available(unsigned relative_index);

	/**
	 * Guards onboard mission
	 */
	bool		onboard_mission_available(unsigned relative_index);

	/**
	 * Check if current mission item has been reached.
	 */
	bool		mission_item_reached();

	/**
	 * Move to next waypoint
	 */
	int		advance_mission();

	/**
	 * Helper function to set a mission item
	 */
	int		set_mission_item(unsigned relative_index, struct mission_item_s *mission_item) ;

	/**
	 * Helper function to set a loiter item
	 */
	void		set_loiter_item(mission_item_s *new_loiter_position);

	/**
	 * Publish a new mission item triplet for position controller
	 */
	void		publish_mission_item_triplet();



	/**
	 * Compare two mission items if they are equivalent
	 * Two mission items can be considered equivalent for the purpose of the navigator even if some fields differ.
	 *
	 * @return true if equivalent, false otherwise
	 */
	bool		cmp_mission_item_equivalent(const struct mission_item_s a, const struct mission_item_s b);
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
	StateTable(&myTable[0][0], MAX_STATE, MAX_EVENT),

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

/* publications */
	_triplet_pub(-1),
	_fence_pub(-1),
	_mission_result_pub(-1),

/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "navigator")),

/* states */
	_fence_valid(false),
	_inside_fence(true),
	_current_offboard_mission_index(0),
	_current_onboard_mission_index(0),
	_offboard_mission_item_count(0),
	_onboard_mission_item_count(0),
	_waypoint_position_reached(false),
	_waypoint_yaw_reached(false),
	_time_first_inside_orbit(0)
{
	memset(&_fence, 0, sizeof(_fence));

	_parameter_handles.min_altitude = param_find("NAV_MIN_ALT");
	_parameter_handles.loiter_radius = param_find("NAV_LOITER_RAD");
	_parameter_handles.onboard_mission_enabled = param_find("NAV_ONB_MIS_EN");

	_mission_item_triplet.previous_valid = false;
	_mission_item_triplet.current_valid = false;
	_mission_item_triplet.next_valid = false;
	memset(&_mission_item_triplet.previous, 0, sizeof(struct mission_item_s));
	memset(&_mission_item_triplet.current, 0, sizeof(struct mission_item_s));
	memset(&_mission_item_triplet.next, 0, sizeof(struct mission_item_s));

	memset(&_mission_result, 0, sizeof(struct mission_result_s));

	/* Initialize state machine */
	myState = STATE_INIT;
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
	param_get(_parameter_handles.loiter_radius, &(_parameters.loiter_radius));
	param_get(_parameter_handles.onboard_mission_enabled, &(_parameters.onboard_mission_enabled));
}

void
Navigator::global_position_update()
{
	/* load local copies */
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
Navigator::offboard_mission_update()
{
	struct mission_s offboard_mission;
	if (orb_copy(ORB_ID(mission), _offboard_mission_sub, &offboard_mission) == OK) {

		_offboard_mission_item_count = offboard_mission.count;
		
		if (offboard_mission.current_index != -1) {
			_current_offboard_mission_index = offboard_mission.current_index;
		}

	} else {
		_offboard_mission_item_count = 0;
		_current_offboard_mission_index = 0;
	}
}

void
Navigator::onboard_mission_update()
{
	struct mission_s onboard_mission;
	if (orb_copy(ORB_ID(onboard_mission), _onboard_mission_sub, &onboard_mission) == OK) {

		_onboard_mission_item_count = onboard_mission.count;

		if (onboard_mission.current_index != -1) {
			_current_onboard_mission_index = onboard_mission.current_index;
		}

	} else {
		_onboard_mission_item_count = 0;
		_current_onboard_mission_index = 0;
	}
}

void
Navigator::vehicle_status_update()
{
	/* try to load initial states */
	if (orb_copy(ORB_ID(vehicle_status), _vstatus_sub, &_vstatus) != OK) {
		_vstatus.arming_state = ARMING_STATE_STANDBY; /* in case the commander is not be running */
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

	_fence_valid = load_fence(GEOFENCE_MAX_VERTICES);

	/*
	 * do subscriptions
	 */
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_offboard_mission_sub = orb_subscribe(ORB_ID(mission));
	_onboard_mission_sub = orb_subscribe(ORB_ID(onboard_mission));
	_capabilities_sub = orb_subscribe(ORB_ID(navigation_capabilities));
	_vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_home_pos_sub = orb_subscribe(ORB_ID(home_position));

	
	/* copy all topics first time */
	parameters_update();
	global_position_update();
	home_position_update();
	navigation_capabilities_update();
	offboard_mission_update();
	onboard_mission_update();
	vehicle_status_update();

	/* rate limit vehicle status updates to 5Hz */
	orb_set_interval(_vstatus_sub, 200);
	/* rate limit position updates to 50 Hz */
	orb_set_interval(_global_pos_sub, 20);

	/* wakeup source(s) */
	struct pollfd fds[7];

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

		/* only update vehicle status if it changed */
		if (fds[6].revents & POLLIN) {
			
			vehicle_status_update();

			/* Evaluate state machine from commander and set the navigator mode accordingly */
			if (_vstatus.main_state == MAIN_STATE_AUTO) {

				switch (_vstatus.navigation_state) {
					case NAVIGATION_STATE_DIRECT:
					case NAVIGATION_STATE_STABILIZE:
					case NAVIGATION_STATE_ALTHOLD:
					case NAVIGATION_STATE_VECTOR:
						
						/* don't publish a mission item triplet */
						dispatch(EVENT_NONE_REQUESTED);
						break;

					case NAVIGATION_STATE_AUTO_READY:
					case NAVIGATION_STATE_AUTO_TAKEOFF:
					case NAVIGATION_STATE_AUTO_MISSION:

						/* try mission if none is available, fallback to loiter instead */
						if (onboard_mission_available(0) || offboard_mission_available(0)) {
							dispatch(EVENT_MISSION_REQUESTED);
						} else {
							dispatch(EVENT_LOITER_REQUESTED);
						}
						break;

					case NAVIGATION_STATE_AUTO_LOITER:

						dispatch(EVENT_LOITER_REQUESTED);
						break;


					case NAVIGATION_STATE_AUTO_RTL:
						
						dispatch(EVENT_RTL_REQUESTED);
						break;

					case NAVIGATION_STATE_AUTO_LAND:

						/* TODO add this */
						
						break;

					default:
						warnx("ERROR: Navigation state not supported");
						break;
				}

			} else {
				/* not in AUTO */
				dispatch(EVENT_NONE_REQUESTED);
			}

			/* XXX Hack to get mavlink output going, try opening the fd with 5Hz */
			if (_mavlink_fd < 0) {
				/* try to open the mavlink log device every once in a while */
				_mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
			}
		}


		/* only update parameters if it changed */
		if (fds[0].revents & POLLIN) {

			parameters_update();
			/* note that these new parameters won't be in effect until a mission triplet is published again */
		}

		/* only update craft capabilities if they have changed */
		if (fds[3].revents & POLLIN) {
			navigation_capabilities_update();
		}

		if (fds[4].revents & POLLIN) {
			offboard_mission_update();
			// XXX check if mission really changed 
			dispatch(EVENT_MISSION_CHANGED);
		}

		if (fds[5].revents & POLLIN) {
			onboard_mission_update();
			// XXX check if mission really changed
			dispatch(EVENT_MISSION_CHANGED);
		}

		if (fds[2].revents & POLLIN) {
			home_position_update();
			// XXX check if home position really changed
			dispatch(EVENT_HOME_POSITION_CHANGED);
		}

		/* only run controller if position changed */
		if (fds[1].revents & POLLIN) {
			global_position_update();
			/* only check if waypoint has been reached in Mission or RTL mode */
			if (mission_item_reached()) {
				/* try to advance mission */
				if (advance_mission() != OK) {
					dispatch(EVENT_MISSION_FINISHED);
				}
			}
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
					 2048,
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
	warnx("Global position is %svalid", _global_pos.valid ? "" : "in");
	if (_global_pos.valid) {
		warnx("Longitude %5.5f degrees, latitude %5.5f degrees", _global_pos.lon / 1e7, _global_pos.lat / 1e7);
		warnx("Altitude %5.5f meters, altitude above home %5.5f meters",
			(double)_global_pos.alt, (double)_global_pos.relative_alt);
		warnx("Ground velocity in m/s, x %5.5f, y %5.5f, z %5.5f",
			(double)_global_pos.vx, (double)_global_pos.vy, (double)_global_pos.vz);
		warnx("Compass heading in degrees %5.5f", (double)(_global_pos.yaw * M_RAD_TO_DEG_F));
	}
	if (_fence_valid) {
		warnx("Geofence is valid");
		warnx("Vertex longitude latitude");
		for (unsigned i = 0; i < _fence.count; i++)
		warnx("%6u %9.5f %8.5f", i, (double)_fence.vertices[i].lon, (double)_fence.vertices[i].lat);
	} else {
		warnx("Geofence not set");
	}

	switch (myState) {
		case STATE_INIT:
			warnx("State: Init");
			break;
		case STATE_NONE:
			warnx("State: None");
			break;
		case STATE_LOITER:
			warnx("State: Loiter");
			break;
		case STATE_MISSION:
			warnx("State: Mission");
			switch (_mission_type) {
				case MISSION_TYPE_ONBOARD:
					warnx("Mission type: Onboard");
					break;
				case MISSION_TYPE_OFFBOARD:
					warnx("Mission type: Offboard");
					break;
				case MISSION_TYPE_NONE:
				default:
					warnx("ERROR: Mission type unsupported");
					break;
			}
			warnx("Onboard mission: %d of %d", _current_onboard_mission_index, _onboard_mission_item_count);
			warnx("Offboard mission: %d of %d", _current_offboard_mission_index, _offboard_mission_item_count);
			break;
		case STATE_MISSION_LOITER:
			warnx("State: Loiter after Mission");
			break;
		case STATE_RTL:
			warnx("State: RTL");
			break;
		case STATE_RTL_LOITER:
			warnx("State: Loiter after RTL");
			break;
		default:
			warnx("State: Unknown");
			break;
	}
}

void
Navigator::publish_fence(unsigned vertices)
{
	if (_fence_pub == -1)
		_fence_pub = orb_advertise(ORB_ID(fence), &vertices);
	else
		orb_publish(ORB_ID(fence), _fence_pub, &vertices);
}

bool
Navigator::fence_valid(const struct fence_s &fence)
{
	// NULL fence is valid
	if (fence.count == 0) {
		return true;
	}

	// Otherwise
	if ((fence.count < 4) || (fence.count > GEOFENCE_MAX_VERTICES)) {
		warnx("Fence must have at least 3 sides and not more than %d", GEOFENCE_MAX_VERTICES - 1);
		return false;
	}

	return true;
}

bool
Navigator::load_fence(unsigned vertices)
{
	struct fence_s temp_fence;

	unsigned i;
	for (i = 0; i < vertices; i++) {
		if (dm_read(DM_KEY_FENCE_POINTS, i, temp_fence.vertices + i, sizeof(struct fence_vertex_s)) != sizeof(struct fence_vertex_s)) {
			break;
		}
	}

	temp_fence.count = i;

	if (fence_valid(temp_fence))
		memcpy(&_fence, &temp_fence, sizeof(_fence));
	else
		warnx("Invalid fence file, ignored!");

	return _fence.count != 0;
}

void
Navigator::fence_point(int argc, char *argv[])
{
	int ix, last;
	double lon, lat;
	struct fence_vertex_s vertex;
	char *end;

	if ((argc == 1) && (strcmp("-clear", argv[0]) == 0)) {
		dm_clear(DM_KEY_FENCE_POINTS);
		publish_fence(0);
		return;
	}

	if (argc < 3)
		errx(1, "Specify: -clear | sequence latitude longitude [-publish]");

	ix = atoi(argv[0]);
	if (ix >= DM_KEY_FENCE_POINTS_MAX)
		errx(1, "Sequence must be less than %d", DM_KEY_FENCE_POINTS_MAX);

	lat = strtod(argv[1], &end);
	lon = strtod(argv[2], &end);

	last = 0;
	if ((argc > 3) && (strcmp(argv[3], "-publish") == 0))
		last = 1;

	vertex.lat = (float)lat;
	vertex.lon = (float)lon;

	if (dm_write(DM_KEY_FENCE_POINTS, ix, DM_PERSIST_POWER_ON_RESET, &vertex, sizeof(vertex)) == sizeof(vertex)) {
		if (last)
			publish_fence((unsigned)ix + 1);
		return;
	}

	errx(1, "can't store fence point");
}



StateTable::Tran const Navigator::myTable[MAX_STATE][MAX_EVENT] = {
	{	
							/* STATE_INIT */
		/* EVENT_NONE_REQUESTED */		{ACTION(&Navigator::start_none), STATE_NONE},
		/* EVENT_LOITER_REQUESTED */		{ACTION(&Navigator::start_loiter), STATE_LOITER},
		/* EVENT_MISSION_REQUESTED */		{ACTION(&Navigator::start_mission), STATE_MISSION},
		/* EVENT_RTL_REQUESTED */		{ACTION(&Navigator::start_rtl), STATE_RTL},
		/* EVENT_MISSION_FINISHED */		{NO_ACTION, STATE_INIT},
		/* EVENT_MISSION_CHANGED */		{NO_ACTION, STATE_INIT},
		/* EVENT_HOME_POSITION_CHANGED */	{NO_ACTION, STATE_INIT},
	},
	{	
							/* STATE_NONE */
		/* EVENT_NONE_REQUESTED */		{NO_ACTION, STATE_NONE},
		/* EVENT_LOITER_REQUESTED */		{ACTION(&Navigator::start_loiter), STATE_LOITER},
		/* EVENT_MISSION_REQUESTED */		{ACTION(&Navigator::start_mission), STATE_MISSION},
		/* EVENT_RTL_REQUESTED */		{ACTION(&Navigator::start_rtl), STATE_RTL},
		/* EVENT_MISSION_FINISHED */		{NO_ACTION, STATE_NONE},
		/* EVENT_MISSION_CHANGED */		{NO_ACTION, STATE_NONE},
		/* EVENT_HOME_POSITION_CHANGED */	{NO_ACTION, STATE_NONE},
	},
	{	
							/* STATE_LOITER */
		/* EVENT_NONE_REQUESTED */		{ACTION(&Navigator::start_none), STATE_NONE},
		/* EVENT_LOITER_REQUESTED */		{NO_ACTION, STATE_LOITER},
		/* EVENT_MISSION_REQUESTED */		{ACTION(&Navigator::start_mission), STATE_MISSION},
		/* EVENT_RTL_REQUESTED */		{ACTION(&Navigator::start_rtl), STATE_RTL},
		/* EVENT_MISSION_FINISHED */		{NO_ACTION, STATE_LOITER},
		/* EVENT_MISSION_CHANGED */		{NO_ACTION, STATE_LOITER},
		/* EVENT_HOME_POSITION_CHANGED */	{NO_ACTION, STATE_LOITER},
	},
	{	
							/* STATE_MISSION */
		/* EVENT_NONE_REQUESTED */		{ACTION(&Navigator::start_none), STATE_NONE},
		/* EVENT_LOITER_REQUESTED */		{ACTION(&Navigator::start_loiter), STATE_LOITER},
		/* EVENT_MISSION_REQUESTED */		{NO_ACTION, STATE_MISSION},
		/* EVENT_RTL_REQUESTED */		{ACTION(&Navigator::start_rtl), STATE_RTL},
		/* EVENT_MISSION_FINISHED */		{ACTION(&Navigator::start_mission_loiter), STATE_MISSION_LOITER},
		/* EVENT_MISSION_CHANGED */		{ACTION(&Navigator::start_mission), STATE_MISSION},
		/* EVENT_HOME_POSITION_CHANGED */	{NO_ACTION, STATE_MISSION},
	},
	{	
							/* STATE_MISSION_LOITER */
		/* EVENT_NONE_REQUESTED */		{ACTION(&Navigator::start_none), STATE_NONE},
		/* EVENT_LOITER_REQUESTED */		{NO_ACTION, STATE_MISSION_LOITER},
		/* EVENT_MISSION_REQUESTED */		{NO_ACTION, STATE_MISSION_LOITER},
		/* EVENT_RTL_REQUESTED */		{ACTION(&Navigator::start_rtl), STATE_RTL},
		/* EVENT_MISSION_FINISHED */		{NO_ACTION, STATE_MISSION_LOITER},
		/* EVENT_MISSION_CHANGED */		{ACTION(&Navigator::start_mission), STATE_MISSION},
		/* EVENT_HOME_POSITION_CHANGED */	{NO_ACTION, STATE_MISSION_LOITER},
	},
	{	
							/* STATE_RTL */
		/* EVENT_NONE_REQUESTED */		{ACTION(&Navigator::start_none), STATE_NONE},
		/* EVENT_LOITER_REQUESTED */		{ACTION(&Navigator::start_loiter), STATE_LOITER},
		/* EVENT_MISSION_REQUESTED */		{ACTION(&Navigator::start_mission), STATE_MISSION},
		/* EVENT_RTL_REQUESTED */		{NO_ACTION, STATE_RTL},
		/* EVENT_MISSION_FINISHED */		{ACTION(&Navigator::start_rtl_loiter), STATE_RTL_LOITER},
		/* EVENT_MISSION_CHANGED */		{NO_ACTION, STATE_RTL},
		/* EVENT_HOME_POSITION_CHANGED */	{ACTION(&Navigator::start_loiter), STATE_RTL},
	},
	{	
							/* STATE_RTL_LOITER */
		/* EVENT_NONE_REQUESTED */		{ACTION(&Navigator::start_none), STATE_NONE},
		/* EVENT_LOITER_REQUESTED */		{NO_ACTION,  STATE_RTL_LOITER},
		/* EVENT_MISSION_REQUESTED */		{ACTION(&Navigator::start_mission), STATE_MISSION},
		/* EVENT_RTL_REQUESTED */		{NO_ACTION, STATE_RTL_LOITER},
		/* EVENT_MISSION_FINISHED */		{NO_ACTION, STATE_RTL_LOITER},
		/* EVENT_MISSION_CHANGED */		{NO_ACTION, STATE_RTL_LOITER},
		/* EVENT_HOME_POSITION_CHANGED */	{ACTION(&Navigator::start_loiter), STATE_LOITER},
	},
};

void
Navigator::start_none()
{
	_mission_item_triplet.previous_valid = false;
	_mission_item_triplet.current_valid = false;
	_mission_item_triplet.next_valid = false;

	publish_mission_item_triplet();
}

void
Navigator::start_loiter()
{
	struct mission_item_s loiter_item;

	loiter_item.lat = (double)_global_pos.lat / 1e7;
	loiter_item.lon = (double)_global_pos.lon / 1e7;
	loiter_item.yaw = 0.0f;

	/* XXX get rid of ugly conversion for home position altitude */
	float global_min_alt = _parameters.min_altitude + (float)_home_pos.alt/1e3f;

	/* Use current altitude if above min altitude set by parameter */
	if (_global_pos.alt < global_min_alt) {
		loiter_item.altitude = global_min_alt;
		mavlink_log_info(_mavlink_fd, "[navigator] loiter %.1fm higher", (double)(global_min_alt - _global_pos.alt));
	} else {
		loiter_item.altitude = _global_pos.alt;
		mavlink_log_info(_mavlink_fd, "[navigator] loiter here");
	}

	set_loiter_item(&loiter_item);

	publish_mission_item_triplet();
}


void
Navigator::start_mission()
{
	/* leave previous mission item as isas is */

	if(set_mission_item(0, &_mission_item_triplet.current) == OK) {
		_mission_item_triplet.current_valid = true;
	} else {
		_mission_item_triplet.current_valid = false;
		warnx("ERROR: current WP can't be set");
	}

	if(set_mission_item(1, &_mission_item_triplet.next) == OK) {
		_mission_item_triplet.next_valid = true;
	} else {
		_mission_item_triplet.next_valid = false;
	}

	switch (_mission_type) {
		case MISSION_TYPE_ONBOARD:
			mavlink_log_info(_mavlink_fd, "[navigator] heading to onboard WP %d", _current_onboard_mission_index);
			break;
		case MISSION_TYPE_OFFBOARD:
			mavlink_log_info(_mavlink_fd, "[navigator] heading to offboard WP %d", _current_offboard_mission_index);
			break;
		case MISSION_TYPE_NONE:
		default:
			warnx("ERROR: Mission type unsupported");
			break;
	}

	publish_mission_item_triplet();
}



int
Navigator::advance_mission()
{
	switch (_mission_type) {
		case MISSION_TYPE_ONBOARD:
			warnx("advance onboard before: %d", _current_onboard_mission_index);
			_current_onboard_mission_index++;
			warnx("advance onboard after: %d", _current_onboard_mission_index);
			break;
		case MISSION_TYPE_OFFBOARD:
			warnx("advance offboard before: %d", _current_offboard_mission_index);
			_current_offboard_mission_index++;
			warnx("advance offboard after: %d", _current_offboard_mission_index);
			break;
		case MISSION_TYPE_NONE:
		default:
			warnx("ERROR: Mission type unsupported");
			return ERROR;
	}

	/* if there is no more mission available, don't advance and switch to loiter at current WP */
	if (!_mission_item_triplet.next_valid) {
		warnx("no next valid");
		return ERROR;
	}

	/* copy current mission to previous item */
	memcpy(&_mission_item_triplet.previous, &_mission_item_triplet.current, sizeof(mission_item_s));
	_mission_item_triplet.previous_valid = _mission_item_triplet.current_valid;

	if(set_mission_item(0, &_mission_item_triplet.current) == OK) {
		_mission_item_triplet.current_valid = true;
		
	} else {
		/* should never ever happen */
		_mission_item_triplet.current_valid = false;
		warnx("no current available");
		return ERROR;
	}
	
	if(set_mission_item(1, &_mission_item_triplet.next) == OK) {
		_mission_item_triplet.next_valid = true;
		
	} else {
		_mission_item_triplet.next_valid = false;
	}

	publish_mission_item_triplet();
	return OK;
}

void
Navigator::start_mission_loiter()
{
	/* make sure the current WP is valid */
	if (!_mission_item_triplet.current_valid) {
		warnx("ERROR: cannot switch to offboard mission loiter");
		return;
	}

	set_loiter_item(&_mission_item_triplet.current);

	switch (_mission_type) {
		case MISSION_TYPE_ONBOARD:
			mavlink_log_info(_mavlink_fd, "[navigator] loiter at onboard WP %d", _current_onboard_mission_index-1);
			break;
		case MISSION_TYPE_OFFBOARD:
			mavlink_log_info(_mavlink_fd, "[navigator] loiter at offboard WP %d", _current_offboard_mission_index-1);
			break;
		case MISSION_TYPE_NONE:
		default:
			warnx("ERROR: Mission type unsupported");
			break;
	}
}

void
Navigator::start_rtl()
{

	/* discard all mission item and insert RTL item */
	_mission_item_triplet.previous_valid = false;
	_mission_item_triplet.current_valid = true;
	_mission_item_triplet.next_valid = false;

	_mission_item_triplet.current.lat = (double)_home_pos.lat / 1e7;
	_mission_item_triplet.current.lon = (double)_home_pos.lon / 1e7;
	_mission_item_triplet.current.altitude = (float)_home_pos.alt / 1e3f + _parameters.min_altitude;
	_mission_item_triplet.current.yaw = 0.0f;
	_mission_item_triplet.current.nav_cmd = NAV_CMD_RETURN_TO_LAUNCH;
	_mission_item_triplet.current.loiter_direction = 1;
	_mission_item_triplet.current.loiter_radius = _parameters.loiter_radius; // TODO: get rid of magic number
	_mission_item_triplet.current.radius = 50.0f; // TODO: get rid of magic number
	_mission_item_triplet.current.autocontinue = false;
	_mission_item_triplet.current_valid = true;

	publish_mission_item_triplet();

	mavlink_log_info(_mavlink_fd, "[navigator] return to launch");
}	


void
Navigator::start_rtl_loiter()
{
	mission_item_s home_position_mission_item;
	home_position_mission_item.lat = (double)_home_pos.lat / 1e7;
	home_position_mission_item.lon = (double)_home_pos.lon / 1e7;
	home_position_mission_item.altitude = _home_pos.alt / 1e3f + _parameters.min_altitude;
	
	set_loiter_item(&home_position_mission_item);

	mavlink_log_info(_mavlink_fd, "[navigator] loiter after RTL");
}

bool
Navigator::offboard_mission_available(unsigned relative_index)
{
	return _offboard_mission_item_count > _current_offboard_mission_index + relative_index;
}

bool
Navigator::onboard_mission_available(unsigned relative_index)
{
	return _onboard_mission_item_count > _current_onboard_mission_index + relative_index && _parameters.onboard_mission_enabled;
}

int
Navigator::set_mission_item(unsigned relative_index, struct mission_item_s *mission_item)
{
	struct mission_item_s new_mission_item;

	/* try onboard mission first */
	if (onboard_mission_available(relative_index)) {
		if (_mission_type != MISSION_TYPE_ONBOARD && relative_index == 1) {
			relative_index--;
		}
		if (dm_read(DM_KEY_WAYPOINTS_ONBOARD, _current_onboard_mission_index + relative_index, &new_mission_item, sizeof(struct mission_item_s)) != sizeof(struct mission_item_s)) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			_mission_type = MISSION_TYPE_NONE;
			return ERROR;
		}
		/* base the mission type on the current mission item, not on future ones */
		if (relative_index == 0) {
			_mission_type = MISSION_TYPE_ONBOARD;
		}
	/* otherwise fallback to offboard */
	} else if (offboard_mission_available(relative_index)) {

		warnx("fallback try offboard: %d / %d", _current_offboard_mission_index + relative_index, _offboard_mission_item_count);

		if (_mission_type != MISSION_TYPE_OFFBOARD && relative_index == 1) {
			relative_index--;
		}

		if (dm_read(DM_KEY_WAYPOINTS_OFFBOARD, _current_offboard_mission_index + relative_index, &new_mission_item, sizeof(struct mission_item_s)) != sizeof(struct mission_item_s)) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			_mission_type = MISSION_TYPE_NONE;
			warnx("failed");
			return ERROR;
		}
		/* base the mission type on the current mission item, not on future ones */
		if (relative_index == 0) {
			_mission_type = MISSION_TYPE_OFFBOARD;
		}
	} else {
		/* happens when no more mission items can be added as a next item */
		return ERROR;
	}

	if (new_mission_item.nav_cmd == NAV_CMD_RETURN_TO_LAUNCH) {
		/* if it is a RTL waypoint, append the home position */
		new_mission_item.lat = (double)_home_pos.lat / 1e7;
		new_mission_item.lon = (double)_home_pos.lon / 1e7;
		new_mission_item.altitude = (float)_home_pos.alt / 1e3f + _parameters.min_altitude;
		new_mission_item.loiter_radius = _parameters.loiter_radius; // TODO: get rid of magic number
		new_mission_item.radius = 50.0f; // TODO: get rid of magic number
	}

	memcpy(mission_item, &new_mission_item, sizeof(struct mission_item_s));
	
	return OK;
}

bool
Navigator::mission_item_reached()
{
	/* only check if there is actually a mission item to check */
	if (!_mission_item_triplet.current_valid) {
		return false;
	}

	/* don't try to reach the landing mission, just stay in that mode, XXX maybe add another state for this */
	if (_mission_item_triplet.current.nav_cmd == NAV_CMD_LAND) {
		return false;
	}

	/* XXX TODO count turns */
	if ((_mission_item_triplet.current.nav_cmd == NAV_CMD_LOITER_TURN_COUNT ||
	     _mission_item_triplet.current.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
	     _mission_item_triplet.current.nav_cmd == NAV_CMD_LOITER_UNLIMITED) &&
	    _mission_item_triplet.current.loiter_radius > 0.01f) {

		return false;
	}	

	uint64_t now = hrt_absolute_time();
	float orbit;

	if (_mission_item_triplet.current.nav_cmd == NAV_CMD_WAYPOINT && _mission_item_triplet.current.radius > 0.01f) {

		orbit = _mission_item_triplet.current.radius;

	} else {

		// XXX set default orbit via param
		orbit = 15.0f;
	}

	/* keep vertical orbit */
	float vertical_switch_distance = orbit;


	// TODO add frame
	// int coordinate_frame = wpm->waypoints[wpm->current_active_wp_id].frame;
	
	float dist = -1.0f;
	float dist_xy = -1.0f;
	float dist_z = -1.0f;

	// if (coordinate_frame == (int)MAV_FRAME_GLOBAL) {
	dist = get_distance_to_point_global_wgs84(_mission_item_triplet.current.lat, _mission_item_triplet.current.lon, _mission_item_triplet.current.altitude,
		                              (double)_global_pos.lat / 1e7, (double)_global_pos.lon / 1e7, _global_pos.alt,
		                              &dist_xy, &dist_z);

	// warnx("1 lat: %2.2f, lon: %2.2f, alt: %2.2f", _mission_item_triplet.current.lat, _mission_item_triplet.current.lon, _mission_item_triplet.current.altitude);
	// warnx("2 lat: %2.2f, lon: %2.2f, alt: %2.2f", (double)_global_pos.lat / 1e7, (double)_global_pos.lon / 1e7, _global_pos.alt);

	// warnx("Dist: %4.4f", dist);

	// } else if (coordinate_frame == (int)MAV_FRAME_GLOBAL_RELATIVE_ALT) {
	// 	dist = mavlink_wpm_distance_to_point_global_wgs84(wpm->current_active_wp_id, (float)global_pos->lat * 1e-7f, (float)global_pos->lon * 1e-7f, global_pos->relative_alt, &dist_xy, &dist_z);

	// } else if (coordinate_frame == (int)MAV_FRAME_LOCAL_ENU || coordinate_frame == (int)MAV_FRAME_LOCAL_NED) {
	// 	dist = mavlink_wpm_distance_to_point_local(wpm->current_active_wp_id, local_pos->x, local_pos->y, local_pos->z, &dist_xy, &dist_z);

	// } else if (coordinate_frame == (int)MAV_FRAME_MISSION) {
	// 	/* Check if conditions of mission item are satisfied */
	// 	// XXX TODO
	// }

	if (dist >= 0.0f && dist_xy <= orbit && dist_z >= 0.0f && dist_z <= vertical_switch_distance) {
		_waypoint_position_reached = true;
	}

	/* check if required yaw reached */
	float yaw_sp = _wrap_pi(_mission_item_triplet.current.yaw);
	float yaw_err = _wrap_pi(yaw_sp - _global_pos.yaw);

	if (fabsf(yaw_err) < 0.05f) { /* XXX get rid of magic number */
		_waypoint_yaw_reached = true;
	}

	/* check if the current waypoint was reached */
	if (_waypoint_position_reached /* && _waypoint_yaw_reached */) { /* XXX what about yaw? */
		
		if (_time_first_inside_orbit == 0) {
			/* XXX announcment? */
			_time_first_inside_orbit = now;
		}
		
		/* check if the MAV was long enough inside the waypoint orbit */
		if ((now - _time_first_inside_orbit >= (uint64_t)_mission_item_triplet.current.time_inside * 1e6)
		     || _mission_item_triplet.current.nav_cmd == NAV_CMD_TAKEOFF) {

			_time_first_inside_orbit = 0;
			_waypoint_yaw_reached = false;
			_waypoint_position_reached = false;
			return true;
		}
	}
	return false;

}

void
Navigator::set_loiter_item(struct mission_item_s *new_loiter_position)
{
	_mission_item_triplet.previous_valid = false;
	_mission_item_triplet.current_valid = true;
	_mission_item_triplet.next_valid = false;

	_mission_item_triplet.current.nav_cmd = NAV_CMD_LOITER_UNLIMITED;
	_mission_item_triplet.current.loiter_direction = 1;
	_mission_item_triplet.current.loiter_radius = _parameters.loiter_radius; // TODO: get rid of magic number
	_mission_item_triplet.current.radius = 50.0f; // TODO: get rid of magic number
	_mission_item_triplet.current.autocontinue = false;

	_mission_item_triplet.current.lat = new_loiter_position->lat;
	_mission_item_triplet.current.lon = new_loiter_position->lon;
	_mission_item_triplet.current.altitude = new_loiter_position->altitude;
	_mission_item_triplet.current.yaw = new_loiter_position->yaw;

	publish_mission_item_triplet();
}

void
Navigator::publish_mission_item_triplet()
{
	/* lazily publish the mission triplet only once available */
	if (_triplet_pub > 0) {
		/* publish the mission triplet */
		orb_publish(ORB_ID(mission_item_triplet), _triplet_pub, &_mission_item_triplet);

	} else {
		/* advertise and publish */
		_triplet_pub = orb_advertise(ORB_ID(mission_item_triplet), &_mission_item_triplet);
	}
}


bool Navigator::cmp_mission_item_equivalent(const struct mission_item_s a, const struct mission_item_s b) {
	if (fabsf(a.altitude_is_relative - b.altitude_is_relative) < FLT_EPSILON &&
	    fabsf(a.lat - b.lat) < FLT_EPSILON &&
	    fabsf(a.lon - b.lon) < FLT_EPSILON &&
	    fabsf(a.altitude - b.altitude) < FLT_EPSILON &&
	    fabsf(a.yaw - b.yaw) < FLT_EPSILON &&
	    fabsf(a.loiter_radius - b.loiter_radius) < FLT_EPSILON &&
	    fabsf(a.loiter_direction - b.loiter_direction) < FLT_EPSILON &&
	    fabsf(a.nav_cmd - b.nav_cmd) < FLT_EPSILON &&
	    fabsf(a.radius - b.radius) < FLT_EPSILON &&
	    fabsf(a.time_inside - b.time_inside) < FLT_EPSILON &&
	    fabsf(a.autocontinue - b.autocontinue) < FLT_EPSILON &&
	    fabsf(a.index - b.index) < FLT_EPSILON) {
		return true;
	} else {
		warnx("a.index %d, b.index %d", a.index, b.index);
		return false;
	}
}


static void usage()
{
	errx(1, "usage: navigator {start|stop|status|fence}");
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
		navigator::g_navigator->fence_point(argc - 2, argv + 2);

	} else {
		usage();
	}

	return 0;
}

