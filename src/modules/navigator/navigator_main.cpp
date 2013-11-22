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
#include <uORB/topics/mission_item_triplet.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/fence.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <dataman/dataman.h>


/**
 * navigator app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int navigator_main(int argc, char *argv[]);

class Navigator
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

	int		_global_pos_sub;
	int		_vstatus_sub;			/**< vehicle status subscription */
	int 		_params_sub;			/**< notification of parameter updates */
	int		_mission_sub;			/**< notification of mission updates */
	int		_capabilities_sub;		/**< notification of vehicle capabilities updates */

	orb_advert_t	_triplet_pub;			/**< publish position setpoint triplet */
	orb_advert_t	_fence_pub;			/**< publish fence topic */ 

	struct vehicle_status_s				_vstatus;		/**< vehicle status */
	struct vehicle_global_position_s		_global_pos;		/**< global vehicle position */
	struct mission_item_triplet_s			_mission_item_triplet;	/**< triplet of mission items */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	unsigned	_max_mission_item_count;	/**< maximum number of mission items supported */
	
	unsigned	_mission_item_count;		/** number of mission items copied */
	struct mission_item_s				*_mission_item;	/**< storage for mission */

	struct	fence_s 				_fence;			/**< storage for fence vertices */
	bool						_fence_valid;		/**< flag if fence is valid */
        bool						_inside_fence;		/**< vehicle is inside fence */

	struct navigation_capabilities_s		_nav_caps;

	bool		_waypoint_position_reached;
	bool		_waypoint_yaw_reached;
	uint64_t	_time_first_inside_orbit;

	/** manual control states */
	float		_seatbelt_hold_heading;		/**< heading the system should hold in seatbelt mode */
	float		_loiter_hold_lat;
	float		_loiter_hold_lon;
	float		_loiter_hold_alt;
	bool		_loiter_hold;

	struct {
		float throttle_cruise;
	}		_parameters;			/**< local copies of interesting parameters */

	struct {
		param_t throttle_cruise;

	}		_parameter_handles;		/**< handles for interesting parameters */


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Update control outputs
	 *
	 */
	void		control_update();

	/**
	* Retrieve mission.
	*/
	void		mission_update();

	/**
	 * Control throttle.
	 */
	float		control_throttle(float energy_error);

	/**
	 * Control pitch.
	 */
	float		control_pitch(float altitude_error);

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

	void		add_mission_item(unsigned mission_item_index, 
			                 const struct mission_item_s *existing_mission_item,
			                 struct mission_item_s *new_mission_item);

	void		update_mission_item_triplet();

	void		advance_current_mission_item();

	void		restart_mission();

	void		reset_mission_item_reached();

	bool		check_mission_item_reached();
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

	_task_should_exit(false),
	_navigator_task(-1),

/* subscriptions */
	_global_pos_sub(-1),
	_vstatus_sub(-1),
	_params_sub(-1),
	_mission_sub(-1),
	_capabilities_sub(-1),

/* publications */
	_triplet_pub(-1),
	_fence_pub(-1),

/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "navigator")),
/* states */
	_max_mission_item_count(10),
	_fence_valid(false),
	_inside_fence(true),
	_waypoint_position_reached(false),
	_waypoint_yaw_reached(false),
	_time_first_inside_orbit(0),
	_loiter_hold(false)
{
	_global_pos.valid = false;
	memset(&_fence, 0, sizeof(_fence));
	_parameter_handles.throttle_cruise = param_find("NAV_DUMMY");

	_mission_item = (mission_item_s*)malloc(sizeof(mission_item_s) * _max_mission_item_count);

	/* fetch initial values */
	parameters_update();
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

int
Navigator::parameters_update()
{

	//param_get(_parameter_handles.throttle_cruise, &(_parameters.throttle_cruise));

	return OK;
}

void
Navigator::mission_update()
{
	struct mission_s mission;
	if (orb_copy(ORB_ID(mission), _mission_sub, &mission) == OK) {
		// XXX this is not optimal yet, but a first prototype /
		// test implementation

		if (mission.count <= _max_mission_item_count) {
			/*
			 * Perform an atomic copy & state update
			 */
			irqstate_t flags = irqsave();

			memcpy(_mission_item, mission.items, mission.count * sizeof(struct mission_item_s));
			_mission_item_count = mission.count;

			irqrestore(flags);

			/* start new mission at beginning */
			restart_mission();
		} else {
			warnx("ERROR: too many waypoints, not supported");
		}

		/* Reset to 0 for now when a waypoint is changed */
		/* TODO add checks if and how the mission has changed */
		
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

	/*
	 * do subscriptions
	 */
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_mission_sub = orb_subscribe(ORB_ID(mission));
	_capabilities_sub = orb_subscribe(ORB_ID(navigation_capabilities));
	_vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));

	// Load initial states
	if (orb_copy(ORB_ID(vehicle_status), _vstatus_sub, &_vstatus) != OK) {
		_vstatus.arming_state = ARMING_STATE_STANDBY;    // for testing... commander may not be running
	}

	mission_update();

	/* rate limit vehicle status updates to 5Hz */
	orb_set_interval(_vstatus_sub, 200);
	/* rate limit position updates to 50 Hz */
	orb_set_interval(_global_pos_sub, 20);

	parameters_update();

	_fence_valid = load_fence(GEOFENCE_MAX_VERTICES);

	/* load the craft capabilities */
	orb_copy(ORB_ID(navigation_capabilities), _capabilities_sub, &_nav_caps);

	/* wakeup source(s) */
	struct pollfd fds[5];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _global_pos_sub;
	fds[1].events = POLLIN;
	fds[2].fd = _capabilities_sub;
	fds[2].events = POLLIN;
	fds[3].fd = _mission_sub;
	fds[3].events = POLLIN;
	fds[4].fd = _vstatus_sub;
	fds[4].events = POLLIN;

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

		/* only update parameters if they changed */
		if (fds[4].revents & POLLIN) {
			/* read from param to clear updated flag */
			orb_copy(ORB_ID(vehicle_status), _vstatus_sub, &_vstatus);
		}

		/* only update vehicle status if it changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* only update craft capabilities if they have changed */
		if (fds[2].revents & POLLIN) {
			orb_copy(ORB_ID(navigation_capabilities), _capabilities_sub, &_nav_caps);
		}

		if (fds[3].revents & POLLIN) {
			mission_update();
		}

		/* only run controller if position changed */
		if (fds[1].revents & POLLIN) {

			/* load local copies */
			orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);

			static uint64_t last_run = 0;
			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too large deltaT's */
			if (deltaT > 1.0f) {
				deltaT = 0.01f;
			}

			if (_fence_valid && _global_pos.valid) {
				_inside_fence = inside_geofence(&_global_pos, &_fence);
			}

			math::Vector2f ground_speed(_global_pos.vx, _global_pos.vy);
			/* Current waypoint */
			math::Vector2f next_wp(_mission_item_triplet.current.lat / 1e7f, _mission_item_triplet.current.lon / 1e7f);
			/* Global position */
			math::Vector2f current_position(_global_pos.lat / 1e7f, _global_pos.lon / 1e7f);

			/* Autonomous flight */
			if (1 /* autonomous flight */) {

				/* proceed to next waypoint if we reach it */
				if (check_mission_item_reached()) {
					advance_current_mission_item();
				}
			}

			/* lazily publish the setpoint only once available */
			if (_triplet_pub > 0) {
				/* publish the attitude setpoint */
				orb_publish(ORB_ID(mission_item_triplet), _triplet_pub, &_mission_item_triplet);

			} else {
				/* advertise and publish */
				_triplet_pub = orb_advertise(ORB_ID(mission_item_triplet), &_mission_item_triplet);
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
		warnx("Compass heading in degrees %5.5f", (double)_global_pos.yaw * 57.2957795);
	}
	if (_fence_valid) {
		warnx("Geofence is valid");
		warnx("Vertex longitude latitude");
		for (unsigned i = 0; i < _fence.count; i++)
		warnx("%6u %9.5f %8.5f", i, (double)_fence.vertices[i].lon, (double)_fence.vertices[i].lat);
	} else
		warnx("Geofence not set");
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

void
Navigator::add_mission_item(unsigned mission_item_index, const struct mission_item_s *existing_mission_item, struct mission_item_s *new_mission_item) {

	/* Check if there is a further mission as the new next item */
	while (mission_item_index < _mission_item_count) {

		if (1 /* TODO: check for correct frame */) {

			memcpy(new_mission_item, &_mission_item[mission_item_index], sizeof(mission_item_s));
			return;
		}
		mission_item_index++;
	}

	/* if no existing mission item exists, take curent location */
	if (existing_mission_item == nullptr) {
		
		new_mission_item->lat = (double)_global_pos.lat / 1e7;
		new_mission_item->lon = (double)_global_pos.lon / 1e7;
		new_mission_item->altitude = _global_pos.alt;
		new_mission_item->yaw = _global_pos.yaw;
		new_mission_item->nav_cmd = NAV_CMD_LOITER_UNLIMITED;
		new_mission_item->loiter_direction = 1;
		new_mission_item->loiter_radius = 100.0f; // TODO: get rid of magic number
		new_mission_item->radius = 10.0f; // TODO: get rid of magic number
		new_mission_item->autocontinue = false;

	} else {

		switch (existing_mission_item->nav_cmd) {

			/* if the last mission is not a loiter item, set it to one */
			case NAV_CMD_WAYPOINT:
			case NAV_CMD_RETURN_TO_LAUNCH:
			case NAV_CMD_TAKEOFF:

				/* copy current mission to next item */
				memcpy(new_mission_item, existing_mission_item, sizeof(mission_item_s));

				/* and set it to a loiter item */
				new_mission_item->nav_cmd = NAV_CMD_LOITER_UNLIMITED;
				/* also adapt the loiter_radius */
				new_mission_item->loiter_radius = 100.0f;
				//_mission_item_triplet.current.loiter_radius = _nav_caps.turn_distance; // TODO: publish capabilities somewhere
				new_mission_item->loiter_direction = 1;

				break;

			/* if the last mission item was to loiter, continue unlimited */
			case NAV_CMD_LOITER_TURN_COUNT:
			case NAV_CMD_LOITER_TIME_LIMIT:

				/* copy current mission to next item */
				memcpy(new_mission_item, existing_mission_item, sizeof(mission_item_s));
				/* and set it to loiter */
				new_mission_item->nav_cmd = NAV_CMD_LOITER_UNLIMITED;

				break;
			/* if already in loiter, don't change anything */
			case NAV_CMD_LOITER_UNLIMITED:
				break;
			/* if landed, stay in land mode */
			case NAV_CMD_LAND:
				break;

			default:
				warnx("Unsupported nav_cmd");
				break;
		}
	}
}

void
Navigator::update_mission_item_triplet()
{
	if (!_mission_item_triplet.current_valid) {
		
		/* the current mission item is missing, add one */
		if (_mission_item_triplet.previous_valid) {
			/* if we know the last one, proceed to succeeding one */
			add_mission_item(_mission_item_triplet.previous.index + 1, &_mission_item_triplet.previous, &_mission_item_triplet.current);
		}
		else {
			/* if we don't remember the last one, start new */
			add_mission_item(0, nullptr, &_mission_item_triplet.current);
		}
		_mission_item_triplet.current_valid = true;
	}

	if (_mission_item_triplet.current_valid && !_mission_item_triplet.next_valid) {
		
		if (_mission_item_triplet.current.nav_cmd == NAV_CMD_LOITER_UNLIMITED) {
			/* if we are already loitering, don't bother about a next mission item */

			_mission_item_triplet.next_valid = false;
		} else {

			add_mission_item(_mission_item_triplet.current.index + 1, &_mission_item_triplet.current, &_mission_item_triplet.next);
			_mission_item_triplet.next_valid = true;
		}
	}
}

void
Navigator::advance_current_mission_item()
{
	/* if there is no more mission available, don't advance and return */
	if (!_mission_item_triplet.next_valid) {
		return;
	}

	reset_mission_item_reached();

	/* copy current mission to previous item */
	memcpy(&_mission_item_triplet.previous, &_mission_item_triplet.current, sizeof(mission_item_s));
	_mission_item_triplet.previous_valid = _mission_item_triplet.current_valid;

	/* copy the next to current */
	memcpy(&_mission_item_triplet.current, &_mission_item_triplet.next, sizeof(mission_item_s));
	_mission_item_triplet.current_valid = _mission_item_triplet.next_valid;

	/* flag the next mission as invalid */
	_mission_item_triplet.next_valid = false;
	
	update_mission_item_triplet();
}

void
Navigator::restart_mission()
{
	reset_mission_item_reached();

	/* forget about the all mission items */
	_mission_item_triplet.previous_valid = false;
	_mission_item_triplet.current_valid = false;
	_mission_item_triplet.next_valid = false;

	update_mission_item_triplet();
}




void
Navigator::reset_mission_item_reached()
{
	/* reset all states */
	_waypoint_position_reached = false;
	_waypoint_yaw_reached = false;
	_time_first_inside_orbit = 0;
}

bool
Navigator::check_mission_item_reached()
{
	uint64_t now = hrt_absolute_time();
	float orbit;

	if (_mission_item_triplet.current.nav_cmd == NAV_CMD_WAYPOINT) {

		orbit = _mission_item_triplet.current.radius;

	} else if (_mission_item_triplet.current.nav_cmd == NAV_CMD_LOITER_TURN_COUNT ||
			_mission_item_triplet.current.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
			_mission_item_triplet.current.nav_cmd == NAV_CMD_LOITER_UNLIMITED) {

		orbit = _mission_item_triplet.current.loiter_radius;
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
		     || _mission_item_triplet.current.nav_cmd == (int)MAV_CMD_NAV_TAKEOFF) {

			return true;
		}
	}
	return false;

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
