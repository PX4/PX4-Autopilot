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
 * @file bottle_drop.cpp
 *
 * Bottle drop module for Outback Challenge 2014, Team Swiss Fang
 *
 * @author Dominik Juchli <juchlid@ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
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
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/parameter_update.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <geo/geo.h>
#include <dataman/dataman.h>
#include <mathlib/mathlib.h>
#include <mavlink/mavlink_log.h>


/**
 * bottle_drop app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int bottle_drop_main(int argc, char *argv[]);

class BottleDrop
{
public:
	/**
	 * Constructor
	 */
	BottleDrop();

	/**
	 * Destructor, also kills task.
	 */
	~BottleDrop();

	/**
	 * Start the task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	/**
	 * Display status.
	 */
	void		status();

	void		open_bay();
	void		close_bay();
	void		drop();

private:
	bool		_task_should_exit;		/**< if true, task should exit */
	int		_main_task;			/**< handle for task */
	int		_mavlink_fd;

	int		_command_sub;
	int		_wind_estimate_sub;
	struct vehicle_command_s	_command;
	struct vehicle_global_position_s _global_pos;
	map_projection_reference_s ref;

	orb_advert_t	_actuator_pub;
	struct actuator_controls_s _actuators;

	bool		_drop_approval;
	bool		_open_door;
	bool		_drop;
	hrt_abstime	_doors_opened;
	hrt_abstime	_drop_time;

	float		_alt_clearance;

	struct position_s {
		double lat; //degrees
		double lon; //degrees
		float alt; //m
		bool initialized;
	} _target_position, _drop_position;

	void		task_main();

	void		handle_command(struct vehicle_command_s *cmd);

	void		answer_command(struct vehicle_command_s *cmd, enum VEHICLE_CMD_RESULT result);

	/**
	 * Set the actuators
	 */
	int		actuators_publish();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);
};

namespace bottle_drop
{
BottleDrop	*g_bottle_drop;
}

BottleDrop::BottleDrop() :

	_task_should_exit(false),
	_main_task(-1),
	_mavlink_fd(-1),
	_command_sub(-1),
	_wind_estimate_sub(-1),
	_command {},
	_global_pos {},
	ref {},
	_actuator_pub(-1),
	_actuators {},
	_drop_approval(false),
	_open_door(false),
	_drop(false),
	_doors_opened(0),
	_drop_time(0),
	_alt_clearance(0)
{
}

BottleDrop::~BottleDrop()
{
	if (_main_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				task_delete(_main_task);
				break;
			}
		} while (_main_task != -1);
	}

	bottle_drop::g_bottle_drop = nullptr;
}

int
BottleDrop::start()
{
	ASSERT(_main_task == -1);

	/* start the task */
	_main_task = task_spawn_cmd("bottle_drop",
				    SCHED_DEFAULT,
				    SCHED_PRIORITY_MAX - 5,
				    2048,
				    (main_t)&BottleDrop::task_main_trampoline,
				    nullptr);

	if (_main_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}


void
BottleDrop::status()
{
	warnx("Doors: %s", _open_door ? "OPEN" : "CLOSED");
	warnx("Dropping: %s", _drop ? "YES" : "NO");
}

void
BottleDrop::open_bay()
{
	_actuators.control[0] = -1.0f;
	_actuators.control[1] = 1.0f;

	if (_doors_opened == 0) {
		_doors_opened = hrt_absolute_time();
	}
	warnx("open doors");

	actuators_publish();
}

void
BottleDrop::close_bay()
{
	// closed door and locked survival kit
	_actuators.control[0] = 1.0f;
	_actuators.control[1] = -1.0f;

	_doors_opened = 0;

	actuators_publish();
}

void
BottleDrop::drop()
{

	// update drop actuator, wait 0.5s until the doors are open before dropping
	hrt_abstime starttime = hrt_absolute_time();

	// force the door open if we have to
	if (_doors_opened == 0) {
		open_bay();
		warnx("bay not ready, forced open");
	}

	while (hrt_elapsed_time(&_doors_opened) < 400000 && hrt_elapsed_time(&starttime) < 2000000) {
		usleep(50000);
		warnx("delayed by door!");
	}

	if (_drop && _doors_opened != 0 && hrt_elapsed_time(&_doors_opened) < 500000) {
		_actuators.control[2] = -1.0f;
	} else {
		_actuators.control[2] = 1.0f;
	}

	_drop_time = hrt_absolute_time();

	warnx("dropping now");

	actuators_publish();
}

int
BottleDrop::actuators_publish()
{
	_actuators.timestamp = hrt_absolute_time();

	// lazily publish _actuators only once available
	if (_actuator_pub > 0) {
		return orb_publish(ORB_ID(actuator_controls_2), _actuator_pub, &_actuators);

	} else {
		_actuator_pub = orb_advertise(ORB_ID(actuator_controls_2), &_actuators);
		if (_actuator_pub > 0) {
			return OK;
		} else {
			return -1;
		}
	}
}

void
BottleDrop::task_main()
{

	_mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	mavlink_log_info(_mavlink_fd, "[bottle_drop] started");

	_command_sub = orb_subscribe(ORB_ID(vehicle_command));
	_wind_estimate_sub = orb_subscribe(ORB_ID(wind_estimate));

	bool updated = false;

	float z_0;		// ground properties
	float turn_radius;	// turn radius of the UAV
	float precision;	// Expected precision of the UAV

	// XXX we do not measure the exact ground altitude yet, but eventually we have to
	float ground_distance = 70.0f;

	// constant
	float g = CONSTANTS_ONE_G;               		// constant of gravity [m/s^2]
	float m = 0.5f;                		// mass of bottle [kg]
	float rho = 1.2f;              		// air density [kg/m^3]
	float A = ((0.063f * 0.063f) / 4.0f * M_PI_F); // Bottle cross section [m^2]
	float dt_freefall_prediction = 0.01f;   // step size of the free fall prediction [s]
	float dt_runs = 0.05f;			// step size 2 [s]

	// Has to be estimated by experiment
	float cd = 0.86f;              	// Drag coefficient for a cylinder with a d/l ratio of 1/3 []
	float t_signal =
		0.084f;	// Time span between sending the signal and the bottle top reaching level height with the bottom of the plane [s]
	float t_door =
		0.7f;		// The time the system needs to open the door + safety, is also the time the palyload needs to safely escape the shaft [s]


	// Definition
	float h_0;						// height over target
	float az;                 				// acceleration in z direction[m/s^2]
	float vz; 						// velocity in z direction [m/s]
	float z; 						// fallen distance [m]
	float h; 						// height over target [m]
	float ax;						// acceleration in x direction [m/s^2]
	float vx;						// ground speed in x direction [m/s]
	float x;					        // traveled distance in x direction [m]
	float vw;                 				// wind speed [m/s]
	float vrx;						// relative velocity in x direction [m/s]
	float v;						// relative speed vector [m/s]
	float Fd;						// Drag force [N]
	float Fdx;						// Drag force in x direction [N]
	float Fdz;						// Drag force in z direction [N]
	float x_drop, y_drop;					// coordinates of the drop point in reference to the target (projection of NED)
	float x_t, y_t;						// coordinates of the target in reference to the target x_t = 0, y_t = 0 (projection of NED)
	float x_l, y_l;						// local position in projected coordinates
	float x_f, y_f;						// to-be position of the UAV after dt_runs seconds in projected coordinates
	double x_f_NED, y_f_NED;				// to-be position of the UAV after dt_runs seconds in NED
	float distance_open_door;				// The distance the UAV travels during its doors open [m]
	float distance_real = 0;				// The distance between the UAVs position and the drop point [m]
	float future_distance = 0;				// The distance between the UAVs to-be position and the drop point [m]

	// states
	bool state_drop = false;				// Drop occurred = true, Drop din't occur = false
	bool state_run = false;					// A drop was attempted = true, the drop is still in progress = false

	unsigned counter = 0;

	param_t param_gproperties = param_find("BD_GPROPERTIES");
	param_t param_turn_radius = param_find("BD_TURNRADIUS");
	param_t param_precision = param_find("BD_PRECISION");


	param_get(param_precision, &precision);
	param_get(param_turn_radius, &turn_radius);
	param_get(param_gproperties, &z_0);

	int vehicle_global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));

	struct parameter_update_s update;
	memset(&update, 0, sizeof(update));
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));

	struct mission_item_s flight_vector_s;
	struct mission_item_s flight_vector_e;

	flight_vector_s.nav_cmd = NAV_CMD_WAYPOINT;
	flight_vector_s.acceptance_radius = 50; // TODO: make parameter
	flight_vector_s.autocontinue = true;

	flight_vector_e.nav_cmd = NAV_CMD_WAYPOINT;
	flight_vector_e.acceptance_radius = 50; // TODO: make parameter
	flight_vector_e.autocontinue = true;

	struct mission_s onboard_mission;
	memset(&onboard_mission, 0, sizeof(onboard_mission));
	orb_advert_t onboard_mission_pub = -1;

	double latitude;
	double longitude;

	struct wind_estimate_s wind;

	/* wakeup source(s) */
	struct pollfd fds[1];

	/* Setup of loop */
	fds[0].fd = _command_sub;
	fds[0].events = POLLIN;

	// Whatever state the bay is in, we want it closed on startup
	close_bay();

	while (!_task_should_exit) {

		/* wait for up to 100ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 50);

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		/* vehicle commands updated */
		if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(vehicle_command), _command_sub, &_command);
			handle_command(&_command);
		}

		orb_check(vehicle_global_position_sub, &updated);
		if (updated) {
			/* copy global position */
			orb_copy(ORB_ID(vehicle_global_position), vehicle_global_position_sub, &_global_pos);

			latitude = (double)_global_pos.lat / 1e7;
			longitude = (double)_global_pos.lon / 1e7;
		}

		if (_global_pos.timestamp == 0) {
			continue;
		}

		const unsigned sleeptime_us = 10000;

		hrt_abstime last_run = hrt_absolute_time();
		dt_runs = 1e6f / sleeptime_us;

		/* switch to faster updates during the drop */
		while (_drop) {

			/* 20s after drop, reset and close everything again */
			if (_drop && _doors_opened != 0 && hrt_elapsed_time(&_doors_opened) > 20000000) {
				_open_door = false;
				_drop = false;
			}

			orb_check(_wind_estimate_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(wind_estimate), _wind_estimate_sub, &wind);
			}

			orb_check(vehicle_global_position_sub, &updated);

			if (updated) {
				/* copy global position */
				orb_copy(ORB_ID(vehicle_global_position), vehicle_global_position_sub, &_global_pos);

				latitude = (double)_global_pos.lat / 1e7;
				longitude = (double)_global_pos.lon / 1e7;
			}

			orb_check(parameter_update_sub, &updated);

			if (updated) {
				/* copy global position */
				orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);

				/* update all parameters */
				param_get(param_gproperties, &z_0);
				param_get(param_turn_radius, &turn_radius);
				param_get(param_precision, &precision);
			}

			// Initialization

			float windspeed_norm = sqrtf(wind.windspeed_north * wind.windspeed_north + wind.windspeed_east * wind.windspeed_east);
			float groundspeed_body = sqrtf(_global_pos.vel_n * _global_pos.vel_n + _global_pos.vel_e * _global_pos.vel_e);

			distance_open_door = fabsf(t_door * groundspeed_body);


			//warnx("absolut wind speed = %.4f", vr); //////////////////////////////////////////////////////////////////// DEBUGGING


			//warnx("Initialization complete\n"); //////////////////////////////////////////////////////////////////// DEBUGGING


			if (_drop_approval && !state_drop) {
				//warnx("approval given\n"); //////////////////////////////////////////////////////////////////// DEBUGGING
				// drop here
				//open_now = true;
				//drop = false;
				//drop_start = hrt_absolute_time();

				// Update drop point at 10 Hz
				if (counter % 10 == 0) {

					az = g;                 				// acceleration in z direction[m/s^2]
					vz = 0; 						// velocity in z direction [m/s]
					z = 0; 							// fallen distance [m]
					h_0 = _global_pos.alt - _target_position.alt; 		// height over target at start[m]
					h = h_0;						// height over target [m]
					ax = 0;							// acceleration in x direction [m/s^2]
					vx = groundspeed_body;// XXX project					// ground speed in x direction [m/s]
					x = 0;							// traveled distance in x direction [m]
					vw = 0;                 				// wind speed [m/s]
					vrx = 0;						// relative velocity in x direction [m/s]
					v = groundspeed_body;					// relative speed vector [m/s]
					Fd = 0;							// Drag force [N]
					Fdx = 0;						// Drag force in x direction [N]
					Fdz = 0;						// Drag force in z direction [N]


					// Compute the distance the bottle will travel after it is dropped in body frame coordinates --> x
					while (h > 0.05f) {
						// z-direction
						vz = vz + az * dt_freefall_prediction;
						z = z + vz * dt_freefall_prediction;
						h = h_0 - z;

						// x-direction
						vw = windspeed_norm * logf(h / z_0) / logf(ground_distance / z_0);
						vx = vx + ax * dt_freefall_prediction;
						x = x + vx * dt_freefall_prediction;
						vrx = vx + vw;

						//Drag force
						v = sqrtf(vz * vz + vrx * vrx);
						Fd = 0.5f * rho * A * cd * powf(v, 2.0f);
						Fdx = Fd * vrx / v;
						Fdz = Fd * vz / v;

						//acceleration
						az = g - Fdz / m;
						ax = -Fdx / m;
					}

					// Compute drop vector
					x = groundspeed_body * t_signal + x;
				}

				x_t = 0.0f;
				y_t = 0.0f;

				float wind_direction_n, wind_direction_e;

				if (windspeed_norm < 0.5f) {	// If there is no wind, an arbitrarily direction is chosen
					wind_direction_n = 1.0f;
					wind_direction_e = 0.0f;

				} else {
					wind_direction_n = wind.windspeed_north / windspeed_norm;
					wind_direction_e = wind.windspeed_east / windspeed_norm;
				}

				x_drop = x_t + x * wind_direction_n;
				y_drop = y_t + x * wind_direction_e;
				map_projection_reproject(&ref, x_drop, y_drop, &_drop_position.lat, &_drop_position.lon);
				//warnx("drop point: lat = %.7f , lon = %.7f", drop_position.lat, drop_position.lon); //////////////////////////////////////////////////////////////////// DEBUGGING



				// Compute flight vector
				map_projection_reproject(&ref, x_drop + 2 * turn_radius * wind_direction_n, y_drop + 2 * turn_radius * wind_direction_n,
							 &(flight_vector_s.lat), &(flight_vector_s.lon));
				flight_vector_s.altitude = _drop_position.alt + _alt_clearance;
				map_projection_reproject(&ref, x_drop - turn_radius * wind_direction_e, y_drop - turn_radius * wind_direction_e,
							 &flight_vector_e.lat, &flight_vector_e.lon);
				flight_vector_e.altitude = _drop_position.alt + _alt_clearance;
				//warnx("Flight vector: starting point = %.7f  %.7f , end point = %.7f  %.7f", flight_vector_s.lat,flight_vector_s.lon,flight_vector_e.lat,flight_vector_e.lon); //////////////////////////////////////////////////////////////////// DEBUGGING

				// Drop Cancellation if terms are not met

				// warnx("latitude:%.2f", latitude);
				// warnx("longitude:%.2f", longitude);
				// warnx("drop_position.lat:%.2f", drop_position.lat);
				// warnx("drop_position.lon:%.2f", drop_position.lon);

				distance_real = fabsf(get_distance_to_next_waypoint(latitude, longitude, _drop_position.lat, _drop_position.lon));
				map_projection_project(&ref, latitude, longitude, &x_l, &y_l);
				x_f = x_l + _global_pos.vel_n * dt_runs;
				y_f = y_l + _global_pos.vel_e * dt_runs;
				map_projection_reproject(&ref, x_f, y_f, &x_f_NED, &y_f_NED);
				future_distance = fabsf(get_distance_to_next_waypoint(x_f_NED, y_f_NED, _drop_position.lat, _drop_position.lon));
				//warnx("position to-be: = %.7f  %.7f" ,x_f_NED, y_f_NED ); //////////////////////////////////////////////////////////////////// DEBUGGING

				// if (counter % 10 ==0) {
				// 	warnx("x: %.4f", x);
				// 	warnx("drop_position.lat: %.4f, drop_position.lon: %.4f", drop_position.lat, drop_position.lon);
				// 	warnx("latitude %.4f, longitude: %.4f", latitude, longitude);
				// 	warnx("future_distance: %.2f, precision: %.2f", future_distance, precision);
				// }

				/* Save WPs in datamanager */
				const ssize_t len = sizeof(struct mission_item_s);

				if (dm_write(DM_KEY_WAYPOINTS_ONBOARD, 0, DM_PERSIST_IN_FLIGHT_RESET, &flight_vector_s, len) != len) {
					warnx("ERROR: could not save onboard WP");
				}

				if (dm_write(DM_KEY_WAYPOINTS_ONBOARD, 1, DM_PERSIST_IN_FLIGHT_RESET, &flight_vector_e, len) != len) {
					warnx("ERROR: could not save onboard WP");
				}

				onboard_mission.count = 2;

				if (state_run && !state_drop) {
					onboard_mission.current_seq = 0;

				} else {
					onboard_mission.current_seq = -1;
				}

				if (counter % 10 == 0)
					warnx("Distance real: %.2f, distance_open_door: %.2f, angle to wind: %.2f", (double)distance_real,
					      (double)distance_open_door,
					      (double)(_wrap_pi(_global_pos.yaw - atan2f(wind.windspeed_north, wind.windspeed_east))));

				if (onboard_mission_pub > 0) {
					orb_publish(ORB_ID(onboard_mission), onboard_mission_pub, &onboard_mission);

				} else {
					onboard_mission_pub = orb_advertise(ORB_ID(onboard_mission), &onboard_mission);
				}

			}

			if (isfinite(distance_real) && distance_real < distance_open_door && _drop_approval) {
				open_bay();
			} else {
				close_bay();
			}

			if (isfinite(distance_real) && distance_real < precision && distance_real < future_distance) { // Drop only if the distance between drop point and actual position is getting larger again
				// XXX this check needs to be carefully validated - right now we prefer to drop if we're close to the goal

				// if(fabsf(_wrap_pi(_global_pos.yaw-atan2f(wind.windspeed_north,wind_speed.windspeed_east)+M_PI_F)) < 80.0f/180.0f*M_PI_F) // if flight trajectory deviates more than 80 degrees from calculated path, it will no drop
				// {
					drop();
				// }
				// else
				// {
				// 	state_run = true;
				// }
			}

			counter++;

			// update_actuators();

			// run at roughly 100 Hz
			usleep(sleeptime_us);

			dt_runs = 1e6f / hrt_elapsed_time(&last_run);
			last_run = hrt_absolute_time();
		}
	}

	warnx("exiting.");

	_main_task = -1;
	_exit(0);
}

void
BottleDrop::handle_command(struct vehicle_command_s *cmd)
{
	switch (cmd->command) {
	case VEHICLE_CMD_CUSTOM_0:
		/*
		 * param1 and param2 set to 1: open and drop
		 * param1 set to 1: open
		 * else: close (and don't drop)
		 */
		if (cmd->param1 > 0.5f && cmd->param2 > 0.5f) {
			_open_door = true;
			_drop = true;
			drop();
			mavlink_log_info(_mavlink_fd, "#audio: drop bottle");

		} else if (cmd->param1 > 0.5f) {
			_open_door = true;
			_drop = false;
			open_bay();
			mavlink_log_info(_mavlink_fd, "#audio: open doors");

		} else {
			_open_door = false;
			_drop = false;
			close_bay();
			mavlink_log_info(_mavlink_fd, "#audio: close doors");
		}

		answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
		break;

	case VEHICLE_CMD_PAYLOAD_PREPARE_DEPLOY:

		switch ((int)(cmd->param1 + 0.5f)) {
		case 0:
			_drop_approval = false;
			break;

		case 1:
			_drop_approval = true;
			break;

		default:
			_drop_approval = false;
			warnx("param1 val unknown");
			break;
		}

		// XXX check all fields (2-3)
		_alt_clearance = cmd->param4;
		_target_position.lat = cmd->param5;
		_target_position.lon = cmd->param6;
		_target_position.alt = cmd->param7;
		_target_position.initialized;
		map_projection_init(&ref, _target_position.lat, _target_position.lon);
		answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
		break;

	case VEHICLE_CMD_PAYLOAD_CONTROL_DEPLOY:
		switch ((int)(cmd->param1 + 0.5f)) {
		case 0:
			_drop_approval = false;
			break;

		case 1:
			_drop_approval = true;
			break;

		default:
			_drop_approval = false;
			break;
			// XXX handle other values
		}

		answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
		break;

	default:
		break;
	}
}

void
BottleDrop::answer_command(struct vehicle_command_s *cmd, enum VEHICLE_CMD_RESULT result)
{
	switch (result) {
	case VEHICLE_CMD_RESULT_ACCEPTED:
		break;

	case VEHICLE_CMD_RESULT_DENIED:
		mavlink_log_critical(_mavlink_fd, "#audio: command denied: %u", cmd->command);
		break;

	case VEHICLE_CMD_RESULT_FAILED:
		mavlink_log_critical(_mavlink_fd, "#audio: command failed: %u", cmd->command);
		break;

	case VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
		mavlink_log_critical(_mavlink_fd, "#audio: command temporarily rejected: %u", cmd->command);
		break;

	case VEHICLE_CMD_RESULT_UNSUPPORTED:
		mavlink_log_critical(_mavlink_fd, "#audio: command unsupported: %u", cmd->command);
		break;

	default:
		break;
	}
}

void
BottleDrop::task_main_trampoline(int argc, char *argv[])
{
	bottle_drop::g_bottle_drop->task_main();
}

static void usage()
{
	errx(1, "usage: bottle_drop {start|stop|status}");
}

int bottle_drop_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
	}

	if (!strcmp(argv[1], "start")) {

		if (bottle_drop::g_bottle_drop != nullptr) {
			errx(1, "already running");
		}

		bottle_drop::g_bottle_drop = new BottleDrop;

		if (bottle_drop::g_bottle_drop == nullptr) {
			errx(1, "alloc failed");
		}

		if (OK != bottle_drop::g_bottle_drop->start()) {
			delete bottle_drop::g_bottle_drop;
			bottle_drop::g_bottle_drop = nullptr;
			err(1, "start failed");
		}

		return 0;
	}

	if (bottle_drop::g_bottle_drop == nullptr) {
		errx(1, "not running");
	}

	if (!strcmp(argv[1], "stop")) {
		delete bottle_drop::g_bottle_drop;
		bottle_drop::g_bottle_drop = nullptr;

	} else if (!strcmp(argv[1], "status")) {
		bottle_drop::g_bottle_drop->status();

	} else if (!strcmp(argv[1], "drop")) {
		bottle_drop::g_bottle_drop->drop();

	} else if (!strcmp(argv[1], "open")) {
		bottle_drop::g_bottle_drop->open_bay();

	} else if (!strcmp(argv[1], "close")) {
		bottle_drop::g_bottle_drop->close_bay();

	} else {
		usage();
	}

	return 0;
}
