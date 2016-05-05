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

#include <px4_config.h>
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
#include <uORB/topics/actuator_controls_0.h>
#include <uORB/topics/actuator_controls_1.h>
#include <uORB/topics/actuator_controls_2.h>
#include <uORB/topics/actuator_controls_3.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_global_position.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <geo/geo.h>
#include <dataman/dataman.h>
#include <mathlib/mathlib.h>


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
	void		lock_release();

private:
	bool		_task_should_exit;		/**< if true, task should exit */
	int		_main_task;			/**< handle for task */
	orb_advert_t	_mavlink_log_pub;

	int		_command_sub;
	int		_wind_estimate_sub;
	struct vehicle_command_s	_command;
	struct vehicle_global_position_s _global_pos;
	map_projection_reference_s ref;

	orb_advert_t	_actuator_pub;
	struct actuator_controls_s _actuators;

	bool		_drop_approval;
	hrt_abstime	_doors_opened;
	hrt_abstime	_drop_time;

	float		_alt_clearance;

	struct position_s {
		double lat;	///< degrees
		double lon;	///< degrees
		float alt;	///< m
	} _target_position, _drop_position;

	enum DROP_STATE {
		DROP_STATE_INIT = 0,
		DROP_STATE_TARGET_VALID,
		DROP_STATE_TARGET_SET,
		DROP_STATE_BAY_OPEN,
		DROP_STATE_DROPPED,
		DROP_STATE_BAY_CLOSED
	} _drop_state;

	struct mission_s	_onboard_mission;
	orb_advert_t		_onboard_mission_pub;

	void		task_main();

	void		handle_command(struct vehicle_command_s *cmd);

	void		answer_command(struct vehicle_command_s *cmd, unsigned result);

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
	_command_sub(-1),
	_wind_estimate_sub(-1),
	_command {},
	_global_pos {},
	ref {},
	_actuator_pub(nullptr),
	_actuators {},
	_drop_approval(false),
	_doors_opened(0),
	_drop_time(0),
	_alt_clearance(70.0f),
	_target_position {},
	_drop_position {},
	_drop_state(DROP_STATE_INIT),
	_onboard_mission {},
	_onboard_mission_pub(nullptr)
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
	_main_task = px4_task_spawn_cmd("bottle_drop",
					SCHED_DEFAULT,
					SCHED_PRIORITY_DEFAULT + 15,
					1500,
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
	warnx("drop state: %d", _drop_state);
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

	usleep(500 * 1000);
}

void
BottleDrop::close_bay()
{
	// closed door and locked survival kit
	_actuators.control[0] = 1.0f;
	_actuators.control[1] = -1.0f;

	_doors_opened = 0;

	actuators_publish();

	// delay until the bay is closed
	usleep(500 * 1000);
}

void
BottleDrop::drop()
{

	// update drop actuator, wait 0.5s until the doors are open before dropping
	hrt_abstime starttime = hrt_absolute_time();

	// force the door open if we have to
	if (_doors_opened == 0) {
		open_bay();
	}

	while (hrt_elapsed_time(&_doors_opened) < 500 * 1000 && hrt_elapsed_time(&starttime) < 2000000) {
		usleep(50000);
		warnx("delayed by door!");
	}

	_actuators.control[2] = 1.0f;

	_drop_time = hrt_absolute_time();
	actuators_publish();

	warnx("dropping now");

	// Give it time to drop
	usleep(1000 * 1000);
}

void
BottleDrop::lock_release()
{
	_actuators.control[2] = -1.0f;
	actuators_publish();

	warnx("closing release");
}

int
BottleDrop::actuators_publish()
{
	_actuators.timestamp = hrt_absolute_time();

	// lazily publish _actuators only once available
	if (_actuator_pub != nullptr) {
		return orb_publish(ORB_ID(actuator_controls_2), _actuator_pub, &_actuators);

	} else {
		_actuator_pub = orb_advertise(ORB_ID(actuator_controls_2), &_actuators);

		if (_actuator_pub != nullptr) {
			return OK;

		} else {
			return -1;
		}
	}
}

void
BottleDrop::task_main()
{

	mavlink_log_info(&_mavlink_log_pub, "[bottle_drop] started");

	_command_sub = orb_subscribe(ORB_ID(vehicle_command));
	_wind_estimate_sub = orb_subscribe(ORB_ID(wind_estimate));

	bool updated = false;

	float z_0;		// ground properties
	float turn_radius;	// turn radius of the UAV
	float precision;	// Expected precision of the UAV

	float ground_distance = _alt_clearance;		// Replace by closer estimate in loop

	// constant
	float g = CONSTANTS_ONE_G;               		// constant of gravity [m/s^2]
	float m = 0.5f;                		// mass of bottle [kg]
	float rho = 1.2f;              		// air density [kg/m^3]
	float A = ((0.063f * 0.063f) / 4.0f * M_PI_F); // Bottle cross section [m^2]
	float dt_freefall_prediction = 0.01f;   // step size of the free fall prediction [s]

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
	float approach_error = 0.0f;				// The error in radians between current ground vector and desired ground vector
	float distance_real = 0;				// The distance between the UAVs position and the drop point [m]
	float future_distance = 0;				// The distance between the UAVs to-be position and the drop point [m]

	unsigned counter = 0;

	param_t param_gproperties = param_find("BD_GPROPERTIES");
	param_t param_turn_radius = param_find("BD_TURNRADIUS");
	param_t param_precision = param_find("BD_PRECISION");
	param_t param_cd = param_find("BD_OBJ_CD");
	param_t param_mass = param_find("BD_OBJ_MASS");
	param_t param_surface = param_find("BD_OBJ_SURFACE");


	param_get(param_precision, &precision);
	param_get(param_turn_radius, &turn_radius);
	param_get(param_gproperties, &z_0);
	param_get(param_cd, &cd);
	param_get(param_mass, &m);
	param_get(param_surface, &A);

	int vehicle_global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));

	struct parameter_update_s update;
	memset(&update, 0, sizeof(update));
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));

	struct mission_item_s flight_vector_s {};
	struct mission_item_s flight_vector_e {};

	flight_vector_s.nav_cmd = NAV_CMD_WAYPOINT;
	flight_vector_s.acceptance_radius = 50; // TODO: make parameter
	flight_vector_s.autocontinue = true;
	flight_vector_s.altitude_is_relative = false;

	flight_vector_e.nav_cmd = NAV_CMD_WAYPOINT;
	flight_vector_e.acceptance_radius = 50; // TODO: make parameter
	flight_vector_e.autocontinue = true;
	flight_vector_s.altitude_is_relative = false;

	struct wind_estimate_s wind;

	// wakeup source(s)
	struct pollfd fds[1];

	// Setup of loop
	fds[0].fd = _command_sub;
	fds[0].events = POLLIN;

	// Whatever state the bay is in, we want it closed on startup
	lock_release();
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
		}

		if (_global_pos.timestamp == 0) {
			continue;
		}

		const unsigned sleeptime_us = 9500;

		hrt_abstime last_run = hrt_absolute_time();
		float dt_runs = sleeptime_us / 1e6f;

		// switch to faster updates during the drop
		while (_drop_state > DROP_STATE_INIT) {

			// Get wind estimate
			orb_check(_wind_estimate_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(wind_estimate), _wind_estimate_sub, &wind);
			}

			// Get vehicle position
			orb_check(vehicle_global_position_sub, &updated);

			if (updated) {
				// copy global position
				orb_copy(ORB_ID(vehicle_global_position), vehicle_global_position_sub, &_global_pos);
			}

			// Get parameter updates
			orb_check(parameter_update_sub, &updated);

			if (updated) {
				// copy global position
				orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);

				// update all parameters
				param_get(param_gproperties, &z_0);
				param_get(param_turn_radius, &turn_radius);
				param_get(param_precision, &precision);
			}

			orb_check(_command_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(vehicle_command), _command_sub, &_command);
				handle_command(&_command);
			}


			float windspeed_norm = sqrtf(wind.windspeed_north * wind.windspeed_north + wind.windspeed_east * wind.windspeed_east);
			float groundspeed_body = sqrtf(_global_pos.vel_n * _global_pos.vel_n + _global_pos.vel_e * _global_pos.vel_e);
			ground_distance = _global_pos.alt - _target_position.alt;

			// Distance to drop position and angle error to approach vector
			// are relevant in all states greater than target valid (which calculates these positions)
			if (_drop_state > DROP_STATE_TARGET_VALID) {
				distance_real = fabsf(get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon, _drop_position.lat,
						      _drop_position.lon));

				float ground_direction = atan2f(_global_pos.vel_e, _global_pos.vel_n);
				float approach_direction = get_bearing_to_next_waypoint(flight_vector_s.lat, flight_vector_s.lon, flight_vector_e.lat,
							   flight_vector_e.lon);

				approach_error = _wrap_pi(ground_direction - approach_direction);

				if (counter % 90 == 0) {
					mavlink_log_info(&_mavlink_log_pub, "drop distance %u, heading error %u", (unsigned)distance_real,
							 (unsigned)math::degrees(approach_error));
				}
			}

			switch (_drop_state) {
			case DROP_STATE_INIT:
				// do nothing
				break;

			case DROP_STATE_TARGET_VALID: {

					az = g;							// acceleration in z direction[m/s^2]
					vz = 0; 						// velocity in z direction [m/s]
					z = 0; 							// fallen distance [m]
					h_0 = _global_pos.alt - _target_position.alt; 		// height over target at start[m]
					h = h_0;						// height over target [m]
					ax = 0;							// acceleration in x direction [m/s^2]
					vx = groundspeed_body;// XXX project					// ground speed in x direction [m/s]
					x = 0;							// traveled distance in x direction [m]
					vw = 0;							// wind speed [m/s]
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

						// drag force
						v = sqrtf(vz * vz + vrx * vrx);
						Fd = 0.5f * rho * A * cd * (v * v);
						Fdx = Fd * vrx / v;
						Fdz = Fd * vz / v;

						// acceleration
						az = g - Fdz / m;
						ax = -Fdx / m;
					}

					// compute drop vector
					x = groundspeed_body * t_signal + x;

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
					_drop_position.alt = _target_position.alt + _alt_clearance;

					// Compute flight vector
					map_projection_reproject(&ref, x_drop + 2 * turn_radius * wind_direction_n, y_drop + 2 * turn_radius * wind_direction_e,
								 &(flight_vector_s.lat), &(flight_vector_s.lon));
					flight_vector_s.altitude = _drop_position.alt;
					map_projection_reproject(&ref, x_drop - turn_radius * wind_direction_n, y_drop - turn_radius * wind_direction_e,
								 &flight_vector_e.lat, &flight_vector_e.lon);
					flight_vector_e.altitude = _drop_position.alt;

					// Save WPs in datamanager
					const ssize_t len = sizeof(struct mission_item_s);

					if (dm_write(DM_KEY_WAYPOINTS_ONBOARD, 0, DM_PERSIST_IN_FLIGHT_RESET, &flight_vector_s, len) != len) {
						warnx("ERROR: could not save onboard WP");
					}

					if (dm_write(DM_KEY_WAYPOINTS_ONBOARD, 1, DM_PERSIST_IN_FLIGHT_RESET, &flight_vector_e, len) != len) {
						warnx("ERROR: could not save onboard WP");
					}

					_onboard_mission.count = 2;
					_onboard_mission.current_seq = 0;

					if (_onboard_mission_pub != nullptr) {
						orb_publish(ORB_ID(onboard_mission), _onboard_mission_pub, &_onboard_mission);

					} else {
						_onboard_mission_pub = orb_advertise(ORB_ID(onboard_mission), &_onboard_mission);
					}

					float approach_direction = get_bearing_to_next_waypoint(flight_vector_s.lat, flight_vector_s.lon, flight_vector_e.lat,
								   flight_vector_e.lon);
					mavlink_log_critical(&_mavlink_log_pub, "position set, approach heading: %u", (unsigned)distance_real,
							     (unsigned)math::degrees(approach_direction + M_PI_F));

					_drop_state = DROP_STATE_TARGET_SET;
				}
				break;

			case DROP_STATE_TARGET_SET: {
					float distance_wp2 = get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon, flight_vector_e.lat,
							     flight_vector_e.lon);

					if (distance_wp2 < distance_real) {
						_onboard_mission.current_seq = 0;
						orb_publish(ORB_ID(onboard_mission), _onboard_mission_pub, &_onboard_mission);

					} else {

						// We're close enough - open the bay
						distance_open_door = math::max(10.0f, 3.0f * fabsf(t_door * groundspeed_body));

						if (isfinite(distance_real) && distance_real < distance_open_door &&
						    fabsf(approach_error) < math::radians(20.0f)) {
							open_bay();
							_drop_state = DROP_STATE_BAY_OPEN;
							mavlink_log_info(&_mavlink_log_pub, "#audio: opening bay");
						}
					}
				}
				break;

			case DROP_STATE_BAY_OPEN: {
					if (_drop_approval) {
						map_projection_project(&ref, _global_pos.lat, _global_pos.lon, &x_l, &y_l);
						x_f = x_l + _global_pos.vel_n * dt_runs;
						y_f = y_l + _global_pos.vel_e * dt_runs;
						map_projection_reproject(&ref, x_f, y_f, &x_f_NED, &y_f_NED);
						future_distance = get_distance_to_next_waypoint(x_f_NED, y_f_NED, _drop_position.lat, _drop_position.lon);

						if (isfinite(distance_real) &&
						    (distance_real < precision) && ((distance_real < future_distance))) {
							drop();
							_drop_state = DROP_STATE_DROPPED;
							mavlink_log_info(&_mavlink_log_pub, "#audio: payload dropped");

						} else {

							float distance_wp2 = get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon, flight_vector_e.lat,
									     flight_vector_e.lon);

							if (distance_wp2 < distance_real) {
								_onboard_mission.current_seq = 0;
								orb_publish(ORB_ID(onboard_mission), _onboard_mission_pub, &_onboard_mission);
							}
						}
					}
				}
				break;

			case DROP_STATE_DROPPED:

				/* 2s after drop, reset and close everything again */
				if ((hrt_elapsed_time(&_doors_opened) > 2 * 1000 * 1000)) {
					_drop_state = DROP_STATE_INIT;
					_drop_approval = false;
					lock_release();
					close_bay();
					mavlink_log_info(&_mavlink_log_pub, "#audio: closing bay");

					// remove onboard mission
					_onboard_mission.current_seq = -1;
					_onboard_mission.count = 0;
					orb_publish(ORB_ID(onboard_mission), _onboard_mission_pub, &_onboard_mission);
				}

				break;

			case DROP_STATE_BAY_CLOSED:
				// do nothing
				break;
			}

			counter++;

			// update_actuators();

			// run at roughly 100 Hz
			usleep(sleeptime_us);

			dt_runs = hrt_elapsed_time(&last_run) / 1e6f;
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
	case vehicle_command_s::VEHICLE_CMD_CUSTOM_0:

		/*
		 * param1 and param2 set to 1: open and drop
		 * param1 set to 1: open
		 * else: close (and don't drop)
		 */
		if (cmd->param1 > 0.5f && cmd->param2 > 0.5f) {
			open_bay();
			drop();
			mavlink_log_critical(&_mavlink_log_pub, "drop bottle");

		} else if (cmd->param1 > 0.5f) {
			open_bay();
			mavlink_log_critical(&_mavlink_log_pub, "opening bay");

		} else {
			lock_release();
			close_bay();
			mavlink_log_critical(&_mavlink_log_pub, "closing bay");
		}

		answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
		break;

	case vehicle_command_s::VEHICLE_CMD_PAYLOAD_PREPARE_DEPLOY:

		switch ((int)(cmd->param1 + 0.5f)) {
		case 0:
			_drop_approval = false;
			mavlink_log_critical(&_mavlink_log_pub, "got drop position, no approval");
			break;

		case 1:
			_drop_approval = true;
			mavlink_log_critical(&_mavlink_log_pub, "got drop position and approval");
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
		_drop_state = DROP_STATE_TARGET_VALID;
		mavlink_log_info(&_mavlink_log_pub, "got target: %8.4f, %8.4f, %8.4f", (double)_target_position.lat,
				 (double)_target_position.lon, (double)_target_position.alt);
		map_projection_init(&ref, _target_position.lat, _target_position.lon);
		answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
		break;

	case vehicle_command_s::VEHICLE_CMD_PAYLOAD_CONTROL_DEPLOY:

		if (cmd->param1 < 0) {

			// Clear internal states
			_drop_approval = false;
			_drop_state = DROP_STATE_INIT;

			// Abort if mission is present
			_onboard_mission.current_seq = -1;

			if (_onboard_mission_pub != nullptr) {
				orb_publish(ORB_ID(onboard_mission), _onboard_mission_pub, &_onboard_mission);
			}

		} else {
			switch ((int)(cmd->param1 + 0.5f)) {
			case 0:
				_drop_approval = false;
				break;

			case 1:
				_drop_approval = true;
				mavlink_log_info(&_mavlink_log_pub, "#audio: got drop approval");
				break;

			default:
				_drop_approval = false;
				break;
				// XXX handle other values
			}
		}

		answer_command(cmd, vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
		break;

	default:
		break;
	}
}

void
BottleDrop::answer_command(struct vehicle_command_s *cmd, unsigned result)
{
	switch (result) {
	case vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED:
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_DENIED:
		mavlink_log_critical(&_mavlink_log_pub, "command denied: %u", cmd->command);
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_FAILED:
		mavlink_log_critical(&_mavlink_log_pub, "command failed: %u", cmd->command);
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
		mavlink_log_critical(&_mavlink_log_pub, "command temporarily rejected: %u", cmd->command);
		break;

	case vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED:
		mavlink_log_critical(&_mavlink_log_pub, "command unsupported: %u", cmd->command);
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

	} else if (!strcmp(argv[1], "lock")) {
		bottle_drop::g_bottle_drop->lock_release();

	} else {
		usage();
	}

	return 0;
}
