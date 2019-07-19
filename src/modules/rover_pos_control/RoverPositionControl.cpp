/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 *
 * This module is a modification of the fixed wing module and it is designed for ground rovers.
 * It has been developed starting from the fw module, simplified and improved with dedicated items.
 *
 * All the acknowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Marco Zorzi <mzorzi@student.ethz.ch>
 */


#include "RoverPositionControl.hpp"
#include <lib/ecl/geo/geo.h>

#define ACTUATOR_PUBLISH_PERIOD_MS 4

static int _control_task = -1;			/**< task handle for sensor task */

using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

/**
 * L1 control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int rover_pos_control_main(int argc, char *argv[]);


namespace gnd_control
{
RoverPositionControl	*g_control = nullptr;
}

RoverPositionControl::RoverPositionControl() :
	/* performance counters */
	_sub_sensors(ORB_ID(sensor_bias)),
	_loop_perf(perf_alloc(PC_ELAPSED, "rover position control")) // TODO : do we even need these perf counters
{
	_parameter_handles.l1_period = param_find("GND_L1_PERIOD");
	_parameter_handles.l1_damping = param_find("GND_L1_DAMPING");
	_parameter_handles.l1_distance = param_find("GND_L1_DIST");

	_parameter_handles.gndspeed_trim = param_find("GND_SPEED_TRIM");
	_parameter_handles.gndspeed_max = param_find("GND_SPEED_MAX");

	_parameter_handles.speed_control_mode = param_find("GND_SP_CTRL_MODE");
	_parameter_handles.speed_p = param_find("GND_SPEED_P");
	_parameter_handles.speed_i = param_find("GND_SPEED_I");
	_parameter_handles.speed_d = param_find("GND_SPEED_D");
	_parameter_handles.speed_imax = param_find("GND_SPEED_IMAX");
	_parameter_handles.throttle_speed_scaler = param_find("GND_SPEED_THR_SC");

	_parameter_handles.throttle_min = param_find("GND_THR_MIN");
	_parameter_handles.throttle_max = param_find("GND_THR_MAX");
	_parameter_handles.throttle_cruise = param_find("GND_THR_CRUISE");

	_parameter_handles.wheel_base = param_find("GND_WHEEL_BASE");
	_parameter_handles.max_turn_angle = param_find("GND_MAX_ANG");

	/* fetch initial parameter values */
	parameters_update();
}

RoverPositionControl::~RoverPositionControl()
{
	if (_control_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			px4_usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	gnd_control::g_control = nullptr;
}

int
RoverPositionControl::parameters_update()
{
	/* L1 control parameters */
	param_get(_parameter_handles.l1_damping, &(_parameters.l1_damping));
	param_get(_parameter_handles.l1_period, &(_parameters.l1_period));
	param_get(_parameter_handles.l1_distance, &(_parameters.l1_distance));

	param_get(_parameter_handles.gndspeed_trim, &(_parameters.gndspeed_trim));
	param_get(_parameter_handles.gndspeed_max, &(_parameters.gndspeed_max));

	param_get(_parameter_handles.speed_control_mode, &(_parameters.speed_control_mode));
	param_get(_parameter_handles.speed_p, &(_parameters.speed_p));
	param_get(_parameter_handles.speed_i, &(_parameters.speed_i));
	param_get(_parameter_handles.speed_d, &(_parameters.speed_d));
	param_get(_parameter_handles.speed_imax, &(_parameters.speed_imax));
	param_get(_parameter_handles.throttle_speed_scaler, &(_parameters.throttle_speed_scaler));

	param_get(_parameter_handles.throttle_min, &(_parameters.throttle_min));
	param_get(_parameter_handles.throttle_max, &(_parameters.throttle_max));
	param_get(_parameter_handles.throttle_cruise, &(_parameters.throttle_cruise));

	param_get(_parameter_handles.wheel_base, &(_parameters.wheel_base));
	param_get(_parameter_handles.max_turn_angle, &(_parameters.max_turn_angle));

	_gnd_control.set_l1_damping(_parameters.l1_damping);
	_gnd_control.set_l1_period(_parameters.l1_period);
	_gnd_control.set_l1_roll_limit(math::radians(0.0f));

	pid_init(&_speed_ctrl, PID_MODE_DERIVATIV_CALC, 0.01f);
	pid_set_parameters(&_speed_ctrl,
			   _parameters.speed_p,
			   _parameters.speed_d,
			   _parameters.speed_i,
			   _parameters.speed_imax,
			   _parameters.gndspeed_max);

	return OK;
}

void
RoverPositionControl::vehicle_control_mode_poll()
{
	bool updated;
	orb_check(_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
	}
}

void
RoverPositionControl::manual_control_setpoint_poll()
{
	bool manual_updated;
	orb_check(_manual_control_sub, &manual_updated);

	if (manual_updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sub, &_manual);
	}
}

void
RoverPositionControl::position_setpoint_triplet_poll()
{
	bool pos_sp_triplet_updated;
	orb_check(_pos_sp_triplet_sub, &pos_sp_triplet_updated);

	if (pos_sp_triplet_updated) {
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);
	}
}

void
RoverPositionControl::vehicle_attitude_poll()
{
	bool att_updated;
	orb_check(_vehicle_attitude_sub, &att_updated);

	if (att_updated) {
		orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_vehicle_att);
	}
}

bool
RoverPositionControl::control_position(const matrix::Vector2f &current_position,
				       const matrix::Vector3f &ground_speed, const position_setpoint_triplet_s &pos_sp_triplet)
{
	float dt = 0.01; // Using non zero value to a avoid division by zero

	if (_control_position_last_called > 0) {
		dt = hrt_elapsed_time(&_control_position_last_called) * 1e-6f;
	}

	_control_position_last_called = hrt_absolute_time();

	bool setpoint = true;

	if (_control_mode.flag_control_auto_enabled && pos_sp_triplet.current.valid) {
		/* AUTONOMOUS FLIGHT */

		_control_mode_current = UGV_POSCTRL_MODE_AUTO;

		/* get circle mode */
		//bool was_circle_mode = _gnd_control.circle_mode();

		/* current waypoint (the one currently heading for) */
		matrix::Vector2f curr_wp((float)pos_sp_triplet.current.lat, (float)pos_sp_triplet.current.lon);

		/* previous waypoint */
		matrix::Vector2f prev_wp = curr_wp;

		if (pos_sp_triplet.previous.valid) {
			prev_wp(0) = (float)pos_sp_triplet.previous.lat;
			prev_wp(1) = (float)pos_sp_triplet.previous.lon;
		}

		matrix::Vector2f ground_speed_2d(ground_speed);

		float mission_throttle = _parameters.throttle_cruise;

		/* Just control the throttle */
		if (_parameters.speed_control_mode == 1) {
			/* control the speed in closed loop */

			float mission_target_speed = _parameters.gndspeed_trim;

			if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_speed) &&
			    _pos_sp_triplet.current.cruising_speed > 0.1f) {
				mission_target_speed = _pos_sp_triplet.current.cruising_speed;
			}

			// Velocity in body frame
			const Dcmf R_to_body(Quatf(_vehicle_att.q).inversed());
			const Vector3f vel = R_to_body * Vector3f(ground_speed(0), ground_speed(1), ground_speed(2));

			const float x_vel = vel(0);
			const float x_acc = _sub_sensors.get().accel_x;

			// Compute airspeed control out and just scale it as a constant
			mission_throttle = _parameters.throttle_speed_scaler
					   * pid_calculate(&_speed_ctrl, mission_target_speed, x_vel, x_acc, dt);

			// Constrain throttle between min and max
			mission_throttle = math::constrain(mission_throttle, _parameters.throttle_min, _parameters.throttle_max);

		} else {
			/* Just control throttle in open loop */
			if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_throttle) &&
			    _pos_sp_triplet.current.cruising_throttle > 0.01f) {

				mission_throttle = _pos_sp_triplet.current.cruising_throttle;
			}
		}

		float dist = get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon,
				pos_sp_triplet.current.lat, pos_sp_triplet.current.lon);

		bool should_idle = true;

		if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER) {
			// Because of noise in measurements, if the rover was always trying to reach an exact point, it would
			// move around when it should be parked. So, I try to get the rover within loiter_radius/2, but then
			// once I reach that point, I don't move until I'm outside of loiter_radius.
			// TODO: Find out if there's a better measurement to use than loiter_radius.
			if (dist > pos_sp_triplet.current.loiter_radius) {
				_waypoint_reached = false;

			} else if (dist <= pos_sp_triplet.current.loiter_radius / 2) {
				_waypoint_reached = true;
			}

			should_idle = _waypoint_reached;

		} else if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_POSITION ||
			   pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF ||
			   pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
			should_idle = false;
		}


		if (should_idle) {
			_act_controls.control[actuator_controls_s::INDEX_YAW] = 0.0f;
			_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;

		} else {

			/* waypoint is a plain navigation waypoint or the takeoff waypoint, does not matter */
			_gnd_control.navigate_waypoints(prev_wp, curr_wp, current_position, ground_speed_2d);

			_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = mission_throttle;

			float desired_r = ground_speed_2d.norm_squared() / math::abs_t(_gnd_control.nav_lateral_acceleration_demand());
			float desired_theta = (0.5f * M_PI_F) - atan2f(desired_r, _parameters.wheel_base);
			float control_effort = (desired_theta / _parameters.max_turn_angle) * math::sign(
						       _gnd_control.nav_lateral_acceleration_demand());
			control_effort = math::constrain(control_effort, -1.0f, 1.0f);
			_act_controls.control[actuator_controls_s::INDEX_YAW] = control_effort;

		}

	} else {
		_control_mode_current = UGV_POSCTRL_MODE_OTHER;
		setpoint = false;
	}

	return setpoint;
}

void
RoverPositionControl::task_main()
{
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	_vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));

	/* rate limit control mode updates to 5Hz */
	orb_set_interval(_control_mode_sub, 200);

	/* rate limit position updates to 50 Hz */
	orb_set_interval(_global_pos_sub, 20);

	/* abort on a nonzero return value from the parameter init */
	if (parameters_update()) {
		/* parameter setup went wrong, abort */
		warnx("aborting startup due to errors.");
		_task_should_exit = true;
	}

	/* wakeup source(s) */
	px4_pollfd_struct_t fds[3];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _global_pos_sub;
	fds[1].events = POLLIN;
	fds[2].fd = _manual_control_sub;
	fds[2].events = POLLIN;

	_task_running = true;

	// Absolute time (in us) at which the actuators were last published
	long actuators_last_published = 0;
	// Timeout for poll in ms
	int timeout = 0;

	while (!_task_should_exit) {

		// The +500 is to round to the nearest millisecond, instead of to the smallest millisecond.
		timeout = ACTUATOR_PUBLISH_PERIOD_MS - (hrt_absolute_time() - actuators_last_published) / 1000 - 1;
		timeout = timeout > 0 ? timeout : 0;

		//PX4_INFO("TIMEOUT: %d", timeout);

		/* wait for up to 500ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), timeout);

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		/* check vehicle control mode for changes to publication state */
		vehicle_control_mode_poll();
		//manual_control_setpoint_poll();

		_sub_sensors.update();

		bool manual_mode = _control_mode.flag_control_manual_enabled;

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* only run controller if position changed */
		if (fds[1].revents & POLLIN) {
			perf_begin(_loop_perf);

			/* load local copies */
			orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);

			position_setpoint_triplet_poll();
			vehicle_attitude_poll();

			// update the reset counters in any case
			_pos_reset_counter = _global_pos.lat_lon_reset_counter;

			matrix::Vector3f ground_speed(_global_pos.vel_n, _global_pos.vel_e,  _global_pos.vel_d);
			matrix::Vector2f current_position((float)_global_pos.lat, (float)_global_pos.lon);

			// This if statement depends upon short-circuiting: If !manual_mode, then control_position(...)
			// should not be called.
			// It doesn't really matter if it is called, it will just be bad for performance.
			if (!manual_mode && control_position(current_position, ground_speed, _pos_sp_triplet)) {

				/* XXX check if radius makes sense here */
				float turn_distance = _parameters.l1_distance; //_gnd_control.switch_distance(100.0f);

				// publish status
				position_controller_status_s pos_ctrl_status = {};

				pos_ctrl_status.nav_roll = 0.0f;
				pos_ctrl_status.nav_pitch = 0.0f;
				pos_ctrl_status.nav_bearing = _gnd_control.nav_bearing();

				pos_ctrl_status.target_bearing = _gnd_control.target_bearing();
				pos_ctrl_status.xtrack_error = _gnd_control.crosstrack_error();

				pos_ctrl_status.wp_dist = get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon,
							  _pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon);

				pos_ctrl_status.acceptance_radius = turn_distance;
				pos_ctrl_status.yaw_acceptance = NAN;

				pos_ctrl_status.timestamp = hrt_absolute_time();

				if (_pos_ctrl_status_pub != nullptr) {
					orb_publish(ORB_ID(position_controller_status), _pos_ctrl_status_pub, &pos_ctrl_status);

				} else {
					_pos_ctrl_status_pub = orb_advertise(ORB_ID(position_controller_status), &pos_ctrl_status);
				}

			}


			perf_end(_loop_perf);
		}

		if (fds[2].revents & POLLIN) {

			// This should be copied even if not in manual mode. Otherwise, the poll(...) call will keep
			// returning immediately and this loop will eat up resources.
			orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sub, &_manual);

			if (manual_mode) {
				/* manual/direct control */
				//PX4_INFO("Manual mode!");
				_act_controls.control[actuator_controls_s::INDEX_ROLL] = _manual.y;
				_act_controls.control[actuator_controls_s::INDEX_PITCH] = -_manual.x;
				_act_controls.control[actuator_controls_s::INDEX_YAW] = _manual.r; //TODO: Readd yaw scale param
				_act_controls.control[actuator_controls_s::INDEX_THROTTLE] = _manual.z;
			}
		}

		if (pret == 0) {

			//orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_vehicle_att);
			_act_controls.timestamp = hrt_absolute_time();

			if (_actuator_controls_pub != nullptr) {
				//PX4_INFO("Publishing actuator from pos control");
				orb_publish(ORB_ID(actuator_controls_0), _actuator_controls_pub, &_act_controls);

			} else {

				_actuator_controls_pub = orb_advertise(ORB_ID(actuator_controls_0), &_act_controls);
			}

			actuators_last_published = hrt_absolute_time();
		}

	}

	orb_unsubscribe(_control_mode_sub);
	orb_unsubscribe(_global_pos_sub);
	orb_unsubscribe(_manual_control_sub);
	orb_unsubscribe(_params_sub);
	orb_unsubscribe(_pos_sp_triplet_sub);
	orb_unsubscribe(_vehicle_attitude_sub);

	_task_running = false;

	warnx("exiting.\n");

	_control_task = -1;
}

int
RoverPositionControl::task_main_trampoline(int argc, char *argv[])
{
	gnd_control::g_control = new RoverPositionControl();

	if (gnd_control::g_control == nullptr) {
		warnx("OUT OF MEM");
		return -1;
	}

	/* only returns on exit */
	gnd_control::g_control->task_main();
	delete gnd_control::g_control;
	gnd_control::g_control = nullptr;
	return 0;
}

int
RoverPositionControl::start()
{
	/* start the task */
	_control_task = px4_task_spawn_cmd("rover_pos_ctrl",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_POSITION_CONTROL,
					   1700,
					   (px4_main_t)&RoverPositionControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int rover_pos_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: rover_pos_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (gnd_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		if (OK != RoverPositionControl::start()) {
			warn("start failed");
			return 1;
		}

		/* avoid memory fragmentation by not exiting start handler until the task has fully started */
		while (gnd_control::g_control == nullptr || !gnd_control::g_control->task_running()) {
			px4_usleep(50000);
			printf(".");
			fflush(stdout);
		}

		printf("\n");

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (gnd_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete gnd_control::g_control;
		gnd_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (gnd_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
