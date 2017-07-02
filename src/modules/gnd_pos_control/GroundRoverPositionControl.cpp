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


#include "GroundRoverPositionControl.hpp"

static int _control_task = -1;			/**< task handle for sensor task */

using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

/**
 * L1 control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int gnd_pos_control_main(int argc, char *argv[]);


namespace gnd_control
{
GroundRoverPositionControl	*g_control = nullptr;
}

GroundRoverPositionControl::GroundRoverPositionControl() :
	/* performance counters */
	_sub_attitude(ORB_ID(vehicle_attitude), 0, 0, nullptr),
	_sub_sensors(ORB_ID(sensor_combined), 0, 0, nullptr),
	_loop_perf(perf_alloc(PC_ELAPSED, "fw l1 control"))
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

	/* fetch initial parameter values */
	parameters_update();
}

GroundRoverPositionControl::~GroundRoverPositionControl()
{
	if (_control_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

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
GroundRoverPositionControl::parameters_update()
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

	/* Update and publish the navigation capabilities */
	_gnd_pos_ctrl_status.landing_slope_angle_rad = 0;
	_gnd_pos_ctrl_status.landing_horizontal_slope_displacement = 0;
	_gnd_pos_ctrl_status.landing_flare_length = 0;
	gnd_pos_ctrl_status_publish();

	return OK;
}

void
GroundRoverPositionControl::vehicle_control_mode_poll()
{
	bool updated;
	orb_check(_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
	}
}

void
GroundRoverPositionControl::manual_control_setpoint_poll()
{
	bool manual_updated;
	orb_check(_manual_control_sub, &manual_updated);

	if (manual_updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sub, &_manual);
	}
}

void
GroundRoverPositionControl::position_setpoint_triplet_poll()
{
	bool pos_sp_triplet_updated;
	orb_check(_pos_sp_triplet_sub, &pos_sp_triplet_updated);

	if (pos_sp_triplet_updated) {
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);
	}
}

void GroundRoverPositionControl::gnd_pos_ctrl_status_publish()
{
	_gnd_pos_ctrl_status.timestamp = hrt_absolute_time();

	if (_gnd_pos_ctrl_status_pub != nullptr) {
		orb_publish(ORB_ID(fw_pos_ctrl_status), _gnd_pos_ctrl_status_pub, &_gnd_pos_ctrl_status);

	} else {
		_gnd_pos_ctrl_status_pub = orb_advertise(ORB_ID(fw_pos_ctrl_status), &_gnd_pos_ctrl_status);
	}
}

bool
GroundRoverPositionControl::control_position(const math::Vector<2> &current_position,
		const math::Vector<3> &ground_speed, const position_setpoint_triplet_s &pos_sp_triplet)
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
		bool was_circle_mode = _gnd_control.circle_mode();

		/* current waypoint (the one currently heading for) */
		math::Vector<2> curr_wp((float)pos_sp_triplet.current.lat, (float)pos_sp_triplet.current.lon);

		/* previous waypoint */
		math::Vector<2> prev_wp = curr_wp;

		if (pos_sp_triplet.previous.valid) {
			prev_wp(0) = (float)pos_sp_triplet.previous.lat;
			prev_wp(1) = (float)pos_sp_triplet.previous.lon;
		}

		math::Vector<2> ground_speed_2d = {ground_speed(0), ground_speed(1)};

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
			const Dcmf R_to_body(Quatf(_sub_attitude.get().q).inversed());
			const Vector3f vel = R_to_body * Vector3f(ground_speed(0), ground_speed(1), ground_speed(2));

			const float x_vel = vel(0);
			const float x_acc = _sub_sensors.get().accelerometer_m_s2[0];

			//Compute airspeed control out and just scale it as a constant
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

		if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
			_att_sp.roll_body = 0.0f;
			_att_sp.pitch_body = 0.0f;
			_att_sp.yaw_body = 0.0f;
			_att_sp.thrust = 0.0f;

		} else if ((pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_POSITION)
			   || (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF)) {

			/* waypoint is a plain navigation waypoint or the takeoff waypoint, does not matter */
			_gnd_control.navigate_waypoints(prev_wp, curr_wp, current_position, ground_speed_2d);
			_att_sp.roll_body = _gnd_control.nav_roll();
			_att_sp.pitch_body = 0.0f;
			_att_sp.yaw_body = _gnd_control.nav_bearing();
			_att_sp.fw_control_yaw = true;
			_att_sp.thrust = mission_throttle;

		} else if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER) {

			/* waypoint is a loiter waypoint so we want to stop*/
			_gnd_control.navigate_loiter(curr_wp, current_position, pos_sp_triplet.current.loiter_radius,
						     pos_sp_triplet.current.loiter_direction, ground_speed_2d);

			_att_sp.roll_body = _gnd_control.nav_roll();
			_att_sp.pitch_body = 0.0f;
			_att_sp.yaw_body = _gnd_control.nav_bearing();
			_att_sp.fw_control_yaw = true;
			_att_sp.thrust = 0.0f;
		}

		if (was_circle_mode && !_gnd_control.circle_mode()) {
			/* just kicked out of loiter, reset integrals */
			_att_sp.yaw_reset_integral = true;
		}

	} else {
		_control_mode_current = UGV_POSCTRL_MODE_OTHER;

		_att_sp.roll_body = 0.0f;
		_att_sp.pitch_body = 0.0f;
		_att_sp.yaw_body = 0.0f;
		_att_sp.fw_control_yaw = true;
		_att_sp.thrust = 0.0f;

		/* do not publish the setpoint */
		setpoint = false;
	}

	return setpoint;
}

void
GroundRoverPositionControl::task_main()
{
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));

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
	px4_pollfd_struct_t fds[2];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _global_pos_sub;
	fds[1].events = POLLIN;

	_task_running = true;

	while (!_task_should_exit) {

		/* wait for up to 500ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		/* check vehicle control mode for changes to publication state */
		vehicle_control_mode_poll();

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* only run controller if position changed */
		if (fds[1].revents & POLLIN) {
			perf_begin(_loop_perf);

			/* load local copies */
			orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);

			// handle estimator reset events. we only adjust setpoins for manual modes
			if (_control_mode.flag_control_manual_enabled) {

				// adjust navigation waypoints in position control mode
				if (_control_mode.flag_control_altitude_enabled && _control_mode.flag_control_velocity_enabled
				    && _global_pos.lat_lon_reset_counter != _pos_reset_counter) {
				}
			}

			// update the reset counters in any case
			_pos_reset_counter = _global_pos.lat_lon_reset_counter;

			manual_control_setpoint_poll();
			position_setpoint_triplet_poll();
			_sub_attitude.update();
			_sub_sensors.update();

			math::Vector<3> ground_speed(_global_pos.vel_n, _global_pos.vel_e,  _global_pos.vel_d);
			math::Vector<2> current_position((float)_global_pos.lat, (float)_global_pos.lon);

			/*
			 * Attempt to control position, on success (= sensors present and not in manual mode),
			 * publish setpoint.
			 */
			if (control_position(current_position, ground_speed, _pos_sp_triplet)) {
				_att_sp.timestamp = hrt_absolute_time();

				Quatf q(Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body));
				q.copyTo(_att_sp.q_d);
				_att_sp.q_d_valid = true;

				if (!_control_mode.flag_control_offboard_enabled ||
				    _control_mode.flag_control_position_enabled ||
				    _control_mode.flag_control_velocity_enabled ||
				    _control_mode.flag_control_acceleration_enabled) {

					/* lazily publish the setpoint only once available */
					if (_attitude_sp_pub != nullptr) {
						/* publish the attitude setpoint */
						orb_publish(ORB_ID(vehicle_attitude_setpoint), _attitude_sp_pub, &_att_sp);

					} else {
						/* advertise and publish */
						_attitude_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_att_sp);
					}
				}

				/* XXX check if radius makes sense here */
				float turn_distance = _parameters.l1_distance; //_gnd_control.switch_distance(100.0f);

				/* lazily publish navigation capabilities */
				if ((hrt_elapsed_time(&_gnd_pos_ctrl_status.timestamp) > 1000000)
				    || (fabsf(turn_distance - _gnd_pos_ctrl_status.turn_distance) > FLT_EPSILON
					&& turn_distance > 0)) {

					/* set new turn distance */
					_gnd_pos_ctrl_status.turn_distance = turn_distance;

					_gnd_pos_ctrl_status.nav_roll = _gnd_control.nav_roll();
					_gnd_pos_ctrl_status.nav_pitch = 0.0f;
					_gnd_pos_ctrl_status.nav_bearing = _gnd_control.nav_bearing();

					_gnd_pos_ctrl_status.target_bearing = _gnd_control.target_bearing();
					_gnd_pos_ctrl_status.xtrack_error = _gnd_control.crosstrack_error();

					math::Vector<2> curr_wp((float)_pos_sp_triplet.current.lat, (float)_pos_sp_triplet.current.lon);
					_gnd_pos_ctrl_status.wp_dist = get_distance_to_next_waypoint(current_position(0), current_position(1), curr_wp(0),
								       curr_wp(1));

					gnd_pos_ctrl_status_publish();
				}
			}

			perf_end(_loop_perf);
		}
	}

	_task_running = false;

	warnx("exiting.\n");

	_control_task = -1;
}

void
GroundRoverPositionControl::task_main_trampoline(int argc, char *argv[])
{
	gnd_control::g_control = new GroundRoverPositionControl();

	if (gnd_control::g_control == nullptr) {
		warnx("OUT OF MEM");
		return;
	}

	/* only returns on exit */
	gnd_control::g_control->task_main();
	delete gnd_control::g_control;
	gnd_control::g_control = nullptr;
}

int
GroundRoverPositionControl::start()
{
	ASSERT(_control_task == -1);
	warn("Starting by marco");

	/* start the task */
	_control_task = px4_task_spawn_cmd("gnd_pos_ctrl",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1700,
					   (px4_main_t)&GroundRoverPositionControl::task_main_trampoline,
					   nullptr);
	warn("done");

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int gnd_pos_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: gnd_pos_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (gnd_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		if (OK != GroundRoverPositionControl::start()) {
			warn("start failed");
			return 1;
		}

		/* avoid memory fragmentation by not exiting start handler until the task has fully started */
		while (gnd_control::g_control == nullptr || !gnd_control::g_control->task_running()) {
			usleep(50000);
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
