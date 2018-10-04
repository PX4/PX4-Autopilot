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

#include "GroundRoverAttitudeControl.hpp"

/**
 * GroundRover attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int gnd_att_control_main(int argc, char *argv[]);

namespace att_gnd_control
{
GroundRoverAttitudeControl	*g_control = nullptr;
}

GroundRoverAttitudeControl::GroundRoverAttitudeControl() :
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "gnda_dt")),

	_nonfinite_input_perf(perf_alloc(PC_COUNT, "gnda_nani")),
	_nonfinite_output_perf(perf_alloc(PC_COUNT, "gnda_nano"))
{
	_parameter_handles.w_p = param_find("GND_WR_P");
	_parameter_handles.w_i = param_find("GND_WR_I");
	_parameter_handles.w_d = param_find("GND_WR_D");
	_parameter_handles.w_imax = param_find("GND_WR_IMAX");

	_parameter_handles.trim_yaw = param_find("TRIM_YAW");

	_parameter_handles.man_yaw_scale = param_find("GND_MAN_Y_SC");

	_parameter_handles.bat_scale_en = param_find("GND_BAT_SCALE_EN");

	/* fetch initial parameter values */
	parameters_update();
}

GroundRoverAttitudeControl::~GroundRoverAttitudeControl()
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

	perf_free(_loop_perf);
	perf_free(_nonfinite_input_perf);
	perf_free(_nonfinite_output_perf);

	att_gnd_control::g_control = nullptr;
}

void
GroundRoverAttitudeControl::parameters_update()
{
	param_get(_parameter_handles.w_p, &(_parameters.w_p));
	param_get(_parameter_handles.w_i, &(_parameters.w_i));
	param_get(_parameter_handles.w_d, &(_parameters.w_d));
	param_get(_parameter_handles.w_imax, &(_parameters.w_imax));

	param_get(_parameter_handles.trim_yaw, &(_parameters.trim_yaw));
	param_get(_parameter_handles.man_yaw_scale, &(_parameters.man_yaw_scale));

	param_get(_parameter_handles.bat_scale_en, &_parameters.bat_scale_en);

	/* Steering pid parameters*/
	pid_init(&_steering_ctrl, PID_MODE_DERIVATIV_SET, 0.01f);
	pid_set_parameters(&_steering_ctrl,
			   _parameters.w_p,
			   _parameters.w_i,
			   _parameters.w_d,
			   _parameters.w_imax,
			   1.0f);
}

void
GroundRoverAttitudeControl::vehicle_control_mode_poll()
{
	bool updated = false;
	orb_check(_vcontrol_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &_vcontrol_mode);
	}
}

void
GroundRoverAttitudeControl::manual_control_setpoint_poll()
{
	bool updated = false;
	orb_check(_manual_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}
}

void
GroundRoverAttitudeControl::vehicle_attitude_setpoint_poll()
{
	bool updated = false;
	orb_check(_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
	}
}

void
GroundRoverAttitudeControl::battery_status_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_battery_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
	}
}

int
GroundRoverAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
	att_gnd_control::g_control->task_main();
	return 0;
}

void
GroundRoverAttitudeControl::task_main()
{
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));

	parameters_update();

	/* get an initial update for all sensor and status data */
	vehicle_attitude_setpoint_poll();
	vehicle_control_mode_poll();
	manual_control_setpoint_poll();
	battery_status_poll();

	/* wakeup source */
	px4_pollfd_struct_t fds[2];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _att_sub;
	fds[1].events = POLLIN;

	_task_running = true;

	while (!_task_should_exit) {
		static int loop_counter = 0;

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

		perf_begin(_loop_perf);

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* only run controller if attitude changed */
		if (fds[1].revents & POLLIN) {
			static uint64_t last_run = 0;
			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too large deltaT's */
			if (deltaT > 1.0f ||
			    fabsf(deltaT) < 0.00001f ||
			    !PX4_ISFINITE(deltaT)) {
				deltaT = 0.01f;
			}

			/* load local copies */
			orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);

			vehicle_attitude_setpoint_poll();
			vehicle_control_mode_poll();
			manual_control_setpoint_poll();
			battery_status_poll();

			/* decide if in stabilized or full manual control */
			if (_vcontrol_mode.flag_control_rates_enabled) {
				/* Run attitude controllers */
				if (_vcontrol_mode.flag_control_attitude_enabled) {

					Eulerf euler_angles(matrix::Quatf(_att.q));

					/* Calculate the control output for the steering as yaw */
					float yaw_u = pid_calculate(&_steering_ctrl, _att_sp.yaw_body, euler_angles.psi(), _att.yawspeed, deltaT);

					float angle_diff = 0.0f;

					if (_att_sp.yaw_body * euler_angles.psi() < 0.0f) {
						if (_att_sp.yaw_body < 0.0f) {
							angle_diff = euler_angles.psi() - _att_sp.yaw_body ;

						} else {
							angle_diff = _att_sp.yaw_body - euler_angles.psi();
						}

						// a switch might have happened
						if ((double)angle_diff > M_PI) {
							yaw_u = -yaw_u;
						}

					}

					math::constrain(yaw_u, -1.0f, 1.0f);

					if (PX4_ISFINITE(yaw_u)) {
						_actuators.control[actuator_controls_s::INDEX_YAW] = yaw_u + _parameters.trim_yaw;

					} else {
						_actuators.control[actuator_controls_s::INDEX_YAW] = _parameters.trim_yaw;

						perf_count(_nonfinite_output_perf);

						if (_debug && loop_counter % 10 == 0) {
							warnx("yaw_u %.4f", (double)yaw_u);
						}
					}

					/* throttle passed through if it is finite and if no engine failure was detected */
					_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _att_sp.thrust_body[0];

					/* scale effort by battery status */
					if (_parameters.bat_scale_en && _battery_status.scale > 0.0f &&
					    _actuators.control[actuator_controls_s::INDEX_THROTTLE] > 0.1f) {

						_actuators.control[actuator_controls_s::INDEX_THROTTLE] *= _battery_status.scale;
					}
				}

			} else {
				/* manual/direct control */
				_actuators.control[actuator_controls_s::INDEX_ROLL] = _manual.y;
				_actuators.control[actuator_controls_s::INDEX_PITCH] = -_manual.x;
				_actuators.control[actuator_controls_s::INDEX_YAW] = _manual.r * _parameters.man_yaw_scale + _parameters.trim_yaw;
				_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _manual.z;
			}

			/* lazily publish the setpoint only once available */
			_actuators.timestamp = hrt_absolute_time();
			_actuators.timestamp_sample = _att.timestamp;

			/* Only publish if any of the proper modes are enabled */
			if (_vcontrol_mode.flag_control_attitude_enabled ||
			    _vcontrol_mode.flag_control_manual_enabled) {

				/* publish the actuator controls */
				if (_actuators_0_pub != nullptr) {
					orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, _actuators_0_pub, &_actuators);

				} else {
					_actuators_0_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &_actuators);
				}
			}
		}

		loop_counter++;
		perf_end(_loop_perf);
	}

	warnx("exiting.\n");

	_control_task = -1;
	_task_running = false;
}

int
GroundRoverAttitudeControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("gnd_att_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1500,
					   (px4_main_t)&GroundRoverAttitudeControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return PX4_OK;
}

int gnd_att_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: gnd_att_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (att_gnd_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		att_gnd_control::g_control = new GroundRoverAttitudeControl;

		if (att_gnd_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (PX4_OK != att_gnd_control::g_control->start()) {
			delete att_gnd_control::g_control;
			att_gnd_control::g_control = nullptr;
			warn("start failed");
			return 1;
		}

		/* check if the waiting is necessary at all */
		if (att_gnd_control::g_control == nullptr || !att_gnd_control::g_control->task_running()) {

			/* avoid memory fragmentation by not exiting start handler until the task has fully started */
			while (att_gnd_control::g_control == nullptr || !att_gnd_control::g_control->task_running()) {
				px4_usleep(50000);
				printf(".");
				fflush(stdout);
			}

			printf("\n");
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (att_gnd_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete att_gnd_control::g_control;
		att_gnd_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (att_gnd_control::g_control) {
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
