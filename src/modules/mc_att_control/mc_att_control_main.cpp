/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *   Author: @author Tobias Naegeli <naegelit@student.ethz.ch>
 *           @author Lorenz Meier <lm@inf.ethz.ch>
 *           @author Anton Babushkin <anton.babushkin@me.com>
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
 * @file mc_att_control_main.c
 * Multicopter attitude controller.
 *
 * The controller has two loops: P loop for angular error and PD loop for angular rate error.
 * Desired rotation calculated keeping in mind that yaw response is normally slower than roll/pitch.
 * For small deviations controller rotates copter to have shortest path of thrust vector and independently rotates around yaw,
 * so actual rotation axis is not constant. For large deviations controller rotates copter around fixed axis.
 * These two approaches fused seamlessly with weight depending on angular error.
 * When thrust vector directed near-horizontally (e.g. roll ~= PI/2) yaw setpoint ignored because of singularity.
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 * If rotation matrix setpoint is invalid it will be generated from Euler angles for compatibility with old position controllers.
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
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/pid/pid.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>

/**
 * Multicopter attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_att_control_main(int argc, char *argv[]);

#define MIN_TAKEOFF_THROTTLE	0.3f
#define YAW_DEADZONE	0.01f

class MulticopterAttitudeControl
{
public:
	/**
	 * Constructor
	 */
	MulticopterAttitudeControl();

	/**
	 * Destructor, also kills the sensors task.
	 */
	~MulticopterAttitudeControl();

	/**
	 * Start the sensors task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:

	bool	_task_should_exit;		/**< if true, sensor task should exit */
	int		_control_task;			/**< task handle for sensor task */

	int		_att_sub;				/**< vehicle attitude subscription */
	int		_att_sp_sub;			/**< vehicle attitude setpoint */
	int		_control_mode_sub;		/**< vehicle control mode subscription */
	int 	_params_sub;			/**< notification of parameter updates */
	int 	_manual_sub;			/**< notification of manual control updates */
	int		_arming_sub;			/**< arming status of outputs */

	orb_advert_t	_att_sp_pub;			/**< attitude setpoint publication */
	orb_advert_t	_rates_sp_pub;			/**< rate setpoint publication */
	orb_advert_t	_actuators_0_pub;		/**< actuator control group 0 setpoint */

	struct vehicle_attitude_s			_att;			/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s	_att_sp;		/**< vehicle attitude setpoint */
	struct manual_control_setpoint_s	_manual;		/**< r/c channel data */
	struct vehicle_control_mode_s		_control_mode;	/**< vehicle control mode */
	struct actuator_controls_s			_actuators;		/**< actuator control inputs */
	struct actuator_armed_s				_arming;		/**< actuator arming status */
	struct vehicle_rates_setpoint_s		_rates_sp;		/**< vehicle rates setpoint */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	math::Matrix<3, 3> _K;					/**< diagonal gain matrix for position error */
	math::Matrix<3, 3> _K_rate_p;				/**< diagonal gain matrix for angular rate error */
	math::Matrix<3, 3> _K_rate_d;				/**< diagonal gain matrix for angular rate derivative */

	math::Vector<3> _rates_prev;			/**< angular rates on previous step */

	struct {
		param_t att_p;
		param_t att_rate_p;
		param_t att_rate_d;
		param_t yaw_p;
		param_t yaw_rate_p;
		param_t yaw_rate_d;
	}		_parameter_handles;		/**< handles for interesting parameters */

	/**
	 * Update our local parameter cache.
	 */
	int			parameters_update();

	/**
	 * Update control outputs
	 */
	void		control_update();

	/**
	 * Check for changes in vehicle control mode.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in manual inputs.
	 */
	void		vehicle_manual_poll();

	/**
	 * Check for set triplet updates.
	 */
	void		vehicle_setpoint_poll();

	/**
	 * Check for arming status updates.
	 */
	void		arming_status_poll();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main() __attribute__((noreturn));
};

namespace att_control
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

MulticopterAttitudeControl	*g_control;
}

MulticopterAttitudeControl::MulticopterAttitudeControl() :

	_task_should_exit(false),
	_control_task(-1),

/* subscriptions */
	_att_sub(-1),
	_att_sp_sub(-1),
	_control_mode_sub(-1),
	_params_sub(-1),
	_manual_sub(-1),
	_arming_sub(-1),

/* publications */
	_att_sp_pub(-1),
	_rates_sp_pub(-1),
	_actuators_0_pub(-1),

/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "fw att control"))

{
	memset(&_att, 0, sizeof(_att));
	memset(&_att_sp, 0, sizeof(_att_sp));
	memset(&_manual, 0, sizeof(_manual));
	memset(&_control_mode, 0, sizeof(_control_mode));
	memset(&_arming, 0, sizeof(_arming));

	_K.zero();
	_K_rate_p.zero();
	_K_rate_d.zero();

	_rates_prev.zero();

	_parameter_handles.att_p		= 	param_find("MC_ATT_P");
	_parameter_handles.att_rate_p	= 	param_find("MC_ATTRATE_P");
	_parameter_handles.att_rate_d	= 	param_find("MC_ATTRATE_D");
	_parameter_handles.yaw_p		=	param_find("MC_YAWPOS_P");
	_parameter_handles.yaw_rate_p	= 	param_find("MC_YAWRATE_P");
	_parameter_handles.yaw_rate_d	= 	param_find("MC_YAWRATE_D");

	/* fetch initial parameter values */
	parameters_update();
}

MulticopterAttitudeControl::~MulticopterAttitudeControl()
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
				task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	att_control::g_control = nullptr;
}

int
MulticopterAttitudeControl::parameters_update()
{
	float att_p;
	float att_rate_p;
	float att_rate_d;
	float yaw_p;
	float yaw_rate_p;
	float yaw_rate_d;

	param_get(_parameter_handles.att_p, &att_p);
	param_get(_parameter_handles.att_rate_p, &att_rate_p);
	param_get(_parameter_handles.att_rate_d, &att_rate_d);
	param_get(_parameter_handles.yaw_p, &yaw_p);
	param_get(_parameter_handles.yaw_rate_p, &yaw_rate_p);
	param_get(_parameter_handles.yaw_rate_d, &yaw_rate_d);

	_K(0, 0) = att_p;
	_K(1, 1) = att_p;
	_K(2, 2) = yaw_p;

	_K_rate_p(0, 0) = att_rate_p;
	_K_rate_p(1, 1) = att_rate_p;
	_K_rate_p(2, 2) = yaw_rate_p;

	_K_rate_d(0, 0) = att_rate_d;
	_K_rate_d(1, 1) = att_rate_d;
	_K_rate_d(2, 2) = yaw_rate_d;

	return OK;
}

void
MulticopterAttitudeControl::vehicle_control_mode_poll()
{
	bool control_mode_updated;

	/* Check HIL state if vehicle status has changed */
	orb_check(_control_mode_sub, &control_mode_updated);

	if (control_mode_updated) {

		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
	}
}

void
MulticopterAttitudeControl::vehicle_manual_poll()
{
	bool manual_updated;

	/* get pilots inputs */
	orb_check(_manual_sub, &manual_updated);

	if (manual_updated) {

		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}
}

void
MulticopterAttitudeControl::vehicle_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool att_sp_updated;
	orb_check(_att_sp_sub, &att_sp_updated);

	if (att_sp_updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
	}
}

void
MulticopterAttitudeControl::arming_status_poll()
{
	/* check if there is a new setpoint */
	bool arming_updated;
	orb_check(_arming_sub, &arming_updated);

	if (arming_updated) {
		orb_copy(ORB_ID(actuator_armed), _arming_sub, &_arming);
	}
}

void
MulticopterAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
	att_control::g_control->task_main();
}

void
MulticopterAttitudeControl::task_main()
{
	/* inform about start */
	warnx("started");
	fflush(stdout);

	/*
	 * do subscriptions
	 */
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_arming_sub = orb_subscribe(ORB_ID(actuator_armed));

	/* rate limit attitude updates to 100Hz */
	orb_set_interval(_att_sub, 10);

	parameters_update();

	/* initialize values of critical structs until first regular update */
	_arming.armed = false;

	/* get an initial update for all sensor and status data */
	vehicle_setpoint_poll();
	vehicle_control_mode_poll();
	vehicle_manual_poll();
	arming_status_poll();

	/* setpoint rotation matrix */
	math::Matrix<3, 3> R_sp;
	R_sp.identity();

	/* rotation matrix for current state */
	math::Matrix<3, 3> R;
	R.identity();

	/* current angular rates */
	math::Vector<3> rates;
	rates.zero();

	/* identity matrix */
	math::Matrix<3, 3> I;
	I.identity();

	math::Quaternion q;

	bool reset_yaw_sp = true;

	/* wakeup source(s) */
	struct pollfd fds[2];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _att_sub;
	fds[1].events = POLLIN;

	while (!_task_should_exit) {

		/* wait for up to 500ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0)
			continue;

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		perf_begin(_loop_perf);

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* copy the topic to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			parameters_update();
		}

		/* only run controller if attitude changed */
		if (fds[1].revents & POLLIN) {
			static uint64_t last_run = 0;
			float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too large dt's */
			if (dt > 0.02f)
				dt = 0.02f;

			/* copy attitude topic */
			orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);

			vehicle_setpoint_poll();
			vehicle_control_mode_poll();
			arming_status_poll();
			vehicle_manual_poll();

			float yaw_sp_move_rate = 0.0f;
			bool publish_att_sp = false;

			/* define which input is the dominating control input */
			if (_control_mode.flag_control_manual_enabled) {
				/* manual input */
				if (!_control_mode.flag_control_climb_rate_enabled) {
					/* pass throttle directly if not in altitude control mode */
					_att_sp.thrust = _manual.throttle;
				}

				if (!_arming.armed) {
					/* reset yaw setpoint when disarmed */
					reset_yaw_sp = true;
				}

				if (_control_mode.flag_control_attitude_enabled) {
					/* control attitude, update attitude setpoint depending on mode */

					if (_att_sp.thrust < 0.1f) {
						// TODO
						//if (_status.condition_landed) {
						/* reset yaw setpoint if on ground */
						//	reset_yaw_sp = true;
						//}
					} else {
						if (_manual.yaw < -YAW_DEADZONE || YAW_DEADZONE < _manual.yaw) {
							/* move yaw setpoint */
							yaw_sp_move_rate = _manual.yaw;
							_att_sp.yaw_body = _wrap_pi(_att_sp.yaw_body + yaw_sp_move_rate * dt);
							_att_sp.R_valid = false;
							publish_att_sp = true;
						}
					}

					/* reset yaw setpint to current position if needed */
					if (reset_yaw_sp) {
						reset_yaw_sp = false;
						_att_sp.yaw_body = _att.yaw;
						_att_sp.R_valid = false;
						publish_att_sp = true;
					}

					if (!_control_mode.flag_control_velocity_enabled) {
						/* update attitude setpoint if not in position control mode */
						_att_sp.roll_body = _manual.roll;
						_att_sp.pitch_body = _manual.pitch;
						_att_sp.R_valid = false;
						publish_att_sp = true;
					}

				} else {
					/* manual rate inputs (ACRO) */
					// TODO
					/* reset yaw setpoint after ACRO */
					reset_yaw_sp = true;
				}

			} else {
				/* reset yaw setpoint after non-manual control */
				reset_yaw_sp = true;
			}

			if (_att_sp.R_valid) {
				/* rotation matrix in _att_sp is valid, use it */
				R_sp.set(&_att_sp.R_body[0][0]);

			} else {
				/* rotation matrix in _att_sp is not valid, use euler angles instead */
				R_sp.from_euler(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body);

				/* copy rotation matrix back to setpoint struct */
				memcpy(&_att_sp.R_body[0][0], &R_sp.data[0][0], sizeof(_att_sp.R_body));
				_att_sp.R_valid = true;
			}

			if (publish_att_sp) {
				/* publish the attitude setpoint */
				_att_sp.timestamp = hrt_absolute_time();

				if (_att_sp_pub > 0) {
					orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &_att_sp);

				} else {
					_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_att_sp);
				}
			}

			/* rotation matrix for current state */
			R.set(_att.R);

			/* current body angular rates */
			rates(0) = _att.rollspeed;
			rates(1) = _att.pitchspeed;
			rates(2) = _att.yawspeed;

			/* try to move thrust vector shortest way, because yaw response is slower than roll/pitch */
			math::Vector<3> R_z(R(0, 2), R(1, 2), R(2, 2));
			math::Vector<3> R_sp_z(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));

			/* axis and sin(angle) of desired rotation */
			math::Vector<3> e_R = R.transposed() * (R_z % R_sp_z);

			/* calculate angle error */
			float e_R_z_sin = e_R.length();
			float e_R_z_cos = R_z * R_sp_z;

			/* calculate weight for yaw control */
			float yaw_w = R_sp(2, 2) * R_sp(2, 2);

			/* calculate rotation matrix after roll/pitch only rotation */
			math::Matrix<3, 3> R_rp;

			if (e_R_z_sin > 0.0f) {
				/* get axis-angle representation */
				float e_R_z_angle = atan2f(e_R_z_sin, e_R_z_cos);
				math::Vector<3> e_R_z_axis = e_R / e_R_z_sin;

				e_R = e_R_z_axis * e_R_z_angle;

				/* cross product matrix for e_R_axis */
				math::Matrix<3, 3> e_R_cp;
				e_R_cp.zero();
				e_R_cp(0, 1) = -e_R_z_axis(2);
				e_R_cp(0, 2) = e_R_z_axis(1);
				e_R_cp(1, 0) = e_R_z_axis(2);
				e_R_cp(1, 2) = -e_R_z_axis(0);
				e_R_cp(2, 0) = -e_R_z_axis(1);
				e_R_cp(2, 1) = e_R_z_axis(0);

				/* rotation matrix for roll/pitch only rotation */
				R_rp = R * (I + e_R_cp * e_R_z_sin + e_R_cp * e_R_cp * (1.0f - e_R_z_cos));

			} else {
				/* zero roll/pitch rotation */
				R_rp = R;
			}

			/* R_rp and R_sp has the same Z axis, calculate yaw error */
			math::Vector<3> R_sp_x(R_sp(0, 0), R_sp(1, 0), R_sp(2, 0));
			math::Vector<3> R_rp_x(R_rp(0, 0), R_rp(1, 0), R_rp(2, 0));
			e_R(2) = atan2f((R_rp_x % R_sp_x) * R_sp_z, R_rp_x * R_sp_x) * yaw_w;

			if (e_R_z_cos < 0.0f) {
				/* for large thrust vector rotations use another rotation method:
				 * calculate angle and axis for R -> R_sp rotation directly */
				q.from_dcm(R.transposed() * R_sp);
				math::Vector<3> e_R_d = q.imag();
				e_R_d.normalize();
				e_R_d *= 2.0f * atan2f(e_R_d.length(), q(0));

				/* use fusion of Z axis based rotation and direct rotation */
				float direct_w = e_R_z_cos * e_R_z_cos * yaw_w;
				e_R = e_R * (1.0f - direct_w) + e_R_d * direct_w;
			}

			/* angular rates setpoint*/
			math::Vector<3> rates_sp = _K * e_R;

			/* feed forward yaw setpoint rate */
			rates_sp(2) += yaw_sp_move_rate * yaw_w;
			math::Vector<3> control = _K_rate_p * (rates_sp - rates) + _K_rate_d * (_rates_prev - rates) / fmaxf(dt, 0.003f);
			_rates_prev = rates;

			/* publish the attitude rates setpoint */
			_rates_sp.roll = rates_sp(0);
			_rates_sp.pitch = rates_sp(1);
			_rates_sp.yaw = rates_sp(2);
			_rates_sp.thrust = _att_sp.thrust;
			_rates_sp.timestamp = hrt_absolute_time();

			if (_rates_sp_pub > 0) {
				orb_publish(ORB_ID(vehicle_rates_setpoint), _rates_sp_pub, &_rates_sp);

			} else {
				_rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &_rates_sp);
			}

			/* publish the attitude controls */
			if (_control_mode.flag_control_rates_enabled) {
				_actuators.control[0] = (isfinite(control(0))) ? control(0) : 0.0f;
				_actuators.control[1] = (isfinite(control(1))) ? control(1) : 0.0f;
				_actuators.control[2] = (isfinite(control(2))) ? control(2) : 0.0f;
				_actuators.control[3] = (isfinite(_rates_sp.thrust)) ? _rates_sp.thrust : 0.0f;
				_actuators.timestamp = hrt_absolute_time();
			} else {
				/* controller disabled, publish zero attitude controls */
				_actuators.control[0] = 0.0f;
				_actuators.control[1] = 0.0f;
				_actuators.control[2] = 0.0f;
				_actuators.control[3] = 0.0f;
				_actuators.timestamp = hrt_absolute_time();
			}

			if (_actuators_0_pub > 0) {
				/* publish the attitude setpoint */
				orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);

			} else {
				/* advertise and publish */
				_actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);
			}
		}

		perf_end(_loop_perf);
	}

	warnx("exit");

	_control_task = -1;
	_exit(0);
}

int
MulticopterAttitudeControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = task_spawn_cmd("mc_att_control_vector",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       2048,
				       (main_t)&MulticopterAttitudeControl::task_main_trampoline,
				       nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int mc_att_control_main(int argc, char *argv[])
{
	if (argc < 1)
		errx(1, "usage: mc_att_control_vector {start|stop|status}");

	if (!strcmp(argv[1], "start")) {

		if (att_control::g_control != nullptr)
			errx(1, "already running");

		att_control::g_control = new MulticopterAttitudeControl;

		if (att_control::g_control == nullptr)
			errx(1, "alloc failed");

		if (OK != att_control::g_control->start()) {
			delete att_control::g_control;
			att_control::g_control = nullptr;
			err(1, "start failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (att_control::g_control == nullptr)
			errx(1, "not running");

		delete att_control::g_control;
		att_control::g_control = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (att_control::g_control) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}
