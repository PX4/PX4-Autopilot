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

#include "jetdrive_control.h"



JetdriveControl::JetdriveControl() :

	_task_should_exit(false),
	_control_task(-1),

/* subscriptions */
	_v_att_sub(-1),
	_v_att_sp_sub(-1),
	_v_control_mode_sub(-1),
	_params_sub(-1),
	_manual_control_sp_sub(-1),
	_armed_sub(-1),

/* publications */
	_att_sp_pub(-1),
	_v_rates_sp_pub(-1),
	_actuators_0_pub(-1),

/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "fw att control"))

{
	memset(&_v_att, 0, sizeof(_v_att));
	memset(&_v_att_sp, 0, sizeof(_v_att_sp));
	memset(&_manual_control_sp, 0, sizeof(_manual_control_sp));
	memset(&_v_control_mode, 0, sizeof(_v_control_mode));
	memset(&_armed, 0, sizeof(_armed));

	_params.att_p.zero();
	_params.rate_p.zero();
	_params.rate_i.zero();
	_params.rate_d.zero();

	_R_sp.identity();
	_R.identity();
	_rates_prev.zero();
	_rates_sp.zero();
	_rates_int.zero();
	_thrust_sp = 0.0f;
	_att_control.zero();

	I.identity();

	_params_handles.roll_p			= 	param_find("JC_ROLL_P");
	_params_handles.roll_rate_p		= 	param_find("JC_ROLLRATE_P");
	_params_handles.roll_rate_i		= 	param_find("JC_ROLLRATE_I");
	_params_handles.roll_rate_d		= 	param_find("JC_ROLLRATE_D");
	_params_handles.pitch_p			= 	param_find("JC_PITCH_P");
	_params_handles.pitch_rate_p	= 	param_find("JC_PITCHRATE_P");
	_params_handles.pitch_rate_i	= 	param_find("JC_PITCHRATE_I");
	_params_handles.pitch_rate_d	= 	param_find("JC_PITCHRATE_D");
	_params_handles.yaw_p			=	param_find("JC_YAW_P");
	_params_handles.yaw_rate_p		= 	param_find("JC_YAWRATE_P");
	_params_handles.yaw_rate_i		= 	param_find("JC_YAWRATE_I");
	_params_handles.yaw_rate_d		= 	param_find("JC_YAWRATE_D");
	_params_handles.yaw_ff			= 	param_find("JC_YAW_FF");

	_params_handles.rc_scale_yaw	= 	param_find("RC_SCALE_YAW");

	/* fetch initial parameter values */
	parameters_update();
}

JetdriveControl::~JetdriveControl()
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

	jetdrive_control::g_control = nullptr;
}

int
JetdriveControl::parameters_update()
{
	float v;

	/* roll */
	param_get(_params_handles.roll_p, &v);
	_params.att_p(0) = v;
	param_get(_params_handles.roll_rate_p, &v);
	_params.rate_p(0) = v;
	param_get(_params_handles.roll_rate_i, &v);
	_params.rate_i(0) = v;
	param_get(_params_handles.roll_rate_d, &v);
	_params.rate_d(0) = v;

	/* pitch */
	param_get(_params_handles.pitch_p, &v);
	_params.att_p(1) = v;
	param_get(_params_handles.pitch_rate_p, &v);
	_params.rate_p(1) = v;
	param_get(_params_handles.pitch_rate_i, &v);
	_params.rate_i(1) = v;
	param_get(_params_handles.pitch_rate_d, &v);
	_params.rate_d(1) = v;

	/* yaw */
	param_get(_params_handles.yaw_p, &v);
	_params.att_p(2) = v;
	param_get(_params_handles.yaw_rate_p, &v);
	_params.rate_p(2) = v;
	param_get(_params_handles.yaw_rate_i, &v);
	_params.rate_i(2) = v;
	param_get(_params_handles.yaw_rate_d, &v);
	_params.rate_d(2) = v;

	param_get(_params_handles.yaw_ff, &_params.yaw_ff);

	param_get(_params_handles.rc_scale_yaw, &_params.rc_scale_yaw);

	return OK;
}

void
JetdriveControl::parameter_update_poll()
{
	bool updated;

	/* Check HIL state if vehicle status has changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		parameters_update();
	}
}

void
JetdriveControl::vehicle_control_mode_poll()
{
	bool updated;

	/* Check HIL state if vehicle status has changed */
	orb_check(_v_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
	}
}

void
JetdriveControl::vehicle_manual_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {

		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
	}
}

void
JetdriveControl::vehicle_attitude_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
	}
}

void
JetdriveControl::vehicle_rates_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_rates_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
	}
}

void
JetdriveControl::arming_status_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
	}
}

/*
 * Attitude controller.
 * Input: 'manual_control_setpoint' and 'vehicle_attitude_setpoint' topics (depending on mode)
 * Output: '_rates_sp' vector, '_thrust_sp', 'vehicle_attitude_setpoint' topic (for manual modes)
 */
void
JetdriveControl::control_attitude(float dt)
{
	float yaw_sp_move_rate = 0.0f;
	bool publish_att_sp = false;

	if (_v_control_mode.flag_control_manual_enabled) {
		/* manual input, set or modify attitude setpoint */

		if (_v_control_mode.flag_control_velocity_enabled || _v_control_mode.flag_control_climb_rate_enabled) {
			/* in assisted modes poll 'vehicle_attitude_setpoint' topic and modify it */
			vehicle_attitude_setpoint_poll();
		}

		if (!_v_control_mode.flag_control_climb_rate_enabled) {
			/* pass throttle directly if not in altitude stabilized mode */
			_v_att_sp.thrust = _manual_control_sp.throttle;
			publish_att_sp = true;
		}

		if (!_armed.armed) {
			/* reset yaw setpoint when disarmed */
			_reset_yaw_sp = true;
		}

		/* move yaw setpoint in all modes */
		if (_v_att_sp.thrust < 0.1f) {
			// TODO
			//if (_status.condition_landed) {
			/* reset yaw setpoint if on ground */
			//	reset_yaw_sp = true;
			//}
		} else {
			float yaw_dz_scaled = YAW_DEADZONE * _params.rc_scale_yaw;

			if (_params.rc_scale_yaw > 0.001f && fabs(_manual_control_sp.yaw) > yaw_dz_scaled) {
				/* move yaw setpoint */
				yaw_sp_move_rate = _manual_control_sp.yaw / _params.rc_scale_yaw;

				if (_manual_control_sp.yaw > 0.0f) {
					yaw_sp_move_rate -= YAW_DEADZONE;

				} else {
					yaw_sp_move_rate += YAW_DEADZONE;
				}

				yaw_sp_move_rate *= _params.rc_scale_yaw;
				_v_att_sp.yaw_body = _wrap_pi(_v_att_sp.yaw_body + yaw_sp_move_rate * dt);
				_v_att_sp.R_valid = false;
				publish_att_sp = true;
			}
		}

		/* reset yaw setpint to current position if needed */
		if (_reset_yaw_sp) {
			_reset_yaw_sp = false;
			_v_att_sp.yaw_body = _v_att.yaw;
			_v_att_sp.R_valid = false;
			publish_att_sp = true;
		}

		if (!_v_control_mode.flag_control_velocity_enabled) {
			/* update attitude setpoint if not in position control mode */
			_v_att_sp.roll_body = _manual_control_sp.roll;
			_v_att_sp.pitch_body = _manual_control_sp.pitch;
			_v_att_sp.R_valid = false;
			publish_att_sp = true;
		}

	} else {
		/* in non-manual mode use 'vehicle_attitude_setpoint' topic */
		vehicle_attitude_setpoint_poll();

		/* reset yaw setpoint after non-manual control mode */
		_reset_yaw_sp = true;
	}

	_thrust_sp = _v_att_sp.thrust;

	/* construct attitude setpoint rotation matrix */
	if (_v_att_sp.R_valid) {
		/* rotation matrix in _att_sp is valid, use it */
		_R_sp.set(&_v_att_sp.R_body[0][0]);

	} else {
		/* rotation matrix in _att_sp is not valid, use euler angles instead */
		_R_sp.from_euler(_v_att_sp.roll_body, _v_att_sp.pitch_body, _v_att_sp.yaw_body);

		/* copy rotation matrix back to setpoint struct */
		memcpy(&_v_att_sp.R_body[0][0], &_R_sp.data[0][0], sizeof(_v_att_sp.R_body));
		_v_att_sp.R_valid = true;
	}

	/* publish the attitude setpoint if needed */
	if (publish_att_sp) {
		_v_att_sp.timestamp = hrt_absolute_time();

		if (_att_sp_pub > 0) {
			orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &_v_att_sp);

		} else {
			_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_v_att_sp);
		}
	}

	/* rotation matrix for current state */
	_R.set(_v_att.R);

	/* all input data is ready, run controller itself */

	/* try to move thrust vector shortest way, because yaw response is slower than roll/pitch */
	math::Vector<3> R_z(_R(0, 2), _R(1, 2), _R(2, 2));
	math::Vector<3> R_sp_z(_R_sp(0, 2), _R_sp(1, 2), _R_sp(2, 2));

	/* axis and sin(angle) of desired rotation */
	math::Vector<3> e_R = _R.transposed() * (R_z % R_sp_z);

	/* calculate angle error */
	float e_R_z_sin = e_R.length();
	float e_R_z_cos = R_z * R_sp_z;

	/* calculate weight for yaw control */
	float yaw_w = _R_sp(2, 2) * _R_sp(2, 2);

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
		R_rp = _R * (I + e_R_cp * e_R_z_sin + e_R_cp * e_R_cp * (1.0f - e_R_z_cos));

	} else {
		/* zero roll/pitch rotation */
		R_rp = _R;
	}

	/* R_rp and _R_sp has the same Z axis, calculate yaw error */
	math::Vector<3> R_sp_x(_R_sp(0, 0), _R_sp(1, 0), _R_sp(2, 0));
	math::Vector<3> R_rp_x(R_rp(0, 0), R_rp(1, 0), R_rp(2, 0));
	e_R(2) = atan2f((R_rp_x % R_sp_x) * R_sp_z, R_rp_x * R_sp_x) * yaw_w;

	if (e_R_z_cos < 0.0f) {
		/* for large thrust vector rotations use another rotation method:
		 * calculate angle and axis for R -> R_sp rotation directly */
		math::Quaternion q;
		q.from_dcm(_R.transposed() * _R_sp);
		math::Vector<3> e_R_d = q.imag();
		e_R_d.normalize();
		e_R_d *= 2.0f * atan2f(e_R_d.length(), q(0));

		/* use fusion of Z axis based rotation and direct rotation */
		float direct_w = e_R_z_cos * e_R_z_cos * yaw_w;
		e_R = e_R * (1.0f - direct_w) + e_R_d * direct_w;
	}

	/* calculate angular rates setpoint */
	_rates_sp = _params.att_p.emult(e_R);

	/* feed forward yaw setpoint rate */
	_rates_sp(2) += yaw_sp_move_rate * yaw_w * _params.yaw_ff;
}

/*
 * Attitude rates controller.
 * Input: '_rates_sp' vector, '_thrust_sp'
 * Output: '_att_control' vector
 */
void
JetdriveControl::control_attitude_rates(float dt)
{
	/* reset integral if disarmed */
	if (!_armed.armed) {
		_rates_int.zero();
	}

	/* current body angular rates */
	math::Vector<3> rates;
	rates(0) = _v_att.rollspeed;
	rates(1) = _v_att.pitchspeed;
	rates(2) = _v_att.yawspeed;

	/* angular rates error */
	math::Vector<3> rates_err = _rates_sp - rates;

	// _att_control will be loaded to controls
	//
	// _att_control = (JC_XXXRATE_P * rates_err) + (JC_XXXRATE_D * (_rates_prev - rate) / dt) + _rates_int;

	_att_control = _params.rate_p.emult(rates_err) + _params.rate_d.emult(_rates_prev - rates) / dt + _rates_int;
	_rates_prev = rates;

	/* update integral only if not saturated on low limit */
	if (_thrust_sp > 0.1f) {
		for (int i = 0; i < 3; i++) {
			if (fabsf(_att_control(i)) < _thrust_sp) {
				float rate_i = _rates_int(i) + _params.rate_i(i) * rates_err(i) * dt;

				if (isfinite(rate_i) && rate_i > -RATES_I_LIMIT && rate_i < RATES_I_LIMIT &&
				    _att_control(i) > -RATES_I_LIMIT && _att_control(i) < RATES_I_LIMIT) {
					_rates_int(i) = rate_i;
				}
			}
		}
	}
}

void
JetdriveControl::task_main_trampoline(int argc, char *argv[])
{
	jetdrive_control::g_control->task_main();
}

void
JetdriveControl::task_main()
{
	warnx("started");
	fflush(stdout);

	/*
	 * do subscriptions
	 */
	_v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	_v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));

	/* rate limit attitude updates to 200Hz, failsafe against spam, normally runs at the same rate as attitude estimator */
	orb_set_interval(_v_att_sub, 5);

	/* initialize parameters cache */
	parameters_update();

	/* wakeup source: vehicle attitude */
	struct pollfd fds[1];

	fds[0].fd = _v_att_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {

		/* wait for up to 100ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0)
			continue;

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		perf_begin(_loop_perf);

		/* run controller on attitude changes */
		if (fds[0].revents & POLLIN) {
			static uint64_t last_run = 0;
			float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too small (< 2ms) and too large (> 20ms) dt's */
			if (dt < 0.002f) {
				dt = 0.002f;

			} else if (dt > 0.02f) {
				dt = 0.02f;
			}

			/* copy attitude topic */
			orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);

			/* check for updates in other topics */
			parameter_update_poll();
			vehicle_control_mode_poll();
			arming_status_poll();
			vehicle_manual_poll();

			if(!_v_control_mode.flag_control_offboard_enabled)
			{
				/* manual/direct control */
				_actuators.control[0] = _manual_control_sp .roll;
				_actuators.control[1] = _manual_control_sp.pitch;
				_actuators.control[2] = _manual_control_sp.yaw;
				_actuators.control[3] = _manual_control_sp.throttle;
				_actuators.timestamp = hrt_absolute_time();

				if (_actuators_0_pub > 0) {
					orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);

				} else {
					_actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);
				}
			} else {
				// FIX this: This is the atitude control logic for mc's

				if (_v_control_mode.flag_control_attitude_enabled) {
					control_attitude(dt);

					/* publish attitude rates setpoint */
					_v_rates_sp.roll = _rates_sp(0);
					_v_rates_sp.pitch = _rates_sp(1);
					_v_rates_sp.yaw = _rates_sp(2);
					_v_rates_sp.thrust = _thrust_sp;
					_v_rates_sp.timestamp = hrt_absolute_time();

					if (_v_rates_sp_pub > 0) {
						orb_publish(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_pub, &_v_rates_sp);

					} else {
						_v_rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &_v_rates_sp);
					}

				} else {
					/* attitude controller disabled */
					// TODO poll 'attitude_rates_setpoint' topic
					vehicle_rates_setpoint_poll();
					_rates_sp(0) = _v_rates_sp.roll;
					_rates_sp(1) = _v_rates_sp.pitch;
					_rates_sp(2) = _v_rates_sp.yaw;
					_thrust_sp = _v_rates_sp.thrust;					
//					_rates_sp.zero();
//					_thrust_sp = 0.0f;
				}

				if (_v_control_mode.flag_control_rates_enabled) {
					control_attitude_rates(dt);

					/* publish actuator controls */
					_actuators.control[0] = (isfinite(_att_control(0))) ? _att_control(0) : 0.0f;
					_actuators.control[1] = (isfinite(_att_control(1))) ? _att_control(1) : 0.0f;
					_actuators.control[2] = (isfinite(_att_control(2))) ? _att_control(2) : 0.0f;
					_actuators.control[3] = (isfinite(_thrust_sp)) ? _thrust_sp : 0.0f;
					_actuators.timestamp = hrt_absolute_time();

					if (_actuators_0_pub > 0) {
						orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);

					} else {
						_actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);
					}
				}
			}
		}

		perf_end(_loop_perf);
	}

	warnx("exit");

	_control_task = -1;
	_exit(0);
}

int
JetdriveControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = task_spawn_cmd("jetdrive_control",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       2048,
					   (main_t)&JetdriveControl::task_main_trampoline,
				       nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int jetdrive_control_main(int argc, char *argv[])
{
	if (argc < 1)
		errx(1, "usage: jetdrive_control {start|stop|status}");

	if (!strcmp(argv[1], "start")) {

		if (jetdrive_control::g_control != nullptr)
			errx(1, "already running");

		jetdrive_control::g_control = new JetdriveControl;

		if (jetdrive_control::g_control == nullptr)
			errx(1, "alloc failed");

		if (OK != jetdrive_control::g_control->start()) {
			delete jetdrive_control::g_control;
			jetdrive_control::g_control = nullptr;
			err(1, "start failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (jetdrive_control::g_control == nullptr)
			errx(1, "not running");

		delete jetdrive_control::g_control;
		jetdrive_control::g_control = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (jetdrive_control::g_control) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}
