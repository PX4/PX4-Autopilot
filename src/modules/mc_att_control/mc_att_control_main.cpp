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
 * @file mc_att_control_main.cpp
 * Multicopter attitude controller.
 *
 * @author Tobias Naegeli <naegelit@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 * @author Roman Bapst <bapstr@ethz.ch>
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

#include <px4.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>

#include "mc_att_control_base.h"

static bool thread_running = false;     /**< Deamon status flag */
static int daemon_task;             /**< Handle of deamon task / thread */

using namespace px4;

/**
 * Multicopter attitude control app start / stop handling function
 *
 * @ingroup apps
 */

PX4_MAIN_FUNCTION(mc_att_control);

int mc_attitude_thread_main(int argc, char *argv[]);

#define YAW_DEADZONE	0.05f
#define MIN_TAKEOFF_THRUST    0.2f
#define RATES_I_LIMIT	0.3f

void handle_vehicle_attitude2(const PX4_TOPIC_T(rc_channels) &msg) {
		PX4_INFO("RCHandler class heard: [%llu]", msg.timestamp);
}


namespace px4
{
bool task_should_exit = false;
}

// PX4_MAIN_FUNCTION(mc_att_control) { px4::init(argc, argv, "listener");

	// px4::NodeHandle n;

	// PX4_SUBSCRIBE(n, rc_channels, handle_vehicle_attitude2, 1000);

	/**
	 * px4::spin() will enter a loop, pumping callbacks.  With this version, all
	 * callbacks will be called from within this thread (the main one).  px4::spin()
	 * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	 */
	// n.spin();
	// PX4_INFO("finished, returning");

	// return 0;
// }

class MulticopterAttitudeControl :
	public MulticopterAttitudeControlBase
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

	void  handle_vehicle_attitude(const PX4_TOPIC_T(vehicle_attitude) &msg);

	void spin() { n.spin(); }

private:
	bool		_task_should_exit;			/**< if true, sensor task should exit */
	int		_control_task;				/**< task handle for sensor task */
	bool		_actuators_0_circuit_breaker_enabled;	/**< circuit breaker to suppress output */


	int		_v_att_sub;				/**< vehicle attitude subscription */
	int		_v_att_sp_sub;			/**< vehicle attitude setpoint subscription */
	int		_v_rates_sp_sub;		/**< vehicle rates setpoint subscription */
	int		_v_control_mode_sub;	/**< vehicle control mode subscription */
	int		_params_sub;			/**< parameter updates subscription */
	int		_manual_control_sp_sub;	/**< manual control setpoint subscription */
	int		_armed_sub;				/**< arming status subscription */

	orb_advert_t	_att_sp_pub;			/**< attitude setpoint publication */
	orb_advert_t	_v_rates_sp_pub;		/**< rate setpoint publication */
	orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */

	px4::NodeHandle n;

	struct {
		param_t roll_p;
		param_t roll_rate_p;
		param_t roll_rate_i;
		param_t roll_rate_d;
		param_t pitch_p;
		param_t pitch_rate_p;
		param_t pitch_rate_i;
		param_t pitch_rate_d;
		param_t yaw_p;
		param_t yaw_rate_p;
		param_t yaw_rate_i;
		param_t yaw_rate_d;
		param_t yaw_ff;
		param_t yaw_rate_max;

		param_t man_roll_max;
		param_t man_pitch_max;
		param_t man_yaw_max;
		param_t acro_roll_max;
		param_t acro_pitch_max;
		param_t acro_yaw_max;
	}		_params_handles;		/**< handles for interesting parameters */

	perf_counter_t _loop_perf; /**< loop performance counter */

	/**
	 * Update our local parameter cache.
	 */
	int			parameters_update();

	/**
	 * Check for parameter update and handle it.
	 */
	void		parameter_update_poll();

	/**
	 * Check for changes in vehicle control mode.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in manual inputs.
	 */
	void		vehicle_manual_poll();

	/**
	 * Check for attitude setpoint updates.
	 */
	void		vehicle_attitude_setpoint_poll();

	/**
	 * Check for rates setpoint updates.
	 */
	void		vehicle_rates_setpoint_poll();

	/**
	 * Check for arming status updates.
	 */
	void		arming_status_poll();

};

namespace mc_att_control
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

}

MulticopterAttitudeControl::MulticopterAttitudeControl() :
	MulticopterAttitudeControlBase(),
	_task_should_exit(false),
	_control_task(-1),
	_actuators_0_circuit_breaker_enabled(false),
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
	n(),

	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "mc_att_control"))

{
	_params_handles.roll_p			= 	param_find("MC_ROLL_P");
	_params_handles.roll_rate_p		= 	param_find("MC_ROLLRATE_P");
	_params_handles.roll_rate_i		= 	param_find("MC_ROLLRATE_I");
	_params_handles.roll_rate_d		= 	param_find("MC_ROLLRATE_D");
	_params_handles.pitch_p			= 	param_find("MC_PITCH_P");
	_params_handles.pitch_rate_p	= 	param_find("MC_PITCHRATE_P");
	_params_handles.pitch_rate_i	= 	param_find("MC_PITCHRATE_I");
	_params_handles.pitch_rate_d	= 	param_find("MC_PITCHRATE_D");
	_params_handles.yaw_p			=	param_find("MC_YAW_P");
	_params_handles.yaw_rate_p		= 	param_find("MC_YAWRATE_P");
	_params_handles.yaw_rate_i		= 	param_find("MC_YAWRATE_I");
	_params_handles.yaw_rate_d		= 	param_find("MC_YAWRATE_D");
	_params_handles.yaw_ff			= 	param_find("MC_YAW_FF");
	_params_handles.yaw_rate_max	= 	param_find("MC_YAWRATE_MAX");
	_params_handles.man_roll_max	= 	param_find("MC_MAN_R_MAX");
	_params_handles.man_pitch_max	= 	param_find("MC_MAN_P_MAX");
	_params_handles.man_yaw_max		= 	param_find("MC_MAN_Y_MAX");
	_params_handles.acro_roll_max	= 	param_find("MC_ACRO_R_MAX");
	_params_handles.acro_pitch_max	= 	param_find("MC_ACRO_P_MAX");
	_params_handles.acro_yaw_max		= 	param_find("MC_ACRO_Y_MAX");

	/* fetch initial parameter values */
	parameters_update();

	/*
	 * do subscriptions
	 */
	// _v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	PX4_SUBSCRIBE(n, vehicle_attitude, MulticopterAttitudeControl::handle_vehicle_attitude, this, 0);
	// _v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	PX4_SUBSCRIBE(n, vehicle_attitude_setpoint, 0);
	// _v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	PX4_SUBSCRIBE(n, vehicle_rates_setpoint, 0);
	// _v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	PX4_SUBSCRIBE(n, vehicle_control_mode, 0);
	// _params_sub = orb_subscribe(ORB_ID(parameter_update));
	PX4_SUBSCRIBE(n, parameter_update, 0);
	// _manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	PX4_SUBSCRIBE(n, manual_control_setpoint, 0);
	// _armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	PX4_SUBSCRIBE(n, actuator_armed, 0);

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

	// mc_att_control::g_control = nullptr;
}

int
MulticopterAttitudeControl::parameters_update()
{
	float v;

	/* roll gains */
	param_get(_params_handles.roll_p, &v);
	_params.att_p(0) = v;
	param_get(_params_handles.roll_rate_p, &v);
	_params.rate_p(0) = v;
	param_get(_params_handles.roll_rate_i, &v);
	_params.rate_i(0) = v;
	param_get(_params_handles.roll_rate_d, &v);
	_params.rate_d(0) = v;

	/* pitch gains */
	param_get(_params_handles.pitch_p, &v);
	_params.att_p(1) = v;
	param_get(_params_handles.pitch_rate_p, &v);
	_params.rate_p(1) = v;
	param_get(_params_handles.pitch_rate_i, &v);
	_params.rate_i(1) = v;
	param_get(_params_handles.pitch_rate_d, &v);
	_params.rate_d(1) = v;

	/* yaw gains */
	param_get(_params_handles.yaw_p, &v);
	_params.att_p(2) = v;
	param_get(_params_handles.yaw_rate_p, &v);
	_params.rate_p(2) = v;
	param_get(_params_handles.yaw_rate_i, &v);
	_params.rate_i(2) = v;
	param_get(_params_handles.yaw_rate_d, &v);
	_params.rate_d(2) = v;

	param_get(_params_handles.yaw_ff, &_params.yaw_ff);
	param_get(_params_handles.yaw_rate_max, &_params.yaw_rate_max);
	_params.yaw_rate_max = math::radians(_params.yaw_rate_max);

	/* manual control scale */
	param_get(_params_handles.man_roll_max, &_params.man_roll_max);
	param_get(_params_handles.man_pitch_max, &_params.man_pitch_max);
	param_get(_params_handles.man_yaw_max, &_params.man_yaw_max);
	_params.man_roll_max = math::radians(_params.man_roll_max);
	_params.man_pitch_max = math::radians(_params.man_pitch_max);
	_params.man_yaw_max = math::radians(_params.man_yaw_max);

	/* acro control scale */
	param_get(_params_handles.acro_roll_max, &v);
	_params.acro_rate_max(0) = math::radians(v);
	param_get(_params_handles.acro_pitch_max, &v);
	_params.acro_rate_max(1) = math::radians(v);
	param_get(_params_handles.acro_yaw_max, &v);
	_params.acro_rate_max(2) = math::radians(v);

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);

	return OK;
}

void
MulticopterAttitudeControl::parameter_update_poll()
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
MulticopterAttitudeControl::vehicle_control_mode_poll()
{
	bool updated;

	/* Check HIL state if vehicle status has changed */
	orb_check(_v_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
	}
}

void
MulticopterAttitudeControl::vehicle_manual_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
	}
}

void
MulticopterAttitudeControl::vehicle_attitude_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
	}
}

void
MulticopterAttitudeControl::vehicle_rates_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_rates_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
	}
}

void
MulticopterAttitudeControl::arming_status_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
	}
}

// void
// MulticopterAttitudeControl::task_main()
// {


	// [> wakeup source: vehicle attitude <]
	// struct pollfd fds[1];

	// fds[0].fd = _v_att_sub;
	// fds[0].events = POLLIN;

	// while (!_task_should_exit) {


		// perf_end(_loop_perf);
	// }

	// warnx("exit");

	// _control_task = -1;
	// _exit(0);
// }

void  MulticopterAttitudeControl::handle_vehicle_attitude(const PX4_TOPIC_T(vehicle_attitude) &msg) {

	perf_begin(_loop_perf);

	/* run controller on attitude changes */
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

	if (_v_control_mode.flag_control_attitude_enabled) {
		control_attitude(dt);

		/* publish the attitude setpoint if needed */
		if (_publish_att_sp) {
			_v_att_sp.timestamp = hrt_absolute_time();

			if (_att_sp_pub > 0) {
				orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub,
					    &_v_att_sp);

			} else {
				_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint),
							    &_v_att_sp);
			}
		}

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
		/* attitude controller disabled, poll rates setpoint topic */
		if (_v_control_mode.flag_control_manual_enabled) {
			/* manual rates control - ACRO mode */
			_rates_sp = math::Vector<3>(_manual_control_sp.y, -_manual_control_sp.x,
						    _manual_control_sp.r).emult(_params.acro_rate_max);
			_thrust_sp = _manual_control_sp.z;

			/* reset yaw setpoint after ACRO */
			_reset_yaw_sp = true;

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
			/* attitude controller disabled, poll rates setpoint topic */
			vehicle_rates_setpoint_poll();
			_rates_sp(0) = _v_rates_sp.roll;
			_rates_sp(1) = _v_rates_sp.pitch;
			_rates_sp(2) = _v_rates_sp.yaw;
			_thrust_sp = _v_rates_sp.thrust;
		}
	}

	if (_v_control_mode.flag_control_rates_enabled) {
		control_attitude_rates(dt);

		/* publish actuator controls */
		_actuators.control[0] = (isfinite(_att_control(0))) ? _att_control(0) : 0.0f;
		_actuators.control[1] = (isfinite(_att_control(1))) ? _att_control(1) : 0.0f;
		_actuators.control[2] = (isfinite(_att_control(2))) ? _att_control(2) : 0.0f;
		_actuators.control[3] = (isfinite(_thrust_sp)) ? _thrust_sp : 0.0f;
		_actuators.timestamp = hrt_absolute_time();

		if (!_actuators_0_circuit_breaker_enabled) {
			if (_actuators_0_pub > 0) {
				orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);

			} else {
				_actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);
			}
		}
	}
}

PX4_MAIN_FUNCTION(mc_att_control)
{
	px4::init(argc, argv, "mc_att_control");

	if (argc < 1) {
		errx(1, "usage: mc_att_control {start|stop|status}");
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("already running");
			/* this is not an error */
			exit(0);
		}

		task_should_exit = false;

		daemon_task = task_spawn_cmd("mc_att_control",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       2000,
				       mc_attitude_thread_main,
					(argv) ? (const char **)&argv[2] : (const char **)NULL);

		exit(0);
	}

	// if (!strcmp(argv[1], "stop")) {
		// if (mc_att_control::g_control == nullptr) {
			// errx(1, "not running");
		// }

		// delete mc_att_control::g_control;
		// mc_att_control::g_control = nullptr;
		// exit(0);
	// }

	// if (!strcmp(argv[1], "status")) {
		// if (mc_att_control::g_control) {
			// errx(0, "running");

		// } else {
			// errx(1, "not running");
		// }
	// }

	warnx("unrecognized command");
	return 1;
}

int mc_attitude_thread_main(int argc, char *argv[])
{

	warnx("starting");

	MulticopterAttitudeControl attctl;

	thread_running = true;

	attctl.spin();

	// while (!task_should_exit) {
		// attctl.update();
	// }

	warnx("exiting.");

	thread_running = false;

	return 0;
}


