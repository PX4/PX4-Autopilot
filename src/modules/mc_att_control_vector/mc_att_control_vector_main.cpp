/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: 	Lorenz Meier
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
 * @file mc_att_control_vector_main.c
 * Implementation of a multicopter attitude controller based on desired thrust vector.
 *
 * @author Tobias Naegeli <naegelit@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 *
 * Please refer to the library files for the authors and acknowledgements of
 * the used control library functions.
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
#include <drivers/drv_accel.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/parameter_update.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/pid/pid.h>
#include <systemlib/geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>

//#include <ecl/attitude_mc/ecl_mc_att_control_vector.h>
#include "ecl_mc_att_control_vector.h"

/**
 * Multicopter attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_att_control_vector_main(int argc, char *argv[]);

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

	bool		_task_should_exit;		/**< if true, sensor task should exit */
	int		_control_task;			/**< task handle for sensor task */

	int		_att_sub;			/**< vehicle attitude subscription */
	int		_accel_sub;			/**< accelerometer subscription */
	int		_att_sp_sub;			/**< vehicle attitude setpoint */
	int		_attitude_sub;			/**< raw rc channels data subscription */
	int		_airspeed_sub;			/**< airspeed subscription */
	int		_vstatus_sub;			/**< vehicle status subscription */
	int 		_params_sub;			/**< notification of parameter updates */
	int 		_manual_sub;			/**< notification of manual control updates */
	int		_arming_sub;			/**< arming status of outputs */

	orb_advert_t	_rate_sp_pub;			/**< rate setpoint publication */
	orb_advert_t	_actuators_0_pub;		/**< actuator control group 0 setpoint */

	struct vehicle_attitude_s			_att;			/**< vehicle attitude */
	struct accel_report				_accel;			/**< body frame accelerations */
	struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct airspeed_s				_airspeed;		/**< airspeed */
	struct vehicle_status_s				_vstatus;		/**< vehicle status */
	struct actuator_controls_s			_actuators;		/**< actuator control inputs */
	struct actuator_armed_s				_arming;		/**< actuator arming status */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	bool		_setpoint_valid;		/**< flag if the position control setpoint is valid */
	bool		_airspeed_valid;		/**< flag if the airspeed measurement is valid */

	// ECL_L1_Pos_Control				_att_control;

	struct {
		float yaw_p;
		float yaw_i;
		float yaw_d;
		float yaw_imax;

		float att_p;
		float att_i;
		float att_d;
		float att_imax;

		float att_rate_p;

		float yaw_rate_p;
	}		_parameters;			/**< local copies of interesting parameters */

	struct {
		param_t yaw_p;
		param_t yaw_i;
		param_t yaw_d;
		param_t yaw_imax;

		param_t att_p;
		param_t att_i;
		param_t att_d;
		param_t att_imax;

		param_t att_rate_p;

		param_t yaw_rate_p;
	}		_parameter_handles;		/**< handles for interesting parameters */


	ECL_MCAttControlVector				_att_control;

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
	 * Check for changes in vehicle status.
	 */
	void		vehicle_status_poll();

	/**
	 * Check for changes in manual inputs.
	 */
	void		vehicle_manual_poll();


	/**
	 * Check for airspeed updates.
	 */
	bool		vehicle_airspeed_poll();

	/**
	 * Check for accel updates.
	 */
	void		vehicle_accel_poll();

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
	_accel_sub(-1),
	_airspeed_sub(-1),
	_vstatus_sub(-1),
	_params_sub(-1),
	_manual_sub(-1),
	_arming_sub(-1),

/* publications */
	_rate_sp_pub(-1),
	_actuators_0_pub(-1),

/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "fw att control")),
/* states */
	_setpoint_valid(false),
	_airspeed_valid(false)
{
	_parameter_handles.yaw_p	=	param_find("MC_YAWPOS_P");
	_parameter_handles.yaw_i	=	param_find("MC_YAWPOS_I");
	_parameter_handles.yaw_d	=	param_find("MC_YAWPOS_D");
	_parameter_handles.yaw_imax	=	param_find("MC_YAWPOS_IMAX");

	_parameter_handles.att_p	= 	param_find("MC_ATT_P");
	_parameter_handles.att_i	= 	param_find("MC_ATT_I");
	_parameter_handles.att_d	= 	param_find("MC_ATT_D");
	_parameter_handles.att_imax	= 	param_find("MC_ATT_IMAX");

	_parameter_handles.att_rate_p	= 	param_find("MC_ATTRATE_P");
	_parameter_handles.yaw_rate_p	= 	param_find("MC_YAWRATE_P");

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

	//param_get(_parameter_handles.tconst, &(_parameters.tconst));

	param_get(_parameter_handles.yaw_p, &(_parameters.yaw_p));
	param_get(_parameter_handles.yaw_i, &(_parameters.yaw_i));
	param_get(_parameter_handles.yaw_d, &(_parameters.yaw_d));
	param_get(_parameter_handles.yaw_imax, &(_parameters.yaw_imax));

	param_get(_parameter_handles.att_p, &(_parameters.att_p));
	param_get(_parameter_handles.att_i, &(_parameters.att_i));
	param_get(_parameter_handles.att_d, &(_parameters.att_d));
	param_get(_parameter_handles.att_imax, &(_parameters.att_imax));

	param_get(_parameter_handles.yaw_rate_p, &(_parameters.yaw_rate_p));

	param_get(_parameter_handles.att_rate_p, &(_parameters.att_rate_p));

	/* class control parameters */
	// _att_ctrl.set_tau(_parameters.p_tconst);
	// _att_ctrl.set_k_p(math::radians(_parameters.p_p));
	// _att_ctrl.set_k_i(math::radians(_parameters.p_i));
	// _att_ctrl.set_k_d(math::radians(_parameters.p_d));

	return OK;
}

void
MulticopterAttitudeControl::vehicle_status_poll()
{
	bool vstatus_updated;

	/* Check HIL state if vehicle status has changed */
	orb_check(_vstatus_sub, &vstatus_updated);

	if (vstatus_updated) {

		orb_copy(ORB_ID(vehicle_status), _vstatus_sub, &_vstatus);
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

bool
MulticopterAttitudeControl::vehicle_airspeed_poll()
{
	/* check if there is a new position */
	bool airspeed_updated;
	orb_check(_airspeed_sub, &airspeed_updated);

	if (airspeed_updated) {
		orb_copy(ORB_ID(airspeed), _airspeed_sub, &_airspeed);
		return true;
	}

	return false;
}

void
MulticopterAttitudeControl::vehicle_accel_poll()
{
	/* check if there is a new position */
	bool accel_updated;
	orb_check(_accel_sub, &accel_updated);

	if (accel_updated) {
		orb_copy(ORB_ID(sensor_accel), _accel_sub, &_accel);
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
		_setpoint_valid = true;
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
	warnx("Initializing..");
	fflush(stdout);

	/*
	 * do subscriptions
	 */
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_accel_sub = orb_subscribe(ORB_ID(sensor_accel));
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	_vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_arming_sub = orb_subscribe(ORB_ID(actuator_armed));

	/* rate limit vehicle status updates to 5Hz */
	orb_set_interval(_vstatus_sub, 200);
	orb_set_interval(_att_sub, 100);

	parameters_update();

	/* initialize values of critical structs until first regular update */
	_arming.armed = false;

	/* get an initial update for all sensor and status data */
	(void)vehicle_airspeed_poll();
	vehicle_setpoint_poll();
	vehicle_accel_poll();
	vehicle_status_poll();
	vehicle_manual_poll();
	arming_status_poll();

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
			if (deltaT > 1.0f)
				deltaT = 0.01f;

			/* load local copies */
			orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);

			_airspeed_valid = vehicle_airspeed_poll();

			vehicle_setpoint_poll();

			vehicle_accel_poll();

			/* check vehicle status for changes to publication state */
			vehicle_status_poll();

			/* check for arming status changes */
			arming_status_poll();

			vehicle_manual_poll();

			/* lock integrator until armed */
			bool lock_integrator;
			if (_arming.armed) {
				lock_integrator = false;
			} else {
				lock_integrator = true;
			}

			/* decide if in auto or full manual control */
			float roll_sp, pitch_sp;
			float throttle_sp = 0.0f;
			float yaw_sp = 0.0f;

			if (_vstatus.state_machine == SYSTEM_STATE_MANUAL ||
				(_vstatus.state_machine == SYSTEM_STATE_STABILIZED)) {

				/*
				 * Scale down roll and pitch as the setpoints are radians
				 * and a typical remote can only do 45 degrees, the mapping is
				 * -1..+1 to -45..+45 degrees or -0.75..+0.75 radians.
				 * 
				 * With this mapping the stick angle is a 1:1 representation of
				 * the commanded attitude. If more than 45 degrees are desired,
				 * a scaling parameter can be applied to the remote.
				 */
				roll_sp = _manual.roll * 0.75f;
				pitch_sp = _manual.pitch * 0.75f;
				yaw_sp = _manual.yaw;
				throttle_sp = _manual.throttle;

			} else if (_vstatus.state_machine == SYSTEM_STATE_AUTO) {

					roll_sp = _att_sp.roll_body;
					pitch_sp = _att_sp.pitch_body;
					yaw_sp = _att_sp.yaw_body;
					throttle_sp = _att_sp.thrust;
			}

			// XXX take rotation matrix directly from att_sp for auto mode
			math::Vector3 F_des(roll_sp, pitch_sp, yaw_sp);

			math::Vector3 rates_des;

			math::Dcm R_nb(_att.R);
			math::Vector3 angular_rates(_att.rollspeed, _att.pitchspeed, _att.yawspeed);

			_att_control.control(deltaT, R_nb, _att.yaw, F_des,
                                _parameters.att_p, _parameters.att_d, _parameters.att_i,
                                angular_rates, rates_des, throttle_sp);

			float roll_out = _parameters.att_rate_p * rates_des(0);
			float pitch_out = _parameters.att_rate_p * rates_des(1);
			float yaw_out = _parameters.yaw_rate_p * rates_des(2);
			
			_actuators.control[0] = (isfinite(roll_out)) ? roll_out : 0.0f;
			_actuators.control[1] = (isfinite(pitch_out)) ? pitch_out : 0.0f;
			_actuators.control[2] = (isfinite(yaw_out)) ? yaw_out : 0.0f;

			/* throttle passed through */
			_actuators.control[3] = (isfinite(throttle_sp)) ? throttle_sp : 0.0f;

			/*
			 * Lazily publish the rate setpoint (for analysis, the actuators are published below)
			 * only once available
			 */
			vehicle_rates_setpoint_s rates_sp;
			rates_sp.roll = rates_des(0);
			rates_sp.pitch = rates_des(1);
			rates_sp.yaw = rates_des(2);

			rates_sp.timestamp = hrt_absolute_time();

			if (_rate_sp_pub > 0) {
				/* publish the attitude setpoint */
				orb_publish(ORB_ID(vehicle_rates_setpoint), _actuators_0_pub, &rates_sp);

			} else {
				/* advertise and publish */
				_rate_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &rates_sp);
			}

			/* lazily publish the setpoint only once available */
			_actuators.timestamp = hrt_absolute_time();

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

	warnx("exiting.\n");

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

int mc_att_control_vector_main(int argc, char *argv[])
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
