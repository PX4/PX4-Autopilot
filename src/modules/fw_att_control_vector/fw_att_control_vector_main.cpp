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
 * @file fw_att_control_vector_main.c
 * Implementation of a generic attitude controller based on classic orthogonal PIDs.
 *
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

#include "ecl_fw_att_control_vector.h"

/**
 * Fixedwing attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int fw_att_control_vector_main(int argc, char *argv[]);

class FixedwingAttitudeControlVector
{
public:
	/**
	 * Constructor
	 */
	FixedwingAttitudeControlVector();

	/**
	 * Destructor, also kills the sensors task.
	 */
	~FixedwingAttitudeControlVector();

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
	int 		_manual_control_sub;		/**< notification of manual control updates */
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

	struct {

		float p_tconst;
		float p_p;
		float p_d;
		float p_i;
		float p_rmax_up;
		float p_rmax_dn;
		float p_imax;
		float p_rll;
		float r_tconst;
		float r_p;
		float r_d;
		float r_i;
		float r_imax;
		float r_rmax;
		float y_slip;
		float y_int;
		float y_damp;
		float y_rll;
		float y_imax;

		float airspeed_min;
		float airspeed_trim;
		float airspeed_max;
	}		_parameters;			/**< local copies of interesting parameters */

	struct {

		param_t p_tconst;
		param_t p_p;
		param_t p_d;
		param_t p_i;
		param_t p_rmax_up;
		param_t p_rmax_dn;
		param_t p_imax;
		param_t p_rll;
		param_t r_tconst;
		param_t r_p;
		param_t r_d;
		param_t r_i;
		param_t r_imax;
		param_t r_rmax;
		param_t y_slip;
		param_t y_int;
		param_t y_damp;
		param_t y_rll;
		param_t y_imax;

		param_t airspeed_min;
		param_t airspeed_trim;
		param_t airspeed_max;
	}		_parameter_handles;		/**< handles for interesting parameters */


	ECL_FWAttControlVector	_att_control;


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

FixedwingAttitudeControlVector	*g_control;
}

FixedwingAttitudeControlVector::FixedwingAttitudeControlVector() :

	_task_should_exit(false),
	_control_task(-1),

/* subscriptions */
	_att_sub(-1),
	_accel_sub(-1),
	_airspeed_sub(-1),
	_vstatus_sub(-1),
	_params_sub(-1),
	_manual_control_sub(-1),
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
	// _parameter_handles.roll_p = param_find("FW_ROLL_P");
	// _parameter_handles.pitch_p = param_find("FW_PITCH_P");
	_parameter_handles.p_tconst = param_find("FW_P_TCONST");
	_parameter_handles.p_p = param_find("FW_P_P");
	_parameter_handles.p_d = param_find("FW_P_D");
	_parameter_handles.p_i = param_find("FW_P_I");
	_parameter_handles.p_rmax_up = param_find("FW_P_RMAX_UP");
	_parameter_handles.p_rmax_dn = param_find("FW_P_RMAX_DN");
	_parameter_handles.p_imax = param_find("FW_P_IMAX");
	_parameter_handles.p_rll = param_find("FW_P_RLL");

	_parameter_handles.r_tconst = param_find("FW_R_TCONST");
	_parameter_handles.r_p = param_find("FW_R_P");
	_parameter_handles.r_d = param_find("FW_R_D");
	_parameter_handles.r_i = param_find("FW_R_I");
	_parameter_handles.r_imax = param_find("FW_R_IMAX");
	_parameter_handles.r_rmax = param_find("FW_R_RMAX");

	_parameter_handles.y_slip = param_find("FW_Y_SLIP");
	_parameter_handles.y_int = param_find("FW_Y_INT");
	_parameter_handles.y_damp = param_find("FW_Y_DAMP");
	_parameter_handles.y_rll = param_find("FW_Y_RLL");
	_parameter_handles.y_imax = param_find("FW_Y_IMAX");

	_parameter_handles.airspeed_min = param_find("FW_AIRSPD_MIN");
	_parameter_handles.airspeed_trim = param_find("FW_AIRSPD_TRIM");
	_parameter_handles.airspeed_max = param_find("FW_AIRSPD_MAX");

	/* fetch initial parameter values */
	parameters_update();
}

FixedwingAttitudeControlVector::~FixedwingAttitudeControlVector()
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
FixedwingAttitudeControlVector::parameters_update()
{

	param_get(_parameter_handles.p_tconst, &(_parameters.p_tconst));
	param_get(_parameter_handles.p_p, &(_parameters.p_p));
	param_get(_parameter_handles.p_d, &(_parameters.p_d));
	param_get(_parameter_handles.p_i, &(_parameters.p_i));
	param_get(_parameter_handles.p_rmax_up, &(_parameters.p_rmax_up));
	param_get(_parameter_handles.p_rmax_dn, &(_parameters.p_rmax_dn));
	param_get(_parameter_handles.p_imax, &(_parameters.p_imax));
	param_get(_parameter_handles.p_rll, &(_parameters.p_rll));

	param_get(_parameter_handles.r_tconst, &(_parameters.r_tconst));
	param_get(_parameter_handles.r_p, &(_parameters.r_p));
	param_get(_parameter_handles.r_d, &(_parameters.r_d));
	param_get(_parameter_handles.r_i, &(_parameters.r_i));
	param_get(_parameter_handles.r_imax, &(_parameters.r_imax));
	param_get(_parameter_handles.r_rmax, &(_parameters.r_rmax));

	param_get(_parameter_handles.y_slip, &(_parameters.y_slip));
	param_get(_parameter_handles.y_int, &(_parameters.y_int));
	param_get(_parameter_handles.y_damp, &(_parameters.y_damp));
	param_get(_parameter_handles.y_rll, &(_parameters.y_rll));
	param_get(_parameter_handles.y_imax, &(_parameters.y_imax));

	param_get(_parameter_handles.airspeed_min, &(_parameters.airspeed_min));
	param_get(_parameter_handles.airspeed_trim, &(_parameters.airspeed_trim));
	param_get(_parameter_handles.airspeed_max, &(_parameters.airspeed_max));

	/* pitch control parameters */
	_att_control.set_tconst(_parameters.p_tconst);
	// _pitch_ctrl.set_k_p(math::radians(_parameters.p_p));
	// _pitch_ctrl.set_k_i(math::radians(_parameters.p_i));
	// _pitch_ctrl.set_k_d(math::radians(_parameters.p_d));
	// _pitch_ctrl.set_imax(math::radians(_parameters.p_imax));
	// _pitch_ctrl.set_max_rate_pos(math::radians(_parameters.p_rmax_up));
	// _pitch_ctrl.set_max_rate_neg(math::radians(_parameters.p_rmax_dn));
	// _pitch_ctrl.set_roll_ff(math::radians(_parameters.p_rll));

	// /* roll control parameters */
	// _roll_ctrl.set_tau(_parameters.r_tconst);
	// _roll_ctrl.set_k_p(math::radians(_parameters.r_p));
	// _roll_ctrl.set_k_i(math::radians(_parameters.r_i));
	// _roll_ctrl.set_k_d(math::radians(_parameters.r_d));
	// _roll_ctrl.set_imax(math::radians(_parameters.r_imax));
	// _roll_ctrl.set_max_rate(math::radians(_parameters.r_rmax));

	// /* yaw control parameters */
	// _yaw_ctrl.set_k_a(math::radians(_parameters.y_slip));
	// _yaw_ctrl.set_k_i(math::radians(_parameters.y_int));
	// _yaw_ctrl.set_k_d(math::radians(_parameters.y_damp));
	// _yaw_ctrl.set_k_ff(math::radians(_parameters.y_rll));
	// _yaw_ctrl.set_imax(math::radians(_parameters.y_imax));

	return OK;
}

void
FixedwingAttitudeControlVector::vehicle_status_poll()
{
	bool vstatus_updated;

	/* Check HIL state if vehicle status has changed */
	orb_check(_vstatus_sub, &vstatus_updated);

	if (vstatus_updated) {

		orb_copy(ORB_ID(vehicle_status), _vstatus_sub, &_vstatus);
	}
}

bool
FixedwingAttitudeControlVector::vehicle_airspeed_poll()
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
FixedwingAttitudeControlVector::vehicle_accel_poll()
{
	/* check if there is a new position */
	bool accel_updated;
	orb_check(_accel_sub, &accel_updated);

	if (accel_updated) {
		orb_copy(ORB_ID(sensor_accel), _accel_sub, &_accel);
	}
}

void
FixedwingAttitudeControlVector::vehicle_setpoint_poll()
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
FixedwingAttitudeControlVector::arming_status_poll()
{
	/* check if there is a new setpoint */
	bool arming_updated;
	orb_check(_arming_sub, &arming_updated);

	if (arming_updated) {
		orb_copy(ORB_ID(actuator_armed), _arming_sub, &_arming);
	}
}

void
FixedwingAttitudeControlVector::task_main_trampoline(int argc, char *argv[])
{
	att_control::g_control->task_main();
}

void
FixedwingAttitudeControlVector::task_main()
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
	_manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
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

			/* lock integrator until armed */
			bool lock_integrator;
			if (_arming.armed) {
				lock_integrator = false;
			} else {
				lock_integrator = true;
			}

			/* decide if in stabilized or full manual control */

			if (_vstatus.state_machine == SYSTEM_STATE_MANUAL && !(_vstatus.manual_control_mode == VEHICLE_MANUAL_CONTROL_MODE_SAS)
				) {

				_actuators.control[0] = _manual.roll;
				_actuators.control[1] = _manual.pitch;
				_actuators.control[2] = _manual.yaw;
				_actuators.control[3] = _manual.throttle;
				_actuators.control[4] = _manual.flaps;

			} else if (_vstatus.state_machine == SYSTEM_STATE_AUTO ||
				(_vstatus.state_machine == SYSTEM_STATE_STABILIZED) ||
				(_vstatus.state_machine == SYSTEM_STATE_MANUAL && _vstatus.manual_control_mode == VEHICLE_MANUAL_CONTROL_MODE_SAS)) {

				/* scale from radians to normalized -1 .. 1 range */
				const float actuator_scaling = 1.0f / (M_PI_F / 4.0f);

				/* scale around tuning airspeed */

				float airspeed;

				/* if airspeed is smaller than min, the sensor is not giving good readings */
				if (!_airspeed_valid ||
					(_airspeed.indicated_airspeed_m_s < _parameters.airspeed_min) ||
					!isfinite(_airspeed.indicated_airspeed_m_s)) {
					airspeed = _parameters.airspeed_min + (_parameters.airspeed_max - _parameters.airspeed_min) / 2.0f;
				} else {
					airspeed = _airspeed.indicated_airspeed_m_s;
				}

				float airspeed_scaling = _parameters.airspeed_trim / airspeed;

				float roll_sp, pitch_sp;
				float throttle_sp = 0.0f;

				if (_vstatus.state_machine == SYSTEM_STATE_AUTO) {
					roll_sp = _att_sp.roll_body;
					pitch_sp = _att_sp.pitch_body;
					throttle_sp = _att_sp.thrust;
				} else if ((_vstatus.state_machine == SYSTEM_STATE_STABILIZED) ||
					   (_vstatus.state_machine == SYSTEM_STATE_MANUAL && _vstatus.manual_control_mode == VEHICLE_MANUAL_CONTROL_MODE_SAS)) {
					
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
					throttle_sp = _manual.throttle;
					_actuators.control[4] = _manual.flaps;
				}

				math::Dcm R_nb(_att.R);

				// Transform body frame forces to
				// global frame
				const math::Vector3 F_des(pitch_sp, roll_sp, throttle_sp);
				const math::Vector3 angular_rates(_att.rollspeed, _att.pitchspeed, _att.yawspeed);

				// Return variables
				math::Vector3 moments_des;
				float thrust;

				_att_control.control(deltaT, airspeed_scaling, airspeed, R_nb, _att.roll, _att.pitch, _att.yaw,
					F_des, angular_rates, moments_des, thrust);

				_actuators.control[0] = (isfinite(moments_des(0))) ? moments_des(0) * actuator_scaling : 0.0f;
				_actuators.control[1] = (isfinite(moments_des(1))) ? moments_des(1) * actuator_scaling : 0.0f;
				_actuators.control[2] = 0.0f;//(isfinite(moments_des(0))) ? moments_des(0) * actuator_scaling : 0.0f;

				/* throttle passed through */
				_actuators.control[3] = (isfinite(thrust)) ? thrust : 0.0f;

				// warnx("aspd: %s: %6.2f, aspd scaling: %6.2f, controls: %5.2f %5.2f %5.2f %5.2f", (_airspeed_valid) ? "valid" : "unknown",
				// 			airspeed, airspeed_scaling, _actuators.control[0], _actuators.control[1],
				// 			_actuators.control[2], _actuators.control[3]);

				/*
				 * Lazily publish the rate setpoint (for analysis, the actuators are published below)
				 * only once available
				 */
				vehicle_rates_setpoint_s rates_sp;
				math::Vector3 rates_des = _att_control.get_rates_des();
				rates_sp.roll = rates_des(0);
				rates_sp.pitch = rates_des(1);
				rates_sp.yaw = 0.0f; // XXX rates_des(2);

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

		}

		perf_end(_loop_perf);
	}

	warnx("exiting.\n");

	_control_task = -1;
	_exit(0);
}

int
FixedwingAttitudeControlVector::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = task_spawn_cmd("fw_att_control",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       2048,
				       (main_t)&FixedwingAttitudeControlVector::task_main_trampoline,
				       nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int fw_att_control_vector_main(int argc, char *argv[])
{
	if (argc < 1)
		errx(1, "usage: fw_att_control {start|stop|status}");

	if (!strcmp(argv[1], "start")) {

		if (att_control::g_control != nullptr)
			errx(1, "already running");

		att_control::g_control = new FixedwingAttitudeControlVector;

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
