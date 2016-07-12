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
 * @file mc_quat_control_main.cpp
 * Multicopter attitude controller.
 *
 * @author Tobias Naegeli <naegelit@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Moses Bangura <moses.bangura@anu.edu.au, dnovichman@hotmail.com>
 *
 * This controller uses quaternions and is based on controller theory presented in the paper
 * Bangura, Lim, Kim and Mahony, "An Open-Source Implementation of a Unit Quaternion based Attitude and Trajectory Tracking for 	  	Quadrotors", In ACRA, 2014.
 * The controller operates in two modes: manual and autonomous. In autonomous mode, the desired quaternion,  attitude rate and attitude 
 * acceleration are determined which are controlled by this controller. In manual mode, the controller regulates the desired quaternion
 * and desired feedforward terms (angular velocity and acceleration) which are zero. The controller is a PD controller on 
 * attitude/attitude ratte with the momentum of inertia (Kd = I) used as a feedforward only for autonomous mode. Though quaternions are
 * non-intuitive, the manual control handles this.
 * In the momentum of inertia matrix, we assume I = diag(Ixx Iyy Izz) = diag(ROLLR_D, PITCHR_D, YAW_FF)
*/

#include <drivers/device/device.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_sbus.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_mixer.h>

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
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
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <uORB/topics/vehicle_omega_ff_setpoint.h>

#include <uORB/topics/rc_channels.h>
#include <uORB/topics/battery_status.h>

/**
 * Quadroter quaternion based attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_quat_control_main(int argc, char *argv[]);


class MulticopterQuaternionControl
{
public:
	/**
	 * Constructor
	 */
	MulticopterQuaternionControl();

	/**
	 * Destructor, also kills the quaternion attitude task.
	 */
	~MulticopterQuaternionControl();

	/**
	 * Start the quaternion control task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:

	bool	_task_should_exit;		/**< if true, task should exit */
	int		_control_task;			/**< task handle for quaternion attitude task */

	int		_v_att_sub;				/**< vehicle attitude subscription */
	int		_v_att_sp_sub;			/**< vehicle attitude setpoint subscription */
	int		_v_rates_sp_sub;		/**< vehicle rates setpoint subscription */
	int		_v_control_mode_sub;	/**< vehicle control mode subscription */
	int		_params_sub;			/**< parameter updates subscription */
	int		_manual_control_sp_sub;	/**< manual control setpoint subscription */
	int		_armed_sub;				/**< arming status subscription */
	int 	_omega_d_sub;			/**< Feedforward angular velocity */
	int 	_v_rc_sub;
	int 	_battery_sub;

	orb_advert_t	_att_sp_pub;			/**< attitude setpoint publication */
	orb_advert_t	_v_rates_sp_pub;		/**< rate setpoint publication */
	orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */

	bool		_actuators_0_circuit_breaker_enabled;	/**< circuit breaker to suppress output */

	struct vehicle_attitude_s			_v_att;				/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s	_v_att_sp;			/**< vehicle attitude setpoint */
	struct vehicle_rates_setpoint_s		_v_rates_sp;		/**< vehicle rates setpoint */
	struct manual_control_setpoint_s	_manual_control_sp;	/**< manual control setpoint */
	struct vehicle_control_mode_s		_v_control_mode;	/**< vehicle control mode */
	struct actuator_controls_s			_actuators;			/**< actuator controls */
	struct actuator_armed_s				_armed;				/**< actuator arming status */
	struct vehicle_omega_ff_setpoint_s 	_omega_d;	/**< Feedforward angular velocity */
	struct rc_channels_s _v_rc;
	struct battery_status_s _battery;

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	math::Vector<3>		_rates_prev;	/**< angular rates on previous step */
	math::Vector<3>		_rates_sp;		/**< angular rates setpoint */
	math::Vector<3>		_rates_int;		/**< angular rates integral error */
	float				_thrust_sp;		/**< thrust setpoint */
	math::Vector<3>		_att_control;	/**< attitude control vector */
	float int_yawrate = 0.0f;

	math::Matrix<3, 3>  _I;				/**< identity matrix */

	bool	_reset_yaw_sp;			/**< reset yaw setpoint flag */
	float params_yaw_rate_i = 0.1;	 


	struct {
		param_t roll_p;
		param_t roll_rate_p;
		param_t roll_rate_d;
		param_t pitch_p;
		param_t pitch_rate_p;
		param_t pitch_rate_d;
		param_t yaw_p;
		param_t yaw_rate_p;
		param_t yaw_rate_d;
		param_t yaw_rate_i;
		param_t yaw_rate_i_max;

		param_t man_roll_max;
		param_t man_pitch_max;
		param_t man_yaw_max;
		param_t acro_roll_max;
		param_t acro_pitch_max;
		param_t acro_yaw_max;

		param_t battery_switch;
	}		_params_handles;		/**< handles for interesting parameters */

	struct {
		math::Vector<3> att_p;					/**< P gain for angular error */
		math::Vector<3> rate_p;				/**< P gain for angular rate error */
		math::Vector<3> rate_d;				/**< D gain for angular rate error */
		float yaw_ff;						/**< yaw control feed-forward */
		float yaw_rate_max;					/**< max yaw rate */
		float yaw_rate_i = 0.1;
		float yaw_rate_i_max = 0.0f;

		float man_roll_max;
		float man_pitch_max;
		float man_yaw_max;
		math::Vector<3> acro_rate_max;		/**< max attitude rates in acro mode */

		int battery_switch;
	}		_params;

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
	  * Obtain feedforward in vehicle angular velocity
	*/
	void 		feedforwad_omega_poll();

	/**
	  * Obtain battery status for esc
	*/
	void 		battery_status_poll();

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

	/**
	 * Quaternion controller.
	 */
	void		control_attitude(float dt);

	/**
	 * Attitude rates controller.
	 */
	void		control_attitude_rates(float dt);

	/**
	 * Quaternion compilation files.
	*/
	float invSqrt(float number);

	void getQuatFromRPY123(const float roll, const float pitch, const float yaw, float* qd);

	void quat_mult(const float *a,const float *b,float *prod);

	void quatConj(float *q, float *qconj);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main quaternion control task.
	 */
	void		task_main();

	void rc_poll();
};

namespace mc_quat_control
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

MulticopterQuaternionControl	*g_control;
}

MulticopterQuaternionControl::MulticopterQuaternionControl() :

	_task_should_exit(false),
	_control_task(-1),

/* subscriptions */
	_v_att_sub(-1),
	_v_att_sp_sub(-1),
	_v_control_mode_sub(-1),
	_params_sub(-1),
	_manual_control_sp_sub(-1),
	_armed_sub(-1),
	_omega_d_sub(-1),
	_v_rc_sub(-1),
	_battery_sub(-1),

/* publications */
	_att_sp_pub(nullptr),
	_v_rates_sp_pub(nullptr),
	_actuators_0_pub(nullptr),

	_actuators_0_circuit_breaker_enabled(false),

/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "mc_quat_control"))

{
	memset(&_v_att, 0, sizeof(_v_att));
	memset(&_v_att_sp, 0, sizeof(_v_att_sp));
	memset(&_v_rates_sp, 0, sizeof(_v_rates_sp));
	memset(&_manual_control_sp, 0, sizeof(_manual_control_sp));
	memset(&_v_control_mode, 0, sizeof(_v_control_mode));
	memset(&_actuators, 0, sizeof(_actuators));
	memset(&_armed, 0, sizeof(_armed));

	_params.att_p.zero();
	_params.rate_p.zero();
	_params.rate_d.zero();
	_params.yaw_rate_max = 0.0f;
	_params.man_roll_max = 0.0f;
	_params.man_pitch_max = 0.0f;
	_params.man_yaw_max = 0.0f;
	_params.acro_rate_max.zero();

	_rates_prev.zero();
	_rates_sp.zero();
	_rates_int.zero();
	_thrust_sp = 0.0f;
	_att_control.zero();

	_I.identity();

	_params_handles.roll_p			= 	param_find("MC_QUAT_ROLL_P");
	_params_handles.roll_rate_p		= 	param_find("MC_QUAT_ROLLR_P");
	_params_handles.roll_rate_d		= 	param_find("MC_QUAT_ROLLR_D");
	_params_handles.pitch_p			= 	param_find("MC_QUAT_PITCH_P");
	_params_handles.pitch_rate_p	= 	param_find("MC_QUAT_PITCHR_P");
	_params_handles.pitch_rate_d	= 	param_find("MC_QUAT_PITCHR_D");
	_params_handles.yaw_p			=	param_find("MC_QUAT_YAW_P");
	_params_handles.yaw_rate_p		= 	param_find("MC_QUAT_YAWR_P");
	_params_handles.yaw_rate_d		= 	param_find("MC_QUAT_YAWR_D");
	_params_handles.yaw_rate_i		= 	param_find("MC_QUAT_YAWR_I");
	_params_handles.yaw_rate_i_max		= 	param_find("MC_QUAT_YR_IMAX");
	_params_handles.man_roll_max	= 	param_find("MC_MAN_R_MAX");
	_params_handles.man_pitch_max	= 	param_find("MC_MAN_P_MAX");
	_params_handles.man_yaw_max		= 	param_find("MC_MAN_Y_MAX");
	_params_handles.acro_roll_max	= 	param_find("MC_ACRO_R_MAX");
	_params_handles.acro_pitch_max	= 	param_find("MC_ACRO_P_MAX");
	_params_handles.acro_yaw_max	= 	param_find("MC_ACRO_Y_MAX");
	_params_handles.battery_switch	= 	param_find("MC_QUAT_BAT_V");

	/* fetch initial parameter values */
	parameters_update();
}

/*******************************************************************************/
/* Quaternion specific files */

/* Fast inverse square-root see: http://en.wikipedia.org/wiki/Fast_inverse_square_root */
float 
MulticopterQuaternionControl::invSqrt(float number) {
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );
    return y;
}

void 
MulticopterQuaternionControl::getQuatFromRPY123(const float roll, const float pitch, const float yaw, float* qd)
{
	float c_phi, c_theta, c_psi;
	float s_phi, s_theta, s_psi;
	float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

	/* Precompute frequently used variables. */
	c_phi = cosf(0.5f*roll);
	c_theta = cosf(0.5f*pitch);
	c_psi = cosf(0.5f*yaw);

	s_phi = sinf(0.5f*roll);
	s_theta = sinf(0.5f*pitch);
	s_psi = sinf(0.5f*yaw);

	/* Convert RPY to Quaternion by 1-2-3 representation. See equation (297) of James Diebel paper. */	
	q[0] = c_phi*c_theta*c_psi 	+ s_phi*s_theta*s_psi;
	q[1] = -c_phi*s_theta*s_psi + c_theta*c_psi*s_phi;
	q[2] = c_phi*c_psi*s_theta 	+ s_phi*c_theta*s_psi;
	q[3] = c_phi*c_theta*s_psi 	- s_phi*c_psi*s_theta;

	float recipNorm = invSqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);

	/* Normalization */
	qd[0] = q[0]*recipNorm;
	qd[1] = q[1]*recipNorm;
	qd[2] = q[2]*recipNorm;
	qd[3] = q[3]*recipNorm;
}

/* Quaternion multiplication */
void 
MulticopterQuaternionControl::quat_mult(const float *a,const float *b,float *prod)
{
	prod[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    prod[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    prod[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    prod[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

/* Quaternion conjugation */
void 
MulticopterQuaternionControl::quatConj(float *q, float *qconj)
{
	float recipNorm = invSqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);

	/* Ensure qconj is unit quaternion */
	qconj[0] = q[0]*recipNorm;
	qconj[1] = -q[1]*recipNorm;
	qconj[2] = -q[2]*recipNorm;
	qconj[3] = -q[3]*recipNorm;
}

/* Quaternion to Euler */
/* End Quaternion specific files */
/*******************************************************************************/

MulticopterQuaternionControl::~MulticopterQuaternionControl()
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

	mc_quat_control::g_control = nullptr;
}

int
MulticopterQuaternionControl::parameters_update()
{
	float v;

	/* roll gains */
	param_get(_params_handles.roll_p, &v);
	_params.att_p(0) = v;
	param_get(_params_handles.roll_rate_p, &v);
	_params.rate_p(0) = v;

	/* pitch gains */
	param_get(_params_handles.pitch_p, &v);
	_params.att_p(1) = v;
	param_get(_params_handles.pitch_rate_p, &v);
	_params.rate_p(1) = v;

	/* yaw gains */
	param_get(_params_handles.yaw_p, &v);
	_params.att_p(2) = v;
	param_get(_params_handles.yaw_rate_p, &v);
	_params.rate_p(2) = v;

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

	param_get(_params_handles.yaw_rate_i, &_params.yaw_rate_i);
	params_yaw_rate_i = _params.yaw_rate_i;
	param_get(_params_handles.yaw_rate_i_max, &_params.yaw_rate_i_max);	

	param_get(_params_handles.battery_switch, &_params.battery_switch);

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);

	return OK;
}

void
MulticopterQuaternionControl::parameter_update_poll()
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
MulticopterQuaternionControl::vehicle_control_mode_poll()
{
	bool updated;

	/* Check if vehicle status has changed */
	orb_check(_v_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
	}
}

void
MulticopterQuaternionControl::battery_status_poll()
{
	bool updated;

	/* Check for battery status update */
	orb_check(_battery_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(battery_status), _battery_sub, &_battery);
	}
}

void
MulticopterQuaternionControl::feedforwad_omega_poll()
{
	bool updated;
	orb_check(_omega_d_sub, &updated);
	{
		orb_copy(ORB_ID(vehicle_omega_ff_setpoint), _omega_d_sub, &_omega_d);
	}	
}

void
MulticopterQuaternionControl::rc_poll()
{
	bool updated;
	orb_check(_v_rc_sub, &updated);
	{
		orb_copy(ORB_ID(rc_channels), _v_rc_sub, &_v_rc);
	}
}

void
MulticopterQuaternionControl::vehicle_manual_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
	}
}

void
MulticopterQuaternionControl::vehicle_attitude_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
	}
}

void
MulticopterQuaternionControl::vehicle_rates_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_rates_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
	}
}

void
MulticopterQuaternionControl::arming_status_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
	}
}

/*
 * Quaternion Attitude controller.
 * Input: 'manual_control_setpoint' and 'vehicle_attitude_setpoint' topics (depending on mode)
 * Output: '_rates_sp' vector, '_thrust_sp', 'vehicle_attitude_setpoint' topic (for manual modes)
 */
void
MulticopterQuaternionControl::control_attitude(float dt)
{
	float yaw_sp_move_rate = 0.0f;
	bool publish_att_sp = false;

	static float q_d[4] = {1.0f, 0.0f, 0.0f, 0.0f};
	static float q_d_inv[4] = {1.0f, 0.0f, 0.0f, 0.0f};
	static float q_e[4] = {1.0f, 0.0f, 0.0f, 0.0f};

	/* Determine the actual attitude of the vehicle in quaternion since we don't know which observer is running */
	if (_v_att.q_valid) {	

		if (_v_control_mode.flag_control_manual_enabled) {	
			getQuatFromRPY123(_v_att.roll, _v_att.pitch, 0, _v_att.q);
		}
	} else 	{
		if (_v_control_mode.flag_control_manual_enabled) {
			getQuatFromRPY123(_v_att.roll, _v_att.pitch, 0, _v_att.q);
		} else {
			getQuatFromRPY123(_v_att.roll, _v_att.pitch, _v_att.yaw, _v_att.q);
		}
	}
//printf("control mode %d %d\n",_v_control_mode.flag_control_auto_enabled, _v_control_mode.flag_control_manual_enabled);
	/*************** Now lets calculate the desired quaternion **************************/
	if (_v_control_mode.flag_control_manual_enabled) {

		/* manual input, set or modify attitude setpoint */

		if (_v_control_mode.flag_control_velocity_enabled || _v_control_mode.flag_control_climb_rate_enabled) {
			/* in assisted modes poll 'vehicle_attitude_setpoint' topic and modify it */
			vehicle_attitude_setpoint_poll();
		}

		if (!_v_control_mode.flag_control_climb_rate_enabled) {
			/* pass throttle directly if not in altitude stabilized mode */
			_v_att_sp.thrust = _manual_control_sp.z;
			publish_att_sp = true;
		}
		

		/* move yaw setpoint in all modes */
		if (_v_att_sp.thrust < 0.1f) {
			// TODO
			//if (_status.condition_landed) {
			/* reset yaw setpoint if on ground */
			//	reset_yaw_sp = true;
			//}
		} else {
			/* move yaw setpoint */
			yaw_sp_move_rate = _manual_control_sp.r * _params.man_yaw_max;
			_v_att_sp.yaw_body = _wrap_pi(_v_att_sp.yaw_body + yaw_sp_move_rate * dt);
			float yaw_offs_max = _params.man_yaw_max / _params.att_p(2);
			float yaw_offs = _wrap_pi(_v_att_sp.yaw_body - _v_att.yaw);
			if (yaw_offs < - yaw_offs_max) {
				_v_att_sp.yaw_body = _wrap_pi(_v_att.yaw - yaw_offs_max);

			} else if (yaw_offs > yaw_offs_max) {
				_v_att_sp.yaw_body = _wrap_pi(_v_att.yaw + yaw_offs_max);
			}
			_v_att_sp.R_valid = false;
			publish_att_sp = true;
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
			_v_att_sp.roll_body = _manual_control_sp.y * _params.man_roll_max;
			_v_att_sp.pitch_body = -_manual_control_sp.x * _params.man_pitch_max;
			_v_att_sp.R_valid = false;
			_v_att_sp.q_d_valid = false;
			publish_att_sp = true;
		}


		/* Compute desired quaternion q_d: This is for future */	
		getQuatFromRPY123(_v_att_sp.roll_body, _v_att_sp.pitch_body, 0, q_d); 
		
		/* Update the attitude setpoint */
		_v_att_sp.q_d[0] = q_d[0];
		_v_att_sp.q_d[1] = q_d[1];
		_v_att_sp.q_d[2] = q_d[2];
		_v_att_sp.q_d[3] = q_d[3];
		if (_att_sp_pub != nullptr) {
			orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &_v_att_sp);			

		} else {
			_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_v_att_sp);
		}
		_thrust_sp = _v_att_sp.thrust;

		if (_thrust_sp < 0.2f)
			int_yawrate = 0.0f;
//printf("att manual mode\n");
	} else {
		/* Autonomous mode use 'vehicle_attitude_setpoint' topic */
		vehicle_attitude_setpoint_poll();

		//printf("In auto att %3.3f\n", double(_v_att_sp.q_d[0]));

		/* reset yaw setpoint after non-manual control mode */
		_reset_yaw_sp = true;

		if (_v_att_sp.q_d_valid)
		{
			q_d[0] = _v_att_sp.q_d[0];
			q_d[1] = _v_att_sp.q_d[1];
			q_d[2] = _v_att_sp.q_d[2];		
			q_d[3] = _v_att_sp.q_d[3];
			_thrust_sp = _manual_control_sp.z; //_v_att_sp.thrust; //_manual_control_sp.z
			//_thrust_sp = _v_att_sp.thrust;
		} else {
			//TODO maybe we should have here conversion from att_d(rpy) to quaternion
			_thrust_sp = 0.1;
			int_yawrate = 0.0;
		}
	}
	/********************* End calculation of the desired quaternion ************************/
	//printf("thr stp %3.3f %3.3f %3.3f %3.3f %3.3f\n",double(_thrust_sp), double(q_d[0]), double( q_d[1]), double(q_d[2]), double(q_d[3]));

	/* publish the attitude setpoint if needed */
	if (publish_att_sp) {
		_v_att_sp.timestamp = hrt_absolute_time();

		if (_att_sp_pub != nullptr) {
			orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &_v_att_sp);

		} else {
			_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_v_att_sp);
		}
	}


	/* Compute or obtain error quaternion q_e */	
	if (_v_control_mode.flag_control_manual_enabled) {
		quatConj(q_d,q_d_inv);
		quat_mult(q_d_inv,_v_att.q,q_e);

		_v_att_sp.q_e[0] = q_e[0];
		_v_att_sp.q_e[1] = q_e[1];
		_v_att_sp.q_e[2] = q_e[2];
		_v_att_sp.q_e[3] = q_e[3];
		_v_att_sp.q_e_valid = true;

		_v_att_sp.q_d[0] = q_d[0];
		_v_att_sp.q_d[1] = q_d[1];
		_v_att_sp.q_d[2] = q_d[2];
		_v_att_sp.q_d[3] = q_d[3];
		_v_att_sp.q_d_valid = true;
	} else { //TODO handle other apps producing not only qd but also qe
		quatConj(q_d,q_d_inv);
		quat_mult(q_d_inv,_v_att.q,q_e);
	}

	/* Calculate the desired Omega and publish */
	_rates_sp(0) = -_params.att_p(0)*q_e[1]; 
	_rates_sp(1) = -_params.att_p(1)*q_e[2]; 

	if (_v_control_mode.flag_control_manual_enabled) {
		_rates_sp(2) = _manual_control_sp.r * _params.acro_rate_max(2); 
	} else 
		_rates_sp(2) = -_params.att_p(2)*q_e[3]; 

}

/*
 * Attitude rates controller tau = K_d*\tilde{Omega} + I Omega_d +....
 * Input: '_rates_sp' _'rate_rates_sp', '_thrust_sp'
 * Output: '_att_control' 
 */
void
MulticopterQuaternionControl::control_attitude_rates(float dt)
{
	/* reset integral if disarmed */
	if (!_armed.armed) {
		_rates_int.zero();
	}

	/* current body angular rates */
	math::Vector<3> rates, vehicles_Omega_rates_d;
	rates(0) = _v_att.rollspeed;
	rates(1) = _v_att.pitchspeed;
	rates(2) = _v_att.yawspeed;

	/* angular rates error */
	//here we fix rates = rates + Omega_d
	//tau_d = k_d*omega_rate_d + S(omega_d)KdOmega - 0. The zero is for Ga which is hard to determine
	math::Vector<3> omega_d_vec;
	if (_v_control_mode.flag_control_auto_enabled && _omega_d.validity) //modify this to incorporate auto mode
	{
		omega_d_vec(0) = _omega_d.rates[0];
		omega_d_vec(1) = _omega_d.rates[1];
		omega_d_vec(2) = _omega_d.rates[2];
	} else
	{
		omega_d_vec(0) = 0.0f;
		omega_d_vec(1) = 0.0f;
		omega_d_vec(2) = 0.0f;
	}
	
	math::Vector<3> rates_err = rates - _rates_sp - omega_d_vec;

	/* Feedforward for angular velocity */
	math::Vector<3> vehicles_Omega_rates = (rates - _rates_prev)/dt;

	/* To keep up with PX4 Notation, Ixx = _params.rate_d(0), Iyy = _params.rate_d(1), Izz = _params.rate_d(2). For manual flight, 		these are set to zero */
	_params.rate_d(0) = 0.0f;
	_params.rate_d(1) = 0.0f;
	_params.rate_d(2) = 0.0f;

	/* There should be a topic for these and they should come from high-level trajectory controller */
	vehicles_Omega_rates_d(0) = 0.0f;
	vehicles_Omega_rates_d(1) = 0.0f;
	vehicles_Omega_rates_d(2) = 0.0f;


	_att_control(0) = -_params.rate_p(0)*rates_err(0) - _params.rate_d(0)*(vehicles_Omega_rates(0) - vehicles_Omega_rates_d(0));
	_att_control(1) = -_params.rate_p(1)*rates_err(1) - _params.rate_d(1)*(vehicles_Omega_rates(1) - vehicles_Omega_rates_d(1));
	_att_control(2) = -_params.rate_p(2)*rates_err(2) - _params.rate_d(2)*(vehicles_Omega_rates(2) - vehicles_Omega_rates_d(2));

	//printf("acct %3.3f %3.3f %3.3f\n", double(_att_control(0)), double(_att_control(1)), double(_att_control(2)) );

	/* Here we implement the battery voltage if we are using open-loop voltage control 16.4*/
	//printf("Battery voltage %3.3f %3.3f %d\n", (double)_battery.voltage_v, double(_battery.voltage_filtered_v), _params.battery_switch);

	//if (_params.battery_switch == 1) {
		//_thrust_sp = _thrust_sp * (1 + (16.4f - _battery.voltage_filtered_v)/16.4f);
	//}

	// yaw integral
	int_yawrate += rates_err(2)*dt;

	if (int_yawrate > _params.yaw_rate_i_max)
			int_yawrate = _params.yaw_rate_i_max;
	if (int_yawrate < -_params.yaw_rate_i_max)
		int_yawrate = -_params.yaw_rate_i_max;

	if (_thrust_sp < 0.3f)
			int_yawrate = 0.0f;

	_att_control(2) -= int_yawrate*params_yaw_rate_i;

	_rates_prev = rates;
}

void
MulticopterQuaternionControl::task_main_trampoline(int argc, char *argv[])
{
	mc_quat_control::g_control->task_main();
}

void
MulticopterQuaternionControl::task_main()
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

	_omega_d_sub = orb_subscribe(ORB_ID(vehicle_omega_ff_setpoint));
	_v_rc_sub =  orb_subscribe(ORB_ID(rc_channels));
	_battery_sub = orb_subscribe(ORB_ID(battery_status));


	/* initialize parameters cache */
	parameters_update();

	/* wakeup source: vehicle attitude */
	struct pollfd fds[1];

	fds[0].fd = _v_att_sub;
	fds[0].events = POLLIN;

	int fdt, ret, servo_count;
	int pwm_value = 1000;

	fdt = open(PX4IO_DEVICE_PATH, O_WRONLY);
	if (fdt < 0) {
		err(1, "failed to open device");
	}
	if (ioctl(fdt, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count)) {
		err(1, "failed to get servo count"); // This is only for testing as servo_count = 8
	}
	servo_position_t servos[servo_count];

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

			/* Testing servo control */
			servos[5] += dt*500;
			if (servos[5] > 2000)
				servos[5] = pwm_value;
			//ret = write(fdt, servos, sizeof(servos));
			if (_v_rc.channels[5] < 0)
			{
				servos[5] = pwm_value;
			} else
			{
				servos[5] = 2000;
			}
			ret = ioctl(fdt, PWM_SERVO_SET(5), servos[5]);

			if (ret != (int)sizeof(servos)) {
				//printf("error writing PWM servo data, wrote %u got %d\n", sizeof(servos), ret);
			}
			/* End testing */

			/* copy attitude topic */
			orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);

			/* check for updates in other topics */
			parameter_update_poll();
			vehicle_control_mode_poll();
			arming_status_poll();
			vehicle_manual_poll();
			feedforwad_omega_poll();
			rc_poll();
			battery_status_poll();
			//printf("RC is %3.3f %3.3f\n",double(_v_rc.channels[4]), double(_v_rc.channels[5]));

			if (_v_control_mode.flag_control_attitude_enabled) {
				control_attitude(dt);

				/* publish attitude rates setpoint */
				_v_rates_sp.roll = _rates_sp(0);
				_v_rates_sp.pitch = _rates_sp(1);
				_v_rates_sp.yaw = _rates_sp(2);
				_v_rates_sp.thrust = _thrust_sp;
				_v_rates_sp.timestamp = hrt_absolute_time();

				if (_v_rates_sp_pub != nullptr) {
					orb_publish(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_pub, &_v_rates_sp);

				} else {
					_v_rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &_v_rates_sp);
				}

			} else {
				/* quaternion controller disabled, poll rates setpoint topic */
				if (_v_control_mode.flag_control_manual_enabled) {
					/* manual rates control - ACRO mode */
					_rates_sp = math::Vector<3>(_manual_control_sp.y, -_manual_control_sp.x, _manual_control_sp.r).emult(_params.acro_rate_max);
					_thrust_sp = _manual_control_sp.z;

					/* publish attitude rates setpoint */
					_v_rates_sp.roll = _rates_sp(0);
					_v_rates_sp.pitch = _rates_sp(1);
					_v_rates_sp.yaw = _rates_sp(2);
					_v_rates_sp.thrust = _thrust_sp;
					_v_rates_sp.timestamp = hrt_absolute_time();

					if (_v_rates_sp_pub != nullptr) {
						orb_publish(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_pub, &_v_rates_sp);

					} else {
						_v_rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &_v_rates_sp);
					}

				} else {
					/* quaternion controller disabled, poll rates setpoint topic */
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
					if (_actuators_0_pub != nullptr) {
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
MulticopterQuaternionControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("mc_quat_control",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       2000,
				       (main_t)&MulticopterQuaternionControl::task_main_trampoline,
				       nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int mc_quat_control_main(int argc, char *argv[])
{
	if (argc < 1)
		errx(1, "usage: mc_quat_control {start|stop|status}");

	if (!strcmp(argv[1], "start")) {

		if (mc_quat_control::g_control != nullptr)
			errx(1, "already running");

		mc_quat_control::g_control = new MulticopterQuaternionControl;

		if (mc_quat_control::g_control == nullptr)
			errx(1, "alloc failed");

		if (OK != mc_quat_control::g_control->start()) {
			delete mc_quat_control::g_control;
			mc_quat_control::g_control = nullptr;
			err(1, "start failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (mc_quat_control::g_control == nullptr)
			errx(1, "not running");

		delete mc_quat_control::g_control;
		mc_quat_control::g_control = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (mc_quat_control::g_control) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}
