/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file mc_att_control
 * @author James Goppert <james.goppert@gmail.com>
 */

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <math.h>
#include <lib/mathlib/mathlib.h>
#include <fcntl.h>
#include <px4_posix.h>
#include <controllib/blocks.hpp>
#include <matrix/matrix/math.hpp>
#include <systemlib/mavlink_log.h>
#include <lib/tailsitter_recovery/tailsitter_recovery.h>

// subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/topics/control_state.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/multirotor_motor_limits.h>

// publications
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_0.h>
#include <uORB/topics/actuator_controls_virtual_mc.h>
#include <uORB/topics/mc_att_ctrl_status.h>

// pub and sub
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/mc_virtual_rates_setpoint.h>

static volatile bool thread_should_exit = false;     /**< Deamon exit flag */
static volatile bool thread_running = false;     /**< Deamon status flag */
static int deamon_task;             /**< Handle of deamon task / thread */
static const float deg2rad = M_PI_F / 180.0f;
static const float rad2deg = 180.0F / M_PI_F;
static const float MIN_TAKEOFF_THRUST = 0.2f;

using namespace control;
using namespace matrix;
using namespace uORB;

class AttCtrl : public control::SuperBlock
{
public:
	enum {POLL_STATE, n_poll};
	enum {ACT_ROLL, ACT_PITCH, ACT_YAW, ACT_THRUST, n_act};
private:
	// controllers
	BlockPID _ctrl_roll_rate;
	BlockPID _ctrl_pitch_rate;
	BlockPID _ctrl_yaw_rate;
	BlockP _ctrl_roll;
	BlockP _ctrl_pitch;
	BlockP _ctrl_yaw;

	// limits
	BlockLimitSym _roll_rate_max;
	BlockLimitSym _pitch_rate_max;
	BlockLimitSym _yaw_rate_max;
	BlockLimitSym _acro_roll_rate_max;
	BlockLimitSym _acro_pitch_rate_max;
	BlockLimitSym _acro_yaw_rate_max;

	// derivatives
	BlockDerivative _roll_rate_sp_deriv;
	BlockDerivative _pitch_rate_sp_deriv;
	BlockDerivative _yaw_rate_sp_deriv;

	// parameters
	BlockParamFloat _yaw_rate_auto_max;
	BlockParamFloat _yaw_ff;
	BlockParamFloat _roll_rate_ff;
	BlockParamFloat _pitch_rate_ff;
	BlockParamFloat _yaw_rate_ff;
	BlockParamFloat _ratt_thresh;
	BlockParamInt _vtol_type;
	BlockParamInt _vtol_opt_recovery_enabled;
	BlockParamFloat _vtol_wv_yaw_rate_scale;

	// subscriptions
	Subscription<control_state_s> _sub_cntrl_state;
	Subscription<vehicle_attitude_setpoint_s> _sub_att_sp;
	Subscription<vehicle_rates_setpoint_s> _sub_rates_sp;
	Subscription<vehicle_control_mode_s> _sub_mode;
	Subscription<manual_control_setpoint_s> _sub_manual;
	Subscription<parameter_update_s> _sub_params;
	Subscription<vehicle_status_s> _sub_status;
	Subscription<actuator_armed_s> _sub_armed;
	Subscription<multirotor_motor_limits_s> _sub_motor_limits;

	// publications
	Publication<actuator_controls_0_s> _pub_actuators;
	Publication<vehicle_rates_setpoint_s> _pub_rates_sp;

	// misc
	px4_pollfd_struct_t _polls[n_poll];
	uint64_t _timestamp;
	TailsitterRecovery *_ts_opt_recovery;	/**< Computes optimal rates for tailsitter recovery */

	// methods
	float wrap_pi(float val);

public:
	AttCtrl();
	void update();
	void control_attitude();
	void control_attitude_rates();
};


float AttCtrl::wrap_pi(float val)
{
	if (val > M_PI_F) {
		val -= 2 * M_PI_F;

	} else if (val < -M_PI_F) {
		val += 2 * M_PI_F;
	}

	return val;
}

AttCtrl::AttCtrl() :
	SuperBlock(NULL, "MC"),
	// controllers
	_ctrl_roll_rate(this, "ROLLR"),
	_ctrl_pitch_rate(this, "PITCHR"),
	_ctrl_yaw_rate(this, "YAWR"),
	_ctrl_roll(this, "ROLL_P"),
	_ctrl_pitch(this, "PITCH_P"),
	_ctrl_yaw(this, "YAW_P"),
	// limits
	_roll_rate_max(this, "ROLLR"),
	_pitch_rate_max(this, "PITCHR"),
	_yaw_rate_max(this, "YAWR"),
	_acro_roll_rate_max(this, "ACRO_R"),
	_acro_pitch_rate_max(this, "ACRO_P"),
	_acro_yaw_rate_max(this, "ACRO_Y"),
	//derivs
	_roll_rate_sp_deriv(this, "ROLLR_SP_D"),
	_pitch_rate_sp_deriv(this, "PITCHR_SP_D"),
	_yaw_rate_sp_deriv(this, "YAWR_SP_D"),
	// params
	_yaw_rate_auto_max(this, "YAWRAUTO_MAX"),
	_yaw_ff(this, "YAW_FF"),
	_roll_rate_ff(this, "ROLLR_FF"),
	_pitch_rate_ff(this, "PITCHR_FF"),
	_yaw_rate_ff(this, "YAWR_FF"),
	_ratt_thresh(this, "RATT_TH"),
	_vtol_type(this, "VT_TYPE", false),
	_vtol_opt_recovery_enabled(this, "VT_OPT_RECOV_EN", false),
	_vtol_wv_yaw_rate_scale(this, "VT_WV_YAWR_SCL", false),
	// subscriptions
	_sub_cntrl_state(ORB_ID(control_state),
			 0, 0, &getSubscriptions()),
	_sub_att_sp(ORB_ID(vehicle_attitude_setpoint),
		    0, 0, &getSubscriptions()),
	_sub_rates_sp(NULL,
		      0, 0, &getSubscriptions()),
	_sub_mode(ORB_ID(vehicle_control_mode),
		  0, 0, &getSubscriptions()),
	_sub_manual(ORB_ID(manual_control_setpoint),
		    0, 0, &getSubscriptions()),
	_sub_params(ORB_ID(parameter_update),
		    0, 0, &getSubscriptions()),
	_sub_status(ORB_ID(vehicle_status),
		    0, 0, &getSubscriptions()),
	_sub_armed(ORB_ID(actuator_armed),
		   0, 0, &getSubscriptions()),
	_sub_motor_limits(ORB_ID(multirotor_motor_limits),
			  0, 0, &getSubscriptions()),
	// publications
	_pub_actuators(NULL,
		       -1, &getPublications()),
	_pub_rates_sp(ORB_ID(vehicle_rates_setpoint),
		      -1, &getPublications()),
	// misc
	_polls(),
	_timestamp(hrt_absolute_time()),
	_ts_opt_recovery(NULL)
{
	_polls[POLL_STATE].fd = _sub_cntrl_state.getHandle();
	_polls[POLL_STATE].events = POLLIN;
	updateParams();

	if (_vtol_type.get() == 0 && _vtol_opt_recovery_enabled.get()) {
		_ts_opt_recovery = new TailsitterRecovery();
	}
}

void AttCtrl::update()
{
	// wait for a sensor update, check for exit condition every 100 ms
	int ret = px4_poll(_polls, n_poll, 100);

	if (ret < 0) {
		return;
	}

	// calculate dt
	uint64_t new_timestamp = hrt_absolute_time();
	float dt = (new_timestamp - _timestamp) / 1.0e6f;
	_timestamp = new_timestamp;

	// set dt for all child blocks
	setDt(dt);

	bool params_updated = _sub_params.updated();
	bool status_updated = _sub_status.updated();

	// update all subscriptions
	updateSubscriptions();

	// update params
	if (params_updated) { updateParams(); }

	// set correct uORB ID, depending on if vehicle is VTOL or not
	if (status_updated && _sub_rates_sp.getMeta() == NULL) {
		if (_sub_status.get().is_vtol) {
			_sub_rates_sp.setMeta(ORB_ID(mc_virtual_rates_setpoint));
			_pub_actuators.setMeta(ORB_ID(actuator_controls_virtual_mc));

		} else {
			_sub_rates_sp.setMeta(ORB_ID(vehicle_rates_setpoint));
			_pub_actuators.setMeta(ORB_ID(actuator_controls_0));
		}
	}

	// get the control mode struct
	const vehicle_control_mode_s &mode = _sub_mode.get();
	const manual_control_setpoint_s &manual = _sub_manual.get();

	// handle auto mode
	if (mode.flag_control_attitude_enabled) {
		// track attitude from subscription
		control_attitude();

		// handle rates mode

	} else if (mode.flag_control_rates_enabled) {
		// user manual input in rate mode
		vehicle_rates_setpoint_s &pub = _pub_rates_sp.get();
		pub.timestamp = _timestamp;
		pub.roll = manual.x;
		pub.pitch = manual.y;
		pub.thrust = manual.z;
		pub.yaw = manual.r;
		_pub_rates_sp.update();
	}

	// inner rates loop
	if (mode.flag_control_rates_enabled) {
		control_attitude_rates();
	}
}

void AttCtrl::control_attitude()
{
	// update subscriptions
	_sub_cntrl_state.update();
	_sub_att_sp.update();

	// get current state
	const control_state_s &x = _sub_cntrl_state.get();
	Quaternionf q(x.q);
	const vehicle_attitude_setpoint_s &r = _sub_att_sp.get();
	Eulerf euler_d(Quaternionf(r.q_d));
	Eulerf euler(Quaternionf(x.q));

	// compute attitdue errors
	float roll_err = wrap_pi(euler_d.phi() - euler.phi());
	float pitch_err = wrap_pi(euler_d.theta() - euler.theta());
	float yaw_err = wrap_pi(euler_d.psi() - euler.psi());

	// set points
	float roll_rate_sp = deg2rad * _roll_rate_max.update(
				     rad2deg * _ctrl_roll.update(roll_err));
	float pitch_rate_sp = deg2rad * _pitch_rate_max.update(
				      rad2deg * _ctrl_pitch.update(pitch_err));
	float yaw_rate_sp = deg2rad * _yaw_rate_max.update(
				    rad2deg * (_ctrl_yaw.update(yaw_err) +
					       r.yaw_sp_move_rate * _yaw_ff.get()));
	float thrust_sp = r.thrust;

	// weathervane mode
	if (_sub_att_sp.get().disable_mc_yaw_control == true &&
	    _sub_mode.get().flag_control_velocity_enabled &&
	    ! _sub_mode.get().flag_control_manual_enabled) {
		float wv_yaw_rate_max = _yaw_rate_auto_max.get() * _vtol_wv_yaw_rate_scale.get();
		// dampen yaw rate
		yaw_rate_sp = math::constrain(
				      yaw_rate_sp,
				      -wv_yaw_rate_max,
				      wv_yaw_rate_max);
		// prevent integral windup
		_ctrl_yaw_rate.getIntegral().reset();
	}

	// publish rates sp
	vehicle_rates_setpoint_s &pub = _pub_rates_sp.get();
	pub.roll = roll_rate_sp;
	pub.pitch = pitch_rate_sp;
	pub.yaw = yaw_rate_sp;
	pub.thrust = thrust_sp;
	_pub_rates_sp.update();
}

void AttCtrl::control_attitude_rates()
{
	// reset integral if disarmed
	if (!_sub_armed.get().armed || !_sub_status.get().is_rotary_wing) {
		_ctrl_roll_rate.getIntegral().reset();
		_ctrl_pitch_rate.getIntegral().reset();
		_ctrl_yaw_rate.getIntegral().reset();
	}

	// save previous integral states
	float roll_int_prev = _ctrl_roll_rate.getIntegral().getY();
	float pitch_int_prev = _ctrl_pitch_rate.getIntegral().getY();
	float yaw_int_prev = _ctrl_yaw_rate.getIntegral().getY();

	// update subscriptions
	_sub_rates_sp.update();

	// get current state
	const control_state_s &x = _sub_cntrl_state.get();
	const vehicle_rates_setpoint_s &r = _sub_rates_sp.get();

	// compute rate errors
	float roll_rate_err =  r.roll - x.roll_rate;
	float pitch_rate_err = r.pitch - x.pitch_rate;
	float yaw_rate_err = r.yaw - x.yaw_rate;

	// compute mix outputs
	float roll_mix = _ctrl_roll_rate.update(roll_rate_err) +
			 _roll_rate_sp_deriv.update(r.roll) * _roll_rate_ff.get();
	float pitch_mix = _ctrl_pitch_rate.update(pitch_rate_err) +
			  _pitch_rate_sp_deriv.update(r.pitch) * _pitch_rate_ff.get();
	float yaw_mix = _ctrl_yaw_rate.update(yaw_rate_err) +
			_yaw_rate_sp_deriv.update(r.yaw) * _yaw_rate_ff.get();
	float thrust_mix = r.thrust;

	// lock out integrals if motors saturated or on ground
	if (r.thrust < MIN_TAKEOFF_THRUST ||
	    _sub_motor_limits.get().lower_limit ||
	    _sub_motor_limits.get().upper_limit) {
		_ctrl_roll_rate.getIntegral().setY(roll_int_prev);
		_ctrl_pitch_rate.getIntegral().setY(pitch_int_prev);
		_ctrl_yaw_rate.getIntegral().setY(yaw_int_prev);
	}

	// publish mix outputs
	actuator_controls_0_s &pub = _pub_actuators.get();
	pub.timestamp = _timestamp;
	pub.timestamp_sample = x.timestamp;
	pub.control[actuator_controls_s::INDEX_ROLL] = roll_mix;
	pub.control[actuator_controls_s::INDEX_PITCH] = pitch_mix;
	pub.control[actuator_controls_s::INDEX_YAW] = yaw_mix;
	pub.control[actuator_controls_s::INDEX_THROTTLE] = thrust_mix;
	_pub_actuators.update();
}

/**
 * Multicopter attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_att_control_main(int argc, char *argv[]);

/**
 * Mainloop of deamon.
 */
int mc_att_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static int usage(const char *reason);

static int
usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: mc_att_control {start|stop|status} [-p <additional params>]\n\n");
	return 1;
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int mc_att_control_main(int argc, char *argv[])
{

	if (argc < 2) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("already running");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;

		deamon_task = px4_task_spawn_cmd("mc_att_control",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 5,
						 4500,
						 mc_att_control_thread_main,
						 (argv && argc > 2) ? (char *const *) &argv[2] : (char *const *) NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (thread_running) {
			warnx("stop");
			thread_should_exit = true;

		} else {
			warnx("not started");
		}

		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("is running");

		} else {
			warnx("not started");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int mc_att_control_thread_main(int argc, char *argv[])
{
	warnx("[mc_att_control] starting");

	AttCtrl attCtrl;

	thread_running = true;

	while (!thread_should_exit) {
		attCtrl.update();
	}

	warnx("[mc_att_control] exiting");

	thread_running = false;

	return 0;
}
