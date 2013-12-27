/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: Anton Babushkin <anton.babushkin@me.com>
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
 * @file mc_pos_control_main.cpp
 *
 * Multirotor position controller.
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
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_global_position_setpoint.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/pid/pid.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <mavlink/mavlink_log.h>

#define TILT_COS_MAX	0.7f
#define SIGMA			0.000001f

/**
 * Multicopter position control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_pos_control_main(int argc, char *argv[]);

class MulticopterPositionControl
{
public:
	/**
	 * Constructor
	 */
	MulticopterPositionControl();

	/**
	 * Destructor, also kills task.
	 */
	~MulticopterPositionControl();

	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:

	bool	_task_should_exit;		/**< if true, task should exit */
	int		_control_task;			/**< task handle for task */

	int		_att_sub;				/**< vehicle attitude subscription */
	int		_att_sp_sub;			/**< vehicle attitude setpoint */
	int		_control_mode_sub;		/**< vehicle control mode subscription */
	int 	_params_sub;			/**< notification of parameter updates */
	int 	_manual_sub;			/**< notification of manual control updates */
	int		_arming_sub;			/**< arming status of outputs */
	int		_local_pos_sub;			/**< vehicle local position */
	int		_global_pos_sp_sub;		/**< vehicle global position setpoint */

	orb_advert_t	_local_pos_sp_pub;		/**< local position setpoint publication */
	orb_advert_t	_att_sp_pub;			/**< attitude setpoint publication */
	orb_advert_t	_global_vel_sp_pub;		/**< vehicle global velocity setpoint */

	struct vehicle_attitude_s			_att;			/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s	_att_sp;		/**< vehicle attitude setpoint */
	struct manual_control_setpoint_s	_manual;		/**< r/c channel data */
	struct vehicle_control_mode_s		_control_mode;	/**< vehicle control mode */
	struct actuator_armed_s				_arming;		/**< actuator arming status */
	struct vehicle_local_position_s		_local_pos;		/**< vehicle local position */
	struct vehicle_local_position_setpoint_s		_local_pos_sp;		/**< vehicle local position */
	struct vehicle_global_position_setpoint_s	_global_pos_sp;	/**< vehicle global position setpoint */
	struct vehicle_global_velocity_setpoint_s	_global_vel_sp;	/**< vehicle global velocity setpoint */

	struct {
		param_t takeoff_alt;
		param_t takeoff_gap;
		param_t thr_min;
		param_t thr_max;
		param_t z_p;
		param_t z_vel_p;
		param_t z_vel_i;
		param_t z_vel_d;
		param_t z_vel_max;
		param_t z_ff;
		param_t xy_p;
		param_t xy_vel_p;
		param_t xy_vel_i;
		param_t xy_vel_d;
		param_t xy_vel_max;
		param_t xy_ff;
		param_t tilt_max;

		param_t rc_scale_pitch;
		param_t rc_scale_roll;
		param_t rc_scale_yaw;
	}		_params_handles;		/**< handles for interesting parameters */

	struct {
		float takeoff_alt;
		float takeoff_gap;
		float thr_min;
		float thr_max;
		float tilt_max;

		float rc_scale_pitch;
		float rc_scale_roll;
		float rc_scale_yaw;

		math::Vector<3> pos_p;
		math::Vector<3> vel_p;
		math::Vector<3> vel_i;
		math::Vector<3> vel_d;
		math::Vector<3> vel_ff;
		math::Vector<3> vel_max;
		math::Vector<3> sp_offs_max;
	}		_params;

	math::Vector<3> _pos;
	math::Vector<3> _vel;
	math::Vector<3> _pos_sp;
	math::Vector<3> _vel_sp;
	math::Vector<3> _vel_err_prev;			/**< velocity on previous step */

	/**
	 * Update our local parameter cache.
	 */
	int			parameters_update(bool force);

	/**
	 * Update control outputs
	 */
	void		control_update();

	/**
	 * Check for changes in subscribed topics.
	 */
	void		poll_subscriptions();

	static float	scale_control(float ctl, float end, float dz);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main() __attribute__((noreturn));
};

namespace pos_control
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

MulticopterPositionControl	*g_control;
}

MulticopterPositionControl::MulticopterPositionControl() :

	_task_should_exit(false),
	_control_task(-1),

/* subscriptions */
	_att_sub(-1),
	_att_sp_sub(-1),
	_control_mode_sub(-1),
	_params_sub(-1),
	_manual_sub(-1),
	_arming_sub(-1),
	_local_pos_sub(-1),
	_global_pos_sp_sub(-1),

/* publications */
	_local_pos_sp_pub(-1),
	_att_sp_pub(-1),
	_global_vel_sp_pub(-1)
{
	memset(&_att, 0, sizeof(_att));
	memset(&_att_sp, 0, sizeof(_att_sp));
	memset(&_manual, 0, sizeof(_manual));
	memset(&_control_mode, 0, sizeof(_control_mode));
	memset(&_arming, 0, sizeof(_arming));
	memset(&_local_pos, 0, sizeof(_local_pos));
	memset(&_local_pos_sp, 0, sizeof(_local_pos_sp));
	memset(&_global_pos_sp, 0, sizeof(_global_pos_sp));
	memset(&_global_vel_sp, 0, sizeof(_global_vel_sp));

	_params.pos_p.zero();
	_params.vel_p.zero();
	_params.vel_i.zero();
	_params.vel_d.zero();
	_params.vel_max.zero();
	_params.vel_ff.zero();
	_params.sp_offs_max.zero();

	_pos.zero();
	_vel.zero();
	_pos_sp.zero();
	_vel_sp.zero();
	_vel_err_prev.zero();

	_params_handles.takeoff_alt	= param_find("NAV_TAKEOFF_ALT");
	_params_handles.takeoff_gap	= param_find("NAV_TAKEOFF_GAP");
	_params_handles.thr_min		= param_find("MPC_THR_MIN");
	_params_handles.thr_max		= param_find("MPC_THR_MAX");
	_params_handles.z_p			= param_find("MPC_Z_P");
	_params_handles.z_vel_p		= param_find("MPC_Z_VEL_P");
	_params_handles.z_vel_i		= param_find("MPC_Z_VEL_I");
	_params_handles.z_vel_d		= param_find("MPC_Z_VEL_D");
	_params_handles.z_vel_max	= param_find("MPC_Z_VEL_MAX");
	_params_handles.z_ff		= param_find("MPC_Z_FF");
	_params_handles.xy_p		= param_find("MPC_XY_P");
	_params_handles.xy_vel_p	= param_find("MPC_XY_VEL_P");
	_params_handles.xy_vel_i	= param_find("MPC_XY_VEL_I");
	_params_handles.xy_vel_d	= param_find("MPC_XY_VEL_D");
	_params_handles.xy_vel_max	= param_find("MPC_XY_VEL_MAX");
	_params_handles.xy_ff		= param_find("MPC_XY_FF");
	_params_handles.tilt_max	= param_find("MPC_TILT_MAX");
	_params_handles.rc_scale_pitch	= param_find("RC_SCALE_PITCH");
	_params_handles.rc_scale_roll	= param_find("RC_SCALE_ROLL");
	_params_handles.rc_scale_yaw	= param_find("RC_SCALE_YAW");

	/* fetch initial parameter values */
	parameters_update(true);
}

MulticopterPositionControl::~MulticopterPositionControl()
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

	pos_control::g_control = nullptr;
}

int
MulticopterPositionControl::parameters_update(bool force)
{
	bool updated;
	struct parameter_update_s param_upd;

	orb_check(_params_sub, &updated);
	if (updated)
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_upd);

	if (updated || force) {
		param_get(_params_handles.takeoff_alt, &_params.takeoff_alt);
		param_get(_params_handles.takeoff_gap, &_params.takeoff_gap);
		param_get(_params_handles.thr_min, &_params.thr_min);
		param_get(_params_handles.thr_max, &_params.thr_max);
		param_get(_params_handles.tilt_max, &_params.tilt_max);
		param_get(_params_handles.rc_scale_pitch, &_params.rc_scale_pitch);
		param_get(_params_handles.rc_scale_roll, &_params.rc_scale_roll);
		param_get(_params_handles.rc_scale_yaw, &_params.rc_scale_yaw);

		float v;
		param_get(_params_handles.xy_p, &v);
		_params.pos_p(0) = v;
		_params.pos_p(1) = v;
		param_get(_params_handles.z_p, &v);
		_params.pos_p(2) = v;
		param_get(_params_handles.xy_vel_p, &v);
		_params.vel_p(0) = v;
		_params.vel_p(1) = v;
		param_get(_params_handles.z_vel_p, &v);
		_params.vel_p(2) = v;
		param_get(_params_handles.xy_vel_i, &v);
		_params.vel_i(0) = v;
		_params.vel_i(1) = v;
		param_get(_params_handles.z_vel_i, &v);
		_params.vel_i(2) = v;
		param_get(_params_handles.xy_vel_d, &v);
		_params.vel_d(0) = v;
		_params.vel_d(1) = v;
		param_get(_params_handles.z_vel_d, &v);
		_params.vel_d(2) = v;
		param_get(_params_handles.xy_vel_max, &v);
		_params.vel_max(0) = v;
		_params.vel_max(1) = v;
		param_get(_params_handles.z_vel_max, &v);
		_params.vel_max(2) = v;
		param_get(_params_handles.xy_ff, &v);
		_params.vel_ff(0) = v;
		_params.vel_ff(1) = v;
		param_get(_params_handles.z_ff, &v);
		_params.vel_ff(2) = v;

		_params.sp_offs_max = _params.vel_max.edivide(_params.pos_p) * 2.0f;
	}

	return OK;
}

void
MulticopterPositionControl::poll_subscriptions()
{
	bool updated;

	orb_check(_att_sub, &updated);
	if (updated)
		orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);

	orb_check(_att_sp_sub, &updated);
	if (updated)
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);

	orb_check(_control_mode_sub, &updated);
	if (updated)
		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);

	orb_check(_manual_sub, &updated);
	if (updated)
		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);

	orb_check(_arming_sub, &updated);
	if (updated)
		orb_copy(ORB_ID(actuator_armed), _arming_sub, &_arming);

	orb_check(_global_pos_sp_sub, &updated);
	if (updated)
		orb_copy(ORB_ID(vehicle_global_position_setpoint), _global_pos_sp_sub, &_global_pos_sp);
}

float
MulticopterPositionControl::scale_control(float ctl, float end, float dz)
{
	if (ctl > dz) {
		return (ctl - dz) / (end - dz);

	} else if (ctl < -dz) {
		return (ctl + dz) / (end - dz);

	} else {
		return 0.0f;
	}
}

void
MulticopterPositionControl::task_main_trampoline(int argc, char *argv[])
{
	pos_control::g_control->task_main();
}

void
MulticopterPositionControl::task_main()
{
	warnx("started");

	static int mavlink_fd;
	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	mavlink_log_info(mavlink_fd, "[mpc] started");

	/*
	 * do subscriptions
	 */
	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_arming_sub = orb_subscribe(ORB_ID(actuator_armed));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_global_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_global_position_setpoint));

	parameters_update(true);

	/* initialize values of critical structs until first regular update */
	_arming.armed = false;

	/* get an initial update for all sensor and status data */
	poll_subscriptions();

	bool reset_mission_sp = false;
	bool global_pos_sp_valid = false;
	bool reset_man_sp_z = true;
	bool reset_man_sp_xy = true;
	bool reset_int_z = true;
	bool reset_int_z_manual = false;
	bool reset_int_xy = true;
	bool was_armed = false;
	bool reset_auto_sp_xy = true;
	bool reset_auto_sp_z = true;
	bool reset_takeoff_sp = true;

	hrt_abstime t_prev = 0;

	const float alt_ctl_dz = 0.2f;
	const float pos_ctl_dz = 0.05f;

	float ref_alt = 0.0f;
	hrt_abstime ref_alt_t = 0;
	hrt_abstime local_ref_timestamp = 0;

	math::Vector<3> sp_move_rate;
	sp_move_rate.zero();
	math::Vector<3> thrust_int;
	thrust_int.zero();
	math::Matrix<3, 3> R;
	R.identity();

	/* wakeup source */
	struct pollfd fds[1];

	/* Setup of loop */
	fds[0].fd = _local_pos_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {
		/* wait for up to 500ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0)
			continue;

		/* this is undesirable but not much we can do */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);

		poll_subscriptions();
		parameters_update(false);

		hrt_abstime t = hrt_absolute_time();
		float dt = t_prev != 0 ? (t - t_prev) * 0.000001f : 0.0f;
		t_prev = t;

		if (_control_mode.flag_armed && !was_armed) {
			/* reset setpoints and integrals on arming */
			reset_man_sp_z = true;
			reset_man_sp_xy = true;
			reset_auto_sp_z = true;
			reset_auto_sp_xy = true;
			reset_takeoff_sp = true;
			reset_int_z = true;
			reset_int_xy = true;
		}
		was_armed = _control_mode.flag_armed;

		if (_control_mode.flag_control_altitude_enabled ||
				_control_mode.flag_control_position_enabled ||
				_control_mode.flag_control_climb_rate_enabled ||
				_control_mode.flag_control_velocity_enabled) {

			_pos(0) = _local_pos.x;
			_pos(1) = _local_pos.y;
			_pos(2) = _local_pos.z;
			_vel(0) = _local_pos.vx;
			_vel(1) = _local_pos.vy;
			_vel(2) = _local_pos.vz;

			sp_move_rate.zero();

			if (_control_mode.flag_control_manual_enabled) {
				/* manual control */
				/* check for reference point updates and correct setpoint */
				if (_local_pos.ref_timestamp != ref_alt_t) {
					if (ref_alt_t != 0) {
						/* home alt changed, don't follow large ground level changes in manual flight */
						_pos_sp(2) += _local_pos.ref_alt - ref_alt;
					}

					ref_alt_t = _local_pos.ref_timestamp;
					ref_alt = _local_pos.ref_alt;
					// TODO also correct XY setpoint
				}

				/* reset setpoints to current position if needed */
				if (_control_mode.flag_control_altitude_enabled) {
					if (reset_man_sp_z) {
						reset_man_sp_z = false;
						_pos_sp(2) = _pos(2);
						mavlink_log_info(mavlink_fd, "[mpc] reset alt sp: %.2f", (double) - _pos_sp(2));
					}

					/* move altitude setpoint with throttle stick */
					sp_move_rate(2) = -scale_control(_manual.throttle - 0.5f, 0.5f, alt_ctl_dz);
				}

				if (_control_mode.flag_control_position_enabled) {
					if (reset_man_sp_xy) {
						reset_man_sp_xy = false;
						_pos_sp(0) = _pos(0);
						_pos_sp(1) = _pos(1);
						mavlink_log_info(mavlink_fd, "[mpc] reset pos sp: %.2f, %.2f", (double)_pos_sp(0), (double)_pos_sp(1));
					}

					/* move position setpoint with roll/pitch stick */
					sp_move_rate(0) = scale_control(-_manual.pitch / _params.rc_scale_pitch, 1.0f, pos_ctl_dz);
					sp_move_rate(1) = scale_control(_manual.roll / _params.rc_scale_roll, 1.0f, pos_ctl_dz);
				}

				/* limit setpoint move rate */
				float sp_move_norm = sp_move_rate.length();
				if (sp_move_norm > 1.0f) {
					sp_move_rate /= sp_move_norm;
				}

				/* scale to max speed and rotate around yaw */
				math::Matrix<3, 3> R_yaw_sp;
				R_yaw_sp.from_euler(0.0f, 0.0f, _att_sp.yaw_body);
				sp_move_rate = R_yaw_sp * sp_move_rate.emult(_params.vel_max);

				/* move position setpoint */
				_pos_sp += sp_move_rate * dt;

				/* check if position setpoint is too far from actual position */
				math::Vector<3> pos_sp_offs = (_pos_sp - _pos).edivide(_params.vel_max);
				float pos_sp_offs_norm = pos_sp_offs.length();
				if (pos_sp_offs_norm > 1.0f) {
					pos_sp_offs /= pos_sp_offs_norm;
					_pos_sp = _pos + pos_sp_offs.emult(_params.vel_max);
				}

				/* copy yaw setpoint to vehicle_local_position_setpoint topic */
				_local_pos_sp.yaw = _att_sp.yaw_body;

				/* local position setpoint is valid and can be used for auto loiter after position controlled mode */
				reset_auto_sp_xy = !_control_mode.flag_control_position_enabled;
				reset_auto_sp_z = !_control_mode.flag_control_altitude_enabled;
				reset_takeoff_sp = true;

				/* force reprojection of global setpoint after manual mode */
				reset_mission_sp = true;
			} else {
				// TODO AUTO
				_pos_sp = _pos;
			}

			_local_pos_sp.x = _pos_sp(0);
			_local_pos_sp.y = _pos_sp(1);
			_local_pos_sp.z = _pos_sp(2);

			/* publish local position setpoint */
			if (_local_pos_sp_pub > 0) {
				orb_publish(ORB_ID(vehicle_local_position_setpoint), _local_pos_sp_pub, &_local_pos_sp);

			} else {
				_local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &_local_pos_sp);
			}

			/* run position & altitude controllers, calculate velocity setpoint */
			math::Vector<3> pos_err = _pos_sp - _pos;
			_vel_sp = pos_err.emult(_params.pos_p) + sp_move_rate.emult(_params.vel_ff);

			if (!_control_mode.flag_control_altitude_enabled) {
				reset_man_sp_z = true;
				_vel_sp(2) = 0.0f;
			}

			if (!_control_mode.flag_control_position_enabled) {
				reset_man_sp_xy = true;
				_vel_sp(0) = 0.0f;
				_vel_sp(1) = 0.0f;
			}

			if (!_control_mode.flag_control_manual_enabled) {
				/* limit 3D speed only in AUTO mode */
				float vel_sp_norm = _vel_sp.edivide(_params.vel_max).length();

				if (vel_sp_norm > 1.0f) {
					_vel_sp /= vel_sp_norm;
				}
			}

			_global_vel_sp.vx = _vel_sp(0);
			_global_vel_sp.vy = _vel_sp(1);
			_global_vel_sp.vz = _vel_sp(2);

			/* publish velocity setpoint */
			if (_global_vel_sp_pub > 0) {
				orb_publish(ORB_ID(vehicle_global_velocity_setpoint), _global_vel_sp_pub, &_global_vel_sp);

			} else {
				_global_vel_sp_pub = orb_advertise(ORB_ID(vehicle_global_velocity_setpoint), &_global_vel_sp);
			}

			if (_control_mode.flag_control_climb_rate_enabled || _control_mode.flag_control_velocity_enabled) {
				/* reset integrals if needed */
				if (_control_mode.flag_control_climb_rate_enabled) {
					if (reset_int_z) {
						reset_int_z = false;
						float i = _params.thr_min;

						if (reset_int_z_manual) {
							i = _manual.throttle;

							if (i < _params.thr_min) {
								i = _params.thr_min;

							} else if (i > _params.thr_max) {
								i = _params.thr_max;
							}
						}

						thrust_int(2) = -i;
						mavlink_log_info(mavlink_fd, "[mpc] reset hovering thrust: %.2f", (double)i);
					}
				} else {
					reset_int_z = true;
				}

				if (_control_mode.flag_control_velocity_enabled) {
					if (reset_int_xy) {
						reset_int_xy = false;
						thrust_int(0) = 0.0f;
						thrust_int(1) = 0.0f;
						mavlink_log_info(mavlink_fd, "[mpc] reset xy vel integral");
					}
				} else {
					reset_int_xy = true;
				}

				/* calculate desired thrust vector in NED frame */
				math::Vector<3> vel_err = _vel_sp - _vel;
				math::Vector<3> thrust_sp = vel_err.emult(_params.vel_p) + (vel_err - _vel_err_prev).emult(_params.vel_d) / dt + thrust_int;

				_vel_err_prev = vel_err;

				if (!_control_mode.flag_control_velocity_enabled) {
					thrust_sp(0) = 0.0f;
					thrust_sp(1) = 0.0f;
				}
				if (!_control_mode.flag_control_climb_rate_enabled) {
					thrust_sp(2) = 0.0f;
				}

				/* limit thrust vector and check for saturation */
				bool saturation_xy = false;
				bool saturation_z = false;

				/* limit min lift */
				float thr_min = _params.thr_min;
				if (!_control_mode.flag_control_velocity_enabled && thr_min < 0.0f) {
					/* don't allow downside thrust direction in manual attitude mode */
					thr_min = 0.0f;
				}

				if (-thrust_sp(2) < thr_min) {
					thrust_sp(2) = -thr_min;
					saturation_z = true;
				}

				/* limit max tilt */
				float tilt = atan2f(math::Vector<2>(thrust_sp(0), thrust_sp(1)).length(), -thrust_sp(2));

				if (tilt > _params.tilt_max && _params.thr_min >= 0.0f) {
					/* crop horizontal component */
					float k = tanf(_params.tilt_max) / tanf(tilt);
					thrust_sp(0) *= k;
					thrust_sp(1) *= k;
					saturation_xy = true;
				}

				/* limit max thrust */
				float thrust_abs = thrust_sp.length();

				if (thrust_abs > _params.thr_max) {
					if (thrust_sp(2) < 0.0f) {
						if (-thrust_sp(2) > _params.thr_max) {
							/* thrust Z component is too large, limit it */
							thrust_sp(0) = 0.0f;
							thrust_sp(1) = 0.0f;
							thrust_sp(2) = -_params.thr_max;
							saturation_xy = true;
							saturation_z = true;

						} else {
							/* preserve thrust Z component and lower XY, keeping altitude is more important than position */
							float thrust_xy_max = sqrtf(_params.thr_max * _params.thr_max - thrust_sp(2) * thrust_sp(2));
							float thrust_xy_abs = math::Vector<2>(thrust_sp(0), thrust_sp(1)).length();
							float k = thrust_xy_max / thrust_xy_abs;
							thrust_sp(0) *= k;
							thrust_sp(1) *= k;
							saturation_xy = true;
						}

					} else {
						/* Z component is negative, going down, simply limit thrust vector */
						float k = _params.thr_max / thrust_abs;
						thrust_sp *= k;
						saturation_xy = true;
						saturation_z = true;
					}

					thrust_abs = _params.thr_max;
				}

				/* update integrals */
				math::Vector<3> m;
				m(0) = (_control_mode.flag_control_velocity_enabled && !saturation_xy) ? 1.0f : 0.0f;
				m(1) = m(0);
				m(2) = (_control_mode.flag_control_climb_rate_enabled && !saturation_z) ? 1.0f : 0.0f;

				thrust_int += vel_err.emult(_params.vel_i) * dt;

				/* protection against flipping on ground when landing */
				if (thrust_int(2) > 0.0f)
					thrust_int(2) = 0.0f;

				/* calculate attitude and thrust from thrust vector */
				if (_control_mode.flag_control_velocity_enabled) {
					/* desired body_z axis = -normalize(thrust_vector) */
					math::Vector<3> body_x;
					math::Vector<3> body_y;
					math::Vector<3> body_z;

					if (thrust_abs > SIGMA) {
						body_z = -thrust_sp / thrust_abs;

					} else {
						/* no thrust, set Z axis to safe value */
						body_z.zero();
						body_z(2) = 1.0f;
					}

					/* vector of desired yaw direction in XY plane, rotated by PI/2 */
					math::Vector<3> y_C(-sinf(_att_sp.yaw_body), cosf(_att_sp.yaw_body), 0.0f);

					if (fabsf(body_z(2)) > SIGMA) {
						/* desired body_x axis, orthogonal to body_z */
						body_x = y_C % body_z;

						/* keep nose to front while inverted upside down */
						if (body_z(2) < 0.0f) {
							body_x = -body_x;
						}
						body_x.normalize();
					} else {
						/* desired thrust is in XY plane, set X downside to construct correct matrix,
						 * but yaw component will not be used actually */
						body_x.zero();
						body_x(2) = 1.0f;
					}

					/* desired body_y axis */
					body_y = body_z % body_x;

					/* fill rotation matrix */
					for (int i = 0; i < 3; i++) {
						R(i, 0) = body_x(i);
						R(i, 1) = body_y(i);
						R(i, 2) = body_z(i);
					}

					/* copy rotation matrix to attitude setpoint topic */
					memcpy(&_att_sp.R_body[0][0], R.data, sizeof(_att_sp.R_body));
					_att_sp.R_valid = true;

					/* calculate euler angles, for logging only, must not be used for control */
					math::Vector<3> euler = R.to_euler();
					_att_sp.roll_body = euler(0);
					_att_sp.pitch_body = euler(1);
					/* yaw already used to construct rot matrix, but actual rotation matrix can have different yaw near singularity */

				} else {
					/* thrust compensation for altitude only control mode */
					float att_comp;

					if (_att.R[2][2] > TILT_COS_MAX)
						att_comp = 1.0f / _att.R[2][2];
					else if (_att.R[2][2] > 0.0f)
						att_comp = ((1.0f / TILT_COS_MAX - 1.0f) / TILT_COS_MAX) * _att.R[2][2] + 1.0f;
					else
						att_comp = 1.0f;

					thrust_abs *= att_comp;
				}

				_att_sp.thrust = thrust_abs;

				_att_sp.timestamp = hrt_absolute_time();

				/* publish attitude setpoint */
				if (_att_sp_pub > 0) {
					orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &_att_sp);

				} else {
					_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_att_sp);
				}

			} else {
				reset_int_z = true;
			}
		} else {
			/* position controller disabled, reset setpoints */
			reset_man_sp_z = true;
			reset_man_sp_xy = true;
			reset_int_z = true;
			reset_int_xy = true;
			reset_mission_sp = true;
			reset_auto_sp_xy = true;
			reset_auto_sp_z = true;
		}

		/* reset altitude controller integral (hovering throttle) to manual throttle after manual throttle control */
		reset_int_z_manual = _control_mode.flag_armed && _control_mode.flag_control_manual_enabled && !_control_mode.flag_control_climb_rate_enabled;
	}

	warnx("stopped");
	mavlink_log_info(mavlink_fd, "[mpc] stopped");

	_control_task = -1;
	_exit(0);
}

int
MulticopterPositionControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = task_spawn_cmd("mc_pos_control",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       2048,
				       (main_t)&MulticopterPositionControl::task_main_trampoline,
				       nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int mc_pos_control_main(int argc, char *argv[])
{
	if (argc < 1)
		errx(1, "usage: mc_pos_control {start|stop|status}");

	if (!strcmp(argv[1], "start")) {

		if (pos_control::g_control != nullptr)
			errx(1, "already running");

		pos_control::g_control = new MulticopterPositionControl;

		if (pos_control::g_control == nullptr)
			errx(1, "alloc failed");

		if (OK != pos_control::g_control->start()) {
			delete pos_control::g_control;
			pos_control::g_control = nullptr;
			err(1, "start failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (pos_control::g_control == nullptr)
			errx(1, "not running");

		delete pos_control::g_control;
		pos_control::g_control = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (pos_control::g_control) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}
