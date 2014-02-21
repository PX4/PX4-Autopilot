/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: @author Anton Babushkin <anton.babushkin@me.com>
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
 * Multicopter position controller.
 *
 * The controller has two loops: P loop for position error and PID loop for velocity error.
 * Output of velocity controller is thrust vector that splitted to thrust direction
 * (i.e. rotation matrix for multicopter orientation) and thrust module (i.e. multicopter thrust itself).
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
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
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
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
	int		_mavlink_fd;			/**< mavlink fd */

	int		_att_sub;				/**< vehicle attitude subscription */
	int		_att_sp_sub;			/**< vehicle attitude setpoint */
	int		_control_mode_sub;		/**< vehicle control mode subscription */
	int 	_params_sub;			/**< notification of parameter updates */
	int 	_manual_sub;			/**< notification of manual control updates */
	int		_arming_sub;			/**< arming status of outputs */
	int		_global_pos_sub;			/**< vehicle local position */
	int		_pos_sp_triplet_sub;		/**< position setpoint triplet */

	orb_advert_t	_att_sp_pub;			/**< attitude setpoint publication */
	orb_advert_t	_pos_sp_triplet_pub;	/**< position setpoint triplet publication */
	orb_advert_t	_global_vel_sp_pub;		/**< vehicle global velocity setpoint */

	struct vehicle_attitude_s			_att;			/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s	_att_sp;		/**< vehicle attitude setpoint */
	struct manual_control_setpoint_s	_manual;		/**< r/c channel data */
	struct vehicle_control_mode_s		_control_mode;	/**< vehicle control mode */
	struct actuator_armed_s				_arming;		/**< actuator arming status */
	struct vehicle_global_position_s		_global_pos;	/**< vehicle global position */
	struct position_setpoint_triplet_s		_pos_sp_triplet;	/**< vehicle global position setpoint triplet */
	struct vehicle_global_velocity_setpoint_s	_global_vel_sp;	/**< vehicle global velocity setpoint */

	struct {
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
		param_t land_speed;
		param_t land_tilt_max;

		param_t rc_scale_pitch;
		param_t rc_scale_roll;
	}		_params_handles;		/**< handles for interesting parameters */

	struct {
		float thr_min;
		float thr_max;
		float tilt_max;
		float land_speed;
		float land_tilt_max;

		float rc_scale_pitch;
		float rc_scale_roll;

		math::Vector<3> pos_p;
		math::Vector<3> vel_p;
		math::Vector<3> vel_i;
		math::Vector<3> vel_d;
		math::Vector<3> vel_ff;
		math::Vector<3> vel_max;
		math::Vector<3> sp_offs_max;
	}		_params;

	double _lat_sp;
	double _lon_sp;
	float _alt_sp;

	bool _reset_lat_lon_sp;
	bool _reset_alt_sp;
	bool _use_global_alt;			/**< switch between global (AMSL) and barometric altitudes */

	math::Vector<3> _vel;
	math::Vector<3> _vel_sp;
	math::Vector<3> _vel_prev;			/**< velocity on previous step */

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
	 * Reset lat/lon to current position
	 */
	void		reset_lat_lon_sp();

	/**
	 * Reset altitude setpoint to current altitude
	 */
	void		reset_alt_sp();

	/**
	 * Select between barometric and global (AMSL) altitudes
	 */
	void		select_alt(bool global);

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
	_mavlink_fd(-1),

/* subscriptions */
	_att_sub(-1),
	_att_sp_sub(-1),
	_control_mode_sub(-1),
	_params_sub(-1),
	_manual_sub(-1),
	_arming_sub(-1),
	_global_pos_sub(-1),
	_pos_sp_triplet_sub(-1),

/* publications */
	_att_sp_pub(-1),
	_pos_sp_triplet_pub(-1),
	_global_vel_sp_pub(-1),

	_lat_sp(0.0),
	_lon_sp(0.0),
	_alt_sp(0.0f),

	_reset_lat_lon_sp(true),
	_reset_alt_sp(true),
	_use_global_alt(false)
{
	memset(&_att, 0, sizeof(_att));
	memset(&_att_sp, 0, sizeof(_att_sp));
	memset(&_manual, 0, sizeof(_manual));
	memset(&_control_mode, 0, sizeof(_control_mode));
	memset(&_arming, 0, sizeof(_arming));
	memset(&_global_pos, 0, sizeof(_global_pos));
	memset(&_pos_sp_triplet, 0, sizeof(_pos_sp_triplet));
	memset(&_global_vel_sp, 0, sizeof(_global_vel_sp));

	_params.pos_p.zero();
	_params.vel_p.zero();
	_params.vel_i.zero();
	_params.vel_d.zero();
	_params.vel_max.zero();
	_params.vel_ff.zero();
	_params.sp_offs_max.zero();

	_vel.zero();
	_vel_sp.zero();
	_vel_prev.zero();

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
	_params_handles.land_speed	= param_find("MPC_LAND_SPEED");
	_params_handles.land_tilt_max	= param_find("MPC_LAND_TILT");
	_params_handles.rc_scale_pitch	= param_find("RC_SCALE_PITCH");
	_params_handles.rc_scale_roll	= param_find("RC_SCALE_ROLL");

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
		param_get(_params_handles.thr_min, &_params.thr_min);
		param_get(_params_handles.thr_max, &_params.thr_max);
		param_get(_params_handles.tilt_max, &_params.tilt_max);
		param_get(_params_handles.land_speed, &_params.land_speed);
		param_get(_params_handles.land_tilt_max, &_params.land_tilt_max);
		param_get(_params_handles.rc_scale_pitch, &_params.rc_scale_pitch);
		param_get(_params_handles.rc_scale_roll, &_params.rc_scale_roll);

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

	orb_check(_global_pos_sub, &updated);

	if (updated)
		orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
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
MulticopterPositionControl::reset_lat_lon_sp()
{
	if (_reset_lat_lon_sp) {
		_reset_lat_lon_sp = false;
		_lat_sp = _global_pos.lat;
		_lon_sp = _global_pos.lon;
		mavlink_log_info(_mavlink_fd, "[mpc] reset lat/lon sp: %.7f, %.7f", _lat_sp, _lon_sp);
	}
}

void
MulticopterPositionControl::reset_alt_sp()
{
	if (_reset_alt_sp) {
		_reset_alt_sp = false;
		_alt_sp = _use_global_alt ? _global_pos.alt : _global_pos.baro_alt;
		mavlink_log_info(_mavlink_fd, "[mpc] reset alt (%s) sp: %.2f", _use_global_alt ? "AMSL" : "baro", (double)_alt_sp);
	}
}

void
MulticopterPositionControl::select_alt(bool global)
{
	if (global != _use_global_alt) {
		_use_global_alt = global;

		if (global) {
			/* switch from barometric to global altitude */
			_alt_sp += _global_pos.alt - _global_pos.baro_alt;

		} else {
			/* switch from global to barometric altitude */
			_alt_sp += _global_pos.baro_alt - _global_pos.alt;
		}
	}
}

void
MulticopterPositionControl::task_main()
{
	warnx("started");

	_mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	mavlink_log_info(_mavlink_fd, "[mpc] started");

	/*
	 * do subscriptions
	 */
	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_arming_sub = orb_subscribe(ORB_ID(actuator_armed));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));

	parameters_update(true);

	/* initialize values of critical structs until first regular update */
	_arming.armed = false;

	/* get an initial update for all sensor and status data */
	poll_subscriptions();

	bool reset_int_z = true;
	bool reset_int_z_manual = false;
	bool reset_int_xy = true;
	bool was_armed = false;

	hrt_abstime t_prev = 0;

	const float alt_ctl_dz = 0.2f;
	const float pos_ctl_dz = 0.05f;

	math::Vector<3> sp_move_rate;
	sp_move_rate.zero();
	math::Vector<3> thrust_int;
	thrust_int.zero();
	math::Matrix<3, 3> R;
	R.identity();

	/* wakeup source */
	struct pollfd fds[1];

	/* Setup of loop */
	fds[0].fd = _global_pos_sub;
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

		poll_subscriptions();
		parameters_update(false);

		hrt_abstime t = hrt_absolute_time();
		float dt = t_prev != 0 ? (t - t_prev) * 0.000001f : 0.0f;
		t_prev = t;

		if (_control_mode.flag_armed && !was_armed) {
			/* reset setpoints and integrals on arming */
			_reset_lat_lon_sp = true;
			_reset_alt_sp = true;
			reset_int_z = true;
			reset_int_xy = true;
		}

		was_armed = _control_mode.flag_armed;

		if (_control_mode.flag_control_altitude_enabled ||
		    _control_mode.flag_control_position_enabled ||
		    _control_mode.flag_control_climb_rate_enabled ||
		    _control_mode.flag_control_velocity_enabled) {

			_vel(0) = _global_pos.vel_n;
			_vel(1) = _global_pos.vel_e;
			_vel(2) = _global_pos.vel_d;

			sp_move_rate.zero();

			float alt = _global_pos.alt;

			/* select control source */
			if (_control_mode.flag_control_manual_enabled) {
				/* select altitude source and update setpoint */
				select_alt(_global_pos.global_valid);

				if (!_use_global_alt) {
					alt = _global_pos.baro_alt;
				}

				/* manual control */
				if (_control_mode.flag_control_altitude_enabled) {
					/* reset alt setpoint to current altitude if needed */
					reset_alt_sp();

					/* move altitude setpoint with throttle stick */
					sp_move_rate(2) = -scale_control(_manual.throttle - 0.5f, 0.5f, alt_ctl_dz);
				}

				if (_control_mode.flag_control_position_enabled) {
					/* reset lat/lon setpoint to current position if needed */
					reset_lat_lon_sp();

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
				add_vector_to_global_position(_lat_sp, _lon_sp, sp_move_rate(0) * dt, sp_move_rate(1) * dt, &_lat_sp, &_lon_sp);
				_alt_sp -= sp_move_rate(2) * dt;

				/* check if position setpoint is too far from actual position */
				math::Vector<3> pos_sp_offs;
				pos_sp_offs.zero();

				if (_control_mode.flag_control_position_enabled) {
					get_vector_to_next_waypoint_fast(_global_pos.lat, _global_pos.lon, _lat_sp, _lon_sp, &pos_sp_offs.data[0], &pos_sp_offs.data[1]);
					pos_sp_offs(0) /= _params.sp_offs_max(0);
					pos_sp_offs(1) /= _params.sp_offs_max(1);
				}

				if (_control_mode.flag_control_altitude_enabled) {
					pos_sp_offs(2) = -(_alt_sp - alt) / _params.sp_offs_max(2);
				}

				float pos_sp_offs_norm = pos_sp_offs.length();

				if (pos_sp_offs_norm > 1.0f) {
					pos_sp_offs /= pos_sp_offs_norm;
					add_vector_to_global_position(_global_pos.lat, _global_pos.lon, pos_sp_offs(0) * _params.sp_offs_max(0), pos_sp_offs(1) * _params.sp_offs_max(1), &_lat_sp, &_lon_sp);
					_alt_sp = alt - pos_sp_offs(2) * _params.sp_offs_max(2);
				}

				/* fill position setpoint triplet */
				_pos_sp_triplet.previous.valid = true;
				_pos_sp_triplet.current.valid = true;
				_pos_sp_triplet.next.valid = true;

				_pos_sp_triplet.nav_state = NAV_STATE_NONE;
				_pos_sp_triplet.current.type = SETPOINT_TYPE_NORMAL;
				_pos_sp_triplet.current.lat = _lat_sp;
				_pos_sp_triplet.current.lon = _lon_sp;
				_pos_sp_triplet.current.alt = _alt_sp;
				_pos_sp_triplet.current.yaw = _att_sp.yaw_body;
				_pos_sp_triplet.current.loiter_radius = 0.0f;
				_pos_sp_triplet.current.loiter_direction = 1.0f;
				_pos_sp_triplet.current.pitch_min = 0.0f;

				/* publish position setpoint triplet */
				if (_pos_sp_triplet_pub > 0) {
					orb_publish(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_pub, &_pos_sp_triplet);

				} else {
					_pos_sp_triplet_pub = orb_advertise(ORB_ID(position_setpoint_triplet), &_pos_sp_triplet);
				}

			} else {
				/* always use AMSL altitude for AUTO */
				select_alt(true);

				/* AUTO */
				bool updated;
				orb_check(_pos_sp_triplet_sub, &updated);

				if (updated)
					orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);

				if (_pos_sp_triplet.current.valid) {
					/* in case of interrupted mission don't go to waypoint but stay at current position */
					_reset_lat_lon_sp = true;
					_reset_alt_sp = true;

					/* update position setpoint */
					_lat_sp = _pos_sp_triplet.current.lat;
					_lon_sp = _pos_sp_triplet.current.lon;
					_alt_sp = _pos_sp_triplet.current.alt;

					/* update yaw setpoint if needed */
					if (isfinite(_pos_sp_triplet.current.yaw)) {
						_att_sp.yaw_body = _pos_sp_triplet.current.yaw;
					}

				} else {
					/* no waypoint, loiter, reset position setpoint if needed */
					reset_lat_lon_sp();
					reset_alt_sp();
				}
			}

			if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid && _pos_sp_triplet.current.type == SETPOINT_TYPE_IDLE) {
				/* idle state, don't run controller and set zero thrust */
				R.identity();
				memcpy(&_att_sp.R_body[0][0], R.data, sizeof(_att_sp.R_body));
				_att_sp.R_valid = true;

				_att_sp.roll_body = 0.0f;
				_att_sp.pitch_body = 0.0f;
				_att_sp.yaw_body = _att.yaw;
				_att_sp.thrust = 0.0f;

				_att_sp.timestamp = hrt_absolute_time();

				/* publish attitude setpoint */
				if (_att_sp_pub > 0) {
					orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &_att_sp);

				} else {
					_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_att_sp);
				}

			} else {
				/* run position & altitude controllers, calculate velocity setpoint */
				math::Vector<3> pos_err;
				get_vector_to_next_waypoint_fast(_global_pos.lat, _global_pos.lon, _lat_sp, _lon_sp, &pos_err.data[0], &pos_err.data[1]);
				pos_err(2) = -(_alt_sp - alt);

				_vel_sp = pos_err.emult(_params.pos_p) + sp_move_rate.emult(_params.vel_ff);

				if (!_control_mode.flag_control_altitude_enabled) {
					_reset_alt_sp = true;
					_vel_sp(2) = 0.0f;
				}

				if (!_control_mode.flag_control_position_enabled) {
					_reset_lat_lon_sp = true;
					_vel_sp(0) = 0.0f;
					_vel_sp(1) = 0.0f;
				}

				/* use constant descend rate when landing, ignore altitude setpoint */
				if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid && _pos_sp_triplet.current.type == SETPOINT_TYPE_LAND) {
					_vel_sp(2) = _params.land_speed;
				}

				if (!_control_mode.flag_control_manual_enabled) {
					/* limit 3D speed only in non-manual modes */
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
							mavlink_log_info(_mavlink_fd, "[mpc] reset hovering thrust: %.2f", (double)i);
						}

					} else {
						reset_int_z = true;
					}

					if (_control_mode.flag_control_velocity_enabled) {
						if (reset_int_xy) {
							reset_int_xy = false;
							thrust_int(0) = 0.0f;
							thrust_int(1) = 0.0f;
							mavlink_log_info(_mavlink_fd, "[mpc] reset xy vel integral");
						}

					} else {
						reset_int_xy = true;
					}

					/* velocity error */
					math::Vector<3> vel_err = _vel_sp - _vel;

					/* derivative of velocity error, not includes setpoint acceleration */
					math::Vector<3> vel_err_d = (sp_move_rate - _vel).emult(_params.pos_p) - (_vel - _vel_prev) / dt;
					_vel_prev = _vel;

					/* thrust vector in NED frame */
					math::Vector<3> thrust_sp = vel_err.emult(_params.vel_p) + vel_err_d.emult(_params.vel_d) + thrust_int;

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

					float tilt_max = _params.tilt_max;

					/* adjust limits for landing mode */
					if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid &&
					    _pos_sp_triplet.current.type == SETPOINT_TYPE_LAND) {
						/* limit max tilt and min lift when landing */
						tilt_max = _params.land_tilt_max;

						if (thr_min < 0.0f)
							thr_min = 0.0f;
					}

					/* limit min lift */
					if (-thrust_sp(2) < thr_min) {
						thrust_sp(2) = -thr_min;
						saturation_z = true;
					}

					if (_control_mode.flag_control_velocity_enabled) {
						/* limit max tilt */
						if (thr_min >= 0.0f && tilt_max < M_PI / 2 - 0.05f) {
							/* absolute horizontal thrust */
							float thrust_sp_xy_len = math::Vector<2>(thrust_sp(0), thrust_sp(1)).length();

							if (thrust_sp_xy_len > 0.01f) {
								/* max horizontal thrust for given vertical thrust*/
								float thrust_xy_max = -thrust_sp(2) * tanf(tilt_max);

								if (thrust_sp_xy_len > thrust_xy_max) {
									float k = thrust_xy_max / thrust_sp_xy_len;
									thrust_sp(0) *= k;
									thrust_sp(1) *= k;
									saturation_xy = true;
								}
							}
						}

					} else {
						/* thrust compensation for altitude only control mode */
						float att_comp;

						if (_att.R[2][2] > TILT_COS_MAX) {
							att_comp = 1.0f / _att.R[2][2];

						} else if (_att.R[2][2] > 0.0f) {
							att_comp = ((1.0f / TILT_COS_MAX - 1.0f) / TILT_COS_MAX) * _att.R[2][2] + 1.0f;
							saturation_z = true;

						} else {
							att_comp = 1.0f;
							saturation_z = true;
						}

						thrust_sp(2) *= att_comp;
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
					if (_control_mode.flag_control_velocity_enabled && !saturation_xy) {
						thrust_int(0) += vel_err(0) * _params.vel_i(0) * dt;
						thrust_int(1) += vel_err(1) * _params.vel_i(1) * dt;
					}

					if (_control_mode.flag_control_climb_rate_enabled && !saturation_z) {
						thrust_int(2) += vel_err(2) * _params.vel_i(2) * dt;

						/* protection against flipping on ground when landing */
						if (thrust_int(2) > 0.0f)
							thrust_int(2) = 0.0f;
					}

					/* calculate attitude setpoint from thrust vector */
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
			}

		} else {
			/* position controller disabled, reset setpoints */
			_reset_alt_sp = true;
			_reset_lat_lon_sp = true;
			reset_int_z = true;
			reset_int_xy = true;
		}

		/* reset altitude controller integral (hovering throttle) to manual throttle after manual throttle control */
		reset_int_z_manual = _control_mode.flag_armed && _control_mode.flag_control_manual_enabled && !_control_mode.flag_control_climb_rate_enabled;
	}

	warnx("stopped");
	mavlink_log_info(_mavlink_fd, "[mpc] stopped");

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
