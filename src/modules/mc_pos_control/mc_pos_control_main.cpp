/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: @author Anton Babushkin <anton.babushkin@me.com>
 *   Author: @author Thomas Gubler <thomasgubler@gmail.com>
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
 *
 *  @author Anton Babushkin <anton.babushkin@me.com>
 *  @author Thomas Gubler <thomasgubler@gmail.com>
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
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
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
#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#define TILT_COS_MAX	0.7f
#define SIGMA			0.000001f

/**
 * Multicopter position control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_pos_control_main(int argc, char *argv[]);

class MulticopterPositionControl : public control::SuperBlock
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

	uORB::Publication<vehicle_attitude_setpoint_s> _att_sp;       /**< attitude setpoint publication */
	uORB::Publication<position_setpoint_triplet_s> _pos_sp_triplet;	/**< position setpoint triplet publication */
	uORB::Publication<vehicle_global_velocity_setpoint_s> _global_vel_sp;	/**< vehicle global velocity setpoint */

	uORB::Subscription<vehicle_attitude_s> 			_att;			/**< vehicle attitude */
	uORB::Subscription<manual_control_setpoint_s>		_manual;		/**< r/c channel data */
	uORB::Subscription<vehicle_control_mode_s>		_control_mode;		/**< vehicle control mode */
	uORB::Subscription<actuator_armed_s>			_arming;		/**< actuator arming status */
	uORB::Subscription<vehicle_global_position_s>		_global_pos;		/**< vehicle global position */

	control::BlockParamFloat _thr_min;
	control::BlockParamFloat _thr_max;
	control::BlockParamFloat _z_p;
	control::BlockParamFloat _z_vel_p;
	control::BlockParamFloat _z_vel_i;
	control::BlockParamFloat _z_vel_d;
	control::BlockParamFloat _z_vel_max;
	control::BlockParamFloat _z_ff;
	control::BlockParamFloat _xy_p;
	control::BlockParamFloat _xy_vel_p;
	control::BlockParamFloat _xy_vel_i;
	control::BlockParamFloat _xy_vel_d;
	control::BlockParamFloat _xy_vel_max;
	control::BlockParamFloat _xy_ff;
	control::BlockParamFloat _tilt_max;
	control::BlockParamFloat _land_speed;
	control::BlockParamFloat _land_tilt_max;

	control::BlockParamFloat _rc_scale_pitch;
	control::BlockParamFloat _rc_scale_roll;

	struct {
		math::Vector<3> pos_p;
		math::Vector<3> vel_p;
		math::Vector<3> vel_i;
		math::Vector<3> vel_d;
		math::Vector<3> vel_ff;
		math::Vector<3> vel_max;
		math::Vector<3> sp_offs_max;
	} _params_vec;

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
	void updateParams();

	/**
	 * Update control outputs
	 */
	void		control_update();

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

/* Super block */
	SuperBlock(NULL, "MPC"),

/* uORB publications */
	_att_sp(&getPublications(), ORB_ID(vehicle_global_position)),
	_pos_sp_triplet(&getPublications(), ORB_ID(vehicle_global_position)),
	_global_vel_sp(&getPublications(), ORB_ID(vehicle_global_position)),

	/* uORB subscriptions */
	_att(&getSubscriptions(), ORB_ID(vehicle_attitude), 1000),
	_manual(&getSubscriptions(), ORB_ID(manual_control_setpoint), 1000),
	_control_mode(&getSubscriptions(), ORB_ID(vehicle_control_mode), 1000),
	_arming(&getSubscriptions(), ORB_ID(actuator_armed), 1000),
	_global_pos(&getSubscriptions(), ORB_ID(vehicle_global_position), 1000),

	/* params */
	_thr_min(this, "THR_MIN"),
	_thr_max(this, "THR_MAX"),
	_z_p(this, "Z_P"),
	_z_vel_p(this, "Z_VEL_P"),
	_z_vel_i(this, "Z_VEL_I"),
	_z_vel_d(this, "Z_VEL_D"),
	_z_vel_max(this, "Z_VEL_MAX"),
	_z_ff(this, "Z_FF"),
	_xy_p(this, "XY_P"),
	_xy_vel_p(this, "XY_VEL_P"),
	_xy_vel_i(this, "XY_VEL_I"),
	_xy_vel_d(this, "XY_VEL_D"),
	_xy_vel_max(this, "XY_VEL_MAX"),
	_xy_ff(this, "XY_FF"),
	_tilt_max(this, "TILT_MAX"),
	_land_speed(this, "LAND_SPEED"),
	_land_tilt_max(this, "LAND_TILT"),
	_rc_scale_pitch(this, "RC_SCALE_PITCH", false),
	_rc_scale_roll(this, "RC_SCALE_ROLL", false),

	_lat_sp(0.0),
	_lon_sp(0.0),
	_alt_sp(0.0f),

	_reset_lat_lon_sp(true),
	_reset_alt_sp(true),
	_use_global_alt(false)
{
	/* Initialize all vectors to zero */
	_params_vec.pos_p.zero();
	_params_vec.vel_p.zero();
	_params_vec.vel_i.zero();
	_params_vec.vel_d.zero();
	_params_vec.vel_max.zero();
	_params_vec.vel_ff.zero();
	_params_vec.sp_offs_max.zero();

	_vel.zero();
	_vel_sp.zero();
	_vel_prev.zero();

	/* fetch initial parameter values */
	updateParams();
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
}

void
MulticopterPositionControl::updateParams()
{
	SuperBlock::updateParams();


	_params_vec.pos_p(0) = _xy_p.get();
	_params_vec.pos_p(1) = _xy_p.get();

	_params_vec.pos_p(2) = _z_p.get();

	_params_vec.vel_p(0) = _xy_vel_p.get();
	_params_vec.vel_p(1) = _xy_vel_p.get();

	_params_vec.vel_p(2) = _z_vel_p.get();

	_params_vec.vel_i(0) = _xy_vel_i.get();
	_params_vec.vel_i(1) = _xy_vel_i.get();

	_params_vec.vel_i(2) = _z_vel_i.get();

	_params_vec.vel_d(0) = _xy_vel_d.get();
	_params_vec.vel_d(1) = _xy_vel_d.get();

	_params_vec.vel_d(2) = _z_vel_d.get();

	_params_vec.vel_max(0) = _xy_vel_max.get();
	_params_vec.vel_max(1) = _xy_vel_max.get();

	_params_vec.vel_max(2) = _z_vel_max.get();

	_params_vec.vel_ff(0) = _xy_ff.get();
	_params_vec.vel_ff(1) = _xy_ff.get();

	_params_vec.vel_ff(2) = _z_ff.get();

	_params_vec.sp_offs_max = _params_vec.vel_max.edivide(_params_vec.pos_p) * 2.0f;
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

	updateParams();

	/* initialize values of critical structs until first regular update */
	_arming.armed = false;

	/* get an initial update for all sensor and status data */
	updateSubscriptions();

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
	fds[0].fd = _global_pos.getHandle();
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

		updateSubscriptions();
		updateParams();

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
					sp_move_rate(0) = scale_control(-_manual.pitch / _rc_scale_pitch.get(), 1.0f, pos_ctl_dz);
					sp_move_rate(1) = scale_control(_manual.roll / _rc_scale_roll.get(), 1.0f, pos_ctl_dz);
				}

				/* limit setpoint move rate */
				float sp_move_norm = sp_move_rate.length();

				if (sp_move_norm > 1.0f) {
					sp_move_rate /= sp_move_norm;
				}

				/* scale to max speed and rotate around yaw */
				math::Matrix<3, 3> R_yaw_sp;
				R_yaw_sp.from_euler(0.0f, 0.0f, _att_sp.yaw_body);
				sp_move_rate = R_yaw_sp * sp_move_rate.emult(_params_vec.vel_max);

				/* move position setpoint */
				add_vector_to_global_position(_lat_sp, _lon_sp, sp_move_rate(0) * dt, sp_move_rate(1) * dt, &_lat_sp, &_lon_sp);
				_alt_sp -= sp_move_rate(2) * dt;

				/* check if position setpoint is too far from actual position */
				math::Vector<3> pos_sp_offs;
				pos_sp_offs.zero();

				if (_control_mode.flag_control_position_enabled) {
					get_vector_to_next_waypoint_fast(_global_pos.lat, _global_pos.lon, _lat_sp, _lon_sp, &pos_sp_offs.data[0], &pos_sp_offs.data[1]);
					pos_sp_offs(0) /= _params_vec.sp_offs_max(0);
					pos_sp_offs(1) /= _params_vec.sp_offs_max(1);
				}

				if (_control_mode.flag_control_altitude_enabled) {
					pos_sp_offs(2) = -(_alt_sp - alt) / _params_vec.sp_offs_max(2);
				}

				float pos_sp_offs_norm = pos_sp_offs.length();

				if (pos_sp_offs_norm > 1.0f) {
					pos_sp_offs /= pos_sp_offs_norm;
					add_vector_to_global_position(_global_pos.lat, _global_pos.lon, pos_sp_offs(0) * _params_vec.sp_offs_max(0), pos_sp_offs(1) * _params_vec.sp_offs_max(1), &_lat_sp, &_lon_sp);
					_alt_sp = alt - pos_sp_offs(2) * _params_vec.sp_offs_max(2);
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
				_pos_sp_triplet.update();

			} else {
				/* always use AMSL altitude for AUTO */
				select_alt(true);

				_pos_sp_triplet.update();

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
				_att_sp.update();

			} else {
				/* run position & altitude controllers, calculate velocity setpoint */
				math::Vector<3> pos_err;
				get_vector_to_next_waypoint_fast(_global_pos.lat, _global_pos.lon, _lat_sp, _lon_sp, &pos_err.data[0], &pos_err.data[1]);
				pos_err(2) = -(_alt_sp - alt);

				_vel_sp = pos_err.emult(_params_vec.pos_p) + sp_move_rate.emult(_params_vec.vel_ff);

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
					_vel_sp(2) = _land_speed.get();
				}

				if (!_control_mode.flag_control_manual_enabled) {
					/* limit 3D speed only in non-manual modes */
					float vel_sp_norm = _vel_sp.edivide(_params_vec.vel_max).length();

					if (vel_sp_norm > 1.0f) {
						_vel_sp /= vel_sp_norm;
					}
				}

				_global_vel_sp.vx = _vel_sp(0);
				_global_vel_sp.vy = _vel_sp(1);
				_global_vel_sp.vz = _vel_sp(2);

				/* publish velocity setpoint */
				_global_vel_sp.update();

				if (_control_mode.flag_control_climb_rate_enabled || _control_mode.flag_control_velocity_enabled) {
					/* reset integrals if needed */
					if (_control_mode.flag_control_climb_rate_enabled) {
						if (reset_int_z) {
							reset_int_z = false;
							float i = _thr_min.get();

							if (reset_int_z_manual) {
								i = _manual.throttle;

								if (i < _thr_min.get()) {
									i = _thr_min.get();

								} else if (i > _thr_max.get()) {
									i = _thr_max.get();
								}
							}

							thrust_int(2) = -i;
						}

					} else {
						reset_int_z = true;
					}

					if (_control_mode.flag_control_velocity_enabled) {
						if (reset_int_xy) {
							reset_int_xy = false;
							thrust_int(0) = 0.0f;
							thrust_int(1) = 0.0f;
						}

					} else {
						reset_int_xy = true;
					}

					/* velocity error */
					math::Vector<3> vel_err = _vel_sp - _vel;

					/* derivative of velocity error, not includes setpoint acceleration */
					math::Vector<3> vel_err_d = (sp_move_rate - _vel).emult(_params_vec.pos_p) - (_vel - _vel_prev) / dt;
					_vel_prev = _vel;

					/* thrust vector in NED frame */
					math::Vector<3> thrust_sp = vel_err.emult(_params_vec.vel_p) + vel_err_d.emult(_params_vec.vel_d) + thrust_int;

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
					float thr_min = _thr_min.get();

					if (!_control_mode.flag_control_velocity_enabled && thr_min < 0.0f) {
						/* don't allow downside thrust direction in manual attitude mode */
						thr_min = 0.0f;
					}

					float tilt_max = _tilt_max.get();

					/* adjust limits for landing mode */
					if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid &&
					    _pos_sp_triplet.current.type == SETPOINT_TYPE_LAND) {
						/* limit max tilt and min lift when landing */
						tilt_max = _land_tilt_max.get();

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

					if (thrust_abs > _thr_max.get()) {
						if (thrust_sp(2) < 0.0f) {
							if (-thrust_sp(2) > _thr_max.get()) {
								/* thrust Z component is too large, limit it */
								thrust_sp(0) = 0.0f;
								thrust_sp(1) = 0.0f;
								thrust_sp(2) = -_thr_max.get();
								saturation_xy = true;
								saturation_z = true;

							} else {
								/* preserve thrust Z component and lower XY, keeping altitude is more important than position */
								float thrust_xy_max = sqrtf(_thr_max.get() * _thr_max.get() - thrust_sp(2) * thrust_sp(2));
								float thrust_xy_abs = math::Vector<2>(thrust_sp(0), thrust_sp(1)).length();
								float k = thrust_xy_max / thrust_xy_abs;
								thrust_sp(0) *= k;
								thrust_sp(1) *= k;
								saturation_xy = true;
							}

						} else {
							/* Z component is negative, going down, simply limit thrust vector */
							float k = _thr_max.get() / thrust_abs;
							thrust_sp *= k;
							saturation_xy = true;
							saturation_z = true;
						}

						thrust_abs = _thr_max.get();
					}

					/* update integrals */
					if (_control_mode.flag_control_velocity_enabled && !saturation_xy) {
						thrust_int(0) += vel_err(0) * _params_vec.vel_i(0) * dt;
						thrust_int(1) += vel_err(1) * _params_vec.vel_i(1) * dt;
					}

					if (_control_mode.flag_control_climb_rate_enabled && !saturation_z) {
						thrust_int(2) += vel_err(2) * _params_vec.vel_i(2) * dt;

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
					_att_sp.update();

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
