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
 * @file mc_pos_control_main.cpp
 * Multicopter position controller.
 *
 * The controller has two loops: P loop for position error and PID loop for velocity error.
 * Output of velocity controller is thrust vector that splitted to thrust direction
 * (i.e. rotation matrix for multicopter orientation) and thrust module (i.e. multicopter thrust itself).
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
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
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/target_global_position.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <mavlink/mavlink_log.h>

#define TILT_COS_MAX	0.7f
#define SIGMA			0.000001f
#define FOLLOW_OFFS_XY_MIN	2.0f

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
	int		_local_pos_sub;			/**< vehicle local position */
	int		_pos_sp_triplet_sub;		/**< position setpoint triplet */
	int		_target_pos_sub;			/**< target global position subscription */

	orb_advert_t	_att_sp_pub;			/**< attitude setpoint publication */
	orb_advert_t	_local_pos_sp_pub;		/**< vehicle local position setpoint publication */
	orb_advert_t	_global_vel_sp_pub;		/**< vehicle global velocity setpoint publication */
	orb_advert_t	_cam_control_pub;		/**< camera control publication */

	struct vehicle_attitude_s			_att;			/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s	_att_sp;		/**< vehicle attitude setpoint */
	struct manual_control_setpoint_s	_manual;		/**< r/c channel data */
	struct vehicle_control_mode_s		_control_mode;	/**< vehicle control mode */
	struct actuator_armed_s				_arming;		/**< actuator arming status */
	struct vehicle_local_position_s		_local_pos;		/**< vehicle local position */
	struct position_setpoint_triplet_s		_pos_sp_triplet;	/**< vehicle global position setpoint triplet */
	struct vehicle_local_position_setpoint_s	_local_pos_sp;		/**< vehicle local position setpoint */
	struct vehicle_global_velocity_setpoint_s	_global_vel_sp;	/**< vehicle global velocity setpoint */
	struct target_global_position_s		_target_pos;	/**< target global position */
	struct actuator_controls_s			_cam_control;	/**< camera control */

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
		param_t follow_ff;
		param_t follow_dist;
		param_t follow_alt_offs;
		param_t follow_scale_yaw;

		param_t rc_scale_pitch;
		param_t rc_scale_roll;
		param_t rc_scale_yaw;
	}		_params_handles;		/**< handles for interesting parameters */

	struct {
		float thr_min;
		float thr_max;
		float tilt_max;
		float land_speed;
		float land_tilt_max;
		float follow_ff;
		float follow_dist;
		float follow_alt_offs;
		float follow_scale_yaw;

		math::Vector<3> pos_p;
		math::Vector<3> vel_p;
		math::Vector<3> vel_i;
		math::Vector<3> vel_d;
		math::Vector<3> vel_ff;
		math::Vector<3> vel_max;
		math::Vector<3> sp_offs_max;

		float rc_scale_pitch;
		float rc_scale_roll;
		float rc_scale_yaw;
	}		_params;

	struct map_projection_reference_s _ref_pos;
	float _ref_alt;
	hrt_abstime _ref_timestamp;
	float _target_alt_offs;		/**< target altitude offset, add this value to target altitude */

	bool _reset_pos_sp;
	bool _reset_alt_sp;
	bool _reset_follow_offset;

	math::Vector<3> _pos;
	math::Vector<3> _pos_sp;
	math::Vector<3> _vel;
	math::Vector<3> _vel_sp;
	math::Vector<3> _vel_prev;			/**< velocity on previous step */
	math::Vector<3> _vel_ff;

	math::Vector<3> _tpos;
	math::Vector<3> _tvel;

	math::Vector<3> _sp_move_rate;
	math::Vector<3> _att_rates_ff;

	math::Vector<3> _follow_offset;		/**< offset from target for FOLLOW mode, vector in NED frame */

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
	 * Update reference for local position projection
	 */
	void		update_ref();
	/**
	 * Reset position setpoint to current position
	 */
	void		reset_pos_sp();

	/**
	 * Reset altitude setpoint to current altitude
	 */
	void		reset_alt_sp();

	/**
	 * Reset follow offset to current offset.
	 */
	void		reset_follow_offset();

	/**
	 * Control setpoint if "follow target" mode
	 */
	void		control_sp_follow(float dt);

	/**
	 * Control camera and copter yaw depending on mode
	 */
	void		control_camera();

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
	_local_pos_sub(-1),
	_pos_sp_triplet_sub(-1),

/* publications */
	_att_sp_pub(-1),
	_local_pos_sp_pub(-1),
	_global_vel_sp_pub(-1),

	_ref_alt(0.0f),
	_ref_timestamp(0),
	_target_alt_offs(0.0f),

	_reset_pos_sp(true),
	_reset_alt_sp(true),
	_reset_follow_offset(true)
{
	memset(&_att, 0, sizeof(_att));
	memset(&_att_sp, 0, sizeof(_att_sp));
	memset(&_manual, 0, sizeof(_manual));
	memset(&_control_mode, 0, sizeof(_control_mode));
	memset(&_arming, 0, sizeof(_arming));
	memset(&_local_pos, 0, sizeof(_local_pos));
	memset(&_pos_sp_triplet, 0, sizeof(_pos_sp_triplet));
	memset(&_local_pos_sp, 0, sizeof(_local_pos_sp));
	memset(&_global_vel_sp, 0, sizeof(_global_vel_sp));
	memset(&_target_pos, 0, sizeof(_target_pos));
	memset(&_cam_control, 0, sizeof(_cam_control));

	memset(&_ref_pos, 0, sizeof(_ref_pos));

	_params.pos_p.zero();
	_params.vel_p.zero();
	_params.vel_i.zero();
	_params.vel_d.zero();
	_params.vel_max.zero();
	_params.vel_ff.zero();
	_params.sp_offs_max.zero();

	_pos.zero();
	_pos_sp.zero();
	_vel.zero();
	_vel_sp.zero();
	_vel_prev.zero();
	_vel_ff.zero();

	_tpos.zero();
	_tvel.zero();

	_sp_move_rate.zero();
	_att_rates_ff.zero();

	/* initialize to safe value to avoid flying into target */
	_follow_offset.zero();
	_follow_offset(2) = -20.0f;

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
	_params_handles.follow_ff	= param_find("MPC_FOLLOW_FF");
	_params_handles.follow_dist	= param_find("MPC_FOLLOW_DIST");
	_params_handles.follow_alt_offs	= param_find("MPC_FOLLOW_AOFF");
	_params_handles.follow_scale_yaw	= param_find("MPC_FOLLOW_YAW");

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

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_upd);
	}

	if (updated || force) {
		param_get(_params_handles.thr_min, &_params.thr_min);
		param_get(_params_handles.thr_max, &_params.thr_max);
		param_get(_params_handles.tilt_max, &_params.tilt_max);
		param_get(_params_handles.land_speed, &_params.land_speed);
		param_get(_params_handles.land_tilt_max, &_params.land_tilt_max);
		param_get(_params_handles.follow_ff, &_params.follow_ff);
		param_get(_params_handles.follow_dist, &_params.follow_dist);
		param_get(_params_handles.follow_alt_offs, &_params.follow_alt_offs);
		param_get(_params_handles.follow_scale_yaw, &_params.follow_scale_yaw);

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

		param_get(_params_handles.rc_scale_pitch, &_params.rc_scale_pitch);
		param_get(_params_handles.rc_scale_roll, &_params.rc_scale_roll);
		param_get(_params_handles.rc_scale_yaw, &_params.rc_scale_yaw);
	}

	return OK;
}

void
MulticopterPositionControl::poll_subscriptions()
{
	bool updated;

	orb_check(_att_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);
	}

	orb_check(_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
	}

	orb_check(_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
	}

	orb_check(_manual_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}

	orb_check(_arming_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _arming_sub, &_arming);
	}

	orb_check(_local_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
	}

	orb_check(_target_pos_sub, &updated);

	if (updated)
		orb_copy(ORB_ID(target_global_position), _target_pos_sub, &_target_pos);
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
MulticopterPositionControl::update_ref()
{
	if (_local_pos.ref_timestamp != _ref_timestamp) {
		if (_ref_timestamp != 0) {
			/* reproject local position setpoint to new reference */
			float dx, dy;
			map_projection_project(&_ref_pos, _local_pos.ref_lat, _local_pos.ref_lon, &dx, &dy);
			_pos_sp(0) -= dx;
			_pos_sp(1) -= dy;
		}

		_ref_timestamp = _local_pos.ref_timestamp;

		map_projection_init(&_ref_pos, _local_pos.ref_lat, _local_pos.ref_lon);
		_ref_alt = _local_pos.ref_alt;
	}
}

void
MulticopterPositionControl::reset_pos_sp()
{
	if (_reset_pos_sp) {
		_reset_pos_sp = false;
		_pos_sp(0) = _pos(0);
		_pos_sp(1) = _pos(1);
		mavlink_log_info(_mavlink_fd, "[mpc] reset pos sp: %.2f, %.2f", (double)_pos_sp(0), (double)_pos_sp(1));
	}
}

void
MulticopterPositionControl::reset_follow_offset()
{
	if (_reset_follow_offset) {
		_reset_follow_offset = false;

		/* use current position or position setpoint */
		math::Vector<3> pos;
		if (_reset_pos_sp) {
			pos(0) = _pos(0);
			pos(1) = _pos(1);

		} else {
			pos(0) = _pos_sp(0);
			pos(1) = _pos_sp(1);
		}
		pos(2) = _reset_alt_sp ? _pos(2) : _pos_sp(2);

		_follow_offset = pos - _tpos;

		mavlink_log_info(_mavlink_fd, "[mpc] reset follow offs: %.2f, %.2f, %.2f", _follow_offset(0), _follow_offset(1), _follow_offset(2));
	}
}


void
MulticopterPositionControl::reset_alt_sp()
{
	if (_reset_alt_sp) {
		_reset_alt_sp = false;
		_pos_sp(2) = _pos(2);
		mavlink_log_info(_mavlink_fd, "[mpc] reset alt sp: %.2f", -(double)_pos_sp(2));
	}
}

void
MulticopterPositionControl::control_sp_follow(float dt)
{
	/* follow target, change offset from target instead of moving setpoint directly */
	reset_follow_offset();

	/* new value for _follow_offset vector */
	math::Vector<3> follow_offset_new(_follow_offset);

	/* move follow offset using polar coordinates */
	math::Vector<2> follow_offset_xy(_follow_offset(0), _follow_offset(1));
	math::Vector<2> sp_move_rate_xy(_sp_move_rate(0), _sp_move_rate(1));
	float follow_offset_xy_len = follow_offset_xy.length();

	if (sp_move_rate_xy.length_squared() > 0.0f) {
		if (_control_mode.flag_point_to_target && follow_offset_xy_len > FOLLOW_OFFS_XY_MIN) {
			/* calculate change rate in polar coordinates phi, d */
			float rate_phi = -sp_move_rate_xy(1) / follow_offset_xy_len;
			float rate_d = -sp_move_rate_xy(0);

			/* current direction of offset vector */
			float phi = atan2f(_follow_offset(1), _follow_offset(0));

			/* change length of horizontal component of _follow_offset vector with rate_d */
			follow_offset_new(0) += rate_d * cosf(phi) * dt;
			follow_offset_new(1) += rate_d * sinf(phi) * dt;

			/* rotate _follow_offset around vertical axis with rate_phi */
			math::Matrix<3, 3> R_phi;
			R_phi.from_euler(0.0f, 0.0f, rate_phi * dt);
			follow_offset_new = R_phi * follow_offset_new;

			/* update horizontal components of _sp_move_rate */
			_sp_move_rate(0) = rate_d * cosf(phi) - rate_phi * sinf(phi) * follow_offset_xy_len;
			_sp_move_rate(1) = rate_d * sinf(phi) + rate_phi * cosf(phi) * follow_offset_xy_len;

		} else {
			/* 'point_to_target' disabled or copter is too close to target */
			math::Matrix<3, 3> R_yaw_sp;
			R_yaw_sp.from_euler(0.0f, 0.0f, _att_sp.yaw_body);
			_sp_move_rate = R_yaw_sp * _sp_move_rate;
			follow_offset_new += _sp_move_rate * dt;
		}
	}

	/* change altitude */
	follow_offset_new(2) += _sp_move_rate(2) * dt;

	/* don't allow to get closer than MC_FOLLOW_DIST */
	float follow_offset_len = _follow_offset.length();
	float follow_offset_new_len = follow_offset_new.length();

	if (follow_offset_new_len > _params.follow_dist || follow_offset_new_len > follow_offset_len) {
		_follow_offset = follow_offset_new;

	} else {
		_sp_move_rate.zero();
	}

	_pos_sp = _tpos + _follow_offset;

	/* feed forward manual setpoint move rate with weight vel_ff */
	_vel_ff = _sp_move_rate.emult(_params.vel_ff);

	/* add target velocity to setpoint move rate */
	_sp_move_rate += _tvel;

	/* feed forward target velocity */
	_vel_ff += _tvel * _params.follow_ff;
}

void
MulticopterPositionControl::control_camera()
{
	if (_control_mode.flag_point_to_target) {
		/* change yaw to keep direction to target */
		/* calculate current offset (not offset setpoint) */
		math::Vector<3> current_offset = _pos - _tpos;
		math::Vector<2> current_offset_xy(current_offset(0), current_offset(1));

		/* don't try to rotate near singularity */
		float current_offset_xy_len = current_offset_xy.length();
		if (current_offset_xy_len > FOLLOW_OFFS_XY_MIN) {
			/* calculate yaw setpoint from current positions and control offset with yaw stick */
			_att_sp.yaw_body = _wrap_pi(atan2f(-current_offset_xy(1), -current_offset_xy(0)) + _manual.yaw / _params.rc_scale_yaw * _params.follow_scale_yaw);

			/* feed forward attitude rates */
			math::Vector<2> offs_vel_xy(_vel(0) - _tvel(0), _vel(1) - _tvel(1));
			_att_rates_ff(2) = (current_offset_xy % offs_vel_xy) / current_offset_xy_len / current_offset_xy_len;
		}

		/* control camera pitch in global frame (for BL camera gimbal) */
		_cam_control.control[1] = atan2f(current_offset(2), current_offset_xy_len) + _manual.aux2;

	} else {
		/* manual camera pitch control */
		_cam_control.control[1] = _manual.aux2;
	}

	/* publish camera control */
	if (_cam_control_pub > 0) {
		orb_publish(ORB_ID(actuator_controls_2), _cam_control_pub, &_cam_control);

	} else {
		_cam_control_pub = orb_advertise(ORB_ID(actuator_controls_2), &_cam_control);
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
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	_target_pos_sub = orb_subscribe(ORB_ID(target_global_position));

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

	math::Vector<3> thrust_int;
	thrust_int.zero();
	math::Matrix<3, 3> R;
	R.identity();

	/* wakeup source */
	struct pollfd fds[1];

	fds[0].fd = _local_pos_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {
		/* wait for up to 500ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
			continue;
		}

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
			_reset_pos_sp = true;
			_reset_alt_sp = true;
			_reset_follow_offset = true;
			reset_int_z = true;
			reset_int_xy = true;

			if (_target_pos.valid && _target_pos.timestamp < hrt_absolute_time() + 1000000) {
				_target_alt_offs = _local_pos.ref_alt - _local_pos.z - _target_pos.alt - _params.follow_alt_offs;

			} else {
				_target_alt_offs = 0.0f;
			}
		}

		was_armed = _control_mode.flag_armed;

		update_ref();

		/* update position and velocity vectors */
		_pos(0) = _local_pos.x;
		_pos(1) = _local_pos.y;
		_pos(2) = _local_pos.z;

		_vel(0) = _local_pos.vx;
		_vel(1) = _local_pos.vy;
		_vel(2) = _local_pos.vz;

		/* project target position to local frame */
		map_projection_project(&_ref_pos, _target_pos.lat, _target_pos.lon, &_tpos.data[0], &_tpos.data[1]);
		_tpos(2) = -(_target_pos.alt + _target_alt_offs - _ref_alt);

		_tvel(0) = _target_pos.vel_n;
		_tvel(1) = _target_pos.vel_e;
		_tvel(2) = _target_pos.vel_d;

		/* calculate delay between position estimates for vehicle and target */
		float target_dt = math::constrain(((int64_t)_local_pos.time_gps_usec - (int64_t)_target_pos.time_gps_usec) / 1000000.0f, 0.0f, 1.0f);

		/* target position prediction */
		_tpos += _tvel * target_dt;

		control_camera();

		if (_control_mode.flag_control_altitude_enabled ||
		    _control_mode.flag_control_position_enabled ||
		    _control_mode.flag_control_climb_rate_enabled ||
		    _control_mode.flag_control_velocity_enabled) {

			_vel_ff.zero();
			_sp_move_rate.zero();
			_att_rates_ff.zero();

			/* select control source */
			if (_control_mode.flag_control_manual_enabled) {
				/* manual control */
				if (_control_mode.flag_control_altitude_enabled) {
					/* reset alt setpoint to current altitude if needed */
					reset_alt_sp();

					/* move altitude setpoint with throttle stick */
					_sp_move_rate(2) = -scale_control(_manual.throttle - 0.5f, 0.5f, alt_ctl_dz);
				}

				if (_control_mode.flag_control_position_enabled) {
					/* reset position setpoint to current position if needed */
					reset_pos_sp();

					/* move position setpoint with roll/pitch stick */
					_sp_move_rate(0) = scale_control(-_manual.pitch / _params.rc_scale_pitch, 1.0f, pos_ctl_dz);
					_sp_move_rate(1) = scale_control(_manual.roll / _params.rc_scale_roll, 1.0f, pos_ctl_dz);
				}

				/* limit setpoint move rate */
				float sp_move_norm = _sp_move_rate.length();

				if (sp_move_norm > 1.0f) {
					_sp_move_rate /= sp_move_norm;
				}

				/* scale speed */
				_sp_move_rate = _sp_move_rate.emult(_params.vel_max);

				if (_control_mode.flag_follow_target) {
					/* follow target mode */
					control_sp_follow(dt);

				} else {
					/* normal node, move position setpoint */
					/* rotate moving vector around yaw (use yaw setpoint) */
					math::Matrix<3, 3> R_yaw_sp;
					R_yaw_sp.from_euler(0.0f, 0.0f, _att_sp.yaw_body);
					_sp_move_rate = R_yaw_sp * _sp_move_rate;

					/* feed forward setpoint move rate with weight vel_ff */
					_vel_ff = _sp_move_rate.emult(_params.vel_ff);

					/* move position setpoint */
					_pos_sp += _sp_move_rate * dt;

					_reset_follow_offset = true;
				}

				/* check if position setpoint is too far from actual position */
				math::Vector<3> pos_sp_offs;
				pos_sp_offs.zero();

				if (_control_mode.flag_control_position_enabled) {
					pos_sp_offs(0) = (_pos_sp(0) - _pos(0)) / _params.sp_offs_max(0);
					pos_sp_offs(1) = (_pos_sp(1) - _pos(1)) / _params.sp_offs_max(1);
				}

				if (_control_mode.flag_control_altitude_enabled) {
					pos_sp_offs(2) = (_pos_sp(2) - _pos(2)) / _params.sp_offs_max(2);
				}

				float pos_sp_offs_norm = pos_sp_offs.length();

				if (pos_sp_offs_norm > 1.0f) {
					pos_sp_offs /= pos_sp_offs_norm;
					_pos_sp = _pos + pos_sp_offs.emult(_params.sp_offs_max);
				}

			} else {
				/* AUTO */
				bool updated;
				orb_check(_pos_sp_triplet_sub, &updated);

				if (updated) {
					orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);
				}

				if (_pos_sp_triplet.current.valid) {
					/* in case of interrupted mission don't go to waypoint but stay at current position */
					_reset_pos_sp = true;
					_reset_alt_sp = true;

					/* project setpoint to local frame */
					map_projection_project(&_ref_pos,
							       _pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon,
							       &_pos_sp.data[0], &_pos_sp.data[1]);
					_pos_sp(2) = -(_pos_sp_triplet.current.alt - _ref_alt);

					/* update yaw setpoint if needed */
					if (isfinite(_pos_sp_triplet.current.yaw)) {
						_att_sp.yaw_body = _pos_sp_triplet.current.yaw;
					}

				} else {
					/* no waypoint, loiter, reset position setpoint if needed */
					reset_pos_sp();
					reset_alt_sp();
				}

				_reset_follow_offset = true;
			}

			/* fill local position setpoint */
			_local_pos_sp.x = _pos_sp(0);
			_local_pos_sp.y = _pos_sp(1);
			_local_pos_sp.z = _pos_sp(2);
			_local_pos_sp.yaw = _att_sp.yaw_body;

			/* publish local position setpoint */
			if (_local_pos_sp_pub > 0) {
				orb_publish(ORB_ID(vehicle_local_position_setpoint), _local_pos_sp_pub, &_local_pos_sp);

			} else {
				_local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &_local_pos_sp);
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
				_att_sp.rollrate_ff = 0.0f;
				_att_sp.pitchrate_ff = 0.0f;
				_att_sp.yawrate_ff = 0.0f;

				_att_sp.timestamp = hrt_absolute_time();

				/* publish attitude setpoint */
				if (_att_sp_pub > 0) {
					orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &_att_sp);

				} else {
					_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_att_sp);
				}

			} else {
				/* run position & altitude controllers, calculate velocity setpoint */
				math::Vector<3> pos_err = _pos_sp - _pos;

				_vel_sp = pos_err.emult(_params.pos_p) + _vel_ff;

				if (!_control_mode.flag_control_altitude_enabled) {
					_reset_alt_sp = true;
					_vel_sp(2) = 0.0f;
				}

				if (!_control_mode.flag_control_position_enabled) {
					_reset_pos_sp = true;
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

					/* derivative of velocity error */
					math::Vector<3> vel_err_d = (_sp_move_rate - _vel).emult(_params.pos_p) - (_vel - _vel_prev) / dt;
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

						if (thr_min < 0.0f) {
							thr_min = 0.0f;
						}
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
						if (thrust_int(2) > 0.0f) {
							thrust_int(2) = 0.0f;
						}
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

					/* convert attitude rates from NED to body frame */
					_att_rates_ff = R.transposed() * _att_rates_ff;

					_att_sp.rollrate_ff = _att_rates_ff(0);
					_att_sp.pitchrate_ff = _att_rates_ff(1);
					_att_sp.yawrate_ff = _att_rates_ff(2);
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
			_reset_pos_sp = true;
			_reset_follow_offset = true;
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
	if (argc < 1) {
		errx(1, "usage: mc_pos_control {start|stop|status}");
	}

	if (!strcmp(argv[1], "start")) {

		if (pos_control::g_control != nullptr) {
			errx(1, "already running");
		}

		pos_control::g_control = new MulticopterPositionControl;

		if (pos_control::g_control == nullptr) {
			errx(1, "alloc failed");
		}

		if (OK != pos_control::g_control->start()) {
			delete pos_control::g_control;
			pos_control::g_control = nullptr;
			err(1, "start failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (pos_control::g_control == nullptr) {
			errx(1, "not running");
		}

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
