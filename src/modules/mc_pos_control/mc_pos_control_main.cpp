/****************************************************************************
 *
 *   Copyright (c) 2013 - 2017 PX4 Development Team. All rights reserved.
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
 * Original publication for the desired attitude generation:
 * Daniel Mellinger and Vijay Kumar. Minimum Snap Trajectory Generation and Control for Quadrotors.
 * Int. Conf. on Robotics and Automation, Shanghai, China, May 2011
 *
 * Also inspired by https://pixhawk.org/firmware/apps/fw_pos_control_l1
 *
 * The controller has two loops: P loop for position error and PID loop for velocity error.
 * Output of velocity controller is thrust vector that splitted to thrust direction
 * (i.e. rotation matrix for multicopter orientation) and thrust module (i.e. multicopter thrust itself).
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_module_params.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <drivers/drv_hrt.h>
#include <systemlib/hysteresis/hysteresis.h>

#include <uORB/topics/home_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>

#include <float.h>
#include <lib/ecl/geo/geo.h>
#include <mathlib/mathlib.h>
#include <systemlib/mavlink_log.h>

#include <controllib/blocks.hpp>

#include <lib/FlightTasks/FlightTasks.hpp>
#include "PositionControl.hpp"
#include "Utility/ControlMath.hpp"

#define SIGMA_SINGLE_OP			0.000001f
#define SIGMA_NORM			0.001f
/**
 * Multicopter position control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_pos_control_main(int argc, char *argv[]);

class MulticopterPositionControl : public control::SuperBlock, public ModuleParams
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

	bool		cross_sphere_line(const matrix::Vector3f &sphere_c, const float sphere_r,
					  const matrix::Vector3f &line_a, const matrix::Vector3f &line_b, matrix::Vector3f &res);

private:

	/** Time in us that direction change condition has to be true for direction change state */
	static constexpr uint64_t DIRECTION_CHANGE_TRIGGER_TIME_US = 100000;

	bool		_task_should_exit = false;			/**<true if task should exit */
	bool 		_in_smooth_takeoff = false; 				/**<true if takeoff ramp is applied */

	int		_control_task;			/**< task handle for task */
	orb_advert_t	_mavlink_log_pub;		/**< mavlink log advert */

	int		_vehicle_status_sub;		/**< vehicle status subscription */
	int		_vehicle_land_detected_sub;	/**< vehicle land detected subscription */
	int		_vehicle_attitude_sub;		/**< control state subscription */
	int		_control_mode_sub;		/**< vehicle control mode subscription */
	int		_params_sub;			/**< notification of parameter updates */
	int		_manual_sub;			/**< notification of manual control updates */
	int		_local_pos_sub;			/**< vehicle local position */
	int		_pos_sp_triplet_sub;		/**< position setpoint triplet */
	int		_home_pos_sub; 			/**< home position */

	orb_advert_t	_att_sp_pub;			/**< attitude setpoint publication */
	orb_advert_t	_local_pos_sp_pub;		/**< vehicle local position setpoint publication */

	orb_id_t _attitude_setpoint_id;

	struct vehicle_status_s 			_vehicle_status; 	/**< vehicle status */
	struct vehicle_land_detected_s 			_vehicle_land_detected;	/**< vehicle land detected */
	struct vehicle_attitude_s			_att;			/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct vehicle_control_mode_s			_control_mode;		/**< vehicle control mode */
	struct vehicle_local_position_s			_local_pos;		/**< vehicle local position */
	struct position_setpoint_triplet_s		_pos_sp_triplet;	/**< vehicle global position setpoint triplet */
	struct vehicle_local_position_setpoint_s	_local_pos_sp;		/**< vehicle local position setpoint */
	struct home_position_s				_home_pos; 				/**< home position */

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MPC_FLT_TSK>) _test_flight_tasks, /**< temporary flag for the transition to flight tasks */
		(ParamFloat<px4::params::MPC_MANTHR_MIN>) _manual_thr_min, /**< minimal throttle output when flying in manual mode */
		(ParamFloat<px4::params::MPC_MANTHR_MAX>) _manual_thr_max, /**< maximal throttle output when flying in manual mode */
		(ParamFloat<px4::params::MPC_XY_MAN_EXPO>)
		_xy_vel_man_expo, /**< ratio of exponential curve for stick input in xy direction pos mode */
		(ParamFloat<px4::params::MPC_Z_MAN_EXPO>)
		_z_vel_man_expo, /**< ratio of exponential curve for stick input in xy direction pos mode */
		(ParamFloat<px4::params::MPC_HOLD_DZ>)
		_hold_dz, /**< deadzone around the center for the sticks when flying in position mode */
		(ParamFloat<px4::params::MPC_ACC_HOR_MAX>)
		_acceleration_hor_max, /**<maximum velocity setpoint slewrate for auto & fast manual brake */
		(ParamFloat<px4::params::MPC_ACC_HOR>)
		_acceleration_hor, /**<acceleration for auto and maximum for manual in velocity control mode*/
		(ParamFloat<px4::params::MPC_DEC_HOR_SLOW>)
		_deceleration_hor_slow, /**< slow velocity setpoint slewrate for manual deceleration*/
		(ParamFloat<px4::params::MPC_ACC_UP_MAX>) _acceleration_z_max_up, /** max acceleration up */
		(ParamFloat<px4::params::MPC_ACC_DOWN_MAX>) _acceleration_z_max_down, /** max acceleration down */
		(ParamFloat<px4::params::MPC_CRUISE_90>)
		_cruise_speed_90, /**<speed when angle is 90 degrees between prev-current/current-next*/
		(ParamFloat<px4::params::MPC_VEL_MANUAL>)
		_velocity_hor_manual, /**< target velocity in manual controlled mode at full speed*/
		(ParamFloat<px4::params::NAV_ACC_RAD>)
		_nav_rad, /**< radius that is used by navigator that defines when to update triplets */
		(ParamFloat<px4::params::MPC_TKO_RAMP_T>) _takeoff_ramp_time, /**< time contant for smooth takeoff ramp */
		(ParamFloat<px4::params::MPC_JERK_MAX>)
		_jerk_hor_max, /**< maximum jerk in manual controlled mode when braking to zero */
		(ParamFloat<px4::params::MPC_JERK_MIN>)
		_jerk_hor_min, /**< minimum jerk in manual controlled mode when braking to zero */
		(ParamFloat<px4::params::MIS_YAW_ERR>)
		_mis_yaw_error, /**< yaw error threshold that is used in mission as update criteria */

		(ParamFloat<px4::params::MPC_THR_MIN>) _thr_min,
		(ParamFloat<px4::params::MPC_THR_MAX>) _thr_max,
		(ParamFloat<px4::params::MPC_THR_HOVER>) _thr_hover,
		(ParamFloat<px4::params::MPC_Z_P>) _z_p,
		(ParamFloat<px4::params::MPC_Z_VEL_P>) _z_vel_p,
		(ParamFloat<px4::params::MPC_Z_VEL_I>) _z_vel_i,
		(ParamFloat<px4::params::MPC_Z_VEL_D>) _z_vel_d,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _vel_max_up,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _vel_max_down,
		(ParamFloat<px4::params::MPC_LAND_ALT1>) _slow_land_alt1,
		(ParamFloat<px4::params::MPC_LAND_ALT2>) _slow_land_alt2,
		(ParamFloat<px4::params::MPC_XY_P>) _xy_p,
		(ParamFloat<px4::params::MPC_XY_VEL_P>) _xy_vel_p,
		(ParamFloat<px4::params::MPC_XY_VEL_I>) _xy_vel_i,
		(ParamFloat<px4::params::MPC_XY_VEL_D>) _xy_vel_d,
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>) _vel_max_xy_param,
		(ParamFloat<px4::params::MPC_XY_CRUISE>) _vel_cruise_xy,
		(ParamFloat<px4::params::MPC_TILTMAX_AIR>) _tilt_max_air_deg,
		(ParamFloat<px4::params::MPC_LAND_SPEED>) _land_speed,
		(ParamFloat<px4::params::MPC_TKO_SPEED>) _tko_speed,
		(ParamFloat<px4::params::MPC_TILTMAX_LND>) _tilt_max_land_deg,
		(ParamFloat<px4::params::MPC_MAN_TILT_MAX>) _man_tilt_max_deg,
		(ParamFloat<px4::params::MPC_MAN_Y_MAX>) _man_yaw_max_deg,
		(ParamFloat<px4::params::MC_YAWRATE_MAX>) _global_yaw_max_deg,
		(ParamFloat<px4::params::MC_YAW_P>) _mc_att_yaw_p,
		(ParamFloat<px4::params::MPC_HOLD_MAX_XY>) _hold_max_xy,
		(ParamFloat<px4::params::MPC_HOLD_MAX_Z>) _hold_max_z,
		(ParamInt<px4::params::MPC_ALT_MODE>) _alt_mode,
		(ParamFloat<px4::params::RC_FLT_CUTOFF>) _rc_flt_cutoff,
		(ParamFloat<px4::params::RC_FLT_SMP_RATE>) _rc_flt_smp_rate,
		(ParamFloat<px4::params::MPC_ACC_HOR_FLOW>) _acc_max_flow_xy
	);

	control::BlockDerivative _vel_x_deriv;
	control::BlockDerivative _vel_y_deriv;
	control::BlockDerivative _vel_z_deriv;

	FlightTasks _flight_tasks; /**< class handling all ways to generate position controller setpoints */
	PositionControl _control; /**< class handling the core PID position controller */

	systemlib::Hysteresis _manual_direction_change_hysteresis;

	math::LowPassFilter2p _filter_manual_pitch;
	math::LowPassFilter2p _filter_manual_roll;

	enum manual_stick_input {
		brake,
		direction_change,
		acceleration,
		deceleration
	};

	manual_stick_input _user_intention_xy; /**< defines what the user intends to do derived from the stick input */
	manual_stick_input
	_user_intention_z; /**< defines what the user intends to do derived from the stick input in z direciton */

	matrix::Vector3f _pos_p;
	matrix::Vector3f _vel_p;
	matrix::Vector3f _vel_i;
	matrix::Vector3f _vel_d;
	float _tilt_max_air; /**< maximum tilt angle [rad] */
	float _tilt_max_land; /**< maximum tilt angle during landing [rad] */
	float _man_tilt_max;
	float _man_yaw_max;
	float _global_yaw_max;

	struct map_projection_reference_s _ref_pos;
	float _ref_alt;
	bool _ref_alt_is_global; /** true when the reference altitude is defined in a global reference frame */
	hrt_abstime _ref_timestamp;
	hrt_abstime _last_warn;

	matrix::Vector3f _thrust_int;
	matrix::Vector3f _pos;
	matrix::Vector3f _pos_sp;
	matrix::Vector3f _vel;
	matrix::Vector3f _vel_sp;
	matrix::Vector3f _vel_prev;			/**< velocity on previous step */
	matrix::Vector3f _vel_sp_prev;
	matrix::Vector3f _vel_err_d;		/**< derivative of current velocity */
	matrix::Vector3f _curr_pos_sp;  /**< current setpoint of the triplets */
	matrix::Vector3f _prev_pos_sp; /**< previous setpoint of the triples */
	matrix::Vector2f _stick_input_xy_prev; /**< for manual controlled mode to detect direction change */

	matrix::Dcmf _R;			/**< rotation matrix from attitude quaternions */
	float _yaw;				/**< yaw angle (euler) */
	float _yaw_takeoff;	/**< home yaw angle present when vehicle was taking off (euler) */
	float _man_yaw_offset; /**< current yaw offset in manual mode */

	float _vel_max_xy;  /**< equal to vel_max except in auto mode when close to target */
	bool _vel_sp_significant; /** true when the velocity setpoint is over 50% of the _vel_max_xy limit */
	float _acceleration_state_dependent_xy; /**< acceleration limit applied in manual mode */
	float _acceleration_state_dependent_z; /**< acceleration limit applied in manual mode in z */
	float _manual_jerk_limit_xy; /**< jerk limit in manual mode dependent on stick input */
	float _manual_jerk_limit_z; /**< jerk limit in manual mode in z */
	float _z_derivative; /**< velocity in z that agrees with position rate */

	float _takeoff_vel_limit; /**< velocity limit value which gets ramped up */

	float _min_hagl_limit; /**< minimum continuous height above ground (m) */

	float _takeoff_speed; /**< For flighttask interface used only. It can be thrust or velocity setpoints */
	// counters for reset events on position and velocity states
	// they are used to identify a reset event
	uint8_t _z_reset_counter;
	uint8_t _xy_reset_counter;
	uint8_t _heading_reset_counter;

	matrix::Dcmf _R_setpoint;

	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update(bool force);

	/**
	 * Check for changes in subscribed topics.
	 */
	void		poll_subscriptions();

	float		throttle_curve(float ctl, float ctr);

	void update_velocity_derivative(const float &velocity_z);

	/**
	 * Limit altitude based on landdetector.
	 */
	void limit_altitude(vehicle_local_position_setpoint_s &setpoint);

	void warn_rate_limited(const char *str);

	void set_idle_state();

	/**
	 * New methods for flighttask
	 */
	void publish_attitude();

	void publish_local_pos_sp();

	void check_takeoff_state(const float &z, const float &vz);

	void update_takeoff_setpoint(const float &z, const float &vz);

	void limit_thrust_during_landing(matrix::Vector3f &thrust_sepoint);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static int	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main();
};

namespace pos_control
{
MulticopterPositionControl	*g_control;
}


MulticopterPositionControl::MulticopterPositionControl() :
	SuperBlock(nullptr, "MPC"),
	ModuleParams(nullptr),
	_control_task(-1),
	_mavlink_log_pub(nullptr),

	/* subscriptions */
	_vehicle_attitude_sub(-1),
	_control_mode_sub(-1),
	_params_sub(-1),
	_manual_sub(-1),
	_local_pos_sub(-1),
	_pos_sp_triplet_sub(-1),
	_home_pos_sub(-1),

	/* publications */
	_att_sp_pub(nullptr),
	_local_pos_sp_pub(nullptr),
	_attitude_setpoint_id(nullptr),
	_vehicle_status{},
	_vehicle_land_detected{},
	_att{},
	_att_sp{},
	_manual{},
	_control_mode{},
	_local_pos{},
	_pos_sp_triplet{},
	_local_pos_sp{},
	_home_pos{},
	_vel_x_deriv(this, "VELD"),
	_vel_y_deriv(this, "VELD"),
	_vel_z_deriv(this, "VELD"),
	_control(this),
	_manual_direction_change_hysteresis(false),
	_filter_manual_pitch(50.0f, 10.0f),
	_filter_manual_roll(50.0f, 10.0f),
	_user_intention_xy(brake),
	_user_intention_z(brake),
	_ref_alt(0.0f),
	_ref_alt_is_global(false),
	_ref_timestamp(0),
	_last_warn(0),
	_yaw(0.0f),
	_yaw_takeoff(0.0f),
	_man_yaw_offset(0.0f),
	_vel_max_xy(0.0f),
	_vel_sp_significant(false),
	_acceleration_state_dependent_xy(0.0f),
	_acceleration_state_dependent_z(0.0f),
	_manual_jerk_limit_xy(1.0f),
	_manual_jerk_limit_z(1.0f),
	_z_derivative(0.0f),
	_takeoff_vel_limit(0.0f),
	_min_hagl_limit(0.0f),
	_z_reset_counter(0),
	_xy_reset_counter(0),
	_heading_reset_counter(0)
{
	/* Make the attitude quaternion valid */
	_att.q[0] = 1.0f;

	_ref_pos = {};

	/* set trigger time for manual direction change detection */
	_manual_direction_change_hysteresis.set_hysteresis_time_from(false, DIRECTION_CHANGE_TRIGGER_TIME_US);

	_pos.zero();
	_pos_sp.zero();
	_vel.zero();
	_vel_sp.zero();
	_vel_prev.zero();
	_vel_sp_prev.zero();
	_vel_err_d.zero();
	_curr_pos_sp.zero();
	_prev_pos_sp.zero();
	_stick_input_xy_prev.zero();

	_R.identity();
	_R_setpoint.identity();

	_thrust_int.zero();

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
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	pos_control::g_control = nullptr;
}

void
MulticopterPositionControl::warn_rate_limited(const char *string)
{
	hrt_abstime now = hrt_absolute_time();

	if (now - _last_warn > 200000) {
		PX4_WARN(string);
		_last_warn = now;
	}
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
		ModuleParams::updateParams();
		SuperBlock::updateParams();

		_flight_tasks.handleParameterUpdate();

		/* initialize vectors from params and enforce constraints */

		_pos_p(0) = _xy_p.get();
		_pos_p(1) = _xy_p.get();
		_pos_p(2) = _z_p.get();

		_vel_p(0) = _xy_vel_p.get();
		_vel_p(1) = _xy_vel_p.get();
		_vel_p(2) = _z_vel_p.get();

		_vel_i(0) = _xy_vel_i.get();
		_vel_i(1) = _xy_vel_i.get();
		_vel_i(2) = _z_vel_i.get();

		_vel_d(0) = _xy_vel_d.get();
		_vel_d(1) = _xy_vel_d.get();
		_vel_d(2) = _z_vel_d.get();

		_thr_hover.set(math::constrain(_thr_hover.get(), _thr_min.get(), _thr_max.get()));

		_tilt_max_air = math::radians(_tilt_max_air_deg.get());
		_tilt_max_land = math::radians(_tilt_max_land_deg.get());

		_hold_max_xy.set(math::max(0.f, _hold_max_xy.get()));
		_hold_max_z.set(math::max(0.f, _hold_max_z.get()));
		_rc_flt_smp_rate.set(math::max(1.0f, _rc_flt_smp_rate.get()));
		/* make sure the filter is in its stable region -> fc < fs/2 */
		_rc_flt_cutoff.set(math::min(_rc_flt_cutoff.get(), (_rc_flt_smp_rate.get() / 2.0f) - 1.f));

		/* update filters */
		_filter_manual_pitch.set_cutoff_frequency(_rc_flt_smp_rate.get(), _rc_flt_cutoff.get());
		_filter_manual_roll.set_cutoff_frequency(_rc_flt_smp_rate.get(), _rc_flt_cutoff.get());

		/* make sure that vel_cruise_xy is always smaller than vel_max */
		_vel_cruise_xy.set(math::min(_vel_cruise_xy.get(), _vel_max_xy_param.get()));

		/* mc attitude control parameters*/
		_slow_land_alt1.set(math::max(_slow_land_alt1.get(), _slow_land_alt2.get()));

		/* manual control scale */
		_man_tilt_max = math::radians(_man_tilt_max_deg.get());
		_man_yaw_max = math::radians(_man_yaw_max_deg.get());
		_global_yaw_max = math::radians(_global_yaw_max_deg.get());

		/* takeoff and land velocities should not exceed maximum */
		_tko_speed.set(math::min(_tko_speed.get(), _vel_max_up.get()));
		_land_speed.set(math::min(_land_speed.get(), _vel_max_down.get()));

		/* default limit for acceleration and manual jerk*/
		_acceleration_state_dependent_xy = _acceleration_hor_max.get();
		_manual_jerk_limit_xy = _jerk_hor_max.get();

		/* acceleration up must be larger than acceleration down */
		if (_acceleration_z_max_up.get() < _acceleration_z_max_down.get()) {
			_acceleration_z_max_up.set(_acceleration_z_max_down.get());
		}

		/* acceleration horizontal max > deceleration hor */
		if (_acceleration_hor_max.get() < _deceleration_hor_slow.get()) {
			_acceleration_hor_max.set(_deceleration_hor_slow.get());
		}

		/* for z direction we use fixed jerk for now
		 * TODO: check if other jerk value is required */
		_acceleration_state_dependent_z = _acceleration_z_max_up.get();
		/* we only use jerk for braking if jerk_hor_max > jerk_hor_min; otherwise just set jerk very large */
		_manual_jerk_limit_z = (_jerk_hor_max.get() > _jerk_hor_min.get()) ? _jerk_hor_max.get() : 1000000.f;


		/* Get parameter values used to fly within optical flow sensor limits */
		param_t handle = param_find("SENS_FLOW_MINRNG");

		if (handle != PARAM_INVALID) {
			param_get(handle, &_min_hagl_limit);
		}

	}

	return OK;
}

void
MulticopterPositionControl::poll_subscriptions()
{
	bool updated;

	orb_check(_vehicle_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_attitude_setpoint_id) {
			if (_vehicle_status.is_vtol) {
				_attitude_setpoint_id = ORB_ID(mc_virtual_attitude_setpoint);

			} else {
				_attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
			}
		}
	}

	orb_check(_vehicle_land_detected_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);
	}

	orb_check(_vehicle_attitude_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_att);

		/* get current rotation matrix and euler angles from control state quaternions */
		_R = matrix::Quatf(_att.q);
		_yaw = matrix::Eulerf(_R).psi();

		if (_control_mode.flag_control_manual_enabled) {
			if (_heading_reset_counter != _att.quat_reset_counter) {

				_heading_reset_counter = _att.quat_reset_counter;

				// we only extract the heading change from the delta quaternion
				_att_sp.yaw_body += matrix::Eulerf(matrix::Quatf(_att.delta_q_reset)).psi();
			}
		}
	}

	orb_check(_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
	}

	orb_check(_manual_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}

	orb_check(_local_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);

		// check if a reset event has happened
		// if the vehicle is in manual mode we will shift the setpoints of the
		// states which were reset. In auto mode we do not shift the setpoints
		// since we want the vehicle to track the original state.
		if (_control_mode.flag_control_manual_enabled) {
			if (_z_reset_counter != _local_pos.z_reset_counter) {
				_pos_sp(2) = _local_pos.z;
			}

			if (_xy_reset_counter != _local_pos.xy_reset_counter) {
				_pos_sp(0) = _local_pos.x;
				_pos_sp(1) = _local_pos.y;
			}
		}

		// update the reset counters in any case
		_z_reset_counter = _local_pos.z_reset_counter;
		_xy_reset_counter = _local_pos.xy_reset_counter;
	}

	orb_check(_pos_sp_triplet_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);

		/* to be a valid current triplet, altitude has to be finite */

		if (!PX4_ISFINITE(_pos_sp_triplet.current.alt)) {
			_pos_sp_triplet.current.valid = false;
		}

		/* to be a valid previous triplet, lat/lon/alt has to be finite */

		if (!PX4_ISFINITE(_pos_sp_triplet.previous.lat) ||
		    !PX4_ISFINITE(_pos_sp_triplet.previous.lon) ||
		    !PX4_ISFINITE(_pos_sp_triplet.previous.alt)) {
			_pos_sp_triplet.previous.valid = false;
		}
	}

	orb_check(_home_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(home_position), _home_pos_sub, &_home_pos);
	}
}

float
MulticopterPositionControl::throttle_curve(float ctl, float ctr)
{
	/* piecewise linear mapping: 0:ctr -> 0:0.5
	 * and ctr:1 -> 0.5:1 */
	if (ctl < 0.5f) {
		return 2 * ctl * ctr;

	} else {
		return ctr + 2 * (ctl - 0.5f) * (1.0f - ctr);
	}
}

int
MulticopterPositionControl::task_main_trampoline(int argc, char *argv[])
{
	pos_control::g_control->task_main();
	return 0;
}

void
MulticopterPositionControl::limit_altitude(vehicle_local_position_setpoint_s &setpoint)
{
	if (_vehicle_land_detected.alt_max < 0.0f) {
		// there is no altitude limitation present
		return;
	}

	float altitude_above_home = -(_pos(2) - _home_pos.z);

	if (altitude_above_home > _vehicle_land_detected.alt_max) {
		// we are above maximum altitude
		setpoint.z = -_vehicle_land_detected.alt_max +  _home_pos.z;
		setpoint.vz = 0.0f;

	} else if (setpoint.vz <= 0.0f) {
		// we want to fly upwards: check if vehicle does not exceed altitude

		float delta_p = _vehicle_land_detected.alt_max - altitude_above_home;

		if (fabsf(setpoint.vz) * _dt > delta_p) {
			setpoint.z = -_vehicle_land_detected.alt_max +  _home_pos.z;
			setpoint.vz = 0.0f;
		}
	}
}

bool
MulticopterPositionControl::cross_sphere_line(const matrix::Vector3f &sphere_c, const float sphere_r,
		const matrix::Vector3f &line_a, const matrix::Vector3f &line_b, matrix::Vector3f &res)
{
	/* project center of sphere on line */
	/* normalized AB */
	matrix::Vector3f ab_norm = line_b - line_a;

	if (ab_norm.length() < 0.01f) {
		return true;
	}

	ab_norm.normalize();
	matrix::Vector3f d = line_a + ab_norm * ((sphere_c - line_a) * ab_norm);
	float cd_len = (sphere_c - d).length();

	if (sphere_r > cd_len) {
		/* we have triangle CDX with known CD and CX = R, find DX */
		float dx_len = sqrtf(sphere_r * sphere_r - cd_len * cd_len);

		if ((sphere_c - line_b) * ab_norm > 0.0f) {
			/* target waypoint is already behind us */
			res = line_b;

		} else {
			/* target is in front of us */
			res = d + ab_norm * dx_len; // vector A->B on line
		}

		return true;

	} else {

		/* have no roots, return D */
		res = d; /* go directly to line */

		/* previous waypoint is still in front of us */
		if ((sphere_c - line_a) * ab_norm < 0.0f) {
			res = line_a;
		}

		/* target waypoint is already behind us */
		if ((sphere_c - line_b) * ab_norm > 0.0f) {
			res = line_b;
		}

		return false;
	}
}

void
MulticopterPositionControl::update_velocity_derivative(const float &vz)
{
	/* Update velocity derivative,
	 * independent of the current flight mode
	 */
	if (_local_pos.timestamp == 0) {
		return;
	}

	// TODO: this logic should be in the estimator, not the controller!
	if (PX4_ISFINITE(_local_pos.x) &&
	    PX4_ISFINITE(_local_pos.y) &&
	    PX4_ISFINITE(_local_pos.z)) {

		_pos(0) = _local_pos.x;
		_pos(1) = _local_pos.y;

		if (_alt_mode.get() == 1 && _local_pos.dist_bottom_valid) {
			_pos(2) = -_local_pos.dist_bottom;

		} else {
			_pos(2) = _local_pos.z;
		}
	}

	if (PX4_ISFINITE(_local_pos.vx) &&
	    PX4_ISFINITE(_local_pos.vy) &&
	    PX4_ISFINITE(_local_pos.vz)) {

		_vel(0) = _local_pos.vx;
		_vel(1) = _local_pos.vy;

		if (_alt_mode.get() == 1 && _local_pos.dist_bottom_valid) {
			_vel(2) = -_local_pos.dist_bottom_rate;

		} else {
			_vel(2) = _local_pos.vz;
		}

		if (PX4_ISFINITE(vz) && fabsf(vz) > FLT_EPSILON) {
			/* set velocity to the derivative of position
			 * because it has less bias but blend it in across the landing speed range*/
			float weighting = fminf(fabsf(_vel_sp(2)) / _land_speed.get(), 1.0f);
			_vel(2) = _z_derivative * weighting + _vel(2) * (1.0f - weighting);

		}

	}

	if (PX4_ISFINITE(_local_pos.z_deriv)) {
		_z_derivative = _local_pos.z_deriv;
	};

	_vel_err_d(0) = _vel_x_deriv.update(-_vel(0));

	_vel_err_d(1) = _vel_y_deriv.update(-_vel(1));

	_vel_err_d(2) = _vel_z_deriv.update(-_vel(2));
}

void
MulticopterPositionControl::task_main()
{
	/*
	 * do subscriptions
	 */
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	_home_pos_sub = orb_subscribe(ORB_ID(home_position));

	parameters_update(true);

	/* get an initial update for all sensor and status data */
	poll_subscriptions();

	/* We really need to know from the beginning if we're landed or in-air. */
	orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);

	bool was_landed = true;

	hrt_abstime t_prev = 0;

	// Let's be safe and have the landing gear down by default
	_att_sp.landing_gear = vehicle_attitude_setpoint_s::LANDING_GEAR_DOWN;

	/* wakeup source */
	px4_pollfd_struct_t fds[1];

	fds[0].fd = _local_pos_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {
		/* wait for up to 20ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 20);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
			// Go through the loop anyway to copy manual input at 50 Hz.
		}

		/* this is undesirable but not much we can do */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		poll_subscriptions();

		parameters_update(false);

		hrt_abstime t = hrt_absolute_time();
		const float dt = t_prev != 0 ? (t - t_prev) / 1e6f : 0.004f;
		t_prev = t;

		/* set dt for control blocks */
		setDt(dt);

		switch (_vehicle_status.nav_state) {
		case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
			_flight_tasks.switchTask(FlightTaskIndex::Altitude);
			break;

		case vehicle_status_s::NAVIGATION_STATE_POSCTL:
			_flight_tasks.switchTask(FlightTaskIndex::Position);
			break;

		case vehicle_status_s::NAVIGATION_STATE_MANUAL:
			_flight_tasks.switchTask(FlightTaskIndex::Stabilized);
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
		case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
		case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
		case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
		case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:

			/*TODO: clean up navigation state and commander state, which both share too many equal states */
			_flight_tasks.switchTask(FlightTaskIndex::AutoLine);
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
			_flight_tasks.switchTask(FlightTaskIndex::AutoFollowMe);
			break;

		case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
			_flight_tasks.switchTask(FlightTaskIndex::Offboard);
			break;

		default:
			/* not supported yet */
			_flight_tasks.switchTask(FlightTaskIndex::None);
		}


		if (_flight_tasks.isAnyTaskActive()) {

			_flight_tasks.update();
			vehicle_local_position_setpoint_s setpoint = _flight_tasks.getPositionSetpoint();
			vehicle_constraints_s constraints = _flight_tasks.getConstraints();

			update_velocity_derivative(setpoint.vz);
			limit_altitude(setpoint);

			check_takeoff_state(setpoint.z, setpoint.vz);
			update_takeoff_setpoint(setpoint.z, setpoint.vz);

			if (_in_smooth_takeoff) {constraints.speed_up = _takeoff_speed;}

			// We can only run the control if we're already in-air, have a takeoff setpoint, and are not
			// in pure manual. Otherwise just stay idle.
			if (_vehicle_land_detected.landed && !_in_smooth_takeoff && !PX4_ISFINITE(setpoint.thrust[2])) {
				// Keep throttle low
				setpoint.thrust[0] = setpoint.thrust[1] = setpoint.thrust[2] = 0.0f;
				setpoint.yawspeed = 0.0f;
				setpoint.yaw = _yaw;
				constraints.landing_gear = vehicle_constraints_s::GEAR_KEEP;
			}

			// Update states, setpoints and constraints.
			_control.updateConstraints(constraints);
			_control.updateState(_local_pos, matrix::Vector3f(&(_vel_err_d(0))));
			_control.updateSetpoint(setpoint);

			// Generate desired thrust and yaw.
			_control.generateThrustYawSetpoint(_dt);
			matrix::Vector3f thr_sp = _control.getThrustSetpoint();

			// Adjust thrust setpoint based on landdetector only if the
			// vehicle is NOT in pure Manual mode.
			if (!_in_smooth_takeoff && !PX4_ISFINITE(setpoint.thrust[2])) {limit_thrust_during_landing(thr_sp);}

			// Fill local position, velocity and thrust setpoint.
			_local_pos_sp.timestamp = hrt_absolute_time();
			_local_pos_sp.x = _control.getPosSp()(0);
			_local_pos_sp.y = _control.getPosSp()(1);
			_local_pos_sp.z = _control.getPosSp()(2);
			_local_pos_sp.yaw = _control.getYawSetpoint();
			_local_pos_sp.yawspeed = _control.getYawspeedSetpoint();

			_local_pos_sp.vx = _control.getVelSp()(0);
			_local_pos_sp.vy = _control.getVelSp()(1);
			_local_pos_sp.vz = _control.getVelSp()(2);
			thr_sp.copyTo(_local_pos_sp.thrust);

			// Fill attitude setpoint. Attitude is computed from yaw and thrust setpoint.
			_att_sp = ControlMath::thrustToAttitude(thr_sp, _control.getYawSetpoint());
			_att_sp.yaw_sp_move_rate = _control.getYawspeedSetpoint();
			_att_sp.fw_control_yaw = false;
			_att_sp.disable_mc_yaw_control = false;
			_att_sp.apply_flaps = false;

			if (!constraints.landing_gear) {
				if (constraints.landing_gear == vehicle_constraints_s::GEAR_UP) {
					_att_sp.landing_gear = vehicle_attitude_setpoint_s::LANDING_GEAR_UP;
				}

				if (constraints.landing_gear == vehicle_constraints_s::GEAR_DOWN) {
					_att_sp.landing_gear = vehicle_attitude_setpoint_s::LANDING_GEAR_DOWN;
				}
			}

			// Publish local position setpoint (for logging only) and attitude setpoint (for attitude controller).
			publish_local_pos_sp();
			publish_attitude();

		} else {

			/* set default max velocity in xy to vel_max
			 * Apply estimator limits if applicable */
			if (_local_pos.vxy_max > 0.001f) {
				// use the minimum of the estimator and user specified limit
				_vel_max_xy = fminf(_vel_max_xy_param.get(), _local_pos.vxy_max);
				// Allow for a minimum of 0.3 m/s for repositioning
				_vel_max_xy = fmaxf(_vel_max_xy, 0.3f);

			} else if (_vel_sp_significant) {
				// raise the limit at a constant rate up to the user specified value
				if (_vel_max_xy >= _vel_max_xy_param.get()) {
					_vel_max_xy = _vel_max_xy_param.get();

				} else {
					_vel_max_xy += dt * _acc_max_flow_xy.get();
				}
			}

			/* reset flags when landed */
			if (_vehicle_land_detected.landed) {

				/* also reset previous setpoints */
				_yaw_takeoff = _yaw;
				_vel_sp_prev.zero();
				_vel_prev.zero();

				/* make sure attitude setpoint output "disables" attitude control
				 * TODO: we need a defined setpoint to do this properly especially when adjusting the mixer */
				_att_sp.thrust = 0.0f;
				_att_sp.timestamp = hrt_absolute_time();

				/* reset velocity limit */
				_vel_max_xy = _vel_max_xy_param.get();
			}

			/* reset setpoints and integrators VTOL in FW mode */
			if (_vehicle_status.is_vtol && !_vehicle_status.is_rotary_wing) {
				_vel_sp_prev = _vel;
			}

			/* set triplets to invalid if we just landed */
			if (_vehicle_land_detected.landed && !was_landed) {
				_pos_sp_triplet.current.valid = false;
			}

			was_landed = _vehicle_land_detected.landed;

			// reset the horizontal and vertical position hold flags for non-manual modes
			// or if position / altitude is not controlled
			if (!_control_mode.flag_control_position_enabled
			    || !_control_mode.flag_control_manual_enabled) {
			}

			if (!_control_mode.flag_control_altitude_enabled
			    || !_control_mode.flag_control_manual_enabled) {
			}

			if (_control_mode.flag_control_altitude_enabled ||
			    _control_mode.flag_control_position_enabled ||
			    _control_mode.flag_control_climb_rate_enabled ||
			    _control_mode.flag_control_velocity_enabled ||
			    _control_mode.flag_control_acceleration_enabled) {

				/* fill local position, velocity and thrust setpoint */
				_local_pos_sp.timestamp = hrt_absolute_time();
				_local_pos_sp.x = _pos_sp(0);
				_local_pos_sp.y = _pos_sp(1);
				_local_pos_sp.z = _pos_sp(2);
				_local_pos_sp.yaw = _att_sp.yaw_body;
				_local_pos_sp.vx = _vel_sp(0);
				_local_pos_sp.vy = _vel_sp(1);
				_local_pos_sp.vz = _vel_sp(2);

				/* publish local position setpoint */
				if (_local_pos_sp_pub != nullptr) {
					orb_publish(ORB_ID(vehicle_local_position_setpoint), _local_pos_sp_pub, &_local_pos_sp);

				} else {
					_local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &_local_pos_sp);
				}

			} else {
				/* position controller disabled, reset setpoints */

				/* store last velocity in case a mode switch to position control occurs */
				_vel_sp_prev = _vel;
			}

			/* generate attitude setpoint from manual controls */
			if (_control_mode.flag_control_manual_enabled && _control_mode.flag_control_attitude_enabled) {

			} else {
				_att_sp.yaw_sp_move_rate = 0.0f;
			}

			/* update previous velocity for velocity controller D part */
			_vel_prev = _vel;

			/* publish attitude setpoint
			 * Do not publish if
			 * - offboard is enabled but position/velocity/accel control is disabled,
			 * in this case the attitude setpoint is published by the mavlink app.
			 * - if the vehicle is a VTOL and it's just doing a transition (the VTOL attitude control module will generate
			 * attitude setpoints for the transition).
			 */
			if (!(_control_mode.flag_control_offboard_enabled &&
			      !(_control_mode.flag_control_position_enabled ||
				_control_mode.flag_control_velocity_enabled ||
				_control_mode.flag_control_acceleration_enabled))) {

				if (_att_sp_pub != nullptr) {
					orb_publish(_attitude_setpoint_id, _att_sp_pub, &_att_sp);

				} else if (_attitude_setpoint_id) {
					_att_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
				}
			}
		}
	}

	mavlink_log_info(&_mavlink_log_pub, "[mpc] stopped");

	_control_task = -1;
}

void
MulticopterPositionControl::check_takeoff_state(const float &z, const float &vz)
{
	// Check for smooth takeoff
	if (_vehicle_land_detected.landed && !_in_smooth_takeoff
	    && _control_mode.flag_armed) {
		// Vehicle is still landed and no takeoff was initiated yet.
		// Adjust for different takeoff cases.
		// The minimum takeoff altitude needs to be at least 20cm above current position
		if ((PX4_ISFINITE(z) && z < _pos(2) - 0.2f) ||
		    (PX4_ISFINITE(vz) && vz < math::min(-_tko_speed.get(), -0.6f))) {
			// There is a position setpoint above current position or velocity setpoint larger than
			// takeoff speed. Enable smooth takeoff.
			_in_smooth_takeoff = true;
			_takeoff_speed = -0.5f;

		} else {
			// Default
			_in_smooth_takeoff = false;
		}
	}
}

void
MulticopterPositionControl::update_takeoff_setpoint(const float &z, const float &vz)
{
	// If in smooth takeoff, adjust setpoints based on what is valid:
	// 1. position setpoint is valid -> go with takeoffspeed to specific altitude
	// 2. position setpoint not valid but velocity setpoint valid: ramp up velocity
	if (_in_smooth_takeoff) {
		float desired_tko_speed = -vz;

		// If there is a valid position setpoint, then set the desired speed to the takeoff speed.
		if (PX4_ISFINITE(z)) {
			desired_tko_speed = _tko_speed.get();
		}

		// Ramp up takeoff speed.
		_takeoff_speed += desired_tko_speed * _dt / _takeoff_ramp_time.get();
		_takeoff_speed = math::min(_takeoff_speed, desired_tko_speed);

		// Smooth takeoff is achieved once desired altitude/velocity setpoint is reached.
		if (PX4_ISFINITE(z)) {
			_in_smooth_takeoff = _pos(2) + 0.2f > z;

		} else  {
			_in_smooth_takeoff = _takeoff_speed < -vz;
		}

	} else {
		_in_smooth_takeoff = false;
	}
}

void
MulticopterPositionControl::limit_thrust_during_landing(matrix::Vector3f &thr_sp)
{
	if (_vehicle_land_detected.ground_contact) {
		// Set thrust in xy to zero
		thr_sp(0) = 0.0f;
		thr_sp(1) = 0.0f;
		// Reset integral in xy is required because PID-controller does
		// know about the overwrite and would therefore increase the intragral term
		_control.resetIntegralXY();
	}

	if (_vehicle_land_detected.maybe_landed) {
		// we set thrust to zero
		// this will help to decide if we are actually landed or not
		thr_sp.zero();
		// We need to reset all integral terms otherwise the PID-controller
		// will continue to integrate
		_control.resetIntegralXY();
		_control.resetIntegralZ();
	}
}

void
MulticopterPositionControl::publish_attitude()
{
	/* publish attitude setpoint
	 * Do not publish if
	 * - offboard is enabled but position/velocity/accel control is disabled,
	 * in this case the attitude setpoint is published by the mavlink app.
	 * - if the vehicle is a VTOL and it's just doing a transition (the VTOL attitude control module will generate
	 * attitude setpoints for the transition).
	 * - if not armed
	 */
	if (_control_mode.flag_armed &&
	    (!(_control_mode.flag_control_offboard_enabled &&
	       !(_control_mode.flag_control_position_enabled ||
		 _control_mode.flag_control_velocity_enabled ||
		 _control_mode.flag_control_acceleration_enabled)))) {

		_att_sp.timestamp = hrt_absolute_time();

		if (_att_sp_pub != nullptr) {
			orb_publish(_attitude_setpoint_id, _att_sp_pub, &_att_sp);

		} else if (_attitude_setpoint_id) {
			_att_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
		}
	}
}

void
MulticopterPositionControl::publish_local_pos_sp()
{

	_local_pos_sp.timestamp = hrt_absolute_time();

	/* publish local position setpoint */
	if (_local_pos_sp_pub != nullptr) {
		orb_publish(ORB_ID(vehicle_local_position_setpoint),
			    _local_pos_sp_pub, &_local_pos_sp);

	} else {
		_local_pos_sp_pub = orb_advertise(
					    ORB_ID(vehicle_local_position_setpoint),
					    &_local_pos_sp);
	}
}

int
MulticopterPositionControl::start()
{
	/* start the task */
	_control_task = px4_task_spawn_cmd("mc_pos_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_POSITION_CONTROL,
					   1900,
					   (px4_main_t)&MulticopterPositionControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int mc_pos_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: mc_pos_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (pos_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		pos_control::g_control = new MulticopterPositionControl;

		if (pos_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != pos_control::g_control->start()) {
			delete pos_control::g_control;
			pos_control::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (pos_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete pos_control::g_control;
		pos_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (pos_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
