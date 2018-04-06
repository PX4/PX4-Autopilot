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
#include <controllib/block/BlockParam.hpp>

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

	bool		cross_sphere_line(const matrix::Vector3f &sphere_c, const float sphere_r,
					  const matrix::Vector3f &line_a, const matrix::Vector3f &line_b, matrix::Vector3f &res);

private:

	/** Time in us that direction change condition has to be true for direction change state */
	static constexpr uint64_t DIRECTION_CHANGE_TRIGGER_TIME_US = 100000;

	bool		_task_should_exit = false;			/**<true if task should exit */
	bool		_gear_state_initialized = false;		/**<true if the gear state has been initialized */
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

	control::BlockParamInt _test_flight_tasks; /**< temporary flag for the transition to flight tasks */
	control::BlockParamFloat _manual_thr_min; /**< minimal throttle output when flying in manual mode */
	control::BlockParamFloat _manual_thr_max; /**< maximal throttle output when flying in manual mode */
	control::BlockParamFloat _xy_vel_man_expo; /**< ratio of exponential curve for stick input in xy direction pos mode */
	control::BlockParamFloat _z_vel_man_expo; /**< ratio of exponential curve for stick input in xy direction pos mode */
	control::BlockParamFloat _hold_dz; /**< deadzone around the center for the sticks when flying in position mode */
	control::BlockParamFloat _acceleration_hor_max; /**<maximum velocity setpoint slewrate for auto & fast manual brake */
	control::BlockParamFloat _acceleration_hor; /**<acceleration for auto and maximum for manual in velocity control mode*/
	control::BlockParamFloat _deceleration_hor_slow; /**< slow velocity setpoint slewrate for manual deceleration*/
	control::BlockParamFloat _acceleration_z_max_up; /** max acceleration up */
	control::BlockParamFloat _acceleration_z_max_down; /** max acceleration down */
	control::BlockParamFloat _cruise_speed_90; /**<speed when angle is 90 degrees between prev-current/current-next*/
	control::BlockParamFloat _velocity_hor_manual; /**< target velocity in manual controlled mode at full speed*/
	control::BlockParamFloat _nav_rad; /**< radius that is used by navigator that defines when to update triplets */
	control::BlockParamFloat _takeoff_ramp_time; /**< time contant for smooth takeoff ramp */
	control::BlockParamFloat _jerk_hor_max; /**< maximum jerk in manual controlled mode when braking to zero */
	control::BlockParamFloat _jerk_hor_min; /**< minimum jerk in manual controlled mode when braking to zero */
	control::BlockParamFloat _mis_yaw_error; /**< yaw error threshold that is used in mission as update criteria */
	control::BlockDerivative _vel_x_deriv;
	control::BlockDerivative _vel_y_deriv;
	control::BlockDerivative _vel_z_deriv;


	FlightTasks _flight_tasks; /**< class handling all ways to generate position controller setpoints */
	PositionControl _control{}; /**< class handling the core PID position controller */

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

	struct {
		param_t thr_min;
		param_t thr_max;
		param_t thr_hover;
		param_t z_p;
		param_t z_vel_p;
		param_t z_vel_i;
		param_t z_vel_d;
		param_t z_vel_max_up;
		param_t z_vel_max_down;
		param_t slow_land_alt1;
		param_t slow_land_alt2;
		param_t xy_p;
		param_t xy_vel_p;
		param_t xy_vel_i;
		param_t xy_vel_d;
		param_t xy_vel_max;
		param_t xy_vel_cruise;
		param_t tilt_max_air;
		param_t land_speed;
		param_t tko_speed;
		param_t tilt_max_land;
		param_t man_tilt_max;
		param_t man_yaw_max;
		param_t global_yaw_max;
		param_t mc_att_yaw_p;
		param_t hold_max_xy;
		param_t hold_max_z;
		param_t alt_mode;
		param_t rc_flt_smp_rate;
		param_t rc_flt_cutoff;
		param_t acc_max_flow_xy;
	}		_params_handles;		/**< handles for interesting parameters */

	struct {
		float thr_min;
		float thr_max;
		float thr_hover;
		float tilt_max_air;
		float land_speed;
		float tko_speed;
		float tilt_max_land;
		float man_tilt_max;
		float man_yaw_max;
		float global_yaw_max;
		float mc_att_yaw_p;
		float hold_max_xy;
		float hold_max_z;
		float vel_max_xy;
		float vel_cruise_xy;
		float vel_max_up;
		float vel_max_down;
		float slow_land_alt1;
		float slow_land_alt2;
		int32_t alt_mode;

		float rc_flt_smp_rate;
		float rc_flt_cutoff;
		float acc_max_flow_xy;

		matrix::Vector3f pos_p;
		matrix::Vector3f vel_p;
		matrix::Vector3f vel_i;
		matrix::Vector3f vel_d;
	} _params{};

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

	/**
	 * Update reference for local position projection
	 */
	void		update_ref();

	void update_velocity_derivative();

	/**
	 * limit altitude based on several conditions
	 */
	void limit_altitude();

	void warn_rate_limited(const char *str);

	void updateTiltConstraints(Controller::Constraints &constrains);

	void publish_attitude();

	void publish_local_pos_sp();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

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
	_test_flight_tasks(this, "FLT_TSK"),
	_manual_thr_min(this, "MANTHR_MIN"),
	_manual_thr_max(this, "MANTHR_MAX"),
	_xy_vel_man_expo(this, "XY_MAN_EXPO"),
	_z_vel_man_expo(this, "Z_MAN_EXPO"),
	_hold_dz(this, "HOLD_DZ"),
	_acceleration_hor_max(this, "ACC_HOR_MAX", true),
	_acceleration_hor(this, "ACC_HOR", true),
	_deceleration_hor_slow(this, "DEC_HOR_SLOW", true),
	_acceleration_z_max_up(this, "ACC_UP_MAX", true),
	_acceleration_z_max_down(this, "ACC_DOWN_MAX", true),
	_cruise_speed_90(this, "CRUISE_90", true),
	_velocity_hor_manual(this, "VEL_MANUAL", true),
	_nav_rad(this, "NAV_ACC_RAD", false),
	_takeoff_ramp_time(this, "TKO_RAMP_T", true),
	_jerk_hor_max(this, "JERK_MAX", true),
	_jerk_hor_min(this, "JERK_MIN", true),
	_mis_yaw_error(this, "MIS_YAW_ERR", false),
	_vel_x_deriv(this, "VELD"),
	_vel_y_deriv(this, "VELD"),
	_vel_z_deriv(this, "VELD"),
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

	_params.pos_p.zero();
	_params.vel_p.zero();
	_params.vel_i.zero();
	_params.vel_d.zero();

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

	_params_handles.thr_min		= param_find("MPC_THR_MIN");
	_params_handles.thr_max		= param_find("MPC_THR_MAX");
	_params_handles.thr_hover	= param_find("MPC_THR_HOVER");
	_params_handles.z_p		= param_find("MPC_Z_P");
	_params_handles.z_vel_p		= param_find("MPC_Z_VEL_P");
	_params_handles.z_vel_i		= param_find("MPC_Z_VEL_I");
	_params_handles.z_vel_d		= param_find("MPC_Z_VEL_D");
	_params_handles.z_vel_max_up	= param_find("MPC_Z_VEL_MAX_UP");
	_params_handles.z_vel_max_down	= param_find("MPC_Z_VEL_MAX_DN");
	_params_handles.xy_p		= param_find("MPC_XY_P");
	_params_handles.xy_vel_p	= param_find("MPC_XY_VEL_P");
	_params_handles.xy_vel_i	= param_find("MPC_XY_VEL_I");
	_params_handles.xy_vel_d	= param_find("MPC_XY_VEL_D");
	_params_handles.xy_vel_max	= param_find("MPC_XY_VEL_MAX");
	_params_handles.xy_vel_cruise	= param_find("MPC_XY_CRUISE");
	_params_handles.slow_land_alt1  = param_find("MPC_LAND_ALT1");
	_params_handles.slow_land_alt2  = param_find("MPC_LAND_ALT2");
	_params_handles.tilt_max_air	= param_find("MPC_TILTMAX_AIR");
	_params_handles.land_speed	= param_find("MPC_LAND_SPEED");
	_params_handles.tko_speed	= param_find("MPC_TKO_SPEED");
	_params_handles.tilt_max_land	= param_find("MPC_TILTMAX_LND");
	_params_handles.man_tilt_max = param_find("MPC_MAN_TILT_MAX");
	_params_handles.man_yaw_max = param_find("MPC_MAN_Y_MAX");
	_params_handles.global_yaw_max = param_find("MC_YAWRATE_MAX");
	_params_handles.mc_att_yaw_p = param_find("MC_YAW_P");
	_params_handles.hold_max_xy = param_find("MPC_HOLD_MAX_XY");
	_params_handles.hold_max_z = param_find("MPC_HOLD_MAX_Z");
	_params_handles.alt_mode = param_find("MPC_ALT_MODE");
	_params_handles.rc_flt_cutoff = param_find("RC_FLT_CUTOFF");
	_params_handles.rc_flt_smp_rate = param_find("RC_FLT_SMP_RATE");
	_params_handles.acc_max_flow_xy = param_find("MPC_ACC_HOR_FLOW");

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
		/* update C++ param system */
		updateParams();

		/* update legacy C interface params */
		param_get(_params_handles.thr_min, &_params.thr_min);
		param_get(_params_handles.thr_max, &_params.thr_max);
		param_get(_params_handles.thr_hover, &_params.thr_hover);
		_params.thr_hover = math::constrain(_params.thr_hover, _params.thr_min, _params.thr_max);
		param_get(_params_handles.tilt_max_air, &_params.tilt_max_air);
		_params.tilt_max_air = math::radians(_params.tilt_max_air);
		param_get(_params_handles.land_speed, &_params.land_speed);
		param_get(_params_handles.tko_speed, &_params.tko_speed);
		param_get(_params_handles.tilt_max_land, &_params.tilt_max_land);
		_params.tilt_max_land = math::radians(_params.tilt_max_land);

		float v;
		int32_t v_i;
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
		param_get(_params_handles.z_vel_max_up, &v);
		_params.vel_max_up = v;
		param_get(_params_handles.z_vel_max_down, &v);
		_params.vel_max_down = v;
		param_get(_params_handles.xy_vel_max, &v);
		_params.vel_max_xy = v;
		param_get(_params_handles.xy_vel_cruise, &v);
		_params.vel_cruise_xy = v;
		param_get(_params_handles.hold_max_xy, &v);
		_params.hold_max_xy = math::max(0.f, v);
		param_get(_params_handles.hold_max_z, &v);
		_params.hold_max_z = math::max(0.f, v);
		param_get(_params_handles.rc_flt_smp_rate, &(_params.rc_flt_smp_rate));
		_params.rc_flt_smp_rate = math::max(1.0f, _params.rc_flt_smp_rate);
		/* since we use filter to detect manual direction change, take half the cutoff of the stick filtering */
		param_get(_params_handles.rc_flt_cutoff, &(_params.rc_flt_cutoff));
		/* make sure the filter is in its stable region -> fc < fs/2 */
		_params.rc_flt_cutoff = math::constrain(_params.rc_flt_cutoff, 0.1f, (_params.rc_flt_smp_rate / 2.0f) - 1.f);

		/* update filters */
		_filter_manual_pitch.set_cutoff_frequency(_params.rc_flt_smp_rate, _params.rc_flt_cutoff);
		_filter_manual_roll.set_cutoff_frequency(_params.rc_flt_smp_rate, _params.rc_flt_cutoff);

		/* make sure that vel_cruise_xy is always smaller than vel_max */
		_params.vel_cruise_xy = math::min(_params.vel_cruise_xy, _params.vel_max_xy);

		param_get(_params_handles.slow_land_alt2, &v);
		_params.slow_land_alt2 = v;
		param_get(_params_handles.slow_land_alt1, &v);
		_params.slow_land_alt1 = math::max(v, _params.slow_land_alt2);

		param_get(_params_handles.alt_mode, &v_i);
		_params.alt_mode = v_i;

		/* mc attitude control parameters*/
		/* manual control scale */
		param_get(_params_handles.man_tilt_max, &_params.man_tilt_max);
		param_get(_params_handles.man_yaw_max, &_params.man_yaw_max);
		param_get(_params_handles.global_yaw_max, &_params.global_yaw_max);
		_params.man_tilt_max = math::radians(_params.man_tilt_max);
		_params.man_yaw_max = math::radians(_params.man_yaw_max);
		_params.global_yaw_max = math::radians(_params.global_yaw_max);

		param_get(_params_handles.mc_att_yaw_p, &v);
		_params.mc_att_yaw_p = v;

		/* takeoff and land velocities should not exceed maximum */
		_params.tko_speed = fminf(_params.tko_speed, _params.vel_max_up);
		_params.land_speed = fminf(_params.land_speed, _params.vel_max_down);

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

		if (_params_handles.acc_max_flow_xy != PARAM_INVALID) {
			param_get(handle, &_params.acc_max_flow_xy);
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

				parameters_update(true);

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

void
MulticopterPositionControl::task_main_trampoline(int argc, char *argv[])
{
	pos_control::g_control->task_main();
}

void
MulticopterPositionControl::update_ref()
{
	// The reference point is only allowed to change when the vehicle is in standby state which is the
	// normal state when the estimator origin is set. Changing reference point in flight causes large controller
	// setpoint changes. Changing reference point in other arming states is untested and shoud not be performed.
	if ((_local_pos.ref_timestamp != _ref_timestamp)
	    && ((_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY)
		|| (!_ref_alt_is_global && _local_pos.z_global))) {
		double lat_sp, lon_sp;
		float alt_sp = 0.0f;

		if (_ref_timestamp != 0) {
			// calculate current position setpoint in global frame
			map_projection_reproject(&_ref_pos, _pos_sp(0), _pos_sp(1), &lat_sp, &lon_sp);

			// the altitude setpoint is the reference altitude (Z up) plus the (Z down)
			// NED setpoint, multiplied out to minus
			alt_sp = _ref_alt - _pos_sp(2);
		}

		// update local projection reference including altitude
		map_projection_init(&_ref_pos, _local_pos.ref_lat, _local_pos.ref_lon);
		_ref_alt = _local_pos.ref_alt;

		if (_local_pos.z_global) {
			_ref_alt_is_global = true;
		}

		if (_ref_timestamp != 0) {
			// reproject position setpoint to new reference
			// this effectively adjusts the position setpoint to keep the vehicle
			// in its current local position. It would only change its
			// global position on the next setpoint update.
			map_projection_project(&_ref_pos, lat_sp, lon_sp, &_pos_sp(0), &_pos_sp(1));
			_pos_sp(2) = -(alt_sp - _ref_alt);
		}

		_ref_timestamp = _local_pos.ref_timestamp;

	}
}

void
MulticopterPositionControl::update_velocity_derivative()
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

		if (_params.alt_mode == 1 && _local_pos.dist_bottom_valid) {
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

		if (_params.alt_mode == 1 && _local_pos.dist_bottom_valid) {
			_vel(2) = -_local_pos.dist_bottom_rate;

		} else {
			_vel(2) = _local_pos.vz;
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

//		/* set default max velocity in xy to vel_max
//		 * Apply estimator limits if applicable */
//		if (_local_pos.vxy_max > 0.001f) {
//			// use the minimum of the estimator and user specified limit
//			_vel_max_xy = fminf(_params.vel_max_xy, _local_pos.vxy_max);
//			// Allow for a minimum of 0.3 m/s for repositioning
//			_vel_max_xy = fmaxf(_vel_max_xy, 0.3f);
//
//		} else if (_vel_sp_significant) {
//			// raise the limit at a constant rate up to the user specified value
//			if (_vel_max_xy >= _params.vel_max_xy) {
//				_vel_max_xy = _params.vel_max_xy;
//
//			} else {
//				_vel_max_xy += dt * _params.acc_max_flow_xy;
//			}
//		}


		update_ref();

		update_velocity_derivative();

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

			// structure that replaces global constraints such as tilt,
			// maximum velocity in z-direction ...
			Controller::Constraints constraints;
			constraints.vel_max_z_up = NAN; // NAN for not used

			// Check for smooth takeoff
			if (_vehicle_land_detected.landed && !_in_smooth_takeoff && _control_mode.flag_armed) {
				// Vehicle is still landed and no takeoff was initiated yet.
				// Adjust for different takeoff cases.
				// The minimum takeoff altitude needs to be at least 20cm above current position
				if ((PX4_ISFINITE(setpoint.z) && setpoint.z  < _pos(2) - 0.2f) ||
				    (PX4_ISFINITE(setpoint.vz) && setpoint.vz < math::min(-_params.tko_speed, -0.6f))) {
					// There is a position setpoint above current position or velocity setpoint larger than
					// takeoff speed. Enable smooth takeoff.
					_in_smooth_takeoff = true;
					_takeoff_speed = -0.5f;

				} else {
					// Default
					_in_smooth_takeoff = false;
				}
			}


			// If in smooth takeoff, adjust setpoints based on what is valid:
			// 1. position setpoint is valid -> go with takeoffspeed to specific altitude
			// 2. position setpoint not valid but velocity setpoint valid: ramp up velocity
			if (_in_smooth_takeoff) {
				float desired_tko_speed = -setpoint.vz;

				// If there is a valid position setpoint, then set the desired speed to the takeoff speed.
				if (PX4_ISFINITE(setpoint.z)) {
					desired_tko_speed =  _params.tko_speed;
				}

				// Ramp up takeoff speed.
				_takeoff_speed += desired_tko_speed * _dt / _takeoff_ramp_time.get();
				_takeoff_speed = math::min(_takeoff_speed,  desired_tko_speed);
				// Limit the velocity setpoint from the position controller
				constraints.vel_max_z_up = _takeoff_speed;

			} else {
				_in_smooth_takeoff = false;
			}

			// We can only run the control if we're already in-air, have a takeoff setpoint, and are not
			// in pure manual. Otherwise just stay idle.
			if (_vehicle_land_detected.landed && !_in_smooth_takeoff && !PX4_ISFINITE(setpoint.thrust[2])) {
				// Keep throttle low
				setpoint.thrust[0] = setpoint.thrust[1] = setpoint.thrust[2] = 0.0f;
				setpoint.yawspeed = 0.0f;
				setpoint.yaw = _yaw;
			}

			// Update tilt constraints. For now it still requires to know about control mode
			// TODO: check if it makes sense to have a tilt constraint for landing.
			updateTiltConstraints(constraints);

			// Update states, setpoints and constraints.
			_control.updateConstraints(constraints);
			_control.updateState(_local_pos, matrix::Vector3f(&(_vel_err_d(0))));
			_control.updateSetpoint(setpoint);

			// Generate desired thrust and yaw.
			_control.generateThrustYawSetpoint(_dt);
			matrix::Vector3f thr_sp = _control.getThrustSetpoint();

			// Check if vehicle is still in smooth takeoff.
			if (_in_smooth_takeoff) {
				// Smooth takeoff is achieved once desired altitude/velocity setpoint is reached.
				if (PX4_ISFINITE(setpoint.z)) {
					_in_smooth_takeoff = _pos(2) + 0.2f > setpoint.z;

				} else  {
					_in_smooth_takeoff = _takeoff_speed < -setpoint.vz;
				}
			}

			// Adjust thrust setpoint based on landdetector only if the
			// vehicle is NOT in pure Manual mode.
			if (!_in_smooth_takeoff && !PX4_ISFINITE(setpoint.thrust[2])) {
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
					// will end up with wrong integral sums
					_control.resetIntegralXY();
					_control.resetIntegralZ();
				}
			}

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
			_att_sp.landing_gear = vehicle_attitude_setpoint_s::LANDING_GEAR_DOWN;

			// Publish local position setpoint (for logging only) and attitude setpoint (for attitude controller).
			publish_local_pos_sp();
			publish_attitude();

		}
	}

	mavlink_log_info(&_mavlink_log_pub, "[mpc] stopped");

	_control_task = -1;
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

void
MulticopterPositionControl::updateTiltConstraints(Controller::Constraints &constraints)
{
	/* _contstraints */
	constraints.tilt_max = NAN; // Default no maximum tilt

	/* Set maximum tilt  */
	if (!_control_mode.flag_control_manual_enabled
	    && _pos_sp_triplet.current.valid
	    && _pos_sp_triplet.current.type
	    == position_setpoint_s::SETPOINT_TYPE_LAND) {

		/* Auto landing tilt */
		constraints.tilt_max = _params.tilt_max_land;

	} else {
		/* Velocity/acceleration control tilt */
		constraints.tilt_max = _params.tilt_max_air;
	}
}

int
MulticopterPositionControl::start()
{
	ASSERT(_control_task == -1);

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

bool MulticopterPositionControl::cross_sphere_line(const matrix::Vector3f &sphere_c, const float sphere_r,
		const matrix::Vector3f &line_a, const matrix::Vector3f &line_b, matrix::Vector3f &res)
{
	// dummy class
	return true;
}
