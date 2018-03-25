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
#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>
#include <systemlib/mavlink_log.h>

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

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
	bool 		_reset_pos_sp = true;  				/**<true if position setpoint needs a reset */
	bool 		_reset_alt_sp = true; 				/**<true if altitude setpoint needs a reset */
	bool 		_do_reset_alt_pos_flag = true; 		/**< TODO: check if we need this */
	bool		_mode_auto = false ;  				/**<true if in auot mode */
	bool 		_pos_hold_engaged = false; 			/**<true if hold positon in xy desired */
	bool 		_alt_hold_engaged = false; 			/**<true if hold in z desired */
	bool 		_run_pos_control = true;  			/**< true if position controller should be used */
	bool 		_run_alt_control = true; 			/**<true if altitude controller should be used */
	bool 		_reset_int_z = true; 				/**<true if reset integral in z */
	bool 		_reset_int_xy = true; 				/**<true if reset integral in xy */
	bool		 _reset_yaw_sp = true; 				/**<true if reset yaw setpoint */
	bool 		_hold_offboard_xy = false; 			/**<TODO : check if we need this extra hold_offboard flag */
	bool 		_hold_offboard_z = false;
	bool 		_in_smooth_takeoff = false; 				/**<true if takeoff ramp is applied */
	bool 		_in_landing = false;				/**<true if landing descent (only used in auto) */
	bool 		_lnd_reached_ground = false; 		/**<true if controller assumes the vehicle has reached the ground after landing */
	bool 		_triplet_lat_lon_finite = true; 		/**<true if triplets current is non-finite */

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

	/**
	 * Update reference for local position projection
	 */
	void		update_ref();

	/**
	 * Reset position setpoint to current position.
	 *
	 * This reset will only occur if the _reset_pos_sp flag has been set.
	 * The general logic is to first "activate" the flag in the flight
	 * regime where a switch to a position control mode should hold the
	 * very last position. Once switching to a position control mode
	 * the last position is stored once.
	 */
	void		reset_pos_sp();

	/**
	 * Reset altitude setpoint to current altitude.
	 *
	 * This reset will only occur if the _reset_alt_sp flag has been set.
	 * The general logic follows the reset_pos_sp() architecture.
	 */
	void		reset_alt_sp();

	/**
	 * Set position setpoint using manual control
	 */
	void		control_manual();

	void		control_non_manual();

	/**
	 * Set position setpoint using offboard control
	 */
	void		control_offboard();

	/**
	 * Set position setpoint for AUTO
	 */
	void		control_auto();

	void control_position();
	void calculate_velocity_setpoint();
	void calculate_thrust_setpoint();

	void vel_sp_slewrate();

	void update_velocity_derivative();

	void do_control();

	void generate_attitude_setpoint();

	float get_cruising_speed_xy();

	bool in_auto_takeoff();

	float get_vel_close(const matrix::Vector2f &unit_prev_to_current, const matrix::Vector2f &unit_current_to_next);

	void set_manual_acceleration_xy(matrix::Vector2f &stick_input_xy_NED);

	void set_manual_acceleration_z(float &max_acc_z, const float stick_input_z_NED);

	/**
	 * limit altitude based on several conditions
	 */
	void limit_altitude();

	void warn_rate_limited(const char *str);

	bool manual_wants_takeoff();

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
	_manual_direction_change_hysteresis.set_time_from_false(DIRECTION_CHANGE_TRIGGER_TIME_US);

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
MulticopterPositionControl::reset_pos_sp()
{
	if (_reset_pos_sp) {
		_reset_pos_sp = false;

		// we have logic in the main function which chooses the velocity setpoint such that the attitude setpoint is
		// continuous when switching into velocity controlled mode, therefore, we don't need to bother about resetting
		// position in a special way. In position control mode the position will be reset anyway until the vehicle has reduced speed.
		_pos_sp(0) = _pos(0);
		_pos_sp(1) = _pos(1);
	}
}

void
MulticopterPositionControl::reset_alt_sp()
{
	if (_reset_alt_sp) {
		_reset_alt_sp = false;

		// we have logic in the main function which choosed the velocity setpoint such that the attitude setpoint is
		// continuous when switching into velocity controlled mode, therefore, we don't need to bother about resetting
		// altitude in a special way
		_pos_sp(2) = _pos(2);
	}
}

void
MulticopterPositionControl::limit_altitude()
{
	if (_vehicle_land_detected.alt_max < 0.0f) {
		// there is no altitude limitation present
		return;
	}

	float altitude_above_home = -(_pos(2) - _home_pos.z);

	if (_run_alt_control && (altitude_above_home > _vehicle_land_detected.alt_max)) {
		// we are above maximum altitude
		_pos_sp(2) = -_vehicle_land_detected.alt_max +  _home_pos.z;

	} else if (!_run_alt_control && _vel_sp(2) <= 0.0f) {
		// we want to fly upwards: check if vehicle does not exceed altitude

		// time to reach zero velocity
		float delta_t = -_vel(2) / _acceleration_z_max_down.get();

		// predict next position based on current position, velocity, max acceleration downwards and time to reach zero velocity
		float pos_z_next = _pos(2) + _vel(2) * delta_t + 0.5f * _acceleration_z_max_down.get() * delta_t *delta_t;

		if (-(pos_z_next - _home_pos.z) > _vehicle_land_detected.alt_max) {
			// prevent the vehicle from exceeding maximum altitude by switching back to altitude control with maximum altitude as setpoint
			_pos_sp(2) = -_vehicle_land_detected.alt_max + _home_pos.z;
			_run_alt_control = true;
		}
	}
}

bool
MulticopterPositionControl::in_auto_takeoff()
{
	/*
	 * in auto mode, check if we do a takeoff
	 */
	return (_pos_sp_triplet.current.valid &&
		_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF) ||
	       _control_mode.flag_control_offboard_enabled;
}

float
MulticopterPositionControl::get_vel_close(const matrix::Vector2f &unit_prev_to_current,
		const matrix::Vector2f &unit_current_to_next)
{

	/* minimum cruise speed when passing waypoint */
	float min_cruise_speed = 1.0f;

	/* make sure that cruise speed is larger than minimum*/
	if ((get_cruising_speed_xy() - min_cruise_speed) < SIGMA_NORM) {
		return get_cruising_speed_xy();
	}

	/* middle cruise speed is a number between maximum cruising speed and minimum cruising speed and corresponds to speed at angle of 90degrees
	 * it needs to be always larger than minimum cruise speed */
	float middle_cruise_speed = _cruise_speed_90.get();

	if ((middle_cruise_speed - min_cruise_speed) < SIGMA_NORM) {
		middle_cruise_speed = min_cruise_speed + SIGMA_NORM;
	}

	if ((get_cruising_speed_xy() - middle_cruise_speed) < SIGMA_NORM) {
		middle_cruise_speed = (get_cruising_speed_xy() + min_cruise_speed) * 0.5f;
	}

	/* if middle cruise speed is exactly in the middle, then compute
	 * vel_close linearly
	 */
	bool use_linear_approach = false;

	if (((get_cruising_speed_xy() + min_cruise_speed) * 0.5f) - middle_cruise_speed < SIGMA_NORM) {
		use_linear_approach = true;
	}

	/* angle = cos(x) + 1.0
	 * angle goes from 0 to 2 with 0 = large angle, 2 = small angle:   0 = PI ; 2 = PI*0 */
	float angle = 2.0f;

	if (unit_current_to_next.length() > SIGMA_NORM) {
		angle = unit_current_to_next * (unit_prev_to_current * -1.0f) + 1.0f;
	}

	/* compute velocity target close to waypoint */
	float vel_close;

	if (use_linear_approach) {

		/* velocity close to target adjusted to angle
		 * vel_close =  m*x+q
		 */
		float slope = -(get_cruising_speed_xy() - min_cruise_speed) / 2.0f;
		vel_close = slope * angle + get_cruising_speed_xy();

	} else {

		/* velocity close to target adjusted to angle
		 * vel_close = a *b ^x + c; where at angle = 0 -> vel_close = vel_cruise; angle = 1 -> vel_close = middle_cruise_speed (this means that at 90degrees
		 * the velocity at target is middle_cruise_speed);
		 * angle = 2 -> vel_close = min_cruising_speed */

		/* from maximum cruise speed, minimum cruise speed and middle cruise speed compute constants a, b and c */
		float a = -((middle_cruise_speed -  get_cruising_speed_xy()) * (middle_cruise_speed -  get_cruising_speed_xy())) /
			  (2.0f * middle_cruise_speed - get_cruising_speed_xy() - min_cruise_speed);
		float c =  get_cruising_speed_xy() - a;
		float b = (middle_cruise_speed - c) / a;
		vel_close = a * powf(b, angle) + c;
	}

	/* vel_close needs to be in between max and min */
	return math::constrain(vel_close, min_cruise_speed, get_cruising_speed_xy());

}

float
MulticopterPositionControl::get_cruising_speed_xy()
{
	/*
	 * in mission the user can choose cruising speed different to default
	 */
	return ((PX4_ISFINITE(_pos_sp_triplet.current.cruising_speed) && !(_pos_sp_triplet.current.cruising_speed < 0.0f)) ?
		_pos_sp_triplet.current.cruising_speed : _params.vel_cruise_xy);
}

void
MulticopterPositionControl::set_manual_acceleration_z(float &max_acceleration, const float stick_z)
{


	/* in manual altitude control apply acceleration limit based on stick input
	 * we consider two states
	 * 1.) brake
	 * 2.) accelerate */

	/* check if zero input stick */
	const bool is_current_zero = (fabsf(stick_z) <= FLT_EPSILON);

	/* default is acceleration */
	manual_stick_input intention = acceleration;

	/* check zero input stick */
	if (is_current_zero) {
		intention = brake;
	}

	/* get max and min acceleration where min acceleration is just 1/5 of max acceleration */
	max_acceleration = (stick_z <= 0.0f) ? _acceleration_z_max_up.get() : _acceleration_z_max_down.get();

	/*
	 * update user input
	 */
	if ((_user_intention_z != brake) && (intention  == brake)) {

		/* we start with lowest acceleration */
		_acceleration_state_dependent_z = _acceleration_z_max_down.get();

		/* reset slew rate */
		_vel_sp_prev(2) = _vel(2);
		_user_intention_z = brake;
	}

	_user_intention_z = intention;

	/*
	 * apply acceleration depending on state
	 */
	if (_user_intention_z == brake) {

		/* limit jerk when braking to zero */
		float jerk = (_acceleration_z_max_up.get() - _acceleration_state_dependent_z) / _dt;

		if (jerk > _manual_jerk_limit_z) {
			_acceleration_state_dependent_z = _manual_jerk_limit_z * _dt + _acceleration_state_dependent_z;

		} else {
			_acceleration_state_dependent_z = _acceleration_z_max_up.get();
		}
	}

	if (_user_intention_z == acceleration) {
		_acceleration_state_dependent_z = (max_acceleration - _acceleration_z_max_down.get()) * fabsf(
				stick_z) + _acceleration_z_max_down.get();
	}
}

void
MulticopterPositionControl::set_manual_acceleration_xy(matrix::Vector2f &stick_xy)
{

	/*
	 * In manual mode we consider four states with different acceleration handling:
	 * 1. user wants to stop
	 * 2. user wants to quickly change direction
	 * 3. user wants to accelerate
	 * 4. user wants to decelerate
	 */

	/* get normalized stick input vector */
	matrix::Vector2f stick_xy_norm = (stick_xy.length() > 0.0f) ? stick_xy.normalized() : stick_xy;
	matrix::Vector2f stick_xy_prev_norm = (_stick_input_xy_prev.length() > 0.0f) ? _stick_input_xy_prev.normalized() :
					      _stick_input_xy_prev;

	/* check if stick direction and current velocity are within 60angle */
	const bool is_aligned = (stick_xy_norm * stick_xy_prev_norm) > 0.5f;

	/* check if zero input stick */
	const bool is_prev_zero = (fabsf(_stick_input_xy_prev.length()) <= FLT_EPSILON);
	const bool is_current_zero = (fabsf(stick_xy.length()) <= FLT_EPSILON);

	/* check acceleration */
	const bool do_acceleration = is_prev_zero || (is_aligned &&
				     ((stick_xy.length() > _stick_input_xy_prev.length()) || (fabsf(stick_xy.length() - 1.0f) < FLT_EPSILON)));

	const bool do_deceleration = (is_aligned && (stick_xy.length() <= _stick_input_xy_prev.length()));

	const bool do_direction_change = !is_aligned;

	manual_stick_input intention;

	if (is_current_zero) {
		/* we want to stop */
		intention = brake;

	} else if (do_acceleration) {
		/* we do manual acceleration */
		intention = acceleration;

	} else if (do_deceleration) {
		/* we do manual deceleration */
		intention = deceleration;

	} else if (do_direction_change) {
		/* we have a direction change */
		intention = direction_change;

	} else {
		/* catchall: acceleration */
		intention = acceleration;
	}


	/*
	 * update user intention
	 */

	/* we always want to break starting with slow deceleration */
	if ((_user_intention_xy != brake) && (intention  == brake)) {

		if (_jerk_hor_max.get() > _jerk_hor_min.get()) {
			_manual_jerk_limit_xy = (_jerk_hor_max.get() - _jerk_hor_min.get()) / _velocity_hor_manual.get() *
						sqrtf(_vel(0) * _vel(0) + _vel(1) * _vel(1)) + _jerk_hor_min.get();

			/* we start braking with lowest accleration */
			_acceleration_state_dependent_xy = _deceleration_hor_slow.get();

		} else {

			/* set the jerk limit large since we don't know it better*/
			_manual_jerk_limit_xy = 1000000.f;

			/* at brake we use max acceleration */
			_acceleration_state_dependent_xy = _acceleration_hor_max.get();

		}

		/* reset slew rate */
		_vel_sp_prev(0) = _vel(0);
		_vel_sp_prev(1) = _vel(1);

	}

	switch (_user_intention_xy) {
	case brake: {
			if (intention != brake) {
				_user_intention_xy = acceleration;
				/* we initialize with lowest acceleration */
				_acceleration_state_dependent_xy = _deceleration_hor_slow.get();
			}

			break;
		}

	case direction_change: {
			/* only exit direction change if brake or aligned */
			matrix::Vector2f vel_xy(_vel(0), _vel(1));
			matrix::Vector2f vel_xy_norm = (vel_xy.length() > 0.0f) ? vel_xy.normalized() : vel_xy;
			bool stick_vel_aligned = (vel_xy_norm * stick_xy_norm > 0.0f);

			/* update manual direction change hysteresis */
			_manual_direction_change_hysteresis.update(!stick_vel_aligned);

			/* exit direction change if one of the condition is met */
			if (intention == brake) {
				_user_intention_xy = intention;

			} else if (stick_vel_aligned) {
				_user_intention_xy = acceleration;

			} else if (_manual_direction_change_hysteresis.get_state()) {

				/* TODO: find conditions which are always continuous
				 * only if stick input is large*/
				if (stick_xy.length() > 0.6f) {
					_acceleration_state_dependent_xy = _acceleration_hor_max.get();
				}
			}

			break;
		}

	case acceleration: {
			_user_intention_xy = intention;

			if (_user_intention_xy == direction_change) {
				_vel_sp_prev(0) = _vel(0);
				_vel_sp_prev(1) = _vel(1);
			}

			break;
		}

	case deceleration: {
			_user_intention_xy = intention;

			if (_user_intention_xy == direction_change) {
				_vel_sp_prev(0) = _vel(0);
				_vel_sp_prev(1) = _vel(1);
			}

			break;
		}
	}

	/*
	 * apply acceleration based on state
	*/
	switch (_user_intention_xy) {
	case brake: {

			/* limit jerk when braking to zero */
			float jerk = (_acceleration_hor_max.get() - _acceleration_state_dependent_xy) / _dt;

			if (jerk > _manual_jerk_limit_xy) {
				_acceleration_state_dependent_xy = _manual_jerk_limit_xy * _dt + _acceleration_state_dependent_xy;

			} else {
				_acceleration_state_dependent_xy = _acceleration_hor_max.get();
			}

			break;
		}

	case direction_change: {

			/* limit acceleration linearly on stick input*/
			_acceleration_state_dependent_xy = (_acceleration_hor.get() - _deceleration_hor_slow.get()) * stick_xy.length() +
							   _deceleration_hor_slow.get();
			break;
		}

	case acceleration: {
			/* limit acceleration linearly on stick input*/
			float acc_limit  = (_acceleration_hor.get() - _deceleration_hor_slow.get()) * stick_xy.length()
					   + _deceleration_hor_slow.get();

			if (_acceleration_state_dependent_xy > acc_limit) {
				acc_limit = _acceleration_state_dependent_xy;
			}

			_acceleration_state_dependent_xy = acc_limit;
			break;
		}

	case deceleration: {
			_acceleration_state_dependent_xy = _deceleration_hor_slow.get();
			break;
		}

	default :
		warn_rate_limited("User intention not recognized");
		_acceleration_state_dependent_xy = _acceleration_hor_max.get();

	}

	/* update previous stick input */
	_stick_input_xy_prev = matrix::Vector2f(_filter_manual_pitch.apply(stick_xy(0)),
						_filter_manual_roll.apply(stick_xy(1)));


	if (_stick_input_xy_prev.length() > 1.0f) {
		_stick_input_xy_prev = _stick_input_xy_prev.normalized();
	}
}

void
MulticopterPositionControl::control_manual()
{
	/* Entering manual control from non-manual control mode, reset alt/pos setpoints */
	if (_mode_auto) {
		_mode_auto = false;

		/* Reset alt pos flags if resetting is enabled */
		if (_do_reset_alt_pos_flag) {
			_reset_pos_sp = true;
			_reset_alt_sp = true;
		}
	}

	/*
	 * Map from stick input to velocity setpoint
	 */

	/* velocity setpoint commanded by user stick input */
	matrix::Vector3f man_vel_sp;

	if (_control_mode.flag_control_altitude_enabled) {
		/* set vertical velocity setpoint with throttle stick, remapping of manual.z [0,1] to up and down command [-1,1] */
		man_vel_sp(2) = -math::expo_deadzone((_manual.z - 0.5f) * 2.f, _z_vel_man_expo.get(), _hold_dz.get());

		/* reset alt setpoint to current altitude if needed */
		reset_alt_sp();
	}

	if (_control_mode.flag_control_position_enabled) {
		/* set horizontal velocity setpoint with roll/pitch stick */
		man_vel_sp(0) = math::expo_deadzone(_manual.x, _xy_vel_man_expo.get(), _hold_dz.get());
		man_vel_sp(1) = math::expo_deadzone(_manual.y, _xy_vel_man_expo.get(), _hold_dz.get());

		const float man_vel_hor_length = ((matrix::Vector2f)man_vel_sp.slice<2, 1>(0, 0)).length();

		/* saturate such that magnitude is never larger than 1 */
		if (man_vel_hor_length > 1.0f) {
			man_vel_sp(0) /= man_vel_hor_length;
			man_vel_sp(1) /= man_vel_hor_length;
		}

		/* reset position setpoint to current position if needed */
		reset_pos_sp();

	}

	/* prepare yaw to rotate into NED frame */
	float yaw_input_frame = _control_mode.flag_control_fixed_hdg_enabled ? _yaw_takeoff : _att_sp.yaw_body;

	/* setpoint in NED frame */
	man_vel_sp = matrix::Dcmf(matrix::Eulerf(0.0f, 0.0f, yaw_input_frame)) * man_vel_sp;

	/* adjust acceleration based on stick input */
	matrix::Vector2f stick_xy(man_vel_sp(0), man_vel_sp(1));
	set_manual_acceleration_xy(stick_xy);
	float stick_z = man_vel_sp(2);
	float max_acc_z;
	set_manual_acceleration_z(max_acc_z, stick_z);

	/* prepare cruise speed (m/s) vector to scale the velocity setpoint */
	float vel_mag = (_velocity_hor_manual.get() < _vel_max_xy) ? _velocity_hor_manual.get() : _vel_max_xy;
	matrix::Vector3f vel_cruise_scale(vel_mag, vel_mag, (man_vel_sp(2) > 0.0f) ? _params.vel_max_down : _params.vel_max_up);
	/* Setpoint scaled to cruise speed */
	man_vel_sp = man_vel_sp.emult(vel_cruise_scale);

	/*
	 * assisted velocity mode: user controls velocity, but if velocity is small enough, position
	 * hold is activated for the corresponding axis
	 */

	/* want to get/stay in altitude hold if user has z stick in the middle (accounted for deadzone already) */
	const bool alt_hold_desired = _control_mode.flag_control_altitude_enabled && (_user_intention_z == brake);

	/* want to get/stay in position hold if user has xy stick in the middle (accounted for deadzone already) */
	const bool pos_hold_desired = _control_mode.flag_control_position_enabled && (_user_intention_xy ==  brake);

	/* check vertical hold engaged flag */
	if (_alt_hold_engaged) {
		_alt_hold_engaged = alt_hold_desired;

	} else {

		/* check if we switch to alt_hold_engaged */
		bool smooth_alt_transition = alt_hold_desired && ((max_acc_z - _acceleration_state_dependent_z) < FLT_EPSILON) &&
					     (_params.hold_max_z < FLT_EPSILON || fabsf(_vel(2)) < _params.hold_max_z);

		/* during transition predict setpoint forward */
		if (smooth_alt_transition) {

			/* time to travel from current velocity to zero velocity */
			float delta_t = fabsf(_vel(2) / max_acc_z);

			/* set desired position setpoint assuming max acceleration */
			_pos_sp(2) = _pos(2) + _vel(2) * delta_t + 0.5f * max_acc_z * delta_t *delta_t;

			_alt_hold_engaged = true;
		}
	}

	/* check horizontal hold engaged flag */
	if (_pos_hold_engaged) {

		/* check if contition still true */
		_pos_hold_engaged = pos_hold_desired;

		/* use max acceleration */
		if (_pos_hold_engaged) {
			_acceleration_state_dependent_xy = _acceleration_hor_max.get();
		}

	} else {

		/* check if we switch to pos_hold_engaged */
		float vel_xy_mag = sqrtf(_vel(0) * _vel(0) + _vel(1) * _vel(1));
		bool smooth_pos_transition = pos_hold_desired
					     && (fabsf(_acceleration_hor_max.get() - _acceleration_state_dependent_xy) < FLT_EPSILON) &&
					     (_params.hold_max_xy < FLT_EPSILON || vel_xy_mag < _params.hold_max_xy);

		/* during transition predict setpoint forward */
		if (smooth_pos_transition) {

			/* time to travel from current velocity to zero velocity */
			float delta_t = sqrtf(_vel(0) * _vel(0) + _vel(1) * _vel(1)) / _acceleration_hor_max.get();

			/* p pos_sp in xy from max acceleration and current velocity */
			matrix::Vector2f pos(_pos(0), _pos(1));
			matrix::Vector2f vel(_vel(0), _vel(1));
			matrix::Vector2f pos_sp = pos + vel * delta_t - vel.normalized() * 0.5f * _acceleration_hor_max.get() * delta_t
						  * delta_t;
			_pos_sp(0) = pos_sp(0);
			_pos_sp(1) = pos_sp(1);

			_pos_hold_engaged = true;
		}
	}

	/* set requested velocity setpoints */
	if (!_alt_hold_engaged) {
		_pos_sp(2) = _pos(2);
		_run_alt_control = false; /* request velocity setpoint to be used, instead of altitude setpoint */
		_vel_sp(2) = man_vel_sp(2);
	}

	if (!_pos_hold_engaged) {
		_pos_sp(0) = _pos(0);
		_pos_sp(1) = _pos(1);
		_run_pos_control = false; /* request velocity setpoint to be used, instead of position setpoint */
		_vel_sp(0) = man_vel_sp(0);
		_vel_sp(1) = man_vel_sp(1);
	}

	control_position();
}

void
MulticopterPositionControl::control_non_manual()
{
	/* select control source */
	if (_control_mode.flag_control_offboard_enabled) {
		/* offboard control */
		control_offboard();
		_mode_auto = false;

	} else {
		_hold_offboard_xy = false;
		_hold_offboard_z = false;

		/* AUTO */
		control_auto();
	}

	// guard against any bad velocity values
	bool velocity_valid = PX4_ISFINITE(_pos_sp_triplet.current.vx) &&
			      PX4_ISFINITE(_pos_sp_triplet.current.vy) &&
			      _pos_sp_triplet.current.velocity_valid;

	// do not go slower than the follow target velocity when position tracking is active (set to valid)
	if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET &&
	    velocity_valid &&
	    _pos_sp_triplet.current.position_valid) {

		matrix::Vector3f ft_vel(_pos_sp_triplet.current.vx, _pos_sp_triplet.current.vy, 0.0f);

		float cos_ratio = (ft_vel * _vel_sp) / (ft_vel.length() * _vel_sp.length());

		// only override velocity set points when uav is traveling in same direction as target and vector component
		// is greater than calculated position set point velocity component

		if (cos_ratio > 0) {
			ft_vel *= (cos_ratio);
			// min speed a little faster than target vel
			ft_vel += ft_vel.normalized() * 1.5f;

		} else {
			ft_vel.zero();
		}

		_vel_sp(0) = fabsf(ft_vel(0)) > fabsf(_vel_sp(0)) ? ft_vel(0) : _vel_sp(0);
		_vel_sp(1) = fabsf(ft_vel(1)) > fabsf(_vel_sp(1)) ? ft_vel(1) : _vel_sp(1);

		// track target using velocity only

	} else if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET &&
		   velocity_valid) {

		_vel_sp(0) = _pos_sp_triplet.current.vx;
		_vel_sp(1) = _pos_sp_triplet.current.vy;
	}

	/* use constant descend rate when landing, ignore altitude setpoint */
	if (_pos_sp_triplet.current.valid
	    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
		_vel_sp(2) = _params.land_speed;
		_run_alt_control = false;
	}

	if (_pos_sp_triplet.current.valid
	    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
		/* idle state, don't run controller and set zero thrust */
		_R_setpoint.identity();

		matrix::Quatf qd = _R_setpoint;
		qd.copyTo(_att_sp.q_d);
		_att_sp.q_d_valid = true;

		_att_sp.roll_body = 0.0f;
		_att_sp.pitch_body = 0.0f;
		_att_sp.yaw_body = _yaw;
		_att_sp.thrust = 0.0f;

		_att_sp.timestamp = hrt_absolute_time();

	} else {
		control_position();
	}
}

void
MulticopterPositionControl::control_offboard()
{
	if (_pos_sp_triplet.current.valid) {

		if (_control_mode.flag_control_position_enabled && _pos_sp_triplet.current.position_valid) {
			/* control position */
			_pos_sp(0) = _pos_sp_triplet.current.x;
			_pos_sp(1) = _pos_sp_triplet.current.y;
			_run_pos_control = true;

			_hold_offboard_xy = false;

		} else if (_control_mode.flag_control_velocity_enabled && _pos_sp_triplet.current.velocity_valid) {
			/* control velocity */

			/* reset position setpoint to current position if needed */
			reset_pos_sp();

			if (fabsf(_pos_sp_triplet.current.vx) <= FLT_EPSILON &&
			    fabsf(_pos_sp_triplet.current.vy) <= FLT_EPSILON &&
			    _local_pos.xy_valid) {

				if (!_hold_offboard_xy) {
					_pos_sp(0) = _pos(0);
					_pos_sp(1) = _pos(1);
					_hold_offboard_xy = true;
				}

				_run_pos_control = true;

			} else {

				if (_pos_sp_triplet.current.velocity_frame == position_setpoint_s::VELOCITY_FRAME_LOCAL_NED) {
					/* set position setpoint move rate */
					_vel_sp(0) = _pos_sp_triplet.current.vx;
					_vel_sp(1) = _pos_sp_triplet.current.vy;

				} else if (_pos_sp_triplet.current.velocity_frame == position_setpoint_s::VELOCITY_FRAME_BODY_NED) {
					// Transform velocity command from body frame to NED frame
					_vel_sp(0) = cosf(_yaw) * _pos_sp_triplet.current.vx - sinf(_yaw) * _pos_sp_triplet.current.vy;
					_vel_sp(1) = sinf(_yaw) * _pos_sp_triplet.current.vx + cosf(_yaw) * _pos_sp_triplet.current.vy;

				} else {
					warn_rate_limited("Unknown velocity offboard coordinate frame");
				}

				_run_pos_control = false;

				_hold_offboard_xy = false;
			}
		}

		if (_control_mode.flag_control_altitude_enabled && _pos_sp_triplet.current.alt_valid) {
			/* control altitude as it is enabled */
			_pos_sp(2) = _pos_sp_triplet.current.z;
			_run_alt_control = true;

			_hold_offboard_z = false;

		} else if (_control_mode.flag_control_climb_rate_enabled && _pos_sp_triplet.current.velocity_valid) {

			/* reset alt setpoint to current altitude if needed */
			reset_alt_sp();

			if (fabsf(_pos_sp_triplet.current.vz) <= FLT_EPSILON &&
			    _local_pos.z_valid) {

				if (!_hold_offboard_z) {
					_pos_sp(2) = _pos(2);
					_hold_offboard_z = true;
				}

				_run_alt_control = true;

			} else {
				/* set position setpoint move rate */
				_vel_sp(2) = _pos_sp_triplet.current.vz;
				_run_alt_control = false;

				_hold_offboard_z = false;
			}
		}

		if (_pos_sp_triplet.current.yaw_valid) {
			_att_sp.yaw_body = _pos_sp_triplet.current.yaw;

		} else if (_pos_sp_triplet.current.yawspeed_valid) {
			float yaw_target = _wrap_pi(_att_sp.yaw_body + _pos_sp_triplet.current.yawspeed * _dt);
			float yaw_offs = _wrap_pi(yaw_target - _yaw);
			const float yaw_rate_max = (_params.man_yaw_max < _params.global_yaw_max) ? _params.man_yaw_max :
						   _params.global_yaw_max;
			const float yaw_offset_max = yaw_rate_max / _params.mc_att_yaw_p;

			// If the yaw offset became too big for the system to track stop
			// shifting it, only allow if it would make the offset smaller again.
			if (fabsf(yaw_offs) < yaw_offset_max ||
			    (_pos_sp_triplet.current.yawspeed > 0 && yaw_offs < 0) ||
			    (_pos_sp_triplet.current.yawspeed < 0 && yaw_offs > 0)) {
				_att_sp.yaw_body = yaw_target;
			}
		}

	} else {
		_hold_offboard_xy = false;
		_hold_offboard_z = false;
		reset_pos_sp();
		reset_alt_sp();
	}
}

void
MulticopterPositionControl::vel_sp_slewrate()
{
	matrix::Vector2f vel_sp_xy(_vel_sp(0), _vel_sp(1));
	matrix::Vector2f vel_sp_prev_xy(_vel_sp_prev(0), _vel_sp_prev(1));
	matrix::Vector2f acc_xy = (vel_sp_xy - vel_sp_prev_xy) / _dt;

	/* limit total horizontal acceleration */
	if (acc_xy.length() > _acceleration_state_dependent_xy) {
		vel_sp_xy = _acceleration_state_dependent_xy * acc_xy.normalized() * _dt + vel_sp_prev_xy;
		_vel_sp(0) = vel_sp_xy(0);
		_vel_sp(1) = vel_sp_xy(1);
	}

	/* limit vertical acceleration */
	float acc_z = (_vel_sp(2) - _vel_sp_prev(2)) / _dt;
	float max_acc_z;

	if (_control_mode.flag_control_manual_enabled) {
		max_acc_z = (acc_z < 0.0f) ? -_acceleration_state_dependent_z : _acceleration_state_dependent_z;

	} else {
		max_acc_z = (acc_z < 0.0f) ? -_acceleration_z_max_up.get() : _acceleration_z_max_down.get();
	}

	if (fabsf(acc_z) > fabsf(max_acc_z)) {
		_vel_sp(2) = max_acc_z * _dt + _vel_sp_prev(2);
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

void MulticopterPositionControl::control_auto()
{
	/* reset position setpoint on AUTO mode activation or if we are not in MC mode */
	if (!_mode_auto || !_vehicle_status.is_rotary_wing) {
		if (!_mode_auto) {
			_mode_auto = true;
			//set _triplet_lat_lon_finite true once switch to AUTO(e.g. LAND)
			_triplet_lat_lon_finite = true;
		}

		_reset_pos_sp = true;
		_reset_alt_sp = true;
	}

	// Always check reset state of altitude and position control flags in auto
	reset_pos_sp();
	reset_alt_sp();

	bool current_setpoint_valid = false;
	bool previous_setpoint_valid = false;
	bool next_setpoint_valid = false;
	bool triplet_updated = false;

	matrix::Vector3f prev_sp;
	matrix::Vector3f next_sp;

	if (_pos_sp_triplet.current.valid) {

		matrix::Vector3f curr_pos_sp = _curr_pos_sp;

		//only project setpoints if they are finite, else use current position
		if (PX4_ISFINITE(_pos_sp_triplet.current.lat) &&
		    PX4_ISFINITE(_pos_sp_triplet.current.lon)) {
			/* project setpoint to local frame */
			map_projection_project(&_ref_pos,
					       _pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon,
					       &curr_pos_sp(0), &curr_pos_sp(1));

			_triplet_lat_lon_finite = true;

		} else { // use current position if NAN -> e.g. land
			if (_triplet_lat_lon_finite) {
				curr_pos_sp(0) = _pos(0);
				curr_pos_sp(1) = _pos(1);
				_triplet_lat_lon_finite = false;
			}
		}

		// only project setpoints if they are finite, else use current position
		if (PX4_ISFINITE(_pos_sp_triplet.current.alt)) {
			curr_pos_sp(2) = -(_pos_sp_triplet.current.alt - _ref_alt);

		}


		/* sanity check */
		if (PX4_ISFINITE(_curr_pos_sp(0)) &&
		    PX4_ISFINITE(_curr_pos_sp(1)) &&
		    PX4_ISFINITE(_curr_pos_sp(2))) {
			current_setpoint_valid = true;
		}

		/* check if triplets have been updated
		 * note: we only can look at xy since navigator applies slewrate to z */
		float  diff;

		if (_triplet_lat_lon_finite) {
			diff = matrix::Vector2f((_curr_pos_sp(0) - curr_pos_sp(0)), (_curr_pos_sp(1) - curr_pos_sp(1))).length();

		} else {
			diff = fabsf(_curr_pos_sp(2) - curr_pos_sp(2));
		}

		if (diff > FLT_EPSILON || !PX4_ISFINITE(diff)) {
			triplet_updated = true;
		}

		/* we need to update _curr_pos_sp always since navigator applies slew rate on z */
		_curr_pos_sp = curr_pos_sp;
	}

	if (_pos_sp_triplet.previous.valid) {
		map_projection_project(&_ref_pos,
				       _pos_sp_triplet.previous.lat, _pos_sp_triplet.previous.lon,
				       &prev_sp(0), &prev_sp(1));
		prev_sp(2) = -(_pos_sp_triplet.previous.alt - _ref_alt);

		if (PX4_ISFINITE(prev_sp(0)) &&
		    PX4_ISFINITE(prev_sp(1)) &&
		    PX4_ISFINITE(prev_sp(2))) {
			_prev_pos_sp = prev_sp;
			previous_setpoint_valid = true;
		}
	}

	/* set previous setpoint to current position if no previous setpoint available */
	if (!previous_setpoint_valid && triplet_updated) {
		_prev_pos_sp = _pos;
		previous_setpoint_valid = true; /* currrently not necessary to set to true since not used*/
	}

	if (_pos_sp_triplet.next.valid) {
		map_projection_project(&_ref_pos,
				       _pos_sp_triplet.next.lat, _pos_sp_triplet.next.lon,
				       &next_sp(0), &next_sp(1));

		next_sp(2) = -(_pos_sp_triplet.next.alt - _ref_alt);

		if (PX4_ISFINITE(next_sp(0)) &&
		    PX4_ISFINITE(next_sp(1)) &&
		    PX4_ISFINITE(next_sp(2))) {
			next_setpoint_valid = true;
		}
	}

	/* Auto logic:
	 * The vehicle should follow the line previous-current.
	 * - if there is no next setpoint or the current is a loiter point, then slowly approach the current along the line
	 * - if there is a next setpoint, then the velocity is adjusted depending on the angle of the corner prev-current-next.
	 * When following the line, the pos_sp is computed from the orthogonal distance to the closest point on line and the desired cruise speed along the track.
	 */

	/* create new _pos_sp from triplets */
	if (current_setpoint_valid &&
	    (_pos_sp_triplet.current.type != position_setpoint_s::SETPOINT_TYPE_IDLE)) {

		/* update yaw setpoint if needed */
		if (_pos_sp_triplet.current.yawspeed_valid
		    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET) {
			_att_sp.yaw_body = _att_sp.yaw_body + _pos_sp_triplet.current.yawspeed * _dt;

		} else if (PX4_ISFINITE(_pos_sp_triplet.current.yaw)) {
			_att_sp.yaw_body = _pos_sp_triplet.current.yaw;
		}

		float yaw_diff = _wrap_pi(_att_sp.yaw_body - _yaw);

		/* only follow previous-current-line for specific triplet type */
		if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_POSITION  ||
		    _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER ||
		    _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET) {

			/* by default use current setpoint as is */
			matrix::Vector3f pos_sp = _curr_pos_sp;

			/*
			 * Z-DIRECTION
			 */

			/* get various distances */
			float total_dist_z = fabsf(_curr_pos_sp(2) - _prev_pos_sp(2));
			float dist_to_prev_z = fabsf(_pos(2) - _prev_pos_sp(2));
			float dist_to_current_z = fabsf(_curr_pos_sp(2) - _pos(2));

			/* if pos_sp has not reached target setpoint (=curr_pos_sp(2)),
			 * then compute setpoint depending on vel_max */
			if ((total_dist_z >  SIGMA_NORM) && (fabsf(_pos_sp(2) - _curr_pos_sp(2)) > SIGMA_NORM)) {

				/* check sign */
				bool flying_upward = _curr_pos_sp(2) < _pos(2);

				/* final_vel_z is the max velocity which depends on the distance of total_dist_z
				 * with default params.vel_max_up/down
				 */
				float final_vel_z = (flying_upward) ? _params.vel_max_up : _params.vel_max_down;

				/* target threshold defines the distance to _curr_pos_sp(2) at which
				 * the vehicle starts to slow down to approach the target smoothly
				 */
				float target_threshold_z = final_vel_z * 1.5f;

				/* if the total distance in z is NOT 2x distance of target_threshold, we
				 * will need to adjust the final_vel_z
				 */
				bool is_2_target_threshold_z = total_dist_z >= 2.0f * target_threshold_z;
				float slope = (final_vel_z) / (target_threshold_z); /* defines the the acceleration when slowing down */
				float min_vel_z = 0.2f; // minimum velocity: this is needed since estimation is not perfect

				if (!is_2_target_threshold_z) {
					/* adjust final_vel_z since we are already very close
					 * to current and therefore it is not necessary to accelerate
					 * up to full speed (=final_vel_z)
					 */
					target_threshold_z = total_dist_z * 0.5f;
					/* get the velocity at target_threshold_z */
					float final_vel_z_tmp = slope * (target_threshold_z) + min_vel_z;

					/* make sure that final_vel_z is never smaller than 0.5 of the default final_vel_z
					 * this is mainly done because the estimation in z is not perfect and therefore
					 * it is necessary to have a minimum speed
					 */
					final_vel_z = math::constrain(final_vel_z_tmp, final_vel_z * 0.5f, final_vel_z);
				}

				float vel_sp_z = final_vel_z;

				/* we want to slow down */
				if (dist_to_current_z < target_threshold_z) {

					vel_sp_z = slope * dist_to_current_z + min_vel_z;

				} else if (dist_to_prev_z < target_threshold_z) {
					/* we want to accelerate */

					float acc_z = (vel_sp_z - fabsf(_vel_sp(2))) / _dt;
					float acc_max = (flying_upward) ? (_acceleration_z_max_up.get() * 0.5f) : (_acceleration_z_max_down.get() * 0.5f);

					if (acc_z > acc_max) {
						vel_sp_z = _acceleration_z_max_up.get() * _dt + fabsf(_vel_sp(2));
					}

				}

				/* if we already close to current, then just take over the velocity that
				 * we would have computed if going directly to the current setpoint
				 */
				if (vel_sp_z >= (dist_to_current_z * _params.pos_p(2))) {
					vel_sp_z = dist_to_current_z * _params.pos_p(2);
				}

				/* make sure vel_sp_z is always positive */
				vel_sp_z = math::constrain(vel_sp_z, 0.0f, final_vel_z);
				/* get the sign of vel_sp_z */
				vel_sp_z = (flying_upward) ? -vel_sp_z : vel_sp_z;
				/* compute pos_sp(2) */
				pos_sp(2) = _pos(2) + vel_sp_z / _params.pos_p(2);
			}

			/*
			 * XY-DIRECTION
			 */

			/* line from previous to current and from pos to current */
			matrix::Vector2f vec_prev_to_current((_curr_pos_sp(0) - _prev_pos_sp(0)), (_curr_pos_sp(1) - _prev_pos_sp(1)));
			matrix::Vector2f vec_pos_to_current((_curr_pos_sp(0) - _pos(0)), (_curr_pos_sp(1) - _pos(1)));


			/* check if we just want to stay at current position */
			matrix::Vector2f pos_sp_diff((_curr_pos_sp(0) - _pos_sp(0)), (_curr_pos_sp(1) - _pos_sp(1)));
			bool stay_at_current_pos = (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER
						    || !next_setpoint_valid)
						   && ((pos_sp_diff.length()) < SIGMA_NORM);

			/* only follow line if previous to current has a minimum distance */
			if ((vec_prev_to_current.length()  > _nav_rad.get()) && !stay_at_current_pos) {

				/* normalize prev-current line (always > nav_rad) */
				matrix::Vector2f unit_prev_to_current = vec_prev_to_current.normalized();

				/* unit vector from current to next */
				matrix::Vector2f unit_current_to_next(0.0f, 0.0f);

				if (next_setpoint_valid) {
					unit_current_to_next = matrix::Vector2f((next_sp(0) - pos_sp(0)), (next_sp(1) - pos_sp(1)));
					unit_current_to_next = (unit_current_to_next.length() > SIGMA_NORM) ? unit_current_to_next.normalized() :
							       unit_current_to_next;
				}

				/* point on line closest to pos */
				matrix::Vector2f closest_point = matrix::Vector2f(_prev_pos_sp(0), _prev_pos_sp(1)) + unit_prev_to_current *
								 (matrix::Vector2f((_pos(0) - _prev_pos_sp(0)), (_pos(1) - _prev_pos_sp(1))) * unit_prev_to_current);

				matrix::Vector2f vec_closest_to_current((_curr_pos_sp(0) - closest_point(0)), (_curr_pos_sp(1) - closest_point(1)));

				/* compute vector from position-current and previous-position */
				matrix::Vector2f vec_prev_to_pos((_pos(0) - _prev_pos_sp(0)), (_pos(1) - _prev_pos_sp(1)));

				/* current velocity along track */
				float vel_sp_along_track_prev = matrix::Vector2f(_vel_sp(0), _vel_sp(1)) * unit_prev_to_current;

				/* distance to target when brake should occur */
				float target_threshold_xy = 1.5f * get_cruising_speed_xy();

				bool close_to_current = vec_pos_to_current.length() < target_threshold_xy;
				bool close_to_prev = (vec_prev_to_pos.length() < target_threshold_xy) &&
						     (vec_prev_to_pos.length() < vec_pos_to_current.length());

				/* indicates if we are at least half the distance from previous to current close to previous */
				bool is_2_target_threshold = vec_prev_to_current.length() >= 2.0f * target_threshold_xy;

				/* check if the current setpoint is behind */
				bool current_behind = ((vec_pos_to_current * -1.0f) * unit_prev_to_current) > 0.0f;

				/* check if the previous is in front */
				bool previous_in_front = (vec_prev_to_pos * unit_prev_to_current) < 0.0f;

				/* default velocity along line prev-current */
				float vel_sp_along_track = get_cruising_speed_xy();

				/*
				 * compute velocity setpoint along track
				 */

				/* only go directly to previous setpoint if more than 5m away and previous in front*/
				if (previous_in_front && (vec_prev_to_pos.length() > 5.0f)) {

					/* just use the default velocity along track */
					vel_sp_along_track = vec_prev_to_pos.length() * _params.pos_p(0);

					if (vel_sp_along_track > get_cruising_speed_xy()) {
						vel_sp_along_track = get_cruising_speed_xy();
					}

				} else if (current_behind) {
					/* go directly to current setpoint */
					vel_sp_along_track = vec_pos_to_current.length() * _params.pos_p(0);
					vel_sp_along_track = (vel_sp_along_track < get_cruising_speed_xy()) ? vel_sp_along_track : get_cruising_speed_xy();

				} else if (close_to_prev) {
					/* accelerate from previous setpoint towards current setpoint */

					/* we are close to previous and current setpoint
					 * we first compute the start velocity when close to current septoint and use
					 * this velocity as final velocity when transition occurs from acceleration to deceleration.
					 * This ensures smooth transition */
					float final_cruise_speed = get_cruising_speed_xy();

					if (!is_2_target_threshold) {

						/* set target threshold to half dist pre-current */
						float target_threshold_tmp = target_threshold_xy;
						target_threshold_xy = vec_prev_to_current.length() * 0.5f;

						if ((target_threshold_xy - _nav_rad.get()) < SIGMA_NORM) {
							target_threshold_xy = _nav_rad.get();
						}

						/* velocity close to current setpoint with default zero if no next setpoint is available */
						float vel_close = 0.0f;
						float acceptance_radius = 0.0f;

						/* we want to pass and need to compute the desired velocity close to current setpoint */
						if (next_setpoint_valid &&  !(_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER)) {
							/* get velocity close to current that depends on angle between prev-current and current-next line */
							vel_close = get_vel_close(unit_prev_to_current, unit_current_to_next);
							acceptance_radius = _nav_rad.get();
						}

						/* compute velocity at transition where vehicle switches from acceleration to deceleration */
						if ((target_threshold_tmp - acceptance_radius) < SIGMA_NORM) {
							final_cruise_speed = vel_close;

						} else {
							float slope = (get_cruising_speed_xy() - vel_close) / (target_threshold_tmp - acceptance_radius);
							final_cruise_speed = slope  * (target_threshold_xy - acceptance_radius) + vel_close;
							final_cruise_speed = (final_cruise_speed > vel_close) ? final_cruise_speed : vel_close;
						}
					}

					/* make sure final cruise speed is larger than 0*/
					final_cruise_speed = (final_cruise_speed > SIGMA_NORM) ? final_cruise_speed : SIGMA_NORM;
					vel_sp_along_track = final_cruise_speed;

					/* we want to accelerate not too fast
					* TODO: change the name acceleration_hor_man to something that can
					* be used by auto and manual */
					float acc_track = (final_cruise_speed - vel_sp_along_track_prev) / _dt;

					/* if yaw offset is large, only accelerate with 0.5m/s^2 */
					float acc = (fabsf(yaw_diff) >  math::radians(_mis_yaw_error.get())) ? 0.5f : _acceleration_hor.get();

					if (acc_track > acc) {
						vel_sp_along_track = acc * _dt + vel_sp_along_track_prev;
					}

					/* enforce minimum cruise speed */
					vel_sp_along_track  = math::constrain(vel_sp_along_track, SIGMA_NORM, final_cruise_speed);

				} else if (close_to_current) {
					/* slow down when close to current setpoint */

					/* check if altidue is within acceptance radius */
					bool reached_altitude = (dist_to_current_z < _nav_rad.get()) ? true : false;

					if (reached_altitude && next_setpoint_valid
					    && !(_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER)) {
						/* since we have a next setpoint use the angle prev-current-next to compute velocity setpoint limit */

						/* get velocity close to current that depends on angle between prev-current and current-next line */
						float vel_close = get_vel_close(unit_prev_to_current, unit_current_to_next);

						/* compute velocity along line which depends on distance to current setpoint */
						if (vec_closest_to_current.length() < _nav_rad.get()) {
							vel_sp_along_track = vel_close;

						} else {

							if (target_threshold_xy - _nav_rad.get() < SIGMA_NORM) {
								vel_sp_along_track = vel_close;

							} else {
								float slope = (get_cruising_speed_xy() - vel_close) / (target_threshold_xy - _nav_rad.get()) ;
								vel_sp_along_track = slope  * (vec_closest_to_current.length() - _nav_rad.get()) + vel_close;
							}
						}

						/* since we want to slow down take over previous velocity setpoint along track if it was lower */
						if ((vel_sp_along_track_prev < vel_sp_along_track) && (vel_sp_along_track * vel_sp_along_track_prev > 0.0f)) {
							vel_sp_along_track = vel_sp_along_track_prev;
						}

						/* if we are close to target and the previous velocity setpoints was smaller than
						 * vel_sp_along_track, then take over the previous one
						 * this ensures smoothness since we anyway want to slow down
						 */
						if ((vel_sp_along_track_prev < vel_sp_along_track) && (vel_sp_along_track * vel_sp_along_track_prev > 0.0f)
						    && (vel_sp_along_track_prev > vel_close)) {
							vel_sp_along_track = vel_sp_along_track_prev;
						}

						/* make sure that vel_sp_along track is at least min */
						vel_sp_along_track = (vel_sp_along_track < vel_close) ? vel_close : vel_sp_along_track;


					} else {

						/* we want to stop at current setpoint */
						float slope = (get_cruising_speed_xy())  / target_threshold_xy;
						vel_sp_along_track =  slope * (vec_closest_to_current.length());

						/* since we want to slow down take over previous velocity setpoint along track if it was lower but ensure its not zero */
						if ((vel_sp_along_track_prev < vel_sp_along_track) && (vel_sp_along_track * vel_sp_along_track_prev > 0.0f)
						    && (vel_sp_along_track_prev > 0.5f)) {
							vel_sp_along_track = vel_sp_along_track_prev;
						}
					}
				}

				/* compute velocity orthogonal to prev-current-line to position*/
				matrix::Vector2f vec_pos_to_closest = closest_point - matrix::Vector2f(_pos(0), _pos(1));
				float vel_sp_orthogonal = vec_pos_to_closest.length() * _params.pos_p(0);

				/* compute the cruise speed from velocity along line and orthogonal velocity setpoint */
				float cruise_sp_mag = sqrtf(vel_sp_orthogonal * vel_sp_orthogonal + vel_sp_along_track * vel_sp_along_track);

				/* sanity check */
				cruise_sp_mag = (PX4_ISFINITE(cruise_sp_mag)) ? cruise_sp_mag : vel_sp_orthogonal;

				/* orthogonal velocity setpoint is smaller than cruise speed */
				if (vel_sp_orthogonal < get_cruising_speed_xy() && !current_behind) {

					/* we need to limit vel_sp_along_track such that cruise speed  is never exceeded but still can keep velocity orthogonal to track */
					if (cruise_sp_mag > get_cruising_speed_xy()) {
						vel_sp_along_track = sqrtf(get_cruising_speed_xy() * get_cruising_speed_xy() - vel_sp_orthogonal * vel_sp_orthogonal);
					}

					pos_sp(0) = closest_point(0) + unit_prev_to_current(0) * vel_sp_along_track / _params.pos_p(0);
					pos_sp(1) = closest_point(1) + unit_prev_to_current(1) * vel_sp_along_track / _params.pos_p(1);

				} else if (current_behind) {
					/* current is behind */

					if (vec_pos_to_current.length()  > 0.01f) {
						pos_sp(0) = _pos(0) + vec_pos_to_current(0) / vec_pos_to_current.length() * vel_sp_along_track / _params.pos_p(0);
						pos_sp(1) = _pos(1) + vec_pos_to_current(1) / vec_pos_to_current.length() * vel_sp_along_track / _params.pos_p(1);

					} else {
						pos_sp(0) = _curr_pos_sp(0);
						pos_sp(1) = _curr_pos_sp(1);
					}

				} else {
					/* we are more than cruise_speed away from track */

					/* if previous is in front just go directly to previous point */
					if (previous_in_front) {
						vec_pos_to_closest(0) = _prev_pos_sp(0) - _pos(0);
						vec_pos_to_closest(1) = _prev_pos_sp(1) - _pos(1);
					}

					/* make sure that we never exceed maximum cruise speed */
					float cruise_sp = vec_pos_to_closest.length() * _params.pos_p(0);

					if (cruise_sp > get_cruising_speed_xy()) {
						cruise_sp = get_cruising_speed_xy();
					}

					/* sanity check: don't divide by zero */
					if (vec_pos_to_closest.length() > SIGMA_NORM) {
						pos_sp(0) = _pos(0) + vec_pos_to_closest(0) / vec_pos_to_closest.length() * cruise_sp / _params.pos_p(0);
						pos_sp(1) = _pos(1) + vec_pos_to_closest(1) / vec_pos_to_closest.length() * cruise_sp / _params.pos_p(1);

					} else {
						pos_sp(0) = closest_point(0);
						pos_sp(1) = closest_point(1);
					}
				}
			}

			_pos_sp = pos_sp;

		} else if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_VELOCITY) {

			float vel_xy_mag = sqrtf(_vel(0) * _vel(0) + _vel(1) * _vel(1));

			if (vel_xy_mag > SIGMA_NORM) {
				_vel_sp(0) = _vel(0) / vel_xy_mag * get_cruising_speed_xy();
				_vel_sp(1) = _vel(1) / vel_xy_mag * get_cruising_speed_xy();

			} else {
				/* TODO: we should go in the direction we are heading
				 * if current velocity is zero
				 */
				_vel_sp(0) = 0.0f;
				_vel_sp(1) = 0.0f;
			}

			_run_pos_control = false;

		} else {
			/* just go to the target point */;
			_pos_sp = _curr_pos_sp;

			/* set max velocity to cruise */
			_vel_max_xy = get_cruising_speed_xy();
		}

		/* sanity check */
		if (!(PX4_ISFINITE(_pos_sp(0)) && PX4_ISFINITE(_pos_sp(1)) &&
		      PX4_ISFINITE(_pos_sp(2)))) {

			warn_rate_limited("Auto: Position setpoint not finite");
			_pos_sp = _curr_pos_sp;
		}


		/*
		 * if we're already near the current takeoff setpoint don't reset in case we switch back to posctl.
		 * this makes the takeoff finish smoothly.
		 */
		if ((_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF
		     || _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER)
		    && _pos_sp_triplet.current.acceptance_radius > 0.0f
		    /* need to detect we're close a bit before the navigator switches from takeoff to next waypoint */
		    && (_pos - _pos_sp).length() < _pos_sp_triplet.current.acceptance_radius * 1.2f) {
			_do_reset_alt_pos_flag = false;

		} else {
			/* otherwise: in case of interrupted mission don't go to waypoint but stay at current position */
			_do_reset_alt_pos_flag = true;
		}

		// Handle the landing gear based on the manual landing alt
		const bool high_enough_for_landing_gear = (-_pos(2) + _home_pos.z > 2.0f);

		// During a mission or in loiter it's safe to retract the landing gear.
		if ((_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_POSITION ||
		     _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER) &&
		    !_vehicle_land_detected.landed &&
		    high_enough_for_landing_gear) {

			_att_sp.landing_gear = vehicle_attitude_setpoint_s::LANDING_GEAR_UP;

		} else if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF ||
			   _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND ||
			   !high_enough_for_landing_gear) {

			// During takeoff and landing, we better put it down again.
			_att_sp.landing_gear = vehicle_attitude_setpoint_s::LANDING_GEAR_DOWN;

			// For the rest of the setpoint types, just leave it as is.
		}

	} else {
		/* idle or triplet not valid, set velocity setpoint to zero */
		_vel_sp.zero();
		_run_pos_control = false;
		_run_alt_control = false;
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

		if (!_run_alt_control) {
			/* set velocity to the derivative of position
			 * because it has less bias but blend it in across the landing speed range*/
			float weighting = fminf(fabsf(_vel_sp(2)) / _params.land_speed, 1.0f);
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
MulticopterPositionControl::do_control()
{
	/* by default, run position/altitude controller. the control_* functions
	 * can disable this and run velocity controllers directly in this cycle */
	_run_pos_control = true;
	_run_alt_control = true;

	if (_control_mode.flag_control_manual_enabled) {
		/* manual control */
		control_manual();
		_mode_auto = false;

		/* we set triplets to false
		 * this ensures that when switching to auto, the position
		 * controller will not use the old triplets but waits until triplets
		 * have been updated */
		_pos_sp_triplet.current.valid = false;
		_pos_sp_triplet.previous.valid = false;
		_curr_pos_sp = matrix::Vector3f(NAN, NAN, NAN);

		_hold_offboard_xy = false;
		_hold_offboard_z = false;

	} else {
		/* reset acceleration to default */
		_acceleration_state_dependent_xy = _acceleration_hor_max.get();
		_acceleration_state_dependent_z = _acceleration_z_max_up.get();
		control_non_manual();
	}
}

void
MulticopterPositionControl::control_position()
{
	calculate_velocity_setpoint();

	if (_control_mode.flag_control_climb_rate_enabled || _control_mode.flag_control_velocity_enabled ||
	    _control_mode.flag_control_acceleration_enabled) {
		calculate_thrust_setpoint();

	} else {
		_reset_int_z = true;
	}
}

void
MulticopterPositionControl::calculate_velocity_setpoint()
{
	/* run position & altitude controllers, if enabled (otherwise use already computed velocity setpoints) */
	if (_run_pos_control) {

		// If for any reason, we get a NaN position setpoint, we better just stay where we are.
		if (PX4_ISFINITE(_pos_sp(0)) && PX4_ISFINITE(_pos_sp(1))) {
			_vel_sp(0) = (_pos_sp(0) - _pos(0)) * _params.pos_p(0);
			_vel_sp(1) = (_pos_sp(1) - _pos(1)) * _params.pos_p(1);

		} else {
			_vel_sp(0) = 0.0f;
			_vel_sp(1) = 0.0f;
			warn_rate_limited("Caught invalid pos_sp in x and y");

		}
	}

	/* in auto the setpoint is already limited by the navigator */
	if (!_control_mode.flag_control_auto_enabled) {
		limit_altitude();
	}

	if (_run_alt_control) {
		if (PX4_ISFINITE(_pos_sp(2))) {
			_vel_sp(2) = (_pos_sp(2) - _pos(2)) * _params.pos_p(2);

		} else {
			_vel_sp(2) = 0.0f;
			warn_rate_limited("Caught invalid pos_sp in z");
		}
	}

	if (!_control_mode.flag_control_position_enabled) {
		_reset_pos_sp = true;
	}

	if (!_control_mode.flag_control_altitude_enabled) {
		_reset_alt_sp = true;
	}

	if (!_control_mode.flag_control_velocity_enabled) {
		_vel_sp_prev(0) = _vel(0);
		_vel_sp_prev(1) = _vel(1);
		_vel_sp(0) = 0.0f;
		_vel_sp(1) = 0.0f;
	}

	if (!_control_mode.flag_control_climb_rate_enabled) {
		_vel_sp(2) = 0.0f;
	}

	/* limit vertical upwards speed in auto takeoff and close to ground */
	float altitude_above_home = -_pos(2) + _home_pos.z;

	if (_pos_sp_triplet.current.valid
	    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF
	    && !_control_mode.flag_control_manual_enabled) {
		float vel_limit = math::gradual(altitude_above_home,
						_params.slow_land_alt2, _params.slow_land_alt1,
						_params.tko_speed, _params.vel_max_up);
		_vel_sp(2) = math::max(_vel_sp(2), -vel_limit);
	}

	// encourage pilot to respect flow sensor minimum height limitations
	if (_local_pos.limit_hagl && _local_pos.dist_bottom_valid && _control_mode.flag_control_manual_enabled
	    && _control_mode.flag_control_altitude_enabled) {
		// If distance to ground is less than limit, increment set point upwards at up to the landing descent rate
		if (_local_pos.dist_bottom < _min_hagl_limit) {
			float climb_rate_bias = fminf(1.5f * _params.pos_p(2) * (_min_hagl_limit - _local_pos.dist_bottom), _params.land_speed);
			_vel_sp(2) -= climb_rate_bias;
			_pos_sp(2) -= climb_rate_bias * _dt;

		}
	}

	/* limit vertical downwards speed (positive z) close to ground
	 * for now we use the altitude above home and assume that we want to land at same height as we took off */
	float vel_limit = math::gradual(altitude_above_home,
					_params.slow_land_alt2, _params.slow_land_alt1,
					_params.land_speed, _params.vel_max_down);

	_vel_sp(2) = math::min(_vel_sp(2), vel_limit);

	/* apply slewrate (aka acceleration limit) for smooth flying */
	if (!_control_mode.flag_control_auto_enabled && !_in_smooth_takeoff) {
		vel_sp_slewrate();
	}

	/* special velocity setpoint limitation for smooth takeoff (after slewrate!) */
	if (_in_smooth_takeoff) {
		_in_smooth_takeoff = _takeoff_vel_limit < -_vel_sp(2);
		/* ramp vertical velocity limit up to takeoff speed */
		_takeoff_vel_limit += -_vel_sp(2) * _dt / _takeoff_ramp_time.get();
		/* limit vertical velocity to the current ramp value */
		_vel_sp(2) = math::max(_vel_sp(2), -_takeoff_vel_limit);
	}

	/* make sure velocity setpoint is constrained in all directions (xyz) */
	float vel_norm_xy = sqrtf(_vel_sp(0) * _vel_sp(0) + _vel_sp(1) * _vel_sp(1));

	/* check if the velocity demand is significant */
	_vel_sp_significant =  vel_norm_xy > 0.5f * _vel_max_xy;

	if (vel_norm_xy > _vel_max_xy) {
		_vel_sp(0) = _vel_sp(0) * _vel_max_xy / vel_norm_xy;
		_vel_sp(1) = _vel_sp(1) * _vel_max_xy / vel_norm_xy;
	}

	_vel_sp(2) = math::constrain(_vel_sp(2), -_params.vel_max_up, _params.vel_max_down);

	_vel_sp_prev = _vel_sp;
}

void
MulticopterPositionControl::calculate_thrust_setpoint()
{
	/* reset integrals if needed */
	if (_control_mode.flag_control_climb_rate_enabled) {
		if (_reset_int_z) {
			_reset_int_z = false;
			_thrust_int(2) = 0.0f;
		}

	} else {
		_reset_int_z = true;
	}

	if (_control_mode.flag_control_velocity_enabled) {
		if (_reset_int_xy) {
			_reset_int_xy = false;
			_thrust_int(0) = 0.0f;
			_thrust_int(1) = 0.0f;
		}

	} else {
		_reset_int_xy = true;
	}

	/* if any of the velocity setpoint is bogus, it's probably safest to command no velocity at all. */
	for (int i = 0; i < 3; ++i) {
		if (!PX4_ISFINITE(_vel_sp(i))) {
			_vel_sp(i) = 0.0f;
		}
	}

	/* velocity error */
	matrix::Vector3f vel_err = _vel_sp - _vel;

	/* thrust vector in NED frame */
	matrix::Vector3f thrust_sp;

	if (_control_mode.flag_control_acceleration_enabled && _pos_sp_triplet.current.acceleration_valid) {
		thrust_sp = matrix::Vector3f(_pos_sp_triplet.current.a_x, _pos_sp_triplet.current.a_y, _pos_sp_triplet.current.a_z);

	} else {
		thrust_sp = vel_err.emult(_params.vel_p) + _vel_err_d.emult(_params.vel_d)
			    + _thrust_int - matrix::Vector3f(0.0f, 0.0f, _params.thr_hover);
	}

	if (!_control_mode.flag_control_velocity_enabled && !_control_mode.flag_control_acceleration_enabled) {
		thrust_sp(0) = 0.0f;
		thrust_sp(1) = 0.0f;
	}

	if (!in_auto_takeoff() && !manual_wants_takeoff()) {
		if (_vehicle_land_detected.ground_contact) {
			/* if still or already on ground command zero xy thrust_sp in body
			 * frame to consider uneven ground */

			/* thrust setpoint in body frame*/
			matrix::Vector3f thrust_sp_body = _R.transpose() * thrust_sp;

			/* we dont want to make any correction in body x and y*/
			thrust_sp_body(0) = 0.0f;
			thrust_sp_body(1) = 0.0f;

			/* make sure z component of thrust_sp_body is larger than 0 (positive thrust is downward) */
			thrust_sp_body(2) = thrust_sp(2) > 0.0f ? thrust_sp(2) : 0.0f;

			/* convert back to local frame (NED) */
			thrust_sp = _R * thrust_sp_body;
		}

		if (_vehicle_land_detected.maybe_landed) {
			/* we set thrust to zero
			 * this will help to decide if we are actually landed or not
			 */
			thrust_sp.zero();
		}
	}

	if (!_control_mode.flag_control_climb_rate_enabled && !_control_mode.flag_control_acceleration_enabled) {
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

	float tilt_max = _params.tilt_max_air;
	float thr_max = _params.thr_max;

	// We can only run the control if we're already in-air, have a takeoff setpoint,
	// or if we're in offboard control.
	// Otherwise, we should just bail out
	if (_vehicle_land_detected.landed && !in_auto_takeoff() && !manual_wants_takeoff()) {
		// Keep throttle low while still on ground.
		thr_max = 0.0f;

	} else if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid &&
		   _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {

		/* adjust limits for landing mode */
		/* limit max tilt and min lift when landing */
		tilt_max = _params.tilt_max_land;
	}

	/* limit min lift */
	if (-thrust_sp(2) < thr_min) {
		thrust_sp(2) = -thr_min;
		/* Don't freeze altitude integral if it wants to throttle up */
		saturation_z = vel_err(2) > 0.0f ? true : saturation_z;
	}

	if (_control_mode.flag_control_velocity_enabled || _control_mode.flag_control_acceleration_enabled) {

		/* limit max tilt */
		if (thr_min >= 0.0f && tilt_max < M_PI_F / 2.0f - 0.05f) {
			/* absolute horizontal thrust */
			float thrust_sp_xy_len = matrix::Vector2f(thrust_sp(0), thrust_sp(1)).length();

			if (thrust_sp_xy_len > 0.01f) {
				/* max horizontal thrust for given vertical thrust*/
				float thrust_xy_max = -thrust_sp(2) * tanf(tilt_max);

				if (thrust_sp_xy_len > thrust_xy_max) {
					float k = thrust_xy_max / thrust_sp_xy_len;
					thrust_sp(0) *= k;
					thrust_sp(1) *= k;
					/* Don't freeze x,y integrals if they both want to throttle down */
					saturation_xy = ((vel_err(0) * _vel_sp(0) < 0.0f) && (vel_err(1) * _vel_sp(1) < 0.0f)) ? saturation_xy : true;
				}
			}
		}
	}

	if (_control_mode.flag_control_climb_rate_enabled && !_control_mode.flag_control_velocity_enabled) {
		/* thrust compensation when vertical velocity but not horizontal velocity is controlled */
		float att_comp;

		const float tilt_cos_max = 0.7f;

		if (_R(2, 2) > tilt_cos_max) {
			att_comp = 1.0f / _R(2, 2);

		} else if (_R(2, 2) > 0.0f) {
			att_comp = ((1.0f / tilt_cos_max - 1.0f) / tilt_cos_max) * _R(2, 2) + 1.0f;
			saturation_z = true;

		} else {
			att_comp = 1.0f;
			saturation_z = true;
		}

		thrust_sp(2) *= att_comp;
	}

	/* Calculate desired total thrust amount in body z direction. */
	/* To compensate for excess thrust during attitude tracking errors we
	 * project the desired thrust force vector F onto the real vehicle's thrust axis in NED:
	 * body thrust axis [0,0,-1]' rotated by R is: R*[0,0,-1]' = -R_z */
	matrix::Vector3f R_z(_R(0, 2), _R(1, 2), _R(2, 2));

	/* recalculate because it might have changed */
	float thrust_body_z = thrust_sp.dot(-R_z);

	/* limit max thrust */
	if (fabsf(thrust_body_z) > thr_max) {
		if (thrust_sp(2) < 0.0f) {
			if (-thrust_sp(2) > thr_max) {
				/* thrust Z component is too large, limit it */
				thrust_sp(0) = 0.0f;
				thrust_sp(1) = 0.0f;
				thrust_sp(2) = -thr_max;
				saturation_xy = true;
				/* Don't freeze altitude integral if it wants to throttle down */
				saturation_z = vel_err(2) < 0.0f ? true : saturation_z;

			} else {
				/* preserve thrust Z component and lower XY, keeping altitude is more important than position */
				float thrust_xy_max = sqrtf(thr_max * thr_max - thrust_sp(2) * thrust_sp(2));
				float thrust_xy_abs = matrix::Vector2f(thrust_sp(0), thrust_sp(1)).length();
				float k = thrust_xy_max / thrust_xy_abs;
				thrust_sp(0) *= k;
				thrust_sp(1) *= k;
				/* Don't freeze x,y integrals if they both want to throttle down */
				saturation_xy = ((vel_err(0) * _vel_sp(0) < 0.0f) && (vel_err(1) * _vel_sp(1) < 0.0f)) ? saturation_xy : true;
			}

		} else {
			/* Z component is positive, going down (Z is positive down in NED), simply limit thrust vector */
			float k = thr_max / fabsf(thrust_body_z);
			thrust_sp *= k;
			saturation_xy = true;
			saturation_z = true;
		}

		thrust_body_z = thr_max;
	}

	/* if any of the thrust setpoint is bogus, send out a warning */
	if (!PX4_ISFINITE(thrust_sp(0)) || !PX4_ISFINITE(thrust_sp(1)) || !PX4_ISFINITE(thrust_sp(2))) {
		warn_rate_limited("Thrust setpoint not finite");
	}

	_att_sp.thrust = math::max(thrust_body_z, thr_min);

	/* update integrals */
	if (_control_mode.flag_control_velocity_enabled && !saturation_xy) {
		_thrust_int(0) += vel_err(0) * _params.vel_i(0) * _dt;
		_thrust_int(1) += vel_err(1) * _params.vel_i(1) * _dt;
	}

	if (_control_mode.flag_control_climb_rate_enabled && !saturation_z) {
		_thrust_int(2) += vel_err(2) * _params.vel_i(2) * _dt;
	}

	/* calculate attitude setpoint from thrust vector */
	if (_control_mode.flag_control_velocity_enabled || _control_mode.flag_control_acceleration_enabled) {
		/* desired body_z axis = -normalize(thrust_vector) */
		matrix::Vector3f body_x;
		matrix::Vector3f body_y;
		matrix::Vector3f body_z;

		if (thrust_sp.length() > SIGMA_NORM) {
			body_z = -thrust_sp.normalized();

		} else {
			/* no thrust, set Z axis to safe value */
			body_z = {0.0f, 0.0f, 1.0f};
		}

		/* vector of desired yaw direction in XY plane, rotated by PI/2 */
		matrix::Vector3f y_C(-sinf(_att_sp.yaw_body), cosf(_att_sp.yaw_body), 0.0f);

		if (fabsf(body_z(2)) > SIGMA_SINGLE_OP) {
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
			_R_setpoint(i, 0) = body_x(i);
			_R_setpoint(i, 1) = body_y(i);
			_R_setpoint(i, 2) = body_z(i);
		}

		/* copy quaternion setpoint to attitude setpoint topic */
		matrix::Quatf q_sp = _R_setpoint;
		q_sp.copyTo(_att_sp.q_d);
		_att_sp.q_d_valid = true;

		/* calculate euler angles, for logging only, must not be used for control */
		matrix::Eulerf euler = _R_setpoint;
		_att_sp.roll_body = euler(0);
		_att_sp.pitch_body = euler(1);
		/* yaw already used to construct rot matrix, but actual rotation matrix can have different yaw near singularity */

	} else if (!_control_mode.flag_control_manual_enabled) {
		/* autonomous altitude control without position control (failsafe landing),
		 * force level attitude, don't change yaw */
		_R_setpoint = matrix::Eulerf(0.0f, 0.0f, _att_sp.yaw_body);

		/* copy quaternion setpoint to attitude setpoint topic */
		matrix::Quatf q_sp = _R_setpoint;
		q_sp.copyTo(_att_sp.q_d);
		_att_sp.q_d_valid = true;

		_att_sp.roll_body = 0.0f;
		_att_sp.pitch_body = 0.0f;
	}

	/* save thrust setpoint for logging */
	_local_pos_sp.acc_x = thrust_sp(0) * CONSTANTS_ONE_G;
	_local_pos_sp.acc_y = thrust_sp(1) * CONSTANTS_ONE_G;
	_local_pos_sp.acc_z = thrust_sp(2) * CONSTANTS_ONE_G;

	_att_sp.timestamp = hrt_absolute_time();
}

void
MulticopterPositionControl::generate_attitude_setpoint()
{
	// yaw setpoint is integrated over time, but we don't want to integrate the offset's
	_att_sp.yaw_body -= _man_yaw_offset;
	_man_yaw_offset = 0.f;

	/* reset yaw setpoint to current position if needed */
	if (_reset_yaw_sp) {
		_reset_yaw_sp = false;
		_att_sp.yaw_body = _yaw;

	} else if (!_vehicle_land_detected.landed &&
		   !(!_control_mode.flag_control_altitude_enabled && _manual.z < 0.1f)) {

		/* do not move yaw while sitting on the ground */

		/* we want to know the real constraint, and global overrides manual */
		const float yaw_rate_max = (_params.man_yaw_max < _params.global_yaw_max) ? _params.man_yaw_max :
					   _params.global_yaw_max;
		const float yaw_offset_max = yaw_rate_max / _params.mc_att_yaw_p;

		_att_sp.yaw_sp_move_rate = _manual.r * yaw_rate_max;
		float yaw_target = _wrap_pi(_att_sp.yaw_body + _att_sp.yaw_sp_move_rate * _dt);
		float yaw_offs = _wrap_pi(yaw_target - _yaw);

		// If the yaw offset became too big for the system to track stop
		// shifting it, only allow if it would make the offset smaller again.
		if (fabsf(yaw_offs) < yaw_offset_max ||
		    (_att_sp.yaw_sp_move_rate > 0 && yaw_offs < 0) ||
		    (_att_sp.yaw_sp_move_rate < 0 && yaw_offs > 0)) {
			_att_sp.yaw_body = yaw_target;
		}
	}

	/* control throttle directly if no climb rate controller is active */
	if (!_control_mode.flag_control_climb_rate_enabled) {
		float thr_val = throttle_curve(_manual.z, _params.thr_hover);
		_att_sp.thrust = math::min(thr_val, _manual_thr_max.get());

		/* enforce minimum throttle if not landed */
		if (!_vehicle_land_detected.landed) {
			_att_sp.thrust = math::max(_att_sp.thrust, _manual_thr_min.get());
		}
	}

	/* control roll and pitch directly if no aiding velocity controller is active */
	if (!_control_mode.flag_control_velocity_enabled) {

		/*
		 * Input mapping for roll & pitch setpoints
		 * ----------------------------------------
		 * This simplest thing to do is map the y & x inputs directly to roll and pitch, and scale according to the max
		 * tilt angle.
		 * But this has several issues:
		 * - The maximum tilt angle cannot easily be restricted. By limiting the roll and pitch separately,
		 *   it would be possible to get to a higher tilt angle by combining roll and pitch (the difference is
		 *   around 15 degrees maximum, so quite noticeable). Limiting this angle is not simple in roll-pitch-space,
		 *   it requires to limit the tilt angle = acos(cos(roll) * cos(pitch)) in a meaningful way (eg. scaling both
		 *   roll and pitch).
		 * - Moving the stick diagonally, such that |x| = |y|, does not move the vehicle towards a 45 degrees angle.
		 *   The direction angle towards the max tilt in the XY-plane is atan(1/cos(x)). Which means it even depends
		 *   on the tilt angle (for a tilt angle of 35 degrees, it's off by about 5 degrees).
		 *
		 * So instead we control the following 2 angles:
		 * - tilt angle, given by sqrt(x*x + y*y)
		 * - the direction of the maximum tilt in the XY-plane, which also defines the direction of the motion
		 *
		 * This allows a simple limitation of the tilt angle, the vehicle flies towards the direction that the stick
		 * points to, and changes of the stick input are linear.
		 */
		const float x = _manual.x * _params.man_tilt_max;
		const float y = _manual.y * _params.man_tilt_max;

		// we want to fly towards the direction of (x, y), so we use a perpendicular axis angle vector in the XY-plane
		matrix::Vector2f v = matrix::Vector2f(y, -x);
		float v_norm = v.norm(); // the norm of v defines the tilt angle

		if (v_norm > _params.man_tilt_max) { // limit to the configured maximum tilt angle
			v *= _params.man_tilt_max / v_norm;
		}

		matrix::Quatf q_sp_rpy = matrix::AxisAnglef(v(0), v(1), 0.f);
		// The axis angle can change the yaw as well (but only at higher tilt angles. Note: we're talking
		// about the world frame here, in terms of body frame the yaw rate will be unaffected).
		// This the the formula by how much the yaw changes:
		//   let a := tilt angle, b := atan(y/x) (direction of maximum tilt)
		//   yaw = atan(-2 * sin(b) * cos(b) * sin^2(a/2) / (1 - 2 * cos^2(b) * sin^2(a/2))).
		matrix::Eulerf euler_sp = q_sp_rpy;
		// Since the yaw setpoint is integrated, we extract the offset here,
		// so that we can remove it before the next iteration
		_man_yaw_offset = euler_sp(2);

		// update the setpoints
		_att_sp.roll_body = euler_sp(0);
		_att_sp.pitch_body = euler_sp(1);
		_att_sp.yaw_body += euler_sp(2);

		/* only if we're a VTOL and optimal recovery is not used, modify roll/pitch */
		if (_vehicle_status.is_vtol) {
			// construct attitude setpoint rotation matrix. modify the setpoints for roll
			// and pitch such that they reflect the user's intention even if a yaw error
			// (yaw_sp - yaw) is present. In the presence of a yaw error constructing a rotation matrix
			// from the pure euler angle setpoints will lead to unexpected attitude behaviour from
			// the user's view as the euler angle sequence uses the  yaw setpoint and not the current
			// heading of the vehicle.
			// The effect of that can be seen with:
			// - roll/pitch into one direction, keep it fixed (at high angle)
			// - apply a fast yaw rotation
			// - look at the roll and pitch angles: they should stay pretty much the same as when not yawing

			// calculate our current yaw error
			float yaw_error = _wrap_pi(_att_sp.yaw_body - _yaw);

			// compute the vector obtained by rotating a z unit vector by the rotation
			// given by the roll and pitch commands of the user
			matrix::Vector3f zB = {0.0f, 0.0f, 1.0f};
			matrix::Dcmf R_sp_roll_pitch = matrix::Eulerf(_att_sp.roll_body, _att_sp.pitch_body, 0.0f);
			matrix::Vector3f z_roll_pitch_sp = R_sp_roll_pitch * zB;

			// transform the vector into a new frame which is rotated around the z axis
			// by the current yaw error. this vector defines the desired tilt when we look
			// into the direction of the desired heading
			matrix::Dcmf R_yaw_correction = matrix::Eulerf(0.0f, 0.0f, -yaw_error);
			z_roll_pitch_sp = R_yaw_correction * z_roll_pitch_sp;

			// use the formula z_roll_pitch_sp = R_tilt * [0;0;1]
			// R_tilt is computed from_euler; only true if cos(roll) not equal zero
			// -> valid if roll is not +-pi/2;
			_att_sp.roll_body = -asinf(z_roll_pitch_sp(1));
			_att_sp.pitch_body = atan2f(z_roll_pitch_sp(0), z_roll_pitch_sp(2));
		}

		/* copy quaternion setpoint to attitude setpoint topic */
		matrix::Quatf q_sp = matrix::Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body);
		q_sp.copyTo(_att_sp.q_d);
		_att_sp.q_d_valid = true;
	}

	// Only switch the landing gear up if we are not landed and if
	// the user switched from gear down to gear up.
	// If the user had the switch in the gear up position and took off ignore it
	// until he toggles the switch to avoid retracting the gear immediately on takeoff.
	if (_manual.gear_switch == manual_control_setpoint_s::SWITCH_POS_ON && _gear_state_initialized &&
	    !_vehicle_land_detected.landed) {
		_att_sp.landing_gear = vehicle_attitude_setpoint_s::LANDING_GEAR_UP;

	} else if (_manual.gear_switch == manual_control_setpoint_s::SWITCH_POS_OFF) {
		_att_sp.landing_gear = vehicle_attitude_setpoint_s::LANDING_GEAR_DOWN;
		// Switching the gear off does put it into a safe defined state
		_gear_state_initialized = true;
	}

	_att_sp.timestamp = hrt_absolute_time();
}

bool MulticopterPositionControl::manual_wants_takeoff()
{
	const bool has_manual_control_present = _control_mode.flag_control_manual_enabled && _manual.timestamp > 0;

	// Manual takeoff is triggered if the throttle stick is above 65%.
	return (has_manual_control_present && _manual.z > 0.65f);
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

		/* set default max velocity in xy to vel_max
		 * Apply estimator limits if applicable */
		if (_local_pos.vxy_max > 0.001f) {
			// use the minimum of the estimator and user specified limit
			_vel_max_xy = fminf(_params.vel_max_xy, _local_pos.vxy_max);
			// Allow for a minimum of 0.3 m/s for repositioning
			_vel_max_xy = fmaxf(_vel_max_xy, 0.3f);

		} else if (_vel_sp_significant) {
			// raise the limit at a constant rate up to the user specified value
			if (_vel_max_xy >= _params.vel_max_xy) {
				_vel_max_xy = _params.vel_max_xy;

			} else {
				_vel_max_xy += dt * _params.acc_max_flow_xy;
			}
		}

		/* reset flags when landed */
		if (_vehicle_land_detected.landed) {
			_reset_pos_sp = true;
			_reset_alt_sp = true;
			_do_reset_alt_pos_flag = true;
			_mode_auto = false;
			_pos_hold_engaged = false;
			_alt_hold_engaged = false;
			_run_pos_control = true;
			_run_alt_control = true;
			_reset_int_z = true;
			_reset_int_xy = true;
			_reset_yaw_sp = true;
			_hold_offboard_xy = false;
			_hold_offboard_z = false;
			_in_landing = false;
			_lnd_reached_ground = false;

			/* also reset previous setpoints */
			_yaw_takeoff = _yaw;
			_vel_sp_prev.zero();
			_vel_prev.zero();

			/* make sure attitude setpoint output "disables" attitude control
			 * TODO: we need a defined setpoint to do this properly especially when adjusting the mixer */
			_att_sp.thrust = 0.0f;
			_att_sp.timestamp = hrt_absolute_time();

			/* reset velocity limit */
			_vel_max_xy = _params.vel_max_xy;
		}

		/* reset setpoints and integrators VTOL in FW mode */
		if (_vehicle_status.is_vtol && !_vehicle_status.is_rotary_wing) {
			_reset_alt_sp = true;
			_reset_int_xy = true;
			_reset_int_z = true;
			_reset_pos_sp = true;
			_reset_yaw_sp = true;
			_vel_sp_prev = _vel;
		}

		if (!_in_smooth_takeoff && _vehicle_land_detected.landed && _control_mode.flag_armed &&
		    (in_auto_takeoff() || manual_wants_takeoff())) {
			_in_smooth_takeoff = true;
			// This ramp starts negative and goes to positive later because we want to
			// be as smooth as possible. If we start at 0, we alrady jump to hover throttle.
			_takeoff_vel_limit = -0.5f;
		}

		else if (!_control_mode.flag_armed) {
			// If we're disarmed and for some reason were in a smooth takeoff, we reset that.
			_in_smooth_takeoff = false;
		}

		/* set triplets to invalid if we just landed */
		if (_vehicle_land_detected.landed && !was_landed) {
			_pos_sp_triplet.current.valid = false;
		}

		was_landed = _vehicle_land_detected.landed;

		update_ref();

		update_velocity_derivative();

		// reset the horizontal and vertical position hold flags for non-manual modes
		// or if position / altitude is not controlled
		if (!_control_mode.flag_control_position_enabled || !_control_mode.flag_control_manual_enabled) {
			_pos_hold_engaged = false;
		}

		if (!_control_mode.flag_control_altitude_enabled || !_control_mode.flag_control_manual_enabled) {
			_alt_hold_engaged = false;
		}

		if (_control_mode.flag_control_altitude_enabled ||
		    _control_mode.flag_control_position_enabled ||
		    _control_mode.flag_control_climb_rate_enabled ||
		    _control_mode.flag_control_velocity_enabled ||
		    _control_mode.flag_control_acceleration_enabled) {

			do_control();

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
			_reset_pos_sp = true;
			_reset_alt_sp = true;
			_do_reset_alt_pos_flag = true;
			_mode_auto = false;
			_reset_int_z = true;
			_reset_int_xy = true;

			/* store last velocity in case a mode switch to position control occurs */
			_vel_sp_prev = _vel;
		}

		/* generate attitude setpoint from manual controls */
		if (_control_mode.flag_control_manual_enabled && _control_mode.flag_control_attitude_enabled) {

			generate_attitude_setpoint();

		} else {
			_reset_yaw_sp = true;
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

	mavlink_log_info(&_mavlink_log_pub, "[mpc] stopped");

	_control_task = -1;
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
