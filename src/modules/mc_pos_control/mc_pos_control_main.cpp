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

#include <uORB/topics/control_state.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

#include <float.h>
#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>
#include <systemlib/mavlink_log.h>

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

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

	bool		cross_sphere_line(const math::Vector<3> &sphere_c, const float sphere_r,
					  const math::Vector<3> &line_a, const math::Vector<3> &line_b, math::Vector<3> &res);

private:

	/** Time in us that direction change condition has to be true for direction change state */
	static constexpr uint64_t DIRECTION_CHANGE_TRIGGER_TIME_US = 100000;

	bool		_task_should_exit = false;			/**<true if task should exit */
	bool		_gear_state_initialized = false;	/**<true if the gear state has been initialized */
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
	bool 		_in_takeoff = false; 				/**<true if takeoff ramp is applied */
	bool 		_in_landing = false;				/**<true if landing descent (only used in auto) */
	bool 		_lnd_reached_ground = false; 		/**<true if controller assumes the vehicle has reached the ground after landing */
	bool 		_limit_vel_xy = false;				/**<true if velocity in xy should be farther limited */
	bool 		_transition_to_non_manual = false;  /**<true if transition from manual to auto */


	int		_control_task;			/**< task handle for task */
	orb_advert_t	_mavlink_log_pub;		/**< mavlink log advert */

	int		_vehicle_status_sub;		/**< vehicle status subscription */
	int		_vehicle_land_detected_sub;	/**< vehicle land detected subscription */
	int		_ctrl_state_sub;		/**< control state subscription */
	int		_control_mode_sub;		/**< vehicle control mode subscription */
	int		_params_sub;			/**< notification of parameter updates */
	int		_manual_sub;			/**< notification of manual control updates */
	int		_local_pos_sub;			/**< vehicle local position */
	int		_pos_sp_triplet_sub;		/**< position setpoint triplet */
	int		_home_pos_sub; 			/**< home position */
	orb_advert_t	_att_sp_pub;			/**< attitude setpoint publication */
	orb_advert_t	_local_pos_sp_pub;		/**< vehicle local position setpoint publication */
	orb_advert_t	_global_vel_sp_pub;		/**< vehicle global velocity setpoint publication */

	orb_id_t _attitude_setpoint_id;

	struct vehicle_status_s 			_vehicle_status; 	/**< vehicle status */
	struct vehicle_land_detected_s 			_vehicle_land_detected;	/**< vehicle land detected */
	struct control_state_s				_ctrl_state;		/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct vehicle_control_mode_s			_control_mode;		/**< vehicle control mode */
	struct vehicle_local_position_s			_local_pos;		/**< vehicle local position */
	struct position_setpoint_triplet_s		_pos_sp_triplet;	/**< vehicle global position setpoint triplet */
	struct vehicle_local_position_setpoint_s	_local_pos_sp;		/**< vehicle local position setpoint */
	struct vehicle_global_velocity_setpoint_s	_global_vel_sp;		/**< vehicle global velocity setpoint */
	struct home_position_s				_home_pos; 				/**< home position */

	control::BlockParamFloat _manual_thr_min; /**< minimal throttle output when flying in manual mode */
	control::BlockParamFloat _manual_thr_max; /**< maximal throttle output when flying in manual mode */
	control::BlockParamFloat _manual_land_alt; /**< altitude where landing is likely flying with sticks but in pos mode */
	control::BlockParamFloat _xy_vel_man_expo; /**< ratio of exponential curve for stick input in xy direction pos mode */
	control::BlockParamFloat _z_vel_man_expo; /**< ratio of exponential curve for stick input in xy direction pos mode */
	control::BlockParamFloat _hold_dz; /**< deadzone around the center for the sticks when flying in position mode */
	control::BlockParamFloat _acceleration_hor_max; /**< maximum velocity setpoint slewrate for auto & fast manual brake */
	control::BlockParamFloat _acceleration_hor_manual; /**< maximum velocity setpoint slewrate for manual acceleration */
	control::BlockParamFloat _deceleration_hor_slow; /**< slow velocity setpoint slewrate for manual deceleration*/
	control::BlockParamFloat _acceleration_z_max_up; /** max acceleration up */
	control::BlockParamFloat _acceleration_z_max_down; /** max acceleration down */
	control::BlockParamFloat _target_threshold_xy; /**< distance threshold for slowdown close to target during mission */
	control::BlockParamFloat _min_cruise_speed; /**< minimum cruising speed when passing waypoint */
	control::BlockParamFloat _velocity_hor_manual; /**< target velocity in manual controlled mode at full speed*/
	control::BlockParamFloat _takeoff_ramp_time; /**< time contant for smooth takeoff ramp */
	control::BlockParamFloat _jerk_hor_max; /**< maximum jerk in manual controlled mode when breaking to zero */
	control::BlockParamFloat _jerk_hor_min; /**< minimum jerk in manual controlled mode when breaking to zero */

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
		param_t z_ff;
		param_t slow_land_alt1;
		param_t slow_land_alt2;
		param_t xy_p;
		param_t xy_vel_p;
		param_t xy_vel_i;
		param_t xy_vel_d;
		param_t xy_vel_max;
		param_t xy_vel_cruise;
		param_t xy_ff;
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
		param_t opt_recover;
		param_t rc_flt_smp_rate;
		param_t rc_flt_cutoff;
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
		uint32_t alt_mode;
		int opt_recover;
		float rc_flt_smp_rate;
		float rc_flt_cutoff;

		math::Vector<3> pos_p;
		math::Vector<3> vel_p;
		math::Vector<3> vel_i;
		math::Vector<3> vel_d;
		math::Vector<3> vel_ff;
	}		_params;

	struct map_projection_reference_s _ref_pos;
	float _ref_alt;
	hrt_abstime _ref_timestamp;

	math::Vector<3> _thrust_int;
	math::Vector<3> _pos;
	math::Vector<3> _pos_sp;
	math::Vector<3> _vel;
	math::Vector<3> _vel_sp;
	math::Vector<3> _vel_prev;			/**< velocity on previous step */
	math::Vector<3> _vel_ff;
	math::Vector<3> _vel_sp_prev;
	math::Vector<3> _vel_err_d;		/**< derivative of current velocity */
	math::Vector<3> _curr_pos_sp;  /**< current setpoint of the triplets */
	math::Vector<3> _prev_pos_sp; /**< previous setpoint of the triples */
	matrix::Vector2f _stick_input_xy_prev; /*for manual controlled mode to detect direction change */


	math::Matrix<3, 3> _R;			/**< rotation matrix from attitude quaternions */
	float _yaw;				/**< yaw angle (euler) */
	float _yaw_takeoff;	/**< home yaw angle present when vehicle was taking off (euler) */
	float _vel_z_lp;
	float _acc_z_lp;
	float _vel_max_xy;  /**< equal to vel_max except in auto mode when close to target */
	float _acceleration_state_dependent_xy; /* acceleration limit applied in manual mode */
	float _acceleration_state_dependent_z; /* acceleration limit applied in manual mode in z */
	float _manual_jerk_limit_xy; /**< jerk limit in manual mode dependent on stick input */
	float _manual_jerk_limit_z; /**< jerk limit in manual mode in z */

	float _takeoff_vel_limit; /**< velocity limit value which gets ramped up */

	// counters for reset events on position and velocity states
	// they are used to identify a reset event
	uint8_t _z_reset_counter;
	uint8_t _xy_reset_counter;
	uint8_t _vz_reset_counter;
	uint8_t _vxy_reset_counter;
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

	void		slow_land_gradual_velocity_limit();

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
	void		control_manual(float dt);

	void		control_non_manual(float dt);

	/**
	 * Set position setpoint using offboard control
	 */
	void		control_offboard(float dt);

	/**
	 * Set position setpoint for AUTO
	 */
	void		control_auto(float dt);

	void control_position(float dt);
	void calculate_velocity_setpoint(float dt);
	void calculate_thrust_setpoint(float dt);

	void vel_sp_slewrate(float dt);

	void update_velocity_derivative();

	void do_control(float dt);

	void generate_attitude_setpoint(float dt);

	float get_cruising_speed_xy();

	bool in_auto_takeoff();

	void set_manual_acceleration_xy(matrix::Vector2f &stick_input_xy_NED, const float dt);

	void set_manual_acceleration_z(float &max_acc_z, const float stick_input_z_NED, const float dt);

	/**
	 * limit altitude based on several conditions
	 */
	void limit_altitude();

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
	_ctrl_state_sub(-1),
	_control_mode_sub(-1),
	_params_sub(-1),
	_manual_sub(-1),
	_local_pos_sub(-1),
	_pos_sp_triplet_sub(-1),
	_home_pos_sub(-1),

	/* publications */
	_att_sp_pub(nullptr),
	_local_pos_sp_pub(nullptr),
	_global_vel_sp_pub(nullptr),
	_attitude_setpoint_id(nullptr),
	_vehicle_status{},
	_vehicle_land_detected{},
	_ctrl_state{},
	_att_sp{},
	_manual{},
	_control_mode{},
	_local_pos{},
	_pos_sp_triplet{},
	_local_pos_sp{},
	_global_vel_sp{},
	_home_pos{},
	_manual_thr_min(this, "MANTHR_MIN"),
	_manual_thr_max(this, "MANTHR_MAX"),
	_manual_land_alt(this, "MIS_LTRMIN_ALT", false),
	_xy_vel_man_expo(this, "XY_MAN_EXPO"),
	_z_vel_man_expo(this, "Z_MAN_EXPO"),
	_hold_dz(this, "HOLD_DZ"),
	_acceleration_hor_max(this, "ACC_HOR_MAX", true),
	_acceleration_hor_manual(this, "ACC_HOR_MAN", true),
	_deceleration_hor_slow(this, "DEC_HOR_SLOW", true),
	_acceleration_z_max_up(this, "ACC_UP_MAX", true),
	_acceleration_z_max_down(this, "ACC_DOWN_MAX", true),
	_target_threshold_xy(this, "TARGET_THRE"),
	_velocity_hor_manual(this, "VEL_MAN_MAX", true),
	_takeoff_ramp_time(this, "TKO_RAMP_T", true),
	_min_cruise_speed(this, "CRUISE_MIN", true),
	_jerk_hor_max(this, "JERK_MAX", true),
	_jerk_hor_min(this, "JERK_MIN", true),
	_vel_x_deriv(this, "VELD"),
	_vel_y_deriv(this, "VELD"),
	_vel_z_deriv(this, "VELD"),
	_manual_direction_change_hysteresis(false),
	_filter_manual_pitch(50.0f, 10.0f),
	_filter_manual_roll(50.0f, 10.0f),
	_user_intention_xy(brake),
	_user_intention_z(brake),
	_ref_alt(0.0f),
	_ref_timestamp(0),
	_yaw(0.0f),
	_yaw_takeoff(0.0f),
	_vel_z_lp(0),
	_acc_z_lp(0),
	_vel_max_xy(0.0f),
	_acceleration_state_dependent_xy(0.0f),
	_acceleration_state_dependent_z(0.0f),
	_manual_jerk_limit_xy(1.0f),
	_manual_jerk_limit_z(1.0f),
	_takeoff_vel_limit(0.0f),
	_z_reset_counter(0),
	_xy_reset_counter(0),
	_vz_reset_counter(0),
	_vxy_reset_counter(0),
	_heading_reset_counter(0)
{
	// Make the quaternion valid for control state
	_ctrl_state.q[0] = 1.0f;

	_ref_pos = {};

	/* set trigger time for manual direction change detection */
	_manual_direction_change_hysteresis.set_hysteresis_time_from(false, DIRECTION_CHANGE_TRIGGER_TIME_US);

	memset(&_ref_pos, 0, sizeof(_ref_pos));

	_params.pos_p.zero();
	_params.vel_p.zero();
	_params.vel_i.zero();
	_params.vel_d.zero();
	_params.vel_ff.zero();

	_pos.zero();
	_pos_sp.zero();
	_vel.zero();
	_vel_sp.zero();
	_vel_prev.zero();
	_vel_ff.zero();
	_vel_sp_prev.zero();
	_vel_err_d.zero();
	_curr_pos_sp.zero();
	_prev_pos_sp.zero();
	_stick_input_xy_prev = matrix::Vector2f(0.0f, 0.0f);

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
	_params_handles.z_ff		= param_find("MPC_Z_FF");

	_params_handles.xy_p		= param_find("MPC_XY_P");
	_params_handles.xy_vel_p	= param_find("MPC_XY_VEL_P");
	_params_handles.xy_vel_i	= param_find("MPC_XY_VEL_I");
	_params_handles.xy_vel_d	= param_find("MPC_XY_VEL_D");
	_params_handles.xy_vel_max	= param_find("MPC_XY_VEL_MAX");
	_params_handles.xy_vel_cruise	= param_find("MPC_XY_CRUISE");
	_params_handles.xy_ff		= param_find("MPC_XY_FF");

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
	_params_handles.opt_recover = param_find("VT_OPT_RECOV_EN");
	_params_handles.rc_flt_cutoff = param_find("RC_FLT_CUTOFF");
	_params_handles.rc_flt_smp_rate = param_find("RC_FLT_SMP_RATE");

	/* fetch initial parameter values */
	parameters_update(true);

	/* default limit for acceleration and manual jerk*/
	_acceleration_state_dependent_xy = _acceleration_hor_max.get();
	_manual_jerk_limit_xy = _jerk_hor_max.get();

	/* acceleration up must be larger than acceleration down */
	if (_acceleration_z_max_up.get() < _acceleration_z_max_down.get()) {
		_acceleration_z_max_up.set(_acceleration_z_max_down.get());
	}

	/* for z direction we use fixed jerk for now
	 * TODO: check if other jerk value is required */
	_acceleration_state_dependent_z = _acceleration_z_max_up.get();
	_manual_jerk_limit_z = _jerk_hor_max.get() / 5.0f;


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
		uint32_t v_i;
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
		param_get(_params_handles.xy_ff, &v);
		v = math::constrain(v, 0.0f, 1.0f);
		_params.vel_ff(0) = v;
		_params.vel_ff(1) = v;
		param_get(_params_handles.z_ff, &v);
		v = math::constrain(v, 0.0f, 1.0f);
		_params.vel_ff(2) = v;
		param_get(_params_handles.hold_max_xy, &v);
		_params.hold_max_xy = math::max(0.f, v);
		param_get(_params_handles.hold_max_z, &v);
		_params.hold_max_z = math::max(0.f, v);
		param_get(_params_handles.rc_flt_smp_rate, &(_params.rc_flt_smp_rate));
		_params.rc_flt_smp_rate = math::max(1.0f, _params.rc_flt_smp_rate);
		/* since we use filter to detect manual direction change, take half the cutoff of the stick filtering */
		param_get(_params_handles.rc_flt_cutoff * 0.7f, &(_params.rc_flt_cutoff));
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

		int i;
		param_get(_params_handles.opt_recover, &i);
		_params.opt_recover = i;

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

	orb_check(_ctrl_state_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);

		/* get current rotation matrix and euler angles from control state quaternions */
		math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
		_R = q_att.to_dcm();
		math::Vector<3> euler_angles;
		euler_angles = _R.to_euler();
		_yaw = euler_angles(2);

		if (_control_mode.flag_control_manual_enabled) {
			if (_heading_reset_counter != _ctrl_state.quat_reset_counter) {
				_heading_reset_counter = _ctrl_state.quat_reset_counter;
				math::Quaternion delta_q(_ctrl_state.delta_q_reset[0], _ctrl_state.delta_q_reset[1], _ctrl_state.delta_q_reset[2],
							 _ctrl_state.delta_q_reset[3]);

				// we only extract the heading change from the delta quaternion
				math::Vector<3> delta_euler = delta_q.to_euler();
				_att_sp.yaw_body += delta_euler(2);
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
				_pos_sp(2) += _local_pos.delta_z;
			}

			if (_xy_reset_counter != _local_pos.xy_reset_counter) {
				_pos_sp(0) += _local_pos.delta_xy[0];
				_pos_sp(1) += _local_pos.delta_xy[1];
			}

			if (_vz_reset_counter != _local_pos.vz_reset_counter) {
				_vel_sp(2) += _local_pos.delta_vz;
				_vel_sp_prev(2) +=  _local_pos.delta_vz;
			}

			if (_vxy_reset_counter != _local_pos.vxy_reset_counter) {
				_vel_sp(0) += _local_pos.delta_vxy[0];
				_vel_sp(1) += _local_pos.delta_vxy[1];
				_vel_sp_prev(0) += _local_pos.delta_vxy[0];
				_vel_sp_prev(1) += _local_pos.delta_vxy[1];
			}
		}

		// update the reset counters in any case
		_z_reset_counter = _local_pos.z_reset_counter;
		_xy_reset_counter = _local_pos.xy_reset_counter;
		_vz_reset_counter = _local_pos.vz_reset_counter;
		_vxy_reset_counter = _local_pos.vxy_reset_counter;
	}

	orb_check(_pos_sp_triplet_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);

		//set current position setpoint invalid if none of them (lat, lon and alt) is finite
		if (!PX4_ISFINITE(_pos_sp_triplet.current.lat) &&
		    !PX4_ISFINITE(_pos_sp_triplet.current.lon) &&
		    !PX4_ISFINITE(_pos_sp_triplet.current.alt)) {
			_pos_sp_triplet.current.valid = false;
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
	if (_local_pos.ref_timestamp != _ref_timestamp) {
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

		if (_ref_timestamp != 0) {
			// reproject position setpoint to new reference
			// this effectively adjusts the position setpoint to keep the vehicle
			// in its current local position. It would only change its
			// global position on the next setpoint update.
			map_projection_project(&_ref_pos, lat_sp, lon_sp, &_pos_sp.data[0], &_pos_sp.data[1]);
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
	/* in altitude control, limit setpoint */
	if (_run_alt_control && _pos_sp(2) <= -_vehicle_land_detected.alt_max) {
		_pos_sp(2) = -_vehicle_land_detected.alt_max;
		return;
	}

	/* in velocity control mode and want to fly upwards */
	if (!_run_alt_control && _vel_sp(2) <= 0.0f) {

		/* time to travel to reach zero velocity */
		float delta_t = -_vel(2) / _acceleration_z_max_down.get();

		/* predicted position */
		float pos_z_next = _pos(2) + _vel(2) * delta_t + 0.5f *
				   _acceleration_z_max_down.get() * delta_t *delta_t;

		if (pos_z_next <= -_vehicle_land_detected.alt_max) {
			_pos_sp(2) = -_vehicle_land_detected.alt_max;
			_run_alt_control = true;
			return;
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
MulticopterPositionControl::get_cruising_speed_xy()
{
	/*
	 * in mission the user can choose cruising speed different to default
	 */
	return ((PX4_ISFINITE(_pos_sp_triplet.current.cruising_speed) && (_pos_sp_triplet.current.cruising_speed > 0.1f)) ?
		_pos_sp_triplet.current.cruising_speed : _params.vel_cruise_xy);
}

void
MulticopterPositionControl::set_manual_acceleration_z(float &max_acceleration, const float stick_z, const float dt)
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
	float min_acceleration = max_acceleration / 5.0f;

	/*
	 * update user input
	 */
	if ((_user_intention_z != brake) && (intention  == brake)) {

		/* we start with lowest acceleration */
		_acceleration_state_dependent_z = min_acceleration;

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
		float jerk = (_acceleration_z_max_up.get() - _acceleration_state_dependent_z) / dt;

		if (jerk > _manual_jerk_limit_z) {
			_acceleration_state_dependent_z = _manual_jerk_limit_z * dt + _acceleration_state_dependent_z;

		} else {
			_acceleration_state_dependent_z = _acceleration_z_max_up.get();
		}
	}

	if (_user_intention_z == acceleration) {
		_acceleration_state_dependent_z = (max_acceleration - min_acceleration) * fabsf(stick_z) + min_acceleration;
	}
}

void
MulticopterPositionControl::set_manual_acceleration_xy(matrix::Vector2f &stick_xy, const float dt)
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
		_manual_jerk_limit_xy = (_jerk_hor_max.get() - _jerk_hor_min.get()) / _velocity_hor_manual.get() *
					sqrtf(_vel(0) * _vel(0) + _vel(1) * _vel(1)) + _jerk_hor_min.get();

		/* we start braking with lowest accleration */
		_acceleration_state_dependent_xy = _deceleration_hor_slow.get();

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
			_manual_direction_change_hysteresis.set_state_and_update(!stick_vel_aligned);


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
			float jerk = (_acceleration_hor_max.get() - _acceleration_state_dependent_xy) / dt;

			if (jerk > _manual_jerk_limit_xy) {
				_acceleration_state_dependent_xy = _manual_jerk_limit_xy * dt + _acceleration_state_dependent_xy;

			} else {
				_acceleration_state_dependent_xy = _acceleration_hor_max.get();
			}

			break;
		}

	case direction_change: {

			/* limit acceleration linearly on stick input*/
			_acceleration_state_dependent_xy = (_acceleration_hor_manual.get() - _deceleration_hor_slow.get()) * stick_xy.length() +
							   _deceleration_hor_slow.get();
			break;
		}

	case acceleration: {
			/* limit acceleration linearly on stick input*/
			float acc_limit  = (_acceleration_hor_manual.get() - _deceleration_hor_slow.get()) * stick_xy.length()
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
		PX4_WARN("User intention not recognized");
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
MulticopterPositionControl::slow_land_gradual_velocity_limit()
{
	/*
	 * Make sure downward velocity (positive Z) is limited close to ground.
	 * for now we use the home altitude and assume that we want to land on a similar absolute altitude.
	 */
	float altitude_above_home = -_pos(2) + _home_pos.z;
	float vel_limit = _params.vel_max_down;

	if (altitude_above_home < _params.slow_land_alt2) {
		vel_limit = _params.land_speed;

	} else if (altitude_above_home < _params.slow_land_alt1) {
		/* linear function between the two altitudes */
		float a = (_params.vel_max_down - _params.land_speed) / (_params.slow_land_alt1 - _params.slow_land_alt2);
		float b = _params.land_speed - a * _params.slow_land_alt2;
		vel_limit =  a * altitude_above_home + b;
	}

	_vel_sp(2) = math::min(_vel_sp(2), vel_limit);
}

void
MulticopterPositionControl::control_manual(float dt)
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
	float yaw_input_fame = _control_mode.flag_control_fixed_hdg_enabled ? _yaw_takeoff : _att_sp.yaw_body;

	/* setpoint in NED frame */
	man_vel_sp = matrix::Dcmf(matrix::Eulerf(0.0f, 0.0f, yaw_input_fame)) * man_vel_sp;

	/* adjust acceleration based on stick input */
	matrix::Vector2f stick_xy(man_vel_sp(0), man_vel_sp(1));
	set_manual_acceleration_xy(stick_xy, dt);
	float stick_z = man_vel_sp(2);
	float max_acc_z;
	set_manual_acceleration_z(max_acc_z, stick_z, dt);

	/* prepare cruise speed (m/s) vector to scale the velocity setpoint */
	float vel_mag = (_velocity_hor_manual.get() < _vel_max_xy) ? _velocity_hor_manual.get() : _vel_max_xy;
	matrix::Vector3f vel_cruise_scale(vel_mag, vel_mag,
					  (man_vel_sp(2) > 0.0f) ? _params.vel_max_down : _params.vel_max_up);
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

			/* set desired position setpoint assuming max acceleraiton */
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
			math::Vector<2> pos(_pos(0), _pos(1));
			math::Vector<2> vel(_vel(0), _vel(1));
			math::Vector<2> pos_sp = pos + vel * delta_t - vel.normalized() * 0.5f * _acceleration_hor_max.get() * delta_t *delta_t;
			_pos_sp(0) = pos_sp(0);
			_pos_sp(1) = pos_sp(1);

			_pos_hold_engaged = true;
		}
	}

	/* set requested velocity setpoitns */
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

	if (_vehicle_land_detected.landed) {
		/* don't run controller when landed
		 * NOTE:
		 * This only works in manual since once we give throttle input, the
		 * landdetector will exit the landing state */

	} else {
		control_position(dt);
	}
}

void
MulticopterPositionControl::control_non_manual(float dt)
{
	/* select control source */
	if (_control_mode.flag_control_offboard_enabled) {
		/* offboard control */
		control_offboard(dt);
		_mode_auto = false;

	} else {
		_hold_offboard_xy = false;
		_hold_offboard_z = false;

		/* AUTO */
		control_auto(dt);
	}

	/* weather-vane mode for vtol: disable yaw control */
	if (_vehicle_status.is_vtol) {
		_att_sp.disable_mc_yaw_control = _pos_sp_triplet.current.disable_mc_yaw_control;

	} else {
		_att_sp.disable_mc_yaw_control = false;
	}

	// guard against any bad velocity values
	bool velocity_valid = PX4_ISFINITE(_pos_sp_triplet.current.vx) &&
			      PX4_ISFINITE(_pos_sp_triplet.current.vy) &&
			      _pos_sp_triplet.current.velocity_valid;

	// do not go slower than the follow target velocity when position tracking is active (set to valid)
	if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET &&
	    velocity_valid &&
	    _pos_sp_triplet.current.position_valid) {

		math::Vector<3> ft_vel(_pos_sp_triplet.current.vx, _pos_sp_triplet.current.vy, 0);

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
		control_position(dt);
	}
}

void
MulticopterPositionControl::control_offboard(float dt)
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
					PX4_WARN("Unknown velocity offboard coordinate frame");
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
			_att_sp.yaw_body = _att_sp.yaw_body + _pos_sp_triplet.current.yawspeed * dt;
		}

	} else {
		_hold_offboard_xy = false;
		_hold_offboard_z = false;
		reset_pos_sp();
		reset_alt_sp();
	}
}

void
MulticopterPositionControl::vel_sp_slewrate(float dt)
{
	matrix::Vector2f vel_sp_xy(_vel_sp(0), _vel_sp(1));
	matrix::Vector2f vel_sp_prev_xy(_vel_sp_prev(0), _vel_sp_prev(1));
	matrix::Vector2f vel_xy(_vel(0), _vel(1));
	matrix::Vector2f acc_xy = (vel_sp_xy - vel_sp_prev_xy) / dt;

	/* limit total horizontal acceleration */
	if (acc_xy.length() > _acceleration_state_dependent_xy) {
		vel_sp_xy = _acceleration_state_dependent_xy * acc_xy.normalized() * dt + vel_sp_prev_xy;
		_vel_sp(0) = vel_sp_xy(0);
		_vel_sp(1) = vel_sp_xy(1);
	}

	/* limit vertical acceleration */

	float acc_z = (_vel_sp(2) - _vel_sp_prev(2)) / dt;
	float max_acc_z;

	if (_control_mode.flag_control_manual_enabled) {
		max_acc_z = (acc_z < 0.0f) ? -_acceleration_state_dependent_z : _acceleration_state_dependent_z;

	} else {
		max_acc_z = (acc_z < 0.0f) ? -_acceleration_z_max_up.get() : _acceleration_z_max_down.get();
	}

	if (fabsf(acc_z) > fabsf(max_acc_z)) {
		_vel_sp(2) = max_acc_z * dt + _vel_sp_prev(2);
	}
}

bool
MulticopterPositionControl::cross_sphere_line(const math::Vector<3> &sphere_c, const float sphere_r,
		const math::Vector<3> &line_a, const math::Vector<3> &line_b, math::Vector<3> &res)
{
	/* project center of sphere on line */
	/* normalized AB */
	math::Vector<3> ab_norm = line_b - line_a;

	if (ab_norm.length() < 0.01f) {
		return true;
	}

	ab_norm.normalize();
	math::Vector<3> d = line_a + ab_norm * ((sphere_c - line_a) * ab_norm);
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

void MulticopterPositionControl::control_auto(float dt)
{
	/* reset position setpoint on AUTO mode activation or if we are not in MC mode */
	if (!_mode_auto || !_vehicle_status.is_rotary_wing) {
		if (!_mode_auto) {
			_mode_auto = true;
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

	math::Vector<3> prev_sp;
	math::Vector<3> next_sp;

	if (_pos_sp_triplet.current.valid) {

		math::Vector<3> curr_pos_sp = _curr_pos_sp;

		//only project setpoints if they are finite, else use current position
		if (PX4_ISFINITE(_pos_sp_triplet.current.lat) &&
		    PX4_ISFINITE(_pos_sp_triplet.current.lon)) {
			/* project setpoint to local frame */
			map_projection_project(&_ref_pos,
					       _pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon,
					       &_curr_pos_sp.data[0], &_curr_pos_sp.data[1]);

		} else {
			_curr_pos_sp(0) = _pos(0);
			_curr_pos_sp(1) = _pos(1);
		}

		//only project setpoints if they are finite, else use current position
		if (PX4_ISFINITE(_pos_sp_triplet.current.alt)) {
			_curr_pos_sp(2) = -(_pos_sp_triplet.current.alt - _ref_alt);

		} else {
			_curr_pos_sp(2) = _pos(2);
		}


		if (PX4_ISFINITE(_curr_pos_sp(0)) &&
		    PX4_ISFINITE(_curr_pos_sp(1)) &&
		    PX4_ISFINITE(_curr_pos_sp(2))) {
			current_setpoint_valid = true;
		}

		/* check if triplets have been updated
		 * note: we only can look at xy since navigator applies slewrate to z */
		matrix::Vector2f diff((_curr_pos_sp(0) - curr_pos_sp(0)), (_curr_pos_sp(1) - curr_pos_sp(1)));

		if (diff.length() > FLT_EPSILON) {
			triplet_updated = true;
		}

		/* we need to update _curr_pos_sp always since navigator applies slew rate on z */
		_curr_pos_sp = curr_pos_sp;

	}

	if (_pos_sp_triplet.previous.valid) {
		map_projection_project(&_ref_pos,
				       _pos_sp_triplet.previous.lat, _pos_sp_triplet.previous.lon,
				       &prev_sp.data[0], &prev_sp.data[1]);
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
				       &next_sp.data[0], &next_sp.data[1]);
		next_sp(2) = -(_pos_sp_triplet.next.alt - _ref_alt);

		if (PX4_ISFINITE(next_sp(0)) &&
		    PX4_ISFINITE(next_sp(1)) &&
		    PX4_ISFINITE(next_sp(2))) {
			next_setpoint_valid = true;
		}
	}

	if (current_setpoint_valid &&
	    (_pos_sp_triplet.current.type != position_setpoint_s::SETPOINT_TYPE_IDLE)) {

		float cruising_speed_xy = get_cruising_speed_xy();
		float cruising_speed_z = (_curr_pos_sp(2) > _pos(2)) ? _params.vel_max_down : _params.vel_max_up;

		/* scaled space: 1 == position error resulting max allowed speed */
		math::Vector<3> cruising_speed(cruising_speed_xy, cruising_speed_xy, cruising_speed_z);


		/* we follow line depending on setpoint type */
		if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_POSITION  ||
		    _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER ||
		    _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET) {


			/* by default use current setpoint as is */
			math::Vector<3> pos_sp = _curr_pos_sp;
			matrix::Vector2f unit_prev_to_current((_curr_pos_sp(0) - _prev_pos_sp(0)), (_curr_pos_sp(1) - _prev_pos_sp(1)));

			/* follow line */
			if (unit_prev_to_current.length()  > 0.1f) {

				/* unit vector from previous to current */
				unit_prev_to_current = unit_prev_to_current.normalized();

				/* orthogonal distance from current position to unit_prev_to_current */
				matrix::Vector2f closest_point = matrix::Vector2f(_prev_pos_sp(0), _prev_pos_sp(1)) + unit_prev_to_current *
								 (matrix::Vector2f((_pos(0) - _prev_pos_sp(0)), (_pos(1) - _prev_pos_sp(1))) * unit_prev_to_current);

				/* check if we need to adjust position setpoint based on cruise velocity */
				matrix::Vector2f vec_pos_to_current((_curr_pos_sp(0) - _pos(0)), (_curr_pos_sp(1) - _pos(1)));
				matrix::Vector2f vec_prev_to_pos((_pos(0) - _prev_pos_sp(0)), (_pos(1) - _prev_pos_sp(1)));

				float vel_sp_along_track = get_cruising_speed_xy();

				/* accelerate from previous setpoint */
				if ((vec_prev_to_pos.length() < _target_threshold_xy.get())
				    && (((_curr_pos_sp - _prev_pos_sp).length() * 0.5f) > vec_prev_to_pos.length())) {
					/* accelerate */
					float min_cruise_speed = sqrtf(_vel(0) * _vel(0) + _vel(1) * _vel(1));
					min_cruise_speed = (min_cruise_speed > _min_cruise_speed.get()) ?  min_cruise_speed : _min_cruise_speed.get();
					float slope = (get_cruising_speed_xy() - min_cruise_speed)  / _target_threshold_xy.get();
					vel_sp_along_track =  slope * vec_prev_to_pos.length() + min_cruise_speed;
				}

				/* slow down */
				else if (vec_pos_to_current.length() < _target_threshold_xy.get()) {

					/* angle prev-current-next defines speed close to waypoint */
					if (next_setpoint_valid) {

						/* unit vector to from current to next */
						matrix::Vector2f unit_current_to_next((next_sp(0) - pos_sp(0)), (next_sp(1) - pos_sp(1)));
						unit_current_to_next = (unit_current_to_next.length() > 0.0f) ? unit_current_to_next.normalized() :
								       unit_current_to_next;

						/* angle goes from 0 to 2 with 0 being  large angle, 2 being tight angle:   0 = PI ; 2 = PI*0 */
						float angle = unit_current_to_next * (unit_prev_to_current * -1.0f) + 1.0f;

						/* velocity close to target adjusted to angle
						 * vel_close = a *b ^x + c; where at angle = 0 -> vel_close = vel_cruise; angle = 1 -> vel_cruise/4.0 (this means that at 90degrees
						 * the velocity at target should be 1/5 * cruising speed;
						 * angle = 2 -> vel_close = min_cruising_speed */
						float M = get_cruising_speed_xy() / 5.0f;
						float a = -((M -  get_cruising_speed_xy()) * (M -  get_cruising_speed_xy())) / (2.0f * M - get_cruising_speed_xy() -
								_min_cruise_speed.get());
						float c =  get_cruising_speed_xy() - a;
						float b = (M - c) / a;
						float vel_close = a * powf(b, angle) + c;

						float slope = (get_cruising_speed_xy() - vel_close) / _target_threshold_xy.get();
						vel_sp_along_track = slope  * vec_pos_to_current.length() + vel_close;

					} else {
						/* we want to stop at current setpoint */
						float slope = (get_cruising_speed_xy() - _min_cruise_speed.get())  / _target_threshold_xy.get();
						vel_sp_along_track =  slope * vec_pos_to_current.length() + _min_cruise_speed.get();
					}
				}

				/* we want position setpoint not farther away then cruise speed */
				/* we adjust position setpoint */
				matrix::Vector2f vec_pos_to_closest = closest_point - matrix::Vector2f(_pos(0), _pos(1));
				float vel_sp_orthogonal = vec_pos_to_closest.length() * _params.pos_p(0);
				float cruise_sp_mag = sqrtf(vel_sp_orthogonal * vel_sp_orthogonal + vel_sp_along_track * vel_sp_along_track);

				/* check on which section of the track the vehicle is*/
				bool current_behind = ((vec_pos_to_current * -1.0f) * unit_prev_to_current) > 0.0f;

				/* orthogonal velcoity is smaller then cruise speed */
				if (vel_sp_orthogonal < get_cruising_speed_xy() && !current_behind) {
					/* we need to limit vel_sp_along_track such that cruise speed  is never exceeded */
					if (cruise_sp_mag > get_cruising_speed_xy()) {
						vel_sp_along_track = sqrtf(get_cruising_speed_xy() * get_cruising_speed_xy() - vel_sp_orthogonal * vel_sp_orthogonal);
					}

					pos_sp(0) = closest_point(0) + unit_prev_to_current(0) * vel_sp_along_track / _params.pos_p(0);
					pos_sp(1) = closest_point(1) + unit_prev_to_current(1) * vel_sp_along_track / _params.pos_p(0);

				} else {
					/* we are more than cruise_speed away from track */
					/* check on which section we are with default as closest_point */
					bool previous_in_front = (vec_prev_to_pos * unit_prev_to_current) < 0.0f;

					if (previous_in_front) {
						vec_pos_to_closest(0) = _prev_pos_sp(0) - _pos(0);
						vec_pos_to_closest(1) = _prev_pos_sp(1) - _pos(1);
					}

					if (current_behind) {
						/* we already passed current_sp */
						vec_pos_to_closest(0) = _curr_pos_sp(0) - _pos(0);
						vec_pos_to_closest(1) = _curr_pos_sp(1) - _pos(1);
					}

					float cruise_sp = vec_pos_to_current.length() * _params.pos_p(0);

					if (cruise_sp > get_cruising_speed_xy()) {
						cruise_sp = get_cruising_speed_xy();
					}

					pos_sp(0) = _pos(0) + vec_pos_to_closest(0) / vec_pos_to_closest.length() * cruise_sp / _params.pos_p(0);
					pos_sp(1) = _pos(1) + vec_pos_to_closest(1) / vec_pos_to_closest.length() * cruise_sp / _params.pos_p(1);
				}
			}

			_pos_sp = pos_sp;

		} else {
			/* just go to the target point */;
			_pos_sp = _curr_pos_sp;

			/* set max velocity to cruise */
			_vel_max_xy = get_cruising_speed_xy();
		}

		/* update yaw setpoint if needed */
		if (_pos_sp_triplet.current.yawspeed_valid
		    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET) {
			_att_sp.yaw_body = _att_sp.yaw_body + _pos_sp_triplet.current.yawspeed * dt;

		} else if (PX4_ISFINITE(_pos_sp_triplet.current.yaw)) {
			_att_sp.yaw_body = _pos_sp_triplet.current.yaw;
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
		const bool high_enough_for_landing_gear = (_pos(2) < _manual_land_alt.get() * 2.0f);

		// During a mission or in loiter it's safe to retract the landing gear.
		if ((_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_POSITION ||
		     _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER) &&
		    !_vehicle_land_detected.landed &&
		    high_enough_for_landing_gear) {

			_att_sp.landing_gear = 1.0f;

		} else if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF ||
			   _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND ||
			   !high_enough_for_landing_gear) {

			// During takeoff and landing, we better put it down again.
			_att_sp.landing_gear = -1.0f;

		} else {
			// For the rest of the setpoint types, just leave it as is.
		}

	} else {
		/* idle or triplet not valid, do nothing */
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

	_vel_err_d(0) = _vel_x_deriv.update(-_vel(0));
	_vel_err_d(1) = _vel_y_deriv.update(-_vel(1));
	_vel_err_d(2) = _vel_z_deriv.update(-_vel(2));
}

void
MulticopterPositionControl::do_control(float dt)
{
	_vel_ff.zero();

	/* by default, run position/altitude controller. the control_* functions
	 * can disable this and run velocity controllers directly in this cycle */
	_run_pos_control = true;
	_run_alt_control = true;

	if (_control_mode.flag_control_manual_enabled) {
		/* manual control */
		control_manual(dt);
		_mode_auto = false;

		/* we set tiplets to false
		 * this ensures that when switching to auto, the position
		 * controller will not use the old triplets but waits until triplets
		 * have been updated */
		_pos_sp_triplet.current.valid = false;

		_hold_offboard_xy = false;
		_hold_offboard_z = false;

	} else {
		/* reset acceleration to default */
		_acceleration_state_dependent_xy = _acceleration_hor_max.get();
		_acceleration_state_dependent_z = _acceleration_z_max_up.get();
		control_non_manual(dt);
	}
}

void
MulticopterPositionControl::control_position(float dt)
{
	calculate_velocity_setpoint(dt);

	if (_control_mode.flag_control_climb_rate_enabled || _control_mode.flag_control_velocity_enabled ||
	    _control_mode.flag_control_acceleration_enabled) {
		calculate_thrust_setpoint(dt);


	} else {
		_reset_int_z = true;
	}
}

void
MulticopterPositionControl::calculate_velocity_setpoint(float dt)
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
		}
	}

	limit_altitude();

	if (_run_alt_control) {
		_vel_sp(2) = (_pos_sp(2) - _pos(2)) * _params.pos_p(2);
	}

	/* make sure velocity setpoint is saturated in xy*/
	float vel_norm_xy = sqrtf(_vel_sp(0) * _vel_sp(0) +
				  _vel_sp(1) * _vel_sp(1));

	slow_land_gradual_velocity_limit();

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


	/* limit vertical takeoff speed if we are in auto takeoff */
	if (_pos_sp_triplet.current.valid
	    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF
	    && !_control_mode.flag_control_manual_enabled) {
		_vel_sp(2) = math::max(_vel_sp(2), -_params.tko_speed);
	}

	/* apply slewrate (aka acceleration limit) for smooth flying */
	vel_sp_slewrate(dt);
	_vel_sp_prev = _vel_sp;

	/* make sure velocity setpoint is constrained in all directions*/
	if (vel_norm_xy > _vel_max_xy) {
		_vel_sp(0) = _vel_sp(0) * _vel_max_xy / vel_norm_xy;
		_vel_sp(1) = _vel_sp(1) * _vel_max_xy / vel_norm_xy;
	}

	_vel_sp(2) = math::max(_vel_sp(2), -_params.vel_max_up);

	/* special velocity setpoint limitation for smooth takeoff */
	if (_in_takeoff) {
		_in_takeoff = _takeoff_vel_limit < -_vel_sp(2);
		/* ramp vertical velocity limit up to takeoff speed */
		_takeoff_vel_limit += _params.tko_speed * dt / _takeoff_ramp_time.get();
		/* limit vertical velocity to the current ramp value */
		_vel_sp(2) = math::max(_vel_sp(2), -_takeoff_vel_limit);
	}

	/* publish velocity setpoint */
	_global_vel_sp.timestamp = hrt_absolute_time();
	_global_vel_sp.vx = _vel_sp(0);
	_global_vel_sp.vy = _vel_sp(1);
	_global_vel_sp.vz = _vel_sp(2);

	if (_global_vel_sp_pub != nullptr) {
		orb_publish(ORB_ID(vehicle_global_velocity_setpoint), _global_vel_sp_pub, &_global_vel_sp);

	} else {
		_global_vel_sp_pub = orb_advertise(ORB_ID(vehicle_global_velocity_setpoint), &_global_vel_sp);
	}
}

void
MulticopterPositionControl::calculate_thrust_setpoint(float dt)
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

	/* velocity error */
	math::Vector<3> vel_err = _vel_sp - _vel;

	/* thrust vector in NED frame */
	math::Vector<3> thrust_sp;

	if (_control_mode.flag_control_acceleration_enabled && _pos_sp_triplet.current.acceleration_valid) {
		thrust_sp = math::Vector<3>(_pos_sp_triplet.current.a_x, _pos_sp_triplet.current.a_y, _pos_sp_triplet.current.a_z);

	} else {
		thrust_sp = vel_err.emult(_params.vel_p) + _vel_err_d.emult(_params.vel_d)
			    + _thrust_int - math::Vector<3>(0.0f, 0.0f, _params.thr_hover);
	}

	if (!_control_mode.flag_control_velocity_enabled && !_control_mode.flag_control_acceleration_enabled) {
		thrust_sp(0) = 0.0f;
		thrust_sp(1) = 0.0f;
	}

	/* if still or already on ground command zero xy velcoity and zero xy thrust_sp in body frame to consider uneven ground */
	if (_vehicle_land_detected.ground_contact && !in_auto_takeoff()) {

		/* thrust setpoint in body frame*/
		math::Vector<3> thrust_sp_body = _R.transposed() * thrust_sp;

		/* we dont want to make any correction in body x and y*/
		thrust_sp_body(0) = 0.0f;
		thrust_sp_body(1) = 0.0f;

		/* make sure z component of thrust_sp_body is larger than 0 (positive thrust is downward) */
		thrust_sp_body(2) = thrust_sp(2) > 0.0f ? thrust_sp(2) : 0.0f;

		/* convert back to local frame (NED) */
		thrust_sp = _R * thrust_sp_body;

		/* set velocity setpoint to zero and reset position */
		_vel_sp(0) = 0.0f;
		_vel_sp(1) = 0.0f;
		_pos_sp(0) = _pos(0);
		_pos_sp(1) = _pos(1);
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
	/* filter vel_z over 1/8sec */
	_vel_z_lp = _vel_z_lp * (1.0f - dt * 8.0f) + dt * 8.0f * _vel(2);
	/* filter vel_z change over 1/8sec */
	float vel_z_change = (_vel(2) - _vel_prev(2)) / dt;
	_acc_z_lp = _acc_z_lp * (1.0f - dt * 8.0f) + dt * 8.0f * vel_z_change;

	// We can only run the control if we're already in-air, have a takeoff setpoint,
	// or if we're in offboard control.
	// Otherwise, we should just bail out
	if (_vehicle_land_detected.landed && !in_auto_takeoff()) {
		// Keep throttle low while still on ground.
		thr_max = 0.0f;

	} else if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid &&
		   _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {

		/* adjust limits for landing mode */
		/* limit max tilt and min lift when landing */
		tilt_max = _params.tilt_max_land;

		if (thr_min < 0.0f) {
			thr_min = 0.0f;
		}

		/* descend stabilized, we're landing */
		if (!_in_landing && !_lnd_reached_ground
		    && (float)fabsf(_acc_z_lp) < 0.1f
		    && _vel_z_lp > 0.6f * _params.land_speed) {
			_in_landing = true;
		}

		float land_z_threshold = 0.1f;

		/* assume ground, cut thrust */
		if (_in_landing
		    && _vel_z_lp < land_z_threshold) {
			thr_max = 0.0f;
			_in_landing = false;
			_lnd_reached_ground = true;

		} else if (_in_landing
			   && _vel_z_lp < math::min(0.3f * _params.land_speed, 2.5f * land_z_threshold)) {
			/* not on ground but with ground contact, stop position and velocity control */
			thrust_sp(0) = 0.0f;
			thrust_sp(1) = 0.0f;
			_vel_sp(0) = _vel(0);
			_vel_sp(1) = _vel(1);
			_pos_sp(0) = _pos(0);
			_pos_sp(1) = _pos(1);
		}

		/* once we assumed to have reached the ground always cut the thrust.
			Only free fall detection below can revoke this
		*/
		if (!_in_landing && _lnd_reached_ground) {
			thr_max = 0.0f;
		}

		/* if we suddenly fall, reset landing logic and remove thrust limit */
		if (_lnd_reached_ground
		    /* XXX: magic value, assuming free fall above 4m/s2 acceleration */
		    && (_acc_z_lp > 4.0f
			|| _vel_z_lp > 2.0f * _params.land_speed)) {
			thr_max = _params.thr_max;
			_in_landing = true;
			_lnd_reached_ground = false;
		}

	} else {
		_in_landing = false;
		_lnd_reached_ground = false;
	}

	/* limit min lift */
	if (-thrust_sp(2) < thr_min) {
		thrust_sp(2) = -thr_min;
		/* Don't freeze altitude integral if it wants to throttle up */
		saturation_z = vel_err(2) > 0.0f ? true : saturation_z;
	}

	if (_control_mode.flag_control_velocity_enabled || _control_mode.flag_control_acceleration_enabled) {

		/* limit max tilt */
		if (thr_min >= 0.0f && tilt_max < M_PI_F / 2 - 0.05f) {
			/* absolute horizontal thrust */
			float thrust_sp_xy_len = math::Vector<2>(thrust_sp(0), thrust_sp(1)).length();

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
	matrix::Vector3f F(thrust_sp.data);
	float thrust_body_z = F.dot(-R_z); /* recalculate because it might have changed */

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
				float thrust_xy_abs = math::Vector<2>(thrust_sp(0), thrust_sp(1)).length();
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

	_att_sp.thrust = math::max(thrust_body_z, thr_min);

	/* update integrals */
	if (_control_mode.flag_control_velocity_enabled && !saturation_xy) {
		_thrust_int(0) += vel_err(0) * _params.vel_i(0) * dt;
		_thrust_int(1) += vel_err(1) * _params.vel_i(1) * dt;
	}

	if (_control_mode.flag_control_climb_rate_enabled && !saturation_z) {
		_thrust_int(2) += vel_err(2) * _params.vel_i(2) * dt;
	}

	/* calculate attitude setpoint from thrust vector */
	if (_control_mode.flag_control_velocity_enabled || _control_mode.flag_control_acceleration_enabled) {
		/* desired body_z axis = -normalize(thrust_vector) */
		math::Vector<3> body_x;
		math::Vector<3> body_y;
		math::Vector<3> body_z;

		if (thrust_sp.length() > FLT_EPSILON) {
			body_z = -thrust_sp.normalized();

		} else {
			/* no thrust, set Z axis to safe value */
			body_z.zero();
			body_z(2) = 1.0f;
		}

		/* vector of desired yaw direction in XY plane, rotated by PI/2 */
		math::Vector<3> y_C(-sinf(_att_sp.yaw_body), cosf(_att_sp.yaw_body), 0.0f);

		if (fabsf(body_z(2)) > FLT_EPSILON) {
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
MulticopterPositionControl::generate_attitude_setpoint(float dt)
{
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
		float yaw_target = _wrap_pi(_att_sp.yaw_body + _att_sp.yaw_sp_move_rate * dt);
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
		_att_sp.roll_body = _manual.y * _params.man_tilt_max;
		_att_sp.pitch_body = -_manual.x * _params.man_tilt_max;

		/* only if optimal recovery is not used, modify roll/pitch */
		if (_params.opt_recover <= 0) {
			// construct attitude setpoint rotation matrix. modify the setpoints for roll
			// and pitch such that they reflect the user's intention even if a yaw error
			// (yaw_sp - yaw) is present. In the presence of a yaw error constructing a rotation matrix
			// from the pure euler angle setpoints will lead to unexpected attitude behaviour from
			// the user's view as the euler angle sequence uses the  yaw setpoint and not the current
			// heading of the vehicle.

			// calculate our current yaw error
			float yaw_error = _wrap_pi(_att_sp.yaw_body - _yaw);

			// compute the vector obtained by rotating a z unit vector by the rotation
			// given by the roll and pitch commands of the user
			math::Vector<3> zB = {0, 0, 1};
			math::Matrix<3, 3> R_sp_roll_pitch;
			R_sp_roll_pitch.from_euler(_att_sp.roll_body, _att_sp.pitch_body, 0);
			math::Vector<3> z_roll_pitch_sp = R_sp_roll_pitch * zB;


			// transform the vector into a new frame which is rotated around the z axis
			// by the current yaw error. this vector defines the desired tilt when we look
			// into the direction of the desired heading
			math::Matrix<3, 3> R_yaw_correction;
			R_yaw_correction.from_euler(0.0f, 0.0f, -yaw_error);
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
		_att_sp.landing_gear = 1.0f;

	} else if (_manual.gear_switch == manual_control_setpoint_s::SWITCH_POS_OFF) {
		_att_sp.landing_gear = -1.0f;
		// Switching the gear off does put it into a safe defined state
		_gear_state_initialized = true;
	}

	_att_sp.timestamp = hrt_absolute_time();
}

void
MulticopterPositionControl::task_main()
{
	/*
	 * do subscriptions
	 */
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
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
	_att_sp.landing_gear = -1.0f;

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
		float dt = t_prev != 0 ? (t - t_prev) / 1e6f : 0.004f;
		t_prev = t;

		/* set dt for control blocks */
		setDt(dt);

		/* set default max velocity in xy to vel_max */
		_vel_max_xy = _params.vel_max_xy;


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
			_in_takeoff  = false;
			_in_landing = false;
			_lnd_reached_ground = false;

			/* also reset previous setpoints */
			_yaw_takeoff = _yaw;

			_vel_sp_prev.zero();
			_vel_prev.zero();
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

		/* switch to smooth takeoff if we got out of landed state */
		if (!_vehicle_land_detected.landed && was_landed) {
			_in_takeoff = true;
			_takeoff_vel_limit = -0.5f;
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

			do_control(dt);

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
			_limit_vel_xy = false;

			/* store last velocity in case a mode switch to position control occurs */
			_vel_sp_prev = _vel;
		}

		/* generate attitude setpoint from manual controls */
		if (_control_mode.flag_control_manual_enabled && _control_mode.flag_control_attitude_enabled) {

			generate_attitude_setpoint(dt);

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
		 * - if not armed
		 */
		if (_control_mode.flag_armed  && (!(_control_mode.flag_control_offboard_enabled &&
						    !(_control_mode.flag_control_position_enabled ||
						      _control_mode.flag_control_velocity_enabled ||
						      _control_mode.flag_control_acceleration_enabled)))) {

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
					   SCHED_PRIORITY_MAX - 5,
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
