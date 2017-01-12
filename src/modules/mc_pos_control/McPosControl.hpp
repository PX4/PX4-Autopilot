#pragma once


#include <mathlib/mathlib.h>
#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#include <lib/geo/geo.h>
#include <poll.h>
#include <drivers/drv_hrt.h>

#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/mc_virtual_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_land_detected.h>


class McPosControl : public control::SuperBlock
{
public:
	/**
	 * Constructor
	 */
	McPosControl();

	/**
	 * Destructor
	 */
	~McPosControl();

	/**
	 * Runs the position controller
	 */
	void		run();


	bool		cross_sphere_line(const math::Vector<3> &sphere_c, float sphere_r,
					  const math::Vector<3> line_a, const math::Vector<3> line_b, math::Vector<3> &res);

private:

	int		_vehicle_status_sub;		/**< vehicle status subscription */
	int		_vehicle_land_detected_sub;	/**< vehicle land detected subscription */
	int		_ctrl_state_sub;		/**< control state subscription */
	int		_att_sp_sub;			/**< vehicle attitude setpoint */
	int		_control_mode_sub;		/**< vehicle control mode subscription */
	int		_params_sub;			/**< notification of parameter updates */
	int		_manual_sub;			/**< notification of manual control updates */
	int		_arming_sub;			/**< arming status of outputs */
	int		_local_pos_sub;			/**< vehicle local position */
	int		_pos_sp_triplet_sub;		/**< position setpoint triplet */
	int		_local_pos_sp_sub;		/**< offboard local position setpoint */
	int		_global_vel_sp_sub;		/**< offboard global velocity setpoint */

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
	struct actuator_armed_s				_arming;		/**< actuator arming status */
	struct vehicle_local_position_s			_local_pos;		/**< vehicle local position */
	struct position_setpoint_triplet_s		_pos_sp_triplet;	/**< vehicle global position setpoint triplet */
	struct vehicle_local_position_setpoint_s	_local_pos_sp;		/**< vehicle local position setpoint */
	struct vehicle_global_velocity_setpoint_s	_global_vel_sp;		/**< vehicle global velocity setpoint */


	control::BlockParamFloat _manual_thr_min;
	control::BlockParamFloat _manual_thr_max;

	control::BlockDerivative _vel_x_deriv;
	control::BlockDerivative _vel_y_deriv;
	control::BlockDerivative _vel_z_deriv;

	struct {
		param_t thr_min;
		param_t thr_max;
		param_t thr_hover;
		param_t alt_ctl_dz;
		param_t alt_ctl_dy;
		param_t z_p;
		param_t z_vel_p;
		param_t z_vel_i;
		param_t z_vel_d;
		param_t z_vel_max_up;
		param_t z_vel_max_down;
		param_t z_ff;
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
		param_t man_roll_max;
		param_t man_pitch_max;
		param_t man_yaw_max;
		param_t global_yaw_max;
		param_t mc_att_yaw_p;
		param_t hold_xy_dz;
		param_t hold_max_xy;
		param_t hold_max_z;
		param_t acc_hor_max;
		param_t alt_mode;
		param_t opt_recover;

	}		_params_handles;		/**< handles for interesting parameters */

	struct {
		float thr_min;
		float thr_max;
		float thr_hover;
		float alt_ctl_dz;
		float alt_ctl_dy;
		float tilt_max_air;
		float land_speed;
		float tko_speed;
		float tilt_max_land;
		float man_roll_max;
		float man_pitch_max;
		float man_yaw_max;
		float global_yaw_max;
		float mc_att_yaw_p;
		float hold_xy_dz;
		float hold_max_xy;
		float hold_max_z;
		float acc_hor_max;
		float vel_max_up;
		float vel_max_down;
		uint32_t alt_mode;

		int opt_recover;

		math::Vector<3> pos_p;
		math::Vector<3> vel_p;
		math::Vector<3> vel_i;
		math::Vector<3> vel_d;
		math::Vector<3> vel_ff;
		math::Vector<3> vel_max;
		math::Vector<3> vel_cruise;
		math::Vector<3> sp_offs_max;
	}		_params;

	struct map_projection_reference_s _ref_pos;
	float _ref_alt;
	hrt_abstime _ref_timestamp;

	bool _reset_pos_sp;
	bool _reset_alt_sp;
	bool _do_reset_alt_pos_flag;
	bool _mode_auto;
	bool _pos_hold_engaged;
	bool _alt_hold_engaged;
	bool _run_pos_control;
	bool _run_alt_control;

	bool _reset_int_z = true;
	bool _reset_int_xy = true;
	bool _reset_int_z_manual = false;
	bool _reset_yaw_sp = true;

	bool _hold_offboard_xy = false;
	bool _hold_offboard_z = false;

	math::Vector<3> _thrust_int;

	math::Vector<3> _pos;
	math::Vector<3> _pos_sp;
	math::Vector<3> _vel;
	math::Vector<3> _vel_sp;
	math::Vector<3> _vel_prev;			/**< velocity on previous step */
	math::Vector<3> _vel_ff;
	math::Vector<3> _vel_sp_prev;
	math::Vector<3> _vel_err_d;		/**< derivative of current velocity */

	math::Matrix<3, 3> _R;			/**< rotation matrix from attitude quaternions */
	float _yaw;				/**< yaw angle (euler) */
	bool _in_landing;	/**< the vehicle is in the landing descent */
	bool _lnd_reached_ground; /**< controller assumes the vehicle has reached the ground after landing */
	bool _takeoff_jumped;
	float _vel_z_lp;
	float _acc_z_lp;
	float _takeoff_thrust_sp;
	bool control_vel_enabled_prev;	/**< previous loop was in velocity controlled mode (control_state.flag_control_velocity_enabled) */

	// counters for reset events on position and velocity states
	// they are used to identify a reset event
	uint8_t _z_reset_counter;
	uint8_t _xy_reset_counter;
	uint8_t _vz_reset_counter;
	uint8_t _vxy_reset_counter;
	uint8_t _heading_reset_counter;
	hrt_abstime _t_prev;
	bool _was_armed;

	matrix::Dcmf _R_setpoint;

	px4_pollfd_struct_t _fds[1];

	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update(bool force);

	/**
	 * Update control outputs
	 */
	void		control_update();

	/**
	 * Check for changes in subscribed topics.
	 */
	void		poll_subscriptions();

	static float	scale_control(float ctl, float end, float dz, float dy);
	static float    throttle_curve(float ctl, float ctr);

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
	 * Check if position setpoint is too far from current position and adjust it if needed.
	 */
	void		limit_pos_sp_offset();

	/**
	 * Set position setpoint using manual control
	 */
	void		control_manual(float dt);

	void control_non_manual(float dt);

	/**
	 * Set position setpoint using offboard control
	 */
	void		control_offboard(float dt);

	/**
	 * Set position setpoint for AUTO
	 */
	void		control_auto(float dt);

	void control_position(float dt);

	void limit_acceleration(float dt);

	/**
	 * Select between barometric and global (AMSL) altitudes
	 */
	void		select_alt(bool global);

	void update_velocity_derivative();

	void do_control(float dt);

	void generate_attitude_setpoint(float dt);

};
