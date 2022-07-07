/****************************************************************************
 *
 *   Copyright (c) 2013-2022 PX4 Development Team. All rights reserved.
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
 * @file fw_pos_control_l1_main.hpp
 * Implementation of a generic position controller based on the L1 norm. Outputs a bank / roll
 * angle, equivalent to a lateral motion (for copters and rovers).
 *
 * The implementation for the controllers is in the ECL library. This class only
 * interfaces to the library.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author Andreas Antener <andreas@uaventure.com>
 */

#ifndef FIXEDWINGPOSITIONCONTROL_HPP_
#define FIXEDWINGPOSITIONCONTROL_HPP_

#include "launchdetection/LaunchDetector.h"
#include "runway_takeoff/RunwayTakeoff.h"

#include <float.h>

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/l1/ECL_L1_Pos_Controller.hpp>
#include <lib/npfg/npfg.hpp>
#include <lib/tecs/TECS.hpp>
#include <lib/landing_slope/Landingslope.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <lib/slew_rate/SlewRate.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/npfg_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_controller_landing_status.h>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/tecs_status.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wind.h>
#include <uORB/topics/orbit_status.h>
#include <uORB/uORB.h>

using namespace launchdetection;
using namespace runwaytakeoff;
using namespace time_literals;

using matrix::Vector2d;
using matrix::Vector2f;

// [m] initial distance of waypoint in front of plane in heading hold mode
static constexpr float HDG_HOLD_DIST_NEXT = 3000.0f;

// [m] distance (plane to waypoint in front) at which waypoints are reset in heading hold mode
static constexpr float HDG_HOLD_REACHED_DIST = 1000.0f;

// [m] distance by which previous waypoint is set behind the plane
static constexpr float HDG_HOLD_SET_BACK_DIST = 100.0f;

// [rad/s] max yawrate at which plane locks yaw for heading hold mode
static constexpr float HDG_HOLD_YAWRATE_THRESH = 0.15f;

// [.] max manual roll/yaw normalized input from user which does not change the locked heading
static constexpr float HDG_HOLD_MAN_INPUT_THRESH = 0.01f;

// [us] time after which we abort landing if terrain estimate is not valid
static constexpr hrt_abstime TERRAIN_ALT_TIMEOUT = 1_s;

// [.] max throttle from user which will not lead to motors spinning up in altitude controlled modes
static constexpr float THROTTLE_THRESH = 0.05f;

// [m/s/s] slew rate limit for airspeed setpoint changes
static constexpr float ASPD_SP_SLEW_RATE = 1.f;

// [us] time after which the wind estimate is disabled if no longer updating
static constexpr hrt_abstime WIND_EST_TIMEOUT = 10_s;

// [s] minimum time step between auto control updates
static constexpr float MIN_AUTO_TIMESTEP = 0.01f;

// [s] maximum time step between auto control updates
static constexpr float MAX_AUTO_TIMESTEP = 0.05f;

// [.] minimum ratio between the actual vehicle weight and the vehicle nominal weight (weight at which the performance limits are derived)
static constexpr float MIN_WEIGHT_RATIO = 0.5f;

// [.] maximum ratio between the actual vehicle weight and the vehicle nominal weight (weight at which the performance limits are derived)
static constexpr float MAX_WEIGHT_RATIO = 2.0f;

// air density of standard athmosphere at 5000m above mean sea level [kg/m^3]
static constexpr float AIR_DENSITY_STANDARD_ATMOS_5000_AMSL = 0.7363f;

class FixedwingPositionControl final : public ModuleBase<FixedwingPositionControl>, public ModuleParams,
	public px4::WorkItem
{
public:
	FixedwingPositionControl(bool vtol = false);
	~FixedwingPositionControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	orb_advert_t	_mavlink_log_pub{nullptr};

	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};
	uORB::Subscription _wind_sub{ORB_ID(wind)};
	uORB::Subscription _control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _pos_sp_triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	uORB::Publication<vehicle_attitude_setpoint_s> _attitude_sp_pub;
	uORB::Publication<vehicle_local_position_setpoint_s> _local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Publication<npfg_status_s> _npfg_status_pub{ORB_ID(npfg_status)};
	uORB::Publication<position_controller_status_s>	_pos_ctrl_status_pub{ORB_ID(position_controller_status)};
	uORB::Publication<position_controller_landing_status_s>	_pos_ctrl_landing_status_pub{ORB_ID(position_controller_landing_status)};
	uORB::Publication<tecs_status_s> _tecs_status_pub{ORB_ID(tecs_status)};
	uORB::PublicationMulti<orbit_status_s> _orbit_status_pub{ORB_ID(orbit_status)};

	manual_control_setpoint_s _manual_control_setpoint{};
	position_setpoint_triplet_s _pos_sp_triplet{};
	vehicle_attitude_setpoint_s _att_sp{};
	vehicle_control_mode_s _control_mode{};
	vehicle_local_position_s _local_pos{};
	vehicle_status_s _vehicle_status{};

	double _current_latitude{0};
	double _current_longitude{0};
	float _current_altitude{0.f};

	perf_counter_t _loop_perf; // loop performance counter

	MapProjection _global_local_proj_ref{};
	float _global_local_alt0{NAN};

	// [m] ground altitude where the plane was launched
	float _takeoff_ground_alt{0.0f};

	// [rad] yaw setpoint for manual position mode heading hold
	float _hdg_hold_yaw{0.0f};

	bool _hdg_hold_enabled{false}; // heading hold enabled
	bool _yaw_lock_engaged{false}; // yaw is locked for heading hold

	float _min_current_sp_distance_xy{FLT_MAX};

	position_setpoint_s _hdg_hold_prev_wp{}; // position where heading hold started
	position_setpoint_s _hdg_hold_curr_wp{}; // position to which heading hold flies

	// [us] Last absolute time position control has been called
	hrt_abstime _last_time_position_control_called{0};

	bool _landed{true};

	/* Landing */
	bool _land_noreturn_horizontal{false};
	bool _land_noreturn_vertical{false};
	bool _land_stayonground{false};
	bool _land_motor_lim{false};
	bool _land_onslope{false};
	bool _land_abort{false};

	Landingslope _landingslope;

	// [us] time at which landing started
	hrt_abstime _time_started_landing{0};

	// [m] last terrain estimate which was valid
	float _last_valid_terrain_alt_estimate{0.0f};

	// [us] time at which we had last valid terrain alt
	hrt_abstime _last_time_terrain_alt_was_valid{0};

	// [m] estimated height to ground at which flare started
	float _flare_height{0.0f};

	// [m] current forced (i.e. not determined using TECS) flare pitch setpoint
	float _flare_pitch_sp{0.0f};

	// [m] estimated height to ground at which flare started
	float _flare_curve_alt_rel_last{0.0f};

	float _target_bearing{0.0f}; // [rad]

	// indicates whether the plane was in the air in the previous interation
	bool _was_in_air{false};

	// [us] time at which the plane went in the air
	hrt_abstime _time_went_in_air{0};

	// Takeoff launch detection and runway
	LaunchDetector _launchDetector;
	LaunchDetectionResult _launch_detection_state{LAUNCHDETECTION_RES_NONE};
	hrt_abstime _launch_detection_notify{0};

	RunwayTakeoff _runway_takeoff;

	// true if the last iteration was in manual mode (used to determine when a reset is needed)
	bool _last_manual{false};

	/* throttle and airspeed states */

	bool _airspeed_valid{false};

	// [us] last time airspeed was received. used to detect timeouts.
	hrt_abstime _time_airspeed_last_valid{0};

	float _airspeed{0.0f};
	float _eas2tas{1.0f};
	float _air_density{CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C};

	/* wind estimates */

	// [m/s] wind velocity vector
	Vector2f _wind_vel{0.0f, 0.0f};

	bool _wind_valid{false};

	hrt_abstime _time_wind_last_received{0}; // [us]

	float _pitch{0.0f};
	float _yaw{0.0f};
	float _yawrate{0.0f};

	matrix::Vector3f _body_acceleration{};
	matrix::Vector3f _body_velocity{};

	bool _reinitialize_tecs{true};
	bool _is_tecs_running{false};

	hrt_abstime _time_last_tecs_update{0}; // [us]

	float _airspeed_after_transition{0.0f};
	bool _was_in_transition{false};

	bool _is_vtol_tailsitter{false};

	matrix::Vector2d _transition_waypoint{(double)NAN, (double)NAN};

	// estimator reset counters

	// captures the number of times the estimator has reset the horizontal position
	uint8_t _pos_reset_counter{0};

	// captures the number of times the estimator has reset the altitude state
	uint8_t _alt_reset_counter{0};

	// [.] normalized setpoint for manual altitude control [-1,1]; -1,0,1 maps to min,zero,max height rate commands
	float _manual_control_setpoint_for_height_rate{0.0f};

	// [.] normalized setpoint for manual airspeed control [0,1]; 0,0.5,1 maps to min,cruise,max airspeed commands
	float _manual_control_setpoint_for_airspeed{0.0f};

	// [m/s] airspeed setpoint for manual modes commanded via MAV_CMD_DO_CHANGE_SPEED
	float _commanded_manual_airspeed_setpoint{NAN};

	hrt_abstime _time_in_fixed_bank_loiter{0}; // [us]

	// L1 guidance - lateral-directional position control
	ECL_L1_Pos_Controller _l1_control;

	// nonlinear path following guidance - lateral-directional position control
	NPFG _npfg;

	// total energy control system - airspeed / altitude control
	TECS _tecs;

	uint8_t _position_sp_type{0};

	enum FW_POSCTRL_MODE {
		FW_POSCTRL_MODE_AUTO,
		FW_POSCTRL_MODE_AUTO_ALTITUDE,
		FW_POSCTRL_MODE_AUTO_CLIMBRATE,
		FW_POSCTRL_MODE_AUTO_TAKEOFF,
		FW_POSCTRL_MODE_AUTO_LANDING,
		FW_POSCTRL_MODE_MANUAL_POSITION,
		FW_POSCTRL_MODE_MANUAL_ALTITUDE,
		FW_POSCTRL_MODE_OTHER
	} _control_mode_current{FW_POSCTRL_MODE_OTHER}; // used to check if the mode has changed

	param_t _param_handle_airspeed_trans{PARAM_INVALID};

	float _param_airspeed_trans{NAN}; // [m/s]

	enum StickConfig {
		STICK_CONFIG_SWAP_STICKS_BIT = (1 << 0),
		STICK_CONFIG_ENABLE_AIRSPEED_SP_MANUAL_BIT = (1 << 1)
	};

	// Update our local parameter cache.
	int parameters_update();

	// Update subscriptions
	void airspeed_poll();
	void control_update();
	void manual_control_setpoint_poll();
	void vehicle_attitude_poll();
	void vehicle_command_poll();
	void vehicle_control_mode_poll();
	void vehicle_status_poll();
	void wind_poll();

	void status_publish();
	void landing_status_publish();
	void tecs_status_publish();
	void publishLocalPositionSetpoint(const position_setpoint_s &current_waypoint);

	void abort_landing(bool abort);

	/**
	 * @brief Get a new waypoint based on heading and distance from current position
	 *
	 * @param heading the heading to fly to
	 * @param distance the distance of the generated waypoint
	 * @param waypoint_prev the waypoint at the current position
	 * @param waypoint_next the waypoint in the heading direction
	 */
	void get_waypoint_heading_distance(float heading, position_setpoint_s &waypoint_prev,
					   position_setpoint_s &waypoint_next, bool flag_init);

	/**
	 * @brief Return the terrain estimate during takeoff or takeoff_alt if terrain estimate is not available
	 *
	 * @param takeoff_alt Altitude AMSL at launch or when runway takeoff is detected [m]
	 */
	float get_terrain_altitude_takeoff(float takeoff_alt);

	/**
	 * @brief Maps the manual control setpoint (pilot sticks) to height rate commands
	 *
	 * @return Manual height rate setpoint [m/s]
	 */
	float getManualHeightRateSetpoint();

	/**
	 * @brief Check if we are in a takeoff situation
	 */
	bool in_takeoff_situation();

	/**
	 * @brief Update desired altitude base on user pitch stick input
	 *
	 * @param dt Time step
	 */
	void update_desired_altitude(float dt);

	/**
	 * @brief Updates timing information for landed and in-air states.
	 *
	 * @param now Current system time [us]
	 */
	void update_in_air_states(const hrt_abstime now);

	/**
	 * @brief Moves the current position setpoint to a value far ahead of the current vehicle yaw when in  a VTOL
	 * transition.
	 *
	 * @param[in,out] current_sp Current position setpoint
	 */
	void move_position_setpoint_for_vtol_transition(position_setpoint_s &current_sp);

	/**
	 * @brief Changes the position setpoint type to achieve the desired behavior in some instances.
	 *
	 * @param pos_sp_curr Current position setpoint
	 * @return Adjusted position setpoint type
	 */
	uint8_t	handle_setpoint_type(const position_setpoint_s &pos_sp_curr);

	/* automatic control methods */

	/**
	 * @brief Automatic position control for waypoints, orbits, and velocity control
	 *
	 * @param control_interval Time since last position control call [s]
	 * @param curr_pos Current 2D local position vector of vehicle [m]
	 * @param ground_speed Local 2D ground speed of vehicle [m/s]
	 * @param pos_sp_prev previous position setpoint
	 * @param pos_sp_curr current position setpoint
	 * @param pos_sp_next next position setpoint
	 */
	void control_auto(const float control_interval, const Vector2d &curr_pos, const Vector2f &ground_speed,
			  const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr, const position_setpoint_s &pos_sp_next);

	/**
	 * @brief Controls altitude and airspeed for a fixed-bank loiter.
	 *
	 * Used as a failsafe mode after a lateral position estimate failure.
	 *
	 * @param control_interval Time since last position control call [s]
	 */
	void control_auto_fixed_bank_alt_hold(const float control_interval);

	/**
	 * @brief Control airspeed with a fixed descent rate and roll angle.
	 *
	 * Used as a failsafe mode after a lateral position estimate failure.
	 *
	 * @param control_interval Time since last position control call [s]
	 */
	void control_auto_descend(const float control_interval);

	/**
	 * @brief Vehicle control for position waypoints.
	 *
	 * @param control_interval Time since last position control call [s]
	 * @param curr_pos Current 2D local position vector of vehicle [m]
	 * @param ground_speed Local 2D ground speed of vehicle [m/s]
	 * @param pos_sp_prev previous position setpoint
	 * @param pos_sp_curr current position setpoint
	 */
	void control_auto_position(const float control_interval, const Vector2d &curr_pos, const Vector2f &ground_speed,
				   const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr);

	/**
	 * @brief Vehicle control for loiter waypoints.
	 *
	 * @param control_interval Time since last position control call [s]
	 * @param curr_pos Current 2D local position vector of vehicle [m]
	 * @param ground_speed Local 2D ground speed of vehicle [m/s]
	 * @param pos_sp_prev previous position setpoint
	 * @param pos_sp_curr current position setpoint
	 * @param pos_sp_next next position setpoint
	 */
	void control_auto_loiter(const float control_interval, const Vector2d &curr_pos, const Vector2f &ground_speed,
				 const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr, const position_setpoint_s &pos_sp_next);

	/**
	 * @brief Controls a desired airspeed, bearing, and height rate.
	 *
	 * @param control_interval Time since last position control call [s]
	 * @param curr_pos Current 2D local position vector of vehicle [m]
	 * @param ground_speed Local 2D ground speed of vehicle [m/s]
	 * @param pos_sp_curr current position setpoint
	 */
	void control_auto_velocity(const float control_interval, const Vector2d &curr_pos, const Vector2f &ground_speed,
				   const position_setpoint_s &pos_sp_curr);

	/**
	 * @brief Controls automatic takeoff.
	 *
	 * @param now Current system time [us]
	 * @param control_interval Time since last position control call [s]
	 * @param curr_pos Current 2D local position vector of vehicle [m]
	 * @param ground_speed Local 2D ground speed of vehicle [m/s]
	 * @param pos_sp_prev previous position setpoint
	 * @param pos_sp_curr current position setpoint
	 */
	void control_auto_takeoff(const hrt_abstime &now, const float control_interval, const Vector2d &curr_pos,
				  const Vector2f &ground_speed, const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr);

	/**
	 * @brief Controls automatic landing.
	 *
	 * @param now Current system time [us]
	 * @param control_interval Time since last position control call [s]
	 * @param curr_pos Current 2D local position vector of vehicle [m]
	 * @param control_interval Time since the last position control update [s]
	 * @param ground_speed Local 2D ground speed of vehicle [m/s]
	 * @param pos_sp_prev previous position setpoint
	 * @param pos_sp_curr current position setpoint
	 */
	void control_auto_landing(const hrt_abstime &now, const float control_interval, const Vector2d &curr_pos,
				  const Vector2f &ground_speed, const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr);

	/* manual control methods */

	/**
	 * @brief Controls altitude and airspeed, user commands roll setpoint.
	 *
	 * @param control_interval Time since last position control call [s]
	 * @param curr_pos Current 2D local position vector of vehicle [m]
	 * @param ground_speed Local 2D ground speed of vehicle [m/s]
	 */
	void control_manual_altitude(const float control_interval, const Vector2d &curr_pos, const Vector2f &ground_speed);

	/**
	 * @brief Controls user commanded altitude, airspeed, and bearing.
	 *
	 * @param control_interval Time since last position control call [s]
	 * @param curr_pos Current 2D local position vector of vehicle [m]
	 * @param ground_speed Local 2D ground speed of vehicle [m/s]
	 */
	void control_manual_position(const float control_interval, const Vector2d &curr_pos, const Vector2f &ground_speed);

	float get_tecs_pitch();
	float get_tecs_thrust();

	float get_manual_airspeed_setpoint();

	/**
	 * @brief Returns an calibrated airspeed setpoint for auto modes.
	 *
	 * Adjusts the setpoint for wind, accelerated stall, and slew rates.
	 *
	 * @param control_interval Time since the last position control update [s]
	 * @param pos_sp_cru_airspeed The commanded cruise airspeed from the position setpoint [s]
	 * @param ground_speed Vehicle ground velocity vector [m/s]
	 * @return Calibrated airspeed setpoint [m/s]
	 */
	float get_auto_airspeed_setpoint(const float control_interval, const float pos_sp_cru_airspeed,
					 const Vector2f &ground_speed);

	void reset_takeoff_state(bool force = false);
	void reset_landing_state();

	bool using_npfg_with_wind_estimate() const;

	/**
	 * @brief Returns the velocity vector to be input in the lateral-directional guidance laws.
	 *
	 * Replaces the ground velocity vector with an air velocity vector depending on the wind condition and whether
	 * NPFG or L1 are being used for horizontal position control.
	 *
	 * @param ground_speed Vehicle ground velocity vector [m/s]
	 * @return Velocity vector to control with lateral-directional guidance [m/s]
	 */
	Vector2f get_nav_speed_2d(const Vector2f &ground_speed);

	/**
	 * @brief Decides which control mode to execute.
	 *
	 * May also change the position setpoint type depending on the desired behavior.
	 *
	 * @param now Current system time [us]
	 * @param pos_sp_curr_valid True if the current position setpoint is valid
	 */
	void set_control_mode_current(const hrt_abstime &now, bool pos_sp_curr_valid);

	/**
	 * @brief Compensate trim throttle for air density and vehicle weight.
	 *
	 * @param trim throttle required at sea level during standard conditions.
	 * @param throttle_min Minimum allowed trim throttle.
	 * @param throttle_max Maximum allowed trim throttle.
	 * @return Trim throttle compensated for air density and vehicle weight.
	 */
	float compensateTrimThrottleForDensityAndWeight(float throttle_trim, float throttle_min, float throttle_max);

	void publishOrbitStatus(const position_setpoint_s pos_sp);

	SlewRate<float> _airspeed_slew_rate_controller;

	/**
	 * @brief A wrapper function to call the TECS implementation
	 *
	 * @param control_interval Time since the last position control update [s]
	 * @param alt_sp Altitude setpoint, AMSL [m]
	 * @param airspeed_sp Calibrated airspeed setpoint [m/s]
	 * @param pitch_min_rad Nominal pitch angle command minimum [rad]
	 * @param pitch_max_rad Nominal pitch angle command maximum [rad]
	 * @param throttle_min Minimum throttle command [0,1]
	 * @param throttle_max Maximum throttle command [0,1]
	 * @param climbout_mode True if TECS should engage climbout mode
	 * @param climbout_pitch_min_rad Minimum pitch angle command in climbout mode [rad]
	 * @param disable_underspeed_detection True if underspeed detection should be disabled
	 * @param hgt_rate_sp Height rate setpoint [m/s]
	 */
	void tecs_update_pitch_throttle(const float control_interval, float alt_sp, float airspeed_sp,
					float pitch_min_rad, float pitch_max_rad,
					float throttle_min, float throttle_max,
					bool climbout_mode, float climbout_pitch_min_rad,
					bool disable_underspeed_detection = false, float hgt_rate_sp = NAN);

	DEFINE_PARAMETERS(

		(ParamFloat<px4::params::FW_AIRSPD_MAX>) _param_fw_airspd_max,
		(ParamFloat<px4::params::FW_AIRSPD_MIN>) _param_fw_airspd_min,
		(ParamFloat<px4::params::FW_AIRSPD_TRIM>) _param_fw_airspd_trim,
		(ParamFloat<px4::params::FW_AIRSPD_STALL>) _param_fw_airspd_stall,

		(ParamFloat<px4::params::FW_CLMBOUT_DIFF>) _param_fw_clmbout_diff,

		(ParamFloat<px4::params::FW_GND_SPD_MIN>) _param_fw_gnd_spd_min,

		(ParamFloat<px4::params::FW_L1_DAMPING>) _param_fw_l1_damping,
		(ParamFloat<px4::params::FW_L1_PERIOD>) _param_fw_l1_period,
		(ParamFloat<px4::params::FW_L1_R_SLEW_MAX>) _param_fw_l1_r_slew_max,
		(ParamFloat<px4::params::FW_R_LIM>) _param_fw_r_lim,

		(ParamBool<px4::params::FW_USE_NPFG>) _param_fw_use_npfg,
		(ParamFloat<px4::params::NPFG_PERIOD>) _param_npfg_period,
		(ParamFloat<px4::params::NPFG_DAMPING>) _param_npfg_damping,
		(ParamBool<px4::params::NPFG_LB_PERIOD>) _param_npfg_en_period_lb,
		(ParamBool<px4::params::NPFG_UB_PERIOD>) _param_npfg_en_period_ub,
		(ParamBool<px4::params::NPFG_TRACK_KEEP>) _param_npfg_en_track_keeping,
		(ParamBool<px4::params::NPFG_EN_MIN_GSP>) _param_npfg_en_min_gsp,
		(ParamBool<px4::params::NPFG_WIND_REG>) _param_npfg_en_wind_reg,
		(ParamFloat<px4::params::NPFG_GSP_MAX_TK>) _param_npfg_track_keeping_gsp_max,
		(ParamFloat<px4::params::NPFG_ROLL_TC>) _param_npfg_roll_time_const,
		(ParamFloat<px4::params::NPFG_SW_DST_MLT>) _param_npfg_switch_distance_multiplier,
		(ParamFloat<px4::params::NPFG_PERIOD_SF>) _param_npfg_period_safety_factor,

		(ParamFloat<px4::params::FW_LND_AIRSPD_SC>) _param_fw_lnd_airspd_sc,
		(ParamFloat<px4::params::FW_LND_ANG>) _param_fw_lnd_ang,
		(ParamFloat<px4::params::FW_LND_FL_PMAX>) _param_fw_lnd_fl_pmax,
		(ParamFloat<px4::params::FW_LND_FL_PMIN>) _param_fw_lnd_fl_pmin,
		(ParamFloat<px4::params::FW_LND_FLALT>) _param_fw_lnd_flalt,
		(ParamFloat<px4::params::FW_LND_HHDIST>) _param_fw_lnd_hhdist,
		(ParamFloat<px4::params::FW_LND_HVIRT>) _param_fw_lnd_hvirt,
		(ParamFloat<px4::params::FW_LND_THRTC_SC>) _param_fw_thrtc_sc,
		(ParamFloat<px4::params::FW_LND_TLALT>) _param_fw_lnd_tlalt,
		(ParamBool<px4::params::FW_LND_EARLYCFG>) _param_fw_lnd_earlycfg,
		(ParamBool<px4::params::FW_LND_USETER>) _param_fw_lnd_useter,

		(ParamFloat<px4::params::FW_P_LIM_MAX>) _param_fw_p_lim_max,
		(ParamFloat<px4::params::FW_P_LIM_MIN>) _param_fw_p_lim_min,

		(ParamFloat<px4::params::FW_T_CLMB_MAX>) _param_fw_t_clmb_max,
		(ParamFloat<px4::params::FW_T_HRATE_FF>) _param_fw_t_hrate_ff,
		(ParamFloat<px4::params::FW_T_ALT_TC>) _param_fw_t_h_error_tc,
		(ParamFloat<px4::params::FW_T_I_GAIN_THR>) _param_fw_t_I_gain_thr,
		(ParamFloat<px4::params::FW_T_I_GAIN_PIT>) _param_fw_t_I_gain_pit,
		(ParamFloat<px4::params::FW_T_PTCH_DAMP>) _param_fw_t_ptch_damp,
		(ParamFloat<px4::params::FW_T_RLL2THR>) _param_fw_t_rll2thr,
		(ParamFloat<px4::params::FW_T_SINK_MAX>) _param_fw_t_sink_max,
		(ParamFloat<px4::params::FW_T_SINK_MIN>) _param_fw_t_sink_min,
		(ParamFloat<px4::params::FW_T_SPD_OMEGA>) _param_fw_t_spd_omega,
		(ParamFloat<px4::params::FW_T_SPDWEIGHT>) _param_fw_t_spdweight,
		(ParamFloat<px4::params::FW_T_TAS_TC>) _param_fw_t_tas_error_tc,
		(ParamFloat<px4::params::FW_T_THR_DAMP>) _param_fw_t_thr_damp,
		(ParamFloat<px4::params::FW_T_VERT_ACC>) _param_fw_t_vert_acc,
		(ParamFloat<px4::params::FW_T_STE_R_TC>) _param_ste_rate_time_const,
		(ParamFloat<px4::params::FW_T_TAS_R_TC>) _param_tas_rate_time_const,
		(ParamFloat<px4::params::FW_T_SEB_R_FF>) _param_seb_rate_ff,
		(ParamFloat<px4::params::FW_T_CLMB_R_SP>) _param_climbrate_target,
		(ParamFloat<px4::params::FW_T_SINK_R_SP>) _param_sinkrate_target,

		(ParamFloat<px4::params::FW_THR_TRIM>) _param_fw_thr_trim,
		(ParamFloat<px4::params::FW_THR_IDLE>) _param_fw_thr_idle,
		(ParamFloat<px4::params::FW_THR_LND_MAX>) _param_fw_thr_lnd_max,
		(ParamFloat<px4::params::FW_THR_MAX>) _param_fw_thr_max,
		(ParamFloat<px4::params::FW_THR_MIN>) _param_fw_thr_min,
		(ParamFloat<px4::params::FW_THR_SLEW_MAX>) _param_fw_thr_slew_max,

		(ParamInt<px4::params::FW_POS_STK_CONF>) _param_fw_pos_stk_conf,

		(ParamInt<px4::params::FW_GPSF_LT>) _param_nav_gpsf_lt,
		(ParamFloat<px4::params::FW_GPSF_R>) _param_nav_gpsf_r,

		// external parameters
		(ParamInt<px4::params::FW_ARSP_MODE>) _param_fw_arsp_mode,

		(ParamFloat<px4::params::FW_PSP_OFF>) _param_fw_psp_off,
		(ParamFloat<px4::params::FW_MAN_P_MAX>) _param_fw_man_p_max,
		(ParamFloat<px4::params::FW_MAN_R_MAX>) _param_fw_man_r_max,

		(ParamFloat<px4::params::NAV_LOITER_RAD>) _param_nav_loiter_rad,

		(ParamFloat<px4::params::FW_TKO_PITCH_MIN>) _takeoff_pitch_min,

		(ParamFloat<px4::params::NAV_FW_ALT_RAD>) _param_nav_fw_alt_rad,

		(ParamFloat<px4::params::WEIGHT_BASE>) _param_weight_base,
		(ParamFloat<px4::params::WEIGHT_GROSS>) _param_weight_gross

	)

};

#endif // FIXEDWINGPOSITIONCONTROL_HPP_
