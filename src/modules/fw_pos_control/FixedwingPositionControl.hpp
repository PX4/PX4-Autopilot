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
 * @file fw_pos_control_main.hpp
 * Implementation of various fixed-wing position level navigation/control modes.
 *
 * The implementation for the controllers is in a separate library. This class only
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
#include <lib/npfg/npfg.hpp>
#include <lib/tecs/TECS.hpp>
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
#include <uORB/topics/landing_gear.h>
#include <uORB/topics/launch_detection_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/normalized_unsigned_setpoint.h>
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

// [us] time after which we abort landing if terrain estimate is not valid. this timer start whenever the terrain altitude
// was previously valid, and has changed to invalid.
static constexpr hrt_abstime TERRAIN_ALT_TIMEOUT = 1_s;

// [us] within this timeout, if a distance sensor measurement not yet made, the land waypoint altitude is used for terrain
// altitude. this timer starts at the beginning of the landing glide slope.
static constexpr hrt_abstime TERRAIN_ALT_FIRST_MEASUREMENT_TIMEOUT = 10_s;

// [.] max throttle from user which will not lead to motors spinning up in altitude controlled modes
static constexpr float THROTTLE_THRESH = -.9f;

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

// [rad] minimum pitch while airspeed has not yet reached a controllable value in manual position controlled takeoff modes
static constexpr float MIN_PITCH_DURING_MANUAL_TAKEOFF = 0.0f;

// [m] arbitrary buffer altitude added to clearance altitude setpoint during takeoff to ensure aircraft passes the clearance
// altitude while waiting for navigator to flag it exceeded
static constexpr float kClearanceAltitudeBuffer = 10.0f;

// [m/s] maximum rate at which the touchdown position can be nudged
static constexpr float MAX_TOUCHDOWN_POSITION_NUDGE_RATE = 4.0f;

// [.] normalized deadzone threshold for manual nudging input
static constexpr float MANUAL_TOUCHDOWN_NUDGE_INPUT_DEADZONE = 0.15f;

// [s] time interval after touchdown for ramping in runway clamping constraints (touchdown is assumed at FW_LND_TD_TIME after start of flare)
static constexpr float POST_TOUCHDOWN_CLAMP_TIME = 0.5f;

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
	uORB::Publication<launch_detection_status_s> _launch_detection_status_pub{ORB_ID(launch_detection_status)};
	uORB::PublicationMulti<orbit_status_s> _orbit_status_pub{ORB_ID(orbit_status)};
	uORB::Publication<landing_gear_s> _landing_gear_pub{ORB_ID(landing_gear)};
	uORB::Publication<normalized_unsigned_setpoint_s> _flaps_setpoint_pub{ORB_ID(flaps_setpoint)};
	uORB::Publication<normalized_unsigned_setpoint_s> _spoilers_setpoint_pub{ORB_ID(spoilers_setpoint)};

	manual_control_setpoint_s _manual_control_setpoint{};
	position_setpoint_triplet_s _pos_sp_triplet{};
	vehicle_attitude_setpoint_s _att_sp{};
	vehicle_control_mode_s _control_mode{};
	vehicle_local_position_s _local_pos{};
	vehicle_status_s _vehicle_status{};

	bool _position_setpoint_previous_valid{false};
	bool _position_setpoint_current_valid{false};
	bool _position_setpoint_next_valid{false};

	perf_counter_t _loop_perf; // loop performance counter

	// [us] Last absolute time position control has been called
	hrt_abstime _last_time_position_control_called{0};

	uint8_t _position_sp_type{0};

	enum FW_POSCTRL_MODE {
		FW_POSCTRL_MODE_AUTO,
		FW_POSCTRL_MODE_AUTO_ALTITUDE,
		FW_POSCTRL_MODE_AUTO_CLIMBRATE,
		FW_POSCTRL_MODE_AUTO_TAKEOFF,
		FW_POSCTRL_MODE_AUTO_LANDING_STRAIGHT,
		FW_POSCTRL_MODE_AUTO_LANDING_CIRCULAR,
		FW_POSCTRL_MODE_MANUAL_POSITION,
		FW_POSCTRL_MODE_MANUAL_ALTITUDE,
		FW_POSCTRL_MODE_OTHER
	} _control_mode_current{FW_POSCTRL_MODE_OTHER}; // used to check if the mode has changed

	enum StickConfig {
		STICK_CONFIG_SWAP_STICKS_BIT = (1 << 0),
		STICK_CONFIG_ENABLE_AIRSPEED_SP_MANUAL_BIT = (1 << 1)
	};

	// VEHICLE STATES

	double _current_latitude{0};
	double _current_longitude{0};
	float _current_altitude{0.f};

	float _pitch{0.0f};
	float _yaw{0.0f};
	float _yawrate{0.0f};

	float _body_acceleration_x{0.f};
	float _body_velocity_x{0.f};

	MapProjection _global_local_proj_ref{};
	float _global_local_alt0{NAN};

	bool _landed{true};

	// indicates whether the plane was in the air in the previous interation
	bool _was_in_air{false};

	// [us] time at which the plane went in the air
	hrt_abstime _time_went_in_air{0};

	// MANUAL MODES

	// indicates whether we have completed a manual takeoff in a position control mode
	bool _completed_manual_takeoff{false};

	// [rad] yaw setpoint for manual position mode heading hold
	float _hdg_hold_yaw{0.0f};

	bool _hdg_hold_enabled{false}; // heading hold enabled
	bool _yaw_lock_engaged{false}; // yaw is locked for heading hold

	position_setpoint_s _hdg_hold_prev_wp{}; // position where heading hold started
	position_setpoint_s _hdg_hold_curr_wp{}; // position to which heading hold flies

	// [.] normalized setpoint for manual altitude control [-1,1]; -1,0,1 maps to min,zero,max height rate commands
	float _manual_control_setpoint_for_height_rate{0.0f};

	// [.] normalized setpoint for manual airspeed control [-1,1]; -1,0,1 maps to min,cruise,max airspeed commands
	float _manual_control_setpoint_for_airspeed{0.0f};

	// [m/s] airspeed setpoint for manual modes commanded via MAV_CMD_DO_CHANGE_SPEED
	float _commanded_manual_airspeed_setpoint{NAN};

	// AUTO TAKEOFF

	// [m] ground altitude AMSL where the plane was launched
	float _takeoff_ground_alt{0.0f};

	// class handling launch detection methods for fixed-wing takeoff
	LaunchDetector _launchDetector;

	// true if a launch, specifically using the launch detector, has been detected
	bool _launch_detected{false};

	// [deg] global position of the vehicle at the time launch is detected (using launch detector)
	Vector2d _launch_global_position{0, 0};

	// [rad] current vehicle yaw at the time the launch is detected
	float _launch_current_yaw{0.f};

	// class handling runway takeoff for fixed-wing UAVs with steerable wheels
	RunwayTakeoff _runway_takeoff;

	bool _skipping_takeoff_detection{false};

	// AUTO LANDING

	// corresponds to param FW_LND_NUDGE
	enum LandingNudgingOption {
		kNudgingDisabled = 0,
		kNudgeApproachAngle,
		kNudgeApproachPath
	};

	// [us] Start time of the landing approach. If a fixed-wing landing pattern is used, this timer starts *after any
	// orbit to altitude only when the aircraft has entered the final *straight approach.
	hrt_abstime _time_started_landing{0};

	// [m] lateral touchdown position offset manually commanded during landing
	float _lateral_touchdown_position_offset{0.0f};

	// [m] relative vector from land point to approach entrance (NE)
	Vector2f _landing_approach_entrance_offset_vector{};

	// [m] relative height above land point
	float _landing_approach_entrance_rel_alt{0.0f};

	uint8_t _landing_abort_status{position_controller_landing_status_s::NOT_ABORTED};

	// organize flare states XXX: need to split into a separate class at some point!
	struct FlareStates {
		bool flaring{false};
		hrt_abstime start_time{0}; // [us]
		float initial_height_rate_setpoint{0.0f}; // [m/s]
		float initial_throttle_setpoint{0.0f};
	} _flare_states;

	// [m] last terrain estimate which was valid
	float _last_valid_terrain_alt_estimate{0.0f};

	// [us] time at which we had last valid terrain alt
	hrt_abstime _last_time_terrain_alt_was_valid{0};

	enum TerrainEstimateUseOnLanding {
		kDisableTerrainEstimation = 0,
		kTriggerFlareWithTerrainEstimate,
		kFollowTerrainRelativeLandingGlideSlope
	};

	// AIRSPEED

	float _airspeed{0.0f};
	float _eas2tas{1.0f};
	bool _airspeed_valid{false};
	float _air_density{CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C};

	// [us] last time airspeed was received. used to detect timeouts.
	hrt_abstime _time_airspeed_last_valid{0};

	// WIND

	// [m/s] wind velocity vector
	Vector2f _wind_vel{0.0f, 0.0f};

	bool _wind_valid{false};

	hrt_abstime _time_wind_last_received{0}; // [us]

	// TECS

	// total energy control system - airspeed / altitude control
	TECS _tecs;

	bool _reinitialize_tecs{true};
	bool _tecs_is_running{false};
	hrt_abstime _time_last_tecs_update{0}; // [us]

	// VTOL / TRANSITION

	float _airspeed_after_transition{0.0f};
	bool _was_in_transition{false};
	bool _is_vtol_tailsitter{false};
	matrix::Vector2d _transition_waypoint{(double)NAN, (double)NAN};
	param_t _param_handle_airspeed_trans{PARAM_INVALID};
	float _param_airspeed_trans{NAN}; // [m/s]

	// ESTIMATOR RESET COUNTERS

	// captures the number of times the estimator has reset the horizontal position
	uint8_t _pos_reset_counter{0};

	// captures the number of times the estimator has reset the altitude state
	uint8_t _alt_reset_counter{0};

	// LATERAL-DIRECTIONAL GUIDANCE

	// CLosest point on path to track
	matrix::Vector2f _closest_point_on_path;

	// nonlinear path following guidance - lateral-directional position control
	NPFG _npfg;

	// LANDING GEAR
	int8_t _new_landing_gear_position{landing_gear_s::GEAR_KEEP};

	// FLAPS/SPOILERS
	float _flaps_setpoint{0.f};
	float _spoilers_setpoint{0.f};

	hrt_abstime _time_in_fixed_bank_loiter{0}; // [us]
	float _min_current_sp_distance_xy{FLT_MAX};
	float _target_bearing{0.0f}; // [rad]

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
	void tecs_status_publish(float alt_sp, float equivalent_airspeed_sp, float true_airspeed_derivative_raw,
				 float throttle_trim);
	void publishLocalPositionSetpoint(const position_setpoint_s &current_waypoint);

	/**
	 * @brief Sets the landing abort status and publishes landing status.
	 *
	 * @param new_abort_status Either 0 (not aborted) or the singular bit >0 which triggered the abort
	 */
	void updateLandingAbortStatus(const uint8_t new_abort_status = position_controller_landing_status_s::NOT_ABORTED);

	/**
	 * @brief Checks if the automatic abort bitmask (from FW_LND_ABORT) contains the given abort criterion.
	 *
	 * @param automatic_abort_criteria_bitmask Bitmask containing all active abort criteria
	 * @param landing_abort_criterion The specifc criterion we are checking for
	 * @return true if the bitmask contains the criterion
	 */
	bool checkLandingAbortBitMask(const uint8_t automatic_abort_criteria_bitmask, uint8_t landing_abort_criterion);

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
	 * @brief Updates a state indicating whether a manual takeoff has been completed.
	 *
	 * Criteria include passing an airspeed threshold and not being in a landed state. VTOL airframes always pass.
	 */
	void updateManualTakeoffStatus();

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
	 * @param global_position Vechile global position [deg]
	 * @param ground_speed Local 2D ground speed of vehicle [m/s]
	 * @param pos_sp_curr current position setpoint
	 */
	void control_auto_takeoff(const hrt_abstime &now, const float control_interval, const Vector2d &global_position,
				  const Vector2f &ground_speed, const position_setpoint_s &pos_sp_curr);

	/**
	 * @brief Controls automatic landing with straight approach.
	 *
	 * To be used in Missions that contain a loiter down followed by a land waypoint.
	 *
	 * @param now Current system time [us]
	 * @param control_interval Time since last position control call [s]
	 * @param control_interval Time since the last position control update [s]
	 * @param ground_speed Local 2D ground speed of vehicle [m/s]
	 * @param pos_sp_prev previous position setpoint
	 * @param pos_sp_curr current position setpoint
	 */
	void control_auto_landing_straight(const hrt_abstime &now, const float control_interval, const Vector2f &ground_speed,
					   const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr);

	/**
	 * @brief Controls automatic landing with circular final appraoch.
	 *
	 * To be used outside of Mission landings. Vehicle will orbit down around the landing position setpoint until flaring.
	 *
	 * @param now Current system time [us]
	 * @param control_interval Time since last position control call [s]
	 * @param control_interval Time since the last position control update [s]
	 * @param ground_speed Local 2D ground speed of vehicle [m/s]
	 * @param pos_sp_curr current position setpoint
	 */
	void control_auto_landing_circular(const hrt_abstime &now, const float control_interval, const Vector2f &ground_speed,
					   const position_setpoint_s &pos_sp_curr);

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
	 * @brief Returns an adapted calibrated airspeed setpoint
	 *
	 * Adjusts the setpoint for wind, accelerated stall, and slew rates.
	 *
	 * @param control_interval Time since the last position control update [s]
	 * @param calibrated_airspeed_setpoint Calibrated airspeed septoint (generally from the position setpoint) [m/s]
	 * @param calibrated_min_airspeed Minimum calibrated airspeed [m/s]
	 * @param ground_speed Vehicle ground velocity vector (NE) [m/s]
	 * @return Adjusted calibrated airspeed setpoint [m/s]
	 */
	float adapt_airspeed_setpoint(const float control_interval, float calibrated_airspeed_setpoint,
				      float calibrated_min_airspeed, const Vector2f &ground_speed);

	void reset_takeoff_state();
	void reset_landing_state();

	/**
	 * @brief Decides which control mode to execute.
	 *
	 * May also change the position setpoint type depending on the desired behavior.
	 *
	 * @param now Current system time [us]
	 */
	void set_control_mode_current(const hrt_abstime &now);

	/**
	 * @brief Estimate trim throttle for air density, vehicle weight and current airspeed
	 *
	 * @param throttle_min Minimum allowed trim throttle.
	 * @param throttle_max Maximum allowed trim throttle.
	 * @param airspeed_sp Current airspeed setpoint (CAS) [m/s]
	 * @return Estimated trim throttle
	 */
	float calculateTrimThrottle(float throttle_min, float throttle_max,
				    float airspeed_sp);

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
	 * @param desired_max_sink_rate The desired max sink rate commandable when altitude errors are large [m/s]
	 * @param desired_max_climb_rate The desired max climb rate commandable when altitude errors are large [m/s]
	 * @param disable_underspeed_detection True if underspeed detection should be disabled
	 * @param hgt_rate_sp Height rate setpoint [m/s]
	 */
	void tecs_update_pitch_throttle(const float control_interval, float alt_sp, float airspeed_sp, float pitch_min_rad,
					float pitch_max_rad, float throttle_min, float throttle_max,
					const float desired_max_sink_rate, const float desired_max_climb_rate,
					bool disable_underspeed_detection = false, float hgt_rate_sp = NAN);

	/**
	 * @brief Constrains the roll angle setpoint near ground to avoid wingtip strike.
	 *
	 * @param roll_setpoint Unconstrained roll angle setpoint [rad]
	 * @param altitude Vehicle altitude (AMSL) [m]
	 * @param terrain_altitude Terrain altitude (AMSL) [m]
	 * @return Constrained roll angle setpoint [rad]
	 */
	float constrainRollNearGround(const float roll_setpoint, const float altitude, const float terrain_altitude) const;

	/**
	 * @brief Calculates the unit takeoff bearing vector from the launch position to takeoff waypont.
	 *
	 * @param launch_position Vehicle launch position in local coordinates (NE) [m]
	 * @param takeoff_waypoint Takeoff waypoint position in local coordinates (NE) [m]
	 * @return Unit takeoff bearing vector
	 */
	Vector2f calculateTakeoffBearingVector(const Vector2f &launch_position, const Vector2f &takeoff_waypoint) const;

	/**
	 * @brief Calculates the touchdown position for landing with optional manual lateral adjustments.
	 *
	 * Manual inputs (from the remote) are used to command a rate at which the position moves and the integrated
	 * position is bounded. This is useful for manually adjusting the landing point in real time when map or GNSS
	 * errors cause an offset from the desired landing vector.
	 *
	 * @param control_interval Time since the last position control update [s]
	 * @param local_land_position Originally commanded local land position (NE) [m]
	 * @return (Nudged) Local touchdown position (NE) [m]
	 */
	Vector2f calculateTouchdownPosition(const float control_interval, const Vector2f &local_land_position);

	/**
	 * @brief Calculates the vector from landing approach entrance to touchdown point
	 *
	 * NOTE: calculateTouchdownPosition() MUST be called before this method
	 *
	 * @return Landing approach vector [m]
	 */
	Vector2f calculateLandingApproachVector() const;

	/**
	 * @brief Returns a terrain altitude estimate with consideration of altimeter measurements.
	 *
	 * @param now Current system time [us]
	 * @param land_point_altitude Altitude (AMSL) of the land point [m]
	 * @param abort_on_terrain_measurement_timeout Abort if distance to ground estimation doesn't get valid when we expect it to
	 * @param abort_on_terrain_timeout Abort if distance to ground estimation is invalid after being valid before
	 * @return Terrain altitude (AMSL) [m]
	 */
	float getLandingTerrainAltitudeEstimate(const hrt_abstime &now, const float land_point_altitude,
						const bool abort_on_terrain_measurement_timeout, const bool abort_on_terrain_timeout);

	/**
	 * @brief Initializes landing states
	 *
	 * @param now Current system time [us]
	 * @param pos_sp_prev Previous position setpoint
	 * @param land_point_alt Landing point altitude setpoint AMSL [m]
	 * @param local_position Local aircraft position (NE) [m]
	 * @param local_land_point Local land point (NE) [m]
	 */
	void initializeAutoLanding(const hrt_abstime &now, const position_setpoint_s &pos_sp_prev,
				   const float land_point_alt, const Vector2f &local_position, const Vector2f &local_land_point);

	/*
	 * Waypoint handling logic following closely to the ECL_L1_Pos_Controller
	 * method of the same name. Takes two waypoints and determines the relevant
	 * parameters for evaluating the NPFG guidance law, then updates control setpoints.
	 *
	 * @param[in] waypoint_A Waypoint A (segment start) position in WGS84 coordinates
	 *            (lat,lon) [deg]
	 * @param[in] waypoint_B Waypoint B (segment end) position in WGS84 coordinates
	 *            (lat,lon) [deg]
	 * @param[in] vehicle_pos Vehicle position in WGS84 coordinates (lat,lon) [deg]
	 * @param[in] ground_vel Vehicle ground velocity vector [m/s]
	 * @param[in] wind_vel Wind velocity vector [m/s]
	 */
	void navigateWaypoints(const matrix::Vector2f &waypoint_A, const matrix::Vector2f &waypoint_B,
			       const matrix::Vector2f &vehicle_pos, const matrix::Vector2f &ground_vel,
			       const matrix::Vector2f &wind_vel);

	/*
	 * Loitering (unlimited) logic. Takes loiter center, radius, and direction and
	 * determines the relevant parameters for evaluating the NPFG guidance law,
	 * then updates control setpoints.
	 *
	 * @param[in] loiter_center The position of the center of the loiter circle [m]
	 * @param[in] vehicle_pos Vehicle position in WGS84 coordinates (lat,lon) [deg]
	 * @param[in] radius Loiter radius [m]
	 * @param[in] loiter_direction_counter_clockwise Specifies loiter direction
	 * @param[in] ground_vel Vehicle ground velocity vector [m/s]
	 * @param[in] wind_vel Wind velocity vector [m/s]
	 */
	void navigateLoiter(const matrix::Vector2f &loiter_center, const matrix::Vector2f &vehicle_pos,
			    float radius, bool loiter_direction_counter_clockwise, const matrix::Vector2f &ground_vel,
			    const matrix::Vector2f &wind_vel);

	/*
	 * Path following logic. Takes poisiton, path tangent, curvature and
	 * then updates control setpoints to follow a path setpoint.
	 *
	 * @param[in] vehicle_pos vehicle_pos Vehicle position in WGS84 coordinates (lat,lon) [deg]
	 * @param[in] position_setpoint closest point on a path in WGS84 coordinates (lat,lon) [deg]
	 * @param[in] tangent_setpoint unit tangent vector of the path [m]
	 * @param[in] ground_vel Vehicle ground velocity vector [m/s]
	 * @param[in] wind_vel Wind velocity vector [m/s]
	 * @param[in] curvature of the path setpoint [1/m]
	 */
	void navigatePathTangent(const matrix::Vector2f &vehicle_pos, const matrix::Vector2f &position_setpoint,
				 const matrix::Vector2f &tangent_setpoint,
				 const matrix::Vector2f &ground_vel, const matrix::Vector2f &wind_vel, const float &curvature);

	/*
	 * Navigate on a fixed bearing.
	 *
	 * This only holds a certain (ground relative) direction and does not perform
	 * cross track correction. Helpful for semi-autonomous modes. Similar to navigateHeading.
	 *
	 * @param[in] vehicle_pos vehicle_pos Vehicle position in WGS84 coordinates (lat,lon) [deg]
	 * @param[in] bearing Bearing angle [rad]
	 * @param[in] ground_vel Vehicle ground velocity vector [m/s]
	 * @param[in] wind_vel Wind velocity vector [m/s]
	 */
	void navigateBearing(const matrix::Vector2f &vehicle_pos, float bearing, const matrix::Vector2f &ground_vel,
			     const matrix::Vector2f &wind_vel);

	DEFINE_PARAMETERS(

		(ParamFloat<px4::params::FW_AIRSPD_MAX>) _param_fw_airspd_max,
		(ParamFloat<px4::params::FW_AIRSPD_MIN>) _param_fw_airspd_min,
		(ParamFloat<px4::params::FW_AIRSPD_TRIM>) _param_fw_airspd_trim,
		(ParamFloat<px4::params::FW_AIRSPD_STALL>) _param_fw_airspd_stall,

		(ParamFloat<px4::params::FW_GND_SPD_MIN>) _param_fw_gnd_spd_min,

		(ParamFloat<px4::params::FW_PN_R_SLEW_MAX>) _param_fw_pn_r_slew_max,
		(ParamFloat<px4::params::FW_R_LIM>) _param_fw_r_lim,

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

		(ParamFloat<px4::params::FW_LND_AIRSPD>) _param_fw_lnd_airspd,
		(ParamFloat<px4::params::FW_LND_ANG>) _param_fw_lnd_ang,
		(ParamFloat<px4::params::FW_LND_FL_PMAX>) _param_fw_lnd_fl_pmax,
		(ParamFloat<px4::params::FW_LND_FL_PMIN>) _param_fw_lnd_fl_pmin,
		(ParamFloat<px4::params::FW_LND_FLALT>) _param_fw_lnd_flalt,
		(ParamFloat<px4::params::FW_LND_THRTC_SC>) _param_fw_thrtc_sc,
		(ParamBool<px4::params::FW_LND_EARLYCFG>) _param_fw_lnd_earlycfg,
		(ParamInt<px4::params::FW_LND_USETER>) _param_fw_lnd_useter,

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
		(ParamFloat<px4::params::FW_T_SPDWEIGHT>) _param_fw_t_spdweight,
		(ParamFloat<px4::params::FW_T_TAS_TC>) _param_fw_t_tas_error_tc,
		(ParamFloat<px4::params::FW_T_THR_DAMP>) _param_fw_t_thr_damp,
		(ParamFloat<px4::params::FW_T_VERT_ACC>) _param_fw_t_vert_acc,
		(ParamFloat<px4::params::FW_T_STE_R_TC>) _param_ste_rate_time_const,
		(ParamFloat<px4::params::FW_T_SEB_R_FF>) _param_seb_rate_ff,
		(ParamFloat<px4::params::FW_T_CLMB_R_SP>) _param_climbrate_target,
		(ParamFloat<px4::params::FW_T_SINK_R_SP>) _param_sinkrate_target,
		(ParamFloat<px4::params::FW_T_SPD_STD>) _param_speed_standard_dev,
		(ParamFloat<px4::params::FW_T_SPD_DEV_STD>) _param_speed_rate_standard_dev,
		(ParamFloat<px4::params::FW_T_SPD_PRC_STD>) _param_process_noise_standard_dev,

		(ParamFloat<px4::params::FW_THR_ASPD_MIN>) _param_fw_thr_aspd_min,
		(ParamFloat<px4::params::FW_THR_ASPD_MAX>) _param_fw_thr_aspd_max,

		(ParamFloat<px4::params::FW_THR_TRIM>) _param_fw_thr_trim,
		(ParamFloat<px4::params::FW_THR_IDLE>) _param_fw_thr_idle,
		(ParamFloat<px4::params::FW_THR_MAX>) _param_fw_thr_max,
		(ParamFloat<px4::params::FW_THR_MIN>) _param_fw_thr_min,
		(ParamFloat<px4::params::FW_THR_SLEW_MAX>) _param_fw_thr_slew_max,

		(ParamFloat<px4::params::FW_FLAPS_LND_SCL>) _param_fw_flaps_lnd_scl,
		(ParamFloat<px4::params::FW_FLAPS_TO_SCL>) _param_fw_flaps_to_scl,
		(ParamFloat<px4::params::FW_SPOILERS_LND>) _param_fw_spoilers_lnd,
		(ParamFloat<px4::params::FW_SPOILERS_DESC>) _param_fw_spoilers_desc,

		(ParamInt<px4::params::FW_POS_STK_CONF>) _param_fw_pos_stk_conf,

		(ParamInt<px4::params::FW_GPSF_LT>) _param_nav_gpsf_lt,
		(ParamFloat<px4::params::FW_GPSF_R>) _param_nav_gpsf_r,

		// external parameters
		(ParamInt<px4::params::FW_ARSP_MODE>) _param_fw_arsp_mode,

		(ParamFloat<px4::params::FW_PSP_OFF>) _param_fw_psp_off,

		(ParamFloat<px4::params::NAV_LOITER_RAD>) _param_nav_loiter_rad,

		(ParamFloat<px4::params::FW_TKO_PITCH_MIN>) _takeoff_pitch_min,

		(ParamFloat<px4::params::NAV_FW_ALT_RAD>) _param_nav_fw_alt_rad,

		(ParamFloat<px4::params::WEIGHT_BASE>) _param_weight_base,
		(ParamFloat<px4::params::WEIGHT_GROSS>) _param_weight_gross,

		(ParamFloat<px4::params::FW_WING_SPAN>) _param_fw_wing_span,
		(ParamFloat<px4::params::FW_WING_HEIGHT>) _param_fw_wing_height,

		(ParamFloat<px4::params::RWTO_NPFG_PERIOD>) _param_rwto_npfg_period,
		(ParamBool<px4::params::RWTO_NUDGE>) _param_rwto_nudge,

		(ParamFloat<px4::params::FW_LND_FL_TIME>) _param_fw_lnd_fl_time,
		(ParamFloat<px4::params::FW_LND_FL_SINK>) _param_fw_lnd_fl_sink,
		(ParamFloat<px4::params::FW_LND_TD_TIME>) _param_fw_lnd_td_time,
		(ParamFloat<px4::params::FW_LND_TD_OFF>) _param_fw_lnd_td_off,
		(ParamInt<px4::params::FW_LND_NUDGE>) _param_fw_lnd_nudge,
		(ParamInt<px4::params::FW_LND_ABORT>) _param_fw_lnd_abort,

		(ParamFloat<px4::params::FW_WIND_ARSP_SC>) _param_fw_wind_arsp_sc,

		(ParamFloat<px4::params::FW_TKO_AIRSPD>) _param_fw_tko_airspd,

		(ParamFloat<px4::params::RWTO_PSP>) _param_rwto_psp,
		(ParamBool<px4::params::FW_LAUN_DETCN_ON>) _param_fw_laun_detcn_on
	)

};

#endif // FIXEDWINGPOSITIONCONTROL_HPP_
