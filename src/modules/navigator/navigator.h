/***************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file navigator.h
 * Helper class to access missions
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#pragma once

#include "geofence.h"
#include "land.h"
#include "precland.h"
#include "loiter.h"
#include "mission.h"
#include "navigator_mode.h"
#include "rtl.h"
#include "takeoff.h"
#include "vtol_takeoff.h"

#include "navigation.h"

#include "GeofenceBreachAvoidance/geofence_breach_avoidance.h"

#include <lib/adsb/AdsbConflict.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_controller_landing_status.h>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/transponder_report.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/mode_completed.h>
#include <uORB/uORB.h>

using namespace time_literals;

/**
 * Number of navigation modes that need on_active/on_inactive calls
 */
#define NAVIGATOR_MODE_ARRAY_SIZE 8

class Navigator : public ModuleBase<Navigator>, public ModuleParams
{
public:
	Navigator();
	~Navigator() override;

	Navigator(const Navigator &) = delete;
	Navigator operator=(const Navigator &) = delete;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static Navigator *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	/**
	 * Load fence from file
	 */
	void load_fence_from_file(const char *filename);

	/**
	 * @brief Publish a given specified vehicle command
	 *
	 * Sets the target_component of the vehicle command accordingly depending on the
	 * vehicle command value (e.g. For Camera control, sets target system component id)
	 *
	 * @param vcmd Vehicle command to execute
	 */
	void publish_vehicle_cmd(vehicle_command_s *vcmd);

	/**
	 * Check nearby traffic for potential collisions
	 */
	void check_traffic();

	void take_traffic_conflict_action();

	void run_fake_traffic();

	/**
	 * Setters
	 */
	void set_can_loiter_at_sp(bool can_loiter) { _can_loiter_at_sp = can_loiter; }
	void set_position_setpoint_triplet_updated() { _pos_sp_triplet_updated = true; }
	void set_mission_result_updated() { _mission_result_updated = true; }

	/**
	 * Getters
	 */
	home_position_s             *get_home_position() { return &_home_pos; }
	mission_result_s            *get_mission_result() { return &_mission_result; }
	position_setpoint_triplet_s *get_position_setpoint_triplet() { return &_pos_sp_triplet; }
	position_setpoint_triplet_s *get_reposition_triplet() { return &_reposition_triplet; }
	position_setpoint_triplet_s *get_takeoff_triplet() { return &_takeoff_triplet; }
	vehicle_global_position_s   *get_global_position() { return &_global_pos; }
	vehicle_land_detected_s     *get_land_detected() { return &_land_detected; }
	vehicle_local_position_s    *get_local_position() { return &_local_pos; }
	vehicle_status_s            *get_vstatus() { return &_vstatus; }

	PrecLand *get_precland() { return &_precland; } /**< allow others, e.g. Mission, to use the precision land block */

	const vehicle_roi_s &get_vroi() { return _vroi; }

	void reset_vroi() { _vroi = {}; }

	bool home_alt_valid() { return (_home_pos.valid_alt); }

	bool home_global_position_valid() { return (_home_pos.valid_alt && _home_pos.valid_hpos); }

	Geofence &get_geofence() { return _geofence; }

	bool get_can_loiter_at_sp() { return _can_loiter_at_sp; }

	float get_loiter_radius() { return _param_nav_loiter_rad.get(); }

	/**
	 * Returns the default acceptance radius defined by the parameter
	 */
	float get_default_acceptance_radius();

	/**
	 * Get the acceptance radius
	 *
	 * @return the distance at which the next waypoint should be used
	 */
	float get_acceptance_radius();

	/**
	 * Get the altitude acceptance radius
	 *
	 * @return the distance from the target altitude before considering the waypoint reached
	 */
	float get_altitude_acceptance_radius();

	/**
	 * Get the cruising speed
	 *
	 * @return the desired cruising speed for this mission
	 */
	float get_cruising_speed();

	/**
	 * Set the cruising speed
	 *
	 * Passing a negative value or leaving the parameter away will reset the cruising speed
	 * to its default value.
	 *
	 * For VTOL: sets cruising speed for current mode only (multirotor or fixed-wing).
	 *
	 */
	void set_cruising_speed(float speed = -1.0f);

	/**
	 * Reset cruising speed to default values
	 *
	 * For VTOL: resets both cruising speeds.
	 */
	void reset_cruising_speed();

	/**
	 *  Set triplets to invalid
	 */
	void reset_triplets();

	/**
	 *  Set position setpoint to safe defaults
	 */
	void reset_position_setpoint(position_setpoint_s &sp);

	/**
	 * Get the target throttle
	 *
	 * @return the desired throttle for this mission
	 */
	float get_cruising_throttle();

	/**
	 * Set the target throttle
	 */
	void set_cruising_throttle(float throttle = NAN) { _mission_throttle = throttle; }

	/**
	 * Get the yaw acceptance given the current mission item
	 *
	 * @param mission_item_yaw the yaw to use in case the controller-derived radius is finite
	 *
	 * @return the yaw at which the next waypoint should be used or NaN if the yaw at a waypoint
	 * should be ignored
	 */
	float get_yaw_acceptance(float mission_item_yaw);

	orb_advert_t *get_mavlink_log_pub() { return &_mavlink_log_pub; }

	void increment_mission_instance_count() { _mission_result.instance_count++; }

	int mission_instance_count() const { return _mission_result.instance_count; }

	void set_mission_failure_heading_timeout();

	bool is_planned_mission() const { return _navigation_mode == &_mission; }

	bool on_mission_landing() { return (_mission.landing() && _navigation_mode == &_mission); }

	bool start_mission_landing() { return _mission.land_start(); }

	bool get_mission_start_land_available() { return _mission.get_land_start_available(); }

	int  get_mission_landing_index() { return _mission.get_land_start_index(); }

	double get_mission_landing_start_lat() { return _mission.get_landing_start_lat(); }
	double get_mission_landing_start_lon() { return _mission.get_landing_start_lon(); }
	float  get_mission_landing_start_alt() { return _mission.get_landing_start_alt(); }

	double get_mission_landing_lat() { return _mission.get_landing_lat(); }
	double get_mission_landing_lon() { return _mission.get_landing_lon(); }
	float  get_mission_landing_alt() { return _mission.get_landing_alt(); }

	float get_mission_landing_loiter_radius() { return _mission.get_landing_loiter_rad(); }

	// RTL
	bool in_rtl_state() const { return _vstatus.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL; }

	bool abort_landing();

	void geofence_breach_check(bool &have_geofence_position_data);

	// Param access
	int get_loiter_min_alt() const { return _param_min_ltr_alt.get(); }
	int get_landing_abort_min_alt() const { return _param_mis_lnd_abrt_alt.get(); }
	float get_takeoff_min_alt() const { return _param_mis_takeoff_alt.get(); }
	int  get_takeoff_land_required() const { return _para_mis_takeoff_land_req.get(); }
	float get_yaw_timeout() const { return _param_mis_yaw_tmt.get(); }
	float get_yaw_threshold() const { return math::radians(_param_mis_yaw_err.get()); }
	float get_lndmc_alt_max() const { return _param_lndmc_alt_max.get(); }

	float get_vtol_back_trans_deceleration() const { return _param_back_trans_dec_mss; }

	bool force_vtol();

	void acquire_gimbal_control();
	void release_gimbal_control();

	void calculate_breaking_stop(double &lat, double &lon, float &yaw);

	void stop_capturing_images();
	void disable_camera_trigger();

	void mode_completed(uint8_t nav_state, uint8_t result = mode_completed_s::RESULT_SUCCESS);

private:

	orb_sub_t _local_pos_sub{ORB_SUB_INVALID};
	orb_sub_t _mission_sub{ORB_SUB_INVALID};
	orb_sub_t _vehicle_status_sub{ORB_SUB_INVALID};

	uORB::SubscriptionData<position_controller_status_s>	_position_controller_status_sub{ORB_ID(position_controller_status)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};	/**< global position subscription */
	uORB::Subscription _gps_pos_sub{ORB_ID(vehicle_gps_position)};		/**< gps position subscription */
	uORB::Subscription _home_pos_sub{ORB_ID(home_position)};		/**< home position subscription */
	uORB::Subscription _land_detected_sub{ORB_ID(vehicle_land_detected)};	/**< vehicle land detected subscription */
	uORB::Subscription _pos_ctrl_landing_status_sub{ORB_ID(position_controller_landing_status)};	/**< position controller landing status subscription */
	uORB::Subscription _traffic_sub{ORB_ID(transponder_report)};		/**< traffic subscription */
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};	/**< vehicle commands (onboard and offboard) */

	uORB::Publication<geofence_result_s>		_geofence_result_pub{ORB_ID(geofence_result)};
	uORB::Publication<mission_result_s>		_mission_result_pub{ORB_ID(mission_result)};
	uORB::Publication<position_setpoint_triplet_s>	_pos_sp_triplet_pub{ORB_ID(position_setpoint_triplet)};
	uORB::Publication<vehicle_command_ack_s>	_vehicle_cmd_ack_pub{ORB_ID(vehicle_command_ack)};
	uORB::Publication<vehicle_command_s>		_vehicle_cmd_pub{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_roi_s>		_vehicle_roi_pub{ORB_ID(vehicle_roi)};
	uORB::Publication<mode_completed_s> _mode_completed_pub{ORB_ID(mode_completed)};

	orb_advert_t	_mavlink_log_pub{nullptr};	/**< the uORB advert to send messages over mavlink */

	// Subscriptions
	home_position_s					_home_pos{};		/**< home position for RTL */
	mission_result_s				_mission_result{};
	vehicle_global_position_s			_global_pos{};		/**< global vehicle position */
	sensor_gps_s				_gps_pos{};		/**< gps position */
	vehicle_land_detected_s				_land_detected{};	/**< vehicle land_detected */
	vehicle_local_position_s			_local_pos{};		/**< local vehicle position */
	vehicle_status_s				_vstatus{};		/**< vehicle status */

	bool						_rtl_activated{false};

	// Publications
	geofence_result_s				_geofence_result{};
	position_setpoint_triplet_s			_pos_sp_triplet{};	/**< triplet of position setpoints */
	position_setpoint_triplet_s			_reposition_triplet{};	/**< triplet for non-mission direct position command */
	position_setpoint_triplet_s			_takeoff_triplet{};	/**< triplet for non-mission direct takeoff command */
	vehicle_roi_s					_vroi{};		/**< vehicle ROI */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	Geofence	_geofence;			/**< class that handles the geofence */
	GeofenceBreachAvoidance _gf_breach_avoidance;
	hrt_abstime _last_geofence_check = 0;

	bool		_geofence_violation_warning_sent{false};	/**< prevents spaming to mavlink */
	bool		_can_loiter_at_sp{false};			/**< flags if current position SP can be used to loiter */
	bool		_pos_sp_triplet_updated{false};			/**< flags if position SP triplet needs to be published */
	bool 		_pos_sp_triplet_published_invalid_once{false};	/**< flags if position SP triplet has been published once to UORB */
	bool		_mission_result_updated{false};			/**< flags if mission result has seen an update */

	bool		_shouldEngageMissionForLanding{false};

	Mission		_mission;			/**< class that handles the missions */
	Loiter		_loiter;			/**< class that handles loiter */
	Takeoff		_takeoff;			/**< class for handling takeoff commands */
	VtolTakeoff	_vtol_takeoff;			/**< class for handling VEHICLE_CMD_NAV_VTOL_TAKEOFF command */
	Land		_land;			/**< class for handling land commands */
	PrecLand	_precland;			/**< class for handling precision land commands */
	RTL 		_rtl;				/**< class that handles RTL */
	AdsbConflict 	_adsb_conflict;			/**< class that handles ADSB conflict avoidance */

	NavigatorMode *_navigation_mode{nullptr};	/**< abstract pointer to current navigation mode class */
	NavigatorMode *_navigation_mode_array[NAVIGATOR_MODE_ARRAY_SIZE] {};	/**< array of navigation modes */

	param_t _handle_back_trans_dec_mss{PARAM_INVALID};
	param_t _handle_mpc_jerk_auto{PARAM_INVALID};
	param_t _handle_mpc_acc_hor{PARAM_INVALID};

	float _param_back_trans_dec_mss{0.f};
	float _param_mpc_jerk_auto{4.f}; 	/**< initialized with the default jerk auto value to prevent division by 0 if the parameter is accidentally set to 0 */
	float _param_mpc_acc_hor{3.f};		/**< initialized with the default horizontal acc value to prevent division by 0 if the parameter is accidentally set to 0 */

	float _mission_cruising_speed_mc{-1.0f};
	float _mission_cruising_speed_fw{-1.0f};
	float _mission_throttle{NAN};

	traffic_buffer_s _traffic_buffer{};

	bool _is_capturing_images{false}; // keep track if we need to stop capturing images


	// update subscriptions
	void params_update();

	/**
	 * Publish a new position setpoint triplet for position controllers
	 */
	void publish_position_setpoint_triplet();

	/**
	 * Publish the mission result so commander and mavlink know what is going on
	 */
	void publish_mission_result();

	void publish_vehicle_command_ack(const vehicle_command_s &cmd, uint8_t result);

	bool geofence_allows_position(const vehicle_global_position_s &pos);

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::NAV_LOITER_RAD>)   _param_nav_loiter_rad,	/**< loiter radius for fixedwing */
		(ParamFloat<px4::params::NAV_ACC_RAD>)      _param_nav_acc_rad,		/**< acceptance for takeoff */
		(ParamFloat<px4::params::NAV_FW_ALT_RAD>)   _param_nav_fw_alt_rad,	/**< acceptance rad for fixedwing alt */
		(ParamFloat<px4::params::NAV_FW_ALTL_RAD>)
		_param_nav_fw_altl_rad,	/**< acceptance rad for fixedwing alt before landing*/
		(ParamFloat<px4::params::NAV_MC_ALT_RAD>)   _param_nav_mc_alt_rad,	/**< acceptance rad for multicopter alt */
		(ParamInt<px4::params::NAV_FORCE_VT>)       _param_nav_force_vt,	/**< acceptance radius for multicopter alt */
		(ParamInt<px4::params::NAV_TRAFF_AVOID>)    _param_nav_traff_avoid,	/**< avoiding other aircraft is enabled */
		(ParamFloat<px4::params::NAV_TRAFF_A_HOR>)  _param_nav_traff_a_hor_ct,	/**< avoidance Distance Crosstrack*/
		(ParamFloat<px4::params::NAV_TRAFF_A_VER>)  _param_nav_traff_a_ver,	/**< avoidance Distance Vertical*/
		(ParamInt<px4::params::NAV_TRAFF_COLL_T>)   _param_nav_traff_collision_time,
		(ParamFloat<px4::params::NAV_MIN_LTR_ALT>)   _param_min_ltr_alt,	/**< minimum altitude in Loiter mode*/

		// non-navigator parameters: Mission (MIS_*)
		(ParamFloat<px4::params::MIS_TAKEOFF_ALT>) _param_mis_takeoff_alt,
		(ParamInt<px4::params::MIS_TKO_LAND_REQ>)  _para_mis_takeoff_land_req,
		(ParamFloat<px4::params::MIS_YAW_TMT>)     _param_mis_yaw_tmt,
		(ParamFloat<px4::params::MIS_YAW_ERR>)     _param_mis_yaw_err,
		(ParamFloat<px4::params::MIS_PD_TO>)       _param_mis_payload_delivery_timeout,
		(ParamFloat<px4::params::LNDMC_ALT_MAX>)   _param_lndmc_alt_max,
		(ParamInt<px4::params::MIS_LND_ABRT_ALT>)  _param_mis_lnd_abrt_alt
	)
};
