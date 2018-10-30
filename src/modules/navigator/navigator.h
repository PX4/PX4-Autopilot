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

#include "datalinkloss.h"
#include "enginefailure.h"
#include "follow_target.h"
#include "geofence.h"
#include "gpsfailure.h"
#include "land.h"
#include "precland.h"
#include "loiter.h"
#include "mission.h"
#include "navigator_mode.h"
#include "rcloss.h"
#include "rtl.h"
#include "takeoff.h"

#include "navigation.h"

#include <lib/perf/perf_counter.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/uORB.h>

/**
 * Number of navigation modes that need on_active/on_inactive calls
 */
#define NAVIGATOR_MODE_ARRAY_SIZE 11


class Navigator : public ModuleBase<Navigator>, public ModuleParams
{
public:
	Navigator();
	virtual ~Navigator() = default;
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
	void		load_fence_from_file(const char *filename);

	/**
	 * Publish the geofence result
	 */
	void		publish_geofence_result();

	void		publish_vehicle_cmd(vehicle_command_s *vcmd);

	/**
	 * Generate an artificial traffic indication
	 *
	 * @param distance Horizontal distance to this vehicle
	 * @param direction Direction in earth frame from this vehicle in radians
	 * @param traffic_heading Travel direction of the traffic in earth frame in radians
	 * @param altitude_diff Altitude difference, positive is up
	 * @param hor_velocity Horizontal velocity of traffic, in m/s
	 * @param ver_velocity Vertical velocity of traffic, in m/s
	 */
	void		fake_traffic(const char *callsign, float distance, float direction, float traffic_heading, float altitude_diff,
				     float hor_velocity, float ver_velocity);

	/**
	 * Check nearby traffic for potential collisions
	 */
	void		check_traffic();

	/**
	 * Setters
	 */
	void		set_can_loiter_at_sp(bool can_loiter) { _can_loiter_at_sp = can_loiter; }
	void		set_position_setpoint_triplet_updated() { _pos_sp_triplet_updated = true; }
	void		set_mission_result_updated() { _mission_result_updated = true; }

	/**
	 * Getters
	 */
	struct home_position_s *get_home_position() { return &_home_pos; }
	struct mission_result_s *get_mission_result() { return &_mission_result; }
	struct position_setpoint_triplet_s *get_position_setpoint_triplet() { return &_pos_sp_triplet; }
	struct position_setpoint_triplet_s *get_reposition_triplet() { return &_reposition_triplet; }
	struct position_setpoint_triplet_s *get_takeoff_triplet() { return &_takeoff_triplet; }
	struct vehicle_global_position_s *get_global_position() { return &_global_pos; }
	struct vehicle_land_detected_s *get_land_detected() { return &_land_detected; }
	struct vehicle_local_position_s *get_local_position() { return &_local_pos; }
	struct vehicle_status_s *get_vstatus() { return &_vstatus; }
	PrecLand *get_precland() { return &_precland; } /**< allow others, e.g. Mission, to use the precision land block */

	const vehicle_roi_s &get_vroi() { return _vroi; }

	bool home_alt_valid() { return (_home_pos.timestamp > 0 && _home_pos.valid_alt); }
	bool home_position_valid() { return (_home_pos.timestamp > 0 && _home_pos.valid_alt && _home_pos.valid_hpos); }

	int		get_offboard_mission_sub() { return _offboard_mission_sub; }

	Geofence	&get_geofence() { return _geofence; }

	bool		get_can_loiter_at_sp() { return _can_loiter_at_sp; }
	float		get_loiter_radius() { return _param_loiter_radius.get(); }

	/**
	 * Returns the default acceptance radius defined by the parameter
	 */
	float		get_default_acceptance_radius();

	/**
	 * Get the acceptance radius
	 *
	 * @return the distance at which the next waypoint should be used
	 */
	float		get_acceptance_radius();

	/**
	 * Get the default altitude acceptance radius (i.e. from parameters)
	 *
	 * @return the distance from the target altitude before considering the waypoint reached
	 */
	float		get_default_altitude_acceptance_radius();

	/**
	 * Get the altitude acceptance radius
	 *
	 * @return the distance from the target altitude before considering the waypoint reached
	 */
	float		get_altitude_acceptance_radius();

	/**
	 * Get the cruising speed
	 *
	 * @return the desired cruising speed for this mission
	 */
	float		get_cruising_speed();

	/**
	 * Set the cruising speed
	 *
	 * Passing a negative value or leaving the parameter away will reset the cruising speed
	 * to its default value.
	 *
	 * For VTOL: sets cruising speed for current mode only (multirotor or fixed-wing).
	 *
	 */
	void		set_cruising_speed(float speed = -1.0f);

	/**
	 * Reset cruising speed to default values
	 *
	 * For VTOL: resets both cruising speeds.
	 */
	void		reset_cruising_speed();


	/**
	 *  Set triplets to invalid
	 */
	void 		reset_triplets();

	/**
	 * Get the target throttle
	 *
	 * @return the desired throttle for this mission
	 */
	float		get_cruising_throttle();

	/**
	 * Set the target throttle
	 */
	void		set_cruising_throttle(float throttle = -1.0f) { _mission_throttle = throttle; }

	/**
	 * Get the acceptance radius given the mission item preset radius
	 *
	 * @param mission_item_radius the radius to use in case the controller-derived radius is smaller
	 *
	 * @return the distance at which the next waypoint should be used
	 */
	float		get_acceptance_radius(float mission_item_radius);

	/**
	 * Get the yaw acceptance given the current mission item
	 *
	 * @param mission_item_yaw the yaw to use in case the controller-derived radius is finite
	 *
	 * @return the yaw at which the next waypoint should be used or NaN if the yaw at a waypoint
	 * should be ignored
	 */
	float 		get_yaw_acceptance(float mission_item_yaw);

	orb_advert_t	*get_mavlink_log_pub() { return &_mavlink_log_pub; }

	void		increment_mission_instance_count() { _mission_result.instance_count++; }

	void 		set_mission_failure(const char *reason);

	// MISSION
	bool		is_planned_mission() const { return _navigation_mode == &_mission; }
	bool		on_mission_landing() { return _mission.landing(); }
	bool		start_mission_landing() { return _mission.land_start(); }
	bool		mission_start_land_available() { return _mission.get_land_start_available(); }

	// RTL
	bool		mission_landing_required() { return _rtl.rtl_type() == RTL::RTL_LAND; }
	int			rtl_type() { return _rtl.rtl_type(); }
	bool		in_rtl_state() const { return _vstatus.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL; }

	bool		abort_landing();

	// Param access
	float		get_loiter_min_alt() const { return _param_loiter_min_alt.get(); }
	float		get_takeoff_min_alt() const { return _param_takeoff_min_alt.get(); }
	float		get_yaw_timeout() const { return _param_yaw_timeout.get(); }
	float		get_yaw_threshold() const { return _param_yaw_err.get(); }

	float		get_vtol_back_trans_deceleration() const { return _param_back_trans_dec_mss; }
	float		get_vtol_reverse_delay() const { return _param_reverse_delay; }

	bool		force_vtol();

private:
	int		_global_pos_sub{-1};		/**< global position subscription */
	int		_gps_pos_sub{-1};		/**< gps position subscription */
	int		_home_pos_sub{-1};		/**< home position subscription */
	int		_land_detected_sub{-1};		/**< vehicle land detected subscription */
	int		_local_pos_sub{-1};		/**< local position subscription */
	int		_offboard_mission_sub{-1};	/**< offboard mission subscription */
	int		_param_update_sub{-1};		/**< param update subscription */
	int		_pos_ctrl_landing_status_sub{-1};	/**< position controller landing status subscription */
	int		_traffic_sub{-1};		/**< traffic subscription */
	int		_vehicle_command_sub{-1};	/**< vehicle commands (onboard and offboard) */
	int		_vstatus_sub{-1};		/**< vehicle status subscription */

	orb_advert_t	_geofence_result_pub{nullptr};
	orb_advert_t	_mavlink_log_pub{nullptr};	/**< the uORB advert to send messages over mavlink */
	orb_advert_t	_mission_result_pub{nullptr};
	orb_advert_t	_pos_sp_triplet_pub{nullptr};
	orb_advert_t	_vehicle_cmd_ack_pub{nullptr};
	orb_advert_t	_vehicle_cmd_pub{nullptr};
	orb_advert_t	_vehicle_roi_pub{nullptr};

	// Subscriptions
	home_position_s					_home_pos{};		/**< home position for RTL */
	mission_result_s				_mission_result{};
	vehicle_global_position_s			_global_pos{};		/**< global vehicle position */
	vehicle_gps_position_s				_gps_pos{};		/**< gps position */
	vehicle_land_detected_s				_land_detected{};	/**< vehicle land_detected */
	vehicle_local_position_s			_local_pos{};		/**< local vehicle position */
	vehicle_status_s				_vstatus{};		/**< vehicle status */

	uORB::Subscription<position_controller_status_s>	_position_controller_status_sub{ORB_ID(position_controller_status)};

	uint8_t						_previous_nav_state{}; /**< nav_state of the previous iteration*/

	// Publications
	geofence_result_s				_geofence_result{};
	position_setpoint_triplet_s			_pos_sp_triplet{};	/**< triplet of position setpoints */
	position_setpoint_triplet_s			_reposition_triplet{};	/**< triplet for non-mission direct position command */
	position_setpoint_triplet_s			_takeoff_triplet{};	/**< triplet for non-mission direct takeoff command */
	vehicle_roi_s					_vroi{};		/**< vehicle ROI */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	Geofence	_geofence;			/**< class that handles the geofence */
	bool		_geofence_violation_warning_sent{false}; /**< prevents spaming to mavlink */

	bool		_can_loiter_at_sp{false};			/**< flags if current position SP can be used to loiter */
	bool		_pos_sp_triplet_updated{false};		/**< flags if position SP triplet needs to be published */
	bool 		_pos_sp_triplet_published_invalid_once{false};	/**< flags if position SP triplet has been published once to UORB */
	bool		_mission_result_updated{false};		/**< flags if mission result has seen an update */

	NavigatorMode	*_navigation_mode{nullptr};		/**< abstract pointer to current navigation mode class */
	Mission		_mission;			/**< class that handles the missions */
	Loiter		_loiter;			/**< class that handles loiter */
	Takeoff		_takeoff;			/**< class for handling takeoff commands */
	Land		_land;			/**< class for handling land commands */
	PrecLand	_precland;			/**< class for handling precision land commands */
	RTL 		_rtl;				/**< class that handles RTL */
	RCLoss 		_rcLoss;				/**< class that handles RTL according to OBC rules (rc loss mode) */
	DataLinkLoss	_dataLinkLoss;			/**< class that handles the OBC datalink loss mode */
	EngineFailure	_engineFailure;			/**< class that handles the engine failure mode (FW only!) */
	GpsFailure	_gpsFailure;			/**< class that handles the OBC gpsfailure loss mode */
	FollowTarget	_follow_target;

	NavigatorMode *_navigation_mode_array[NAVIGATOR_MODE_ARRAY_SIZE];	/**< array of navigation modes */

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::NAV_LOITER_RAD>) _param_loiter_radius,	/**< loiter radius for fixedwing */
		(ParamFloat<px4::params::NAV_ACC_RAD>) _param_acceptance_radius,	/**< acceptance for takeoff */
		(ParamFloat<px4::params::NAV_FW_ALT_RAD>)
		_param_fw_alt_acceptance_radius,	/**< acceptance radius for fixedwing altitude */
		(ParamFloat<px4::params::NAV_FW_ALTL_RAD>)
		_param_fw_alt_lnd_acceptance_radius,	/**< acceptance radius for fixedwing altitude before landing*/
		(ParamFloat<px4::params::NAV_MC_ALT_RAD>)
		_param_mc_alt_acceptance_radius,	/**< acceptance radius for multicopter altitude */
		(ParamInt<px4::params::NAV_FORCE_VT>) _param_force_vtol,	/**< acceptance radius for multicopter altitude */
		(ParamInt<px4::params::NAV_TRAFF_AVOID>) _param_traffic_avoidance_mode,	/**< avoiding other aircraft is enabled */

		// non-navigator parameters
		// Mission (MIS_*)
		(ParamFloat<px4::params::MIS_LTRMIN_ALT>) _param_loiter_min_alt,
		(ParamFloat<px4::params::MIS_TAKEOFF_ALT>) _param_takeoff_min_alt,
		(ParamFloat<px4::params::MIS_YAW_TMT>) _param_yaw_timeout,
		(ParamFloat<px4::params::MIS_YAW_ERR>) _param_yaw_err
	)

	param_t _handle_back_trans_dec_mss{PARAM_INVALID};
	param_t _handle_reverse_delay{PARAM_INVALID};
	float _param_back_trans_dec_mss{0.f};
	float _param_reverse_delay{0.f};

	float _mission_cruising_speed_mc{-1.0f};
	float _mission_cruising_speed_fw{-1.0f};
	float _mission_throttle{-1.0f};

	// update subscriptions
	void		global_position_update();
	void		gps_position_update();
	void		home_position_update(bool force = false);
	void		local_position_update();
	void		params_update();
	void		vehicle_land_detected_update();
	void		vehicle_status_update();

	/**
	 * Publish a new position setpoint triplet for position controllers
	 */
	void		publish_position_setpoint_triplet();

	/**
	 * Publish the mission result so commander and mavlink know what is going on
	 */
	void		publish_mission_result();

	void		publish_vehicle_command_ack(const vehicle_command_s &cmd, uint8_t result);
};
