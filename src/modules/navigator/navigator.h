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
 */

#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include "datalinkloss.h"
#include "enginefailure.h"
#include "follow_target.h"
#include "geofence.h"
#include "gpsfailure.h"
#include "land.h"
#include "loiter.h"
#include "mission.h"
#include "navigator_mode.h"
#include "rcloss.h"
#include "rtl.h"
#include "takeoff.h"

#include <controllib/block/BlockParam.hpp>
#include <controllib/blocks.hpp>
#include <navigator/navigation.h>
#include <systemlib/perf_counter.h>
#include <uORB/topics/fw_pos_ctrl_status.h>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/uORB.h>

/**
 * Number of navigation modes that need on_active/on_inactive calls
 */
#define NAVIGATOR_MODE_ARRAY_SIZE 10

class Navigator : public control::SuperBlock
{
public:
	Navigator();
	~Navigator();
	Navigator(const Navigator &) = delete;
	Navigator operator=(const Navigator &) = delete;

	/**
	 * Start the navigator task.
	 *
	 * @return		OK on success.
	 */
	int			start();

	/**
	 * Display the navigator status.
	 */
	void		status();

	/**
	 * Add point to geofence
	 */
	void		add_fence_point(int argc, char *argv[]);

	/**
	 * Load fence from file
	 */
	void		load_fence_from_file(const char *filename);

	/**
	 * Publish the geofence result
	 */
	void		publish_geofence_result();

	/**
	 * Publish the attitude sp, only to be used in very special modes when position control is deactivated
	 * Example: mode that is triggered on gps failure
	 */
	void		publish_att_sp();

	void		publish_vehicle_cmd(const struct vehicle_command_s &vcmd);

	/**
	 * Setters
	 */
	void		set_can_loiter_at_sp(bool can_loiter) { _can_loiter_at_sp = can_loiter; }
	void		set_position_setpoint_triplet_updated() { _pos_sp_triplet_updated = true; }
	void		set_mission_result_updated() { _mission_result_updated = true; }

	/**
	 * Getters
	 */
	struct fw_pos_ctrl_status_s *get_fw_pos_ctrl_status() { return &_fw_pos_ctrl_status; }
	struct home_position_s *get_home_position() { return &_home_pos; }
	struct mission_result_s *get_mission_result() { return &_mission_result; }
	struct position_setpoint_triplet_s *get_position_setpoint_triplet() { return &_pos_sp_triplet; }
	struct position_setpoint_triplet_s *get_reposition_triplet() { return &_reposition_triplet; }
	struct position_setpoint_triplet_s *get_takeoff_triplet() { return &_takeoff_triplet; }
	struct vehicle_attitude_setpoint_s *get_att_sp() { return &_att_sp; }
	struct vehicle_global_position_s *get_global_position() { return &_global_pos; }
	struct vehicle_gps_position_s *get_gps_position() { return &_gps_pos; }
	struct vehicle_land_detected_s *get_land_detected() { return &_land_detected; }
	struct vehicle_local_position_s *get_local_position() { return &_local_pos; }
	struct vehicle_status_s *get_vstatus() { return &_vstatus; }

	bool home_position_valid() { return (_home_pos.timestamp > 0); }

	int		get_onboard_mission_sub() { return _onboard_mission_sub; }
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
	orb_advert_t	*get_mavlink_log_pub() { return &_mavlink_log_pub; }

	void		increment_mission_instance_count() { _mission_instance_count++; }

	void 		set_mission_failure(const char *reason);

	bool		is_planned_mission() { return _navigation_mode == &_mission; }

	bool		abort_landing();

private:

	bool		_task_should_exit{false};	/**< if true, sensor task should exit */
	int		_navigator_task{-1};		/**< task handle for sensor task */

	orb_advert_t	_mavlink_log_pub{nullptr};	/**< the uORB advert to send messages over mavlink */

	int		_fw_pos_ctrl_status_sub{-1};	/**< notification of vehicle capabilities updates */
	int		_global_pos_sub{-1};		/**< global position subscription */
	int		_gps_pos_sub{-1};		/**< gps position subscription */
	int		_home_pos_sub{-1};		/**< home position subscription */
	int		_land_detected_sub{-1};		/**< vehicle land detected subscription */
	int		_local_pos_sub{-1};		/**< local position subscription */
	int		_offboard_mission_sub{-1};	/**< offboard mission subscription */
	int		_onboard_mission_sub{-1};	/**< onboard mission subscription */
	int		_param_update_sub{-1};		/**< param update subscription */
	int		_sensor_combined_sub{-1};	/**< sensor combined subscription */
	int		_vehicle_command_sub{-1};	/**< vehicle commands (onboard and offboard) */
	int		_vstatus_sub{-1};		/**< vehicle status subscription */

	orb_advert_t	_att_sp_pub{nullptr};
	orb_advert_t	_geofence_result_pub{nullptr};
	orb_advert_t	_mission_result_pub{nullptr};
	orb_advert_t	_pos_sp_triplet_pub{nullptr};
	orb_advert_t	_vehicle_cmd_pub{nullptr};

	fw_pos_ctrl_status_s				_fw_pos_ctrl_status{};	/**< fixed wing navigation capabilities */
	geofence_result_s				_geofence_result{};
	home_position_s					_home_pos{};		/**< home position for RTL */
	mission_result_s				_mission_result{};
	position_setpoint_triplet_s			_pos_sp_triplet{};	/**< triplet of position setpoints */
	position_setpoint_triplet_s			_reposition_triplet{};	/**< triplet for non-mission direct position command */
	position_setpoint_triplet_s			_takeoff_triplet{};	/**< triplet for non-mission direct takeoff command */
	sensor_combined_s				_sensor_combined{};	/**< sensor values */
	vehicle_attitude_setpoint_s			_att_sp{};
	vehicle_global_position_s			_global_pos{};		/**< global vehicle position */
	vehicle_gps_position_s				_gps_pos{};		/**< gps position */
	vehicle_land_detected_s				_land_detected{};	/**< vehicle land_detected */
	vehicle_local_position_s			_local_pos;		/**< local vehicle position */
	vehicle_status_s				_vstatus{};		/**< vehicle status */

	int		_mission_instance_count{-1};	/**< instance count for the current mission */

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
	RTL 		_rtl;				/**< class that handles RTL */
	RCLoss 		_rcLoss;				/**< class that handles RTL according to OBC rules (rc loss mode) */
	DataLinkLoss	_dataLinkLoss;			/**< class that handles the OBC datalink loss mode */
	EngineFailure	_engineFailure;			/**< class that handles the engine failure mode (FW only!) */
	GpsFailure	_gpsFailure;			/**< class that handles the OBC gpsfailure loss mode */
	FollowTarget	_follow_target;

	NavigatorMode *_navigation_mode_array[NAVIGATOR_MODE_ARRAY_SIZE];	/**< array of navigation modes */

	control::BlockParamFloat _param_loiter_radius;	/**< loiter radius for fixedwing */
	control::BlockParamFloat _param_acceptance_radius;	/**< acceptance for takeoff */
	control::BlockParamFloat _param_fw_alt_acceptance_radius;	/**< acceptance radius for fixedwing altitude */
	control::BlockParamFloat _param_mc_alt_acceptance_radius;	/**< acceptance radius for multicopter altitude */

	control::BlockParamFloat _param_cruising_speed_hover;
	control::BlockParamFloat _param_cruising_speed_plane;
	control::BlockParamFloat _param_cruising_throttle_plane;

	float _mission_cruising_speed_mc{-1.0f};
	float _mission_cruising_speed_fw{-1.0f};
	float _mission_throttle{-1.0f};

	// update subscriptions
	void		fw_pos_ctrl_status_update(bool force = false);
	void		global_position_update();
	void		gps_position_update();
	void		home_position_update(bool force = false);
	void		local_position_update();
	void		params_update();
	void		sensor_combined_update();
	void		vehicle_land_detected_update();
	void		vehicle_status_update();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main task.
	 */
	void		task_main();

	/**
	 * Translate mission item to a position setpoint.
	 */
	void		mission_item_to_position_setpoint(const mission_item_s *item, position_setpoint_s *sp);

	/**
	 * Publish a new position setpoint triplet for position controllers
	 */
	void		publish_position_setpoint_triplet();

	/**
	 * Publish the mission result so commander and mavlink know what is going on
	 */
	void		publish_mission_result();
};
#endif
