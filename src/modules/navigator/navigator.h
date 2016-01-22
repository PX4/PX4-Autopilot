/***************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
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

#include <systemlib/perf_counter.h>

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>
#include <navigator/navigation.h>

#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>

#include "navigator_mode.h"
#include "mission.h"
#include "loiter.h"
#include "takeoff.h"
#include "land.h"
#include "rtl.h"
#include "datalinkloss.h"
#include "enginefailure.h"
#include "gpsfailure.h"
#include "rcloss.h"
#include "geofence.h"

/**
 * Number of navigation modes that need on_active/on_inactive calls
 */
#define NAVIGATOR_MODE_ARRAY_SIZE 9

class Navigator : public control::SuperBlock
{
public:
	/**
	 * Constructor
	 */
	Navigator();

	/**
	 * Destructor, also kills the navigators task.
	 */
	~Navigator();

	/**
	 * Start the navigator task.
	 *
	 * @return		OK on success.
	 */
	int		start();

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
	void publish_geofence_result();

	/**
	 * Publish the attitude sp, only to be used in very special modes when position control is deactivated
	 * Example: mode that is triggered on gps failure
	 */
	void publish_att_sp();

	/**
	 * Setters
	 */
	void		set_can_loiter_at_sp(bool can_loiter) { _can_loiter_at_sp = can_loiter; }
	void		set_position_setpoint_triplet_updated() { _pos_sp_triplet_updated = true; }
	void		set_mission_result_updated() { _mission_result_updated = true; }

	/**
	 * Getters
	 */
	struct vehicle_status_s*	    get_vstatus() { return &_vstatus; }
	struct vehicle_control_mode_s*	    get_control_mode() { return &_control_mode; }
	struct vehicle_global_position_s*   get_global_position() { return &_global_pos; }
	struct vehicle_gps_position_s*	    get_gps_position() { return &_gps_pos; }
	struct sensor_combined_s*	    get_sensor_combined() { return &_sensor_combined; }
	struct home_position_s*		    get_home_position() { return &_home_pos; }
	bool				    home_position_valid() { return (_home_pos.timestamp > 0); }
	struct position_setpoint_triplet_s* get_position_setpoint_triplet() { return &_pos_sp_triplet; }
	struct mission_result_s*	    get_mission_result() { return &_mission_result; }
	struct geofence_result_s*		    get_geofence_result() { return &_geofence_result; }
	struct vehicle_attitude_setpoint_s* get_att_sp() { return &_att_sp; }

	int		get_onboard_mission_sub() { return _onboard_mission_sub; }
	int		get_offboard_mission_sub() { return _offboard_mission_sub; }
	Geofence&	get_geofence() { return _geofence; }
	bool		get_can_loiter_at_sp() { return _can_loiter_at_sp; }
	float		get_loiter_radius() { return _param_loiter_radius.get(); }

	/**
	 * Get the acceptance radius
	 *
	 * @return the distance at which the next waypoint should be used
	 */
	float		get_acceptance_radius();

	/**
	 * Get the cruising speed
	 *
	 * @return the desired cruising speed for this mission
	 */
	float		get_cruising_speed();

	/**
	 * Set the cruising speed
	 */
	void		set_cruising_speed(float speed) { _mission_cruising_speed = speed; }

	/**
	 * Get the acceptance radius given the mission item preset radius
	 *
	 * @param mission_item_radius the radius to use in case the controller-derived radius is smaller
	 *
	 * @return the distance at which the next waypoint should be used
	 */
	float		get_acceptance_radius(float mission_item_radius);
	int		get_mavlink_fd() { return _mavlink_fd; }

	void		increment_mission_instance_count() { _mission_instance_count++; }

	void 		set_mission_failure(const char *reason);

private:

	bool		_task_should_exit;		/**< if true, sensor task should exit */
	int		_navigator_task;		/**< task handle for sensor task */

	int		_mavlink_fd;			/**< the file descriptor to send messages over mavlink */

	int		_global_pos_sub;		/**< global position subscription */
	int		_gps_pos_sub;		/**< gps position subscription */
	int		_sensor_combined_sub;		/**< sensor combined subscription */
	int		_home_pos_sub;			/**< home position subscription */
	int		_vstatus_sub;			/**< vehicle status subscription */
	int		_capabilities_sub;		/**< notification of vehicle capabilities updates */
	int		_control_mode_sub;		/**< vehicle control mode subscription */
	int		_onboard_mission_sub;		/**< onboard mission subscription */
	int		_offboard_mission_sub;		/**< offboard mission subscription */
	int		_param_update_sub;		/**< param update subscription */
	int		_vehicle_command_sub;		/**< vehicle commands (onboard and offboard) */

	orb_advert_t	_pos_sp_triplet_pub;		/**< publish position setpoint triplet */
	orb_advert_t	_mission_result_pub;
	orb_advert_t	_geofence_result_pub;
	orb_advert_t	_att_sp_pub;			/**< publish att sp
							  used only in very special failsafe modes
							  when pos control is deactivated */

	vehicle_status_s				_vstatus;		/**< vehicle status */
	vehicle_control_mode_s				_control_mode;		/**< vehicle control mode */
	vehicle_global_position_s			_global_pos;		/**< global vehicle position */
	vehicle_gps_position_s				_gps_pos;		/**< gps position */
	sensor_combined_s				_sensor_combined;	/**< sensor values */
	home_position_s					_home_pos;		/**< home position for RTL */
	mission_item_s 					_mission_item;		/**< current mission item */
	navigation_capabilities_s			_nav_caps;		/**< navigation capabilities */
	position_setpoint_triplet_s			_pos_sp_triplet;	/**< triplet of position setpoints */

	mission_result_s				_mission_result;
	geofence_result_s				_geofence_result;
	vehicle_attitude_setpoint_s			_att_sp;

	bool 		_mission_item_valid;		/**< flags if the current mission item is valid */
	int		_mission_instance_count;	/**< instance count for the current mission */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	Geofence	_geofence;			/**< class that handles the geofence */
	bool		_geofence_violation_warning_sent; /**< prevents spaming to mavlink */

	bool		_inside_fence;			/**< vehicle is inside fence */

	bool		_can_loiter_at_sp;			/**< flags if current position SP can be used to loiter */
	bool		_pos_sp_triplet_updated;		/**< flags if position SP triplet needs to be published */
	bool 		_pos_sp_triplet_published_invalid_once;	/**< flags if position SP triplet has been published once to UORB */
	bool		_mission_result_updated;		/**< flags if mission result has seen an update */

	NavigatorMode	*_navigation_mode;		/**< abstract pointer to current navigation mode class */
	Mission		_mission;			/**< class that handles the missions */
	Loiter		_loiter;			/**< class that handles loiter */
	Takeoff		_takeoff;			/**< class for handling takeoff commands */
	Land		_land;			/**< class for handling land commands */
	RTL 		_rtl;				/**< class that handles RTL */
	RCLoss 		_rcLoss;				/**< class that handles RTL according to
							  OBC rules (rc loss mode) */
	DataLinkLoss	_dataLinkLoss;			/**< class that handles the OBC datalink loss mode */
	EngineFailure	_engineFailure;			/**< class that handles the engine failure mode
							  (FW only!) */
	GpsFailure	_gpsFailure;			/**< class that handles the OBC gpsfailure loss mode */

	NavigatorMode *_navigation_mode_array[NAVIGATOR_MODE_ARRAY_SIZE];	/**< array of navigation modes */

	control::BlockParamFloat _param_loiter_radius;	/**< loiter radius for fixedwing */
	control::BlockParamFloat _param_acceptance_radius;	/**< acceptance for takeoff */
	control::BlockParamInt _param_datalinkloss_obc;	/**< if true: obc mode on data link loss enabled */
	control::BlockParamInt _param_rcloss_obc;	/**< if true: obc mode on rc loss enabled */
	
	control::BlockParamFloat _param_cruising_speed_hover;
	control::BlockParamFloat _param_cruising_speed_plane;

	float _mission_cruising_speed;

	/**
	 * Retrieve global position
	 */
	void		global_position_update();

	/**
	 * Retrieve gps position
	 */
	void		gps_position_update();

	/**
	 * Retrieve sensor values
	 */
	void		sensor_combined_update();

	/**
	 * Retrieve home position
	 */
	void		home_position_update(bool force=false);

	/**
	 * Retreive navigation capabilities
	 */
	void		navigation_capabilities_update();

	/**
	 * Retrieve vehicle status
	 */
	void		vehicle_status_update();

	/**
	 * Retrieve vehicle control mode
	 */
	void		vehicle_control_mode_update();

	/**
	 * Update parameters
	 */
	void		params_update();

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

	/* this class has ptr data members, so it should not be copied,
	 * consequently the copy constructors are private.
	 */
	Navigator(const Navigator&);
	Navigator operator=(const Navigator&);
};
#endif
