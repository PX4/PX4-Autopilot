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
 */

#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <systemlib/perf_counter.h>

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/parameter_update.h>

#include "navigator_mode.h"
#include "mission.h"
#include "loiter.h"
#include "rtl.h"
#include "geofence.h"

/**
 * Number of navigation modes that need on_active/on_inactive calls
 * Currently: mission, loiter, and rtl
 */
#define NAVIGATOR_MODE_ARRAY_SIZE 3

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
	 * Setters
	 */
	void		set_can_loiter_at_sp(bool can_loiter) { _can_loiter_at_sp = can_loiter; }

	/**
	 * Getters
	 */
	struct vehicle_status_s*	    get_vstatus() { return &_vstatus; }
	struct vehicle_global_position_s*   get_global_position() { return &_global_pos; }
	struct home_position_s*		    get_home_position() { return &_home_pos; }
	int		get_onboard_mission_sub() { return _onboard_mission_sub; }
	int		get_offboard_mission_sub() { return _offboard_mission_sub; }
	Geofence&	get_geofence() { return _geofence; }
	bool		get_can_loiter_at_sp() { return _can_loiter_at_sp; }
	float		get_loiter_radius() { return _param_loiter_radius.get(); }
	float		get_acceptance_radius() { return _param_acceptance_radius.get(); }
	int		get_mavlink_fd() { return _mavlink_fd; }

private:

	bool		_task_should_exit;		/**< if true, sensor task should exit */
	int		_navigator_task;		/**< task handle for sensor task */

	int		_mavlink_fd;			/**< the file descriptor to send messages over mavlink */

	int		_global_pos_sub;		/**< global position subscription */
	int		_home_pos_sub;			/**< home position subscription */
	int		_vstatus_sub;			/**< vehicle status subscription */
	int		_capabilities_sub;		/**< notification of vehicle capabilities updates */
	int		_control_mode_sub;		/**< vehicle control mode subscription */
	int		_onboard_mission_sub;		/**< onboard mission subscription */
	int		_offboard_mission_sub;		/**< offboard mission subscription */
	int		_param_update_sub;		/**< param update subscription */

	orb_advert_t	_pos_sp_triplet_pub;		/**< publish position setpoint triplet */

	vehicle_status_s				_vstatus;		/**< vehicle status */
	vehicle_control_mode_s				_control_mode;		/**< vehicle control mode */
	vehicle_global_position_s			_global_pos;		/**< global vehicle position */
	home_position_s					_home_pos;		/**< home position for RTL */
	mission_item_s 					_mission_item;		/**< current mission item */
	navigation_capabilities_s			_nav_caps;		/**< navigation capabilities */
	position_setpoint_triplet_s			_pos_sp_triplet;	/**< triplet of position setpoints */

	bool 		_mission_item_valid;		/**< flags if the current mission item is valid */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	Geofence	_geofence;			/**< class that handles the geofence */
	bool		_geofence_violation_warning_sent; /**< prevents spaming to mavlink */

	bool		_fence_valid;			/**< flag if fence is valid */
	bool		_inside_fence;			/**< vehicle is inside fence */

	NavigatorMode	*_navigation_mode;		/**< abstract pointer to current navigation mode class */
	Mission		_mission;			/**< class that handles the missions */
	Loiter		_loiter;			/**< class that handles loiter */
	RTL 		_rtl;				/**< class that handles RTL */

	NavigatorMode *_navigation_mode_array[NAVIGATOR_MODE_ARRAY_SIZE];	/**< array of navigation modes */

	bool		_can_loiter_at_sp;			/**< flags if current position SP can be used to loiter */
	bool		_update_triplet;		/**< flags if position SP triplet needs to be published */

	control::BlockParamFloat _param_loiter_radius;	/**< loiter radius for fixedwing */
	control::BlockParamFloat _param_acceptance_radius;	/**< acceptance for takeoff */
	/**
	 * Retrieve global position
	 */
	void		global_position_update();

	/**
	 * Retrieve home position
	 */
	void		home_position_update();

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
};
#endif
