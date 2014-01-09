/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file mavlink_main.h
 * MAVLink 1.0 protocol interface definition.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#pragma once

#include <stdbool.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/offboard_control_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/mission_item_triplet.h>
#include <uORB/topics/vehicle_vicon_position.h>
#include <uORB/topics/mission_item_triplet.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_controls_effective.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <drivers/drv_rc_input.h>
#include <uORB/topics/navigation_capabilities.h>

class Mavlink
{
public:
	/**
	 * Constructor
	 */
	Mavlink(const char *port, unsigned baud_rate);

	/**
	 * Destructor, also kills the mavlinks task.
	 */
	~Mavlink();

	/**
	* Start the mavlink task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	/**
	 * Display the mavlink status.
	 */
	void		status();

	static int	instance_count();

	static Mavlink* new_instance(const char *port, unsigned baud_rate);

	static Mavlink* get_instance(unsigned instance);

	struct mavlink_subscriptions {
		int sensor_sub;
		int att_sub;
		int global_pos_sub;
		int act_0_sub;
		int act_1_sub;
		int act_2_sub;
		int act_3_sub;
		int gps_sub;
		int man_control_sp_sub;
		int safety_sub;
		int actuators_sub;
		int armed_sub;
		int actuators_effective_sub;
		int local_pos_sub;
		int spa_sub;
		int spl_sub;
		int triplet_sub;
		int debug_key_value;
		int input_rc_sub;
		int optical_flow;
		int rates_setpoint_sub;
		int home_sub;
		int airspeed_sub;
		int navigation_capabilities_sub;
		int control_mode_sub;
	};

	struct mavlink_subscriptions subs;

	/** Global position */
	struct vehicle_global_position_s global_pos;
	/** Local position */
	struct vehicle_local_position_s local_pos;
	/** navigation capabilities */
	struct navigation_capabilities_s nav_cap;
	/** Vehicle status */
	struct vehicle_status_s v_status;
	/** Vehicle control mode */
	struct vehicle_control_mode_s control_mode;
	/** RC channels */
	struct rc_channels_s rc;
	/** Actuator armed state */
	struct actuator_armed_s armed;

private:

	bool		_task_should_exit;		/**< if true, sensor task should exit */
	int		_mavlink_task;			/**< task handle for sensor task */

	int		_mavlink_fd;
	int		_mavlink_incoming_fd;		/**< file descriptor on which to receive incoming strings */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	/* states */
	bool		_mavlink_hil_enabled;		/**< Hardware in the loop mode */
	static Mavlink*	_head;

	int		_params_sub;

	orb_advert_t mission_pub = -1;
struct mission_s mission;

uint8_t mavlink_wpm_comp_id = MAV_COMP_ID_MISSIONPLANNER;
	
	/**
	 * Update our local parameter cache.
	 */
	void		parameters_update();

	/**
	 * Enable / disable Hardware in the Loop simulation mode.
	 *
	 * @param hil_enabled	The new HIL enable/disable state.
	 * @return		OK if the HIL state changed, ERROR if the
	 *			requested change could not be made or was
	 *			redundant.
	 */
	int		set_hil_on_off(bool hil_enabled);


	/**
	 * Handle parameter related messages.
	 */
	void mavlink_pm_message_handler(const mavlink_channel_t chan, const mavlink_message_t *msg);

	/**
	 * Send all parameters at once.
	 *
	 * This function blocks until all parameters have been sent.
	 * it delays each parameter by the passed amount of microseconds.
	 *
	 * @param delay		The delay in us between sending all parameters.
	 */
	void mavlink_pm_send_all_params(unsigned int delay);

	/**
	 * Send one parameter.
	 *
	 * @param param		The parameter id to send.
	 * @return		zero on success, nonzero on failure.
	 */
	int mavlink_pm_send_param(param_t param);

	/**
	 * Send one parameter identified by index.
	 *
	 * @param index		The index of the parameter to send.
	 * @return		zero on success, nonzero else.
	 */
	int mavlink_pm_send_param_for_index(uint16_t index);

	/**
	 * Send one parameter identified by name.
	 *
	 * @param name		The index of the parameter to send.
	 * @return		zero on success, nonzero else.
	 */
	int mavlink_pm_send_param_for_name(const char *name);

	/**
	 * Send a queue of parameters, one parameter per function call.
	 *
	 * @return		zero on success, nonzero on failure
	 */
	int mavlink_pm_queued_send(void);

	/**
	 * Start sending the parameter queue.
	 *
	 * This function will not directly send parameters, but instead
	 * activate the sending of one parameter on each call of
	 * mavlink_pm_queued_send().
	 * @see 		mavlink_pm_queued_send()
	 */
	void mavlink_pm_start_queued_send(void);

	/**
	 * Shim for calling task_main from task_create.
	 */
	void		task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main() __attribute__((noreturn));

};
