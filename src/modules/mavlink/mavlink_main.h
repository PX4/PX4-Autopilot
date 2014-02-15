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
#include <uORB/topics/vehicle_vicon_position.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/position_setpoint_triplet.h>
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
#include <uORB/topics/mission.h>
#include <systemlib/param/param.h>

#include <nuttx/fs/fs.h>

 // FIXME XXX - TO BE MOVED TO XML
enum MAVLINK_WPM_STATES {
        MAVLINK_WPM_STATE_IDLE = 0,
        MAVLINK_WPM_STATE_SENDLIST,
        MAVLINK_WPM_STATE_SENDLIST_SENDWPS,
        MAVLINK_WPM_STATE_GETLIST,
        MAVLINK_WPM_STATE_GETLIST_GETWPS,
        MAVLINK_WPM_STATE_GETLIST_GOTALL,
        MAVLINK_WPM_STATE_ENUM_END
};

enum MAVLINK_WPM_CODES {
        MAVLINK_WPM_CODE_OK = 0,
        MAVLINK_WPM_CODE_ERR_WAYPOINT_ACTION_NOT_SUPPORTED,
        MAVLINK_WPM_CODE_ERR_WAYPOINT_FRAME_NOT_SUPPORTED,
        MAVLINK_WPM_CODE_ERR_WAYPOINT_OUT_OF_BOUNDS,
        MAVLINK_WPM_CODE_ERR_WAYPOINT_MAX_NUMBER_EXCEEDED,
        MAVLINK_WPM_CODE_ENUM_END
};


#define MAVLINK_WPM_MAX_WP_COUNT 255
#define MAVLINK_WPM_PROTOCOL_TIMEOUT_DEFAULT 5000000 ///< Protocol communication timeout in useconds
#define MAVLINK_WPM_SETPOINT_DELAY_DEFAULT 1000000 ///< When to send a new setpoint
#define MAVLINK_WPM_PROTOCOL_DELAY_DEFAULT 40000


struct mavlink_wpm_storage {
	uint16_t size;
	uint16_t max_size;
	enum MAVLINK_WPM_STATES current_state;
	int16_t current_wp_id;	///< Waypoint in current transmission
	uint16_t current_count;
	uint8_t current_partner_sysid;
	uint8_t current_partner_compid;
	uint64_t timestamp_lastaction;
	uint64_t timestamp_last_send_setpoint;
	uint32_t timeout;
	int current_dataman_id;
};


class Mavlink
{

public:
	/**
	 * Constructor
	 */
	Mavlink();

	/**
	 * Destructor, also kills the mavlinks task.
	 */
	~Mavlink();

	/**
	* Start the mavlink task.
	 *
	 * @return		OK on success.
	 */
	static int		start();

	/**
	 * Display the mavlink status.
	 */
	void		status();

	static int	instance_count();

	static Mavlink* new_instance();

	static Mavlink* get_instance(unsigned instance);

	static int	destroy_all_instances();

	static bool	instance_exists(const char *device_name, Mavlink *self);

	static int get_uart_fd(unsigned index);

	int get_uart_fd();

	int get_channel();

	const char *device_name;

	enum MAVLINK_MODE {
		MODE_TX_HEARTBEAT_ONLY=0,
		MODE_OFFBOARD,
		MODE_ONBOARD,
		MODE_HIL
	};

	void		set_mode(enum MAVLINK_MODE);
	enum MAVLINK_MODE		get_mode() { return _mode; }

	bool		get_hil_enabled() { return _mavlink_hil_enabled; };

	/**
	 * Handle waypoint related messages.
	 */
	void mavlink_wpm_message_handler(const mavlink_message_t *msg);

	static int start_helper(int argc, char *argv[]);

	/**
	 * Handle parameter related messages.
	 */
	void mavlink_pm_message_handler(const mavlink_channel_t chan, const mavlink_message_t *msg);

	void get_mavlink_mode_and_state(uint8_t *mavlink_state, uint8_t *mavlink_base_mode, uint32_t *mavlink_custom_mode);

	/**
	 * Enable / disable Hardware in the Loop simulation mode.
	 *
	 * @param hil_enabled	The new HIL enable/disable state.
	 * @return		OK if the HIL state changed, ERROR if the
	 *			requested change could not be made or was
	 *			redundant.
	 */
	int		set_hil_enabled(bool hil_enabled);

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
		int get_sub;
		int airspeed_sub;
		int navigation_capabilities_sub;
		int position_setpoint_triplet_sub;
		int rc_sub;
		int status_sub;
		int home_sub;
	};

	struct mavlink_subscriptions subs;

	struct mavlink_subscriptions* get_subs() { return &subs; }
	mavlink_channel_t get_chan() { return _chan; }

	/** Global position */
	struct vehicle_global_position_s global_pos;
	/** Local position */
	struct vehicle_local_position_s local_pos;
	/** navigation capabilities */
	struct navigation_capabilities_s nav_cap;
	/** Vehicle status */
	struct vehicle_status_s v_status;
	/** RC channels */
	struct rc_channels_s rc;
	/** Actuator armed state */
	struct actuator_armed_s armed;
	/** Position setpoint triplet */
	struct position_setpoint_triplet_s pos_sp_triplet;

	bool		_task_should_exit;		/**< if true, mavlink task should exit */

protected:
	/**
	 * Pointer to the default cdev file operations table; useful for
	 * registering clone devices etc.
	 */

	Mavlink*	_next;

private:
	int		_mavlink_fd;
	bool		thread_running;
	int		_mavlink_task;			/**< task handle for sensor task */

	int		_mavlink_incoming_fd;		/**< file descriptor on which to receive incoming strings */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	/* states */
	bool		_mavlink_hil_enabled;		/**< Hardware in the loop mode */

	int		_params_sub;

	orb_advert_t	mission_pub;
	struct mission_s mission;
	uint8_t missionlib_msg_buf[300]; //XXX MAGIC NUMBER
	MAVLINK_MODE _mode;

	uint8_t _mavlink_wpm_comp_id;
	mavlink_channel_t _chan;

//  XXX probably should be in a header... 
// extern pthread_t receive_start(int uart);

	struct mavlink_logbuffer lb;
	unsigned int total_counter;

	pthread_t receive_thread;
	pthread_t uorb_receive_thread;

	/* Allocate storage space for waypoints */
	mavlink_wpm_storage wpm_s;
	mavlink_wpm_storage *wpm;

	bool _verbose;
	int _uart;
	int _baudrate;
	bool gcs_link;
	/**
	 * If the queue index is not at 0, the queue sending
	 * logic will send parameters from the current index
	 * to len - 1, the end of the param list.
	 */
	unsigned int mavlink_param_queue_index;

	bool mavlink_link_termination_allowed;
	
	/**
	 * Update our local parameter cache.
	 */
	void		parameters_update();

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
	void mavlink_pm_start_queued_send();

	void mavlink_update_system();

	void mavlink_waypoint_eventloop(uint64_t now);
	void mavlink_wpm_send_waypoint_reached(uint16_t seq);
	void mavlink_wpm_send_waypoint_request(uint8_t sysid, uint8_t compid, uint16_t seq);
	void mavlink_wpm_send_waypoint(uint8_t sysid, uint8_t compid, uint16_t seq);
	void mavlink_wpm_send_waypoint_count(uint8_t sysid, uint8_t compid, uint16_t count);
	void mavlink_wpm_send_waypoint_current(uint16_t seq);
	void mavlink_wpm_send_waypoint_ack(uint8_t sysid, uint8_t compid, uint8_t type);
	void mavlink_wpm_init(mavlink_wpm_storage *state);
	int map_mission_item_to_mavlink_mission_item(const struct mission_item_s *mission_item, mavlink_mission_item_t *mavlink_mission_item);
	int map_mavlink_mission_item_to_mission_item(const mavlink_mission_item_t *mavlink_mission_item, struct mission_item_s *mission_item);
	void publish_mission();

	void mavlink_missionlib_send_message(mavlink_message_t *msg);
	int mavlink_missionlib_send_gcs_string(const char *string);

	int mavlink_open_uart(int baudrate, const char *uart_name, struct termios *uart_config_original, bool *is_usb);

	int set_mavlink_interval_limit(int mavlink_msg_id, int min_interval);

	/**
	 * Callback for param interface.
	 */
	static void mavlink_pm_callback(void *arg, param_t param);

	static int	mavlink_dev_ioctl(struct file *filep, int cmd, unsigned long arg);

	/**
	 * Main mavlink task.
	 */
	int		task_main(int argc, char *argv[]) __attribute__((noreturn));

};
