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
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#pragma once

#include <stdbool.h>
#include <nuttx/fs/fs.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <pthread.h>
#include <mavlink/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/topics/mission.h>

#include "mavlink_bridge_header.h"
#include "mavlink_orb_subscription.h"
#include "mavlink_stream.h"
#include "mavlink_messages.h"

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
	uint64_t timestamp_last_send_request;
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
	static int		start(int argc, char *argv[]);

	/**
	 * Display the mavlink status.
	 */
	void		status();

	static int stream(int argc, char *argv[]);

	static int	instance_count();

	static Mavlink *new_instance();

	static Mavlink *get_instance(unsigned instance);

	static Mavlink *get_instance_for_device(const char *device_name);

	static int	destroy_all_instances();

	static bool	instance_exists(const char *device_name, Mavlink *self);

	static void	forward_message(mavlink_message_t *msg, Mavlink *self);

	static int get_uart_fd(unsigned index);

	int get_uart_fd();

	const char *_device_name;

	enum MAVLINK_MODE {
		MAVLINK_MODE_NORMAL = 0,
		MAVLINK_MODE_CUSTOM,
		MAVLINK_MODE_CAMERA
	};

	void		set_mode(enum MAVLINK_MODE);
	enum MAVLINK_MODE		get_mode() { return _mode; }

	bool		get_hil_enabled() { return _hil_enabled; }

	bool		get_use_hil_gps() { return _use_hil_gps; }

	bool		get_flow_control_enabled() { return _flow_control_enabled; }

	bool		get_forwarding_on() { return _forwarding_on; }

	/**
	 * Handle waypoint related messages.
	 */
	void mavlink_wpm_message_handler(const mavlink_message_t *msg);

	static int start_helper(int argc, char *argv[]);

	/**
	 * Handle parameter related messages.
	 */
	void mavlink_pm_message_handler(const mavlink_channel_t chan, const mavlink_message_t *msg);

	void get_mavlink_mode_and_state(struct vehicle_status_s *status, struct position_setpoint_triplet_s *pos_sp_triplet, uint8_t *mavlink_state, uint8_t *mavlink_base_mode, uint32_t *mavlink_custom_mode);

	/**
	 * Enable / disable Hardware in the Loop simulation mode.
	 *
	 * @param hil_enabled	The new HIL enable/disable state.
	 * @return		OK if the HIL state changed, ERROR if the
	 *			requested change could not be made or was
	 *			redundant.
	 */
	int		set_hil_enabled(bool hil_enabled);

	MavlinkOrbSubscription *add_orb_subscription(const orb_id_t topic);

	int get_instance_id();

	/**
	 * Enable / disable hardware flow control.
	 *
	 * @param enabled	True if hardware flow control should be enabled
	 */
	int enable_flow_control(bool enabled);

	mavlink_channel_t get_channel();

	bool		_task_should_exit;		/**< if true, mavlink task should exit */

	int get_mavlink_fd() { return _mavlink_fd; }


	/* Functions for waiting to start transmission until message received. */
	void set_has_received_messages(bool received_messages) { _received_messages = received_messages; }
	bool get_has_received_messages() { return _received_messages; }
	void set_wait_to_transmit(bool wait) { _wait_to_transmit = wait; }
	bool get_wait_to_transmit() { return _wait_to_transmit; }
	bool should_transmit() { return (!_wait_to_transmit || (_wait_to_transmit && _received_messages)); }

protected:
	Mavlink	*next;

private:
	int _instance_id;

	int		_mavlink_fd;
	bool		_task_running;

	/* states */
	bool		_hil_enabled;		/**< Hardware In the Loop mode */
	bool		_use_hil_gps;		/**< Accept GPS HIL messages (for example from an external motion capturing system to fake indoor gps) */
	bool		_is_usb_uart;		/**< Port is USB */
	bool        _wait_to_transmit;  /**< Wait to transmit until received messages. */
	bool        _received_messages; /**< Whether we've received valid mavlink messages. */

	unsigned	_main_loop_delay;		/**< mainloop delay, depends on data rate */

	MavlinkOrbSubscription *_subscriptions;
	MavlinkStream *_streams;

	orb_advert_t	_mission_pub;
	struct mission_s mission;
	MAVLINK_MODE _mode;

	uint8_t _mavlink_wpm_comp_id;
	mavlink_channel_t _channel;

	struct mavlink_logbuffer _logbuffer;
	unsigned int _total_counter;

	pthread_t _receive_thread;

	/* Allocate storage space for waypoints */
	mavlink_wpm_storage _wpm_s;
	mavlink_wpm_storage *_wpm;

	bool _verbose;
	bool _forwarding_on;
	bool _passing_on;
	int _uart_fd;
	int _baudrate;
	int _datarate;

	/**
	 * If the queue index is not at 0, the queue sending
	 * logic will send parameters from the current index
	 * to len - 1, the end of the param list.
	 */
	unsigned int _mavlink_param_queue_index;

	bool mavlink_link_termination_allowed;

	char 	*_subscribe_to_stream;
	float	_subscribe_to_stream_rate;

	bool		_flow_control_enabled;

	struct mavlink_message_buffer {
		int write_ptr;
		int read_ptr;
		int size;
		char *data;
	};
	mavlink_message_buffer	_message_buffer;

	pthread_mutex_t		_message_buffer_mutex;

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	bool			_param_initialized;
	param_t			_param_system_id;
	param_t			_param_component_id;
	param_t			_param_system_type;
	param_t			_param_use_hil_gps;

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

	int configure_stream(const char *stream_name, const float rate);
	void configure_stream_threadsafe(const char *stream_name, const float rate);

	int message_buffer_init(int size);

	void message_buffer_destroy();

	int message_buffer_count();

	int message_buffer_is_empty();

	bool message_buffer_write(void *ptr, int size);

	int message_buffer_get_ptr(void **ptr, bool *is_part);

	void message_buffer_mark_read(int n);

	void pass_message(mavlink_message_t *msg);

	static int	mavlink_dev_ioctl(struct file *filep, int cmd, unsigned long arg);

	/**
	 * Main mavlink task.
	 */
	int		task_main(int argc, char *argv[]);

};
