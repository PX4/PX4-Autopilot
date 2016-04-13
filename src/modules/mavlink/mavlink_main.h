/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
#ifdef __PX4_NUTTX
#include <nuttx/fs/fs.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <drivers/device/device.h>
#endif
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <pthread.h>
#include <systemlib/mavlink_log.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/telemetry_status.h>

#include "mavlink_bridge_header.h"
#include "mavlink_orb_subscription.h"
#include "mavlink_stream.h"
#include "mavlink_messages.h"
#include "mavlink_mission.h"
#include "mavlink_parameters.h"
#include "mavlink_ftp.h"
#include "mavlink_log_handler.h"

enum Protocol {
	SERIAL = 0,
	UDP,
	TCP,
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
	void			display_status();

	static int		stream_command(int argc, char *argv[]);

	static int		instance_count();

	static Mavlink		*new_instance();

	static Mavlink		*get_instance(unsigned instance);

	static Mavlink 		*get_instance_for_device(const char *device_name);

	static Mavlink 		*get_instance_for_network_port(unsigned long port);

	static int		destroy_all_instances();

	static int		get_status_all_instances();

	static bool		instance_exists(const char *device_name, Mavlink *self);

	static void		forward_message(const mavlink_message_t *msg, Mavlink *self);

	static int		get_uart_fd(unsigned index);

	int			get_uart_fd();

	/**
	 * Get the MAVLink system id.
	 *
	 * @return		The system ID of this vehicle
	 */
	int			get_system_id();

	/**
	 * Get the MAVLink component id.
	 *
	 * @return		The component ID of this vehicle
	 */
	int			get_component_id();

	const char *_device_name;

	enum MAVLINK_MODE {
		MAVLINK_MODE_NORMAL = 0,
		MAVLINK_MODE_CUSTOM,
		MAVLINK_MODE_ONBOARD,
		MAVLINK_MODE_OSD,
		MAVLINK_MODE_MAGIC,
		MAVLINK_MODE_CONFIG
	};

	void			set_mode(enum MAVLINK_MODE);
	enum MAVLINK_MODE	get_mode() { return _mode; }

	bool			get_hil_enabled() { return _hil_enabled; }

	bool			get_use_hil_gps() { return _use_hil_gps; }

	bool			get_forward_externalsp() { return _forward_externalsp; }

	bool			get_flow_control_enabled() { return _flow_control_enabled; }

	bool			get_forwarding_on() { return _forwarding_on; }

	/**
	 * Set the boot complete flag on all instances
	 *
	 * Setting the flag unblocks parameter transmissions, which are gated
	 * beforehand to ensure that the system is fully initialized.
	 */
	static void		set_boot_complete() { _boot_complete = true; }

	/**
	 * Get the free space in the transmit buffer
	 *
	 * @return free space in the UART TX buffer
	 */
	unsigned		get_free_tx_buf();

	static int		start_helper(int argc, char *argv[]);

	/**
	 * Handle parameter related messages.
	 */
	void			mavlink_pm_message_handler(const mavlink_channel_t chan, const mavlink_message_t *msg);

	void			get_mavlink_mode_and_state(struct vehicle_status_s *status, struct position_setpoint_triplet_s *pos_sp_triplet, uint8_t *mavlink_state, uint8_t *mavlink_base_mode, uint32_t *mavlink_custom_mode);

	/**
	 * Enable / disable Hardware in the Loop simulation mode.
	 *
	 * @param hil_enabled	The new HIL enable/disable state.
	 * @return		OK if the HIL state changed, ERROR if the
	 *			requested change could not be made or was
	 *			redundant.
	 */
	int			set_hil_enabled(bool hil_enabled);

	/**
	 * Set manual input generation mode
	 *
	 * Set to true to generate RC_INPUT messages on the system bus from
	 * MAVLink messages.
	 *
	 * @param generation_enabled If set to true, generate RC_INPUT messages
	 */
	void			set_manual_input_mode_generation(bool generation_enabled) { _generate_rc = generation_enabled; }

	/**
	 * Set communication protocol for this mavlink instance
	 */
	void 		set_protocol(Protocol p) {_protocol = p;};

	/**
	 * Get the manual input generation mode
	 *
	 * @return true if manual inputs should generate RC data
	 */
	bool			get_manual_input_mode_generation() { return _generate_rc; }

	void			send_message(const uint8_t msgid, const void *msg, uint8_t component_ID = 0);

	/**
	 * Resend message as is, don't change sequence number and CRC.
	 */
	void			resend_message(mavlink_message_t *msg);

	void			handle_message(const mavlink_message_t *msg);

	MavlinkOrbSubscription *add_orb_subscription(const orb_id_t topic, int instance=0);

	int			get_instance_id();

#ifndef __PX4_QURT
	/**
	 * Enable / disable hardware flow control.
	 *
	 * @param enabled	True if hardware flow control should be enabled
	 */
	int			enable_flow_control(bool enabled);
#endif

	mavlink_channel_t	get_channel();

	void			configure_stream_threadsafe(const char *stream_name, float rate);

	bool			_task_should_exit;	/**< if true, mavlink task should exit */

	orb_advert_t		*get_mavlink_log_pub() { return &_mavlink_log_pub; }

	/**
	 * Send a status text with loglevel INFO
	 *
	 * @param string the message to send (will be capped by mavlink max string length)
	 */
	void			send_statustext_info(const char *string);

	/**
	 * Send a status text with loglevel CRITICAL
	 *
	 * @param string the message to send (will be capped by mavlink max string length)
	 */
	void			send_statustext_critical(const char *string);

	/**
	 * Send a status text with loglevel EMERGENCY
	 *
	 * @param string the message to send (will be capped by mavlink max string length)
	 */
	void			send_statustext_emergency(const char *string);

	/**
	 * Send a status text with loglevel, the difference from mavlink_log_xxx() is that message sent
	 * only on this mavlink connection. Useful for reporting communication specific, not system-wide info
	 * only to client interested in it. Message will be not sent immediately but queued in buffer as
	 * for mavlink_log_xxx().
	 *
	 * @param string the message to send (will be capped by mavlink max string length)
	 * @param severity the log level
	 */
	void			send_statustext(unsigned char severity, const char *string);
	void 			send_autopilot_capabilites();

	MavlinkStream *		get_streams() const { return _streams; }

	float			get_rate_mult();

	float			get_baudrate() { return _baudrate; }

	/* Functions for waiting to start transmission until message received. */
	void			set_has_received_messages(bool received_messages) { _received_messages = received_messages; }
	bool			get_has_received_messages() { return _received_messages; }
	void			set_wait_to_transmit(bool wait) { _wait_to_transmit = wait; }
	bool			get_wait_to_transmit() { return _wait_to_transmit; }
	bool			should_transmit() { return (!_wait_to_transmit || (_wait_to_transmit && _received_messages)); }

	bool			message_buffer_write(const void *ptr, int size);

	void			lockMessageBufferMutex(void) { pthread_mutex_lock(&_message_buffer_mutex); }
	void			unlockMessageBufferMutex(void) { pthread_mutex_unlock(&_message_buffer_mutex); }

	/**
	 * Count a transmision error
	 */
	void			count_txerr();

	/**
	 * Count transmitted bytes
	 */
	void			count_txbytes(unsigned n) { _bytes_tx += n; };

	/**
	 * Count bytes not transmitted because of errors
	 */
	void			count_txerrbytes(unsigned n) { _bytes_txerr += n; };

	/**
	 * Count received bytes
	 */
	void			count_rxbytes(unsigned n) { _bytes_rx += n; };

	/**
	 * Get the receive status of this MAVLink link
	 */
	struct telemetry_status_s&	get_rx_status() { return _rstatus; }

	ringbuffer::RingBuffer	*get_logbuffer() { return &_logbuffer; }

	unsigned		get_system_type() { return _system_type; }

	Protocol 		get_protocol() { return _protocol; }

	unsigned short		get_network_port() { return _network_port; }

	unsigned short		get_remote_port() { return _remote_port; }

	int 			get_socket_fd () { return _socket_fd; };
#ifdef __PX4_POSIX
	struct sockaddr_in *	get_client_source_address() { return &_src_addr; }

	void			set_client_source_initialized() { _src_addr_initialized = true; }

	bool			get_client_source_initialized() { return _src_addr_initialized; }
#else
	bool			get_client_source_initialized() { return true; }
#endif

	uint64_t		get_start_time() { return _mavlink_start_time; }

	static bool		boot_complete() { return _boot_complete; }

	bool			is_usb_uart() { return _is_usb_uart; }

protected:
	Mavlink			*next;

private:
	int			_instance_id;

	orb_advert_t		_mavlink_log_pub;
	bool			_task_running;
	static bool		_boot_complete;

	/* states */
	bool			_hil_enabled;		/**< Hardware In the Loop mode */
	bool			_generate_rc;		/**< Generate RC messages from manual input MAVLink messages */
	bool			_use_hil_gps;		/**< Accept GPS HIL messages (for example from an external motion capturing system to fake indoor gps) */
	bool			_forward_externalsp;	/**< Forward external setpoint messages to controllers directly if in offboard mode */
	bool			_is_usb_uart;		/**< Port is USB */
	bool			_wait_to_transmit;  	/**< Wait to transmit until received messages. */
	bool			_received_messages;	/**< Whether we've received valid mavlink messages. */

	unsigned		_main_loop_delay;	/**< mainloop delay, depends on data rate */

	MavlinkOrbSubscription	*_subscriptions;
	MavlinkStream		*_streams;

	MavlinkMissionManager		*_mission_manager;
	MavlinkParametersManager	*_parameters_manager;
	MavlinkFTP			*_mavlink_ftp;
	MavlinkLogHandler		*_mavlink_log_handler;

	MAVLINK_MODE 		_mode;

	mavlink_channel_t	_channel;
	int32_t			_radio_id;

	ringbuffer::RingBuffer		_logbuffer;
	unsigned int		_total_counter;

	pthread_t		_receive_thread;

	bool			_verbose;
	bool			_forwarding_on;
	bool			_ftp_on;
#ifndef __PX4_QURT
	int			_uart_fd;
#endif
	int			_baudrate;
	int			_datarate;		///< data rate for normal streams (attitude, position, etc.)
	int			_datarate_events;	///< data rate for params, waypoints, text messages
	float			_rate_mult;
	hrt_abstime		_last_hw_rate_timestamp;

	/**
	 * If the queue index is not at 0, the queue sending
	 * logic will send parameters from the current index
	 * to len - 1, the end of the param list.
	 */
	unsigned int		_mavlink_param_queue_index;

	bool			mavlink_link_termination_allowed;

	char 			*_subscribe_to_stream;
	float			_subscribe_to_stream_rate;
	bool 			_udp_initialised;

	bool			_flow_control_enabled;
	uint64_t		_last_write_success_time;
	uint64_t		_last_write_try_time;
	uint64_t		_mavlink_start_time;

	unsigned		_bytes_tx;
	unsigned		_bytes_txerr;
	unsigned		_bytes_rx;
	uint64_t		_bytes_timestamp;
	float			_rate_tx;
	float			_rate_txerr;
	float			_rate_rx;

#ifdef __PX4_POSIX
	struct sockaddr_in _myaddr;
	struct sockaddr_in _src_addr;
	struct sockaddr_in _bcast_addr;
	bool _src_addr_initialized;
	bool _broadcast_address_found;

#endif
	int _socket_fd;
	Protocol	_protocol;
	unsigned short _network_port;
	unsigned short _remote_port;

	struct telemetry_status_s	_rstatus;			///< receive status

	struct mavlink_message_buffer {
		int write_ptr;
		int read_ptr;
		int size;
		char *data;
	};

	mavlink_message_buffer	_message_buffer;

	pthread_mutex_t		_message_buffer_mutex;
	pthread_mutex_t		_send_mutex;

	bool			_param_initialized;
	param_t			_param_system_id;
	param_t			_param_component_id;
	param_t			_param_radio_id;
	param_t			_param_system_type;
	param_t			_param_use_hil_gps;
	param_t			_param_forward_externalsp;

	unsigned		_system_type;

	perf_counter_t		_loop_perf;			/**< loop performance counter */
	perf_counter_t		_txerr_perf;			/**< TX error counter */

	void			mavlink_update_system();

#ifndef __PX4_QURT
	int			mavlink_open_uart(int baudrate, const char *uart_name, struct termios *uart_config_original);
#endif

	static unsigned int	interval_from_rate(float rate);

	static constexpr unsigned RADIO_BUFFER_CRITICAL_LOW_PERCENTAGE = 25;
	static constexpr unsigned RADIO_BUFFER_LOW_PERCENTAGE = 35;
	static constexpr unsigned RADIO_BUFFER_HALF_PERCENTAGE = 50;

	int configure_stream(const char *stream_name, const float rate);

	/**
	 * Adjust the stream rates based on the current rate
	 *
	 * @param multiplier if greater than 1, the transmission rate will increase, if smaller than one decrease
	 */
	void adjust_stream_rates(const float multiplier);

	int message_buffer_init(int size);

	void message_buffer_destroy();

	int message_buffer_count();

	int message_buffer_is_empty();

	int message_buffer_get_ptr(void **ptr, bool *is_part);

	void message_buffer_mark_read(int n);

	void pass_message(const mavlink_message_t *msg);

	/**
	 * Update rate mult so total bitrate will be equal to _datarate.
	 */
	void update_rate_mult();

	void find_broadcast_address();

	void init_udp();

	/**
	 * Main mavlink task.
	 */
	int		task_main(int argc, char *argv[]);

	/* do not allow copying this class */
	Mavlink(const Mavlink&);
	Mavlink operator=(const Mavlink&);
};
