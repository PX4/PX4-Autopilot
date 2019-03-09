/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 *
 * MAVLink 2.0 protocol interface definition.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#pragma once

#include <px4_posix.h>
#include <px4_module_params.h>

#include <stdbool.h>
#ifdef __PX4_NUTTX
#include <nuttx/fs/fs.h>
#else
#include <sys/socket.h>
#include <arpa/inet.h>
#include <drivers/device/device.h>
#endif

#if defined(CONFIG_NET) || !defined(__PX4_NUTTX)
#include <netinet/in.h>
#include <net/if.h>
#endif

#include <containers/List.hpp>
#include <systemlib/uthash/utlist.h>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <pthread.h>
#include <systemlib/mavlink_log.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/radio_status.h>
#include <uORB/topics/telemetry_status.h>

#include "mavlink_bridge_header.h"
#include "mavlink_orb_subscription.h"
#include "mavlink_stream.h"
#include "mavlink_messages.h"
#include "mavlink_shell.h"
#include "mavlink_ulog.h"

enum Protocol {
	SERIAL = 0,
	UDP,
	TCP,
};

using namespace time_literals;

#define HASH_PARAM "_HASH_CHECK"

class Mavlink : public ModuleParams
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

	/**
	 * Display the status of all enabled streams.
	 */
	void			display_status_streams();

	static int		stream_command(int argc, char *argv[]);

	static int		instance_count();

	static Mavlink		*new_instance();

	static Mavlink		*get_instance(int instance);

	static Mavlink 		*get_instance_for_device(const char *device_name);

	static Mavlink 		*get_instance_for_network_port(unsigned long port);

	mavlink_message_t 	*get_buffer() { return &_mavlink_buffer; }

	mavlink_status_t 	*get_status() { return &_mavlink_status; }

	/**
	 * Set the MAVLink version
	 *
	 * Currently supporting v1 and v2
	 *
	 * @param version MAVLink version
	 */
	void			set_proto_version(unsigned version);

	static int		destroy_all_instances();

	static int		get_status_all_instances(bool show_streams_status);

	static bool		instance_exists(const char *device_name, Mavlink *self);

	static void		forward_message(const mavlink_message_t *msg, Mavlink *self);

	static int		get_uart_fd(unsigned index);

	int			get_uart_fd() const { return _uart_fd; }

	/**
	 * Get the MAVLink system id.
	 *
	 * @return		The system ID of this vehicle
	 */
	int			get_system_id() const { return mavlink_system.sysid; }

	/**
	 * Get the MAVLink component id.
	 *
	 * @return		The component ID of this vehicle
	 */
	int			get_component_id() const { return mavlink_system.compid; }

	const char *_device_name;

	enum MAVLINK_MODE {
		MAVLINK_MODE_NORMAL = 0,
		MAVLINK_MODE_CUSTOM,
		MAVLINK_MODE_ONBOARD,
		MAVLINK_MODE_OSD,
		MAVLINK_MODE_MAGIC,
		MAVLINK_MODE_CONFIG,
		MAVLINK_MODE_IRIDIUM,
		MAVLINK_MODE_MINIMAL,
		MAVLINK_MODE_EXTVISION,

		MAVLINK_MODE_COUNT
	};

	enum BROADCAST_MODE {
		BROADCAST_MODE_OFF = 0,
		BROADCAST_MODE_ON,
		BROADCAST_MODE_MULTICAST
	};

	enum FLOW_CONTROL_MODE {
		FLOW_CONTROL_OFF = 0,
		FLOW_CONTROL_AUTO,
		FLOW_CONTROL_ON
	};

	static const char *mavlink_mode_str(enum MAVLINK_MODE mode)
	{
		switch (mode) {
		case MAVLINK_MODE_NORMAL:
			return "Normal";

		case MAVLINK_MODE_CUSTOM:
			return "Custom";

		case MAVLINK_MODE_ONBOARD:
			return "Onboard";

		case MAVLINK_MODE_OSD:
			return "OSD";

		case MAVLINK_MODE_MAGIC:
			return "Magic";

		case MAVLINK_MODE_CONFIG:
			return "Config";

		case MAVLINK_MODE_IRIDIUM:
			return "Iridium";

		case MAVLINK_MODE_MINIMAL:
			return "Minimal";

		case MAVLINK_MODE_EXTVISION:
			return "ExtVision";

		default:
			return "Unknown";
		}
	}

	enum MAVLINK_MODE	get_mode() { return _mode; }

	bool			get_hil_enabled() { return _hil_enabled; }

	bool			get_use_hil_gps() { return _param_use_hil_gps.get(); }

	bool			get_forward_externalsp() { return _param_forward_externalsp.get(); }

	bool			get_flow_control_enabled() { return _flow_control_mode; }

	bool			get_forwarding_on() { return _forwarding_on; }

	bool			is_connected() { return (hrt_elapsed_time(&_tstatus.heartbeat_time) < 3_s); }

	bool			broadcast_enabled() { return _param_broadcast_mode.get() == BROADCAST_MODE_ON; }

	/**
	 * Set the boot complete flag on all instances
	 *
	 * Setting the flag unblocks parameter transmissions, which are gated
	 * beforehand to ensure that the system is fully initialized.
	 */
	static void		set_boot_complete();

	/**
	 * Get the free space in the transmit buffer
	 *
	 * @return free space in the UART TX buffer
	 */
	unsigned		get_free_tx_buf();

	static int		start_helper(int argc, char *argv[]);

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
	void 			set_protocol(Protocol p) { _protocol = p; }

	/**
	 * Get the manual input generation mode
	 *
	 * @return true if manual inputs should generate RC data
	 */
	bool			get_manual_input_mode_generation() { return _generate_rc; }

	/**
	 * This is the beginning of a MAVLINK_START_UART_SEND/MAVLINK_END_UART_SEND transaction
	 */
	void 			begin_send() { pthread_mutex_lock(&_send_mutex); }

	/**
	 * Send bytes out on the link.
	 *
	 * On a network port these might actually get buffered to form a packet.
	 */
	void			send_bytes(const uint8_t *buf, unsigned packet_len);

	/**
	 * Flush the transmit buffer and send one MAVLink packet
	 *
	 * @return the number of bytes sent or -1 in case of error
	 */
	int             	send_packet();

	/**
	 * Resend message as is, don't change sequence number and CRC.
	 */
	void			resend_message(mavlink_message_t *msg) { _mavlink_resend_uart(_channel, msg); }

	void			handle_message(const mavlink_message_t *msg);

	/**
	 * Add a mavlink orb topic subscription while ensuring that only a single object exists
	 * for a given topic id and instance.
	 * @param topic orb topic id
	 * @param instance topic instance
	 * @param disable_sharing if true, force creating a new instance
	 */
	MavlinkOrbSubscription *add_orb_subscription(const orb_id_t topic, int instance = 0, bool disable_sharing = false);

	int			get_instance_id() const { return _instance_id; }

	/**
	 * Enable / disable hardware flow control.
	 *
	 * @param enabled	True if hardware flow control should be enabled
	 */
	int			enable_flow_control(enum FLOW_CONTROL_MODE enabled);

	mavlink_channel_t	get_channel() const { return _channel; }

	void			configure_stream_threadsafe(const char *stream_name, float rate = -1.0f);

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

	/**
	 * Send the capabilities of this autopilot in terms of the MAVLink spec
	 */
	void 			send_autopilot_capabilites();

	/**
	 * Send the protocol version of MAVLink
	 */
	void			send_protocol_version();

	List<MavlinkStream *> &get_streams() { return _streams; }

	float			get_rate_mult() const { return _rate_mult; }

	float			get_baudrate() { return _baudrate; }

	/* Functions for waiting to start transmission until message received. */
	void			set_has_received_messages(bool received_messages) { _received_messages = received_messages; }
	bool			get_has_received_messages() { return _received_messages; }
	void			set_wait_to_transmit(bool wait) { _wait_to_transmit = wait; }
	bool			get_wait_to_transmit() { return _wait_to_transmit; }
	bool			should_transmit() { return (_transmitting_enabled && _boot_complete && (!_wait_to_transmit || (_wait_to_transmit && _received_messages))); }

	bool			message_buffer_write(const void *ptr, int size);

	void			lockMessageBufferMutex(void) { pthread_mutex_lock(&_message_buffer_mutex); }
	void			unlockMessageBufferMutex(void) { pthread_mutex_unlock(&_message_buffer_mutex); }

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
	telemetry_status_s	&get_telemetry_status() { return _tstatus; }

	void			set_telemetry_status_type(uint8_t type) { _tstatus.type = type; }

	void			update_radio_status(const radio_status_s &radio_status);

	ringbuffer::RingBuffer	*get_logbuffer() { return &_logbuffer; }

	unsigned		get_system_type() { return _param_system_type.get(); }

	Protocol 		get_protocol() { return _protocol; }

	unsigned short		get_network_port() { return _network_port; }

	unsigned short		get_remote_port() { return _remote_port; }

	int 			get_socket_fd() { return _socket_fd; };

#ifdef __PX4_POSIX
	const in_addr query_netmask_addr(const int socket_fd, const ifreq &ifreq);

	const in_addr compute_broadcast_addr(const in_addr &host_addr, const in_addr &netmask_addr);
#endif

#if defined(CONFIG_NET) || defined(__PX4_POSIX)
	struct sockaddr_in 	&get_client_source_address() { return _src_addr; }

	void			set_client_source_initialized() { _src_addr_initialized = true; }

	bool			get_client_source_initialized() { return _src_addr_initialized; }
#else
	bool			get_client_source_initialized() { return true; }
#endif

	uint64_t		get_start_time() { return _mavlink_start_time; }

	static bool		boot_complete() { return _boot_complete; }

	bool			is_usb_uart() { return _is_usb_uart; }

	int			get_data_rate()		{ return _datarate; }
	void			set_data_rate(int rate) { if (rate > 0) { _datarate = rate; } }

	unsigned		get_main_loop_delay() const { return _main_loop_delay; }

	/** get the Mavlink shell. Create a new one if there isn't one. It is *always* created via MavlinkReceiver thread.
	 *  Returns nullptr if shell cannot be created */
	MavlinkShell		*get_shell();
	/** close the Mavlink shell if it is open */
	void			close_shell();

	/** get ulog streaming if active, nullptr otherwise */
	MavlinkULog		*get_ulog_streaming() { return _mavlink_ulog; }
	void			try_start_ulog_streaming(uint8_t target_system, uint8_t target_component)
	{
		if (_mavlink_ulog) { return; }

		_mavlink_ulog = MavlinkULog::try_start(_datarate, 0.7f, target_system, target_component);
	}
	void			request_stop_ulog_streaming()
	{
		if (_mavlink_ulog) { _mavlink_ulog_stop_requested = true; }
	}


	void set_uorb_main_fd(int fd, unsigned int interval);

	bool ftp_enabled() const { return _ftp_on; }

	bool hash_check_enabled() const { return _param_hash_check_enabled.get(); }
	bool forward_heartbeats_enabled() const { return _param_heartbeat_forwarding_enabled.get(); }
	bool odometry_loopback_enabled() const { return _param_send_odom_loopback.get(); }

	struct ping_statistics_s {
		uint64_t last_ping_time;
		uint32_t last_ping_seq;
		uint32_t dropped_packets;
		float last_rtt;
		float mean_rtt;
		float max_rtt;
		float min_rtt;
	};

	/**
	 * Get the ping statistics of this MAVLink link
	 */
	struct ping_statistics_s &get_ping_statistics() { return _ping_stats; }

protected:
	Mavlink			*next;

private:
	int			_instance_id;
	bool			_transmitting_enabled;
	bool			_transmitting_enabled_commanded;
	bool			_first_heartbeat_sent{false};

	orb_advert_t		_mavlink_log_pub{nullptr};
	orb_advert_t		_telem_status_pub{nullptr};

	bool			_task_running;
	static bool		_boot_complete;
	static constexpr int MAVLINK_MAX_INSTANCES = 4;
	static constexpr int MAVLINK_MIN_INTERVAL = 1500;
	static constexpr int MAVLINK_MAX_INTERVAL = 10000;
	static constexpr float MAVLINK_MIN_MULTIPLIER = 0.0005f;
	mavlink_message_t _mavlink_buffer;
	mavlink_status_t _mavlink_status;

	/* states */
	bool			_hil_enabled;		/**< Hardware In the Loop mode */
	bool			_generate_rc;		/**< Generate RC messages from manual input MAVLink messages */
	bool			_is_usb_uart;		/**< Port is USB */
	bool			_wait_to_transmit;  	/**< Wait to transmit until received messages. */
	bool			_received_messages;	/**< Whether we've received valid mavlink messages. */

	unsigned		_main_loop_delay;	/**< mainloop delay, depends on data rate */

	List<MavlinkOrbSubscription *>	_subscriptions;
	List<MavlinkStream *>		_streams;

	MavlinkShell			*_mavlink_shell;
	MavlinkULog			*_mavlink_ulog;
	volatile bool			_mavlink_ulog_stop_requested;

	MAVLINK_MODE 		_mode;

	mavlink_channel_t	_channel;

	ringbuffer::RingBuffer		_logbuffer;

	pthread_t		_receive_thread;

	bool			_forwarding_on;
	bool			_ftp_on;

	int			_uart_fd;

	int			_baudrate;
	int			_datarate;		///< data rate for normal streams (attitude, position, etc.)
	float			_rate_mult;

	bool			_radio_status_available{false};
	bool			_radio_status_critical{false};
	float			_radio_status_mult{1.0f};

	/**
	 * If the queue index is not at 0, the queue sending
	 * logic will send parameters from the current index
	 * to len - 1, the end of the param list.
	 */
	unsigned int		_mavlink_param_queue_index;

	bool			mavlink_link_termination_allowed;

	char 			*_subscribe_to_stream;
	float			_subscribe_to_stream_rate;  ///< rate of stream to subscribe to (0=disable, -1=unlimited, -2=default)
	bool 			_udp_initialised;

	enum FLOW_CONTROL_MODE	_flow_control_mode;
	uint64_t		_last_write_success_time;
	uint64_t		_last_write_try_time;
	uint64_t		_mavlink_start_time;
	int32_t			_protocol_version_switch;
	int32_t			_protocol_version;

	unsigned		_bytes_tx;
	unsigned		_bytes_txerr;
	unsigned		_bytes_rx;
	uint64_t		_bytes_timestamp;

#if defined(CONFIG_NET) || defined(__PX4_POSIX)
	struct sockaddr_in _myaddr;
	struct sockaddr_in _src_addr;
	struct sockaddr_in _bcast_addr;
	bool _src_addr_initialized;
	bool _broadcast_address_found;
	bool _broadcast_address_not_found_warned;
	bool _broadcast_failed_warned;
	uint8_t _network_buf[MAVLINK_MAX_PACKET_LEN];
	unsigned _network_buf_len;
#endif

	const char *_interface_name;

	int _socket_fd;
	Protocol	_protocol;
	unsigned short _network_port;
	unsigned short _remote_port;

	radio_status_s		_rstatus{};
	telemetry_status_s	_tstatus{};

	ping_statistics_s	_ping_stats{};

	struct mavlink_message_buffer {
		int write_ptr;
		int read_ptr;
		int size;
		char *data;
	};

	mavlink_message_buffer	_message_buffer;

	pthread_mutex_t		_message_buffer_mutex;
	pthread_mutex_t		_send_mutex;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MAV_SYS_ID>) _param_system_id,
		(ParamInt<px4::params::MAV_COMP_ID>) _param_component_id,
		(ParamInt<px4::params::MAV_PROTO_VER>) _param_mav_proto_version,
		(ParamInt<px4::params::MAV_RADIO_ID>) _param_radio_id,
		(ParamInt<px4::params::MAV_TYPE>) _param_system_type,
		(ParamBool<px4::params::MAV_USEHILGPS>) _param_use_hil_gps,
		(ParamBool<px4::params::MAV_FWDEXTSP>) _param_forward_externalsp,
		(ParamInt<px4::params::MAV_BROADCAST>) _param_broadcast_mode,
		(ParamBool<px4::params::MAV_HASH_CHK_EN>) _param_hash_check_enabled,
		(ParamBool<px4::params::MAV_HB_FORW_EN>) _param_heartbeat_forwarding_enabled,
		(ParamBool<px4::params::MAV_ODOM_LP>) _param_send_odom_loopback
	)

	perf_counter_t		_loop_perf;			/**< loop performance counter */
	perf_counter_t		_loop_interval_perf;		/**< loop interval performance counter */

	void			mavlink_update_parameters();

	int			mavlink_open_uart(int baudrate, const char *uart_name, bool force_flow_control);

	static constexpr unsigned RADIO_BUFFER_CRITICAL_LOW_PERCENTAGE = 25;
	static constexpr unsigned RADIO_BUFFER_LOW_PERCENTAGE = 35;
	static constexpr unsigned RADIO_BUFFER_HALF_PERCENTAGE = 50;

	/**
	 * Configure a single stream.
	 * @param stream_name
	 * @param rate streaming rate in Hz, -1 = unlimited rate
	 * @return 0 on success, <0 on error
	 */
	int configure_stream(const char *stream_name, const float rate = -1.0f);

	/**
	 * Configure default streams according to _mode for either all streams or only a single
	 * stream.
	 * @param configure_single_stream: if nullptr, configure all streams, else only a single stream
	 * @return 0 on success, <0 on error
	 */
	int configure_streams_to_default(const char *configure_single_stream = nullptr);

	int message_buffer_init(int size);

	void message_buffer_destroy();

	int message_buffer_count();

	int message_buffer_is_empty() const { return (_message_buffer.read_ptr == _message_buffer.write_ptr); }

	int message_buffer_get_ptr(void **ptr, bool *is_part);

	void message_buffer_mark_read(int n) { _message_buffer.read_ptr = (_message_buffer.read_ptr + n) % _message_buffer.size; }

	void pass_message(const mavlink_message_t *msg);

	void publish_telemetry_status();

	/**
	 * Check the configuration of a connected radio
	 *
	 * This convenience function allows to re-configure a connected
	 * radio without removing it from the main system harness.
	 */
	void check_radio_config();

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
	Mavlink(const Mavlink &);
	Mavlink operator=(const Mavlink &);
};
