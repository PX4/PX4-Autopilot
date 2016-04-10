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
 * @file mavlink_main.cpp
 * MAVLink 1.0 protocol implementation.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <px4_config.h>
#include <px4_getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <assert.h>
#include <math.h>
#include <poll.h>
#include <termios.h>
#include <time.h>
#include <math.h> /* isinf / isnan checks */

#ifdef __PX4_POSIX
#include <net/if.h>
#endif

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/mcu_version.h>
#include <systemlib/git_version.h>
#include <systemlib/mavlink_log.h>
#include <geo/geo.h>
#include <dataman/dataman.h>
//#include <mathlib/mathlib.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/mavlink_log.h>

#include "mavlink_bridge_header.h"
#include "mavlink_main.h"
#include "mavlink_messages.h"
#include "mavlink_receiver.h"
#include "mavlink_rate_limiter.h"

#ifndef MAVLINK_CRC_EXTRA
#error MAVLINK_CRC_EXTRA has to be defined on PX4 systems
#endif

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#define DEFAULT_REMOTE_PORT_UDP			14550 ///< GCS port per MAVLink spec
#define DEFAULT_DEVICE_NAME			"/dev/ttyS1"
#define MAX_DATA_RATE				10000000	///< max data rate in bytes/s
#define MAIN_LOOP_DELAY 			10000	///< 100 Hz @ 1000 bytes/s data rate
#define FLOW_CONTROL_DISABLE_THRESHOLD		40	///< picked so that some messages still would fit it.

static Mavlink *_mavlink_instances = nullptr;

static const uint8_t mavlink_message_lengths[256] = MAVLINK_MESSAGE_LENGTHS;
static const uint8_t mavlink_message_crcs[256] = MAVLINK_MESSAGE_CRCS;

/**
 * mavlink app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mavlink_main(int argc, char *argv[]);

extern mavlink_system_t mavlink_system;

static void usage(void);

bool Mavlink::_boot_complete = false;

Mavlink::Mavlink() :
	_device_name("/dev/ttyS1"),
	_task_should_exit(false),
	next(nullptr),
	_instance_id(0),
	_mavlink_log_pub(nullptr),
	_task_running(false),
	_hil_enabled(false),
	_generate_rc(false),
	_use_hil_gps(false),
	_forward_externalsp(false),
	_is_usb_uart(false),
	_wait_to_transmit(false),
	_received_messages(false),
	_main_loop_delay(1000),
	_subscriptions(nullptr),
	_streams(nullptr),
	_mission_manager(nullptr),
	_parameters_manager(nullptr),
	_mavlink_ftp(nullptr),
	_mavlink_log_handler(nullptr),
	_mode(MAVLINK_MODE_NORMAL),
	_channel(MAVLINK_COMM_0),
	_radio_id(0),
	_logbuffer(5, sizeof(mavlink_log_s)),
	_total_counter(0),
	_receive_thread{},
	_verbose(false),
	_forwarding_on(false),
	_ftp_on(false),
	_uart_fd(-1),
	_baudrate(57600),
	_datarate(1000),
	_datarate_events(500),
	_rate_mult(1.0f),
	_last_hw_rate_timestamp(0),
	_mavlink_param_queue_index(0),
	mavlink_link_termination_allowed(false),
	_subscribe_to_stream(nullptr),
	_subscribe_to_stream_rate(0.0f),
	_udp_initialised(false),
	_flow_control_enabled(true),
	_last_write_success_time(0),
	_last_write_try_time(0),
	_mavlink_start_time(0),
	_bytes_tx(0),
	_bytes_txerr(0),
	_bytes_rx(0),
	_bytes_timestamp(0),
	_rate_tx(0.0f),
	_rate_txerr(0.0f),
	_rate_rx(0.0f),
#ifdef __PX4_POSIX
	_myaddr{},
	_src_addr{},
	_bcast_addr{},
	_src_addr_initialized(false),
	_broadcast_address_found(false),
#endif
	_socket_fd(-1),
	_protocol(SERIAL),
	_network_port(14556),
	_remote_port(DEFAULT_REMOTE_PORT_UDP),
	_rstatus {},
	_message_buffer {},
	_message_buffer_mutex {},
	_send_mutex {},
	_param_initialized(false),
	_param_system_id(0),
	_param_component_id(0),
	_param_radio_id(0),
	_param_system_type(MAV_TYPE_FIXED_WING),
	_param_use_hil_gps(0),
	_param_forward_externalsp(0),
	_system_type(0),

	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "mavlink_el")),
	_txerr_perf(perf_alloc(PC_COUNT, "mavlink_txe"))
{
	_instance_id = Mavlink::instance_count();

	/* set channel according to instance id */
	switch (_instance_id) {
	case 0:
		_channel = MAVLINK_COMM_0;
		break;

	case 1:
		_channel = MAVLINK_COMM_1;
		break;

	case 2:
		_channel = MAVLINK_COMM_2;
		break;

	case 3:
		_channel = MAVLINK_COMM_3;
		break;
#ifdef MAVLINK_COMM_4

	case 4:
		_channel = MAVLINK_COMM_4;
		break;
#endif
#ifdef MAVLINK_COMM_5

	case 5:
		_channel = MAVLINK_COMM_5;
		break;
#endif
#ifdef MAVLINK_COMM_6

	case 6:
		_channel = MAVLINK_COMM_6;
		break;
#endif

	default:
		warnx("instance ID is out of range");
		px4_task_exit(1);
		break;
	}

	_rstatus.type = telemetry_status_s::TELEMETRY_STATUS_RADIO_TYPE_GENERIC;
}

Mavlink::~Mavlink()
{
	perf_free(_loop_perf);
	perf_free(_txerr_perf);

	if (_task_running) {
		/* task wakes up every 10ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				//TODO store main task handle in Mavlink instance to allow killing task
				//task_delete(_mavlink_task);
				break;
			}
		} while (_task_running);
	}

	if (_mavlink_instances) {
		LL_DELETE(_mavlink_instances, this);
	}
}

void
Mavlink::count_txerr()
{
	perf_count(_txerr_perf);
}

void
Mavlink::set_mode(enum MAVLINK_MODE mode)
{
	_mode = mode;
}

int
Mavlink::instance_count()
{
	unsigned inst_index = 0;
	Mavlink *inst;

	LL_FOREACH(::_mavlink_instances, inst) {
		inst_index++;
	}

	return inst_index;
}

Mavlink *
Mavlink::get_instance(unsigned instance)
{
	Mavlink *inst;
	unsigned inst_index = 0;
	LL_FOREACH(::_mavlink_instances, inst) {
		if (instance == inst_index) {
			return inst;
		}

		inst_index++;
	}

	return nullptr;
}

Mavlink *
Mavlink::get_instance_for_device(const char *device_name)
{
	Mavlink *inst;

	LL_FOREACH(::_mavlink_instances, inst) {
		if (strcmp(inst->_device_name, device_name) == 0) {
			return inst;
		}
	}

	return nullptr;
}

Mavlink *
Mavlink::get_instance_for_network_port(unsigned long port)
{
	Mavlink *inst;

	LL_FOREACH(::_mavlink_instances, inst) {
		if (inst->_network_port == port) {
			return inst;
		}
	}

	return nullptr;
}

int
Mavlink::destroy_all_instances()
{
	/* start deleting from the end */
	Mavlink *inst_to_del = nullptr;
	Mavlink *next_inst = ::_mavlink_instances;

	unsigned iterations = 0;

	warnx("waiting for instances to stop");

	while (next_inst != nullptr) {
		inst_to_del = next_inst;
		next_inst = inst_to_del->next;

		/* set flag to stop thread and wait for all threads to finish */
		inst_to_del->_task_should_exit = true;

		while (inst_to_del->_task_running) {
			printf(".");
			fflush(stdout);
			usleep(10000);
			iterations++;

			if (iterations > 1000) {
				warnx("ERROR: Couldn't stop all mavlink instances.");
				return ERROR;
			}
		}
	}

	printf("\n");
	warnx("all instances stopped");
	return OK;
}

int
Mavlink::get_status_all_instances()
{
	Mavlink *inst = ::_mavlink_instances;

	unsigned iterations = 0;

	while (inst != nullptr) {

		printf("\ninstance #%u:\n", iterations);
		inst->display_status();

		/* move on */
		inst = inst->next;
		iterations++;
	}

	/* return an error if there are no instances */
	return (iterations == 0);
}

bool
Mavlink::instance_exists(const char *device_name, Mavlink *self)
{
	Mavlink *inst = ::_mavlink_instances;

	while (inst != nullptr) {

		/* don't compare with itself */
		if (inst != self && !strcmp(device_name, inst->_device_name)) {
			return true;
		}

		inst = inst->next;
	}

	return false;
}

void
Mavlink::forward_message(const mavlink_message_t *msg, Mavlink *self)
{

	Mavlink *inst;
	LL_FOREACH(_mavlink_instances, inst) {
		if (inst != self) {

			/* if not in normal mode, we are an onboard link
			 * onboard links should only pass on messages from the same system ID */
			if (!(self->_mode != MAVLINK_MODE_NORMAL && msg->sysid != mavlink_system.sysid)) {
				inst->pass_message(msg);
			}
		}
	}
}

int
Mavlink::get_uart_fd(unsigned index)
{
	Mavlink *inst = get_instance(index);

	if (inst) {
		return inst->get_uart_fd();
	}

	return -1;
}

int
Mavlink::get_uart_fd()
{
	return _uart_fd;
}

int
Mavlink::get_instance_id()
{
	return _instance_id;
}

mavlink_channel_t
Mavlink::get_channel()
{
	return _channel;
}

void Mavlink::mavlink_update_system(void)
{
	if (!_param_initialized) {
		_param_system_id = param_find("MAV_SYS_ID");
		_param_component_id = param_find("MAV_COMP_ID");
		_param_radio_id = param_find("MAV_RADIO_ID");
		_param_system_type = param_find("MAV_TYPE");
		_param_use_hil_gps = param_find("MAV_USEHILGPS");
		_param_forward_externalsp = param_find("MAV_FWDEXTSP");

		/* test param - needs to be referenced, but is unused */
		(void)param_find("MAV_TEST_PAR");
	}

	/* update system and component id */
	int32_t system_id;
	param_get(_param_system_id, &system_id);

	int32_t component_id;
	param_get(_param_component_id, &component_id);

	param_get(_param_radio_id, &_radio_id);

	/* only allow system ID and component ID updates
	 * after reboot - not during operation */
	if (!_param_initialized) {
		if (system_id > 0 && system_id < 255) {
			mavlink_system.sysid = system_id;
		}

		if (component_id > 0 && component_id < 255) {
			mavlink_system.compid = component_id;
		}

		_param_initialized = true;
	}

	/* warn users that they need to reboot to take this
	 * into effect
	 */
	if (system_id != mavlink_system.sysid) {
		send_statustext_critical("Save params and reboot to change SYSID");
	}

	if (component_id != mavlink_system.compid) {
		send_statustext_critical("Save params and reboot to change COMPID");
	}

	int32_t system_type;
	param_get(_param_system_type, &system_type);

	if (system_type >= 0 && system_type < MAV_TYPE_ENUM_END) {
		_system_type = system_type;
	}

	int32_t use_hil_gps;
	param_get(_param_use_hil_gps, &use_hil_gps);

	_use_hil_gps = (bool)use_hil_gps;

	int32_t forward_externalsp;
	param_get(_param_forward_externalsp, &forward_externalsp);

	_forward_externalsp = (bool)forward_externalsp;
}

int Mavlink::get_system_id()
{
	return mavlink_system.sysid;
}

int Mavlink::get_component_id()
{
	return mavlink_system.compid;
}

int Mavlink::mavlink_open_uart(int baud, const char *uart_name, struct termios *uart_config_original)
{
#ifndef B460800
	#define B460800 460800
#endif

#ifndef B921600
	#define B921600 921600
#endif

	/* process baud rate */
	int speed;

	switch (baud) {
	case 0:      speed = B0;      break;

	case 50:     speed = B50;     break;

	case 75:     speed = B75;     break;

	case 110:    speed = B110;    break;

	case 134:    speed = B134;    break;

	case 150:    speed = B150;    break;

	case 200:    speed = B200;    break;

	case 300:    speed = B300;    break;

	case 600:    speed = B600;    break;

	case 1200:   speed = B1200;   break;

	case 1800:   speed = B1800;   break;

	case 2400:   speed = B2400;   break;

	case 4800:   speed = B4800;   break;

	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	case 230400: speed = B230400; break;

	case 460800: speed = B460800; break;

	case 921600: speed = B921600; break;

	default:
		warnx("ERROR: Unsupported baudrate: %d\n\tsupported examples:\n\t9600, 19200, 38400, 57600\t\n115200\n230400\n460800\n921600\n",
		      baud);
		return -EINVAL;
	}

	/* back off 1800 ms to avoid running into the USB setup timing */
	while (_mode == MAVLINK_MODE_CONFIG &&
		hrt_absolute_time() < 1800U * 1000U) {
		usleep(50000);
	}

	/* open uart */
	_uart_fd = ::open(uart_name, O_RDWR | O_NOCTTY);

	/* if this is a config link, stay here and wait for it to open */
	if (_uart_fd < 0 && _mode == MAVLINK_MODE_CONFIG) {

		int armed_fd = orb_subscribe(ORB_ID(actuator_armed));
		struct actuator_armed_s armed;

		/* get the system arming state and abort on arming */
		while (_uart_fd < 0) {

			/* abort if an arming topic is published and system is armed */
			bool updated = false;
			orb_check(armed_fd, &updated);

			if (updated) {
				/* the system is now providing arming status feedback.
				 * instead of timing out, we resort to abort bringing
				 * up the terminal.
				 */
				orb_copy(ORB_ID(actuator_armed), armed_fd, &armed);

				if (armed.armed) {
					/* this is not an error, but we are done */
					return -1;
				}
			}

			usleep(100000);
			_uart_fd = ::open(uart_name, O_RDWR | O_NOCTTY);
		};

		::close(armed_fd);
	}

	if (_uart_fd < 0) {
		return _uart_fd;
	}


	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;
	_is_usb_uart = false;

	/* Back up the original uart configuration to restore it after exit */
	if ((termios_state = tcgetattr(_uart_fd, uart_config_original)) < 0) {
		warnx("ERR GET CONF %s: %d\n", uart_name, termios_state);
		::close(_uart_fd);
		return -1;
	}

	/* Fill the struct for the new configuration */
	tcgetattr(_uart_fd, &uart_config);

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* USB serial is indicated by /dev/ttyACM0*/
	if (strcmp(uart_name, "/dev/ttyACM0") != OK && strcmp(uart_name, "/dev/ttyACM1") != OK) {

		/* Set baud rate */
		if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
			warnx("ERR SET BAUD %s: %d\n", uart_name, termios_state);
			::close(_uart_fd);
			return -1;
		}

	} else {
		_is_usb_uart = true;
		/* USB has no baudrate, but use a magic number for 'fast' */
		_baudrate = 2000000;
		_rstatus.type = telemetry_status_s::TELEMETRY_STATUS_RADIO_TYPE_USB;
	}

#if defined (__PX4_LINUX) || defined (__PX4_DARWIN)
	/* Put in raw mode */
	cfmakeraw(&uart_config);
#endif

	if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
		warnx("ERR SET CONF %s\n", uart_name);
		::close(_uart_fd);
		return -1;
	}

	if (!_is_usb_uart) {
		/*
		 * Setup hardware flow control. If the port has no RTS pin this call will fail,
		 * which is not an issue, but requires a separate call so we can fail silently.
		 */
		(void)tcgetattr(_uart_fd, &uart_config);
#ifdef CRTS_IFLOW
		uart_config.c_cflag |= CRTS_IFLOW;
#else
		uart_config.c_cflag |= CRTSCTS;
#endif
		(void)tcsetattr(_uart_fd, TCSANOW, &uart_config);

		/* setup output flow control */
		if (enable_flow_control(true)) {
			warnx("hardware flow control not supported");
		}

	} else {
		_flow_control_enabled = false;
	}

	return _uart_fd;
}

int
Mavlink::enable_flow_control(bool enabled)
{
	// We can't do this on USB - skip
	if (_is_usb_uart) {
		_flow_control_enabled = false;
		return OK;
	}

	struct termios uart_config;

	int ret = tcgetattr(_uart_fd, &uart_config);

	if (enabled) {
		uart_config.c_cflag |= CRTSCTS;

	} else {
		uart_config.c_cflag &= ~CRTSCTS;
	}

	ret = tcsetattr(_uart_fd, TCSANOW, &uart_config);

	if (!ret) {
		_flow_control_enabled = enabled;
	}

	return ret;
}

int
Mavlink::set_hil_enabled(bool hil_enabled)
{
	int ret = OK;

	/* enable HIL */
	if (hil_enabled && !_hil_enabled) {
		_hil_enabled = true;
		configure_stream("HIL_CONTROLS", 200.0f);
	}

	/* disable HIL */
	if (!hil_enabled && _hil_enabled) {
		_hil_enabled = false;
		configure_stream("HIL_CONTROLS", 0.0f);

	} else {
		ret = ERROR;
	}

	return ret;
}

unsigned
Mavlink::get_free_tx_buf()
{
	/*
	 * Check if the OS buffer is full and disable HW
	 * flow control if it continues to be full
	 */
	int buf_free = 0;

	// if we are using network sockets, return max length of one packet
	if (get_protocol() == UDP || get_protocol() == TCP ) {
		return  1500;
	} else {
		// No FIONWRITE on Linux
#if !defined(__PX4_LINUX) && !defined(__PX4_DARWIN)
		(void) ioctl(_uart_fd, FIONWRITE, (unsigned long)&buf_free);
#endif

		if (get_flow_control_enabled() && buf_free < FLOW_CONTROL_DISABLE_THRESHOLD) {
			/* Disable hardware flow control:
			 * if no successful write since a defined time
			 * and if the last try was not the last successful write
			 */
			if (_last_write_try_time != 0 &&
			    hrt_elapsed_time(&_last_write_success_time) > 500 * 1000UL &&
			    _last_write_success_time != _last_write_try_time) {
				warnx("Disabling hardware flow control");
				enable_flow_control(false);
			}
		}
	}

	return buf_free;
}

void
Mavlink::send_message(const uint8_t msgid, const void *msg, uint8_t component_ID)
{
	/* If the wait until transmit flag is on, only transmit after we've received messages.
	   Otherwise, transmit all the time. */
	if (!should_transmit()) {
		return;
	}

	pthread_mutex_lock(&_send_mutex);

	uint8_t payload_len = mavlink_message_lengths[msgid];
	unsigned packet_len = payload_len + MAVLINK_NUM_NON_PAYLOAD_BYTES;

	_last_write_try_time = hrt_absolute_time();

	if (_mavlink_start_time == 0) {
		_mavlink_start_time = _last_write_try_time;
	}

	if (get_protocol() == SERIAL) {
		/* check if there is space in the buffer, let it overflow else */
		unsigned buf_free = get_free_tx_buf();
		if (buf_free < packet_len) {
			 /* no enough space in buffer to send */
			count_txerr();
			count_txerrbytes(packet_len);
			pthread_mutex_unlock(&_send_mutex);
			return;
		}
	}

	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	/* header */
	buf[0] = MAVLINK_STX;
	buf[1] = payload_len;
	/* use mavlink's internal counter for the TX seq */
	buf[2] = mavlink_get_channel_status(_channel)->current_tx_seq++;
	buf[3] = mavlink_system.sysid;
	buf[4] = (component_ID == 0) ? mavlink_system.compid : component_ID;
	buf[5] = msgid;

	/* payload */
	memcpy(&buf[MAVLINK_NUM_HEADER_BYTES], msg, payload_len);

	/* checksum */
	uint16_t checksum;
	crc_init(&checksum);
	crc_accumulate_buffer(&checksum, (const char *) &buf[1], MAVLINK_CORE_HEADER_LEN + payload_len);
	crc_accumulate(mavlink_message_crcs[msgid], &checksum);

	buf[MAVLINK_NUM_HEADER_BYTES + payload_len] = (uint8_t)(checksum & 0xFF);
	buf[MAVLINK_NUM_HEADER_BYTES + payload_len + 1] = (uint8_t)(checksum >> 8);

	size_t ret = -1;

	/* send message to UART */
	if (get_protocol() == SERIAL) {
		ret = ::write(_uart_fd, buf, packet_len);
	}

#ifdef __PX4_POSIX
	if (get_protocol() == UDP) {
		ret = sendto(_socket_fd, buf, packet_len, 0, (struct sockaddr *)&_src_addr, sizeof(_src_addr));

		struct telemetry_status_s &tstatus = get_rx_status();

		/* resend heartbeat via broadcast */
		if ((_mode != MAVLINK_MODE_ONBOARD) &&
			(!get_client_source_initialized()
			|| (hrt_elapsed_time(&tstatus.heartbeat_time) > 3 * 1000 * 1000))
			&& (msgid == MAVLINK_MSG_ID_HEARTBEAT)) {

			if (!_broadcast_address_found) {
				// Try to initialize UDP and broadcast address again.
				init_udp();
			}

			if (_broadcast_address_found) {

				int bret = sendto(_socket_fd, buf, packet_len, 0, (struct sockaddr *)&_bcast_addr, sizeof(_bcast_addr));

				if (bret <= 0) {
					PX4_WARN("sending broadcast failed, errno: %d: %s", errno, strerror(errno));
				}
			}
		}

	} else if (get_protocol() == TCP) {
		/* not implemented, but possible to do so */
		warnx("TCP transport pending implementation");
	}
#endif

	if (ret != (size_t) packet_len) {
		count_txerr();
		count_txerrbytes(packet_len);

	} else {
		_last_write_success_time = _last_write_try_time;
		count_txbytes(packet_len);
	}

	pthread_mutex_unlock(&_send_mutex);
}

void
Mavlink::resend_message(mavlink_message_t *msg)
{
	/* If the wait until transmit flag is on, only transmit after we've received messages.
	   Otherwise, transmit all the time. */
	if (!should_transmit()) {
		return;
	}

	pthread_mutex_lock(&_send_mutex);

	unsigned buf_free = get_free_tx_buf();

	_last_write_try_time = hrt_absolute_time();

	unsigned packet_len = msg->len + MAVLINK_NUM_NON_PAYLOAD_BYTES;

	/* check if there is space in the buffer, let it overflow else */
	if (buf_free < packet_len) {
		/* no enough space in buffer to send */
		count_txerr();
		count_txerrbytes(packet_len);
		pthread_mutex_unlock(&_send_mutex);
		return;
	}

	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	/* header and payload */
	memcpy(&buf[0], &msg->magic, MAVLINK_NUM_HEADER_BYTES + msg->len);

	/* checksum */
	buf[MAVLINK_NUM_HEADER_BYTES + msg->len] = (uint8_t)(msg->checksum & 0xFF);
	buf[MAVLINK_NUM_HEADER_BYTES + msg->len + 1] = (uint8_t)(msg->checksum >> 8);

	if (_uart_fd >= 0) {
		/* send message to UART */
		ssize_t ret = ::write(_uart_fd, buf, packet_len);

		if (ret != (int) packet_len) {
			count_txerr();
			count_txerrbytes(packet_len);

		} else {
			_last_write_success_time = _last_write_try_time;
			count_txbytes(packet_len);
		}
	}

	pthread_mutex_unlock(&_send_mutex);
}

void
Mavlink::init_udp()
{
#if defined (__PX4_LINUX) || defined (__PX4_DARWIN)
	PX4_INFO("Setting up UDP w/port %d",_network_port);

	_myaddr.sin_family = AF_INET;
	_myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	_myaddr.sin_port = htons(_network_port);

	if ((_socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		PX4_WARN("create socket failed");
		return;
	}

	int broadcast_opt = 1;
	if (setsockopt(_socket_fd, SOL_SOCKET, SO_BROADCAST, &broadcast_opt, sizeof(broadcast_opt)) < 0) {
		PX4_WARN("setting broadcast permission failed");
		return;
	}

	if (bind(_socket_fd, (struct sockaddr *)&_myaddr, sizeof(_myaddr)) < 0) {
		PX4_WARN("bind failed");
		return;
	}

	/* set default target address, but not for onboard mode (will be set on first received packet) */
	if (!_src_addr_initialized) {
		_src_addr.sin_family = AF_INET;
		inet_aton("127.0.0.1", &_src_addr.sin_addr);
	}
	_src_addr.sin_port = htons(_remote_port);

	struct ifconf ifconf;
	int ret;

#if defined(__APPLE__) && defined(__MACH__)
	// On Mac, we can't determine the required buffer
	// size in advance, so we just use what tends to work.
	ifconf.ifc_len = 1024;
#else
	// On Linux, we can determine the required size of the
	// buffer first by providing NULL to ifc_req.
	ifconf.ifc_req = NULL;
	ifconf.ifc_len = 0;

	ret = ioctl(_socket_fd, SIOCGIFCONF, &ifconf);
	if (ret != 0) {
		PX4_WARN("getting required buffer size failed");
		return;
	}
#endif

	PX4_DEBUG("need to allocate %d bytes", ifconf.ifc_len);

	// Allocate buffer.
	ifconf.ifc_req = (struct ifreq *)(new uint8_t[ifconf.ifc_len]);
	if (ifconf.ifc_req == nullptr) {
		PX4_ERR("Could not allocate ifconf buffer");
		return;
	}

	memset(ifconf.ifc_req, 0, ifconf.ifc_len);

	ret = ioctl(_socket_fd, SIOCGIFCONF, &ifconf);
	if (ret != 0) {
		PX4_ERR("getting network config failed");
		delete[] ifconf.ifc_req;
		return;
	}

	size_t offset = 0;
	// Later used to point to next network interface in buffer.
	struct ifreq *cur_ifreq = (struct ifreq *)&(((uint8_t *)ifconf.ifc_req)[offset]);

	// The ugly `for` construct is used because it allows to use
	// `continue` and `break`.
	for (;
	     offset < ifconf.ifc_len;
#if defined(__APPLE__) && defined(__MACH__)
	     // On Mac, to get to next entry in buffer, jump by the size of
	     // the interface name size plus whatever is greater, either the
	     // sizeof sockaddr or ifr_addr.sa_len.
	     offset += IF_NAMESIZE
	               + (sizeof(struct sockaddr) > cur_ifreq->ifr_addr.sa_len ?
		          sizeof(struct sockaddr) : cur_ifreq->ifr_addr.sa_len)
#else
	    // On Linux, it's much easier to traverse the buffer, every entry
	    // has the constant length.
	    offset += sizeof(struct ifreq)
#endif
	    ) {

		// Point to next network interface in buffer.
		cur_ifreq = (struct ifreq *)&(((uint8_t *)ifconf.ifc_req)[offset]);

		PX4_DEBUG("looking at %s", cur_ifreq->ifr_name);

		// ignore loopback network
		if (strcmp(cur_ifreq->ifr_name, "lo") == 0 ||
		    strcmp(cur_ifreq->ifr_name, "lo0") == 0 ||
		    strcmp(cur_ifreq->ifr_name, "lo1") == 0 ||
		    strcmp(cur_ifreq->ifr_name, "lo2") == 0) {
			PX4_DEBUG("skipping loopback");
			continue;
		}

		struct ifreq bc_ifreq;
		memset(&bc_ifreq, 0, sizeof(bc_ifreq));
		strncpy(bc_ifreq.ifr_name, cur_ifreq->ifr_name, IF_NAMESIZE);
		ret = ioctl(_socket_fd, SIOCGIFBRDADDR, &bc_ifreq);
		if (ret != 0) {
			PX4_DEBUG("getting broadcast address failed for %s", cur_ifreq->ifr_name);
			continue;
		}

		struct in_addr &sin_addr = ((struct sockaddr_in *)&cur_ifreq->ifr_addr)->sin_addr;

		// Accept network interfaces to local network only. This means it's an IP starting with:
		// 192./172./10.
		// Also see https://tools.ietf.org/html/rfc1918#section-3

		uint8_t first_byte = sin_addr.s_addr & 0xFF;

		if (first_byte != 192 && first_byte != 172 && first_byte != 10) {
			continue;
		}

		if (!_broadcast_address_found) {
			PX4_INFO("using network interface %s, IP: %s", cur_ifreq->ifr_name, inet_ntoa(sin_addr));

			struct in_addr &bc_addr = ((struct sockaddr_in *)&bc_ifreq.ifr_broadaddr)->sin_addr;
			PX4_INFO("with broadcast IP: %s", inet_ntoa(bc_addr));

			_bcast_addr.sin_family = AF_INET;
			_bcast_addr.sin_addr = bc_addr;

			_broadcast_address_found = true;
		} else {
			PX4_INFO("ignoring additional network interface %s, IP:  %s",
				 cur_ifreq->ifr_name, inet_ntoa(sin_addr));
		}
	}

	if (_broadcast_address_found) {
		_bcast_addr.sin_port = htons(_remote_port);

	} else {
		PX4_WARN("no broadcasting address found");
	}

	delete[] ifconf.ifc_req;
#endif
}

void
Mavlink::handle_message(const mavlink_message_t *msg)
{
	/* handle packet with mission manager */
	_mission_manager->handle_message(msg);

	/* handle packet with parameter component */
	_parameters_manager->handle_message(msg);

	/* handle packet with ftp component */
	_mavlink_ftp->handle_message(msg);

	/* handle packet with log component */
	_mavlink_log_handler->handle_message(msg);

	if (get_forwarding_on()) {
		/* forward any messages to other mavlink instances */
		Mavlink::forward_message(msg, this);
	}
}

void
Mavlink::send_statustext_info(const char *string)
{
	mavlink_log_info(&_mavlink_log_pub, string);
}

void
Mavlink::send_statustext_critical(const char *string)
{
	mavlink_log_critical(&_mavlink_log_pub, string);
}

void
Mavlink::send_statustext_emergency(const char *string)
{
	mavlink_log_emergency(&_mavlink_log_pub, string);
}

void Mavlink::send_autopilot_capabilites()
{
	struct vehicle_status_s status;

	MavlinkOrbSubscription *status_sub = this->add_orb_subscription(ORB_ID(vehicle_status));

	if (status_sub->update(&status)) {
		mavlink_autopilot_version_t msg = {};

		msg.capabilities = MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_COMMAND_INT;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_FTP;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_FTP;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET;
		msg.flight_sw_version = 0;
		msg.middleware_sw_version = 0;
		msg.os_sw_version = 0;
		msg.board_version = 0;
		memcpy(&msg.flight_custom_version, &px4_git_version_binary, sizeof(msg.flight_custom_version));
		memcpy(&msg.middleware_custom_version, &px4_git_version_binary, sizeof(msg.middleware_custom_version));
		memset(&msg.os_custom_version, 0, sizeof(msg.os_custom_version));
#ifdef CONFIG_CDCACM_VENDORID
		msg.vendor_id = CONFIG_CDCACM_VENDORID;
#else
		msg.vendor_id = 0;
#endif
#ifdef CONFIG_CDCACM_PRODUCTID
		msg.product_id = CONFIG_CDCACM_PRODUCTID;
#else
		msg.product_id = 0;
#endif
		uint32_t uid[3];
		mcu_unique_id(uid);
		msg.uid = (((uint64_t)uid[1]) << 32) | uid[2];

		this->send_message(MAVLINK_MSG_ID_AUTOPILOT_VERSION, &msg);
	}
}

MavlinkOrbSubscription *Mavlink::add_orb_subscription(const orb_id_t topic, int instance)
{
	/* check if already subscribed to this topic */
	MavlinkOrbSubscription *sub;

	LL_FOREACH(_subscriptions, sub) {
		if (sub->get_topic() == topic && sub->get_instance() == instance) {
			/* already subscribed */
			return sub;
		}
	}

	/* add new subscription */
	MavlinkOrbSubscription *sub_new = new MavlinkOrbSubscription(topic, instance);

	LL_APPEND(_subscriptions, sub_new);

	return sub_new;
}

unsigned int
Mavlink::interval_from_rate(float rate)
{
	return (rate > 0.0f) ? (1000000.0f / rate) : 0;
}

int
Mavlink::configure_stream(const char *stream_name, const float rate)
{
	/* calculate interval in us, 0 means disabled stream */
	unsigned int interval = interval_from_rate(rate);

	/* search if stream exists */
	MavlinkStream *stream;
	LL_FOREACH(_streams, stream) {
		if (strcmp(stream_name, stream->get_name()) == 0) {
			if (interval > 0) {
				/* set new interval */
				stream->set_interval(interval);

			} else {
				/* delete stream */
				LL_DELETE(_streams, stream);
				delete stream;
			}

			return OK;
		}
	}

	if (interval == 0) {
		/* stream was not active and is requested to be disabled, do nothing */
		return OK;
	}

	/* search for stream with specified name in supported streams list */
	for (unsigned int i = 0; streams_list[i] != nullptr; i++) {

		if (strcmp(stream_name, streams_list[i]->get_name()) == 0) {
			/* create new instance */
			stream = streams_list[i]->new_instance(this);
			stream->set_interval(interval);
			LL_APPEND(_streams, stream);

			return OK;
		}
	}

	/* if we reach here, the stream list does not contain the stream */
	warnx("stream %s not found", stream_name);

	return ERROR;
}

void
Mavlink::adjust_stream_rates(const float multiplier)
{
	/* do not allow to push us to zero */
	if (multiplier < 0.0005f) {
		return;
	}

	/* search if stream exists */
	MavlinkStream *stream;
	LL_FOREACH(_streams, stream) {
		/* set new interval */
		unsigned interval = stream->get_interval();
		interval /= multiplier;

		/* allow max ~2000 Hz */
		if (interval < 1600) {
			interval = 500;
		}

		/* set new interval */
		stream->set_interval(interval * multiplier);
	}
}

void
Mavlink::configure_stream_threadsafe(const char *stream_name, const float rate)
{
	/* orb subscription must be done from the main thread,
	 * set _subscribe_to_stream and _subscribe_to_stream_rate fields
	 * which polled in mavlink main loop */
	if (!_task_should_exit) {
		/* wait for previous subscription completion */
		while (_subscribe_to_stream != nullptr) {
			usleep(MAIN_LOOP_DELAY / 2);
		}

		/* copy stream name */
		unsigned n = strlen(stream_name) + 1;
		char *s = new char[n];
		strcpy(s, stream_name);

		/* set subscription task */
		_subscribe_to_stream_rate = rate;
		_subscribe_to_stream = s;

		/* wait for subscription */
		do {
			usleep(MAIN_LOOP_DELAY / 2);
		} while (_subscribe_to_stream != nullptr);

		delete[] s;
	}
}

int
Mavlink::message_buffer_init(int size)
{

	_message_buffer.size = size;
	_message_buffer.write_ptr = 0;
	_message_buffer.read_ptr = 0;
	_message_buffer.data = (char *)malloc(_message_buffer.size);

	int ret;

	if (_message_buffer.data == 0) {
		ret = ERROR;
		_message_buffer.size = 0;

	} else {
		ret = OK;
	}

	return ret;
}

void
Mavlink::message_buffer_destroy()
{
	_message_buffer.size = 0;
	_message_buffer.write_ptr = 0;
	_message_buffer.read_ptr = 0;
	free(_message_buffer.data);
}

int
Mavlink::message_buffer_count()
{
	int n = _message_buffer.write_ptr - _message_buffer.read_ptr;

	if (n < 0) {
		n += _message_buffer.size;
	}

	return n;
}

int
Mavlink::message_buffer_is_empty()
{
	return _message_buffer.read_ptr == _message_buffer.write_ptr;
}


bool
Mavlink::message_buffer_write(const void *ptr, int size)
{
	// bytes available to write
	int available = _message_buffer.read_ptr - _message_buffer.write_ptr - 1;

	if (available < 0) {
		available += _message_buffer.size;
	}

	if (size > available) {
		// buffer overflow
		return false;
	}

	char *c = (char *) ptr;
	int n = _message_buffer.size - _message_buffer.write_ptr;	// bytes to end of the buffer

	if (n < size) {
		// message goes over end of the buffer
		memcpy(&(_message_buffer.data[_message_buffer.write_ptr]), c, n);
		_message_buffer.write_ptr = 0;

	} else {
		n = 0;
	}

	// now: n = bytes already written
	int p = size - n;	// number of bytes to write
	memcpy(&(_message_buffer.data[_message_buffer.write_ptr]), &(c[n]), p);
	_message_buffer.write_ptr = (_message_buffer.write_ptr + p) % _message_buffer.size;
	return true;
}

int
Mavlink::message_buffer_get_ptr(void **ptr, bool *is_part)
{
	// bytes available to read
	int available = _message_buffer.write_ptr - _message_buffer.read_ptr;

	if (available == 0) {
		return 0;	// buffer is empty
	}

	int n = 0;

	if (available > 0) {
		// read pointer is before write pointer, all available bytes can be read
		n = available;
		*is_part = false;

	} else {
		// read pointer is after write pointer, read bytes from read_ptr to end of the buffer
		n = _message_buffer.size - _message_buffer.read_ptr;
		*is_part = _message_buffer.write_ptr > 0;
	}

	*ptr = &(_message_buffer.data[_message_buffer.read_ptr]);
	return n;
}

void
Mavlink::message_buffer_mark_read(int n)
{
	_message_buffer.read_ptr = (_message_buffer.read_ptr + n) % _message_buffer.size;
}

void
Mavlink::pass_message(const mavlink_message_t *msg)
{
	if (_forwarding_on) {
		/* size is 8 bytes plus variable payload */
		int size = MAVLINK_NUM_NON_PAYLOAD_BYTES + msg->len;
		pthread_mutex_lock(&_message_buffer_mutex);
		message_buffer_write(msg, size);
		pthread_mutex_unlock(&_message_buffer_mutex);
	}
}

float
Mavlink::get_rate_mult()
{
	return _rate_mult;
}

void
Mavlink::update_rate_mult()
{
	float const_rate = 0.0f;
	float rate = 0.0f;

	/* scale down rates if their theoretical bandwidth is exceeding the link bandwidth */
	MavlinkStream *stream;
	LL_FOREACH(_streams, stream) {
		if (stream->const_rate()) {
			const_rate += stream->get_size() * 1000000.0f / stream->get_interval();

		} else {
			rate += stream->get_size() * 1000000.0f / stream->get_interval();
		}
	}

	/* don't scale up rates, only scale down if needed */
	float bandwidth_mult = fminf(1.0f, ((float)_datarate - const_rate) / rate);

	/* check if we have radio feedback */
	struct telemetry_status_s &tstatus = get_rx_status();

	bool radio_critical = false;
	bool radio_found = false;

	/* 2nd pass: Now check hardware limits */
	if (tstatus.type == telemetry_status_s::TELEMETRY_STATUS_RADIO_TYPE_3DR_RADIO) {

		radio_found = true;

		if (tstatus.txbuf < RADIO_BUFFER_LOW_PERCENTAGE) {
			radio_critical = true;
		}
	}

	float hardware_mult = _rate_mult;

	/* scale down if we have a TX err rate suggesting link congestion */
	if (_rate_txerr > 0.0f && !radio_critical) {
		hardware_mult = (_rate_tx) / (_rate_tx + _rate_txerr);
	} else if (radio_found && tstatus.telem_time != _last_hw_rate_timestamp) {

		if (tstatus.txbuf < RADIO_BUFFER_CRITICAL_LOW_PERCENTAGE) {
			/* this indicates link congestion, reduce rate by 20% */
			hardware_mult *= 0.80f;
		} else if (tstatus.txbuf < RADIO_BUFFER_LOW_PERCENTAGE) {
			/* this indicates link congestion, reduce rate by 2.5% */
			hardware_mult *= 0.975f;
		} else if (tstatus.txbuf > RADIO_BUFFER_HALF_PERCENTAGE) {
			/* this indicates spare bandwidth, increase by 2.5% */
			hardware_mult *= 1.025f;
			/* limit to a max multiplier of 1 */
			hardware_mult = fminf(1.0f, hardware_mult);
		}

	} else if (!radio_found) {
		/* no limitation, set hardware to 1 */
		hardware_mult = 1.0f;
	}

	_last_hw_rate_timestamp = tstatus.telem_time;

	/* pick the minimum from bandwidth mult and hardware mult as limit */
	_rate_mult = fminf(bandwidth_mult, hardware_mult);

	/* ensure the rate multiplier never drops below 5% so that something is always sent */
	_rate_mult = fmaxf(0.05f, _rate_mult);
}

int
Mavlink::task_main(int argc, char *argv[])
{
	int ch;
	_baudrate = 57600;
	_datarate = 0;
	_mode = MAVLINK_MODE_NORMAL;

#ifdef __PX4_NUTTX
	/* the NuttX optarg handler does not
	 * ignore argv[0] like the POSIX handler
	 * does, nor does it deal with non-flag
	 * verbs well. Remove the application
	 * name and the verb.
	 */
	argc -= 2;
	argv += 2;
#endif

	/* don't exit from getopt loop to leave getopt global variables in consistent state,
	 * set error flag instead */
	bool err_flag = false;
	int myoptind = 1;
	const char *myoptarg = NULL;
#ifdef __PX4_POSIX
	char* eptr;
	int temp_int_arg;
#endif

	while ((ch = px4_getopt(argc, argv, "b:r:d:u:o:m:t:fpvwx", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			_baudrate = strtoul(myoptarg, NULL, 10);

			if (_baudrate < 9600 || _baudrate > 921600) {
				warnx("invalid baud rate '%s'", myoptarg);
				err_flag = true;
			}

			break;

		case 'r':
			_datarate = strtoul(myoptarg, NULL, 10);

			if (_datarate < 10 || _datarate > MAX_DATA_RATE) {
				warnx("invalid data rate '%s'", myoptarg);
				err_flag = true;
			}

			break;

		case 'd':
			_device_name = myoptarg;
			set_protocol(SERIAL);
			break;

#ifdef __PX4_POSIX
		case 'u':
			temp_int_arg = strtoul(myoptarg, &eptr, 10);
			if ( *eptr == '\0' ) {
				_network_port = temp_int_arg;
				set_protocol(UDP);
			} else {
				warnx("invalid data udp_port '%s'", myoptarg);
				err_flag = true;
			}
			break;

		case 'o':
			temp_int_arg = strtoul(myoptarg, &eptr, 10);
			if ( *eptr == '\0' ) {
				warnx("set remote port %d", temp_int_arg);
				_remote_port = temp_int_arg;
				set_protocol(UDP);
			} else {
				warnx("invalid remote udp_port '%s'", myoptarg);
				err_flag = true;
			}
			break;

		case 't':
			_src_addr.sin_family = AF_INET;
			if (inet_aton(myoptarg, &_src_addr.sin_addr)) {
				_src_addr_initialized = true;
			} else {
				warnx("invalid partner ip '%s'", myoptarg);
				err_flag = true;
			}
			break;
#else
			case 'u':
			case 'o':
			case 't':
				warnx("UDP options not supported on this platform");
				err_flag = true;
				break;
#endif

//		case 'e':
//			mavlink_link_termination_allowed = true;
//			break;

		case 'm':
			if (strcmp(myoptarg, "custom") == 0) {
				_mode = MAVLINK_MODE_CUSTOM;

			} else if (strcmp(myoptarg, "camera") == 0) {
				// left in here for compatibility
				_mode = MAVLINK_MODE_ONBOARD;

			} else if (strcmp(myoptarg, "onboard") == 0) {
				_mode = MAVLINK_MODE_ONBOARD;

			} else if (strcmp(myoptarg, "osd") == 0) {
				_mode = MAVLINK_MODE_OSD;

			} else if (strcmp(myoptarg, "magic") == 0) {
				_mode = MAVLINK_MODE_MAGIC;

			} else if (strcmp(myoptarg, "config") == 0) {
				_mode = MAVLINK_MODE_CONFIG;
			}

			break;

		case 'f':
			_forwarding_on = true;
			break;

		case 'v':
			_verbose = true;
			break;

		case 'w':
			_wait_to_transmit = true;
			break;

		case 'x':
			_ftp_on = true;
			break;

		default:
			err_flag = true;
			break;
		}
	}

	if (err_flag) {
		usage();
		return ERROR;
	}

	if (_datarate == 0) {
		/* convert bits to bytes and use 1/2 of bandwidth by default */
		_datarate = _baudrate / 20;
	}

	if (_datarate > MAX_DATA_RATE) {
		_datarate = MAX_DATA_RATE;
	}

	struct termios uart_config_original = {};

	if (get_protocol() == SERIAL) {
		if (Mavlink::instance_exists(_device_name, this)) {
			warnx("%s already running", _device_name);
			return ERROR;
		}

		warnx("mode: %u, data rate: %d B/s on %s @ %dB", _mode, _datarate, _device_name, _baudrate);

		/* flush stdout in case MAVLink is about to take it over */
		fflush(stdout);

		/* default values for arguments */
		_uart_fd = mavlink_open_uart(_baudrate, _device_name, &uart_config_original);

		if (_uart_fd < 0 && _mode != MAVLINK_MODE_CONFIG) {
			warn("could not open %s", _device_name);
			return ERROR;
		} else if (_uart_fd < 0 && _mode == MAVLINK_MODE_CONFIG) {
			/* the config link is optional */
			return OK;
		}

	} else if (get_protocol() == UDP) {
		if (Mavlink::get_instance_for_network_port(_network_port) != nullptr) {
			warnx("port %d already occupied", _network_port);
			return ERROR;
		}

		warnx("mode: %u, data rate: %d B/s on udp port %hu", _mode, _datarate, _network_port);
	}

	/* initialize send mutex */
	pthread_mutex_init(&_send_mutex, NULL);

	/* if we are passing on mavlink messages, we need to prepare a buffer for this instance */
	if (_forwarding_on || _ftp_on) {
		/* initialize message buffer if multiplexing is on or its needed for FTP.
		 * make space for two messages plus off-by-one space as we use the empty element
		 * marker ring buffer approach.
		 */
		if (OK != message_buffer_init(2 * sizeof(mavlink_message_t) + 1)) {
			warnx("msg buf:");
			return 1;
		}

		/* initialize message buffer mutex */
		pthread_mutex_init(&_message_buffer_mutex, NULL);
	}

	/* Initialize system properties */
	mavlink_update_system();

	/* start the MAVLink receiver */
	MavlinkReceiver::receive_start(&_receive_thread, this);

	MavlinkOrbSubscription *param_sub = add_orb_subscription(ORB_ID(parameter_update));
	uint64_t param_time = 0;
	MavlinkOrbSubscription *status_sub = add_orb_subscription(ORB_ID(vehicle_status));
	uint64_t status_time = 0;
	MavlinkOrbSubscription *ack_sub = add_orb_subscription(ORB_ID(vehicle_command_ack));
	uint64_t ack_time = 0;
	MavlinkOrbSubscription *mavlink_log_sub = add_orb_subscription(ORB_ID(mavlink_log));
	uint64_t mavlink_log_time = 0;

	struct vehicle_status_s status;
	status_sub->update(&status_time, &status);
	struct vehicle_command_ack_s command_ack;
	ack_sub->update(&ack_time, &command_ack);

	/* add default streams depending on mode */

	/* HEARTBEAT is constant rate stream, rate never adjusted */
	configure_stream("HEARTBEAT", 1.0f);

	/* STATUSTEXT stream is like normal stream but gets messages from logbuffer instead of uORB */
	configure_stream("STATUSTEXT", 20.0f);

	/* COMMAND_LONG stream: use high rate to avoid commands skipping */
	configure_stream("COMMAND_LONG", 100.0f);

	/* PARAM_VALUE stream */
	_parameters_manager = (MavlinkParametersManager *) MavlinkParametersManager::new_instance(this);
	_parameters_manager->set_interval(interval_from_rate(120.0f));
	LL_APPEND(_streams, _parameters_manager);

	/* MAVLINK_FTP stream */
	_mavlink_ftp = (MavlinkFTP *) MavlinkFTP::new_instance(this);
	_mavlink_ftp->set_interval(interval_from_rate(80.0f));
	LL_APPEND(_streams, _mavlink_ftp);

	/* MAVLINK_Log_Handler */
	_mavlink_log_handler = (MavlinkLogHandler *) MavlinkLogHandler::new_instance(this);
	_mavlink_log_handler->set_interval(interval_from_rate(80.0f));
	LL_APPEND(_streams, _mavlink_log_handler);

	/* MISSION_STREAM stream, actually sends all MISSION_XXX messages at some rate depending on
	 * remote requests rate. Rate specified here controls how much bandwidth we will reserve for
	 * mission messages. */
	_mission_manager = (MavlinkMissionManager *) MavlinkMissionManager::new_instance(this);
	_mission_manager->set_interval(interval_from_rate(10.0f));
	_mission_manager->set_verbose(_verbose);
	LL_APPEND(_streams, _mission_manager);

	switch (_mode) {
	case MAVLINK_MODE_NORMAL:
		configure_stream("SYS_STATUS", 1.0f);
		configure_stream("HOME_POSITION", 0.5f);
		configure_stream("HIGHRES_IMU", 2.0f);
		configure_stream("ATTITUDE", 20.0f);
		configure_stream("VFR_HUD", 8.0f);
		configure_stream("GPS_RAW_INT", 1.0f);
		configure_stream("GLOBAL_POSITION_INT", 3.0f);
		configure_stream("LOCAL_POSITION_NED", 3.0f);
		configure_stream("RC_CHANNELS", 1.0f);
		configure_stream("SERVO_OUTPUT_RAW_0", 1.0f);
		configure_stream("POSITION_TARGET_GLOBAL_INT", 3.0f);
		configure_stream("ATTITUDE_TARGET", 8.0f);
		configure_stream("DISTANCE_SENSOR", 0.5f);
		configure_stream("OPTICAL_FLOW_RAD", 5.0f);
		configure_stream("EXTENDED_SYS_STATE", 1.0f);
		configure_stream("ALTITUDE", 1.0f);
		configure_stream("VISION_POSITION_NED", 10.0f);
		configure_stream("NAMED_VALUE_FLOAT", 1.0f);
		configure_stream("ESTIMATOR_STATUS", 0.5f);
		break;

	case MAVLINK_MODE_ONBOARD:
		configure_stream("SYS_STATUS", 1.0f);
		configure_stream("ATTITUDE", 250.0f);
		configure_stream("HIGHRES_IMU", 50.0f);
		configure_stream("GPS_RAW_INT", 5.0f);
		configure_stream("GLOBAL_POSITION_INT", 50.0f);
		configure_stream("LOCAL_POSITION_NED", 30.0f);
		configure_stream("NAMED_VALUE_FLOAT", 10.0f);
		configure_stream("CAMERA_CAPTURE", 2.0f);
		configure_stream("HOME_POSITION", 0.5f);
		configure_stream("ATTITUDE_TARGET", 10.0f);
		configure_stream("POSITION_TARGET_GLOBAL_INT", 10.0f);
		configure_stream("POSITION_TARGET_LOCAL_NED", 10.0f);
		configure_stream("DISTANCE_SENSOR", 10.0f);
		configure_stream("OPTICAL_FLOW_RAD", 10.0f);
		configure_stream("RC_CHANNELS", 20.0f);
		configure_stream("SERVO_OUTPUT_RAW_0", 10.0f);
		configure_stream("VFR_HUD", 10.0f);
		configure_stream("SYSTEM_TIME", 1.0f);
		configure_stream("TIMESYNC", 10.0f);
		configure_stream("ACTUATOR_CONTROL_TARGET0", 10.0f);
		//camera trigger is rate limited at the source, do not limit here
		configure_stream("CAMERA_TRIGGER", 500.0f);
		configure_stream("EXTENDED_SYS_STATE", 2.0f);
		configure_stream("ALTITUDE", 10.0f);
		configure_stream("VISION_POSITION_NED", 10.0f);
		configure_stream("NAMED_VALUE_FLOAT", 10.0f);
		configure_stream("ESTIMATOR_STATUS", 1.0f);
		break;

	case MAVLINK_MODE_OSD:
		configure_stream("SYS_STATUS", 5.0f);
		configure_stream("ATTITUDE", 25.0f);
		configure_stream("VFR_HUD", 25.0f);
		configure_stream("GPS_RAW_INT", 1.0f);
		configure_stream("GLOBAL_POSITION_INT", 10.0f);
		configure_stream("HOME_POSITION", 0.5f);
		configure_stream("ATTITUDE_TARGET", 10.0f);
		configure_stream("SYSTEM_TIME", 1.0f);
		configure_stream("RC_CHANNELS", 5.0f);
		configure_stream("SERVO_OUTPUT_RAW_0", 1.0f);
		configure_stream("EXTENDED_SYS_STATE", 1.0f);
		configure_stream("ALTITUDE", 1.0f);
		configure_stream("ESTIMATOR_STATUS", 1.0f);
		break;

	case MAVLINK_MODE_MAGIC:
		//stream nothing
		break;

	case MAVLINK_MODE_CONFIG:
		// Enable a number of interesting streams we want via USB
		configure_stream("SYS_STATUS", 1.0f);
		configure_stream("HOME_POSITION", 0.5f);
		configure_stream("GLOBAL_POSITION_INT", 10.0f);
		configure_stream("ATTITUDE_TARGET", 8.0f);
		//configure_stream("PARAM_VALUE", 300.0f);
		configure_stream("MISSION_ITEM", 50.0f);
		configure_stream("NAMED_VALUE_FLOAT", 50.0f);
		configure_stream("OPTICAL_FLOW_RAD", 10.0f);
		configure_stream("DISTANCE_SENSOR", 10.0f);
		configure_stream("VFR_HUD", 20.0f);
		configure_stream("ATTITUDE", 100.0f);
		configure_stream("ACTUATOR_CONTROL_TARGET0", 30.0f);
		configure_stream("RC_CHANNELS", 10.0f);
		configure_stream("SERVO_OUTPUT_RAW_0", 20.0f);
		configure_stream("SERVO_OUTPUT_RAW_1", 20.0f);
		configure_stream("POSITION_TARGET_GLOBAL_INT", 10.0f);
		configure_stream("LOCAL_POSITION_NED", 30.0f);
		configure_stream("MANUAL_CONTROL", 5.0f);
		configure_stream("HIGHRES_IMU", 50.0f);
		configure_stream("GPS_RAW_INT", 20.0f);
		configure_stream("CAMERA_TRIGGER", 500.0f);
		configure_stream("EXTENDED_SYS_STATE", 2.0f);
		configure_stream("ALTITUDE", 10.0f);
		configure_stream("VISION_POSITION_NED", 10.0f);
		configure_stream("NAMED_VALUE_FLOAT", 50.0f);
		configure_stream("ESTIMATOR_STATUS", 5.0f);
	default:
		break;
	}

	/* set main loop delay depending on data rate to minimize CPU overhead */
	_main_loop_delay = (MAIN_LOOP_DELAY * 1000) / _datarate;

	/* hard limit to 500 Hz at max */
	if (_main_loop_delay < 2000) {
		_main_loop_delay = 2000;
	}

	/* now the instance is fully initialized and we can bump the instance count */
	LL_APPEND(_mavlink_instances, this);

	/* init socket if necessary */
	if (get_protocol() == UDP) {
		init_udp();
	}

	/* if the protocol is serial, we send the system version blindly */
	if (get_protocol() == SERIAL) {
		send_autopilot_capabilites();
	}

	while (!_task_should_exit) {
		/* main loop */
		usleep(_main_loop_delay);

		perf_begin(_loop_perf);

		hrt_abstime t = hrt_absolute_time();

		update_rate_mult();

		_mission_manager->check_active_mission();

		if (param_sub->update(&param_time, nullptr)) {
			/* parameters updated */
			mavlink_update_system();
		}

		/* radio config check */
		if (_uart_fd >= 0 && _radio_id != 0 && _rstatus.type == telemetry_status_s::TELEMETRY_STATUS_RADIO_TYPE_3DR_RADIO) {
			/* request to configure radio and radio is present */
			FILE *fs = fdopen(_uart_fd, "w");

			if (fs) {
				/* switch to AT command mode */
				usleep(1200000);
				fprintf(fs, "+++\n");
				usleep(1200000);

				if (_radio_id > 0) {
					/* set channel */
					fprintf(fs, "ATS3=%u\n", _radio_id);
					usleep(200000);
				} else {
					/* reset to factory defaults */
					fprintf(fs, "AT&F\n");
					usleep(200000);
				}

				/* write config */
				fprintf(fs, "AT&W");
				usleep(200000);

				/* reboot */
				fprintf(fs, "ATZ");
				usleep(200000);

				// XXX NuttX suffers from a bug where
				// fclose() also closes the fd, not just
				// the file stream. Since this is a one-time
				// config thing, we leave the file struct
				// allocated.
				//fclose(fs);
			} else {
				warnx("open fd %d failed", _uart_fd);
			}

			/* reset param and save */
			_radio_id = 0;
			param_set(_param_radio_id, &_radio_id);
		}

		if (status_sub->update(&status_time, &status)) {
			/* switch HIL mode if required */
			set_hil_enabled(status.hil_state == vehicle_status_s::HIL_STATE_ON);

			set_manual_input_mode_generation(status.rc_input_mode == vehicle_status_s::RC_IN_MODE_GENERATED);
		}

		/* send command ACK */
		if (ack_sub->update(&ack_time, &command_ack)) {
			mavlink_command_ack_t msg;
			msg.result = command_ack.result;
			msg.command = command_ack.command;

			send_message(MAVLINK_MSG_ID_COMMAND_ACK, &msg);
		}

		struct mavlink_log_s mavlink_log;
		if (mavlink_log_sub->update(&mavlink_log_time, &mavlink_log)) {
			_logbuffer.put(&mavlink_log);
		}

		/* check for requested subscriptions */
		if (_subscribe_to_stream != nullptr) {
			if (OK == configure_stream(_subscribe_to_stream, _subscribe_to_stream_rate)) {
				if (_subscribe_to_stream_rate > 0.0f) {
					if ( get_protocol() == SERIAL ) {
						warnx("stream %s on device %s enabled with rate %.1f Hz", _subscribe_to_stream, _device_name,
							(double)_subscribe_to_stream_rate);
					} else if ( get_protocol() == UDP ) {
						warnx("stream %s on UDP port %d enabled with rate %.1f Hz", _subscribe_to_stream, _network_port,
							(double)_subscribe_to_stream_rate);
					}

				} else {
					if ( get_protocol() == SERIAL ) {
						warnx("stream %s on device %s disabled", _subscribe_to_stream, _device_name);
					} else if ( get_protocol() == UDP ) {
						warnx("stream %s on UDP port %d disabled", _subscribe_to_stream, _network_port);
					}
				}

			} else {
				if ( get_protocol() == SERIAL ) {
					warnx("stream %s on device %s not found", _subscribe_to_stream, _device_name);
				} else if ( get_protocol() == UDP ) {
					warnx("stream %s on UDP port %d not found", _subscribe_to_stream, _network_port);
				}
			}

			_subscribe_to_stream = nullptr;
		}

		/* update streams */
		MavlinkStream *stream;
		LL_FOREACH(_streams, stream) {
			stream->update(t);
		}

		/* pass messages from other UARTs or FTP worker */
		if (_forwarding_on || _ftp_on) {

			bool is_part;
			uint8_t *read_ptr;
			uint8_t *write_ptr;

			pthread_mutex_lock(&_message_buffer_mutex);
			int available = message_buffer_get_ptr((void **)&read_ptr, &is_part);
			pthread_mutex_unlock(&_message_buffer_mutex);

			if (available > 0) {
				// Reconstruct message from buffer

				mavlink_message_t msg;
				write_ptr = (uint8_t *)&msg;

				// Pull a single message from the buffer
				size_t read_count = available;

				if (read_count > sizeof(mavlink_message_t)) {
					read_count = sizeof(mavlink_message_t);
				}

				memcpy(write_ptr, read_ptr, read_count);

				// We hold the mutex until after we complete the second part of the buffer. If we don't
				// we may end up breaking the empty slot overflow detection semantics when we mark the
				// possibly partial read below.
				pthread_mutex_lock(&_message_buffer_mutex);

				message_buffer_mark_read(read_count);

				/* write second part of buffer if there is some */
				if (is_part && read_count < sizeof(mavlink_message_t)) {
					write_ptr += read_count;
					available = message_buffer_get_ptr((void **)&read_ptr, &is_part);
					read_count = sizeof(mavlink_message_t) - read_count;
					memcpy(write_ptr, read_ptr, read_count);
					message_buffer_mark_read(available);
				}

				pthread_mutex_unlock(&_message_buffer_mutex);

				resend_message(&msg);
			}
		}

		/* update TX/RX rates*/
		if (t > _bytes_timestamp + 1000000) {
			if (_bytes_timestamp != 0) {
				float dt = (t - _bytes_timestamp) / 1000.0f;
				_rate_tx = _bytes_tx / dt;
				_rate_txerr = _bytes_txerr / dt;
				_rate_rx = _bytes_rx / dt;
				_bytes_tx = 0;
				_bytes_txerr = 0;
				_bytes_rx = 0;
			}

			_bytes_timestamp = t;
		}

		perf_end(_loop_perf);

		/* confirm task running only once fully initialized */
		_task_running = true;
	}

	/* first wait for threads to complete before tearing down anything */
	pthread_join(_receive_thread, NULL);

	delete _subscribe_to_stream;
	_subscribe_to_stream = nullptr;

	/* delete streams */
	MavlinkStream *stream_to_del = nullptr;
	MavlinkStream *stream_next = _streams;

	while (stream_next != nullptr) {
		stream_to_del = stream_next;
		stream_next = stream_to_del->next;
		delete stream_to_del;
	}

	_streams = nullptr;

	/* delete subscriptions */
	MavlinkOrbSubscription *sub_to_del = nullptr;
	MavlinkOrbSubscription *sub_next = _subscriptions;

	while (sub_next != nullptr) {
		sub_to_del = sub_next;
		sub_next = sub_to_del->next;
		delete sub_to_del;
	}

	_subscriptions = nullptr;

	if (_uart_fd >= 0 && !_is_usb_uart) {
		/* reset the UART flags to original state */
		tcsetattr(_uart_fd, TCSANOW, &uart_config_original);

		/* close UART */
		::close(_uart_fd);
	}

	if (_forwarding_on || _ftp_on) {
		message_buffer_destroy();
		pthread_mutex_destroy(&_message_buffer_mutex);
	}

	warnx("exiting");
	_task_running = false;

	return OK;
}

int Mavlink::start_helper(int argc, char *argv[])
{
	/* create the instance in task context */
	Mavlink *instance = new Mavlink();

	int res;

	if (!instance) {

		/* out of memory */
		res = -ENOMEM;
		warnx("OUT OF MEM");

	} else {
		/* this will actually only return once MAVLink exits */
		res = instance->task_main(argc, argv);

		/* delete instance on main thread end */
		delete instance;
	}

	return res;
}

int
Mavlink::start(int argc, char *argv[])
{
	// Wait for the instance count to go up one
	// before returning to the shell
	int ic = Mavlink::instance_count();

	if (ic == MAVLINK_COMM_NUM_BUFFERS) {
		warnx("Maximum MAVLink instance count of %d reached.",
			(int)MAVLINK_COMM_NUM_BUFFERS);
		return 1;
	}

	// Instantiate thread
	char buf[24];
	sprintf(buf, "mavlink_if%d", ic);

	// This is where the control flow splits
	// between the starting task and the spawned
	// task - start_helper() only returns
	// when the started task exits.
	px4_task_spawn_cmd(buf,
			   SCHED_DEFAULT,
			   SCHED_PRIORITY_DEFAULT,
			   2700,
			   (px4_main_t)&Mavlink::start_helper,
			   (char *const *)argv);

	// Ensure that this shell command
	// does not return before the instance
	// is fully initialized. As this is also
	// the only path to create a new instance,
	// this is effectively a lock on concurrent
	// instance starting. XXX do a real lock.

	// Sleep 500 us between each attempt
	const unsigned sleeptime = 500;

	// Wait 100 ms max for the startup.
	const unsigned limit = 100 * 1000 / sleeptime;

	unsigned count = 0;

	while (ic == Mavlink::instance_count() && count < limit) {
		::usleep(sleeptime);
		count++;
	}

	return OK;
}

void
Mavlink::display_status()
{

	if (_rstatus.heartbeat_time > 0) {
		printf("\tGCS heartbeat:\t%llu us ago\n", (unsigned long long)hrt_elapsed_time(&_rstatus.heartbeat_time));
	}

	printf("\tmavlink chan: #%u\n", _channel);

	if (_rstatus.timestamp > 0) {

		printf("\ttype:\t\t");

		switch (_rstatus.type) {
		case telemetry_status_s::TELEMETRY_STATUS_RADIO_TYPE_3DR_RADIO:
			printf("3DR RADIO\n");
			break;

		case telemetry_status_s::TELEMETRY_STATUS_RADIO_TYPE_USB:
			printf("USB CDC\n");
			break;

		default:
			printf("GENERIC LINK OR RADIO\n");
			break;
		}

		printf("\trssi:\t\t%d\n", _rstatus.rssi);
		printf("\tremote rssi:\t%u\n", _rstatus.remote_rssi);
		printf("\ttxbuf:\t\t%u\n", _rstatus.txbuf);
		printf("\tnoise:\t\t%d\n", _rstatus.noise);
		printf("\tremote noise:\t%u\n", _rstatus.remote_noise);
		printf("\trx errors:\t%u\n", _rstatus.rxerrors);
		printf("\tfixed:\t\t%u\n", _rstatus.fixed);
		printf("\tflow control:\t%s\n", (_flow_control_enabled) ? "ON" : "OFF");

	} else {
		printf("\tno telem status.\n");
	}

	printf("\trates:\n");
	printf("\ttx: %.3f kB/s\n", (double)_rate_tx);
	printf("\ttxerr: %.3f kB/s\n", (double)_rate_txerr);
	printf("\trx: %.3f kB/s\n", (double)_rate_rx);
	printf("\trate mult: %.3f\n", (double)_rate_mult);
}

int
Mavlink::stream_command(int argc, char *argv[])
{
	const char *device_name = DEFAULT_DEVICE_NAME;
	float rate = -1.0f;
	const char *stream_name = nullptr;
	unsigned short network_port = 0;
	char* eptr;
	int temp_int_arg;
	bool provided_device = false;
	bool provided_network_port = false;

	/*
	 * Called via main with original argv
	 *   mavlink start
	 *
	 *  Remove 2
	 */
	argc -= 2;
	argv += 2;

	/* don't exit from getopt loop to leave getopt global variables in consistent state,
	 * set error flag instead */
	bool err_flag = false;

	int i = 0;

	while (i < argc) {

		if (0 == strcmp(argv[i], "-r") && i < argc - 1) {
			rate = strtod(argv[i + 1], nullptr);

			if (rate < 0.0f) {
				err_flag = true;
			}

			i++;

		} else if (0 == strcmp(argv[i], "-d") && i < argc - 1) {
			provided_device = true;
			device_name = argv[i + 1];
			i++;

		} else if (0 == strcmp(argv[i], "-s") && i < argc - 1) {
			stream_name = argv[i + 1];
			i++;
		} else if (0 == strcmp(argv[i], "-u") && i < argc - 1) {
			provided_network_port = true;
			temp_int_arg = strtoul(argv[i + 1], &eptr, 10);
			if ( *eptr == '\0' ) {
				network_port = temp_int_arg;
			} else {
				err_flag = true;
			}
			i++;
		} else {
			err_flag = true;
		}

		i++;
	}

	if (!err_flag && rate >= 0.0f && stream_name != nullptr) {

		Mavlink *inst = nullptr;
		if (provided_device && !provided_network_port) {
			inst = get_instance_for_device(device_name);
		} else if (provided_network_port && !provided_device) {
			inst = get_instance_for_network_port(network_port);
		} else if (provided_device && provided_network_port) {
			warnx("please provide either a device name or a network port");
			return 1;
		}

		if (inst != nullptr) {
			inst->configure_stream_threadsafe(stream_name, rate);

		} else {

			// If the link is not running we should complain, but not fall over
			// because this is so easy to get wrong and not fatal. Warning is sufficient.
			if (provided_device) {
				warnx("mavlink for device %s is not running", device_name);
			} else {
				warnx("mavlink for network on port %hu is not running", network_port);
			}

			return 1;
		}

	} else {
		warnx("usage: mavlink stream [-d device] [-u network_port] -s stream -r rate");
		return 1;
	}

	return OK;
}

static void usage()
{
	warnx("usage: mavlink {start|stop|stream} [-d device] [-u network_port] [-o remote_port] [-t partner_ip] [-b baudrate]\n\t[-r rate][-m mode] [-s stream] [-f] [-p] [-v] [-w] [-x]");
}

int mavlink_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		return Mavlink::start(argc, argv);

	} else if (!strcmp(argv[1], "stop")) {
		warnx("mavlink stop is deprecated, use stop-all instead");
		usage();
		return 1;

	} else if (!strcmp(argv[1], "stop") || !strcmp(argv[1], "stop-all") ) {
		return Mavlink::destroy_all_instances();

	} else if (!strcmp(argv[1], "status")) {
		return Mavlink::get_status_all_instances();

	} else if (!strcmp(argv[1], "stream")) {
		return Mavlink::stream_command(argc, argv);

	} else if (!strcmp(argv[1], "boot_complete")) {
		Mavlink::set_boot_complete();
		return 0;

	} else {
		usage();
		return 1;
	}

	return 0;
}



