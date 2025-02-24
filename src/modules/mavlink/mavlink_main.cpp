/****************************************************************************
 *
 *   Copyright (c) 2012-2023 PX4 Development Team. All rights reserved.
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
 * @file mavlink_main.cpp
 * MAVLink 1.0 protocol implementation.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <termios.h>

#ifdef CONFIG_NET
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netutils/netlib.h>
#endif

#include <containers/LockGuard.hpp>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/systemlib/mavlink_log.h>
#include <lib/version/version.h>

#include <px4_platform_common/events.h>

#include <uORB/topics/event.h>
#include "mavlink_receiver.h"
#include "mavlink_main.h"

// Guard against MAVLink misconfiguration
#ifndef MAVLINK_CRC_EXTRA
#error MAVLINK_CRC_EXTRA has to be defined on PX4 systems
#endif

// Guard against flow control misconfiguration
#if defined (CRTSCTS) && defined (__PX4_NUTTX) && (CRTSCTS != (CRTS_IFLOW | CCTS_OFLOW))
#error The non-standard CRTSCTS define is incorrect. Fix this in the OS or replace with (CRTS_IFLOW | CCTS_OFLOW)
#endif

#ifdef CONFIG_NET
#define MAVLINK_NET_ADDED_STACK PX4_STACK_ADJUSTED(350)
#else
#define MAVLINK_NET_ADDED_STACK 0
#endif

#define FLOW_CONTROL_DISABLE_THRESHOLD 40              ///< picked so that some messages still would fit it.
#define MAX_DATA_RATE                  10000000        ///< max data rate in bytes/s
#define MAIN_LOOP_DELAY                10000           ///< 100 Hz @ 1000 bytes/s data rate

static pthread_mutex_t mavlink_module_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mavlink_event_buffer_mutex = PTHREAD_MUTEX_INITIALIZER;
events::EventBuffer *Mavlink::_event_buffer = nullptr;

Mavlink *mavlink_module_instances[MAVLINK_COMM_NUM_BUFFERS] {};

void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, int length) { mavlink_module_instances[chan]->send_bytes(ch, length); }
void mavlink_start_uart_send(mavlink_channel_t chan, int length) { mavlink_module_instances[chan]->send_start(length); }
void mavlink_end_uart_send(mavlink_channel_t chan, int length) { mavlink_module_instances[chan]->send_finish(); }
mavlink_status_t *mavlink_get_channel_status(uint8_t channel) { return mavlink_module_instances[channel]->get_status(); }
mavlink_message_t *mavlink_get_channel_buffer(uint8_t channel) { return mavlink_module_instances[channel]->get_buffer(); }

static void usage();

hrt_abstime Mavlink::_first_start_time = {0};

bool Mavlink::_boot_complete = false;

Mavlink::Mavlink() :
	ModuleParams(nullptr),
	_receiver(*this)
{
	// initialise parameter cache
	mavlink_update_parameters();

	// save the current system- and component ID because we don't allow them to change during operation
	int sys_id = _param_mav_sys_id.get();

	if (sys_id > 0 && sys_id < 255) {
		mavlink_system.sysid = sys_id;
	}

	int comp_id = _param_mav_comp_id.get();

	if (comp_id > 0 && comp_id < 255) {
		mavlink_system.compid = comp_id;
	}

	if (_first_start_time == 0) {
		_first_start_time = hrt_absolute_time();
	}

	// ensure topic exists, otherwise we might lose first queued commands
	if (orb_exists(ORB_ID(vehicle_command), 0) == PX4_ERROR) {
		orb_advertise(ORB_ID(vehicle_command), nullptr);
	}

	_vehicle_command_sub.subscribe();

	if (orb_exists(ORB_ID(event), 0) == PX4_ERROR) {
		orb_advertise(ORB_ID(event), nullptr);
	}

	_event_sub.subscribe();
	_telemetry_status_pub.advertise();
}

Mavlink::~Mavlink()
{
	if (running()) {
		/* task wakes up every 10ms or so at the longest */
		request_stop();

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait at least 1 second (10ms * 10) */
			px4_usleep(10000);

			/* if we have given up, kill it */
			if (++i > 100) {
				PX4_ERR("mavlink didn't stop, killing task %d", _task_id);
				px4_task_delete(_task_id);
				break;
			}
		} while (running());
	}

	if (_instance_id >= 0) {
		mavlink_module_instances[_instance_id] = nullptr;
	}

	// if this instance was responsible for checking events then select a new mavlink instance
	if (check_events()) {
		check_events_disable();

		// select next available instance
		for (Mavlink *inst : mavlink_module_instances) {
			if (inst) {
				inst->check_events_enable();
				break;
			}
		}
	}

	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
	perf_free(_send_byte_error_perf);
	perf_free(_forwarding_error_perf);
}

void
Mavlink::mavlink_update_parameters()
{
	updateParams();

	int32_t proto = _param_mav_proto_ver.get();

	if (_protocol_version_switch != proto) {
		_protocol_version_switch = proto;
		set_proto_version(proto);
	}

	if (_param_mav_type.get() < 0 || _param_mav_type.get() >= MAV_TYPE_ENUM_END) {
		_param_mav_type.set(0);
		_param_mav_type.commit_no_notification();
		PX4_ERR("MAV_TYPE parameter invalid, resetting to 0.");
	}
}

bool Mavlink::set_channel()
{
	/* set channel according to instance id */
	switch (_instance_id) {
	case 0:
		_channel = MAVLINK_COMM_0;
		return true;

	case 1:
		_channel = MAVLINK_COMM_1;
		return true;

	case 2:
		_channel = MAVLINK_COMM_2;
		return true;

	case 3:
		_channel = MAVLINK_COMM_3;
		return true;
#ifdef MAVLINK_COMM_4

	case 4:
		_channel = MAVLINK_COMM_4;
		return true;
#endif
#ifdef MAVLINK_COMM_5

	case 5:
		_channel = MAVLINK_COMM_5;
		return true;
#endif
#ifdef MAVLINK_COMM_6

	case 6:
		_channel = MAVLINK_COMM_6;
		return true;
#endif

	default:
		PX4_ERR("instance ID %d is out of range", _instance_id);
		break;
	}

	return false;
}

bool
Mavlink::set_instance_id()
{
	LockGuard lg{mavlink_module_mutex};

	// instance count
	size_t inst_count = 0;

	for (Mavlink *inst : mavlink_module_instances) {
		if (inst != nullptr) {
			inst_count++;
		}
	}

	// if this is the first instance use it to check events
	if (inst_count == 0) {
		check_events_enable();
	}

	for (int instance_id = 0; instance_id < MAVLINK_COMM_NUM_BUFFERS; instance_id++) {
		if (mavlink_module_instances[instance_id] == nullptr) {
			mavlink_module_instances[instance_id] = this;
			_instance_id = instance_id;
			return true;
		}
	}

	return false;
}

void
Mavlink::set_proto_version(unsigned version)
{
	if ((version == 1 || version == 0) &&
	    ((_protocol_version_switch == 0) || (_protocol_version_switch == 1))) {
		get_status()->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
		_protocol_version = 1;

	} else if (version == 2 &&
		   ((_protocol_version_switch == 0) || (_protocol_version_switch == 2))) {
		get_status()->flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
		_protocol_version = 2;
	}
}

int
Mavlink::instance_count()
{
	LockGuard lg{mavlink_module_mutex};
	size_t inst_index = 0;

	for (Mavlink *inst : mavlink_module_instances) {
		if (inst != nullptr) {
			inst_index++;
		}
	}

	return inst_index;
}

Mavlink *
Mavlink::get_instance_for_device(const char *device_name)
{
	LockGuard lg{mavlink_module_mutex};

	for (Mavlink *inst : mavlink_module_instances) {
		if (inst && (inst->_protocol == Protocol::SERIAL) && (strcmp(inst->_device_name, device_name) == 0)) {
			return inst;
		}
	}

	return nullptr;
}

#ifdef MAVLINK_UDP
Mavlink *
Mavlink::get_instance_for_network_port(unsigned long port)
{
	LockGuard lg{mavlink_module_mutex};

	for (Mavlink *inst : mavlink_module_instances) {
		if (inst && (inst->_protocol == Protocol::UDP) && (inst->_network_port == port)) {
			return inst;
		}
	}

	return nullptr;
}
#endif // MAVLINK_UDP

int
Mavlink::destroy_all_instances()
{
	PX4_INFO("waiting for instances to stop");

	unsigned iterations = 0;

	while (iterations < 1000) {
		int running = 0;

		pthread_mutex_lock(&mavlink_module_mutex);

		for (Mavlink *inst_to_del : mavlink_module_instances) {
			if (inst_to_del != nullptr) {
				if (inst_to_del->running()) {
					running++;

					// set flag to stop thread and wait for all threads to finish
					inst_to_del->request_stop();
				}
			}
		}

		pthread_mutex_unlock(&mavlink_module_mutex);

		if (running == 0) {
			break;

		} else if (iterations > 1000) {
			PX4_ERR("Couldn't stop all mavlink instances.");
			return PX4_ERROR;

		} else {
			iterations++;
			printf(".");
			fflush(stdout);
			px4_usleep(10000);
		}
	}

	{
		LockGuard lg{mavlink_module_mutex};

		// we know all threads have exited, so it's safe to delete objects.
		for (Mavlink *inst_to_del : mavlink_module_instances) {
			delete inst_to_del;
		}
	}

	{
		LockGuard lg{mavlink_event_buffer_mutex};

		delete _event_buffer;
		_event_buffer = nullptr;
	}

	PX4_INFO("all instances stopped");
	return OK;
}

int
Mavlink::get_status_all_instances(bool show_streams_status)
{
	LockGuard lg{mavlink_module_mutex};
	unsigned iterations = 0;

	for (Mavlink *inst : mavlink_module_instances) {
		if (inst != nullptr) {
			printf("\ninstance #%u:\n", iterations);

			if (show_streams_status) {
				inst->display_status_streams();

			} else {
				inst->display_status();
			}

			iterations++;
		}
	}

	/* return an error if there are no instances */
	return (iterations == 0);
}

bool
Mavlink::serial_instance_exists(const char *device_name, Mavlink *self)
{
	LockGuard lg{mavlink_module_mutex};

	for (Mavlink *inst : mavlink_module_instances) {
		/* don't compare with itself and with non serial instances*/
		if (inst && (inst != self) && (inst->get_protocol() == Protocol::SERIAL) && !strcmp(device_name, inst->_device_name)) {
			return true;
		}
	}

	return false;
}

bool
Mavlink::component_was_seen(int system_id, int component_id, Mavlink &self)
{
	LockGuard lg{mavlink_module_mutex};

	for (Mavlink *inst : mavlink_module_instances) {
		if (inst && (inst != &self) && (inst->_receiver.component_was_seen(system_id, component_id))) {
			return true;
		}
	}

	return false;
}

void
Mavlink::forward_message(const mavlink_message_t *msg, Mavlink *self)
{
	const mavlink_msg_entry_t *meta = mavlink_get_msg_entry(msg->msgid);

	int target_system_id = 0;
	int target_component_id = 0;

	// might be nullptr if message is unknown
	if (meta) {
		// Extract target system and target component if set
		if (meta->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_SYSTEM) {
			target_system_id = static_cast<uint8_t>((_MAV_PAYLOAD(msg))[meta->target_system_ofs]);
		}

		if (meta->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_COMPONENT) {
			target_component_id = static_cast<uint8_t>((_MAV_PAYLOAD(msg))[meta->target_component_ofs]);
		}
	}

	// If it's a message only for us, we keep it
	if (target_system_id == self->get_system_id() && target_component_id == self->get_component_id()) {
		return;
	}

	// We don't forward heartbeats unless it's specifically enabled.
	if (msg->msgid == MAVLINK_MSG_ID_HEARTBEAT && !self->forward_heartbeats_enabled()) {
		return;
	}

	LockGuard lg{mavlink_module_mutex};

	for (Mavlink *inst : mavlink_module_instances) {
		if (inst && (inst != self) && (inst->get_forwarding_on())) {
			// Pass message only if target component was seen before
			if (inst->_receiver.component_was_seen(target_system_id, target_component_id)) {
				inst->pass_message(msg);
			}
		}
	}
}

int
Mavlink::mavlink_open_uart(const int baud, const char *uart_name, const FLOW_CONTROL_MODE flow_control)
{
#ifndef B460800
#define B460800 460800
#endif

#ifndef B500000
#define B500000 500000
#endif

#ifndef B921600
#define B921600 921600
#endif

#ifndef B1000000
#define B1000000 1000000
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

	case 500000: speed = B500000; break;

	case 921600: speed = B921600; break;

	case 1000000: speed = B1000000; break;

#ifdef B1500000

	case 1500000: speed = B1500000; break;
#endif

#ifdef B2000000

	case 2000000: speed = B2000000; break;
#endif

#ifdef B3000000

	case 3000000: speed = B3000000; break;
#endif

	default:
		PX4_ERR("Unsupported baudrate: %d\n\tsupported examples:\n\t9600, 19200, 38400, 57600\t\n115200\n230400\n460800\n500000\n921600\n1000000\n",
			baud);
		return -EINVAL;
	}

	/* open uart */
	_uart_fd = ::open(uart_name, O_RDWR | O_NOCTTY);

	/*
	 * Return here in the iridium mode since the iridium driver does not
	 * support the subsequent function calls.
	*/
	if (_uart_fd < 0 || _mode == MAVLINK_MODE_IRIDIUM) {
		return _uart_fd;
	}

	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;

	/* Initialize the uart config */
	if ((termios_state = tcgetattr(_uart_fd, &uart_config)) < 0) {
		PX4_ERR("ERR GET CONF %s: %d\n", uart_name, termios_state);
		::close(_uart_fd);
		return -1;
	}

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* Set baud rate */
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		PX4_ERR("ERR SET BAUD %s: %d\n", uart_name, termios_state);
		::close(_uart_fd);
		return -1;
	}

#if defined(__PX4_LINUX) || defined(__PX4_DARWIN) || defined(__PX4_CYGWIN)
	/* Put in raw mode */
	cfmakeraw(&uart_config);
#endif

	if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
		PX4_WARN("ERR SET CONF %s\n", uart_name);
		::close(_uart_fd);
		return -1;
	}

	/* setup hardware flow control */
	if (setup_flow_control(flow_control) && (flow_control != FLOW_CONTROL_AUTO)) {
		PX4_WARN("hardware flow control not supported");
	}

	return _uart_fd;
}

int
Mavlink::setup_flow_control(enum FLOW_CONTROL_MODE mode)
{
	struct termios uart_config;

	int ret = tcgetattr(_uart_fd, &uart_config);

	if (mode != FLOW_CONTROL_OFF) {
		uart_config.c_cflag |= CRTSCTS;

	} else {
		uart_config.c_cflag &= ~CRTSCTS;

	}

	ret = tcsetattr(_uart_fd, TCSANOW, &uart_config);

	if (!ret) {
		_flow_control_mode = mode;
	}

	return ret;
}

int
Mavlink::set_hil_enabled(bool hil_enabled)
{
	int ret = OK;

	/* enable HIL (only on links with sufficient bandwidth) */
	if (hil_enabled && !_hil_enabled && _datarate > 5000) {
		_hil_enabled = true;
		ret = configure_stream("HIL_ACTUATOR_CONTROLS", 200.0f);

		if (_param_sys_hitl.get() == 2) {		// Simulation in Hardware enabled ?
			configure_stream("HIL_STATE_QUATERNION", 25.0f); // ground truth to display the SIH

		} else {
			configure_stream("HIL_STATE_QUATERNION", 0.0f);
		}
	}

	/* disable HIL */
	if (!hil_enabled && _hil_enabled) {
		_hil_enabled = false;
		ret = configure_stream("HIL_ACTUATOR_CONTROLS", 0.0f);

		configure_stream("HIL_STATE_QUATERNION", 0.0f);
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

#if defined(MAVLINK_UDP)

	// if we are using network sockets, return max length of one packet
	if (get_protocol() == Protocol::UDP) {
# if defined(__PX4_POSIX)
		return  1500 * 10; // Speed up FTP transfers
# else
		return  1500;
# endif /* defined(__PX4_POSIX) */

	} else
#endif // MAVLINK_UDP
	{

#if defined(__PX4_NUTTX)
		(void) ioctl(_uart_fd, FIONSPACE, (unsigned long)&buf_free);
#else
		// No FIONSPACE on Linux todo:use SIOCOUTQ  and queue size to emulate FIONSPACE
		//Linux cp210x does not support TIOCOUTQ
		buf_free = MAVLINK_MAX_PACKET_LEN;
#endif

		if (_flow_control_mode == FLOW_CONTROL_AUTO && buf_free < FLOW_CONTROL_DISABLE_THRESHOLD) {
			/* Disable hardware flow control in FLOW_CONTROL_AUTO mode:
			 * if no successful write since a defined time
			 * and if the last try was not the last successful write
			 */
			if (_last_write_try_time != 0 &&
			    hrt_elapsed_time(&_last_write_success_time) > 500_ms &&
			    _last_write_success_time != _last_write_try_time) {

				setup_flow_control(FLOW_CONTROL_OFF);
			}
		}
	}

	return buf_free;
}

void Mavlink::send_start(int length)
{
	pthread_mutex_lock(&_send_mutex);
	_last_write_try_time = hrt_absolute_time();

	// check if there is space in the buffer
	if (length > (int)get_free_tx_buf()) {
		// not enough space in buffer to send
		count_txerrbytes(length);

		_tstatus.tx_buffer_overruns++;

		// prevent writes
		_tx_buffer_low = true;

	} else {
		_tx_buffer_low = false;
	}
}

void Mavlink::send_finish()
{
	if (_tx_buffer_low || (_buf_fill == 0)) {
		pthread_mutex_unlock(&_send_mutex);
		return;
	}

	int ret = -1;

	// send message to UART
	if (get_protocol() == Protocol::SERIAL) {
		ret = ::write(_uart_fd, _buf, _buf_fill);
	}

#if defined(MAVLINK_UDP)

	else if (get_protocol() == Protocol::UDP) {

# if defined(CONFIG_NET)

		if (_src_addr_initialized) {
# endif // CONFIG_NET
			ret = sendto(_socket_fd, _buf, _buf_fill, 0, (struct sockaddr *)&_src_addr, sizeof(_src_addr));
# if defined(CONFIG_NET)
		}

# endif // CONFIG_NET

		if ((_mode != MAVLINK_MODE_ONBOARD) && broadcast_enabled() &&
		    (!get_client_source_initialized() || !is_gcs_connected())) {

			if (!_broadcast_address_found) {
				find_broadcast_address();
			}

			if (_broadcast_address_found && _buf_fill > 0) {

				int bret = sendto(_socket_fd, _buf, _buf_fill, 0, (struct sockaddr *)&_bcast_addr, sizeof(_bcast_addr));

				if (bret <= 0) {
					if (!_broadcast_failed_warned) {
						PX4_ERR("sending broadcast failed, errno: %d: %s", errno, strerror(errno));
						_broadcast_failed_warned = true;
					}

				} else {
					_broadcast_failed_warned = false;
				}
			}
		}
	}

#endif // MAVLINK_UDP

	if (ret == (int)_buf_fill) {
		_tstatus.tx_message_count++;
		count_txbytes(_buf_fill);
		_last_write_success_time = _last_write_try_time;

	} else {
		count_txerrbytes(_buf_fill);
	}

	_buf_fill = 0;

	pthread_mutex_unlock(&_send_mutex);
}

void Mavlink::send_bytes(const uint8_t *buf, unsigned packet_len)
{
	if (!_tx_buffer_low) {
		if (_buf_fill + packet_len < sizeof(_buf)) {
			memcpy(&_buf[_buf_fill], buf, packet_len);
			_buf_fill += packet_len;

		} else {
			perf_count(_send_byte_error_perf);
		}
	}
}

#ifdef MAVLINK_UDP
void Mavlink::find_broadcast_address()
{
	struct ifconf ifconf;
	int ret;

#if defined(__APPLE__) && defined(__MACH__) || defined(__CYGWIN__)
	// On Mac, we can't determine the required buffer
	// size in advance, so we just use what tends to work.
	ifconf.ifc_len = 1024;
#else
	// On Linux, we can determine the required size of the
	// buffer first by providing NULL to ifc_req.
	ifconf.ifc_req = nullptr;
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

	int offset = 0;
	// Later used to point to next network interface in buffer.
	struct ifreq *cur_ifreq = (struct ifreq *) & (((uint8_t *)ifconf.ifc_req)[offset]);

	// The ugly `for` construct is used because it allows to use
	// `continue` and `break`.
	for (;
	     offset < (int)ifconf.ifc_len;
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
		cur_ifreq = (struct ifreq *) & (((uint8_t *)ifconf.ifc_req)[offset]);

		PX4_DEBUG("looking at %s", cur_ifreq->ifr_name);

		// ignore loopback network
		if (strcmp(cur_ifreq->ifr_name, "lo") == 0 ||
		    strcmp(cur_ifreq->ifr_name, "lo0") == 0 ||
		    strcmp(cur_ifreq->ifr_name, "lo1") == 0 ||
		    strcmp(cur_ifreq->ifr_name, "lo2") == 0) {
			PX4_DEBUG("skipping loopback");
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
			const struct in_addr netmask_addr = query_netmask_addr(_socket_fd, *cur_ifreq);
			const struct in_addr broadcast_addr = compute_broadcast_addr(sin_addr, netmask_addr);

			if (_interface_name && strstr(cur_ifreq->ifr_name, _interface_name) == nullptr) { continue; }

			PX4_INFO("using network interface %s, IP: %s", cur_ifreq->ifr_name, inet_ntoa(sin_addr));
			PX4_INFO("with netmask: %s", inet_ntoa(netmask_addr));
			PX4_INFO("and broadcast IP: %s", inet_ntoa(broadcast_addr));

			_bcast_addr.sin_family = AF_INET;
			_bcast_addr.sin_addr = broadcast_addr;

			_broadcast_address_found = true;

		} else {
			PX4_DEBUG("ignoring additional network interface %s, IP:  %s",
				  cur_ifreq->ifr_name, inet_ntoa(sin_addr));
		}
	}

	if (_broadcast_address_found) {
		_bcast_addr.sin_port = htons(_remote_port);

		int broadcast_opt = 1;

		if (setsockopt(_socket_fd, SOL_SOCKET, SO_BROADCAST, &broadcast_opt, sizeof(broadcast_opt)) < 0) {
			PX4_WARN("setting broadcast permission failed");
		}

		_broadcast_address_not_found_warned = false;

	} else {
		if (!_broadcast_address_not_found_warned) {
			PX4_WARN("no broadcasting address found");
			_broadcast_address_not_found_warned = true;
		}
	}

	delete[] ifconf.ifc_req;
}

const in_addr Mavlink::query_netmask_addr(const int socket_fd, const ifreq &ifreq)
{
	struct ifreq netmask_ifreq {};
	strncpy(netmask_ifreq.ifr_name, ifreq.ifr_name, IF_NAMESIZE);
	ioctl(socket_fd, SIOCGIFNETMASK, &netmask_ifreq);

	return ((struct sockaddr_in *)&netmask_ifreq.ifr_addr)->sin_addr;
}

const in_addr Mavlink::compute_broadcast_addr(const in_addr &host_addr, const in_addr &netmask_addr)
{
	struct in_addr broadcast_addr;
	broadcast_addr.s_addr = ~netmask_addr.s_addr | host_addr.s_addr;

	return broadcast_addr;
}

void Mavlink::init_udp()
{
	PX4_DEBUG("Setting up UDP with port %hu", _network_port);

	_myaddr.sin_family = AF_INET;
	_myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	_myaddr.sin_port = htons(_network_port);

	if ((_socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		PX4_WARN("create socket failed: %s", strerror(errno));
		return;
	}

	if (bind(_socket_fd, (struct sockaddr *)&_myaddr, sizeof(_myaddr)) < 0) {
		PX4_WARN("bind failed: %s", strerror(errno));
		return;
	}

	/* set default target address, but not for onboard mode (will be set on first received packet) */
	if (!_src_addr_initialized) {
		_src_addr.sin_family = AF_INET;
		inet_aton("127.0.0.1", &_src_addr.sin_addr);
	}

	_src_addr.sin_port = htons(_remote_port);
}
#endif // MAVLINK_UDP

void
Mavlink::handle_message(const mavlink_message_t *msg)
{
	/*
	 *  NOTE: this is called from the receiver thread
	 */

	if (get_forwarding_on()) {
		/* forward any messages to other mavlink instances */
		Mavlink::forward_message(msg, this);
	}

	// Special case for gimbals that need to forward GIMBAL_DEVICE_ATTITUDE_STATUS.
	else if (msg->msgid == MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS) {
		Mavlink::forward_message(msg, this);

	} else if (msg->msgid == MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION) {
		Mavlink::forward_message(msg, this);
	}
}

void
Mavlink::send_statustext_info(const char *string)
{
	mavlink_log_info(&_mavlink_log_pub, "%s", string);
}

void
Mavlink::send_statustext_critical(const char *string)
{
	mavlink_log_critical(&_mavlink_log_pub, "%s", string);
}

void
Mavlink::send_statustext_emergency(const char *string)
{
	mavlink_log_emergency(&_mavlink_log_pub, "%s", string);
}

bool
Mavlink::send_autopilot_capabilities()
{
	vehicle_status_s status;

	if (_vehicle_status_sub.copy(&status)) {
		mavlink_autopilot_version_t msg{};

		msg.capabilities = MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_MISSION_INT;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_COMMAND_INT;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_FTP;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_MAVLINK2;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_MISSION_FENCE;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_MISSION_RALLY;


		{
			param_t param_handle = param_find_no_notification("MNT_MODE_IN");
			int32_t mnt_mode_in = 0;

			if (mnt_mode_in != PARAM_INVALID) {
				param_get(param_handle, &mnt_mode_in);

				if (mnt_mode_in == 4) {
					msg.capabilities |= MAV_PROTOCOL_CAPABILITY_COMPONENT_IMPLEMENTS_GIMBAL_MANAGER;
				}
			}
		}

		msg.flight_sw_version = px4_firmware_version();
		msg.middleware_sw_version = px4_firmware_version();
		msg.os_sw_version = px4_os_version();
		msg.board_version = px4_board_version();
		/* use only first 5 bytes of git hash for firmware version */
		const uint64_t fw_git_version_binary = px4_firmware_version_binary() & 0xFFFFFFFFFF000000;
		const uint64_t fw_vendor_version = px4_firmware_vendor_version() >> 8;
		constexpr size_t fw_vendor_version_length = 3;
		memcpy(&msg.flight_custom_version, &fw_git_version_binary, sizeof(msg.flight_custom_version));
		memcpy(&msg.flight_custom_version, &fw_vendor_version, fw_vendor_version_length);
		memcpy(&msg.middleware_custom_version, &fw_git_version_binary, sizeof(msg.middleware_custom_version));
		uint64_t os_git_version_binary = px4_os_version_binary();
		memcpy(&msg.os_custom_version, &os_git_version_binary, sizeof(msg.os_custom_version));
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
		uuid_uint32_t uid;
		board_get_uuid32(uid);
		msg.uid = (((uint64_t)uid[PX4_CPU_UUID_WORD32_UNIQUE_M]) << 32) | uid[PX4_CPU_UUID_WORD32_UNIQUE_H];

#ifndef BOARD_HAS_NO_UUID
		px4_guid_t px4_guid;
		board_get_px4_guid(px4_guid);
		static_assert(sizeof(px4_guid_t) == sizeof(msg.uid2), "GUID byte length mismatch");
		memcpy(&msg.uid2, &px4_guid, sizeof(msg.uid2));
#endif /* BOARD_HAS_NO_UUID */

#ifdef CONFIG_ARCH_BOARD_PX4_SITL
		// To avoid that multiple SITL instances have the same UUID, we add the mavlink
		// system ID. We subtract 1, so that the first UUID remains unchanged given the
		// default system ID is 1.
		//
		// Note that the UUID show in `ver` will still be the same for all instances.
		msg.uid += mavlink_system.sysid - 1;
		msg.uid2[0] += mavlink_system.sysid - 1;
#endif /* CONFIG_ARCH_BOARD_PX4_SITL */
		mavlink_msg_autopilot_version_send_struct(get_channel(), &msg);
		return true;
	}

	return false;
}

void
Mavlink::send_protocol_version()
{
	mavlink_protocol_version_t msg = {};

	msg.version = _protocol_version * 100;
	msg.min_version = 100;
	msg.max_version = 200;
	uint64_t mavlink_lib_git_version_binary = px4_mavlink_lib_version_binary();
	// TODO add when available
	//memcpy(&msg.spec_version_hash, &mavlink_spec_git_version_binary, sizeof(msg.spec_version_hash));
	memcpy(&msg.library_version_hash, &mavlink_lib_git_version_binary, sizeof(msg.library_version_hash));

	// Switch to MAVLink 2
	int curr_proto_ver = _protocol_version;
	set_proto_version(2);
	// Send response - if it passes through the link its fine to use MAVLink 2
	mavlink_msg_protocol_version_send_struct(get_channel(), &msg);
	// Reset to previous value
	set_proto_version(curr_proto_ver);
}

int
Mavlink::configure_stream(const char *stream_name, const float rate)
{
	PX4_DEBUG("configure_stream(%s, %.3f)", stream_name, (double)rate);

	/* calculate interval in us, -1 means unlimited stream, 0 means disabled */
	int interval = 0;

	if (rate > 0.000001f) {
		interval = (1000000.0f / rate);

	} else if (rate < 0.0f) {
		interval = -1;
	}

	for (const auto &stream : _streams) {
		if (strcmp(stream_name, stream->get_name()) == 0) {
			if (interval != 0) {
				/* set new interval */
				stream->set_interval(interval);

			} else {
				/* delete stream */
				_streams.deleteNode(stream);
				return OK; // must finish with loop after node is deleted
			}

			return OK;
		}
	}

	// search for stream with specified name in supported streams list
	// create new instance if found
	MavlinkStream *stream = create_mavlink_stream(stream_name, this);

	if (stream != nullptr) {
		stream->set_interval(interval);
		_streams.add(stream);

		return OK;
	}

	// if we reach here, the stream list does not contain the stream.
	// flash constrained target's don't include all streams, and some are only available for the development dialect
#if defined(CONSTRAINED_FLASH) || !defined(MAVLINK_DEVELOPMENT_H)
	return PX4_OK;
#else
	PX4_WARN("stream %s not found", stream_name);
	return PX4_ERROR;
#endif
}

void
Mavlink::configure_stream_threadsafe(const char *stream_name, const float rate)
{
	/* orb subscription must be done from the main thread,
	 * set _subscribe_to_stream and _subscribe_to_stream_rate fields
	 * which polled in mavlink main loop */
	if (!should_exit()) {
		/* wait for previous subscription completion */
		while (_subscribe_to_stream != nullptr) {
			px4_usleep(MAIN_LOOP_DELAY / 2);
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
			px4_usleep(MAIN_LOOP_DELAY / 2);
		} while (_subscribe_to_stream != nullptr);

		delete[] s;
	}
}

void
Mavlink::pass_message(const mavlink_message_t *msg)
{
	/* size is 12 bytes plus variable payload */
	int size = MAVLINK_NUM_NON_PAYLOAD_BYTES + msg->len;
	LockGuard lg{_message_buffer_mutex};

	if (!_message_buffer.push_back(reinterpret_cast<const uint8_t *>(msg), size)) {
		perf_count(_forwarding_error_perf);
	}
}

MavlinkShell *
Mavlink::get_shell()
{
	if (!_mavlink_shell) {
		MavlinkShell *shell = new MavlinkShell();

		if (!shell) {
			PX4_ERR("Failed to allocate a shell");

		} else {
			int ret = shell->start();

			if (ret == 0) {
				_mavlink_shell = shell;

			} else {
				PX4_ERR("Failed to start shell (%i)", ret);
				delete shell;
			}
		}
	}

	return _mavlink_shell;
}

void
Mavlink::close_shell()
{
	if (_mavlink_shell) {
		delete _mavlink_shell;
		_mavlink_shell = nullptr;
	}
}

void
Mavlink::update_rate_mult()
{
	float const_rate = 0.0f;
	float rate = 0.0f;

	/* scale down rates if their theoretical bandwidth is exceeding the link bandwidth */
	for (const auto &stream : _streams) {
		if (stream->const_rate()) {
			const_rate += (stream->get_interval() > 0) ? stream->get_size_avg() * 1000000.0f / stream->get_interval() : 0;

		} else {
			rate += (stream->get_interval() > 0) ? stream->get_size_avg() * 1000000.0f / stream->get_interval() : 0;
		}
	}

	float mavlink_ulog_streaming_rate_inv = 1.0f;

	if (_mavlink_ulog) {
		mavlink_ulog_streaming_rate_inv = 1.0f - _mavlink_ulog->current_data_rate();
	}

	/* scale up and down as the link permits */
	float bandwidth_mult = (float)(_datarate * mavlink_ulog_streaming_rate_inv - const_rate) / rate;

	/* if we do not have flow control, limit to the set data rate */
	if (!get_flow_control_enabled()) {
		bandwidth_mult = fminf(1.0f, bandwidth_mult);
	}

	float hardware_mult = 1.0f;
	bool log_radio_timeout = false;

	pthread_mutex_lock(&_radio_status_mutex);

	// scale down if we have a TX err rate suggesting link congestion
	if ((_tstatus.tx_error_rate_avg > 0.f) && !_radio_status_critical) {
		hardware_mult = _tstatus.tx_rate_avg / (_tstatus.tx_rate_avg + _tstatus.tx_error_rate_avg);

	} else if (_radio_status_available) {

		// check for RADIO_STATUS timeout and reset
		if (hrt_elapsed_time(&_rstatus.timestamp) > (_param_mav_radio_timeout.get() * 1_s)) {
			_radio_status_available = false;
			log_radio_timeout = true;

			if (_use_software_mav_throttling) {
				_radio_status_critical = false;
				_radio_status_mult = 1.0f;
			}
		}

		hardware_mult *= _radio_status_mult;
	}

	pthread_mutex_unlock(&_radio_status_mutex);

	if (log_radio_timeout) {
		PX4_ERR("instance %d: RADIO_STATUS timeout", _instance_id);
	}

	/* pick the minimum from bandwidth mult and hardware mult as limit */
	_rate_mult = fminf(bandwidth_mult, hardware_mult);

	/* ensure the rate multiplier never drops below 5% so that something is always sent */
	_rate_mult = math::constrain(_rate_mult, 0.05f, 1.0f);
}

void
Mavlink::update_radio_status(const radio_status_s &radio_status)
{
	pthread_mutex_lock(&_radio_status_mutex);
	_rstatus = radio_status;
	_radio_status_available = true;

	if (_use_software_mav_throttling) {

		/* check hardware limits */
		_radio_status_critical = (radio_status.txbuf < RADIO_BUFFER_LOW_PERCENTAGE);

		if (radio_status.txbuf < RADIO_BUFFER_CRITICAL_LOW_PERCENTAGE) {
			/* this indicates link congestion, reduce rate by 20% */
			_radio_status_mult *= 0.80f;

		} else if (radio_status.txbuf < RADIO_BUFFER_LOW_PERCENTAGE) {
			/* this indicates link congestion, reduce rate by 2.5% */
			_radio_status_mult *= 0.975f;

		} else if (radio_status.txbuf > RADIO_BUFFER_HALF_PERCENTAGE) {
			/* this indicates spare bandwidth, increase by 2.5% */
			_radio_status_mult *= 1.025f;
		}

		/* Constrain radio status multiplier between 1% and 100% to allow recovery */
		_radio_status_mult = math::constrain(_radio_status_mult, 0.01f, 1.0f);
	}

	pthread_mutex_unlock(&_radio_status_mutex);
}

int
Mavlink::configure_streams_to_default(const char *configure_single_stream)
{
	int ret = 0;
	bool stream_configured = false;

	auto configure_stream_local =
	[&stream_configured, configure_single_stream, &ret, this](const char *stream_name, float rate) {
		if (!configure_single_stream || strcmp(configure_single_stream, stream_name) == 0) {
			int ret_local = configure_stream(stream_name, rate);

			if (ret_local != 0) {
				ret = ret_local;
			}

			stream_configured = true;
		}
	};

	const float unlimited_rate = -1.0f;

	switch (_mode) {
	case MAVLINK_MODE_NORMAL:
		configure_stream_local("ADSB_VEHICLE", unlimited_rate);
		configure_stream_local("ALTITUDE", 1.0f);
		configure_stream_local("ATTITUDE", 15.0f);
		configure_stream_local("ATTITUDE_QUATERNION", 10.0f);
		configure_stream_local("ATTITUDE_TARGET", 2.0f);
		configure_stream_local("AVAILABLE_MODES", 0.3f);
		configure_stream_local("BATTERY_STATUS", 0.5f);
		configure_stream_local("CAMERA_IMAGE_CAPTURED", unlimited_rate);
		configure_stream_local("CURRENT_MODE", 0.5f);
		configure_stream_local("DISTANCE_SENSOR", 0.5f);
		configure_stream_local("EFI_STATUS", 2.0f);
		configure_stream_local("ESC_INFO", 1.0f);
		configure_stream_local("ESC_STATUS", 1.0f);
		configure_stream_local("ESTIMATOR_STATUS", 0.5f);
		configure_stream_local("EXTENDED_SYS_STATE", 1.0f);
		configure_stream_local("GIMBAL_DEVICE_ATTITUDE_STATUS", 1.0f);
		configure_stream_local("GIMBAL_DEVICE_SET_ATTITUDE", 5.0f);
		configure_stream_local("GIMBAL_MANAGER_STATUS", 0.5f);
		configure_stream_local("GLOBAL_POSITION_INT", 5.0f);
		configure_stream_local("GPS2_RAW", 1.0f);
		configure_stream_local("GPS_GLOBAL_ORIGIN", 1.0f);
		configure_stream_local("GPS_RAW_INT", 5.0f);
		configure_stream_local("GPS_STATUS", 1.0f);
		configure_stream_local("HOME_POSITION", 0.5f);
		configure_stream_local("HYGROMETER_SENSOR", 0.1f);
		configure_stream_local("LOCAL_POSITION_NED", 1.0f);
		configure_stream_local("MOUNT_ORIENTATION", 10.0f);
		configure_stream_local("NAV_CONTROLLER_OUTPUT", 1.0f);
		configure_stream_local("OBSTACLE_DISTANCE", 1.0f);
		configure_stream_local("OPEN_DRONE_ID_LOCATION", 1.f);
		configure_stream_local("OPEN_DRONE_ID_SYSTEM", 1.f);
		configure_stream_local("OPEN_DRONE_ID_ARM_STATUS", 1.f);
		configure_stream_local("ORBIT_EXECUTION_STATUS", 2.0f);
		configure_stream_local("PING", 0.1f);
		configure_stream_local("POSITION_TARGET_GLOBAL_INT", 1.0f);
		configure_stream_local("POSITION_TARGET_LOCAL_NED", 1.5f);
		configure_stream_local("RAW_RPM", 2.0f);
		configure_stream_local("RC_CHANNELS", 5.0f);
		configure_stream_local("SCALED_PRESSURE", 1.0f);
		configure_stream_local("SERVO_OUTPUT_RAW_0", 1.0f);
		configure_stream_local("SYS_STATUS", 1.0f);
		configure_stream_local("TIME_ESTIMATE_TO_TARGET", 1.0f);
		configure_stream_local("VFR_HUD", 4.0f);
		configure_stream_local("VIBRATION", 0.1f);
		configure_stream_local("WIND_COV", 0.5f);

#if !defined(CONSTRAINED_FLASH)
		configure_stream_local("DEBUG", 1.0f);
		configure_stream_local("DEBUG_FLOAT_ARRAY", 1.0f);
		configure_stream_local("DEBUG_VECT", 1.0f);
		configure_stream_local("NAMED_VALUE_FLOAT", 1.0f);
		configure_stream_local("LINK_NODE_STATUS", 1.0f);
#if defined(MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS)
		configure_stream_local("FIGURE_EIGHT_EXECUTION_STATUS", 5.0f);
#endif // MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS
#if defined(MAVLINK_MSG_ID_FUEL_STATUS)
		configure_stream_local("FUEL_STATUS", 1.0f);
#endif // MAVLINK_MSG_ID_FUEL_STATUS
#endif // !CONSTRAINED_FLASH

		break;

	case MAVLINK_MODE_ONBOARD:
		// Note: streams requiring low latency come first
		configure_stream_local("TIMESYNC", 10.0f);
		configure_stream_local("CAMERA_TRIGGER", unlimited_rate);
		configure_stream_local("HIGHRES_IMU", 50.0f);
		configure_stream_local("LOCAL_POSITION_NED", 30.0f);
		configure_stream_local("ATTITUDE", 100.0f);
		configure_stream_local("ALTITUDE", 10.0f);
		configure_stream_local("DISTANCE_SENSOR", 10.0f);
		configure_stream_local("ESC_INFO", 10.0f);
		configure_stream_local("ESC_STATUS", 10.0f);
		configure_stream_local("MOUNT_ORIENTATION", 10.0f);
		configure_stream_local("OBSTACLE_DISTANCE", 10.0f);
		configure_stream_local("ODOMETRY", 30.0f);

		configure_stream_local("ADSB_VEHICLE", unlimited_rate);
		configure_stream_local("ATTITUDE_QUATERNION", 50.0f);
		configure_stream_local("ATTITUDE_TARGET", 10.0f);
		configure_stream_local("AVAILABLE_MODES", 0.3f);
		configure_stream_local("BATTERY_STATUS", 0.5f);
		configure_stream_local("CAMERA_IMAGE_CAPTURED", unlimited_rate);
		configure_stream_local("CURRENT_MODE", 0.5f);
		configure_stream_local("EFI_STATUS", 2.0f);
		configure_stream_local("ESTIMATOR_STATUS", 1.0f);
		configure_stream_local("EXTENDED_SYS_STATE", 5.0f);
		configure_stream_local("GIMBAL_DEVICE_ATTITUDE_STATUS", 1.0f);
		configure_stream_local("GIMBAL_DEVICE_SET_ATTITUDE", 5.0f);
		configure_stream_local("GIMBAL_MANAGER_STATUS", 0.5f);
		configure_stream_local("GLOBAL_POSITION_INT", 50.0f);
		configure_stream_local("GPS2_RAW", unlimited_rate);
		configure_stream_local("GPS_GLOBAL_ORIGIN", 1.0f);
		configure_stream_local("GPS_RAW_INT", unlimited_rate);
		configure_stream_local("GPS_STATUS", 1.0f);
		configure_stream_local("HOME_POSITION", 0.5f);
		configure_stream_local("HYGROMETER_SENSOR", 1.0f);
		configure_stream_local("NAV_CONTROLLER_OUTPUT", 10.0f);
		configure_stream_local("OPEN_DRONE_ID_LOCATION", 1.f);
		configure_stream_local("OPEN_DRONE_ID_SYSTEM", 1.f);
		configure_stream_local("OPEN_DRONE_ID_ARM_STATUS", 1.f);
		configure_stream_local("OPTICAL_FLOW_RAD", 10.0f);
		configure_stream_local("ORBIT_EXECUTION_STATUS", 5.0f);
		configure_stream_local("PING", 1.0f);
		configure_stream_local("POSITION_TARGET_GLOBAL_INT", 10.0f);
		configure_stream_local("POSITION_TARGET_LOCAL_NED", 10.0f);
		configure_stream_local("RAW_RPM", 5.0f);
		configure_stream_local("RC_CHANNELS", 20.0f);
		configure_stream_local("SCALED_PRESSURE", 1.0f);
		configure_stream_local("SERVO_OUTPUT_RAW_0", 10.0f);
		configure_stream_local("SYS_STATUS", 5.0f);
		configure_stream_local("SYSTEM_TIME", 1.0f);
		configure_stream_local("TIME_ESTIMATE_TO_TARGET", 1.0f);
		configure_stream_local("VFR_HUD", 10.0f);
		configure_stream_local("VIBRATION", 0.5f);
		configure_stream_local("WIND_COV", 10.0f);

#if !defined(CONSTRAINED_FLASH)
		configure_stream_local("DEBUG", 10.0f);
		configure_stream_local("DEBUG_FLOAT_ARRAY", 10.0f);
		configure_stream_local("DEBUG_VECT", 10.0f);
		configure_stream_local("NAMED_VALUE_FLOAT", 10.0f);
		configure_stream_local("LINK_NODE_STATUS", 1.0f);
#if defined(MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS)
		configure_stream_local("FIGURE_EIGHT_EXECUTION_STATUS", 5.0f);
#endif // MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS
#if defined(MAVLINK_MSG_ID_FUEL_STATUS)
		configure_stream_local("FUEL_STATUS", 1.0f);
#endif // MAVLINK_MSG_ID_FUEL_STATUS
#endif // !CONSTRAINED_FLASH

		break;

	case MAVLINK_MODE_GIMBAL:
		// Note: streams requiring low latency come first
		configure_stream_local("AUTOPILOT_STATE_FOR_GIMBAL_DEVICE", 20.0f);
		configure_stream_local("GIMBAL_DEVICE_SET_ATTITUDE", 20.0f);
		break;

	case MAVLINK_MODE_EXTVISION:
		configure_stream_local("HIGHRES_IMU", unlimited_rate);		// for VIO

	// FALLTHROUGH
	case MAVLINK_MODE_EXTVISIONMIN:
		// Note: streams requiring low latency come first
		configure_stream_local("TIMESYNC", 10.0f);
		configure_stream_local("CAMERA_TRIGGER", unlimited_rate);
		configure_stream_local("LOCAL_POSITION_NED", 30.0f);
		configure_stream_local("ATTITUDE", 20.0f);
		configure_stream_local("ALTITUDE", 10.0f);
		configure_stream_local("DISTANCE_SENSOR", 10.0f);
		configure_stream_local("MOUNT_ORIENTATION", 10.0f);
		configure_stream_local("OBSTACLE_DISTANCE", 10.0f);
		configure_stream_local("ODOMETRY", 30.0f);

		configure_stream_local("ADSB_VEHICLE", unlimited_rate);
		configure_stream_local("ATTITUDE_TARGET", 2.0f);
		configure_stream_local("AVAILABLE_MODES", 0.3f);
		configure_stream_local("BATTERY_STATUS", 0.5f);
		configure_stream_local("CAMERA_IMAGE_CAPTURED", unlimited_rate);
		configure_stream_local("CURRENT_MODE", 0.5f);
		configure_stream_local("ESTIMATOR_STATUS", 1.0f);
		configure_stream_local("EXTENDED_SYS_STATE", 1.0f);
		configure_stream_local("GLOBAL_POSITION_INT", 5.0f);
		configure_stream_local("GPS2_RAW", 1.0f);
		configure_stream_local("GPS_GLOBAL_ORIGIN", 1.0f);
		configure_stream_local("GPS_RAW_INT", 1.0f);
		configure_stream_local("HOME_POSITION", 0.5f);
		configure_stream_local("HYGROMETER_SENSOR", 1.0f);
		configure_stream_local("SCALED_PRESSURE", 1.0f);
		configure_stream_local("NAV_CONTROLLER_OUTPUT", 1.5f);
		configure_stream_local("OPTICAL_FLOW_RAD", 1.0f);
		configure_stream_local("ORBIT_EXECUTION_STATUS", 5.0f);
		configure_stream_local("PING", 0.1f);
		configure_stream_local("POSITION_TARGET_GLOBAL_INT", 1.5f);
		configure_stream_local("POSITION_TARGET_LOCAL_NED", 1.5f);
		configure_stream_local("RC_CHANNELS", 5.0f);
		configure_stream_local("SERVO_OUTPUT_RAW_0", 1.0f);
		configure_stream_local("SYS_STATUS", 5.0f);
		configure_stream_local("VFR_HUD", 4.0f);
		configure_stream_local("VIBRATION", 0.5f);
		configure_stream_local("WIND_COV", 1.0f);

#if !defined(CONSTRAINED_FLASH)
		configure_stream_local("DEBUG", 1.0f);
		configure_stream_local("DEBUG_FLOAT_ARRAY", 1.0f);
		configure_stream_local("DEBUG_VECT", 1.0f);
		configure_stream_local("NAMED_VALUE_FLOAT", 1.0f);
		configure_stream_local("LINK_NODE_STATUS", 1.0f);
#if defined(MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS)
		configure_stream_local("FIGURE_EIGHT_EXECUTION_STATUS", 2.0f);
#endif // MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS
#if defined(MAVLINK_MSG_ID_FUEL_STATUS)
		configure_stream_local("FUEL_STATUS", 1.0f);
#endif // MAVLINK_MSG_ID_FUEL_STATUS
#endif // !CONSTRAINED_FLASH

		break;

	case MAVLINK_MODE_OSD:
		configure_stream_local("ALTITUDE", 10.0f);
		configure_stream_local("ATTITUDE", 25.0f);
		configure_stream_local("ATTITUDE_TARGET", 10.0f);
		configure_stream_local("BATTERY_STATUS", 0.5f);
		configure_stream_local("ESTIMATOR_STATUS", 1.0f);
		configure_stream_local("EXTENDED_SYS_STATE", 1.0f);
		configure_stream_local("GLOBAL_POSITION_INT", 10.0f);
		configure_stream_local("GPS_RAW_INT", 1.0f);
		configure_stream_local("HOME_POSITION", 0.5f);
		configure_stream_local("HYGROMETER_SENSOR", 0.1f);
		configure_stream_local("SCALED_PRESSURE", 1.0f);
		configure_stream_local("RC_CHANNELS", 5.0f);
		configure_stream_local("SERVO_OUTPUT_RAW_0", 1.0f);
		configure_stream_local("SYS_STATUS", 5.0f);
		configure_stream_local("SYSTEM_TIME", 1.0f);
		configure_stream_local("VFR_HUD", 25.0f);
		configure_stream_local("VIBRATION", 0.5f);
		configure_stream_local("WIND_COV", 2.0f);
		break;

	case MAVLINK_MODE_MAGIC:

	/* fallthrough */
	case MAVLINK_MODE_CUSTOM:
		//stream nothing
		break;

	case MAVLINK_MODE_CONFIG: // USB
		// Note: streams requiring low latency come first
		configure_stream_local("TIMESYNC", 10.0f);
		configure_stream_local("CAMERA_TRIGGER", unlimited_rate);
		configure_stream_local("LOCAL_POSITION_NED", 30.0f);
		configure_stream_local("DISTANCE_SENSOR", 10.0f);
		configure_stream_local("MOUNT_ORIENTATION", 10.0f);
		configure_stream_local("ODOMETRY", 30.0f);

		configure_stream_local("ADSB_VEHICLE", unlimited_rate);
		configure_stream_local("ALTITUDE", 10.0f);
		configure_stream_local("ATTITUDE", 50.0f);
		configure_stream_local("ATTITUDE_QUATERNION", 50.0f);
		configure_stream_local("ATTITUDE_TARGET", 8.0f);
		configure_stream_local("AVAILABLE_MODES", 0.3f);
		configure_stream_local("BATTERY_STATUS", 0.5f);
		configure_stream_local("CAMERA_IMAGE_CAPTURED", unlimited_rate);
		configure_stream_local("CURRENT_MODE", 0.5f);
		configure_stream_local("EFI_STATUS", 10.0f);
		configure_stream_local("ESC_INFO", 10.0f);
		configure_stream_local("ESC_STATUS", 10.0f);
		configure_stream_local("ESTIMATOR_STATUS", 5.0f);
		configure_stream_local("EXTENDED_SYS_STATE", 2.0f);
		configure_stream_local("GLOBAL_POSITION_INT", 10.0f);
		configure_stream_local("GPS2_RAW", unlimited_rate);
		configure_stream_local("GPS_GLOBAL_ORIGIN", 1.0f);
		configure_stream_local("GPS_RAW_INT", unlimited_rate);
		configure_stream_local("GPS_STATUS", 1.0f);
		configure_stream_local("GIMBAL_DEVICE_ATTITUDE_STATUS", 0.5f);
		configure_stream_local("GIMBAL_MANAGER_STATUS", 0.5f);
		configure_stream_local("HIGHRES_IMU", 50.0f);
		configure_stream_local("HOME_POSITION", 0.5f);
		configure_stream_local("HYGROMETER_SENSOR", 1.0f);
		configure_stream_local("MAG_CAL_REPORT", 1.0f);
		configure_stream_local("MANUAL_CONTROL", 5.0f);
		configure_stream_local("NAV_CONTROLLER_OUTPUT", 10.0f);
		configure_stream_local("OPEN_DRONE_ID_LOCATION", 1.f);
		configure_stream_local("OPEN_DRONE_ID_SYSTEM", 1.f);
		configure_stream_local("OPEN_DRONE_ID_ARM_STATUS", 1.f);
		configure_stream_local("OPTICAL_FLOW_RAD", 10.0f);
		configure_stream_local("ORBIT_EXECUTION_STATUS", 5.0f);
		configure_stream_local("PING", 1.0f);
		configure_stream_local("POSITION_TARGET_GLOBAL_INT", 10.0f);
		configure_stream_local("RAW_RPM", 5.0f);
		configure_stream_local("RC_CHANNELS", 10.0f);
		configure_stream_local("SCALED_IMU", 25.0f);
		configure_stream_local("SCALED_IMU2", 25.0f);
		configure_stream_local("SCALED_IMU3", 25.0f);
		configure_stream_local("SCALED_PRESSURE", 1.0f);
		configure_stream_local("SCALED_PRESSURE2", 1.0f);
		configure_stream_local("SERVO_OUTPUT_RAW_0", 20.0f);
		configure_stream_local("SERVO_OUTPUT_RAW_1", 20.0f);
		configure_stream_local("SYS_STATUS", 1.0f);
		configure_stream_local("SYSTEM_TIME", 1.0f);
		configure_stream_local("TIME_ESTIMATE_TO_TARGET", 1.0f);
		configure_stream_local("VFR_HUD", 20.0f);
		configure_stream_local("VIBRATION", 2.5f);
		configure_stream_local("WIND_COV", 10.0f);

#if !defined(CONSTRAINED_FLASH)
		configure_stream_local("DEBUG", 50.0f);
		configure_stream_local("DEBUG_FLOAT_ARRAY", 50.0f);
		configure_stream_local("DEBUG_VECT", 50.0f);
		configure_stream_local("NAMED_VALUE_FLOAT", 50.0f);
		configure_stream_local("LINK_NODE_STATUS", 1.0f);
#if defined(MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS)
		configure_stream_local("FIGURE_EIGHT_EXECUTION_STATUS", 5.0f);
#endif // MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS
#if defined(MAVLINK_MSG_ID_FUEL_STATUS)
		configure_stream_local("FUEL_STATUS", 2.0f);
#endif // MAVLINK_MSG_ID_FUEL_STATUS
#endif // !CONSTRAINED_FLASH

		break;

	case MAVLINK_MODE_IRIDIUM:
		configure_stream_local("HIGH_LATENCY2", _high_latency_freq);
		break;

	case MAVLINK_MODE_MINIMAL:
		configure_stream_local("ALTITUDE", 0.5f);
		configure_stream_local("ATTITUDE", 10.0f);
		configure_stream_local("EXTENDED_SYS_STATE", 0.1f);
		configure_stream_local("GLOBAL_POSITION_INT", 5.0f);
		configure_stream_local("GPS_RAW_INT", 0.5f);
		configure_stream_local("HOME_POSITION", 0.1f);
		configure_stream_local("NAMED_VALUE_FLOAT", 1.0f);
		configure_stream_local("RC_CHANNELS", 0.5f);
		configure_stream_local("SYS_STATUS", 0.1f);
		configure_stream_local("VFR_HUD", 1.0f);

#if !defined(CONSTRAINED_FLASH)
		configure_stream_local("LINK_NODE_STATUS", 1.0f);
#endif // !CONSTRAINED_FLASH

		break;

	case MAVLINK_MODE_ONBOARD_LOW_BANDWIDTH:
		// Note: streams requiring low latency come first
		configure_stream_local("TIMESYNC", 10.0f);
		configure_stream_local("CAMERA_TRIGGER", unlimited_rate);
		configure_stream_local("LOCAL_POSITION_NED", 30.0f);
		configure_stream_local("ATTITUDE", 20.0f);
		configure_stream_local("ATTITUDE_QUATERNION", 20.0f);
		configure_stream_local("ALTITUDE", 10.0f);
		configure_stream_local("DISTANCE_SENSOR", 10.0f);
		configure_stream_local("MOUNT_ORIENTATION", 10.0f);
		configure_stream_local("OBSTACLE_DISTANCE", 10.0f);
		configure_stream_local("ODOMETRY", 30.0f);
		configure_stream_local("GIMBAL_DEVICE_ATTITUDE_STATUS", 1.0f);
		configure_stream_local("GIMBAL_MANAGER_STATUS", 0.5f);
		configure_stream_local("GIMBAL_DEVICE_SET_ATTITUDE", 5.0f);
		configure_stream_local("ESC_INFO", 1.0f);
		configure_stream_local("ESC_STATUS", 5.0f);

		configure_stream_local("ADSB_VEHICLE", unlimited_rate);
		configure_stream_local("ATTITUDE_TARGET", 2.0f);
		configure_stream_local("AVAILABLE_MODES", 0.3f);
		configure_stream_local("BATTERY_STATUS", 0.5f);
		configure_stream_local("CAMERA_IMAGE_CAPTURED", unlimited_rate);
		configure_stream_local("CURRENT_MODE", 0.5f);
		configure_stream_local("ESTIMATOR_STATUS", 1.0f);
		configure_stream_local("EXTENDED_SYS_STATE", 1.0f);
		configure_stream_local("GLOBAL_POSITION_INT", 10.0f);
		configure_stream_local("GPS_GLOBAL_ORIGIN", 1.0f);
		configure_stream_local("GPS2_RAW", unlimited_rate);
		configure_stream_local("GPS_RAW_INT", unlimited_rate);
		configure_stream_local("HOME_POSITION", 0.5f);
		configure_stream_local("NAV_CONTROLLER_OUTPUT", 1.5f);
		configure_stream_local("OPEN_DRONE_ID_LOCATION", 1.f);
		configure_stream_local("OPEN_DRONE_ID_SYSTEM", 1.f);
		configure_stream_local("OPEN_DRONE_ID_ARM_STATUS", 1.f);
		configure_stream_local("OPTICAL_FLOW_RAD", 1.0f);
		configure_stream_local("ORBIT_EXECUTION_STATUS", 5.0f);
		configure_stream_local("PING", 0.1f);
		configure_stream_local("POSITION_TARGET_GLOBAL_INT", 1.5f);
		configure_stream_local("POSITION_TARGET_LOCAL_NED", 1.5f);
		configure_stream_local("RAW_RPM", 5.0f);
		configure_stream_local("RC_CHANNELS", 20.0f);
		configure_stream_local("SERVO_OUTPUT_RAW_0", 1.0f);
		configure_stream_local("SYS_STATUS", 5.0f);
		configure_stream_local("SYSTEM_TIME", 2.0f);
		configure_stream_local("TIME_ESTIMATE_TO_TARGET", 1.0f);
		configure_stream_local("VFR_HUD", 4.0f);
		configure_stream_local("VIBRATION", 0.5f);
		configure_stream_local("WIND_COV", 1.0f);

#if !defined(CONSTRAINED_FLASH)
		configure_stream_local("DEBUG", 1.0f);
		configure_stream_local("DEBUG_FLOAT_ARRAY", 1.0f);
		configure_stream_local("DEBUG_VECT", 1.0f);
		configure_stream_local("NAMED_VALUE_FLOAT", 1.0f);
#if defined(MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS)
		configure_stream_local("FIGURE_EIGHT_EXECUTION_STATUS", 5.0f);
#endif // MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS
#if defined(MAVLINK_MSG_ID_FUEL_STATUS)
		configure_stream_local("FUEL_STATUS", 1.0f);
#endif // MAVLINK_MSG_ID_FUEL_STATUS
#endif // !CONSTRAINED_FLASH
		break;

	case MAVLINK_MODE_UAVIONIX:
		configure_stream_local("UAVIONIX_ADSB_OUT_CFG", 0.1f);
		configure_stream_local("UAVIONIX_ADSB_OUT_DYNAMIC", 5.0f);
		break;

	default:
		ret = -1;
		break;
	}

	if (configure_single_stream && !stream_configured && strcmp(configure_single_stream, "HEARTBEAT") != 0) {
		// stream was not found, assume it is disabled by default
		return configure_stream(configure_single_stream, 0.0f);
	}

	return ret;
}

int
Mavlink::task_main(int argc, char *argv[])
{
	// If stdin, stdout and/or stderr file descriptors (0, 1, 2)
	// are not open when mavlink module starts (as might be the case for USB auto-start),
	// use default /dev/null so that these numbers are not used by other other files.
	if (fcntl(0, F_GETFD) == -1) {
		int tmp = open("/dev/null", O_RDONLY);

		if (tmp != 0) {
			dup2(tmp, 0);
			close(tmp);
		}

	}

	if (fcntl(1, F_GETFD) == -1) {
		int tmp = open("/dev/null", O_WRONLY);

		if (tmp != 1) {
			dup2(tmp, 1);
			close(tmp);
		}
	}

	if (fcntl(2, F_GETFD) == -1) {
		int tmp = open("/dev/null", O_WRONLY);

		if (tmp != 2) {
			dup2(tmp, 2);
			close(tmp);
		}
	}

	int ch;
	_baudrate = 57600;
	_datarate = 0;
	_mode = MAVLINK_MODE_COUNT;
	FLOW_CONTROL_MODE _flow_control = FLOW_CONTROL_AUTO;

	_interface_name = nullptr;

	// We don't care about the name and verb at this point.
	argc -= 2;
	argv += 2;

	/* don't exit from getopt loop to leave getopt global variables in consistent state,
	 * set error flag instead */
	bool err_flag = false;
	int myoptind = 1;
	const char *myoptarg = nullptr;
#if defined(CONFIG_NET) || defined(__PX4_POSIX)
	int temp_int_arg;
#endif

	while ((ch = px4_getopt(argc, argv, "b:r:d:n:u:o:m:t:c:F:fswxzZp", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			if (px4_get_parameter_value(myoptarg, _baudrate) != 0) {
				PX4_ERR("baudrate parsing failed");
				err_flag = true;
			}

			if (_baudrate < 9600 || _baudrate > 3000000) {
				PX4_ERR("invalid baud rate '%s'", myoptarg);
				err_flag = true;
			}

			break;

		case 'r':
			if (px4_get_parameter_value(myoptarg, _datarate) != 0) {
				PX4_ERR("datarate parsing failed");
				err_flag = true;
			}

			if (_datarate > MAX_DATA_RATE) {
				PX4_ERR("invalid data rate '%s'", myoptarg);
				err_flag = true;
			}

			break;

		case 'd':
			_device_name = myoptarg;
			set_protocol(Protocol::SERIAL);

			if (access(_device_name, F_OK) == -1) {
				PX4_ERR("Device %s does not exist", _device_name);
				err_flag = true;
			}

			break;

		case 'n':
			_interface_name = myoptarg;
			break;

#if defined(MAVLINK_UDP)

		case 'u':
			if (px4_get_parameter_value(myoptarg, temp_int_arg) != 0) {
				PX4_ERR("invalid data udp_port");
				err_flag = true;

			} else {
				_network_port = temp_int_arg;
				set_protocol(Protocol::UDP);
			}

			break;

		case 'o':
			if (px4_get_parameter_value(myoptarg, temp_int_arg) != 0) {
				PX4_ERR("invalid remote udp_port");
				err_flag = true;

			} else {
				_remote_port = temp_int_arg;
				set_protocol(Protocol::UDP);
			}

			break;

		case 't':
			_src_addr.sin_family = AF_INET;

			if (inet_aton(myoptarg, &_src_addr.sin_addr)) {
				_src_addr_initialized = true;

			} else {
				PX4_ERR("invalid partner ip '%s'", myoptarg);
				err_flag = true;
			}

			break;

		case 'p':
			_mav_broadcast = BROADCAST_MODE_ON;
			break;

#if defined(CONFIG_NET_IGMP) && defined(CONFIG_NET_ROUTE)

		// multicast
		case 'c':
			_src_addr.sin_family = AF_INET;

			if (inet_aton(myoptarg, &_src_addr.sin_addr)) {
				_src_addr_initialized = true;
				_mav_broadcast = BROADCAST_MODE_MULTICAST;

			} else {
				PX4_ERR("invalid partner ip '%s'", myoptarg);
				err_flag = true;
			}

			break;
#else

		case 'c':
			PX4_ERR("Multicast option is not supported on this platform");
			err_flag = true;
			break;
#endif
#else

		case 'p':
		case 'u':
		case 'o':
		case 't':
			PX4_ERR("UDP options not supported on this platform");
			err_flag = true;
			break;
#endif

		case 'm': {

				int mode;

				if (px4_get_parameter_value(myoptarg, mode) == 0) {
					if (mode >= 0 && mode < (int)MAVLINK_MODE_COUNT) {
						_mode = (MAVLINK_MODE)mode;

					} else {
						PX4_ERR("invalid mode");
						err_flag = true;
					}

				} else {
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

					} else if (strcmp(myoptarg, "iridium") == 0) {
						_mode = MAVLINK_MODE_IRIDIUM;
						set_telemetry_status_type(telemetry_status_s::LINK_TYPE_IRIDIUM);

					} else if (strcmp(myoptarg, "minimal") == 0) {
						_mode = MAVLINK_MODE_MINIMAL;

					} else if (strcmp(myoptarg, "extvision") == 0) {
						_mode = MAVLINK_MODE_EXTVISION;

					} else if (strcmp(myoptarg, "extvisionmin") == 0) {
						_mode = MAVLINK_MODE_EXTVISIONMIN;

					} else if (strcmp(myoptarg, "gimbal") == 0) {
						_mode = MAVLINK_MODE_GIMBAL;

					} else if (strcmp(myoptarg, "onboard_low_bandwidth") == 0) {
						_mode = MAVLINK_MODE_ONBOARD_LOW_BANDWIDTH;

					} else if (strcmp(myoptarg, "uavionix") == 0) {
						_mode = MAVLINK_MODE_UAVIONIX;

					} else {
						PX4_ERR("invalid mode");
						err_flag = true;
					}
				}

				break;
			}

		case 'F': {
				float freq;

				if (px4_get_parameter_value(myoptarg, freq) != 0) {
					PX4_ERR("iridium mode frequency parsing failed");
					err_flag = true;

				} else {
					if (freq >= 0.f) {
						_high_latency_freq = freq;

					} else {
						PX4_ERR("Invalid value for iridium mode frequency.");
						err_flag = true;
					}
				}

				break;
			}

		case 'f':
			_forwarding_on = true;
			break;

		case 's':
			_use_software_mav_throttling = true;
			break;

		case 'w':
			_wait_to_transmit = true;
			break;

		case 'x':
			_ftp_on = true;
			break;

		case 'z':
			_flow_control = FLOW_CONTROL_ON;
			break;

		case 'Z':
			_flow_control = FLOW_CONTROL_OFF;
			break;

		default:
			err_flag = true;
			break;
		}
	}

	if (err_flag) {
		usage();
		return PX4_ERROR;
	}

	/* USB serial is indicated by /dev/ttyACMx */
	if (strncmp(_device_name, "/dev/ttyACM", 11) == 0) {
		if (_datarate == 0) {
			_datarate = 100000;
		}

		/* USB has no baudrate, but use a magic number for 'fast' */
		_baudrate = 2000000;
		_ftp_on = true;
		_is_usb_uart = true;

		// Always forward messages to/from the USB instance.
		_forwarding_on = true;

		set_telemetry_status_type(telemetry_status_s::LINK_TYPE_USB);
	}

	if (_mode == MAVLINK_MODE_COUNT) {
		_mode = MAVLINK_MODE_NORMAL;
	}

	if (_datarate == 0) {
		/* convert bits to bytes and use 1/2 of bandwidth by default */
		_datarate = _baudrate / 20;
	}

	if (_datarate > MAX_DATA_RATE) {
		_datarate = MAX_DATA_RATE;
	}

	if (get_protocol() == Protocol::SERIAL) {
		if (Mavlink::serial_instance_exists(_device_name, this)) {
			PX4_ERR("%s already running", _device_name);
			return PX4_ERROR;
		}

		PX4_INFO("mode: %s, data rate: %d B/s on %s @ %dB",
			 mavlink_mode_str(_mode), _datarate, _device_name, _baudrate);

		/* flush stdout in case MAVLink is about to take it over */
		fflush(stdout);
	}

#if defined(MAVLINK_UDP)

	else if (get_protocol() == Protocol::UDP) {
		if (Mavlink::get_instance_for_network_port(_network_port) != nullptr) {
			PX4_ERR("port %hu already occupied", _network_port);
			return PX4_ERROR;
		}

		PX4_INFO("mode: %s, data rate: %d B/s on udp port %hu remote port %hu",
			 mavlink_mode_str(_mode), _datarate, _network_port, _remote_port);
	}

#endif // MAVLINK_UDP

	if (set_instance_id()) {
		if (!set_channel()) {
			PX4_ERR("set channel failed");
			return PX4_ERROR;
		}

		// set thread name
		char thread_name[13];
		snprintf(thread_name, sizeof(thread_name), "mavlink_if%d", get_instance_id());
		px4_prctl(PR_SET_NAME, thread_name, px4_getpid());

	} else {
		PX4_ERR("no instances available");
		return PX4_ERROR;
	}

	pthread_mutex_init(&_message_buffer_mutex, nullptr);
	pthread_mutex_init(&_send_mutex, nullptr);
	pthread_mutex_init(&_radio_status_mutex, nullptr);

	/* if we are passing on mavlink messages, we need to prepare a buffer for this instance */
	if (get_forwarding_on()) {
		/* initialize message buffer if multiplexing is on.
		 * make space for two messages plus off-by-one space as we use the empty element
		 * marker ring buffer approach.
		 */
		LockGuard lg{_message_buffer_mutex};

		if (!_message_buffer.allocate(2 * sizeof(mavlink_message_t) + 1)) {
			PX4_ERR("msg buf alloc fail");
			return PX4_ERROR;
		}
	}

	/* Activate sending the data by default (for the IRIDIUM mode it will be disabled after the first round of packages is sent)*/
	_transmitting_enabled = true;
	_transmitting_enabled_commanded = true;

	if (_mode == MAVLINK_MODE_IRIDIUM) {
		_transmitting_enabled_commanded = false;
	}

	/* add default streams depending on mode */
	if (_mode != MAVLINK_MODE_IRIDIUM) {

		/* HEARTBEAT is constant rate stream, rate never adjusted */
		configure_stream("HEARTBEAT", 1.0f);

		/* STATUSTEXT stream */
		configure_stream("STATUSTEXT", 20.0f);

		/* COMMAND_LONG stream: use unlimited rate to send all commands */
		configure_stream("COMMAND_LONG");

	}

	if (configure_streams_to_default() != 0) {
		PX4_ERR("configure_streams_to_default() failed");
	}

	/* set main loop delay depending on data rate to minimize CPU overhead */
	_main_loop_delay = (MAIN_LOOP_DELAY * 1000) / _datarate;

	/* hard limit to 1000 Hz at max */
	if (_main_loop_delay < MAVLINK_MIN_INTERVAL) {
		_main_loop_delay = MAVLINK_MIN_INTERVAL;
	}

	/* hard limit to 100 Hz at least */
	if (_main_loop_delay > MAVLINK_MAX_INTERVAL) {
		_main_loop_delay = MAVLINK_MAX_INTERVAL;
	}

	/* open the UART device after setting the instance, as it might block */
	if (get_protocol() == Protocol::SERIAL) {

		// NOTE: we attempt to open the port multiple times due to sercon returning before
		// the port is ready to be opened. This avoids needing to sleep() after sercon_main.
		int attempts = 0;
		static const int max_attempts = 3;

		while (_uart_fd < 0) {
			_uart_fd = mavlink_open_uart(_baudrate, _device_name, _flow_control);
			attempts++;

			if (_uart_fd < 0 && attempts < max_attempts) {
				PX4_ERR("could not open %s, retrying", _device_name);
				px4_usleep(1_s);

			} else if (_uart_fd < 0) {
				PX4_ERR("failed to open %s after %d attempts, exiting!", _device_name, attempts);
				return PX4_ERROR;
			}
		}
	}

#if defined(MAVLINK_UDP)

	/* init socket if necessary */
	if (get_protocol() == Protocol::UDP) {
		init_udp();
	}

#endif // MAVLINK_UDP

	_task_id = px4_getpid();

	/* if the protocol is serial, we send the system version blindly */
	if (get_protocol() == Protocol::SERIAL) {
		send_autopilot_capabilities();
	}

	_receiver.start();

	uint16_t event_sequence_offset = 0; // offset to account for skipped events, not sent via MAVLink

	_mavlink_start_time = hrt_absolute_time();

	_task_running.store(true);

	while (!should_exit()) {
		/* main loop */
		px4_usleep(_main_loop_delay);

		if (!should_transmit()) {
			check_requested_subscriptions();
			handleStatus();
			handleCommands();
			handleAndGetCurrentCommandAck();
			continue;
		}

		perf_count(_loop_interval_perf);
		perf_begin(_loop_perf);

		const hrt_abstime t = hrt_absolute_time();

		update_rate_mult();

		// check for parameter updates
		if (_parameter_update_sub.updated()) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			mavlink_update_parameters();
		}

		configure_sik_radio();

		handleStatus();
		handleCommands();
		handleAndGetCurrentCommandAck();

		/* check for shell output */
		if (_mavlink_shell && _mavlink_shell->available() > 0) {
			if (get_free_tx_buf() >= MAVLINK_MSG_ID_SERIAL_CONTROL_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) {
				mavlink_serial_control_t msg;
				msg.baudrate = 0;
				msg.flags = SERIAL_CONTROL_FLAG_REPLY;
				msg.timeout = 0;
				msg.device = SERIAL_CONTROL_DEV_SHELL;
				msg.count = _mavlink_shell->read(msg.data, sizeof(msg.data));
				msg.target_system = _mavlink_shell->targetSysid();
				msg.target_component = _mavlink_shell->targetCompid();
				mavlink_msg_serial_control_send_struct(get_channel(), &msg);
			}
		}

		check_requested_subscriptions();

		/* update streams */
		for (const auto &stream : _streams) {
			stream->update(t);

			if (!_first_heartbeat_sent) {
				if (_mode == MAVLINK_MODE_IRIDIUM) {
					if (stream->get_id() == MAVLINK_MSG_ID_HIGH_LATENCY2) {
						_first_heartbeat_sent = stream->first_message_sent();
					}

				} else {
					if (stream->get_id() == MAVLINK_MSG_ID_HEARTBEAT) {
						_first_heartbeat_sent = stream->first_message_sent();
					}
				}
			}
		}

		/* check for ulog streaming messages */
		if (_mavlink_ulog) {
			const int ret = _mavlink_ulog->handle_update(get_channel());

			if (ret < 0) { // abort the streaming on error
				if (ret != -1) {
					PX4_WARN("mavlink ulog stream update failed, stopping (%i)", ret);
				}

				_mavlink_ulog->stop();
				_mavlink_ulog = nullptr;
			}
		}

		/* handle new events */
		if (check_events()) {
			if (_event_sub.updated()) {
				LockGuard lg{mavlink_event_buffer_mutex};

				event_s orb_event;

				while (_event_sub.update(&orb_event)) {
					if (events::externalLogLevel(orb_event.log_levels) == events::LogLevel::Disabled) {
						++event_sequence_offset; // skip this event

					} else {
						events::Event e;
						e.id = orb_event.id;
						e.timestamp_ms = orb_event.timestamp / 1000;
						e.sequence = orb_event.event_sequence - event_sequence_offset;
						e.log_levels = orb_event.log_levels;
						static_assert(sizeof(e.arguments) == sizeof(orb_event.arguments),
							      "uorb message event: arguments size mismatch");
						memcpy(e.arguments, orb_event.arguments, sizeof(orb_event.arguments));
						_event_buffer->insert_event(e);
					}
				}
			}
		}

		_events.update(t);

		/* pass messages from other instances */
		if (get_forwarding_on()) {

			mavlink_message_t msg;
			size_t available_bytes;
			{
				// We only send one message at a time, not to put too much strain on a
				// link from forwarded messages.
				LockGuard lg{_message_buffer_mutex};
				available_bytes = _message_buffer.pop_front(reinterpret_cast<uint8_t *>(&msg), sizeof(msg));
				// We need to make sure to release the lock here before sending the
				// bytes out via IP or UART which could potentially take longer.
			}

			if (available_bytes > 0) {
				resend_message(&msg);
			}
		}

		/* update TX/RX rates*/
		if (t > _bytes_timestamp + 1_s) {
			if (_bytes_timestamp != 0) {
				const float dt = (t - _bytes_timestamp) * 1e-6f;

				_tstatus.tx_rate_avg = _bytes_tx / dt;
				_tstatus.tx_error_rate_avg = _bytes_txerr / dt;
				_tstatus.rx_rate_avg = _bytes_rx / dt;

				_bytes_tx = 0;
				_bytes_txerr = 0;
				_bytes_rx = 0;
			}

			_bytes_timestamp = t;
		}

		// publish status at 1 Hz, or sooner if HEARTBEAT has updated
		if ((hrt_elapsed_time(&_tstatus.timestamp) >= 1_s) || _tstatus_updated) {
			publish_telemetry_status();
		}

		perf_end(_loop_perf);
	}

	_receiver.stop();

	delete _subscribe_to_stream;
	_subscribe_to_stream = nullptr;

	/* delete streams */
	_streams.clear();

	if (_uart_fd >= 0) {
		/* discard all pending data, as close() might block otherwise on NuttX with flow control enabled */
		tcflush(_uart_fd, TCIOFLUSH);
		/* close UART */
		::close(_uart_fd);
	}

	if (_socket_fd >= 0) {
		close(_socket_fd);
		_socket_fd = -1;
	}

	if (_mavlink_ulog) {
		_mavlink_ulog->stop();
		_mavlink_ulog = nullptr;
	}

	pthread_mutex_destroy(&_send_mutex);
	pthread_mutex_destroy(&_radio_status_mutex);
	pthread_mutex_destroy(&_message_buffer_mutex);

	PX4_INFO("exiting channel %i", (int)_channel);

	_task_running.store(false);

	return OK;
}

void Mavlink::handleStatus()
{
	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			/* switch HIL mode if required */
			set_hil_enabled(vehicle_status.hil_state == vehicle_status_s::HIL_STATE_ON);

			if (_mode == MAVLINK_MODE_IRIDIUM) {

				if (_transmitting_enabled && (!vehicle_status.gcs_connection_lost || (vehicle_status.high_latency_data_link_lost &&
							      !_transmitting_enabled_commanded && _first_heartbeat_sent))) {

					_transmitting_enabled = false;
					mavlink_log_info(&_mavlink_log_pub, "Disable transmitting with IRIDIUM mavlink on device %s\t", _device_name);
					events::send<int8_t>(events::ID("mavlink_iridium_disable"), events::Log::Info,
							     "Disabling transmitting with IRIDIUM mavlink on instance {1}", _instance_id);

				} else if (!_transmitting_enabled && vehicle_status.gcs_connection_lost
					   && !vehicle_status.high_latency_data_link_lost) {
					_transmitting_enabled = true;
					mavlink_log_info(&_mavlink_log_pub, "Enable transmitting with IRIDIUM mavlink on device %s\t", _device_name);
					events::send<int8_t>(events::ID("mavlink_iridium_enable"), events::Log::Info,
							     "Enabling transmitting with IRIDIUM mavlink on instance {1}", _instance_id);
				}
			}
		}
	}
}

void Mavlink::handleCommands()
{
	if (_mode == MAVLINK_MODE_IRIDIUM) {
		int vehicle_command_updates = 0;

		while (_vehicle_command_sub.updated() && (vehicle_command_updates < vehicle_command_s::ORB_QUEUE_LENGTH)) {
			vehicle_command_updates++;
			const unsigned last_generation = _vehicle_command_sub.get_last_generation();
			vehicle_command_s vehicle_cmd;

			if (_vehicle_command_sub.update(&vehicle_cmd)) {
				if (_vehicle_command_sub.get_last_generation() != last_generation + 1) {
					PX4_ERR("vehicle_command lost, generation %u -> %u", last_generation, _vehicle_command_sub.get_last_generation());
				}

				if ((vehicle_cmd.command == vehicle_command_s::VEHICLE_CMD_CONTROL_HIGH_LATENCY) &&
				    _mode == MAVLINK_MODE_IRIDIUM) {

					if (vehicle_cmd.param1 > 0.5f) {
						if (!_transmitting_enabled) {
							mavlink_log_info(&_mavlink_log_pub, "Enable transmitting with IRIDIUM mavlink on device %s by command\t",
									 _device_name);
							events::send<int8_t>(events::ID("mavlink_iridium_enable_cmd"), events::Log::Info,
									     "Enabling transmitting with IRIDIUM mavlink on instance {1} by command", _instance_id);
						}

						_transmitting_enabled = true;
						_transmitting_enabled_commanded = true;

					} else {
						if (_transmitting_enabled) {
							mavlink_log_info(&_mavlink_log_pub, "Disable transmitting with IRIDIUM mavlink on device %s by command\t",
									 _device_name);
							events::send<int8_t>(events::ID("mavlink_iridium_disable_cmd"), events::Log::Info,
									     "Disabling transmitting with IRIDIUM mavlink on instance {1} by command", _instance_id);
						}

						_transmitting_enabled = false;
						_transmitting_enabled_commanded = false;
					}

					// send positive command ack
					vehicle_command_ack_s command_ack{};
					command_ack.command = vehicle_cmd.command;
					command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
					command_ack.from_external = !vehicle_cmd.from_external;
					command_ack.target_system = vehicle_cmd.source_system;
					command_ack.target_component = vehicle_cmd.source_component;
					command_ack.timestamp = vehicle_cmd.timestamp;
					_vehicle_command_ack_pub.publish(command_ack);
				}
			}
		}
	}

	// For legacy gimbals using the mavlink gimbal v1 protocol, we need to send out commands.
	// We don't care about acks for these though.
	if (_gimbal_v1_command_sub.updated()) {
		vehicle_command_s cmd;
		_gimbal_v1_command_sub.copy(&cmd);

		// FIXME: filter for target system/component

		mavlink_command_long_t msg{};
		msg.param1 = cmd.param1;
		msg.param2 = cmd.param2;
		msg.param3 = cmd.param3;
		msg.param4 = cmd.param4;
		msg.param5 = cmd.param5;
		msg.param6 = cmd.param6;
		msg.param7 = cmd.param7;
		msg.command = cmd.command;
		msg.target_system = cmd.target_system;
		msg.target_component = cmd.target_component;
		msg.confirmation = 0;

		mavlink_msg_command_long_send_struct(get_channel(), &msg);
	}
}

void Mavlink::handleAndGetCurrentCommandAck()
{
	if (_vehicle_command_ack_sub.updated()) {
		static constexpr size_t COMMAND_ACK_TOTAL_LEN = MAVLINK_MSG_ID_COMMAND_ACK_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;

		while ((get_free_tx_buf() >= COMMAND_ACK_TOTAL_LEN) && _vehicle_command_ack_sub.updated()) {
			vehicle_command_ack_s command_ack;
			const unsigned last_generation = _vehicle_command_ack_sub.get_last_generation();

			if (_vehicle_command_ack_sub.update(&command_ack)) {
				if (_vehicle_command_ack_sub.get_last_generation() != last_generation + 1) {
					PX4_ERR("vehicle_command_ack lost, generation %u -> %u", last_generation,
						_vehicle_command_ack_sub.get_last_generation());
				}

				const bool is_target_known = _receiver.component_was_seen(command_ack.target_system, command_ack.target_component);

				if (!command_ack.from_external
				    && command_ack.command < vehicle_command_s::VEHICLE_CMD_PX4_INTERNAL_START
				    && is_target_known
				    && command_ack.target_component < vehicle_command_s::COMPONENT_MODE_EXECUTOR_START) {

					mavlink_command_ack_t msg{};
					msg.result = command_ack.result;
					msg.command = command_ack.command;
					msg.progress = command_ack.result_param1;
					msg.result_param2 = command_ack.result_param2;
					msg.target_system = command_ack.target_system;
					msg.target_component = command_ack.target_component;

					// Handle logging acks before sending out the mavlink message to prevent a race condition
					if (command_ack.command == vehicle_command_s::VEHICLE_CMD_LOGGING_START) {
						if (_mavlink_ulog) {
							_mavlink_ulog->start_ack_received();
						}

					} else if (command_ack.command == vehicle_command_s::VEHICLE_CMD_LOGGING_STOP
						   && command_ack.result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED) {
						if (_mavlink_ulog) {
							_mavlink_ulog->stop();
							_mavlink_ulog = nullptr;
						}
					}

					if (_mode == MAVLINK_MODE_IRIDIUM) {
						if (command_ack.from_external) {
							// for MAVLINK_MODE_IRIDIUM send only if external
							mavlink_msg_command_ack_send_struct(get_channel(), &msg);
						}

					} else {
						mavlink_msg_command_ack_send_struct(get_channel(), &msg);
					}

				}
			}
		}
	}
}

void Mavlink::check_requested_subscriptions()
{
	if (_subscribe_to_stream != nullptr) {
		if (_subscribe_to_stream_rate < -1.5f) {
			if (configure_streams_to_default(_subscribe_to_stream) == 0) {
				if (get_protocol() == Protocol::SERIAL) {
					PX4_DEBUG("stream %s on device %s set to default rate", _subscribe_to_stream, _device_name);
				}

#if defined(MAVLINK_UDP)

				else if (get_protocol() == Protocol::UDP) {
					PX4_DEBUG("stream %s on UDP port %hu set to default rate", _subscribe_to_stream, _network_port);
				}

#endif // MAVLINK_UDP

			} else {
				PX4_ERR("setting stream %s to default failed", _subscribe_to_stream);
			}

		} else if (configure_stream(_subscribe_to_stream, _subscribe_to_stream_rate) == 0) {
			if (fabsf(_subscribe_to_stream_rate) > 0.00001f) {
				if (get_protocol() == Protocol::SERIAL) {
					PX4_DEBUG("stream %s on device %s enabled with rate %.1f Hz", _subscribe_to_stream, _device_name,
						  (double)_subscribe_to_stream_rate);

				}

#if defined(MAVLINK_UDP)

				else if (get_protocol() == Protocol::UDP) {
					PX4_DEBUG("stream %s on UDP port %hu enabled with rate %.1f Hz", _subscribe_to_stream, _network_port,
						  (double)_subscribe_to_stream_rate);
				}

#endif // MAVLINK_UDP

			} else {
				if (get_protocol() == Protocol::SERIAL) {
					PX4_DEBUG("stream %s on device %s disabled", _subscribe_to_stream, _device_name);

				}

#if defined(MAVLINK_UDP)

				else if (get_protocol() == Protocol::UDP) {
					PX4_DEBUG("stream %s on UDP port %hu disabled", _subscribe_to_stream, _network_port);
				}

#endif // MAVLINK_UDP
			}

		} else {
			if (get_protocol() == Protocol::SERIAL) {
				PX4_ERR("stream %s on device %s not found", _subscribe_to_stream, _device_name);

			}

#if defined(MAVLINK_UDP)

			else if (get_protocol() == Protocol::UDP) {
				PX4_ERR("stream %s on UDP port %hu not found", _subscribe_to_stream, _network_port);
			}

#endif // MAVLINK_UDP
		}

		_subscribe_to_stream = nullptr;
	}
}

void Mavlink::publish_telemetry_status()
{
	// many fields are populated in place

	_tstatus.mode = _mode;
	_tstatus.data_rate = _datarate;
	_tstatus.rate_multiplier = _rate_mult;
	_tstatus.flow_control = get_flow_control_enabled();
	_tstatus.ftp = ftp_enabled();
	_tstatus.forwarding = get_forwarding_on();
	_tstatus.mavlink_v2 = (_protocol_version == 2);

	_tstatus.streams = _streams.size();

	// telemetry_status is also updated from the receiver thread, but never the same fields
	_tstatus.timestamp = hrt_absolute_time();
	_telemetry_status_pub.publish(_tstatus);
	_tstatus_updated = false;
}

void Mavlink::configure_sik_radio()
{
	/* radio config check */
	if (_uart_fd >= 0 && _param_sik_radio_id.get() != 0) {
		/* request to configure radio and radio is present */
		FILE *fs = fdopen(_uart_fd, "w");

		if (fs) {
			/* switch to AT command mode */
			px4_usleep(1200000);
			fprintf(fs, "+++");
			fflush(fs);
			px4_usleep(1200000);

			if (_param_sik_radio_id.get() > 0) {
				/* set channel */
				fprintf(fs, "ATS3=%" PRIu32 "\r\n", _param_sik_radio_id.get());
				px4_usleep(200000);

			} else {
				/* reset to factory defaults */
				fprintf(fs, "AT&F\r\n");
				px4_usleep(200000);
			}

			/* write config */
			fprintf(fs, "AT&W\r\n");
			px4_usleep(200000);

			/* reboot */
			fprintf(fs, "ATZ\r\n");
			px4_usleep(200000);

			// XXX NuttX suffers from a bug where
			// fclose() also closes the fd, not just
			// the file stream. Since this is a one-time
			// config thing, we leave the file struct
			// allocated.
#ifndef __PX4_NUTTX
			fclose(fs);
#endif

		} else {
			PX4_WARN("open fd %d failed", _uart_fd);
		}

		/* reset param and save */
		_param_sik_radio_id.set(0);
		_param_sik_radio_id.commit_no_notification();
	}
}

int Mavlink::start_helper(int argc, char *argv[])
{
	/* create the instance in task context */
	Mavlink *instance = new Mavlink();

	int res;

	if (!instance) {
		/* out of memory */
		res = -ENOMEM;
		PX4_ERR("OUT OF MEM");

	} else {
		/* this will actually only return once MAVLink exits, unless there's a startup error */
		res = instance->task_main(argc, argv);

		if (res != PX4_OK) {
			delete instance;
		}
	}

	return res;
}

int
Mavlink::start(int argc, char *argv[])
{
	MavlinkULog::initialize();
	MavlinkCommandSender::initialize();

	if (!_event_buffer) {
		_event_buffer = new events::EventBuffer();
		int ret;

		if (_event_buffer && (ret = _event_buffer->init()) != 0) {
			PX4_ERR("EventBuffer init failed (%i)", ret);
			delete _event_buffer;
			_event_buffer = nullptr;
		}

		if (!_event_buffer) {
			PX4_ERR("EventBuffer alloc failed");
			return 1;
		}
	}

	// Wait for the instance count to go up one
	// before returning to the shell
	int ic = Mavlink::instance_count();

	if (ic == MAVLINK_COMM_NUM_BUFFERS) {
		PX4_ERR("Maximum MAVLink instance count of %d reached.", MAVLINK_COMM_NUM_BUFFERS);
		return 1;
	}

	// Instantiate thread

	// This is where the control flow splits
	// between the starting task and the spawned
	// task - start_helper() only returns
	// when the started task exits.
	px4_task_spawn_cmd("mavlink_main",
			   SCHED_DEFAULT,
			   SCHED_PRIORITY_DEFAULT,
			   PX4_STACK_ADJUSTED(2896) + MAVLINK_NET_ADDED_STACK,
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
		px4_usleep(sleeptime);
		count++;
	}

	if (ic == Mavlink::instance_count()) {
		return PX4_ERROR;

	} else {
		return PX4_OK;
	}
}

void
Mavlink::display_status()
{
#if !defined(CONSTRAINED_FLASH)
	_receiver.enable_message_statistics();
#endif // !CONSTRAINED_FLASH

	if (_tstatus.heartbeat_type_gcs) {
		printf("\tGCS heartbeat valid\n");
	}

	printf("\tmavlink chan: #%u\n", static_cast<unsigned>(_channel));

	if (_tstatus.timestamp > 0) {

		printf("\ttype:\t\t");

		if (_radio_status_available) {
			printf("RADIO Link\n");
			printf("\t  rssi:\t\t%" PRIu8 "\n", _rstatus.rssi);
			printf("\t  remote rssi:\t%" PRIu8 "\n", _rstatus.remote_rssi);
			printf("\t  txbuf:\t%" PRIu8 "\n", _rstatus.txbuf);
			printf("\t  noise:\t%" PRIu8 "\n", _rstatus.noise);
			printf("\t  remote noise:\t%" PRIu8 "\n", _rstatus.remote_noise);
			printf("\t  rx errors:\t%" PRIu16 "\n", _rstatus.rxerrors);
			printf("\t  fixed:\t%" PRIu16 "\n", _rstatus.fix);

		} else if (_tstatus.type == telemetry_status_s::LINK_TYPE_USB) {
			printf("USB CDC\n");

		} else {
			printf("GENERIC LINK OR RADIO\n");
		}

	} else {
		printf("\tno radio status.\n");
	}

	printf("\tflow control: %s\n", _flow_control_mode ? "ON" : "OFF");
	printf("\trates:\n");
	printf("\t  tx: %.1f B/s\n", (double)_tstatus.tx_rate_avg);
	printf("\t  txerr: %.1f B/s\n", (double)_tstatus.tx_error_rate_avg);
	printf("\t  tx rate mult: %.3f\n", (double)_rate_mult);
	printf("\t  tx rate max: %i B/s\n", _datarate);
	printf("\t  rx: %.1f B/s\n", (double)_tstatus.rx_rate_avg);
	printf("\t  rx loss: %.1f%%\n", (double)_tstatus.rx_message_lost_rate);

#if !defined(CONSTRAINED_FLASH)
	_receiver.print_detailed_rx_stats();
#endif // !CONSTRAINED_FLASH

	if (_mavlink_ulog) {
		printf("\tULog rate: %.1f%% of max %.1f%%\n", (double)_mavlink_ulog->current_data_rate() * 100.,
		       (double)_mavlink_ulog->maximum_data_rate() * 100.);
	}

	printf("\tFTP enabled: %s, TX enabled: %s\n",
	       _ftp_on ? "YES" : "NO",
	       _transmitting_enabled ? "YES" : "NO");
	printf("\tmode: %s\n", mavlink_mode_str(_mode));

	if (_mode == MAVLINK_MODE_IRIDIUM) {
		printf("\t    iridium tx freq: %.3f\n", (double)(_high_latency_freq));
	}

	printf("\tForwarding: %s\n", get_forwarding_on() ? "On" : "Off");
	printf("\tMAVLink version: %" PRId32 "\n", _protocol_version);

	printf("\ttransport protocol: ");

	switch (_protocol) {
#if defined(MAVLINK_UDP)

	case Protocol::UDP:
		printf("UDP (%hu, remote port: %hu)\n", _network_port, _remote_port);
		printf("\tBroadcast enabled: %s\n",
		       broadcast_enabled() ? "YES" : "NO");
#if defined(CONFIG_NET_IGMP) && defined(CONFIG_NET_ROUTE)
		printf("\tMulticast enabled: %s\n",
		       multicast_enabled() ? "YES" : "NO");
#endif
#ifdef __PX4_POSIX

		if (get_client_source_initialized()) {
			printf("\tpartner IP: %s\n", inet_ntoa(get_client_source_address().sin_addr));
		}

#endif
		break;
#endif // MAVLINK_UDP

	case Protocol::SERIAL:
		printf("serial (%s @%i)\n", _device_name, _baudrate);
		break;
	}

	if (_ping_stats.last_ping_time > 0) {
		printf("\tping statistics:\n");
		printf("\t  last: %0.2f ms\n", (double)_ping_stats.last_rtt);
		printf("\t  mean: %0.2f ms\n", (double)_ping_stats.mean_rtt);
		printf("\t  max: %0.2f ms\n", (double)_ping_stats.max_rtt);
		printf("\t  min: %0.2f ms\n", (double)_ping_stats.min_rtt);
		printf("\t  dropped packets: %" PRIi32 "\n", _ping_stats.dropped_packets);
	}
}

void
Mavlink::display_status_streams()
{
	printf("\t%-20s%-16s %s\n", "Name", "Rate Config (current) [Hz]", "Message Size (if active) [B]");

	const float rate_mult = _rate_mult;

	for (const auto &stream : _streams) {
		const int interval = stream->get_interval();
		const unsigned size = stream->get_size();
		char rate_str[20];

		if (interval < 0) {
			strcpy(rate_str, "unlimited");

		} else {
			float rate = 1000000.0f / (float)interval;
			// Note that the actual current rate can be lower if the associated uORB topic updates at a
			// lower rate.
			float rate_current = stream->const_rate() ? rate : rate * rate_mult;
			snprintf(rate_str, sizeof(rate_str), "%6.2f (%.3f)", (double)rate, (double)rate_current);
		}

		printf("\t%-30s%-16s", stream->get_name(), rate_str);

		if (size > 0) {
			printf(" %3u\n", size);

		} else {
			printf("\n");
		}
	}
}

int
Mavlink::stop_command(int argc, char *argv[])
{
	const char *device_name = nullptr;

#if defined(MAVLINK_UDP)
	char *eptr;
	int temp_int_arg;
	unsigned short network_port = 0;
#endif // MAVLINK_UDP

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
		if (0 == strcmp(argv[i], "-d") && i < argc - 1) {
			provided_device = true;
			device_name = argv[i + 1];
			i++;

#if defined(MAVLINK_UDP)

		} else if (0 == strcmp(argv[i], "-u") && i < argc - 1) {
			provided_network_port = true;
			temp_int_arg = strtoul(argv[i + 1], &eptr, 10);

			if (*eptr == '\0') {
				network_port = temp_int_arg;

			} else {
				err_flag = true;
			}

			i++;
#endif // MAVLINK_UDP

		} else {
			err_flag = true;
		}

		i++;
	}

	if (!err_flag) {
		Mavlink *inst = nullptr;

		if (provided_device && !provided_network_port) {
			inst = get_instance_for_device(device_name);

#if defined(MAVLINK_UDP)

		} else if (provided_network_port && !provided_device) {
			inst = get_instance_for_network_port(network_port);
#endif // MAVLINK_UDP

		} else if (provided_device && provided_network_port) {
			PX4_WARN("please provide either a device name or a network port");
			return PX4_ERROR;
		}

		if (inst != nullptr) {
			/* set flag to stop thread and wait for all threads to finish */
			if (inst->running() && !inst->should_exit()) {
				int iterations = 0;

				while ((iterations < 1000) && inst->running()) {
					inst->request_stop();
					iterations++;
					px4_usleep(1000);
				}

				if (inst->running()) {
					PX4_ERR("unable to stop instance %d", inst->get_instance_id());
					return PX4_ERROR;
				}

				LockGuard lg{mavlink_module_mutex};

				for (int mavlink_instance = 0; mavlink_instance < MAVLINK_COMM_NUM_BUFFERS; mavlink_instance++) {
					if (mavlink_module_instances[mavlink_instance] == inst) {
						delete inst;
						mavlink_module_instances[mavlink_instance] = nullptr;
						return PX4_OK;
					}
				}
			}

			return PX4_ERROR;
		}

	} else {
		usage();
	}

	return PX4_ERROR;
}

int
Mavlink::stream_command(int argc, char *argv[])
{
	const char *device_name = DEFAULT_DEVICE_NAME;
	float rate = -1.0f;
	const char *stream_name = nullptr;
#ifdef MAVLINK_UDP
	int temp_int_arg;
	unsigned short network_port = 0;
#endif // MAVLINK_UDP
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

#ifdef MAVLINK_UDP

		} else if (0 == strcmp(argv[i], "-u") && i < argc - 1) {
			provided_network_port = true;

			if (px4_get_parameter_value(argv[i + 1], temp_int_arg) != 0) {
				err_flag = true;

			} else {
				network_port = temp_int_arg;
			}

			i++;
#endif // MAVLINK_UDP

		} else {
			err_flag = true;
		}

		i++;
	}

	if (!err_flag && stream_name != nullptr) {

		Mavlink *inst = nullptr;

		if (provided_device && !provided_network_port) {
			inst = get_instance_for_device(device_name);

#ifdef MAVLINK_UDP

		} else if (provided_network_port && !provided_device) {
			inst = get_instance_for_network_port(network_port);
#endif // MAVLINK_UDP

		} else if (provided_device && provided_network_port) {
			PX4_WARN("please provide either a device name or a network port");
			return 1;
		}

		if (rate < 0.0f) {
			rate = -2.0f; // use default rate
		}

		if (inst != nullptr) {
			inst->configure_stream_threadsafe(stream_name, rate);

		} else {

			// If the link is not running we should complain, but not fall over
			// because this is so easy to get wrong and not fatal. Warning is sufficient.
			if (provided_device) {
				PX4_WARN("mavlink for device %s is not running", device_name);

			}

#ifdef MAVLINK_UDP

			else {
				PX4_WARN("mavlink for network on port %hu is not running", network_port);
			}

#endif // MAVLINK_UDP

			return 1;
		}

	} else {
		usage();
		return 1;
	}

	return OK;
}

void
Mavlink::set_boot_complete()
{
	_boot_complete = true;

#if defined(MAVLINK_UDP)
	LockGuard lg {mavlink_module_mutex};

	for (Mavlink *inst : mavlink_module_instances) {
		if (inst && (inst->get_mode() != MAVLINK_MODE_ONBOARD) &&
		    !inst->broadcast_enabled() && inst->get_protocol() == Protocol::UDP) {

			PX4_INFO("MAVLink only on localhost (set param MAV_{i}_BROADCAST = 1 to enable network)");
		}
	}

#endif // MAVLINK_UDP

}

static void usage()
{

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module implements the MAVLink protocol, which can be used on a Serial link or UDP network connection.
It communicates with the system via uORB: some messages are directly handled in the module (eg. mission
protocol), others are published via uORB (eg. vehicle_command).

Streams are used to send periodic messages with a specific rate, such as the vehicle attitude.
When starting the mavlink instance, a mode can be specified, which defines the set of enabled streams with their rates.
For a running instance, streams can be configured via `mavlink stream` command.

There can be multiple independent instances of the module, each connected to one serial device or network port.

### Implementation
The implementation uses 2 threads, a sending and a receiving thread. The sender runs at a fixed rate and dynamically
reduces the rates of the streams if the combined bandwidth is higher than the configured rate (`-r`) or the
physical link becomes saturated. This can be checked with `mavlink status`, see if `rate mult` is less than 1.

**Careful**: some of the data is accessed and modified from both threads, so when changing code or extend the
functionality, this needs to be take into account, in order to avoid race conditions and corrupt data.

### Examples
Start mavlink on ttyS1 serial with baudrate 921600 and maximum sending rate of 80kB/s:
$ mavlink start -d /dev/ttyS1 -b 921600 -m onboard -r 80000

Start mavlink on UDP port 14556 and enable the HIGHRES_IMU message with 50Hz:
$ mavlink start -u 14556 -r 1000000
$ mavlink stream -u 14556 -s HIGHRES_IMU -r 50
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mavlink", "communication");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start a new instance");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS1", "<file:dev>", "Select Serial Device", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 57600, 9600, 3000000, "Baudrate (can also be p:<param_name>)", true);
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, 10, 10000000, "Maximum sending data rate in B/s (if 0, use baudrate / 20)", true);
#if defined(CONFIG_NET) || defined(__PX4_POSIX)
	PRINT_MODULE_USAGE_PARAM_FLAG('p', "Enable Broadcast", true);
	PRINT_MODULE_USAGE_PARAM_INT('u', 14556, 0, 65536, "Select UDP Network Port (local)", true);
	PRINT_MODULE_USAGE_PARAM_INT('o', 14550, 0, 65536, "Select UDP Network Port (remote)", true);
	PRINT_MODULE_USAGE_PARAM_STRING('t', "127.0.0.1", nullptr, "Partner IP (broadcasting can be enabled via -p flag)", true);
#endif
	PRINT_MODULE_USAGE_PARAM_STRING('m', "normal", "custom|camera|onboard|osd|magic|config|iridium|minimal|extvision|extvisionmin|gimbal|uavionix",
					"Mode: sets default streams and rates", true);
	PRINT_MODULE_USAGE_PARAM_STRING('n', nullptr, "<interface_name>", "wifi/ethernet interface name", true);
#if defined(CONFIG_NET_IGMP) && defined(CONFIG_NET_ROUTE)
	PRINT_MODULE_USAGE_PARAM_STRING('c', nullptr, "Multicast address in the range [239.0.0.0,239.255.255.255]", "Multicast address (multicasting can be enabled via MAV_{i}_BROADCAST param)", true);
#endif
	PRINT_MODULE_USAGE_PARAM_FLOAT('F', 0.015, 0.0, 50.0, "Sets the transmission frequency for iridium mode", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Enable message forwarding to other Mavlink instances", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('w', "Wait to send, until first message received", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('x', "Enable FTP", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('z', "Force hardware flow control always on", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('Z', "Force hardware flow control always off", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("stop-all", "Stop all instances");

	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop a running instance");
#if defined(CONFIG_NET) || defined(__PX4_POSIX)
	PRINT_MODULE_USAGE_PARAM_INT('u', -1, 0, 65536, "Select Mavlink instance via local Network Port", true);
#endif
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "Select Mavlink instance via Serial Device", true);


	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print status for all instances");
	PRINT_MODULE_USAGE_ARG("streams", "Print all enabled streams", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("stream", "Configure the sending rate of a stream for a running instance");
#if defined(CONFIG_NET) || defined(__PX4_POSIX)
	PRINT_MODULE_USAGE_PARAM_INT('u', -1, 0, 65536, "Select Mavlink instance via local Network Port", true);
#endif
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "Select Mavlink instance via Serial Device", true);
	PRINT_MODULE_USAGE_PARAM_STRING('s', nullptr, nullptr, "Mavlink stream to configure", false);
	PRINT_MODULE_USAGE_PARAM_FLOAT('r', -1.0f, 0.0f, 2000.0f, "Rate in Hz (0 = turn off, -1 = set to default)", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("boot_complete",
					 "Enable sending of messages. (Must be) called as last step in startup script.");

}

extern "C" __EXPORT int mavlink_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		return Mavlink::start(argc, argv);

	} else if (!strcmp(argv[1], "stop-all")) {
		return Mavlink::destroy_all_instances();

	} else if (!strcmp(argv[1], "status")) {
		bool show_streams_status = argc > 2 && strcmp(argv[2], "streams") == 0;
		return Mavlink::get_status_all_instances(show_streams_status);

	} else if (!strcmp(argv[1], "stop")) {
		return Mavlink::stop_command(argc, argv);

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
