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
 * @file mavlink_main.cpp
 * MAVLink 1.0 protocol implementation.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <nuttx/config.h>
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
#include <geo/geo.h>
#include <dataman/dataman.h>
#include <mathlib/mathlib.h>
#include <mavlink/mavlink_log.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>

#include "mavlink_bridge_header.h"
#include "mavlink_main.h"
#include "mavlink_messages.h"
#include "mavlink_receiver.h"
#include "mavlink_rate_limiter.h"
#include "mavlink_commands.h"

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#define DEFAULT_DEVICE_NAME	"/dev/ttyS1"
#define MAX_DATA_RATE	10000	// max data rate in bytes/s
#define MAIN_LOOP_DELAY 10000	// 100 Hz @ 1000 bytes/s data rate

static Mavlink *_mavlink_instances = nullptr;

/* TODO: if this is a class member it crashes */
static struct file_operations fops;

/**
 * mavlink app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mavlink_main(int argc, char *argv[]);

static uint64_t last_write_success_times[6] = {0};
static uint64_t last_write_try_times[6] = {0};

/*
 * Internal function to send the bytes through the right serial port
 */
void
mavlink_send_uart_bytes(mavlink_channel_t channel, const uint8_t *ch, int length)
{
	Mavlink *instance;

	switch (channel) {
	case MAVLINK_COMM_0:
		instance = Mavlink::get_instance(0);
		break;

	case MAVLINK_COMM_1:
		instance = Mavlink::get_instance(1);
		break;

	case MAVLINK_COMM_2:
		instance = Mavlink::get_instance(2);
		break;

	case MAVLINK_COMM_3:
		instance = Mavlink::get_instance(3);
		break;
#ifdef MAVLINK_COMM_4

	case MAVLINK_COMM_4:
		instance = Mavlink::get_instance(4);
		break;
#endif
#ifdef MAVLINK_COMM_5

	case MAVLINK_COMM_5:
		instance = Mavlink::get_instance(5);
		break;
#endif
#ifdef MAVLINK_COMM_6

	case MAVLINK_COMM_6:
		instance = Mavlink::get_instance(6);
		break;
#endif
		default:
		return;
	}

	int uart = instance->get_uart_fd();

	ssize_t desired = (sizeof(uint8_t) * length);

	/*
	 * Check if the OS buffer is full and disable HW
	 * flow control if it continues to be full
	 */
	int buf_free = 0;

	if (instance->get_flow_control_enabled()
		&& ioctl(uart, FIONWRITE, (unsigned long)&buf_free) == 0) {

		/* Disable hardware flow control:
		 * if no successful write since a defined time
		 * and if the last try was not the last successful write
		 */
		if (last_write_try_times[(unsigned)channel] != 0 &&
			hrt_elapsed_time(&last_write_success_times[(unsigned)channel]) > 500 * 1000UL &&
			last_write_success_times[(unsigned)channel] !=
			last_write_try_times[(unsigned)channel])
		{
			warnx("DISABLING HARDWARE FLOW CONTROL");
			instance->enable_flow_control(false);
		}

	}

	/* If the wait until transmit flag is on, only transmit after we've received messages.
	   Otherwise, transmit all the time. */
	if (instance->should_transmit()) {
		last_write_try_times[(unsigned)channel] = hrt_absolute_time();

		/* check if there is space in the buffer, let it overflow else */
		if (!ioctl(uart, FIONWRITE, (unsigned long)&buf_free)) {

			if (buf_free < desired) {
				/* we don't want to send anything just in half, so return */
				return;
			}
		}

		ssize_t ret = write(uart, ch, desired);
		if (ret != desired) {
			warnx("TX FAIL");
		} else {
			last_write_success_times[(unsigned)channel] = last_write_try_times[(unsigned)channel];
		}
	}



}

static void usage(void);

Mavlink::Mavlink() :
	_device_name(DEFAULT_DEVICE_NAME),
	_task_should_exit(false),
	next(nullptr),
	_mavlink_fd(-1),
	_task_running(false),
	_hil_enabled(false),
	_use_hil_gps(false),
	_is_usb_uart(false),
	_wait_to_transmit(false),
	_received_messages(false),
	_main_loop_delay(1000),
	_subscriptions(nullptr),
	_streams(nullptr),
	_mission_pub(-1),
	_mode(MAVLINK_MODE_NORMAL),
	_total_counter(0),
	_verbose(false),
	_forwarding_on(false),
	_passing_on(false),
	_uart_fd(-1),
	_mavlink_param_queue_index(0),
	_subscribe_to_stream(nullptr),
	_subscribe_to_stream_rate(0.0f),
	_flow_control_enabled(true),
	_message_buffer({}),
	_param_initialized(false),
	_param_system_id(0),
	_param_component_id(0),
	_param_system_type(0),
	_param_use_hil_gps(0),

/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "mavlink"))
{
	_wpm = &_wpm_s;
	mission.count = 0;
	fops.ioctl = (int (*)(file *, int, long unsigned int))&mavlink_dev_ioctl;

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
		errx(1, "instance ID is out of range");
		break;
	}
}

Mavlink::~Mavlink()
{
	perf_free(_loop_perf);

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

	LL_DELETE(_mavlink_instances, this);
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
Mavlink::forward_message(mavlink_message_t *msg, Mavlink *self)
{

	Mavlink *inst;
	LL_FOREACH(_mavlink_instances, inst) {
		if (inst != self) {
			inst->pass_message(msg);
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

/****************************************************************************
 * MAVLink text message logger
 ****************************************************************************/

int
Mavlink::mavlink_dev_ioctl(struct file *filep, int cmd, unsigned long arg)
{
	switch (cmd) {
	case (int)MAVLINK_IOC_SEND_TEXT_INFO:
	case (int)MAVLINK_IOC_SEND_TEXT_CRITICAL:
	case (int)MAVLINK_IOC_SEND_TEXT_EMERGENCY: {

			const char *txt = (const char *)arg;
//		printf("logmsg: %s\n", txt);
			struct mavlink_logmessage msg;
			strncpy(msg.text, txt, sizeof(msg.text));

			Mavlink *inst;
			LL_FOREACH(_mavlink_instances, inst) {
				if (!inst->_task_should_exit) {
					mavlink_logbuffer_write(&inst->_logbuffer, &msg);
					inst->_total_counter++;
				}
			}

			return OK;
		}

	default:
		return ENOTTY;
	}
}

void Mavlink::mavlink_update_system(void)
{

	if (!_param_initialized) {
		_param_system_id = param_find("MAV_SYS_ID");
		_param_component_id = param_find("MAV_COMP_ID");
		_param_system_type = param_find("MAV_TYPE");
		_param_use_hil_gps = param_find("MAV_USEHILGPS");
		_param_initialized = true;
	}

	/* update system and component id */
	int32_t system_id;
	param_get(_param_system_id, &system_id);

	if (system_id > 0 && system_id < 255) {
		mavlink_system.sysid = system_id;
	}

	int32_t component_id;
	param_get(_param_component_id, &component_id);

	if (component_id > 0 && component_id < 255) {
		mavlink_system.compid = component_id;
	}

	int32_t system_type;
	param_get(_param_system_type, &system_type);

	if (system_type >= 0 && system_type < MAV_TYPE_ENUM_END) {
		mavlink_system.type = system_type;
	}

	int32_t use_hil_gps;
	param_get(_param_use_hil_gps, &use_hil_gps);

	_use_hil_gps = (bool)use_hil_gps;
}

int Mavlink::mavlink_open_uart(int baud, const char *uart_name, struct termios *uart_config_original, bool *is_usb)
{
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
		warnx("ERROR: Unsupported baudrate: %d\n\tsupported examples:\n\t9600, 19200, 38400, 57600\t\n115200\n230400\n460800\n921600\n", baud);
		return -EINVAL;
	}

	/* open uart */
	_uart_fd = open(uart_name, O_RDWR | O_NOCTTY);

	if (_uart_fd < 0) {
		return _uart_fd;
	}


	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;
	*is_usb = false;

	/* Back up the original uart configuration to restore it after exit */
	if ((termios_state = tcgetattr(_uart_fd, uart_config_original)) < 0) {
		warnx("ERR GET CONF %s: %d\n", uart_name, termios_state);
		close(_uart_fd);
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
			close(_uart_fd);
			return -1;
		}

	}

	if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
		warnx("ERR SET CONF %s\n", uart_name);
		close(_uart_fd);
		return -1;
	}

	if (!_is_usb_uart) {
		/*
		 * Setup hardware flow control. If the port has no RTS pin this call will fail,
		 * which is not an issue, but requires a separate call so we can fail silently.
		 */
		(void)tcgetattr(_uart_fd, &uart_config);
		uart_config.c_cflag |= CRTS_IFLOW;
		(void)tcsetattr(_uart_fd, TCSANOW, &uart_config);

		/* setup output flow control */
		if (enable_flow_control(true)) {
			warnx("hardware flow control not supported");
		}
	}

	return _uart_fd;
}

int
Mavlink::enable_flow_control(bool enabled)
{
	// We can't do this on USB - skip
	if (_is_usb_uart) {
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
		float rate_mult = _datarate / 1000.0f;
		configure_stream("HIL_CONTROLS", 15.0f * rate_mult);
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

extern mavlink_system_t mavlink_system;

int Mavlink::mavlink_pm_queued_send()
{
	if (_mavlink_param_queue_index < param_count()) {
		mavlink_pm_send_param(param_for_index(_mavlink_param_queue_index));
		_mavlink_param_queue_index++;
		return 0;

	} else {
		return 1;
	}
}

void Mavlink::mavlink_pm_start_queued_send()
{
	_mavlink_param_queue_index = 0;
}

int Mavlink::mavlink_pm_send_param_for_index(uint16_t index)
{
	return mavlink_pm_send_param(param_for_index(index));
}

int Mavlink::mavlink_pm_send_param_for_name(const char *name)
{
	return mavlink_pm_send_param(param_find(name));
}

int Mavlink::mavlink_pm_send_param(param_t param)
{
	if (param == PARAM_INVALID) { return 1; }

	/* buffers for param transmission */
	char name_buf[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN];
	float val_buf;
	mavlink_message_t tx_msg;

	/* query parameter type */
	param_type_t type = param_type(param);
	/* copy parameter name */
	strncpy((char *)name_buf, param_name(param), MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);

	/*
	 * Map onboard parameter type to MAVLink type,
	 * endianess matches (both little endian)
	 */
	uint8_t mavlink_type;

	if (type == PARAM_TYPE_INT32) {
		mavlink_type = MAVLINK_TYPE_INT32_T;

	} else if (type == PARAM_TYPE_FLOAT) {
		mavlink_type = MAVLINK_TYPE_FLOAT;

	} else {
		mavlink_type = MAVLINK_TYPE_FLOAT;
	}

	/*
	 * get param value, since MAVLink encodes float and int params in the same
	 * space during transmission, copy param onto float val_buf
	 */

	int ret;

	if ((ret = param_get(param, &val_buf)) != OK) {
		return ret;
	}

	mavlink_msg_param_value_pack_chan(mavlink_system.sysid,
					  mavlink_system.compid,
					  _channel,
					  &tx_msg,
					  name_buf,
					  val_buf,
					  mavlink_type,
					  param_count(),
					  param_get_index(param));
	mavlink_missionlib_send_message(&tx_msg);
	return OK;
}

void Mavlink::mavlink_pm_message_handler(const mavlink_channel_t chan, const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
			mavlink_param_request_list_t req;
			mavlink_msg_param_request_list_decode(msg, &req);
			if (req.target_system == mavlink_system.sysid &&
					(req.target_component == mavlink_system.compid || req.target_component == MAV_COMP_ID_ALL)) {
				/* Start sending parameters */
				mavlink_pm_start_queued_send();
				mavlink_missionlib_send_gcs_string("[mavlink pm] sending list");
			}
		} break;

	case MAVLINK_MSG_ID_PARAM_SET: {

			/* Handle parameter setting */

			if (msg->msgid == MAVLINK_MSG_ID_PARAM_SET) {
				mavlink_param_set_t mavlink_param_set;
				mavlink_msg_param_set_decode(msg, &mavlink_param_set);

				if (mavlink_param_set.target_system == mavlink_system.sysid && ((mavlink_param_set.target_component == mavlink_system.compid) || (mavlink_param_set.target_component == MAV_COMP_ID_ALL))) {
					/* local name buffer to enforce null-terminated string */
					char name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
					strncpy(name, mavlink_param_set.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
					/* enforce null termination */
					name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';
					/* attempt to find parameter, set and send it */
					param_t param = param_find(name);

					if (param == PARAM_INVALID) {
						char buf[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
						sprintf(buf, "[pm] unknown: %s", name);
						mavlink_missionlib_send_gcs_string(buf);

					} else {
						/* set and send parameter */
						param_set(param, &(mavlink_param_set.param_value));
						mavlink_pm_send_param(param);
					}
				}
			}
		} break;

	case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
			mavlink_param_request_read_t mavlink_param_request_read;
			mavlink_msg_param_request_read_decode(msg, &mavlink_param_request_read);

			if (mavlink_param_request_read.target_system == mavlink_system.sysid && ((mavlink_param_request_read.target_component == mavlink_system.compid) || (mavlink_param_request_read.target_component == MAV_COMP_ID_ALL))) {
				/* when no index is given, loop through string ids and compare them */
				if (mavlink_param_request_read.param_index == -1) {
					/* local name buffer to enforce null-terminated string */
					char name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
					strncpy(name, mavlink_param_request_read.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
					/* enforce null termination */
					name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';
					/* attempt to find parameter and send it */
					mavlink_pm_send_param_for_name(name);

				} else {
					/* when index is >= 0, send this parameter again */
					mavlink_pm_send_param_for_index(mavlink_param_request_read.param_index);
				}
			}

		} break;
	}
}

void Mavlink::publish_mission()
{
	/* Initialize mission publication if necessary */
	if (_mission_pub < 0) {
		_mission_pub = orb_advertise(ORB_ID(offboard_mission), &mission);

	} else {
		orb_publish(ORB_ID(offboard_mission), _mission_pub, &mission);
	}
}

int Mavlink::map_mavlink_mission_item_to_mission_item(const mavlink_mission_item_t *mavlink_mission_item, struct mission_item_s *mission_item)
{
	/* only support global waypoints for now */
	switch (mavlink_mission_item->frame) {
	case MAV_FRAME_GLOBAL:
		mission_item->lat = (double)mavlink_mission_item->x;
		mission_item->lon = (double)mavlink_mission_item->y;
		mission_item->altitude = mavlink_mission_item->z;
		mission_item->altitude_is_relative = false;
		break;

	case MAV_FRAME_GLOBAL_RELATIVE_ALT:
		mission_item->lat = (double)mavlink_mission_item->x;
		mission_item->lon = (double)mavlink_mission_item->y;
		mission_item->altitude = mavlink_mission_item->z;
		mission_item->altitude_is_relative = true;
		break;

	case MAV_FRAME_LOCAL_NED:
	case MAV_FRAME_LOCAL_ENU:
		return MAV_MISSION_UNSUPPORTED_FRAME;

	case MAV_FRAME_MISSION:
	default:
		return MAV_MISSION_ERROR;
	}

	switch (mavlink_mission_item->command) {
	case MAV_CMD_NAV_TAKEOFF:
		mission_item->pitch_min = mavlink_mission_item->param1;
		break;

	default:
		mission_item->acceptance_radius = mavlink_mission_item->param2;
		mission_item->time_inside = mavlink_mission_item->param1;
		break;
	}

	mission_item->yaw = _wrap_pi(mavlink_mission_item->param4 * M_DEG_TO_RAD_F);
	mission_item->loiter_radius = fabsf(mavlink_mission_item->param3);
	mission_item->loiter_direction = (mavlink_mission_item->param3 > 0) ? 1 : -1; /* 1 if positive CW, -1 if negative CCW */
	mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;

	mission_item->autocontinue = mavlink_mission_item->autocontinue;
	// mission_item->index = mavlink_mission_item->seq;
	mission_item->origin = ORIGIN_MAVLINK;

	return OK;
}

int Mavlink::map_mission_item_to_mavlink_mission_item(const struct mission_item_s *mission_item, mavlink_mission_item_t *mavlink_mission_item)
{
	if (mission_item->altitude_is_relative) {
		mavlink_mission_item->frame = MAV_FRAME_GLOBAL;

	} else {
		mavlink_mission_item->frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
	}

	switch (mission_item->nav_cmd) {
	case NAV_CMD_TAKEOFF:
		mavlink_mission_item->param1 = mission_item->pitch_min;
		break;

	default:
		mavlink_mission_item->param2 = mission_item->acceptance_radius;
		mavlink_mission_item->param1 = mission_item->time_inside;
		break;
	}

	mavlink_mission_item->x = (float)mission_item->lat;
	mavlink_mission_item->y = (float)mission_item->lon;
	mavlink_mission_item->z = mission_item->altitude;

	mavlink_mission_item->param4 = mission_item->yaw * M_RAD_TO_DEG_F;
	mavlink_mission_item->param3 = mission_item->loiter_radius * (float)mission_item->loiter_direction;
	mavlink_mission_item->command = mission_item->nav_cmd;
	mavlink_mission_item->autocontinue = mission_item->autocontinue;
	// mavlink_mission_item->seq = mission_item->index;

	return OK;
}

void Mavlink::mavlink_wpm_init(mavlink_wpm_storage *state)
{
	state->size = 0;
	state->max_size = MAVLINK_WPM_MAX_WP_COUNT;
	state->current_state = MAVLINK_WPM_STATE_IDLE;
	state->current_partner_sysid = 0;
	state->current_partner_compid = 0;
	state->timestamp_lastaction = 0;
	state->timestamp_last_send_setpoint = 0;
	state->timestamp_last_send_request = 0;
	state->timeout = MAVLINK_WPM_PROTOCOL_TIMEOUT_DEFAULT;
	state->current_dataman_id = 0;
}

/*
 *  @brief Sends an waypoint ack message
 */
void Mavlink::mavlink_wpm_send_waypoint_ack(uint8_t sysid, uint8_t compid, uint8_t type)
{
	mavlink_message_t msg;
	mavlink_mission_ack_t wpa;

	wpa.target_system = sysid;
	wpa.target_component = compid;
	wpa.type = type;

	mavlink_msg_mission_ack_encode_chan(mavlink_system.sysid, _mavlink_wpm_comp_id, _channel, &msg, &wpa);
	mavlink_missionlib_send_message(&msg);

	if (_verbose) { warnx("Sent waypoint ack (%u) to ID %u", wpa.type, wpa.target_system); }
}

/*
 *  @brief Broadcasts the new target waypoint and directs the MAV to fly there
 *
 *  This function broadcasts its new active waypoint sequence number and
 *  sends a message to the controller, advising it to fly to the coordinates
 *  of the waypoint with a given orientation
 *
 *  @param seq The waypoint sequence number the MAV should fly to.
 */
void Mavlink::mavlink_wpm_send_waypoint_current(uint16_t seq)
{
	if (seq < _wpm->size) {
		mavlink_message_t msg;
		mavlink_mission_current_t wpc;

		wpc.seq = seq;

		mavlink_msg_mission_current_encode_chan(mavlink_system.sysid, _mavlink_wpm_comp_id, _channel, &msg, &wpc);
		mavlink_missionlib_send_message(&msg);

	} else if (seq == 0 && _wpm->size == 0) {

		/* don't broadcast if no WPs */

	} else {
		mavlink_missionlib_send_gcs_string("ERROR: wp index out of bounds");
	}
}

void Mavlink::mavlink_wpm_send_waypoint_count(uint8_t sysid, uint8_t compid, uint16_t count)
{
	mavlink_message_t msg;
	mavlink_mission_count_t wpc;

	wpc.target_system = sysid;
	wpc.target_component = compid;
	wpc.count = mission.count;

	mavlink_msg_mission_count_encode_chan(mavlink_system.sysid, _mavlink_wpm_comp_id, _channel, &msg, &wpc);
	mavlink_missionlib_send_message(&msg);

	if (_verbose) { warnx("Sent waypoint count (%u) to ID %u", wpc.count, wpc.target_system); }
}

void Mavlink::mavlink_wpm_send_waypoint(uint8_t sysid, uint8_t compid, uint16_t seq)
{

	struct mission_item_s mission_item;
	ssize_t len = sizeof(struct mission_item_s);

	dm_item_t dm_current;

	if (_wpm->current_dataman_id == 0) {
		dm_current = DM_KEY_WAYPOINTS_OFFBOARD_0;

	} else {
		dm_current = DM_KEY_WAYPOINTS_OFFBOARD_1;
	}

	if (dm_read(dm_current, seq, &mission_item, len) == len) {

		/* create mission_item_s from mavlink_mission_item_t */
		mavlink_mission_item_t wp;
		map_mission_item_to_mavlink_mission_item(&mission_item, &wp);

		mavlink_message_t msg;
		wp.target_system = sysid;
		wp.target_component = compid;
		wp.seq = seq;
		mavlink_msg_mission_item_encode_chan(mavlink_system.sysid, _mavlink_wpm_comp_id, _channel, &msg, &wp);
		mavlink_missionlib_send_message(&msg);

		if (_verbose) { warnx("Sent waypoint %u to ID %u", wp.seq, wp.target_system); }

	} else {
		mavlink_wpm_send_waypoint_ack(_wpm->current_partner_sysid, _wpm->current_partner_compid, MAV_MISSION_ERROR);
		mavlink_missionlib_send_gcs_string("#audio: Unable to read from micro SD");

		if (_verbose) { warnx("ERROR: could not read WP%u", seq); }
	}
}

void Mavlink::mavlink_wpm_send_waypoint_request(uint8_t sysid, uint8_t compid, uint16_t seq)
{
	if (seq < _wpm->max_size) {
		mavlink_message_t msg;
		mavlink_mission_request_t wpr;
		wpr.target_system = sysid;
		wpr.target_component = compid;
		wpr.seq = seq;
		mavlink_msg_mission_request_encode_chan(mavlink_system.sysid, _mavlink_wpm_comp_id, _channel, &msg, &wpr);
		mavlink_missionlib_send_message(&msg);
		_wpm->timestamp_last_send_request = hrt_absolute_time();

		if (_verbose) { warnx("Sent waypoint request %u to ID %u", wpr.seq, wpr.target_system); }

	} else {
		mavlink_missionlib_send_gcs_string("ERROR: Waypoint index exceeds list capacity");
	}
}

/*
 *  @brief emits a message that a waypoint reached
 *
 *  This function broadcasts a message that a waypoint is reached.
 *
 *  @param seq The waypoint sequence number the MAV has reached.
 */
void Mavlink::mavlink_wpm_send_waypoint_reached(uint16_t seq)
{
	mavlink_message_t msg;
	mavlink_mission_item_reached_t wp_reached;

	wp_reached.seq = seq;

	mavlink_msg_mission_item_reached_encode_chan(mavlink_system.sysid, _mavlink_wpm_comp_id, _channel, &msg, &wp_reached);
	mavlink_missionlib_send_message(&msg);

	if (_verbose) { warnx("Sent waypoint %u reached message", wp_reached.seq); }
}

void Mavlink::mavlink_waypoint_eventloop(uint64_t now)
{
	/* check for timed-out operations */
	if (now - _wpm->timestamp_lastaction > _wpm->timeout && _wpm->current_state != MAVLINK_WPM_STATE_IDLE) {

		mavlink_missionlib_send_gcs_string("Operation timeout");

		_wpm->current_state = MAVLINK_WPM_STATE_IDLE;
		_wpm->current_partner_sysid = 0;
		_wpm->current_partner_compid = 0;

	} else if (now - _wpm->timestamp_last_send_request > 500000 && _wpm->current_state == MAVLINK_WPM_STATE_GETLIST_GETWPS) {
		/* try to get WP again after short timeout */
		mavlink_wpm_send_waypoint_request(_wpm->current_partner_sysid, _wpm->current_partner_compid, _wpm->current_wp_id);
	}
}


void Mavlink::mavlink_wpm_message_handler(const mavlink_message_t *msg)
{
	uint64_t now = hrt_absolute_time();

	switch (msg->msgid) {

	case MAVLINK_MSG_ID_MISSION_ACK: {
			mavlink_mission_ack_t wpa;
			mavlink_msg_mission_ack_decode(msg, &wpa);

			if ((msg->sysid == _wpm->current_partner_sysid && msg->compid == _wpm->current_partner_compid) && (wpa.target_system == mavlink_system.sysid /*&& wpa.target_component == mavlink_wpm_comp_id*/)) {
				_wpm->timestamp_lastaction = now;

				if (_wpm->current_state == MAVLINK_WPM_STATE_SENDLIST || _wpm->current_state == MAVLINK_WPM_STATE_SENDLIST_SENDWPS) {
					if (_wpm->current_wp_id == _wpm->size - 1) {

						_wpm->current_state = MAVLINK_WPM_STATE_IDLE;
						_wpm->current_wp_id = 0;
					}
				}

			} else {
				mavlink_missionlib_send_gcs_string("REJ. WP CMD: curr partner id mismatch");
			}

			break;
		}

	case MAVLINK_MSG_ID_MISSION_SET_CURRENT: {
			mavlink_mission_set_current_t wpc;
			mavlink_msg_mission_set_current_decode(msg, &wpc);

			if (wpc.target_system == mavlink_system.sysid /*&& wpc.target_component == mavlink_wpm_comp_id*/) {
				_wpm->timestamp_lastaction = now;

				if (_wpm->current_state == MAVLINK_WPM_STATE_IDLE) {
					if (wpc.seq < _wpm->size) {

						mission.current_index = wpc.seq;
						publish_mission();

						/* don't answer yet, wait for the navigator to respond, then publish the mission_result */
//						mavlink_wpm_send_waypoint_current(wpc.seq);

					} else {
						mavlink_missionlib_send_gcs_string("IGN WP CURR CMD: Not in list");
					}

				} else {
					mavlink_missionlib_send_gcs_string("IGN WP CURR CMD: Busy");
				}
			}

			break;
		}

	case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
			mavlink_mission_request_list_t wprl;
			mavlink_msg_mission_request_list_decode(msg, &wprl);

			if (wprl.target_system == mavlink_system.sysid /*&& wprl.target_component == mavlink_wpm_comp_id*/) {
				_wpm->timestamp_lastaction = now;

				if (_wpm->current_state == MAVLINK_WPM_STATE_IDLE || _wpm->current_state == MAVLINK_WPM_STATE_SENDLIST) {
					if (_wpm->size > 0) {

						_wpm->current_state = MAVLINK_WPM_STATE_SENDLIST;
						_wpm->current_wp_id = 0;
						_wpm->current_partner_sysid = msg->sysid;
						_wpm->current_partner_compid = msg->compid;

					} else {
						if (_verbose) { warnx("No waypoints send"); }
					}

					_wpm->current_count = _wpm->size;
					mavlink_wpm_send_waypoint_count(msg->sysid, msg->compid, _wpm->current_count);

				} else {
					mavlink_missionlib_send_gcs_string("IGN REQUEST LIST: Busy");
				}
			}

			break;
		}

	case MAVLINK_MSG_ID_MISSION_REQUEST: {
			mavlink_mission_request_t wpr;
			mavlink_msg_mission_request_decode(msg, &wpr);

			if (msg->sysid == _wpm->current_partner_sysid && msg->compid == _wpm->current_partner_compid && wpr.target_system == mavlink_system.sysid /*&& wpr.target_component == mavlink_wpm_comp_id*/) {
				_wpm->timestamp_lastaction = now;

				if (wpr.seq >= _wpm->size) {

					mavlink_missionlib_send_gcs_string("REJ. WP CMD: Req. WP not in list");

					break;
				}

				/*
				 * Ensure that we are in the correct state and that the first request has id 0
				 * and the following requests have either the last id (re-send last waypoint) or last_id+1 (next waypoint)
				 */
				if (_wpm->current_state == MAVLINK_WPM_STATE_SENDLIST) {

					if (wpr.seq == 0) {
						if (_verbose) { warnx("Got ITEM_REQUEST of waypoint %u from %u changing to STATE_SENDLIST_SENDWPS", wpr.seq, msg->sysid); }

						_wpm->current_state = MAVLINK_WPM_STATE_SENDLIST_SENDWPS;

					} else {
						mavlink_missionlib_send_gcs_string("REJ. WP CMD: First id != 0");

						break;
					}

				} else if (_wpm->current_state == MAVLINK_WPM_STATE_SENDLIST_SENDWPS) {

					if (wpr.seq == _wpm->current_wp_id) {

						if (_verbose) { warnx("Got ITEM_REQUEST of waypoint %u (again) from %u staying in STATE_SENDLIST_SENDWPS", wpr.seq, msg->sysid); }

					} else if (wpr.seq == _wpm->current_wp_id + 1) {

						if (_verbose) { warnx("Got ITEM_REQUEST of waypoint %u from %u staying in STATE_SENDLIST_SENDWPS", wpr.seq, msg->sysid); }

					} else {
						mavlink_missionlib_send_gcs_string("REJ. WP CMD: Req. WP was unexpected");

						break;
					}

				} else {

					mavlink_missionlib_send_gcs_string("REJ. WP CMD: Busy");

					break;
				}

				_wpm->current_wp_id = wpr.seq;
				_wpm->current_state = MAVLINK_WPM_STATE_SENDLIST_SENDWPS;

				if (wpr.seq < _wpm->size) {

					mavlink_wpm_send_waypoint(_wpm->current_partner_sysid, _wpm->current_partner_compid, _wpm->current_wp_id);

				} else {
					mavlink_wpm_send_waypoint_ack(_wpm->current_partner_sysid, _wpm->current_partner_compid, MAV_MISSION_ERROR);

					mavlink_missionlib_send_gcs_string("ERROR: Waypoint out of bounds");
				}


			} else {
				//we we're target but already communicating with someone else
				if ((wpr.target_system == mavlink_system.sysid /*&& wpr.target_component == mavlink_wpm_comp_id*/) && !(msg->sysid == _wpm->current_partner_sysid && msg->compid == _wpm->current_partner_compid)) {

					mavlink_missionlib_send_gcs_string("REJ. WP CMD: Busy");

					if (_verbose) { warnx("Ignored MAVLINK_MSG_ID_MISSION_ITEM_REQUEST from ID %u because i'm already talking to ID %u.", msg->sysid, _wpm->current_partner_sysid); }
				}
			}

			break;
		}

	case MAVLINK_MSG_ID_MISSION_COUNT: {
			mavlink_mission_count_t wpc;
			mavlink_msg_mission_count_decode(msg, &wpc);

			if (wpc.target_system == mavlink_system.sysid/* && wpc.target_component == mavlink_wpm_comp_id*/) {
				_wpm->timestamp_lastaction = now;

				if (_wpm->current_state == MAVLINK_WPM_STATE_IDLE) {

					if (wpc.count > NUM_MISSIONS_SUPPORTED) {
						if (_verbose) { warnx("Too many waypoints: %d, supported: %d", wpc.count, NUM_MISSIONS_SUPPORTED); }

						mavlink_wpm_send_waypoint_ack(_wpm->current_partner_sysid, _wpm->current_partner_compid, MAV_MISSION_NO_SPACE);
						break;
					}

					if (wpc.count == 0) {
						mavlink_missionlib_send_gcs_string("WP COUNT 0");

						break;
					}

					_wpm->current_state = MAVLINK_WPM_STATE_GETLIST;
					_wpm->current_wp_id = 0;
					_wpm->current_partner_sysid = msg->sysid;
					_wpm->current_partner_compid = msg->compid;
					_wpm->current_count = wpc.count;

					mavlink_wpm_send_waypoint_request(_wpm->current_partner_sysid, _wpm->current_partner_compid, _wpm->current_wp_id);

				} else if (_wpm->current_state == MAVLINK_WPM_STATE_GETLIST) {

					if (_wpm->current_wp_id == 0) {
						mavlink_missionlib_send_gcs_string("WP CMD OK AGAIN");

					} else {
						mavlink_missionlib_send_gcs_string("REJ. WP CMD: Busy with WP");
					}

				} else {
					mavlink_missionlib_send_gcs_string("IGN MISSION_COUNT CMD: Busy");
				}
			}
		}
		break;

	case MAVLINK_MSG_ID_MISSION_ITEM: {
			mavlink_mission_item_t wp;
			mavlink_msg_mission_item_decode(msg, &wp);

			if (wp.target_system == mavlink_system.sysid && wp.target_component == _mavlink_wpm_comp_id) {

				_wpm->timestamp_lastaction = now;

				/*
				 * ensure that we are in the correct state and that the first waypoint has id 0
				 * and the following waypoints have the correct ids
				 */

				if (_wpm->current_state == MAVLINK_WPM_STATE_GETLIST) {

					if (wp.seq != 0) {
						mavlink_missionlib_send_gcs_string("Ignored MISSION_ITEM WP not 0");
						break;
					}

				} else if (_wpm->current_state == MAVLINK_WPM_STATE_GETLIST_GETWPS) {

					if (wp.seq >= _wpm->current_count) {
						mavlink_missionlib_send_gcs_string("Ignored MISSION_ITEM WP out of bounds");
						break;
					}

					if (wp.seq != _wpm->current_wp_id) {
						mavlink_missionlib_send_gcs_string("IGN: waypoint ID mismatch");
						mavlink_wpm_send_waypoint_request(_wpm->current_partner_sysid, _wpm->current_partner_compid, _wpm->current_wp_id);
						break;
					}
				}

				_wpm->current_state = MAVLINK_WPM_STATE_GETLIST_GETWPS;

				struct mission_item_s mission_item;

				int ret = map_mavlink_mission_item_to_mission_item(&wp, &mission_item);

				if (ret != OK) {
					mavlink_wpm_send_waypoint_ack(_wpm->current_partner_sysid, _wpm->current_partner_compid, ret);
					_wpm->current_state = MAVLINK_WPM_STATE_IDLE;
					break;
				}

				ssize_t len = sizeof(struct mission_item_s);

				dm_item_t dm_next;

				if (_wpm->current_dataman_id == 0) {
					dm_next = DM_KEY_WAYPOINTS_OFFBOARD_1;
					mission.dataman_id = 1;

				} else {
					dm_next = DM_KEY_WAYPOINTS_OFFBOARD_0;
					mission.dataman_id = 0;
				}

				if (dm_write(dm_next, wp.seq, DM_PERSIST_IN_FLIGHT_RESET, &mission_item, len) != len) {
					mavlink_wpm_send_waypoint_ack(_wpm->current_partner_sysid, _wpm->current_partner_compid, MAV_MISSION_ERROR);
					mavlink_missionlib_send_gcs_string("#audio: Unable to write on micro SD");
					_wpm->current_state = MAVLINK_WPM_STATE_IDLE;
					break;
				}

//				if (wp.current) {
//					warnx("current is: %d", wp.seq);
//					mission.current_index = wp.seq;
//				}
				// XXX ignore current set
				mission.current_index = -1;

				_wpm->current_wp_id = wp.seq + 1;

				if (_wpm->current_wp_id == _wpm->current_count && _wpm->current_state == MAVLINK_WPM_STATE_GETLIST_GETWPS) {

					if (_verbose) { warnx("Got all %u waypoints, changing state to MAVLINK_WPM_STATE_IDLE", _wpm->current_count); }

					mavlink_wpm_send_waypoint_ack(_wpm->current_partner_sysid, _wpm->current_partner_compid, MAV_MISSION_ACCEPTED);

					mission.count = _wpm->current_count;

					publish_mission();

					_wpm->current_dataman_id = mission.dataman_id;
					_wpm->size = _wpm->current_count;

					_wpm->current_state = MAVLINK_WPM_STATE_IDLE;

				} else {
					mavlink_wpm_send_waypoint_request(_wpm->current_partner_sysid, _wpm->current_partner_compid, _wpm->current_wp_id);
				}
			}

			break;
		}

	case MAVLINK_MSG_ID_MISSION_CLEAR_ALL: {
			mavlink_mission_clear_all_t wpca;
			mavlink_msg_mission_clear_all_decode(msg, &wpca);

			if (wpca.target_system == mavlink_system.sysid /*&& wpca.target_component == mavlink_wpm_comp_id */) {

				if (_wpm->current_state == MAVLINK_WPM_STATE_IDLE) {
					_wpm->timestamp_lastaction = now;

					_wpm->size = 0;

					/* prepare mission topic */
					mission.dataman_id = -1;
					mission.count = 0;
					mission.current_index = -1;
					publish_mission();

					if (dm_clear(DM_KEY_WAYPOINTS_OFFBOARD_0) == OK && dm_clear(DM_KEY_WAYPOINTS_OFFBOARD_1) == OK) {
						mavlink_wpm_send_waypoint_ack(_wpm->current_partner_sysid, _wpm->current_partner_compid, MAV_MISSION_ACCEPTED);

					} else {
						mavlink_wpm_send_waypoint_ack(_wpm->current_partner_sysid, _wpm->current_partner_compid, MAV_MISSION_ERROR);
					}


				} else {
					mavlink_missionlib_send_gcs_string("IGN WP CLEAR CMD: Busy");

					if (_verbose) { warnx("IGN WP CLEAR CMD: Busy"); }
				}
			}

			break;
		}

	default: {
			/* other messages might should get caught by mavlink and others */
			break;
		}
	}
}

void
Mavlink::mavlink_missionlib_send_message(mavlink_message_t *msg)
{
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
	mavlink_send_uart_bytes(_channel, buf, len);
}



int
Mavlink::mavlink_missionlib_send_gcs_string(const char *string)
{
	const int len = MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN;
	mavlink_statustext_t statustext;
	statustext.severity = MAV_SEVERITY_INFO;

	int i = 0;

	while (i < len - 1) {
		statustext.text[i] = string[i];

		if (string[i] == '\0') {
			break;
		}

		i++;
	}

	if (i > 1) {
		/* Enforce null termination */
		statustext.text[i] = '\0';

		mavlink_msg_statustext_send(_channel, statustext.severity, statustext.text);
		return OK;

	} else {
		return 1;
	}
}

MavlinkOrbSubscription *Mavlink::add_orb_subscription(const orb_id_t topic)
{
	/* check if already subscribed to this topic */
	MavlinkOrbSubscription *sub;

	LL_FOREACH(_subscriptions, sub) {
		if (sub->get_topic() == topic) {
			/* already subscribed */
			return sub;
		}
	}

	/* add new subscription */
	MavlinkOrbSubscription *sub_new = new MavlinkOrbSubscription(topic);

	LL_APPEND(_subscriptions, sub_new);

	return sub_new;
}

int
Mavlink::configure_stream(const char *stream_name, const float rate)
{
	/* calculate interval in us, 0 means disabled stream */
	unsigned int interval = (rate > 0.0f) ? (1000000.0f / rate) : 0;

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
				warnx("deleted stream %s", stream->get_name());
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
			stream = streams_list[i]->new_instance();
			stream->set_channel(get_channel());
			stream->set_interval(interval);
			stream->subscribe(this);
			LL_APPEND(_streams, stream);

			return OK;
		}
	}

	/* if we reach here, the stream list does not contain the stream */
	warnx("stream %s not found", stream_name);

	return ERROR;
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
	}
}

int
Mavlink::message_buffer_init(int size)
{
	_message_buffer.size = size;
	_message_buffer.write_ptr = 0;
	_message_buffer.read_ptr = 0;
	_message_buffer.data = (char*)malloc(_message_buffer.size);
	return (_message_buffer.data == 0) ? ERROR : OK;
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
Mavlink::message_buffer_write(void *ptr, int size)
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
Mavlink::pass_message(mavlink_message_t *msg)
{
	if (_passing_on) {
		/* size is 8 bytes plus variable payload */
		int size = MAVLINK_NUM_NON_PAYLOAD_BYTES + msg->len;
		pthread_mutex_lock(&_message_buffer_mutex);
		message_buffer_write(msg, size);
		pthread_mutex_unlock(&_message_buffer_mutex);
	}
}



int
Mavlink::task_main(int argc, char *argv[])
{
	int ch;
	_baudrate = 57600;
	_datarate = 0;
	_mode = MAVLINK_MODE_NORMAL;

	/* work around some stupidity in task_create's argv handling */
	argc -= 2;
	argv += 2;

	/* don't exit from getopt loop to leave getopt global variables in consistent state,
	 * set error flag instead */
	bool err_flag = false;

	while ((ch = getopt(argc, argv, "b:r:d:m:fpvw")) != EOF) {
		switch (ch) {
		case 'b':
			_baudrate = strtoul(optarg, NULL, 10);

			if (_baudrate < 9600 || _baudrate > 921600) {
				warnx("invalid baud rate '%s'", optarg);
				err_flag = true;
			}

			break;

		case 'r':
			_datarate = strtoul(optarg, NULL, 10);

			if (_datarate < 10 || _datarate > MAX_DATA_RATE) {
				warnx("invalid data rate '%s'", optarg);
				err_flag = true;
			}

			break;

		case 'd':
			_device_name = optarg;
			break;

//		case 'e':
//			mavlink_link_termination_allowed = true;
//			break;

		case 'm':
			if (strcmp(optarg, "custom") == 0) {
				_mode = MAVLINK_MODE_CUSTOM;

			} else if (strcmp(optarg, "camera") == 0) {
				_mode = MAVLINK_MODE_CAMERA;
			}

			break;

		case 'f':
			_forwarding_on = true;
			break;

		case 'p':
			_passing_on = true;
			break;

		case 'v':
			_verbose = true;
			break;

		case 'w':
			_wait_to_transmit = true;
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

	if (Mavlink::instance_exists(_device_name, this)) {
		warnx("mavlink instance for %s already running", _device_name);
		return ERROR;
	}

	/* inform about mode */
	switch (_mode) {
	case MAVLINK_MODE_NORMAL:
		warnx("mode: NORMAL");
		break;

	case MAVLINK_MODE_CUSTOM:
		warnx("mode: CUSTOM");
		break;

	case MAVLINK_MODE_CAMERA:
		warnx("mode: CAMERA");
		break;

	default:
		warnx("ERROR: Unknown mode");
		break;
	}

	_mavlink_wpm_comp_id = MAV_COMP_ID_MISSIONPLANNER;

	warnx("data rate: %d Bytes/s, port: %s, baud: %d", _datarate, _device_name, _baudrate);

	/* flush stdout in case MAVLink is about to take it over */
	fflush(stdout);

	struct termios uart_config_original;

	/* default values for arguments */
	_uart_fd = mavlink_open_uart(_baudrate, _device_name, &uart_config_original, &_is_usb_uart);

	if (_uart_fd < 0) {
		warn("could not open %s", _device_name);
		return ERROR;
	}

	/* initialize mavlink text message buffering */
	mavlink_logbuffer_init(&_logbuffer, 5);

	/* if we are passing on mavlink messages, we need to prepare a buffer for this instance */
	if (_passing_on) {
		/* initialize message buffer if multiplexing is on */
		if (OK != message_buffer_init(300)) {
			errx(1, "can't allocate message buffer, exiting");
		}

		/* initialize message buffer mutex */
		pthread_mutex_init(&_message_buffer_mutex, NULL);
	}

	/* create the device node that's used for sending text log messages, etc. */
	register_driver(MAVLINK_LOG_DEVICE, &fops, 0666, NULL);

	/* initialize logging device */
	_mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

	/* Initialize system properties */
	mavlink_update_system();

	/* start the MAVLink receiver */
	_receive_thread = MavlinkReceiver::receive_start(this);

	/* initialize waypoint manager */
	mavlink_wpm_init(_wpm);

	int mission_result_sub = orb_subscribe(ORB_ID(mission_result));
	struct mission_result_s mission_result;
	memset(&mission_result, 0, sizeof(mission_result));

	_task_running = true;

	MavlinkOrbSubscription *param_sub = add_orb_subscription(ORB_ID(parameter_update));
	uint64_t param_time = 0;
	MavlinkOrbSubscription *status_sub = add_orb_subscription(ORB_ID(vehicle_status));
	uint64_t status_time = 0;

	struct vehicle_status_s status;
	status_sub->update(&status_time, &status);

	MavlinkCommandsStream commands_stream(this, _channel);

	/* add default streams depending on mode and intervals depending on datarate */
	float rate_mult = _datarate / 1000.0f;

	configure_stream("HEARTBEAT", 1.0f);

	switch (_mode) {
	case MAVLINK_MODE_NORMAL:
		configure_stream("SYS_STATUS", 1.0f);
		configure_stream("GPS_GLOBAL_ORIGIN", 0.5f);
		configure_stream("HIGHRES_IMU", 1.0f * rate_mult);
		configure_stream("ATTITUDE", 10.0f * rate_mult);
		configure_stream("VFR_HUD", 10.0f * rate_mult);
		configure_stream("GPS_RAW_INT", 1.0f * rate_mult);
		configure_stream("GLOBAL_POSITION_INT", 3.0f * rate_mult);
		configure_stream("LOCAL_POSITION_NED", 3.0f * rate_mult);
		configure_stream("RC_CHANNELS_RAW", 1.0f * rate_mult);
		configure_stream("NAMED_VALUE_FLOAT", 1.0f * rate_mult);
		configure_stream("GLOBAL_POSITION_SETPOINT_INT", 3.0f * rate_mult);
		configure_stream("ROLL_PITCH_YAW_THRUST_SETPOINT", 3.0f * rate_mult);
		configure_stream("DISTANCE_SENSOR", 0.5f);
		break;

	case MAVLINK_MODE_CAMERA:
		configure_stream("SYS_STATUS", 1.0f);
		configure_stream("ATTITUDE", 15.0f * rate_mult);
		configure_stream("GLOBAL_POSITION_INT", 15.0f * rate_mult);
		configure_stream("CAMERA_CAPTURE", 1.0f);
		break;

	default:
		break;
	}

	/* don't send parameters on startup without request */
	_mavlink_param_queue_index = param_count();

	MavlinkRateLimiter slow_rate_limiter(2000000.0f / rate_mult);
	MavlinkRateLimiter fast_rate_limiter(30000.0f / rate_mult);

	/* set main loop delay depending on data rate to minimize CPU overhead */
	_main_loop_delay = MAIN_LOOP_DELAY / rate_mult;

	/* now the instance is fully initialized and we can bump the instance count */
	LL_APPEND(_mavlink_instances, this);

	while (!_task_should_exit) {
		/* main loop */
		usleep(_main_loop_delay);

		perf_begin(_loop_perf);

		hrt_abstime t = hrt_absolute_time();

		if (param_sub->update(&param_time, nullptr)) {
			/* parameters updated */
			mavlink_update_system();
		}

		if (status_sub->update(&status_time, &status)) {
			/* switch HIL mode if required */
			set_hil_enabled(status.hil_state == HIL_STATE_ON);
		}

		/* update commands stream */
		commands_stream.update(t);

		/* check for requested subscriptions */
		if (_subscribe_to_stream != nullptr) {
			if (OK == configure_stream(_subscribe_to_stream, _subscribe_to_stream_rate)) {
				if (_subscribe_to_stream_rate > 0.0f) {
					warnx("stream %s on device %s enabled with rate %.1f Hz", _subscribe_to_stream, _device_name, (double)_subscribe_to_stream_rate);

				} else {
					warnx("stream %s on device %s disabled", _subscribe_to_stream, _device_name);
				}

			} else {
				warnx("stream %s on device %s not found", _subscribe_to_stream, _device_name);
			}

			delete _subscribe_to_stream;
			_subscribe_to_stream = nullptr;
		}

		/* update streams */
		MavlinkStream *stream;
		LL_FOREACH(_streams, stream) {
			stream->update(t);
		}

		bool updated;
		orb_check(mission_result_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(mission_result), mission_result_sub, &mission_result);

			if (_verbose) { warnx("Got mission result: new current: %d", mission_result.index_current_mission); }

			if (mission_result.mission_reached) {
				mavlink_wpm_send_waypoint_reached((uint16_t)mission_result.mission_index_reached);
			}

			mavlink_wpm_send_waypoint_current((uint16_t)mission_result.index_current_mission);

		} else {
			if (slow_rate_limiter.check(t)) {
				mavlink_wpm_send_waypoint_current((uint16_t)mission_result.index_current_mission);
			}
		}

		if (fast_rate_limiter.check(t)) {
			mavlink_pm_queued_send();
			mavlink_waypoint_eventloop(hrt_absolute_time());

			if (!mavlink_logbuffer_is_empty(&_logbuffer)) {
				struct mavlink_logmessage msg;
				int lb_ret = mavlink_logbuffer_read(&_logbuffer, &msg);

				if (lb_ret == OK) {
					mavlink_missionlib_send_gcs_string(msg.text);
				}
			}
		}

		/* pass messages from other UARTs */
		if (_passing_on) {

			bool is_part;
			void *read_ptr;

			/* guard get ptr by mutex */
			pthread_mutex_lock(&_message_buffer_mutex);
			int available = message_buffer_get_ptr(&read_ptr, &is_part);
			pthread_mutex_unlock(&_message_buffer_mutex);

			if (available > 0) {
				/* write first part of buffer */
				_mavlink_resend_uart(_channel, (const mavlink_message_t*)read_ptr);
				message_buffer_mark_read(available);

				/* write second part of buffer if there is some */
				if (is_part) {
					/* guard get ptr by mutex */
					pthread_mutex_lock(&_message_buffer_mutex);
					available = message_buffer_get_ptr(&read_ptr, &is_part);
					pthread_mutex_unlock(&_message_buffer_mutex);

					_mavlink_resend_uart(_channel, (const mavlink_message_t*)read_ptr);
					message_buffer_mark_read(available);
				}
			}
		}



		perf_end(_loop_perf);
	}

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

	warnx("waiting for UART receive thread");

	/* wait for threads to complete */
	pthread_join(_receive_thread, NULL);

	/* reset the UART flags to original state */
	tcsetattr(_uart_fd, TCSANOW, &uart_config_original);

	/* close UART */
	close(_uart_fd);

	/* close mavlink logging device */
	close(_mavlink_fd);

	if (_passing_on) {
		message_buffer_destroy();
		pthread_mutex_destroy(&_message_buffer_mutex);
	}
	/* destroy log buffer */
	mavlink_logbuffer_destroy(&_logbuffer);

	warnx("exiting");
	_task_running = false;

	return OK;
}

int Mavlink::start_helper(int argc, char *argv[])
{
	/* create the instance in task context */
	Mavlink *instance = new Mavlink();

	/* this will actually only return once MAVLink exits */
	int res = instance->task_main(argc, argv);

	/* delete instance on main thread end */
	delete instance;

	return res;
}

int
Mavlink::start(int argc, char *argv[])
{
	// Wait for the instance count to go up one
	// before returning to the shell
	int ic = Mavlink::instance_count();

	// Instantiate thread
	char buf[24];
	sprintf(buf, "mavlink_if%d", ic);

	// This is where the control flow splits
	// between the starting task and the spawned
	// task - start_helper() only returns
	// when the started task exits.
	task_spawn_cmd(buf,
		       SCHED_DEFAULT,
		       SCHED_PRIORITY_DEFAULT,
		       1950,
		       (main_t)&Mavlink::start_helper,
		       (const char **)argv);

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
Mavlink::status()
{
	warnx("running");
}

int
Mavlink::stream(int argc, char *argv[])
{
	const char *device_name = DEFAULT_DEVICE_NAME;
	float rate = -1.0f;
	const char *stream_name = nullptr;

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
			device_name = argv[i + 1];
			i++;

		} else if (0 == strcmp(argv[i], "-s") && i < argc - 1) {
			stream_name = argv[i + 1];
			i++;

		} else {
			err_flag = true;
		}

		i++;
	}

	if (!err_flag && rate >= 0.0f && stream_name != nullptr) {
		Mavlink *inst = get_instance_for_device(device_name);

		if (inst != nullptr) {
			inst->configure_stream_threadsafe(stream_name, rate);

		} else {

			// If the link is not running we should complain, but not fall over
			// because this is so easy to get wrong and not fatal. Warning is sufficient.
			errx(0, "mavlink for device %s is not running", device_name);
		}

	} else {
		errx(1, "usage: mavlink stream [-d device] -s stream -r rate");
	}

	return OK;
}

static void usage()
{
	warnx("usage: mavlink {start|stop-all|stream} [-d device] [-b baudrate] [-r rate] [-m mode] [-s stream] [-f] [-p] [-v] [-w]");
}

int mavlink_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		exit(1);
	}

	if (!strcmp(argv[1], "start")) {
		return Mavlink::start(argc, argv);

	} else if (!strcmp(argv[1], "stop")) {
		warnx("mavlink stop is deprecated, use stop-all instead");
		usage();
		exit(1);

	} else if (!strcmp(argv[1], "stop-all")) {
		return Mavlink::destroy_all_instances();

		// } else if (!strcmp(argv[1], "status")) {
		// 	mavlink::g_mavlink->status();

	} else if (!strcmp(argv[1], "stream")) {
		return Mavlink::stream(argc, argv);

	} else {
		usage();
		exit(1);
	}

	return 0;
}
