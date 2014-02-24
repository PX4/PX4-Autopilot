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
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>

#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <geo/geo.h>
#include <dataman/dataman.h>
#include <mathlib/mathlib.h>
#include <mavlink/mavlink_log.h>

#include <commander/px4_custom_mode.h>

#include "mavlink_bridge_header.h"
#include "mavlink_main.h"
#include "mavlink_orb_listener.h"
#include "mavlink_receiver.h"

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

static Mavlink* _head = nullptr;

/* TODO: if this is a class member it crashes */
static struct file_operations fops;

/**
 * mavlink app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mavlink_main(int argc, char *argv[]);

/*
 * Internal function to send the bytes through the right serial port
 */
void
mavlink_send_uart_bytes(mavlink_channel_t channel, const uint8_t *ch, int length)
{
	int uart = -1;

	switch (channel) {
		case MAVLINK_COMM_0:
			uart = Mavlink::get_uart_fd(0);
		break;
		case MAVLINK_COMM_1:
			uart = Mavlink::get_uart_fd(1);
		break;
		case MAVLINK_COMM_2:
			uart = Mavlink::get_uart_fd(2);
		break;
		case MAVLINK_COMM_3:
			uart = Mavlink::get_uart_fd(3);
		break;
		#ifdef MAVLINK_COMM_4
		case MAVLINK_COMM_4:
			uart = Mavlink::get_uart_fd(4);
		break;
		#endif
		#ifdef MAVLINK_COMM_5
		case MAVLINK_COMM_5:
			uart = Mavlink::get_uart_fd(5);
		break;
		#endif
		#ifdef MAVLINK_COMM_6
		case MAVLINK_COMM_6:
			uart = Mavlink::get_uart_fd(6);
		break;
		#endif
	}

	ssize_t desired = (sizeof(uint8_t) * length);
	ssize_t ret = write(uart, ch, desired);

	if (ret != desired)
		warn("write err");

}

static void usage(void);

namespace mavlink
{

	Mavlink	*g_mavlink;
}

Mavlink::Mavlink() :
	device_name("/dev/ttyS1"),
	_task_should_exit(false),
	_next(nullptr),
	_mavlink_fd(-1),
	thread_running(false),
	_mavlink_task(-1),
	_mavlink_incoming_fd(-1),

/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "mavlink")),
	_mavlink_hil_enabled(false),
	// _params_sub(-1)

	mission_pub(-1)
{
	wpm = &wpm_s;
	fops.ioctl = (int (*)(file*, int, long unsigned int))&mavlink_dev_ioctl;
	// _parameter_handles.min_altitude = param_find("NAV_MIN_ALT");

}

Mavlink::~Mavlink()
{
	if (_mavlink_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				task_delete(_mavlink_task);
				break;
			}
		} while (_mavlink_task != -1);
	}
}

void Mavlink::set_mode(enum MAVLINK_MODE mode)
{
	_mode = mode;
}

int Mavlink::instance_count()
{
	/* note: a local buffer count will help if this ever is called often */
	Mavlink* inst = ::_head;
	unsigned inst_index = 0;
	while (inst != nullptr) {
		inst = inst->_next;
		inst_index++;
	}

	return inst_index;
}

Mavlink* Mavlink::new_instance()
{
	Mavlink* inst = new Mavlink();
	Mavlink* next = ::_head;

	/* create the first instance at _head */
	if (::_head == nullptr) {
		::_head = inst;
	/* afterwards follow the next and append the instance */
	} else {
		while (next->_next != nullptr) {
			next = next->_next;
		}
		/* now parent has a null pointer, fill it */
		next->_next = inst;
	}
	return inst;
}

Mavlink* Mavlink::get_instance(unsigned instance)
{
	Mavlink* inst = ::_head;
	unsigned inst_index = 0;
	while (inst->_next != nullptr && inst_index < instance) {
		inst = inst->_next;
		inst_index++;
	}

	if (inst_index < instance) {
		inst = nullptr;
	}

	return inst;
}

int Mavlink::destroy_all_instances()
{
	/* start deleting from the end */
	Mavlink *inst_to_del = nullptr;
	Mavlink *next_inst = ::_head;

	unsigned iterations = 0;

	warnx("waiting for instances to stop");
	while (next_inst != nullptr) {

		inst_to_del = next_inst;
		next_inst = inst_to_del->_next;

		/* set flag to stop thread and wait for all threads to finish */
		inst_to_del->_task_should_exit = true;
		while (inst_to_del->thread_running) {
			printf(".");
			usleep(10000);
			iterations++;

			if (iterations > 10000) {
				warnx("ERROR: Couldn't stop all mavlink instances.");
				return ERROR;
			}
		}
		delete inst_to_del;
	}

	/* reset head */
	::_head = nullptr;

	printf("\n");
	warnx("all instances stopped");
	return OK;
}

bool Mavlink::instance_exists(const char *device_name, Mavlink *self)
{
	Mavlink* inst = ::_head;
	while (inst != nullptr) {

		/* don't compare with itself */
		if (inst != self && !strcmp(device_name, inst->device_name))
			return true;
		inst = inst->_next;
	}
	return false;
}

int Mavlink::get_uart_fd(unsigned index)
{
	Mavlink* inst = get_instance(index);
	if (inst)
		return inst->get_uart_fd();

	return -1;
}

int Mavlink::get_uart_fd()
{
	return _uart;
}

int Mavlink::get_channel()
{
	return (int)_chan;
}

void
Mavlink::parameters_update()
{
	/* read from param to clear updated flag */
	struct parameter_update_s update;
	orb_copy(ORB_ID(parameter_update), _params_sub, &update);

	// param_get(_parameter_handles.min_altitude, &(_parameters.min_altitude));

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

		Mavlink* inst = ::_head;
		while (inst != nullptr) {

			mavlink_logbuffer_write(&inst->lb, &msg);
			inst->total_counter++;
			inst = inst->_next;

		}
		return OK;
	}

	default:
		return ENOTTY;
	}
}

void Mavlink::mavlink_update_system(void)
{
	static bool initialized = false;
	static param_t param_system_id;
	static param_t param_component_id;
	static param_t param_system_type;

	if (!initialized) {
		param_system_id = param_find("MAV_SYS_ID");
		param_component_id = param_find("MAV_COMP_ID");
		param_system_type = param_find("MAV_TYPE");
		initialized = true;
	}

	/* update system and component id */
	int32_t system_id;
	param_get(param_system_id, &system_id);

	if (system_id > 0 && system_id < 255) {
		mavlink_system.sysid = system_id;
	}

	int32_t component_id;
	param_get(param_component_id, &component_id);

	if (component_id > 0 && component_id < 255) {
		mavlink_system.compid = component_id;
	}

	int32_t system_type;
	param_get(param_system_type, &system_type);

	if (system_type >= 0 && system_type < MAV_TYPE_ENUM_END) {
		mavlink_system.type = system_type;
	}
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
	warnx("UART is %s, baudrate is %d\n", uart_name, baud);
	_uart = open(uart_name, O_RDWR | O_NOCTTY);

	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;
	*is_usb = false;

	/* Back up the original uart configuration to restore it after exit */
	if ((termios_state = tcgetattr(_uart, uart_config_original)) < 0) {
		warnx("ERROR get termios config %s: %d\n", uart_name, termios_state);
		close(_uart);
		return -1;
	}

	/* Fill the struct for the new configuration */
	tcgetattr(_uart, &uart_config);

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* USB serial is indicated by /dev/ttyACM0*/
	if (strcmp(uart_name, "/dev/ttyACM0") != OK && strcmp(uart_name, "/dev/ttyACM1") != OK) {

		/* Set baud rate */
		if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
			warnx("ERROR setting baudrate / termios config for %s: %d (cfsetispeed, cfsetospeed)\n", uart_name, termios_state);
			close(_uart);
			return -1;
		}

	}

	if ((termios_state = tcsetattr(_uart, TCSANOW, &uart_config)) < 0) {
		warnx("ERROR setting baudrate / termios config for %s (tcsetattr)\n", uart_name);
		close(_uart);
		return -1;
	}

	return _uart;
}

int
Mavlink::set_hil_enabled(bool hil_enabled)
{
	int ret = OK;

	/* Enable HIL */
	if (hil_enabled && !_mavlink_hil_enabled) {

		_mavlink_hil_enabled = true;

		/* ramp up some HIL-related subscriptions */
		unsigned hil_rate_interval;

		if (_baudrate < 19200) {
			/* 10 Hz */
			hil_rate_interval = 100;

		} else if (_baudrate < 38400) {
			/* 10 Hz */
			hil_rate_interval = 100;

		} else if (_baudrate < 115200) {
			/* 20 Hz */
			hil_rate_interval = 50;

		} else {
			/* 200 Hz */
			hil_rate_interval = 5;
		}

		orb_set_interval(subs.spa_sub, hil_rate_interval);
		set_mavlink_interval_limit(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, hil_rate_interval);
	}

	if (!hil_enabled && _mavlink_hil_enabled) {
		_mavlink_hil_enabled = false;
		orb_set_interval(subs.spa_sub, 200);

	} else {
		ret = ERROR;
	}

	return ret;
}

void
Mavlink::get_mavlink_mode_and_state(uint8_t *mavlink_state, uint8_t *mavlink_base_mode, uint32_t *mavlink_custom_mode)
{
	/* reset MAVLink mode bitfield */
	*mavlink_base_mode = 0;
	*mavlink_custom_mode = 0;

	/**
	 * Set mode flags
	 **/

	/* HIL */
	if (v_status.hil_state == HIL_STATE_ON) {
		*mavlink_base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
	}

	/* arming state */
	if (v_status.arming_state == ARMING_STATE_ARMED
			|| v_status.arming_state == ARMING_STATE_ARMED_ERROR) {
		*mavlink_base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
	}

	/* main state */
	*mavlink_base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
	union px4_custom_mode custom_mode;
	custom_mode.data = 0;
	if (pos_sp_triplet.nav_state == NAV_STATE_NONE) {
	/* use main state when navigator is not active */
		if (v_status.main_state == MAIN_STATE_MANUAL) {
			*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | (v_status.is_rotary_wing ? MAV_MODE_FLAG_STABILIZE_ENABLED : 0);
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_MANUAL;
		} else if (v_status.main_state == MAIN_STATE_SEATBELT) {
			*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED;
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_SEATBELT;
		} else if (v_status.main_state == MAIN_STATE_EASY) {
			*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED;
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_EASY;
		} else if (v_status.main_state == MAIN_STATE_AUTO) {
			*mavlink_base_mode |= MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED;
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
			custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_READY;
		}
	} else {
		/* use navigation state when navigator is active */
		*mavlink_base_mode |= MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED;
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		if (pos_sp_triplet.nav_state == NAV_STATE_READY) {
			custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_READY;
		} else if (pos_sp_triplet.nav_state == NAV_STATE_LOITER) {
			custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
		} else if (pos_sp_triplet.nav_state == NAV_STATE_MISSION) {
			custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_MISSION;
		} else if (pos_sp_triplet.nav_state == NAV_STATE_RTL) {
			custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_RTL;
		} else if (pos_sp_triplet.nav_state == NAV_STATE_LAND) {
			custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LAND;
		}
	}
	*mavlink_custom_mode = custom_mode.data;

	/**
	 * Set mavlink state
	 **/

	/* set calibration state */
	if (v_status.arming_state == ARMING_STATE_INIT
			|| v_status.arming_state == ARMING_STATE_IN_AIR_RESTORE
			|| v_status.arming_state == ARMING_STATE_STANDBY_ERROR) {	// TODO review
		*mavlink_state = MAV_STATE_UNINIT;
	} else if (v_status.arming_state == ARMING_STATE_ARMED) {
		*mavlink_state = MAV_STATE_ACTIVE;
	} else if (v_status.arming_state == ARMING_STATE_ARMED_ERROR) {
		*mavlink_state = MAV_STATE_CRITICAL;
	} else if (v_status.arming_state == ARMING_STATE_STANDBY) {
		*mavlink_state = MAV_STATE_STANDBY;
	} else if (v_status.arming_state == ARMING_STATE_REBOOT) {
		*mavlink_state = MAV_STATE_POWEROFF;
	} else {
		warnx("Unknown mavlink state");
		*mavlink_state = MAV_STATE_CRITICAL;
	}
}


int Mavlink::set_mavlink_interval_limit(int mavlink_msg_id, int min_interval)
{
	int ret = OK;

	switch (mavlink_msg_id) {
	case MAVLINK_MSG_ID_SCALED_IMU:
		/* sensor sub triggers scaled IMU */
		orb_set_interval(subs.sensor_sub, min_interval);
		break;

	case MAVLINK_MSG_ID_HIGHRES_IMU:
		/* sensor sub triggers highres IMU */
		orb_set_interval(subs.sensor_sub, min_interval);
		break;

	case MAVLINK_MSG_ID_RAW_IMU:
		/* sensor sub triggers RAW IMU */
		orb_set_interval(subs.sensor_sub, min_interval);
		break;

	case MAVLINK_MSG_ID_ATTITUDE:
		/* attitude sub triggers attitude */
		orb_set_interval(subs.att_sub, min_interval);
		break;

	case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
		/* actuator_outputs triggers this message */
		orb_set_interval(subs.act_0_sub, min_interval);
		orb_set_interval(subs.act_1_sub, min_interval);
		orb_set_interval(subs.act_2_sub, min_interval);
		orb_set_interval(subs.act_3_sub, min_interval);
		orb_set_interval(subs.actuators_sub, min_interval);
		orb_set_interval(subs.actuators_effective_sub, min_interval);
		orb_set_interval(subs.spa_sub, min_interval);
		orb_set_interval(subs.rates_setpoint_sub, min_interval);
		break;

	case MAVLINK_MSG_ID_MANUAL_CONTROL:
		/* manual_control_setpoint triggers this message */
		orb_set_interval(subs.man_control_sp_sub, min_interval);
		break;

	case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
		orb_set_interval(subs.debug_key_value, min_interval);
		break;

	default:
		/* not found */
		ret = ERROR;
		break;
	}

	return ret;
}

extern mavlink_system_t mavlink_system;

void Mavlink::mavlink_pm_callback(void *arg, param_t param)
{
	//mavlink_pm_send_param(param);
	usleep(*(unsigned int *)arg);
}

void Mavlink::mavlink_pm_send_all_params(unsigned int delay)
{
	unsigned int dbuf = delay;
	param_foreach(&mavlink_pm_callback, &dbuf, false);
}

int Mavlink::mavlink_pm_queued_send()
{
	if (mavlink_param_queue_index < param_count()) {
		mavlink_pm_send_param(param_for_index(mavlink_param_queue_index));
		mavlink_param_queue_index++;
		return 0;

	} else {
		return 1;
	}
}

void Mavlink::mavlink_pm_start_queued_send()
{
	mavlink_param_queue_index = 0;
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
	if (param == PARAM_INVALID) return 1;

	/* buffers for param transmission */
	static char name_buf[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN];
	float val_buf;
	static mavlink_message_t tx_msg;

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
					  MAVLINK_COMM_0,
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
			/* Start sending parameters */
			mavlink_pm_start_queued_send();
			mavlink_missionlib_send_gcs_string("[mavlink pm] sending list");
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
						sprintf(buf, "[mavlink pm] unknown: %s", name);
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
	if (mission_pub < 0) {
		mission_pub = orb_advertise(ORB_ID(mission), &mission);

	} else {
		orb_publish(ORB_ID(mission), mission_pub, &mission);
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
			mission_item->pitch_min = mavlink_mission_item->param2;
			break;
		default:
			mission_item->acceptance_radius = mavlink_mission_item->param2;
			break;
	}

	mission_item->yaw = _wrap_pi(mavlink_mission_item->param4*M_DEG_TO_RAD_F);
	mission_item->loiter_radius = fabsf(mavlink_mission_item->param3);
	mission_item->loiter_direction = (mavlink_mission_item->param3 > 0) ? 1 : -1; /* 1 if positive CW, -1 if negative CCW */
	mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;

	mission_item->time_inside = mavlink_mission_item->param1;
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
			mavlink_mission_item->param2 = mission_item->pitch_min;
			break;
		default:
			mavlink_mission_item->param2 = mission_item->acceptance_radius;
			break;
	}

	mavlink_mission_item->x = (float)mission_item->lat;
	mavlink_mission_item->y = (float)mission_item->lon;
	mavlink_mission_item->z = mission_item->altitude;

	mavlink_mission_item->param4 = mission_item->yaw*M_RAD_TO_DEG_F;
	mavlink_mission_item->param3 = mission_item->loiter_radius*(float)mission_item->loiter_direction;
	mavlink_mission_item->command = mission_item->nav_cmd;
	mavlink_mission_item->param1 = mission_item->time_inside;
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

	mavlink_msg_mission_ack_encode(mavlink_system.sysid, _mavlink_wpm_comp_id, &msg, &wpa);
	mavlink_missionlib_send_message(&msg);

	if (_verbose) warnx("Sent waypoint ack (%u) to ID %u", wpa.type, wpa.target_system);
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
	if (seq < wpm->size) {
		mavlink_message_t msg;
		mavlink_mission_current_t wpc;

		wpc.seq = seq;

		mavlink_msg_mission_current_encode(mavlink_system.sysid, _mavlink_wpm_comp_id, &msg, &wpc);
		mavlink_missionlib_send_message(&msg);

	} else if (seq == 0 && wpm->size == 0) {

		/* don't broadcast if no WPs */

	} else {
		mavlink_missionlib_send_gcs_string("ERROR: wp index out of bounds");
		if (_verbose) warnx("ERROR: index out of bounds");
	}
}

void Mavlink::mavlink_wpm_send_waypoint_count(uint8_t sysid, uint8_t compid, uint16_t count)
{
	mavlink_message_t msg;
	mavlink_mission_count_t wpc;

	wpc.target_system = sysid;
	wpc.target_component = compid;
	wpc.count = mission.count;

	mavlink_msg_mission_count_encode(mavlink_system.sysid, _mavlink_wpm_comp_id, &msg, &wpc);
	mavlink_missionlib_send_message(&msg);

	if (_verbose) warnx("Sent waypoint count (%u) to ID %u", wpc.count, wpc.target_system);
}

void Mavlink::mavlink_wpm_send_waypoint(uint8_t sysid, uint8_t compid, uint16_t seq)
{

	struct mission_item_s mission_item;
	ssize_t len = sizeof(struct mission_item_s);
	
	dm_item_t dm_current;

	if (wpm->current_dataman_id == 0) {
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
		mavlink_msg_mission_item_encode(mavlink_system.sysid, _mavlink_wpm_comp_id, &msg, &wp);
		mavlink_missionlib_send_message(&msg);

		if (_verbose) warnx("Sent waypoint %u to ID %u", wp.seq, wp.target_system);
	} else {
		mavlink_wpm_send_waypoint_ack(wpm->current_partner_sysid, wpm->current_partner_compid, MAV_MISSION_ERROR);
		if (_verbose) warnx("ERROR: could not read WP%u", seq);
	}
}

void Mavlink::mavlink_wpm_send_waypoint_request(uint8_t sysid, uint8_t compid, uint16_t seq)
{
	if (seq < wpm->max_size) {
		mavlink_message_t msg;
		mavlink_mission_request_t wpr;
		wpr.target_system = sysid;
		wpr.target_component = compid;
		wpr.seq = seq;
		mavlink_msg_mission_request_encode(mavlink_system.sysid, _mavlink_wpm_comp_id, &msg, &wpr);
		mavlink_missionlib_send_message(&msg);

		if (_verbose) warnx("Sent waypoint request %u to ID %u", wpr.seq, wpr.target_system);

	} else {
		mavlink_missionlib_send_gcs_string("ERROR: Waypoint index exceeds list capacity");
		if (_verbose) warnx("ERROR: Waypoint index exceeds list capacity");
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

	mavlink_msg_mission_item_reached_encode(mavlink_system.sysid, _mavlink_wpm_comp_id, &msg, &wp_reached);
	mavlink_missionlib_send_message(&msg);

	if (_verbose) warnx("Sent waypoint %u reached message", wp_reached.seq);
}

void Mavlink::mavlink_waypoint_eventloop(uint64_t now)
{
	/* check for timed-out operations */
	if (now - wpm->timestamp_lastaction > wpm->timeout && wpm->current_state != MAVLINK_WPM_STATE_IDLE) {

		mavlink_missionlib_send_gcs_string("Operation timeout");

		if (_verbose) warnx("Last operation (state=%u) timed out, changing state to MAVLINK_WPM_STATE_IDLE", wpm->current_state);

		wpm->current_state = MAVLINK_WPM_STATE_IDLE;
		wpm->current_partner_sysid = 0;
		wpm->current_partner_compid = 0;
	}
}


void Mavlink::mavlink_wpm_message_handler(const mavlink_message_t *msg)
{
	uint64_t now = hrt_absolute_time();

	switch (msg->msgid) {

		case MAVLINK_MSG_ID_MISSION_ACK: {
			mavlink_mission_ack_t wpa;
			mavlink_msg_mission_ack_decode(msg, &wpa);

			if ((msg->sysid == wpm->current_partner_sysid && msg->compid == wpm->current_partner_compid) && (wpa.target_system == mavlink_system.sysid /*&& wpa.target_component == mavlink_wpm_comp_id*/)) {
				wpm->timestamp_lastaction = now;

				if (wpm->current_state == MAVLINK_WPM_STATE_SENDLIST || wpm->current_state == MAVLINK_WPM_STATE_SENDLIST_SENDWPS) {
					if (wpm->current_wp_id == wpm->size - 1) {

						wpm->current_state = MAVLINK_WPM_STATE_IDLE;
						wpm->current_wp_id = 0;
					}
				}

			} else {
				mavlink_missionlib_send_gcs_string("REJ. WP CMD: curr partner id mismatch");
				if (_verbose) warnx("REJ. WP CMD: curr partner id mismatch");
			}

			break;
		}

		case MAVLINK_MSG_ID_MISSION_SET_CURRENT: {
			mavlink_mission_set_current_t wpc;
			mavlink_msg_mission_set_current_decode(msg, &wpc);

			if (wpc.target_system == mavlink_system.sysid /*&& wpc.target_component == mavlink_wpm_comp_id*/) {
				wpm->timestamp_lastaction = now;

				if (wpm->current_state == MAVLINK_WPM_STATE_IDLE) {
					if (wpc.seq < wpm->size) {

						mission.current_index = wpc.seq;
						publish_mission();
						
						/* don't answer yet, wait for the navigator to respond, then publish the mission_result */
//						mavlink_wpm_send_waypoint_current(wpc.seq);

					} else {
						mavlink_missionlib_send_gcs_string("IGN WP CURR CMD: Not in list");
						if (_verbose) warnx("IGN WP CURR CMD: Not in list");
					}

				} else {
					mavlink_missionlib_send_gcs_string("IGN WP CURR CMD: Busy");
					if (_verbose) warnx("IGN WP CURR CMD: Busy");

				}

			} else {
				mavlink_missionlib_send_gcs_string("REJ. WP CMD: target id mismatch");
				if (_verbose) warnx("REJ. WP CMD: target id mismatch");
			}

			break;
		}

		case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
			mavlink_mission_request_list_t wprl;
			mavlink_msg_mission_request_list_decode(msg, &wprl);

			if (wprl.target_system == mavlink_system.sysid /*&& wprl.target_component == mavlink_wpm_comp_id*/) {
				wpm->timestamp_lastaction = now;

				if (wpm->current_state == MAVLINK_WPM_STATE_IDLE || wpm->current_state == MAVLINK_WPM_STATE_SENDLIST) {
					if (wpm->size > 0) {
						
						wpm->current_state = MAVLINK_WPM_STATE_SENDLIST;
						wpm->current_wp_id = 0;
						wpm->current_partner_sysid = msg->sysid;
						wpm->current_partner_compid = msg->compid;

					} else {
						if (_verbose) warnx("No waypoints send");
					}

					wpm->current_count = wpm->size;
					mavlink_wpm_send_waypoint_count(msg->sysid, msg->compid, wpm->current_count);

				} else {
					mavlink_missionlib_send_gcs_string("IGN REQUEST LIST: Busy");
					if (_verbose) warnx("IGN REQUEST LIST: Busy");
				}
			} else {
				mavlink_missionlib_send_gcs_string("REJ. REQUEST LIST: target id mismatch");
				if (_verbose) warnx("REJ. REQUEST LIST: target id mismatch");
			}

			break;
		}

		case MAVLINK_MSG_ID_MISSION_REQUEST: {
			mavlink_mission_request_t wpr;
			mavlink_msg_mission_request_decode(msg, &wpr);

			if (msg->sysid == wpm->current_partner_sysid && msg->compid == wpm->current_partner_compid && wpr.target_system == mavlink_system.sysid /*&& wpr.target_component == mavlink_wpm_comp_id*/) {
				wpm->timestamp_lastaction = now;

				if (wpr.seq >= wpm->size) {

					mavlink_missionlib_send_gcs_string("REJ. WP CMD: Req. WP not in list");
					if (_verbose) warnx("Ignored MAVLINK_MSG_ID_MISSION_ITEM_REQUEST because the requested waypoint ID (%u) was out of bounds.", wpr.seq);
					break;
				}

				/* 
				 * Ensure that we are in the correct state and that the first request has id 0 
				 * and the following requests have either the last id (re-send last waypoint) or last_id+1 (next waypoint)
				 */
				if (wpm->current_state == MAVLINK_WPM_STATE_SENDLIST) {

					if (wpr.seq == 0) {
						if (_verbose) warnx("Got MAVLINK_MSG_ID_MISSION_ITEM_REQUEST of waypoint %u from %u changing state to MAVLINK_WPM_STATE_SENDLIST_SENDWPS", wpr.seq, msg->sysid);
						wpm->current_state = MAVLINK_WPM_STATE_SENDLIST_SENDWPS;
					} else {
						mavlink_missionlib_send_gcs_string("REJ. WP CMD: First id != 0");
						if (_verbose) warnx("REJ. WP CMD: First id != 0");
						break;
					}

				} else if (wpm->current_state == MAVLINK_WPM_STATE_SENDLIST_SENDWPS) {

					if (wpr.seq == wpm->current_wp_id) {

						if (_verbose) warnx("Got MAVLINK_MSG_ID_MISSION_ITEM_REQUEST of waypoint %u (again) from %u staying in state MAVLINK_WPM_STATE_SENDLIST_SENDWPS", wpr.seq, msg->sysid);

					} else if (wpr.seq == wpm->current_wp_id + 1) {

						if (_verbose) warnx("Got MAVLINK_MSG_ID_MISSION_ITEM_REQUEST of waypoint %u from %u staying in state MAVLINK_WPM_STATE_SENDLIST_SENDWPS", wpr.seq, msg->sysid);
					
					} else {
						mavlink_missionlib_send_gcs_string("REJ. WP CMD: Req. WP was unexpected");
						if (_verbose) warnx("Ignored MAVLINK_MSG_ID_MISSION_ITEM_REQUEST because the requested waypoint ID (%u) was not the expected (%u or %u).", wpr.seq, wpm->current_wp_id, wpm->current_wp_id + 1);
						break;
					}

				} else {

					mavlink_missionlib_send_gcs_string("REJ. WP CMD: Busy");
					if (_verbose) warnx("Ignored MAVLINK_MSG_ID_MISSION_ITEM_REQUEST because i'm doing something else already (state=%i).", wpm->current_state);
					break;
				}

				wpm->current_wp_id = wpr.seq;
				wpm->current_state = MAVLINK_WPM_STATE_SENDLIST_SENDWPS;

				if (wpr.seq < wpm->size) {

					mavlink_wpm_send_waypoint(wpm->current_partner_sysid, wpm->current_partner_compid,wpm->current_wp_id);

				} else {
					mavlink_wpm_send_waypoint_ack(wpm->current_partner_sysid, wpm->current_partner_compid, MAV_MISSION_ERROR);
					if (_verbose) warnx("ERROR: Waypoint %u out of bounds", wpr.seq);
				}


			} else {
				//we we're target but already communicating with someone else
				if ((wpr.target_system == mavlink_system.sysid /*&& wpr.target_component == mavlink_wpm_comp_id*/) && !(msg->sysid == wpm->current_partner_sysid && msg->compid == wpm->current_partner_compid)) {

					mavlink_missionlib_send_gcs_string("REJ. WP CMD: Busy");
					if (_verbose) warnx("Ignored MAVLINK_MSG_ID_MISSION_ITEM_REQUEST from ID %u because i'm already talking to ID %u.", msg->sysid, wpm->current_partner_sysid);

				} else {

					mavlink_missionlib_send_gcs_string("REJ. WP CMD: target id mismatch");
					if (_verbose) warnx("IGNORED WAYPOINT COMMAND BECAUSE TARGET SYSTEM AND COMPONENT OR COMM PARTNER ID MISMATCH");
				}
			}
			break;
		}

		case MAVLINK_MSG_ID_MISSION_COUNT: {
			mavlink_mission_count_t wpc;
			mavlink_msg_mission_count_decode(msg, &wpc);

			if (wpc.target_system == mavlink_system.sysid/* && wpc.target_component == mavlink_wpm_comp_id*/) {
				wpm->timestamp_lastaction = now;

				if (wpm->current_state == MAVLINK_WPM_STATE_IDLE) {

					if (wpc.count > NUM_MISSIONS_SUPPORTED) {
						if (_verbose) warnx("Too many waypoints: %d, supported: %d", wpc.count, NUM_MISSIONS_SUPPORTED);
						mavlink_wpm_send_waypoint_ack(wpm->current_partner_sysid, wpm->current_partner_compid, MAV_MISSION_NO_SPACE);
						break;
					}

					if (wpc.count == 0) {
						mavlink_missionlib_send_gcs_string("COUNT 0");
						if (_verbose) warnx("got waypoint count of 0, clearing waypoint list and staying in state MAVLINK_WPM_STATE_IDLE");
						break;
					}
					
					if (_verbose) warnx("Got MAVLINK_MSG_ID_MISSION_ITEM_COUNT (%u) from %u changing state to MAVLINK_WPM_STATE_GETLIST", wpc.count, msg->sysid);

					wpm->current_state = MAVLINK_WPM_STATE_GETLIST;
					wpm->current_wp_id = 0;
					wpm->current_partner_sysid = msg->sysid;
					wpm->current_partner_compid = msg->compid;
					wpm->current_count = wpc.count;

					mavlink_wpm_send_waypoint_request(wpm->current_partner_sysid, wpm->current_partner_compid, wpm->current_wp_id);

				} else if (wpm->current_state == MAVLINK_WPM_STATE_GETLIST) {

					if (wpm->current_wp_id == 0) {
						mavlink_missionlib_send_gcs_string("WP CMD OK AGAIN");
						if (_verbose) warnx("Got MAVLINK_MSG_ID_MISSION_ITEM_COUNT (%u) again from %u", wpc.count, msg->sysid);
					} else {
						mavlink_missionlib_send_gcs_string("REJ. WP CMD: Busy");
						if (_verbose) warnx("Ignored MAVLINK_MSG_ID_MISSION_ITEM_COUNT because i'm already receiving waypoint %u.", wpm->current_wp_id);
					}
				} else {
						mavlink_missionlib_send_gcs_string("IGN MISSION_COUNT CMD: Busy");
						if (_verbose) warnx("IGN MISSION_COUNT CMD: Busy");
				}
			} else {

				mavlink_missionlib_send_gcs_string("REJ. WP COUNT CMD: target id mismatch");
				if (_verbose) warnx("IGNORED WAYPOINT COUNT COMMAND BECAUSE TARGET SYSTEM AND COMPONENT OR COMM PARTNER ID MISMATCH");
			}
		}
		break;

		case MAVLINK_MSG_ID_MISSION_ITEM: {
			mavlink_mission_item_t wp;
			mavlink_msg_mission_item_decode(msg, &wp);

			if (wp.target_system == mavlink_system.sysid && wp.target_component == _mavlink_wpm_comp_id) {

				wpm->timestamp_lastaction = now;

				/*
				 * ensure that we are in the correct state and that the first waypoint has id 0
				 * and the following waypoints have the correct ids
				 */

				if (wpm->current_state == MAVLINK_WPM_STATE_GETLIST) {

				 	if (wp.seq != 0) {
				 		mavlink_missionlib_send_gcs_string("Ignored MISSION_ITEM WP not 0");
				 		warnx("Ignored MAVLINK_MSG_ID_MISSION_ITEM because the first waypoint ID (%u) was not 0.", wp.seq);
				 		break;
				 	}
				} else if (wpm->current_state == MAVLINK_WPM_STATE_GETLIST_GETWPS) {

					if (wp.seq >= wpm->current_count) {
						mavlink_missionlib_send_gcs_string("Ignored MISSION_ITEM WP out of bounds");
						warnx("Ignored MAVLINK_MSG_ID_MISSION_ITEM because the waypoint ID (%u) was out of bounds.", wp.seq);
						break;
					}

					if (wp.seq != wpm->current_wp_id) {
						warnx("Ignored MAVLINK_MSG_ID_MISSION_ITEM because the waypoint ID (%u) was not the expected %u.", wp.seq, wpm->current_wp_id);
						mavlink_wpm_send_waypoint_request(wpm->current_partner_sysid, wpm->current_partner_compid, wpm->current_wp_id);
						break;
					}
				}

				wpm->current_state = MAVLINK_WPM_STATE_GETLIST_GETWPS;

				struct mission_item_s mission_item;

				int ret = map_mavlink_mission_item_to_mission_item(&wp, &mission_item);

				if (ret != OK) {
					mavlink_wpm_send_waypoint_ack(wpm->current_partner_sysid, wpm->current_partner_compid, ret);
					wpm->current_state = MAVLINK_WPM_STATE_IDLE;
					break;
				}

				ssize_t len = sizeof(struct mission_item_s);

				dm_item_t dm_next;

				if (wpm->current_dataman_id == 0) {
					dm_next = DM_KEY_WAYPOINTS_OFFBOARD_1;
					mission.dataman_id = 1;
				} else {
					dm_next = DM_KEY_WAYPOINTS_OFFBOARD_0;
					mission.dataman_id = 0;
				}

				if (dm_write(dm_next, wp.seq, DM_PERSIST_IN_FLIGHT_RESET, &mission_item, len) != len) {
					mavlink_wpm_send_waypoint_ack(wpm->current_partner_sysid, wpm->current_partner_compid, MAV_MISSION_ERROR);
					wpm->current_state = MAVLINK_WPM_STATE_IDLE;
					break;
				}

//				if (wp.current) {
//					warnx("current is: %d", wp.seq);
//					mission.current_index = wp.seq;
//				}
				// XXX ignore current set
				mission.current_index = -1;

				wpm->current_wp_id = wp.seq + 1;

				if (wpm->current_wp_id == wpm->current_count && wpm->current_state == MAVLINK_WPM_STATE_GETLIST_GETWPS) {
					
					if (_verbose) warnx("Got all %u waypoints, changing state to MAVLINK_WPM_STATE_IDLE", wpm->current_count);

					mavlink_wpm_send_waypoint_ack(wpm->current_partner_sysid, wpm->current_partner_compid, MAV_MISSION_ACCEPTED);

					mission.count = wpm->current_count;
					
					publish_mission();

					wpm->current_dataman_id = mission.dataman_id;
					wpm->size = wpm->current_count;

					wpm->current_state = MAVLINK_WPM_STATE_IDLE;

				} else {
					mavlink_wpm_send_waypoint_request(wpm->current_partner_sysid, wpm->current_partner_compid, wpm->current_wp_id);
				}

			} else {
				mavlink_missionlib_send_gcs_string("REJ. WP CMD: target id mismatch");
				if (_verbose) warnx("IGNORED WAYPOINT COMMAND BECAUSE TARGET SYSTEM AND COMPONENT OR COMM PARTNER ID MISMATCH");
			}

			break;
		}

		case MAVLINK_MSG_ID_MISSION_CLEAR_ALL: {
			mavlink_mission_clear_all_t wpca;
			mavlink_msg_mission_clear_all_decode(msg, &wpca);

			if (wpca.target_system == mavlink_system.sysid /*&& wpca.target_component == mavlink_wpm_comp_id */) {

				if (wpm->current_state == MAVLINK_WPM_STATE_IDLE) {
					wpm->timestamp_lastaction = now;

					wpm->size = 0;

					/* prepare mission topic */
					mission.dataman_id = -1;
					mission.count = 0;
					mission.current_index = -1;
					publish_mission();

					if (dm_clear(DM_KEY_WAYPOINTS_OFFBOARD_0) == OK && dm_clear(DM_KEY_WAYPOINTS_OFFBOARD_1) == OK) {
						mavlink_wpm_send_waypoint_ack(wpm->current_partner_sysid, wpm->current_partner_compid, MAV_MISSION_ACCEPTED);
					} else {
						mavlink_wpm_send_waypoint_ack(wpm->current_partner_sysid, wpm->current_partner_compid, MAV_MISSION_ERROR);
					}

					
				} else {
					mavlink_missionlib_send_gcs_string("IGN WP CLEAR CMD: Busy");
					if (_verbose) warnx("IGN WP CLEAR CMD: Busy");
				}


			} else if (wpca.target_system == mavlink_system.sysid /*&& wpca.target_component == mavlink_wpm_comp_id */ && wpm->current_state != MAVLINK_WPM_STATE_IDLE) {

				mavlink_missionlib_send_gcs_string("REJ. WP CLERR CMD: target id mismatch");
				if (_verbose) warnx("IGNORED WAYPOINT CLEAR COMMAND BECAUSE TARGET SYSTEM AND COMPONENT OR COMM PARTNER ID MISMATCH");
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
	uint16_t len = mavlink_msg_to_send_buffer(missionlib_msg_buf, msg);

	mavlink_send_uart_bytes(_chan, missionlib_msg_buf, len);
}



int
Mavlink::mavlink_missionlib_send_gcs_string(const char *string)
{
	const int len = MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN;
	mavlink_statustext_t statustext;
	int i = 0;

	while (i < len - 1) {
		statustext.text[i] = string[i];

		if (string[i] == '\0')
			break;

		i++;
	}

	if (i > 1) {
		/* Enforce null termination */
		statustext.text[i] = '\0';
		mavlink_message_t msg;

		mavlink_msg_statustext_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &statustext);
		mavlink_missionlib_send_message(&msg);
		return OK;

	} else {
		return 1;
	}
}

int
Mavlink::task_main(int argc, char *argv[])
{
	/* inform about start */
	warnx("Initializing..");
	fflush(stdout);

	/* initialize mavlink text message buffering */
	mavlink_logbuffer_init(&lb, 5);

	int ch;
	_baudrate = 57600;
	_chan = MAVLINK_COMM_0;

	_mode = MODE_OFFBOARD;

	/* work around some stupidity in task_create's argv handling */
	argc -= 2;
	argv += 2;

	while ((ch = getopt(argc, argv, "b:d:eov")) != EOF) {
		switch (ch) {
		case 'b':
			_baudrate = strtoul(optarg, NULL, 10);

			if (_baudrate < 9600 || _baudrate > 921600)
				errx(1, "invalid baud rate '%s'", optarg);

			break;

		case 'd':
			device_name = optarg;
			break;

//		case 'e':
//			mavlink_link_termination_allowed = true;
//			break;

		case 'o':
			_mode = MODE_ONBOARD;
			break;

		case 'v':
			_verbose = true;
			break;

		default:
			usage();
			break;
		}
	}

	if (Mavlink::instance_exists(device_name, this)) {
		errx(1, "mavlink instance for %s already running", device_name);
	}

	struct termios uart_config_original;

	bool usb_uart;

	/* print welcome text */
	warnx("MAVLink v1.0 serial interface starting...");

	/* inform about mode */
	switch (_mode) {
		case MODE_TX_HEARTBEAT_ONLY:
			warnx("MODE_TX_HEARTBEAT_ONLY");
			break;
		case MODE_OFFBOARD:
			warnx("MODE_OFFBOARD");
			break;
		case MODE_ONBOARD:
			warnx("MODE_ONBOARD");
			break;
		case MODE_HIL:
			warnx("MODE_HIL");
			break;
		default:
			warnx("Error: Unknown mode");
			break;
	}

	switch(_mode) {
		case MODE_OFFBOARD:
		case MODE_HIL:
			_mavlink_wpm_comp_id = MAV_COMP_ID_MISSIONPLANNER;
			break;
		case MODE_ONBOARD:
			_mavlink_wpm_comp_id = MAV_COMP_ID_CAMERA;
			break;
		case MODE_TX_HEARTBEAT_ONLY:
		default:
			_mavlink_wpm_comp_id = MAV_COMP_ID_ALL;
			warnx("Error: Unknown mode");
			break;
	}

	/* Flush stdout in case MAVLink is about to take it over */
	fflush(stdout);

	/* default values for arguments */
	_uart = mavlink_open_uart(_baudrate, device_name, &uart_config_original, &usb_uart);

	if (_uart < 0)
		err(1, "could not open %s", device_name);

	/* create the device node that's used for sending text log messages, etc. */
	register_driver(MAVLINK_LOG_DEVICE, &fops, 0666, NULL);

	/* initialize logging device */
	_mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

	mavlink_log_info(_mavlink_fd, "[mavlink] started");

	/* Initialize system properties */
	mavlink_update_system();

	/* start the MAVLink receiver */
	receive_thread = MavlinkReceiver::receive_start(this);

	/* start the ORB receiver */
	uorb_receive_thread = MavlinkOrbListener::uorb_receive_start(this);

	/* initialize waypoint manager */
	mavlink_wpm_init(wpm);

	/* all subscriptions are now active, set up initial guess about rate limits */
	if (_baudrate >= 230400) {
		/* 200 Hz / 5 ms */
		set_mavlink_interval_limit(MAVLINK_MSG_ID_HIGHRES_IMU, 20);
		set_mavlink_interval_limit(MAVLINK_MSG_ID_RAW_IMU, 20);
		/* 50 Hz / 20 ms */
		set_mavlink_interval_limit(MAVLINK_MSG_ID_ATTITUDE, 30);
		/* 20 Hz / 50 ms */
		set_mavlink_interval_limit(MAVLINK_MSG_ID_NAMED_VALUE_FLOAT, 10);
		set_mavlink_interval_limit(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 50);
		/* 10 Hz */
		set_mavlink_interval_limit(MAVLINK_MSG_ID_GPS_RAW_INT, 100);
		/* 10 Hz */
		set_mavlink_interval_limit(MAVLINK_MSG_ID_MANUAL_CONTROL, 100);

	} else if (_baudrate >= 115200) {
		/* 20 Hz / 50 ms */
		set_mavlink_interval_limit(MAVLINK_MSG_ID_HIGHRES_IMU, 50);
		set_mavlink_interval_limit(MAVLINK_MSG_ID_RAW_IMU, 50);
		set_mavlink_interval_limit(MAVLINK_MSG_ID_ATTITUDE, 50);
		set_mavlink_interval_limit(MAVLINK_MSG_ID_NAMED_VALUE_FLOAT, 50);
		/* 5 Hz / 200 ms */
		set_mavlink_interval_limit(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 200);
		/* 5 Hz / 200 ms */
		set_mavlink_interval_limit(MAVLINK_MSG_ID_GPS_RAW_INT, 200);
		/* 2 Hz */
		set_mavlink_interval_limit(MAVLINK_MSG_ID_MANUAL_CONTROL, 500);

	} else if (_baudrate >= 57600) {
		/* 10 Hz / 100 ms */
		set_mavlink_interval_limit(MAVLINK_MSG_ID_RAW_IMU, 300);
		set_mavlink_interval_limit(MAVLINK_MSG_ID_HIGHRES_IMU, 300);
		/* 10 Hz / 100 ms ATTITUDE */
		set_mavlink_interval_limit(MAVLINK_MSG_ID_ATTITUDE, 200);
		/* 5 Hz / 200 ms */
		set_mavlink_interval_limit(MAVLINK_MSG_ID_NAMED_VALUE_FLOAT, 200);
		/* 5 Hz / 200 ms */
		set_mavlink_interval_limit(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 500);
		/* 2 Hz */
		set_mavlink_interval_limit(MAVLINK_MSG_ID_MANUAL_CONTROL, 500);
		/* 2 Hz */
		set_mavlink_interval_limit(MAVLINK_MSG_ID_GPS_RAW_INT, 500);

	} else {
		/* very low baud rate, limit to 1 Hz / 1000 ms */
		set_mavlink_interval_limit(MAVLINK_MSG_ID_RAW_IMU, 1000);
		set_mavlink_interval_limit(MAVLINK_MSG_ID_ATTITUDE, 1000);
		set_mavlink_interval_limit(MAVLINK_MSG_ID_HIGHRES_IMU, 1000);
		/* 1 Hz / 1000 ms */
		set_mavlink_interval_limit(MAVLINK_MSG_ID_NAMED_VALUE_FLOAT, 1000);
		/* 0.5 Hz */
		set_mavlink_interval_limit(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 2000);
		/* 0.1 Hz */
		set_mavlink_interval_limit(MAVLINK_MSG_ID_MANUAL_CONTROL, 10000);
	}

	int mission_result_sub = orb_subscribe(ORB_ID(mission_result));
	struct mission_result_s mission_result;
	memset(&mission_result, 0, sizeof(mission_result));

	thread_running = true;

	unsigned lowspeed_counter = 0;

	/* wakeup source(s) */
	struct pollfd fds[1];

	_params_sub = orb_subscribe(ORB_ID(parameter_update));

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {

		/* wait for up to 100ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		perf_begin(_loop_perf);

		/* parameters updated */
		if (fds[0].revents & POLLIN) {
			parameters_update();
		}

		/* 1 Hz */
		if (lowspeed_counter % 10 == 0) {
			mavlink_update_system();

			/* translate the current system state to mavlink state and mode */
			uint8_t mavlink_state = 0;
			uint8_t mavlink_base_mode = 0;
			uint32_t mavlink_custom_mode = 0;
			get_mavlink_mode_and_state(&mavlink_state, &mavlink_base_mode, &mavlink_custom_mode);

			/* send heartbeat */
			mavlink_msg_heartbeat_send(_chan, mavlink_system.type, MAV_AUTOPILOT_PX4, mavlink_base_mode, mavlink_custom_mode, mavlink_state);

			/* switch HIL mode if required */
			if (v_status.hil_state == HIL_STATE_ON)
				set_hil_enabled(true);
			else if (v_status.hil_state == HIL_STATE_OFF)
				set_hil_enabled(false);

			/* send status (values already copied in the section above) */
			mavlink_msg_sys_status_send(_chan,
						    v_status.onboard_control_sensors_present,
						    v_status.onboard_control_sensors_enabled,
						    v_status.onboard_control_sensors_health,
						    v_status.load * 1000.0f,
						    v_status.battery_voltage * 1000.0f,
						    v_status.battery_current * 1000.0f,
						    v_status.battery_remaining,
						    v_status.drop_rate_comm,
						    v_status.errors_comm,
						    v_status.errors_count1,
						    v_status.errors_count2,
						    v_status.errors_count3,
						    v_status.errors_count4);
		}

		/* 0.5 Hz */
		if (lowspeed_counter % 20 == 0) {

			mavlink_wpm_send_waypoint_current((uint16_t)mission_result.index_current_mission);
		}

		lowspeed_counter++;

		bool updated;
		orb_check(mission_result_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(mission_result), mission_result_sub, &mission_result);

			if (_verbose) warnx("Got mission result: new current: %d", mission_result.index_current_mission);

			if (mission_result.mission_reached) {
				mavlink_wpm_send_waypoint_reached((uint16_t)mission_result.mission_index_reached);
			}

			mavlink_wpm_send_waypoint_current((uint16_t)mission_result.index_current_mission);
		}

		mavlink_waypoint_eventloop(hrt_absolute_time());

		/* check if waypoint has been reached against the last positions */
		mavlink_waypoint_eventloop(hrt_absolute_time());


		/* send parameters at 20 Hz (if queued for sending) */
		mavlink_pm_queued_send();
		mavlink_waypoint_eventloop(hrt_absolute_time());

		mavlink_waypoint_eventloop(hrt_absolute_time());

		if (_baudrate > 57600) {
			mavlink_pm_queued_send();
		}

		 /* send one string at 10 Hz */
		 if (!mavlink_logbuffer_is_empty(&lb)) {
		 	struct mavlink_logmessage msg;
		 	int lb_ret = mavlink_logbuffer_read(&lb, &msg);

		 	if (lb_ret == OK) {
		 		mavlink_missionlib_send_gcs_string(msg.text);
		 	}
		 }

		perf_end(_loop_perf);
	}

	/* wait for threads to complete */
	pthread_join(receive_thread, NULL);
	pthread_join(uorb_receive_thread, NULL);

	/* Reset the UART flags to original state */
	tcsetattr(_uart, TCSANOW, &uart_config_original);

	/* destroy log buffer */
	mavlink_logbuffer_destroy(&lb);

	thread_running = false;

	warnx("exiting.");

	_mavlink_task = -1;
	_exit(0);
}

int Mavlink::start_helper(int argc, char *argv[])
{
	// Create the instance in task context
	Mavlink *instance = Mavlink::new_instance();
	// This will actually only return once MAVLink exits
	return instance->task_main(argc, argv);
}

int
Mavlink::start()
{

	return OK;
}

void
Mavlink::status() 
{
	warnx("Running");
}

static void usage()
{
	errx(1, "usage: mavlink {start|stop-all} [-d device] [-b baudrate] [-o] [-v]");
}

int mavlink_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
	}

	if (!strcmp(argv[1], "start")) {

		// Instantiate thread
		char buf[32];
		sprintf(buf, "mavlink if%d", Mavlink::instance_count());

		/*mavlink->_mavlink_task = */task_spawn_cmd(buf,
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2048,
						 (main_t)&Mavlink::start_helper,
						 (const char **)argv);

		// while (!this->is_running()) {
		// 		usleep(200);
		// }

		// if (mavlink->_mavlink_task < 0) {
		// 	warn("task start failed");
		// 	return -errno;
		// }

		// if (mavlink::g_mavlink != nullptr) {
		// 	errx(1, "already running");
		// }

		// mavlink::g_mavlink = new Mavlink;

		// if (mavlink::g_mavlink == nullptr) {
		// 	errx(1, "alloc failed");
		// }

		// if (OK != mavlink::g_mavlink->start()) {
		// 	delete mavlink::g_mavlink;
		// 	mavlink::g_mavlink = nullptr;
		// 	err(1, "start failed");
		// }

		return 0;

	} else if (!strcmp(argv[1], "stop")) {
		warnx("mavlink stop is deprecated, use stop-all instead");
		usage();

	} else if (!strcmp(argv[1], "stop-all")) {
		return Mavlink::destroy_all_instances();

	// } else if (!strcmp(argv[1], "status")) {
	// 	mavlink::g_mavlink->status();

	 } else {
	 	usage();
	 }

	return 0;
}
