/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * @file mavlink_parameters.c
 * MAVLink parameter protocol implementation (BSD-relicensed).
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include "mavlink_bridge_header.h"
#include <v1.0/common/mavlink.h>
#include "mavlink_parameters.h"
#include <uORB/uORB.h>
#include "math.h" /* isinf / isnan checks */
#include <assert.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <sys/stat.h>

extern mavlink_system_t mavlink_system;

extern int mavlink_missionlib_send_message(mavlink_message_t *msg);
extern int mavlink_missionlib_send_gcs_string(const char *string);

/**
 * If the queue index is not at 0, the queue sending
 * logic will send parameters from the current index
 * to len - 1, the end of the param list.
 */
static unsigned int mavlink_param_queue_index = 0;

/**
 * Callback for param interface.
 */
void mavlink_pm_callback(void *arg, param_t param);

/**
 * Save parameters to EEPROM.
 *
 * Stores the parameters to /eeprom/parameters
 */
static int mavlink_pm_save_eeprom(void);

/**
 * Load parameters from EEPROM.
 *
 * Loads the parameters from /eeprom/parameters
 */
static int mavlink_pm_load_eeprom(void);

void mavlink_pm_callback(void *arg, param_t param)
{
	mavlink_pm_send_param(param);
	usleep(*(unsigned int*)arg);
}

void mavlink_pm_send_all_params(unsigned int delay)
{
	unsigned int dbuf = delay;
	param_foreach(&mavlink_pm_callback, &dbuf, false);
}

int mavlink_pm_queued_send()
{
	if (mavlink_param_queue_index < param_count()) {
		mavlink_pm_send_param(param_for_index(mavlink_param_queue_index));
		mavlink_param_queue_index++;
		return 0;
	} else {
		return 1;
	}
}

void mavlink_pm_start_queued_send()
{
	mavlink_param_queue_index = 0;
}

int mavlink_pm_send_param_for_index(uint16_t index)
{
	return mavlink_pm_send_param(param_for_index(index));
}

int mavlink_pm_send_param_for_name(const char* name)
{
	return mavlink_pm_send_param(param_find(name));
}

int mavlink_pm_send_param(param_t param)
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
	if ((ret = param_get(param, &val_buf)) != OK) return ret;

	mavlink_msg_param_value_pack_chan(mavlink_system.sysid,
					  mavlink_system.compid,
					  MAVLINK_COMM_0,
					  &tx_msg,
					  name_buf,
					  val_buf,
					  mavlink_type,
					  param_count(),
					  param_get_index(param));
	ret = mavlink_missionlib_send_message(&tx_msg);
	return ret;
}

void mavlink_pm_message_handler(const mavlink_channel_t chan, const mavlink_message_t *msg)
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
					char name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN+1];
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
						char name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN+1];
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

		case MAVLINK_MSG_ID_COMMAND_LONG: {
			mavlink_command_long_t cmd_mavlink;
			mavlink_msg_command_long_decode(msg, &cmd_mavlink);

			uint8_t result = MAV_RESULT_UNSUPPORTED;

			if (cmd_mavlink.target_system == mavlink_system.sysid &&
				((cmd_mavlink.target_component == mavlink_system.compid) ||(cmd_mavlink.target_component == MAV_COMP_ID_ALL))) {

				/* preflight parameter load / store */
				if (cmd_mavlink.command == MAV_CMD_PREFLIGHT_STORAGE) {
					/* Read all parameters from EEPROM to RAM */

					if (((int)(cmd_mavlink.param1)) == 0)	{

						/* read all parameters from EEPROM to RAM */
						int read_ret = param_load_default();
						if (read_ret == OK) {
							//printf("[mavlink pm] Loaded EEPROM params in RAM\n");
							mavlink_missionlib_send_gcs_string("[mavlink pm] OK loaded EEPROM params");
							result = MAV_RESULT_ACCEPTED;
						} else if (read_ret == 1) {
							mavlink_missionlib_send_gcs_string("[mavlink pm] No stored parameters to load");
							result = MAV_RESULT_ACCEPTED;
						} else {
							if (read_ret < -1) {
								mavlink_missionlib_send_gcs_string("[mavlink pm] ERR loading params from EEPROM");
							} else {
								mavlink_missionlib_send_gcs_string("[mavlink pm] ERR loading params, no EEPROM found");
							}
							result = MAV_RESULT_FAILED;
						}

					} else if (((int)(cmd_mavlink.param1)) == 1)	{

						/* write all parameters from RAM to EEPROM */
						int write_ret = param_save_default();
						if (write_ret == OK) {
							mavlink_missionlib_send_gcs_string("[mavlink pm] OK params written to EEPROM");
							result = MAV_RESULT_ACCEPTED;

						} else {
							if (write_ret < -1) {
								mavlink_missionlib_send_gcs_string("[mavlink pm] ERR writing params to EEPROM");
							} else {
								mavlink_missionlib_send_gcs_string("[mavlink pm] ERR writing params, no EEPROM found");
							}
							result = MAV_RESULT_FAILED;
						}

					} else {
						//fprintf(stderr, "[mavlink pm] refusing unsupported storage request\n");
						mavlink_missionlib_send_gcs_string("[mavlink pm] refusing unsupported STOR request");
						result = MAV_RESULT_UNSUPPORTED;
					}
				}
			}

			/* send back command result */
			//mavlink_msg_command_ack_send(chan, cmd.command, result);
		} break;
	}
}
