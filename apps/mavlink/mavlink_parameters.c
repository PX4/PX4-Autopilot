/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: @author Lorenz Meier <lm@inf.ethz.ch>
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
 */

#include "mavlink_parameters.h"
#include <uORB/uORB.h>
#include "math.h" /* isinf / isnan checks */
#include <assert.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdbool.h>
#include <string.h>

extern mavlink_system_t mavlink_system;

extern void mavlink_missionlib_send_message(mavlink_message_t *msg);
extern void mavlink_missionlib_send_gcs_string(const char *string);

/* send one parameter, assume lock on global_data_parameter_storage */
void mavlink_pm_send_one_parameter(uint16_t next_param)
{
	if (next_param < global_data_parameter_storage->pm.size) {
		static char name_buf[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN];
		mavlink_message_t tx_msg;

		strncpy((char *)name_buf, global_data_parameter_storage->pm.param_names[next_param], MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);

		mavlink_msg_param_value_pack_chan(mavlink_system.sysid,
						  mavlink_system.compid,
						  MAVLINK_COMM_0,
						  &tx_msg,
						  name_buf,
						  global_data_parameter_storage->pm.param_values[next_param],
						  MAVLINK_TYPE_FLOAT,
						  global_data_parameter_storage->pm.size,
						  next_param);
		mavlink_missionlib_send_message(&tx_msg);


		// mavlink_msg_param_value_send(MAVLINK_COMM_0,
		// 			     name_buf,
		// 			     global_data_parameter_storage->pm.param_values[next_param],
		// 			     MAVLINK_TYPE_FLOAT,
		// 			     global_data_parameter_storage->pm.size,
		// 			     next_param);
	}
}

void mavlink_pm_message_handler(const mavlink_channel_t chan, const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
			/* Start sending parameters */
			global_data_parameter_storage->pm.next_param = 0;
			mavlink_missionlib_send_gcs_string("[pm] sending list");
		} break;

	case MAVLINK_MSG_ID_PARAM_SET: {

			/* Handle parameter setting */

			if (msg->msgid == MAVLINK_MSG_ID_PARAM_SET) {
				mavlink_param_set_t mavlink_param_set;
				mavlink_msg_param_set_decode(msg, &mavlink_param_set);

				if (mavlink_param_set.target_system == mavlink_system.sysid && ((mavlink_param_set.target_component == mavlink_system.compid) || (mavlink_param_set.target_component == MAV_COMP_ID_ALL))) {

					uint16_t i; //parameters
					uint16_t j; //chars
					bool match;

					for (i = 0; i < PARAM_MAX_COUNT; i++) {
						match = true;

						for (j = 0; j < MAX_PARAM_NAME_LEN; j++) {
							/* Compare char by char */
							if (global_data_parameter_storage->pm.param_names[i][j] != mavlink_param_set.param_id[j]) {
								match = false;
							}

							/* End matching if null termination is reached */
							if (global_data_parameter_storage->pm.param_names[i][j] == '\0') {
								break;
							}
						}

						/* Check if matched */
						if (match) {
							// XXX handle param type as well, assuming float here
							global_data_parameter_storage->pm.param_values[i] = mavlink_param_set.param_value;
							mavlink_pm_send_one_parameter(i);
						}
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

					uint16_t i; //parameters
					uint16_t j; //chars
					bool match;

					for (i = 0; i < PARAM_MAX_COUNT; i++) {
						match = true;

						for (j = 0; j < MAX_PARAM_NAME_LEN; j++) {
							/* Compare char by char */
							if (global_data_parameter_storage->pm.param_names[i][j] != mavlink_param_request_read.param_id[j]) {
								match = false;
							}

							/* End matching if null termination is reached */
							if (global_data_parameter_storage->pm.param_names[i][j] == '\0') {
								break;
							}
						}

						/* Check if matched */
						if (match) {
							mavlink_pm_send_one_parameter(i);
						}
					}

				} else {
					/* when index is >= 0, send this parameter again */
					mavlink_pm_send_one_parameter(mavlink_param_request_read.param_index);
				}
			}

		} break;
	}
}


/**
 * Send low-priority messages at a maximum rate of xx Hertz.
 *
 * This function sends messages at a lower rate to not exceed the wireless
 * bandwidth. It sends one message each time it is called until the buffer is empty.
 * Call this function with xx Hertz to increase/decrease the bandwidth.
 */
void mavlink_pm_queued_send(void)
{
	//send parameters one by one:
	mavlink_pm_send_one_parameter(global_data_parameter_storage->pm.next_param);
	global_data_parameter_storage->pm.next_param++;
}
