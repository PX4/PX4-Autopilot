/****************************************************************************
 *
 * Copyright (C) 2022 ModalAI, Inc. All rights reserved.
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

#include <algorithm>
#include <string>
#include <sstream>
#include <string.h>
#include <stdbool.h>
#include <fstream>
#include <iostream>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>

#include "param_server.h"
#include "uORB/uORBManager.hpp"

// uORB topics needed to keep parameter server and client in sync
#include <uORB/topics/parameter_client_reset_request.h>
#include <uORB/topics/parameter_client_reset_response.h>
#include <uORB/topics/parameter_server_set_value_request.h>
#include <uORB/topics/parameter_server_set_value_response.h>
#include <uORB/topics/parameter_client_set_value_request.h>
#include <uORB/topics/parameter_client_set_value_response.h>
#include <uORB/topics/parameter_server_set_used_request.h>
#include <uORB/topics/parameter_server_set_used_response.h>

// Debug flag
static bool debug = false;

#define TIMEOUT_WAIT 1000
#define TIMEOUT_COUNT 50

// Advertisement handles
static orb_advert_t param_set_value_req_h = nullptr;
static orb_advert_t param_reset_req_h     = nullptr;

// Subscription file descriptors
static int param_set_rsp_fd   = PX4_ERROR;
static int param_reset_rsp_fd = PX4_ERROR;

static px4_task_t   sync_thread_tid;
static const char  *sync_thread_name = "server_sync_thread";

static struct param_server_counters param_server_counters;

using namespace std;

static void save_calibration_parameter_to_file(const char *name, param_type_t type, param_value_u value) {
	// If the parameter being set is a calibration parameter then save it out to
	// a separate calibration file so that they can be preserved and reloaded
	// after system updates
	string cal_file_name = param_get_default_file();
	string cal_file_append;
	string param_name(name);
	string cal_strings[] = {"CAL_GYRO", "CAL_MAG", "CAL_BARO", "CAL_ACC"};
	for (auto i: cal_strings) {
		// Check to see if the parameter is one of the desired calibration parameters
		if (param_name.substr(0, i.size()) == i) {
			// We want the filename to be the standard parameters file name with
			// the calibration type appended to it.
			cal_file_append = i.substr(3, i.size());
			// Make sure it is lowercase
			transform(cal_file_append.begin(), cal_file_append.end(), cal_file_append.begin(), ::tolower);
			// And add a cal file extension
			cal_file_append += ".cal";
			break;
		}
	}

	// Check for level horizon calibration parameters
	if (cal_file_append.empty() &&
		(param_name == "SENS_BOARD_X_OFF" || param_name == "SENS_BOARD_Y_OFF")) {
		cal_file_append = "_level.cal";
	}

	// Check for RC calibration parameters
	if (cal_file_append.empty() && name[0] == 'R' && name[1] == 'C' && isdigit(name[2])) {
		cal_file_append = "_rc.cal";
	}

	if (! cal_file_append.empty()) {
		cal_file_name += cal_file_append;

		stringstream param_data_stream;

		switch (type) {
		case PARAM_TYPE_INT32:
			param_data_stream << value.i;
			param_data_stream << "\t" << 6;
			break;

		case PARAM_TYPE_FLOAT:
			param_data_stream << value.f;
			param_data_stream << "\t" << 9;
			break;

		default:
			PX4_ERR("Calibration parameter must be either int or float");
			break;
		}

		string param_data;
		param_data += "1\t1\t";
		param_data += param_name;
		param_data += "\t";
		param_data += param_data_stream.str();

		PX4_INFO("Writing %s to file %s", param_data.c_str(), cal_file_name.c_str());

		// open a file in write (append) mode.
		ofstream cal_file;
		cal_file.open(cal_file_name, ios_base::app);
		if (cal_file) {
			cal_file << param_data << endl;
			cal_file.close();
		} else {
			PX4_ERR("Couldn't open %s for writing calibration value", cal_file_name.c_str());
		}
	}
}

static int param_sync_thread(int argc, char *argv[])
{

	// Need to wait until the uORB and muORB are ready
	// Check for uORB initialization with get_instance
	while (uORB::Manager::get_instance() == nullptr) { usleep(100); }

	// Check for muORB initialization with get_uorb_communicator
	while (uORB::Manager::get_instance()->get_uorb_communicator() == nullptr) { usleep(100); }

	orb_advert_t param_set_used_rsp_h  = nullptr;
	orb_advert_t param_set_value_rsp_h = nullptr;

	int parameter_server_set_used_fd  = orb_subscribe(ORB_ID(parameter_server_set_used_request));
	int parameter_server_set_value_fd = orb_subscribe(ORB_ID(parameter_server_set_value_request));

	struct parameter_server_set_used_request_s   u_req;
	struct parameter_server_set_used_response_s  u_rsp;
	struct parameter_server_set_value_request_s  v_req;
	struct parameter_server_set_value_response_s v_rsp;

	px4_pollfd_struct_t fds[2] = { { .fd = parameter_server_set_used_fd,  .events = POLLIN },
		{ .fd = parameter_server_set_value_fd, .events = POLLIN }
	};

	PX4_INFO("Starting param sync THREAD");

	while (true) {
		px4_poll(fds, 2, 1000);

		if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(parameter_server_set_used_request), parameter_server_set_used_fd, &u_req);

			if (debug) { PX4_INFO("Got parameter_server_set_used_request for %s", u_req.parameter_name); }

			param_server_counters.set_used_received++;

			(void) param_find(u_req.parameter_name);

			u_rsp.timestamp = hrt_absolute_time();

			if (param_set_used_rsp_h == nullptr) {
				param_set_used_rsp_h = orb_advertise(ORB_ID(parameter_server_set_used_response), &u_rsp);

			} else {
				orb_publish(ORB_ID(parameter_server_set_used_response), param_set_used_rsp_h, &u_rsp);
			}

		} else if (fds[1].revents & POLLIN) {
			orb_copy(ORB_ID(parameter_server_set_value_request), parameter_server_set_value_fd, &v_req);

			PX4_DEBUG("Got parameter_server_set_value_request for %s", v_req.parameter_name);

			param_server_counters.set_value_received++;

			param_t param = param_find(v_req.parameter_name);
			param_value_u value;
			value.i = 0;
			value.f = 0.0f;

			switch (param_type(param)) {
			case PARAM_TYPE_INT32:
				param_set_no_remote_update(param, (const void *) &v_req.int_value, true);
				value.i = v_req.int_value;
				break;

			case PARAM_TYPE_FLOAT:
				param_set_no_remote_update(param, (const void *) &v_req.float_value, true);
				value.f =v_req.float_value;
				break;

			default:
				PX4_ERR("Parameter must be either int or float");
				break;
			}

			v_rsp.timestamp = hrt_absolute_time();

			if (param_set_value_rsp_h == nullptr) {
				param_set_value_rsp_h = orb_advertise(ORB_ID(parameter_server_set_value_response), &v_rsp);

			} else {
				orb_publish(ORB_ID(parameter_server_set_value_response), param_set_value_rsp_h, &v_rsp);
			}

			save_calibration_parameter_to_file(v_req.parameter_name, param_type(param), value);
		}
	}

	return 0;
}

void param_server_init()
{
	sync_thread_tid = px4_task_spawn_cmd(sync_thread_name,
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_PARAMS,
					     (1024 * 4),
					     param_sync_thread,
					     NULL);
}

void param_server_set(param_t param, const void *val, bool from_file)
{
	bool send_request = true;
	struct parameter_client_set_value_request_s req;
	req.timestamp = hrt_absolute_time();
	strncpy(req.parameter_name, param_name(param), 16);
	req.parameter_name[16] = 0;
	param_value_u value;
	value.i = 0;
	value.f = 0.0f;

	switch (param_type(param)) {
	case PARAM_TYPE_INT32:
		req.int_value = *(int32_t *)val;
		value.i = req.int_value;

		if (debug) { PX4_INFO("*** Setting %s to %d ***", req.parameter_name, req.int_value); }

		break;

	case PARAM_TYPE_FLOAT:
		req.float_value = *(float *)val;
		value.f = req.float_value;

		if (debug) { PX4_INFO("*** Setting %s to %f ***", req.parameter_name, (double) req.float_value); }

		break;

	default:
		PX4_ERR("Parameter must be either int or float");
		send_request = false;
		break;
	}

	if (! from_file) {
		save_calibration_parameter_to_file(req.parameter_name, param_type(param), value);
	}

	if (param_set_rsp_fd == PX4_ERROR) {
		if (debug) { PX4_INFO("Subscribing to parameter_client_set_value_response"); }

		param_set_rsp_fd = orb_subscribe(ORB_ID(parameter_client_set_value_response));

		if (param_set_rsp_fd == PX4_ERROR) {
			PX4_ERR("Subscription to parameter_client_set_value_response failed");

		} else {
			if (debug) { PX4_INFO("Subscription to parameter_client_set_value_response succeeded"); }
		}
	}

	if (send_request) {
		if (debug) { PX4_INFO("Sending param set request to client for %s", req.parameter_name); }

		if (param_set_value_req_h == nullptr) {
			param_set_value_req_h = orb_advertise(ORB_ID(parameter_client_set_value_request), nullptr);
		}

		orb_publish(ORB_ID(parameter_client_set_value_request), param_set_value_req_h, &req);

		param_server_counters.set_value_sent++;

		// Wait for response
		bool updated = false;

		if (debug) { PX4_INFO("Waiting for parameter_client_set_value_response for %s", req.parameter_name); }

		usleep(TIMEOUT_WAIT);
		int count = TIMEOUT_COUNT;

		while (--count) {
			(void) orb_check(param_set_rsp_fd, &updated);

			if (updated) {
				if (debug) { PX4_INFO("Got parameter_client_set_value_response for %s", req.parameter_name); }

				struct parameter_client_set_value_response_s rsp;

				orb_copy(ORB_ID(parameter_client_set_value_response), param_set_rsp_fd, &rsp);

				break;
			}

			usleep(TIMEOUT_WAIT);
		}

		if (! count) {
			PX4_ERR("Timeout waiting for parameter_client_set_value_response for %s", req.parameter_name);
		}
	}
}

static void param_server_reset_internal(param_t param, bool reset_all)
{
	if (debug) { PX4_INFO("Param reset in server"); }

	struct parameter_client_reset_request_s req;

	req.timestamp = hrt_absolute_time();

	req.reset_all = reset_all;

	if (reset_all == false) {
		strncpy(req.parameter_name, param_name(param), 16);
		req.parameter_name[16] = 0;
	}

	if (param_reset_rsp_fd == PX4_ERROR) {
		if (debug) { PX4_INFO("Subscribing to parameter_client_reset_response"); }

		param_reset_rsp_fd = orb_subscribe(ORB_ID(parameter_client_reset_response));

		if (param_reset_rsp_fd == PX4_ERROR) {
			PX4_ERR("Subscription to parameter_client_reset_response failed");

		} else {
			if (debug) { PX4_INFO("Subscription to parameter_client_reset_response succeeded"); }
		}
	}

	if (debug) { PX4_INFO("Sending param reset request to client"); }

	if (param_reset_req_h == nullptr) {
		param_reset_req_h = orb_advertise(ORB_ID(parameter_client_reset_request), nullptr);
	}

	orb_publish(ORB_ID(parameter_client_reset_request), param_reset_req_h, &req);

	param_server_counters.reset_sent++;

	// Wait for response
	if (debug) { PX4_INFO("Waiting for parameter_client_reset_response"); }

	usleep(TIMEOUT_WAIT);
	bool updated = false;
	int count = TIMEOUT_COUNT;

	while (--count) {
		(void) orb_check(param_reset_rsp_fd, &updated);

		if (updated) {
			if (debug) { PX4_INFO("Got parameter_client_reset_response"); }

			struct parameter_client_reset_response_s rsp;

			orb_copy(ORB_ID(parameter_client_reset_response), param_reset_rsp_fd, &rsp);

			break;
		}

		usleep(TIMEOUT_WAIT);
	}

	if (! count) {
		PX4_ERR("Timeout waiting for parameter_client_reset_response");
	}
}

void param_server_reset(param_t param)
{
	param_server_reset_internal(param, false);
}

void param_server_reset_all()
{
	param_server_reset_internal(0, true);
}

void param_server_get_counters(struct param_server_counters *cnt)
{
	*cnt = param_server_counters;
}
