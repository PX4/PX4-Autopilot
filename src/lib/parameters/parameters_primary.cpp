/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "parameters_primary.h"

#include "uORB/uORBManager.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>

// uORB topics needed to keep parameter server and client in sync
#include <uORB/topics/parameter_reset_request.h>
#include <uORB/topics/parameter_set_used_request.h>
#include <uORB/topics/parameter_set_value_request.h>
#include <uORB/topics/parameter_set_value_response.h>

// Debug flag
static bool debug = false;

#define TIMEOUT_WAIT 1000
#define TIMEOUT_COUNT 50

static px4_task_t sync_thread_tid;
static const char *sync_thread_name = "param_primary_sync";

static orb_advert_t param_set_value_req_h = nullptr;

static int param_set_rsp_fd = PX4_ERROR;

static int primary_sync_thread(int argc, char *argv[]) {
	// Need to wait until the uORB and muORB are ready
	// Check for uORB initialization with get_instance
	while (uORB::Manager::get_instance() == nullptr) { px4_usleep(100); }

	// Check for muORB initialization with get_uorb_communicator
	while (uORB::Manager::get_instance()->get_uorb_communicator() == nullptr) { px4_usleep(100); }

	orb_advert_t _set_value_rsp_h = nullptr;

	int _set_used_req_fd  = orb_subscribe(ORB_ID(parameter_set_used_request));
	int _set_value_req_fd = orb_subscribe(ORB_ID(parameter_primary_set_value_request));

	struct parameter_set_used_request_s   _set_used_request;
	struct parameter_set_value_request_s  _set_value_request;
	struct parameter_set_value_response_s _set_value_response;

	px4_pollfd_struct_t fds[2] = { { .fd = _set_used_req_fd,  .events = POLLIN },
		{ .fd = _set_value_req_fd, .events = POLLIN }
	};

	PX4_INFO("Starting parameter primary sync thread");

	while (true) {
		px4_poll(fds, 2, 1000);

		if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(parameter_set_used_request), _set_used_req_fd, &_set_used_request);

			PX4_INFO("Got parameter_set_used_request for %s", param_name(_set_used_request.parameter_index));

			// param_server_counters.set_used_received++;

			param_find(param_name(_set_used_request.parameter_index));

		} else if (fds[1].revents & POLLIN) {
			orb_copy(ORB_ID(parameter_primary_set_value_request), _set_value_req_fd, &_set_value_request);

			PX4_INFO("Got parameter_primary_set_value_request for %s", param_name(_set_value_request.parameter_index));

			// param_server_counters.set_value_received++;

			param_t param = _set_value_request.parameter_index;
			// param_value_u value;
			// value.i = 0;
			// value.f = 0.0f;

			switch (param_type(param)) {
			case PARAM_TYPE_INT32:
				param_set_no_remote_update(param, (const void *) &_set_value_request.int_value, true);
				// value.i = _set_value_request.int_value;
				break;

			case PARAM_TYPE_FLOAT:
				param_set_no_remote_update(param, (const void *) &_set_value_request.float_value, true);
				// value.f =_set_value_request.float_value;
				break;

			default:
				PX4_ERR("Parameter must be either int or float");
				break;
			}

			_set_value_response.timestamp = hrt_absolute_time();
			_set_value_response.request_timestamp = _set_value_request.timestamp;
			_set_value_response.parameter_index = _set_value_request.parameter_index;

			if (_set_value_rsp_h == nullptr) {
				_set_value_rsp_h = orb_advertise(ORB_ID(parameter_primary_set_value_response), &_set_value_response);

			} else {
				orb_publish(ORB_ID(parameter_primary_set_value_response), _set_value_rsp_h, &_set_value_response);
			}

			// save_calibration_parameter_to_file(v_req.parameter_name, param_type(param), value);
		}
	}

    return 0;
}

void param_primary_init() {

	sync_thread_tid = px4_task_spawn_cmd(sync_thread_name,
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_PARAMS,
					     (1024 * 4),
					     primary_sync_thread,
					     NULL);

}

// void param_primary_set(param_t param, const void *val, bool from_file)
void param_primary_set(param_t param, const void *val)
{
	bool send_request = true;
	struct parameter_set_value_request_s req;
	req.timestamp = hrt_absolute_time();
	req.parameter_index = param;
	// param_value_u value;
	// value.i = 0;
	// value.f = 0.0f;

	switch (param_type(param)) {
	case PARAM_TYPE_INT32:
		req.int_value = *(int32_t *)val;
		// value.i = req.int_value;

		// PX4_INFO("*** Setting %s to %d ***", param_name(req.parameter_index), req.int_value);

		break;

	case PARAM_TYPE_FLOAT:
		req.float_value = *(float *)val;
		// value.f = req.float_value;

		// PX4_INFO("*** Setting %s to %f ***", param_name(req.parameter_index), (double) req.float_value);

		break;

	default:
		PX4_ERR("Parameter must be either int or float");
		send_request = false;
		break;
	}

	// if (! from_file) {
	// 	save_calibration_parameter_to_file(req.parameter_name, param_type(param), value);
	// }

	if (param_set_rsp_fd == PX4_ERROR) {
		// PX4_INFO("Subscribing to parameter_client_set_value_response");

		param_set_rsp_fd = orb_subscribe(ORB_ID(parameter_remote_set_value_response));

		if (param_set_rsp_fd == PX4_ERROR) {
			PX4_ERR("Subscription to parameter_remote_set_value_response failed");

		// } else {
			// PX4_INFO("Subscription to parameter_client_set_value_response succeeded");
		}
	}

	if (send_request) {
		if (debug) { PX4_INFO("Sending param set request to remote for %s", param_name(req.parameter_index)); }

		if (param_set_value_req_h == nullptr) {
			param_set_value_req_h = orb_advertise(ORB_ID(parameter_remote_set_value_request), nullptr);
		}

		orb_publish(ORB_ID(parameter_remote_set_value_request), param_set_value_req_h, &req);

		// param_server_counters.set_value_sent++;

		// Wait for response
		bool updated = false;

		// PX4_INFO("Waiting for parameter_client_set_value_response for %s", param_name(req.parameter_index));

		px4_usleep(TIMEOUT_WAIT);
		int count = TIMEOUT_COUNT;

		while (--count) {
			(void) orb_check(param_set_rsp_fd, &updated);

			struct parameter_set_value_response_s rsp;
			while (updated) {

				orb_copy(ORB_ID(parameter_remote_set_value_response), param_set_rsp_fd, &rsp);

				if ((rsp.request_timestamp == req.timestamp) && (rsp.parameter_index == req.parameter_index)) {
					if (debug) { PX4_INFO("Got parameter_remote_set_value_response for %s", param_name(req.parameter_index)); }
					return;
				}

				(void) orb_check(param_set_rsp_fd, &updated);
			}

			px4_usleep(TIMEOUT_WAIT);
		}

		PX4_ERR("Timeout waiting for parameter_client_set_value_response for %s", param_name(req.parameter_index));
	}
}

