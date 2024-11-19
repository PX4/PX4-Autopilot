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

#include "parameters_remote.h"

#include "uORB/uORBManager.hpp"

#include <inttypes.h>

#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>

// uORB topics needed to keep parameter server and client in sync
#include <uORB/topics/parameter_reset_request.h>
#include <uORB/topics/parameter_set_used_request.h>
#include <uORB/topics/parameter_set_value_request.h>
#include <uORB/topics/parameter_set_value_response.h>

// Debug flag
static bool debug = false;

static struct param_remote_counters param_remote_counters;

#define TIMEOUT_WAIT 1000
#define TIMEOUT_COUNT 50

static orb_advert_t parameter_set_used_h = nullptr;
static orb_advert_t param_set_value_req_h = nullptr;

static int param_set_rsp_fd = PX4_ERROR;

static px4_task_t sync_thread_tid;
static const char *sync_thread_name = "param_remote_sync";

static int remote_sync_thread(int argc, char *argv[])
{
	// This thread gets started by the remote side during PX4 initialization.
	// We cannot send out the subscribe request immediately because the other
	// side will not be ready to receive it on the muorb yet and it will get dropped.
	// So, sleep a little bit to give other side a chance to finish initialization
	// of the muorb. But don't wait too long otherwise a set request from the server
	// side could be missed.
	usleep(200000);

	orb_advert_t _set_value_rsp_h = nullptr;

	int _reset_req_fd  = orb_subscribe(ORB_ID(parameter_reset_request));
	int _set_value_req_fd = orb_subscribe(ORB_ID(parameter_remote_set_value_request));

	struct parameter_reset_request_s      _reset_request;
	struct parameter_set_value_request_s  _set_value_request;
	struct parameter_set_value_response_s _set_value_response;

	px4_pollfd_struct_t fds[2] = { { .fd = _reset_req_fd,  .events = POLLIN },
		{ .fd = _set_value_req_fd, .events = POLLIN }
	};

	PX4_INFO("Starting parameter remote sync thread");

	while (true) {
		px4_poll(fds, 2, 1000);

		if (fds[0].revents & POLLIN) {
			bool updated = true;

			while (updated) {
				orb_copy(ORB_ID(parameter_reset_request), _reset_req_fd, &_reset_request);

				if (debug) {
					PX4_INFO("Got parameter_reset_request for %s", param_name(_reset_request.parameter_index));
				}

				param_remote_counters.reset_received++;

				if (_reset_request.reset_all) {
					param_reset_all();

				} else {
					param_reset_no_notification(_reset_request.parameter_index);

				}

				(void) orb_check(_reset_req_fd, &updated);
			}
		}

		if (fds[1].revents & POLLIN) {
			bool updated = true;

			while (updated) {
				orb_copy(ORB_ID(parameter_primary_set_value_request), _set_value_req_fd, &_set_value_request);

				if (debug) {
					PX4_INFO("Got parameter_remote_set_value_request for %s", param_name(_set_value_request.parameter_index));
				}

				param_remote_counters.set_value_request_received++;

				switch (param_type(_set_value_request.parameter_index)) {
				case PARAM_TYPE_INT32:
					param_set_no_remote_update(_set_value_request.parameter_index,
								   (const void *) &_set_value_request.int_value,
								   false);
					break;

				case PARAM_TYPE_FLOAT:
					param_set_no_remote_update(_set_value_request.parameter_index,
								   (const void *) &_set_value_request.float_value,
								   false);
					break;

				default:
					PX4_ERR("Parameter must be either int or float");
					break;
				}

				_set_value_response.timestamp = hrt_absolute_time();
				_set_value_response.request_timestamp = _set_value_request.timestamp;
				_set_value_response.parameter_index = _set_value_request.parameter_index;

				if (_set_value_rsp_h == nullptr) {
					_set_value_rsp_h = orb_advertise(ORB_ID(parameter_remote_set_value_response), &_set_value_response);

				} else {
					if (debug) {
						PX4_INFO("Sending set value response for %s", param_name(_set_value_request.parameter_index));
					}

					orb_publish(ORB_ID(parameter_remote_set_value_response), _set_value_rsp_h, &_set_value_response);
				}

				param_remote_counters.set_value_response_sent++;

				(void) orb_check(_set_value_req_fd, &updated);
			}
		}
	}

	return 0;
}

void param_remote_init()
{

	sync_thread_tid = px4_task_spawn_cmd(sync_thread_name,
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_PARAMS,
					     (1024 * 4),
					     remote_sync_thread,
					     NULL);

}

void param_remote_set_used(param_t param)
{
	// Notify the parameter server that this parameter has been marked as used
	if (debug) {
		PX4_INFO("Requesting server to mark %s as used", param_name(param));
	}

	struct parameter_set_used_request_s req;

	req.timestamp = hrt_absolute_time();

	req.parameter_index = param;

	if (parameter_set_used_h == nullptr) {
		parameter_set_used_h = orb_advertise(ORB_ID(parameter_set_used_request), &req);

	} else {
		orb_publish(ORB_ID(parameter_set_used_request), parameter_set_used_h, &req);
	}

	param_remote_counters.set_used_sent++;
}

void param_remote_set_value(param_t param, const void *val)
{
	bool send_request = true;
	struct parameter_set_value_request_s req;
	req.timestamp = hrt_absolute_time();
	req.parameter_index = param;

	switch (param_type(param)) {
	case PARAM_TYPE_INT32:
		req.int_value = *(int32_t *)val;

		if (debug) {
			PX4_INFO("*** Setting %s to %" PRIi32 " ***", param_name(req.parameter_index), req.int_value);
		}

		break;

	case PARAM_TYPE_FLOAT:
		req.float_value = *(float *)val;

		if (debug) {
			PX4_INFO("*** Setting %s to %f ***", param_name(req.parameter_index), (double) req.float_value);
		}

		break;

	default:
		PX4_ERR("Parameter must be either int or float");
		send_request = false;
		break;
	}

	if (param_set_rsp_fd == PX4_ERROR) {
		if (debug) {
			PX4_INFO("Subscribing to parameter_primary_set_value_response");
		}

		param_set_rsp_fd = orb_subscribe(ORB_ID(parameter_primary_set_value_response));

		if (param_set_rsp_fd == PX4_ERROR) {
			PX4_ERR("Subscription to parameter_primary_set_value_response failed");

		} else {
			if (debug) {
				PX4_INFO("Subscription to parameter_primary_set_value_response succeeded");
			}
		}
	}

	if (send_request) {
		if (debug) {
			PX4_INFO("Sending param set value request to primary for %s", param_name(req.parameter_index));
		}

		if (param_set_value_req_h == nullptr) {
			param_set_value_req_h = orb_advertise(ORB_ID(parameter_primary_set_value_request), nullptr);
		}

		orb_publish(ORB_ID(parameter_primary_set_value_request), param_set_value_req_h, &req);

		param_remote_counters.set_value_request_sent++;

		// Wait for response
		bool updated = false;

		if (debug) {
			PX4_INFO("Waiting for parameter_client_set_value_response for %s", param_name(req.parameter_index));
		}

		px4_usleep(TIMEOUT_WAIT);
		int count = TIMEOUT_COUNT;

		while (--count) {
			(void) orb_check(param_set_rsp_fd, &updated);

			struct parameter_set_value_response_s rsp;

			while (updated) {

				orb_copy(ORB_ID(parameter_primary_set_value_response), param_set_rsp_fd, &rsp);

				if ((rsp.request_timestamp == req.timestamp) && (rsp.parameter_index == req.parameter_index)) {
					if (debug) {
						PX4_INFO("Got parameter_primary_set_value_response for %s", param_name(req.parameter_index));
					}

					param_remote_counters.set_value_response_received++;
					return;
				}

				(void) orb_check(param_set_rsp_fd, &updated);
			}

			px4_usleep(TIMEOUT_WAIT);
		}

		PX4_ERR("Timeout waiting for parameter_primary_set_value_response for %s", param_name(req.parameter_index));
	}
}

void param_remote_get_counters(struct param_remote_counters *cnt)
{
	*cnt = param_remote_counters;
}
