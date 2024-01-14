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

#include <string.h>
#include <stdbool.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>

#include "param_client.h"
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

static px4_task_t   sync_thread_tid;
static const char  *sync_thread_name = "client_sync_thread";

static orb_advert_t parameter_set_used_h  = nullptr;
static orb_advert_t parameter_set_value_h = nullptr;

static struct param_client_counters param_client_counters;

static int param_set_used_rsp_fd  = PX4_ERROR;
static int param_set_value_rsp_fd = PX4_ERROR;

static int param_sync_thread(int argc, char *argv[])
{

	// This thread gets started by the client side during PX4 initialization.
	// We cannot send out the subscribe request immediately because the server
	// side will not be ready to receive it on the muorb yet and it will get dropped.
	// So, sleep a little bit to give server side a chance to finish initialization
	// of the muorb. But don't wait too long otherwise a set request from the server
	// side could be missed.
	usleep(200000);

	int param_set_req_fd   = orb_subscribe(ORB_ID(parameter_client_set_value_request));
	int param_reset_req_fd = orb_subscribe(ORB_ID(parameter_client_reset_request));

	orb_advert_t param_set_rsp_h   = nullptr;
	orb_advert_t param_reset_rsp_h = nullptr;

	struct parameter_client_set_value_request_s  s_req;
	struct parameter_client_set_value_response_s s_rsp;
	struct parameter_client_reset_request_s      r_req;
	struct parameter_client_reset_response_s     r_rsp;

	PX4_INFO("Starting param sync THREAD");

	bool updated = false;

	while (true) {
		usleep(10000);
		(void) orb_check(param_set_req_fd, &updated);

		if (updated) {
			orb_copy(ORB_ID(parameter_client_set_value_request), param_set_req_fd, &s_req);

			if (debug) { PX4_INFO("Got parameter_client_set_value_request for %s", s_req.parameter_name); }

			param_client_counters.set_value_received++;

			// This will find the parameter and also set its used flag
			param_t param = param_find(s_req.parameter_name);

			switch (param_type(param)) {
			case PARAM_TYPE_INT32:
				param_set_no_remote_update(param, (const void *) &s_req.int_value, false);
				break;

			case PARAM_TYPE_FLOAT:
				param_set_no_remote_update(param, (const void *) &s_req.float_value, false);
				break;

			default:
				PX4_ERR("Parameter must be either int or float");
				break;
			}

			s_rsp.timestamp = hrt_absolute_time();

			if (param_set_rsp_h == nullptr) {
				param_set_rsp_h = orb_advertise(ORB_ID(parameter_client_set_value_response), &s_rsp);

			} else {
				orb_publish(ORB_ID(parameter_client_set_value_response), param_set_rsp_h, &s_rsp);
			}
		}

		(void) orb_check(param_reset_req_fd, &updated);

		if (updated) {
			orb_copy(ORB_ID(parameter_client_reset_request), param_reset_req_fd, &r_req);

			if (debug) { PX4_INFO("Got parameter_client_reset_request"); }

			param_client_counters.reset_received++;

			if (r_req.reset_all) {
				param_reset_all();

			} else {
				param_t param = param_find_no_notification(r_req.parameter_name);
				param_reset_no_notification(param);
			}

			r_rsp.timestamp = hrt_absolute_time();

			if (param_reset_rsp_h == nullptr) {
				param_reset_rsp_h = orb_advertise(ORB_ID(parameter_client_reset_response), &r_rsp);

			} else {
				orb_publish(ORB_ID(parameter_client_reset_response), param_reset_rsp_h, &r_rsp);
			}
		}
	}

	return 0;
}

void
param_client_init()
{
	sync_thread_tid = px4_task_spawn_cmd(sync_thread_name,
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_PARAMS,
					     (1024 * 4),
					     param_sync_thread,
					     NULL);
}

void param_client_set(param_t param, const void *val)
{

	// If this is the parameter client, make sure that the server is updated
	bool send_event = true;
	struct parameter_server_set_value_request_s req;
	req.timestamp = hrt_absolute_time();
	strncpy(req.parameter_name, param_name(param), 16);
	req.parameter_name[16] = 0;

	switch (param_type(param)) {
	case PARAM_TYPE_INT32:
		req.int_value = *(int32_t *)val;
		PX4_DEBUG("*** Setting %s to %d ***", req.parameter_name, req.int_value);
		break;

	case PARAM_TYPE_FLOAT:
		req.float_value = *(float *)val;
		PX4_DEBUG("*** Setting %s to %f ***", req.parameter_name, (double) req.float_value);
		break;

	default:
		PX4_ERR("Parameter must be either int or float");
		send_event = false;
		break;
	}

	if (param_set_value_rsp_fd == PX4_ERROR) {
		if (debug) { PX4_INFO("Subscribing to parameter_server_set_value_response"); }

		param_set_value_rsp_fd = orb_subscribe(ORB_ID(parameter_server_set_value_response));

		if (param_set_value_rsp_fd == PX4_ERROR) {
			PX4_ERR("Subscription to parameter_server_set_value_response failed");

		} else {
			if (debug) { PX4_INFO("Subscription to parameter_server_set_value_response succeeded"); }
		}
	}

	if (send_event) {
		/*
		 * If we don't have a handle to our topic, create one now; otherwise
		 * just publish.
		 */
		if (parameter_set_value_h == nullptr) {
			parameter_set_value_h = orb_advertise(ORB_ID(parameter_server_set_value_request), &req);

		} else {
			orb_publish(ORB_ID(parameter_server_set_value_request), parameter_set_value_h, &req);
		}

		param_client_counters.set_value_sent++;

		// Wait for response
		bool updated = false;
		PX4_DEBUG("Waiting for parameter_server_set_value_response for %s", req.parameter_name);
		usleep(TIMEOUT_WAIT);
		int count = TIMEOUT_COUNT;

		while (--count) {
			(void) orb_check(param_set_value_rsp_fd, &updated);

			if (updated) {
				PX4_DEBUG("Got parameter_server_set_value_response for %s", req.parameter_name);
				struct parameter_server_set_value_response_s rsp;
				orb_copy(ORB_ID(parameter_server_set_value_response), param_set_value_rsp_fd, &rsp);
				break;
			}

			usleep(TIMEOUT_WAIT);
		}

		if (! count) {
			PX4_ERR("Timeout waiting for parameter_server_set_value_response for %s", req.parameter_name);
		}
	}
}

void
param_client_set_used(param_t param)
{
	// Notify the parameter server that this parameter has been marked as used
	if (debug) { PX4_INFO("Requesting server to mark %s as used", param_name(param)); }

	struct parameter_server_set_used_request_s req;

	req.timestamp = hrt_absolute_time();

	strncpy(req.parameter_name, param_name(param), 16);

	req.parameter_name[16] = 0;

	if (parameter_set_used_h == nullptr) {
		parameter_set_used_h = orb_advertise(ORB_ID(parameter_server_set_used_request), &req);

	} else {
		orb_publish(ORB_ID(parameter_server_set_used_request), parameter_set_used_h, &req);
	}

	param_client_counters.set_used_sent++;

	if (param_set_used_rsp_fd == PX4_ERROR) {
		if (debug) { PX4_INFO("Subscribing to parameter_server_set_used_response"); }

		param_set_used_rsp_fd = orb_subscribe(ORB_ID(parameter_server_set_used_response));

		if (param_set_used_rsp_fd == PX4_ERROR) {
			PX4_ERR("Subscription to parameter_server_set_used_response failed");

		} else {
			if (debug) { PX4_INFO("Subscription to parameter_server_set_used_response succeeded"); }
		}
	}

	// Wait for response
	if (debug) { PX4_INFO("Waiting for parameter_server_set_used_response for %s", req.parameter_name); }

	usleep(TIMEOUT_WAIT);
	bool updated = false;
	int count = TIMEOUT_COUNT;

	while (--count) {
		(void) orb_check(param_set_used_rsp_fd, &updated);

		if (updated) {
			if (debug) { PX4_INFO("Got parameter_server_set_used_response for %s", req.parameter_name); }

			struct parameter_server_set_used_response_s rsp;

			orb_copy(ORB_ID(parameter_server_set_used_response), param_set_used_rsp_fd, &rsp);

			break;
		}

		usleep(TIMEOUT_WAIT);
	}

	if (! count) {
		PX4_ERR("Timeout waiting for parameter_server_set_used_response for %s", req.parameter_name);
	}
}

void param_client_get_counters(struct param_client_counters *cnt)
{
	*cnt = param_client_counters;
}
