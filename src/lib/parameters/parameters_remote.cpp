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

#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>

// uORB topics needed to keep parameter server and client in sync
#include <uORB/topics/parameter_reset_request.h>
#include <uORB/topics/parameter_set_used_request.h>
#include <uORB/topics/parameter_set_value_request.h>
#include <uORB/topics/parameter_set_value_response.h>

static px4_task_t sync_thread_tid;
static const char *sync_thread_name = "param_remote_sync";

static int remote_sync_thread(int argc, char *argv[]) {
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
			orb_copy(ORB_ID(parameter_reset_request), _reset_req_fd, &_reset_request);

			// PX4_INFO("Got parameter_reset_request for %s", param_name(_reset_request.parameter_index));

			// param_server_counters.set_used_received++;

			if (_reset_request.reset_all) {
				param_reset_all();

			} else {
				param_reset_no_notification(_reset_request.parameter_index);

			}

		} else if (fds[1].revents & POLLIN) {
			orb_copy(ORB_ID(parameter_primary_set_value_request), _set_value_req_fd, &_set_value_request);

			// PX4_INFO("Got parameter_remote_set_value_request for %s", param_name(_set_value_request.parameter_index));

			// param_server_counters.set_value_received++;

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
				// PX4_INFO("Sending set value response for %s", param_name(_set_value_request.parameter_index));
				orb_publish(ORB_ID(parameter_remote_set_value_response), _set_value_rsp_h, &_set_value_response);
			}
		}
	}

    return 0;
}

void param_remote_init() {

	sync_thread_tid = px4_task_spawn_cmd(sync_thread_name,
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_PARAMS,
					     (1024 * 4),
					     remote_sync_thread,
					     NULL);

}
