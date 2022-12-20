/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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
 * @file parameters.cpp
 *
 * Global parameter store.
 *
 * Note that it might make sense to convert this into a driver.  That would
 * offer some interesting options regarding state for e.g. ORB advertisements
 * and background parameter saving.
 */

#define PARAM_IMPLEMENTATION

#include "param.h"

#if defined(PX4_PARAM_CLIENT_SRV)

#include <uORB/SubscriptionBlocking.hpp>
#include <uORB/Publication.hpp>

#include <uORB/topics/srv_parameter_get_request.h>
#include <uORB/topics/srv_parameter_get_response.h>

#include <uORB/topics/srv_parameter_set_request.h>
#include <uORB/topics/srv_parameter_set_response.h>

using namespace time_literals;

param_t param_find(const char *name)
{
	// request
	uORB::Publication<srv_parameter_get_request_s> request_pub{ORB_ID(srv_parameter_get_request)};
	uORB::SubscriptionBlocking<srv_parameter_get_response_s> response_sub{ORB_ID(srv_parameter_get_response)};
	srv_parameter_get_request_s request{};
	request.index = -1;
	memcpy(request.name, name, 16);
	request.timestamp = hrt_absolute_time();
	request_pub.publish(request);

	// response
	while ((hrt_elapsed_time(&request.timestamp) < 50_ms) || response_sub.updated()) {
		const unsigned last_generation = response_sub.get_last_generation();
		srv_parameter_get_response_s response{};

		if (response_sub.updateBlocking(response, 1'000)) {

			if (response_sub.get_last_generation() != last_generation + 1) {
				PX4_ERR("param_find: missed srv_parameter_get_response, generation %d -> %d", last_generation,
					response_sub.get_last_generation());
			}

			if ((request.timestamp == response.timestamp_requested)
			    && (strncmp(request.name, response.parameter.name, sizeof(request.name)) == 0)) {

				return response.parameter.index;
			}
		}
	}

	return PARAM_INVALID;
}

param_t param_find_no_notification(const char *name)
{
	return param_find(name);
}

int param_get(param_t param, void *val)
{
	// request
	uORB::Publication<srv_parameter_get_request_s> request_pub{ORB_ID(srv_parameter_get_request)};
	uORB::SubscriptionBlocking<srv_parameter_get_response_s> response_sub{ORB_ID(srv_parameter_get_response)};
	srv_parameter_get_request_s request{};
	request.index = param;
	request.timestamp = hrt_absolute_time();
	request_pub.publish(request);

	// response
	while ((hrt_elapsed_time(&request.timestamp) < 50_ms) || response_sub.updated()) {
		const unsigned last_generation = response_sub.get_last_generation();
		srv_parameter_get_response_s response{};

		if (response_sub.updateBlocking(response, 1'000)) {

			if (response_sub.get_last_generation() != last_generation + 1) {
				PX4_ERR("param_get: missed srv_parameter_get_response, generation %d -> %d", last_generation,
					response_sub.get_last_generation());
			}

			if ((request.timestamp == response.timestamp_requested) && (request.index == response.parameter.index)) {
				switch (response.parameter.type) {
				case parameter_s::TYPE_INT32:
					memcpy(val, &response.parameter.int32_value, sizeof(int32_t));
					break;

				case parameter_s::TYPE_FLOAT32:
					memcpy(val, &response.parameter.float32_value, sizeof(float));
					break;
				}

				if (response.result == srv_parameter_get_response_s::RESULT_GET_SUCCESS) {
					return 0;
				}

				return -1;
			}
		}
	}

	return -1;
}

int param_set(param_t param, const void *val)
{
	// request
	uORB::Publication<srv_parameter_set_request_s> request_pub{ORB_ID(srv_parameter_set_request)};
	uORB::SubscriptionBlocking<srv_parameter_set_response_s> response_sub{ORB_ID(srv_parameter_set_response)};
	srv_parameter_set_request_s request{};
	request.parameter.index = param;
	memcpy(&request.parameter.int32_value, val, sizeof(int32_t));
	memcpy(&request.parameter.float32_value, val, sizeof(float));
	request.timestamp = hrt_absolute_time();
	request_pub.publish(request);

	// response
	while ((hrt_elapsed_time(&request.timestamp) < 50_ms) || response_sub.updated()) {
		const unsigned last_generation = response_sub.get_last_generation();
		srv_parameter_set_response_s response{};

		if (response_sub.updateBlocking(response, 1'000)) {

			if (response_sub.get_last_generation() != last_generation + 1) {
				PX4_ERR("param_set: missed srv_parameter_set_response, generation %d -> %d", last_generation,
					response_sub.get_last_generation());
			}

			if ((request.timestamp == response.timestamp_requested) && (request.parameter.index == response.parameter.index)) {
				if (response.result == srv_parameter_set_response_s::RESULT_SET_SUCCESS) {
					return 0;
				}

				PX4_ERR("param_set %d failed", param);
				return -1;
			}
		}
	}

	PX4_ERR("param_set %d failed", param);
	return -1;
}

int param_set_no_notification(param_t param, const void *val)
{
	return param_set(param, val);
}

#endif // PX4_PARAM_CLIENT_SRV
