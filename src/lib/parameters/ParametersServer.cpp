/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "ParametersServer.hpp"

#include <px4_platform_common/atomic.h>

#include "param.h"

static ParametersServer *_param_autosave{nullptr};

float ParametersServer::last_autosave_elapsed() const
{
	return hrt_elapsed_time_atomic(&_last_autosave_timestamp) * 1e-6f;
}

void ParametersServer::AutoSave()
{
	if (_param_autosave == nullptr) {
		return;
	}

	// wait at least 300ms before saving, because:
	// - tasks often call param_set() for multiple params, so this avoids unnecessary save calls
	// - the logger stores changed params. He gets notified on a param change via uORB and then
	//   looks at all unsaved params.
	float delay = 0.3f;

	static constexpr float rate_limit = 2.0f; // rate-limit saving to 2 seconds
	const float last_save_elapsed = _param_autosave->last_autosave_elapsed();

	if (last_save_elapsed < rate_limit && rate_limit > last_save_elapsed + delay) {
		delay = rate_limit - last_save_elapsed;
	}

	uint64_t delay_us = delay * 1e6f;
	_param_autosave->ScheduleDelayed(delay_us);
}

bool ParametersServer::EnableAutoSave(bool enable)
{
	if (enable) {
		if (_param_autosave == nullptr) {
			_param_autosave = new ParametersServer();

			if (_param_autosave == nullptr) {
				PX4_ERR("ParametersServer alloc failed");
				return false;
			}
		}

	} else {
		// TODO: how to prevent delete if currently running?
		delete _param_autosave;
		_param_autosave = nullptr;
	}

	return true;
}

void ParametersServer::print_status()
{
	if (_param_autosave) {
		PX4_INFO("last auto save: %.3f seconds ago", (double)_param_autosave->last_autosave_elapsed());

	} else {
		PX4_INFO("auto save: off");
	}
}

void ParametersServer::Run()
{
	_last_autosave_timestamp = hrt_absolute_time();
	int ret = param_save_default();

	if (ret != 0) {
		PX4_ERR("param auto save failed (%i)", ret);
	}

	// Check for parameter requests (get/set/list)
	if (_param_request_sub.updated()) {
		parameter_request_s request{};
		param_request_sub.copy(&request);

		/*
		 * We know how many parameters are exposed by this node, so
		 * process the request.
		 */
		if (request.message_type == parameter_request_s::MESSAGE_TYPE_PARAM_REQUEST_READ) {
			parameter_value_s parameter_value{};

			if (request.param_index >= 0) {
				param_for_index(request.param_index);

			} else {
				param_find(&request.param_id);
			}

			int call_res = _param_getset_client.call(request.node_id, req);

			if (call_res < 0) {
				PX4_ERR("couldn't send GetSet: %d", call_res);

			} else {
				_param_in_progress = true;
				_param_index = request.param_index;
			}

		} else if (request.message_type == parameter_request_s::MESSAGE_TYPE_PARAM_SET) {
			uavcan::protocol::param::GetSet::Request req;

			if (request.param_index >= 0) {
				req.index = request.param_index;

			} else {
				req.name = (char *)request.param_id;
			}

			if (request.param_type == parameter_request_s::TYPE_REAL32) {
				req.value.to<uavcan::protocol::param::Value::Tag::real_value>() = request.real_value;

			} else if (request.param_type == parameter_request_s::TYPE_UINT8) {
				req.value.to<uavcan::protocol::param::Value::Tag::boolean_value>() = request.int_value;

			} else {
				req.value.to<uavcan::protocol::param::Value::Tag::integer_value>() = request.int_value;
			}

		} else if (request.message_type == parameter_request_s::MESSAGE_TYPE_PARAM_REQUEST_LIST) {
			// This triggers the _param_list_in_progress case below.
			_param_index = 0;
			_param_list_in_progress = true;
			_param_list_node_id = request.node_id;
			_param_list_all_nodes = false;

			PX4_DEBUG("starting component-specific param list");

		} else if (request.node_id == parameter_request_s::NODE_ID_ALL) {
			if (request.message_type == parameter_request_s::MESSAGE_TYPE_PARAM_REQUEST_LIST) {
				/*
				 * This triggers the _param_list_in_progress case below,
				 * but additionally iterates over all active nodes.
				 */
				_param_index = 0;
				_param_list_in_progress = true;
				_param_list_node_id = get_next_active_node_id(0);
				_param_list_all_nodes = true;

				PX4_DEBUG("starting global param list with node %hhu", _param_list_node_id);

				if (_param_counts[_param_list_node_id] == 0) {
					param_count(_param_list_node_id);
				}
			}
		}
	}
}
