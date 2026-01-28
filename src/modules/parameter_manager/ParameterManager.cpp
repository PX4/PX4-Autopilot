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

/**
 * @file ParameterManager.cpp
 *
 * Parameter Manager module implementation.
 * Provides a uORB-based parameter microservice for external systems (ROS2/DDS).
 */

#include "ParameterManager.hpp"

#include <string.h>

ParameterManager::ParameterManager() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

ParameterManager::~ParameterManager()
{
	ScheduleClear();
}

bool ParameterManager::init()
{
	ScheduleOnInterval(POLL_INTERVAL_US);
	return true;
}

void ParameterManager::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// Check for parameter name lookup requests
	if (_name_request_sub.updated()) {
		handle_name_request();
	}

	// Check for parameter list requests
	if (_list_request_sub.updated()) {
		handle_list_request();
	}

	// Check for parameter set value requests
	if (_set_value_request_sub.updated()) {
		handle_set_value_request();
	}
}

void ParameterManager::handle_name_request()
{
	parameter_name_request_s request;

	if (_name_request_sub.copy(&request)) {
		parameter_name_response_s response{};
		response.timestamp = hrt_absolute_time();
		response.request_timestamp = request.timestamp;

		// Ensure null-termination of the parameter name
		char param_name[sizeof(request.param_name) + 1];
		memcpy(param_name, request.param_name, sizeof(request.param_name));
		param_name[sizeof(request.param_name)] = '\0';

		// Copy the name back to the response
		strncpy(response.param_name, param_name, sizeof(response.param_name));

		// Look up the parameter by name using binary search
		param_t param = param_find_no_notification(param_name);

		if (param != PARAM_INVALID) {
			response.valid = true;
			response.parameter_index = param;
			response.param_type = param_type(param);

			// Get the current value
			get_param_value(param, response.int_value, response.float_value);

			// Mark the parameter as used (this is what MAVLink does)
			param_set_used(param);

		} else {
			response.valid = false;
			response.parameter_index = PARAM_INVALID;
			response.param_type = parameter_name_response_s::TYPE_UNKNOWN;
			response.int_value = 0;
			response.float_value = 0.0f;
		}

		_name_response_pub.publish(response);
	}
}

void ParameterManager::handle_list_request()
{
	parameter_list_request_s request;

	if (_list_request_sub.copy(&request)) {
		parameter_list_response_s response{};
		response.timestamp = hrt_absolute_time();
		response.request_timestamp = request.timestamp;
		response.start_index = request.start_index;

		// Clamp count to maximum
		uint8_t count = (request.count > parameter_list_request_s::MAX_COUNT) ?
				parameter_list_request_s::MAX_COUNT : request.count;

		// Determine total count based on whether we want all or only used parameters
		uint16_t total_count;

		if (request.used_only) {
			total_count = param_count_used();

		} else {
			total_count = param_count();
		}

		response.total_count = total_count;
		response.returned_count = 0;

		// Initialize arrays
		memset(response.parameter_indices, 0, sizeof(response.parameter_indices));
		memset(response.param_names, 0, sizeof(response.param_names));
		memset(response.param_types, 0, sizeof(response.param_types));
		memset(response.int_values, 0, sizeof(response.int_values));
		memset(response.float_values, 0, sizeof(response.float_values));

		// Iterate through parameters starting at the requested index
		uint16_t current_index = request.start_index;
		uint8_t filled = 0;

		while (filled < count && current_index < total_count) {
			param_t param;

			if (request.used_only) {
				param = param_for_used_index(current_index);

			} else {
				param = param_for_index(current_index);
			}

			if (param != PARAM_INVALID) {
				// Store the parameter index
				response.parameter_indices[filled] = param;

				// Store the parameter name (packed into param_names array)
				const char *name = param_name(param);

				if (name != nullptr) {
					size_t name_offset = filled * parameter_list_response_s::PARAM_NAME_LEN;
					strncpy(&response.param_names[name_offset], name,
						parameter_list_response_s::PARAM_NAME_LEN - 1);
					response.param_names[name_offset + parameter_list_response_s::PARAM_NAME_LEN - 1] = '\0';
				}

				// Store the parameter type
				response.param_types[filled] = param_type(param);

				// Store the parameter value
				get_param_value(param, response.int_values[filled], response.float_values[filled]);

				filled++;
			}

			current_index++;
		}

		response.returned_count = filled;
		_list_response_pub.publish(response);
	}
}

void ParameterManager::handle_set_value_request()
{
	parameter_set_value_request_s request;

	if (_set_value_request_sub.copy(&request)) {
		parameter_set_value_response_s response{};
		response.timestamp = hrt_absolute_time();
		response.request_timestamp = request.timestamp;
		response.parameter_index = request.parameter_index;

		param_t param = request.parameter_index;

		if (param != PARAM_INVALID && param < param_count()) {
			param_type_t type = param_type(param);

			switch (type) {
			case PARAM_TYPE_INT32:
				param_set(param, &request.int_value);
				break;

			case PARAM_TYPE_FLOAT:
				param_set(param, &request.float_value);
				break;

			default:
				// Unknown type, do nothing
				break;
			}
		}

		_set_value_response_pub.publish(response);
	}
}

bool ParameterManager::get_param_value(param_t param, int32_t &int_value, float &float_value)
{
	if (param == PARAM_INVALID) {
		return false;
	}

	param_type_t type = param_type(param);

	switch (type) {
	case PARAM_TYPE_INT32:
		param_get(param, &int_value);
		float_value = 0.0f;
		return true;

	case PARAM_TYPE_FLOAT:
		param_get(param, &float_value);
		int_value = 0;
		return true;

	default:
		int_value = 0;
		float_value = 0.0f;
		return false;
	}
}

int ParameterManager::task_spawn(int argc, char *argv[])
{
	ParameterManager *instance = new ParameterManager();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int ParameterManager::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ParameterManager::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Parameter Manager provides a uORB-based parameter microservice that enables
external systems (such as ROS2 via DDS) to look up parameters by name and
enumerate available parameters.

This module addresses the limitation where parameter indices are build-specific,
making it difficult for external systems to reliably access parameters without
knowing the exact parameter name-to-index mapping.

### Features

- **Parameter name lookup**: Resolve parameter names to indices, types, and values
- **Parameter enumeration**: List all parameters or only "used" parameters
- **Parameter set by index**: Set parameter values with acknowledgment

### uORB Topics

Subscriptions:
- `parameter_name_request`: Request to look up a parameter by name
- `parameter_list_request`: Request to enumerate parameters
- `parameter_set_value_request`: Request to set a parameter value

Publications:
- `parameter_name_response`: Response with parameter index, type, and value
- `parameter_list_response`: Response with paginated parameter list
- `parameter_set_value_response`: Acknowledgment of parameter set operation

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("parameter_manager", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int parameter_manager_main(int argc, char *argv[])
{
	return ParameterManager::main(argc, argv);
}
