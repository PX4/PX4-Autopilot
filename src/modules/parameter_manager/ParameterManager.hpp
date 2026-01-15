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
 * @file ParameterManager.hpp
 *
 * Parameter Manager module providing a uORB-based parameter microservice.
 * This enables external systems (ROS2/DDS) to look up parameters by name
 * and enumerate available parameters.
 *
 * @author PX4 Development Team
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_name_request.h>
#include <uORB/topics/parameter_name_response.h>
#include <uORB/topics/parameter_list_request.h>
#include <uORB/topics/parameter_list_response.h>
#include <uORB/topics/parameter_set_value_request.h>
#include <uORB/topics/parameter_set_value_response.h>
#include <uORB/topics/parameter_update.h>

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

class ParameterManager : public ModuleBase<ParameterManager>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	ParameterManager();
	~ParameterManager() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	/**
	 * Handle a parameter name lookup request.
	 * Resolves a parameter name to its index, type, and current value.
	 */
	void handle_name_request();

	/**
	 * Handle a parameter list request.
	 * Returns a paginated list of parameters.
	 */
	void handle_list_request();

	/**
	 * Handle a parameter set value request (by index).
	 * Sets a parameter value and publishes a response.
	 */
	void handle_set_value_request();

	/**
	 * Get the current value of a parameter and populate the response fields.
	 * @param param The parameter handle
	 * @param int_value Output: integer value if param is INT32
	 * @param float_value Output: float value if param is FLOAT
	 * @return true if successful
	 */
	bool get_param_value(param_t param, int32_t &int_value, float &float_value);

	// Subscriptions
	uORB::Subscription _name_request_sub{ORB_ID(parameter_name_request)};
	uORB::Subscription _list_request_sub{ORB_ID(parameter_list_request)};
	uORB::Subscription _set_value_request_sub{ORB_ID(parameter_set_value_request)};

	// Publications
	uORB::Publication<parameter_name_response_s> _name_response_pub{ORB_ID(parameter_name_response)};
	uORB::Publication<parameter_list_response_s> _list_response_pub{ORB_ID(parameter_list_response)};
	uORB::Publication<parameter_set_value_response_s> _set_value_response_pub{ORB_ID(parameter_set_value_response)};

	// Polling interval
	static constexpr uint32_t POLL_INTERVAL_US = 10000; // 10ms = 100Hz
};
