/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file FailureInjectionManager.hpp
 *
 * The failure injection manager: the single subscriber to vehicle_command for
 * MAV_CMD_INJECT_FAILURE. It maintains the active failure table and republishes
 * the failure_injection topic only when the configuration changes, so a burst
 * of commands on vehicle_command cannot propagate to the consumers that apply
 * the failures (the bulkhead). It also produces the central vehicle_command_ack.
 */

#pragma once

#include "FailureTable.hpp"

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/failure_injection.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

class FailureInjectionManager : public ModuleBase, public ModuleParams, public px4::WorkItem
{
public:
	FailureInjectionManager();
	~FailureInjectionManager() override = default;

	static Descriptor desc;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	void handleCommand(const vehicle_command_s &cmd);
	void publishAck(const vehicle_command_s &cmd, uint8_t result);

	void evaluateRcInjection();

	uORB::SubscriptionCallbackWorkItem _vehicle_command_sub{this, ORB_ID(vehicle_command)};
	uORB::SubscriptionCallbackWorkItem _manual_control_setpoint_sub{this, ORB_ID(manual_control_setpoint)};

	uORB::Publication<failure_injection_s>  _failure_injection_pub{ORB_ID(failure_injection)};
	uORB::Publication<vehicle_command_ack_s> _command_ack_pub{ORB_ID(vehicle_command_ack)};

	failure_injection::FailureTable _table;

	bool    _rc_active{false};
	uint8_t _rc_active_unit{0};
	uint8_t _rc_active_instance{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_FAIL_RC_SRC>) _param_sys_fail_rc_src,
		(ParamInt<px4::params::SYS_FAIL_RC_UNIT>) _param_sys_fail_rc_unit,
		(ParamInt<px4::params::SYS_FAIL_RC_MODE>) _param_sys_fail_rc_mode,
		(ParamInt<px4::params::SYS_FAIL_RC_INST>) _param_sys_fail_rc_inst
	)
};
