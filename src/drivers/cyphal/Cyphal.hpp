/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/mixer_module/mixer_module.hpp>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gps.h>

#include <canard.h>

#include "CanardInterface.hpp"

#include "Publishers/Publisher.hpp"
#include "Publishers/uORB/uorb_publisher.hpp"

#ifdef CONFIG_CYPHAL_NODE_MANAGER
#include "NodeManager.hpp"
#endif

#ifdef CONFIG_CYPHAL_NODE_CLIENT
#include "NodeClient.hpp"
#endif

#include "PublicationManager.hpp"
#include "SubscriptionManager.hpp"

#include "Actuators/EscClient.hpp" /// TODO: Add EscServer.hpp for node-side service

/**
 * UAVCAN mixing class.
 * It is separate from CyphalNode to have 2 WorkItems and therefore allowing independent scheduling
 * (I.e. UavcanMixingInterface runs upon actuator_control updates, whereas CyphalNode runs at
 * a fixed rate or upon bus updates).
 * Both work items are expected to run on the same work queue.
 */
class UavcanMixingInterface : public OutputModuleInterface
{
public:
	UavcanMixingInterface(pthread_mutex_t &node_mutex,
			      PublicationManager &pub_manager)
		: OutputModuleInterface(MODULE_NAME "-actuators", px4::wq_configurations::uavcan),
		  _node_mutex(node_mutex),
		  _pub_manager(pub_manager) {}

	bool updateOutputs(uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

	void printInfo() { _mixing_output.printStatus(); }

	MixingOutput &mixingOutput() { return _mixing_output; }

	/// For use with PR-16808 once merged
	// const char *get_param_prefix() override { return "UCAN1_ACT"; }

protected:
	void Run() override;
private:
	friend class CyphalNode;
	pthread_mutex_t &_node_mutex;
	PublicationManager &_pub_manager;
	MixingOutput _mixing_output{"UCAN1_ESC", MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};
};

class CyphalNode : public ModuleParams, public px4::ScheduledWorkItem
{

	/*
	 * Base interval, has to be complemented with events from the CAN driver
	 * and uORB topics sending data, to decrease response time.
	 */
	static constexpr unsigned ScheduleIntervalMs = 3;

public:

	CyphalNode(uint32_t node_id, size_t capacity, size_t mtu_bytes);
	~CyphalNode() override;

	static int start(uint32_t node_id, uint32_t bitrate);

	void print_info();

	static CyphalNode *instance() { return _instance; }

	/* The bit rate that can be passed back to the bootloader */
	int32_t active_bitrate{0};

private:
	void init();
	void Run() override;
	void fill_node_info();

	// Sends a heartbeat at 1s intervals
	void sendHeartbeat();

	// Sends a port.List at 3s intervals
	void sendPortList();

	px4::atomic_bool _task_should_exit{false};	///< flag to indicate to tear down the CAN driver

	bool _initialized{false};		///< number of actuators currently available

	static CyphalNode *_instance;

	CanardHandle _canard_handle;

	pthread_mutex_t _node_mutex;

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")};
	perf_counter_t _interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": cycle interval")};

	// uavcan::node::Heartbeat_1_0
	uint8_t _uavcan_node_heartbeat_buffer[uavcan_node_Heartbeat_1_0_EXTENT_BYTES_];
	hrt_abstime _uavcan_node_heartbeat_last{0};
	CanardTransferID _uavcan_node_heartbeat_transfer_id{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CYPHAL_ENABLE>) _param_uavcan_v1_enable,
		(ParamInt<px4::params::CYPHAL_ID>) _param_uavcan_v1_id,
		(ParamInt<px4::params::CYPHAL_BAUD>) _param_uavcan_v1_baud
	)

	UavcanParamManager _param_manager;

#ifdef CONFIG_CYPHAL_NODE_MANAGER
	NodeManager _node_manager {_canard_handle, _param_manager};
#endif

#ifdef CONFIG_CYPHAL_NODE_CLIENT
	NodeClient *_node_client {nullptr};
#endif

	PublicationManager _pub_manager {_canard_handle, _param_manager};
	SubscriptionManager _sub_manager {_canard_handle, _param_manager};

	UavcanMixingInterface _mixing_output {_node_mutex, _pub_manager};

};
