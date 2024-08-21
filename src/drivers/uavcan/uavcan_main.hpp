/****************************************************************************
 *
 *   Copyright (c) 2014-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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
 * @file uavcan_main.hpp
 *
 * Defines basic functinality of UAVCAN node.
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 *		 Andreas Jochum <Andreas@NicaDrone.com>
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#if defined(CONFIG_UAVCAN_OUTPUTS_CONTROLLER)
#include "actuators/esc.hpp"
#include "actuators/servo.hpp"
#endif

#if defined(CONFIG_UAVCAN_HARDPOINT_CONTROLLER)
#include "actuators/hardpoint.hpp"
#endif


#include "allocator.hpp"

#if defined(CONFIG_UAVCAN_ARMING_CONTROLLER)
#include "arming_status.hpp"
#endif

#if defined(CONFIG_UAVCAN_BEEP_CONTROLLER)
#include "beep.hpp"
#endif

#include "logmessage.hpp"

#if defined(CONFIG_UAVCAN_RGB_CONTROLLER)
#include "rgbled.hpp"
#endif

#if defined(CONFIG_UAVCAN_SAFETY_STATE_CONTROLLER)
#include "safety_state.hpp"
#endif

#include "sensors/sensor_bridge.hpp"
#include "uavcan_driver.hpp"
#include "uavcan_servers.hpp"

#include <lib/drivers/device/Device.hpp>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/perf/perf_counter.h>

#include <uavcan/helpers/heap_based_pool_allocator.hpp>
#include <uavcan/protocol/global_time_sync_master.hpp>
#include <uavcan/protocol/global_time_sync_slave.hpp>
#include <uavcan/protocol/node_info_retriever.hpp>
#include <uavcan/protocol/node_status_monitor.hpp>
#include <uavcan/protocol/param/ExecuteOpcode.hpp>
#include <uavcan/protocol/param/GetSet.hpp>
#include <uavcan/protocol/RestartNode.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/can_interface_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/uavcan_parameter_request.h>
#include <uORB/topics/uavcan_parameter_value.h>
#include <uORB/topics/vehicle_command_ack.h>

using namespace time_literals;

class UavcanNode;

/**
 * UAVCAN mixing class for ESCs.
 * It is separate from UavcanNode to have separate WorkItems and therefore allowing independent scheduling
 * (I.e. UavcanMixingInterfaceESC runs upon actuator_control updates, whereas UavcanNode runs at
 * a fixed rate or upon bus updates).
 * All work items are expected to run on the same work queue.
 */

#if defined(CONFIG_UAVCAN_OUTPUTS_CONTROLLER)
class UavcanMixingInterfaceESC : public OutputModuleInterface
{
public:
	UavcanMixingInterfaceESC(pthread_mutex_t &node_mutex, UavcanEscController &esc_controller)
		: OutputModuleInterface(MODULE_NAME "-actuators-esc", px4::wq_configurations::uavcan),
		  _node_mutex(node_mutex),
		  _esc_controller(esc_controller) {}

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

	void mixerChanged() override;

	MixingOutput &mixingOutput() { return _mixing_output; }

protected:
	void Run() override;
private:
	friend class UavcanNode;
	pthread_mutex_t &_node_mutex;
	UavcanEscController &_esc_controller;
	MixingOutput _mixing_output{"UAVCAN_EC", UavcanEscController::MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};
};

/**
 * UAVCAN mixing class for Servos.
 * It is separate from UavcanNode to have separate WorkItems and therefore allowing independent scheduling
 * (I.e. UavcanMixingInterfaceServo runs upon actuator_control updates, whereas UavcanNode runs at
 * a fixed rate or upon bus updates).
 * All work items are expected to run on the same work queue.
 */
class UavcanMixingInterfaceServo : public OutputModuleInterface
{
public:
	UavcanMixingInterfaceServo(pthread_mutex_t &node_mutex, UavcanServoController &servo_controller)
		: OutputModuleInterface(MODULE_NAME "-actuators-servo", px4::wq_configurations::uavcan),
		  _node_mutex(node_mutex),
		  _servo_controller(servo_controller) {}

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

	MixingOutput &mixingOutput() { return _mixing_output; }

protected:
	void Run() override;
private:
	friend class UavcanNode;
	pthread_mutex_t &_node_mutex;
	UavcanServoController &_servo_controller;
	MixingOutput _mixing_output{"UAVCAN_SV", UavcanServoController::MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};
};
#endif

/**
 * A UAVCAN node.
 */
class UavcanNode : public px4::ScheduledWorkItem, public ModuleParams
{
	static constexpr unsigned MaxBitRatePerSec	= 1000000;
	static constexpr unsigned bitPerFrame		= 148;
	static constexpr unsigned FramePerSecond	= MaxBitRatePerSec / bitPerFrame;
	static constexpr unsigned FramePerMSecond	= ((FramePerSecond / 1000) + 1);

	static constexpr unsigned ScheduleIntervalMs		= 3;


	/*
	 * This memory is reserved for uavcan to use for queuing CAN frames.
	 * At 1Mbit there is approximately one CAN frame every 145 uS.
	 * The number of buffers sets how long you can go without calling
	 * node_spin_xxxx. Since our task is the only one running and the
	 * driver will light the callback when there is a CAN frame we can nun with
	 * a minimum number of buffers to conserver memory. Each buffer is
	 * 32 bytes. So 5 buffers costs 160 bytes and gives us a poll rate
	 * of ~1 mS
	 *  1000000/200
	 */

	static constexpr unsigned RxQueueLenPerIface	= FramePerMSecond * ScheduleIntervalMs; // At

public:
	typedef UAVCAN_DRIVER::CanInitHelper<RxQueueLenPerIface> CanInitHelper;
	enum eServerAction : int {None, Start, Stop, CheckFW, Busy};

	UavcanNode(uavcan::ICanDriver &can_driver, uavcan::ISystemClock &system_clock);

	virtual		~UavcanNode();

	static int	start(uavcan::NodeID node_id, uint32_t bitrate);

	uavcan::Node<>	&get_node() { return _node; }

	void		print_info();

	void		shrink();

	static UavcanNode	*instance() { return _instance; }
	static int		 getHardwareVersion(uavcan::protocol::HardwareVersion &hwver);

	void requestCheckAllNodesFirmwareAndUpdate() { _check_fw = true; }

	int			 list_params(int remote_node_id);
	int			 save_params(int remote_node_id);
	int			 set_param(int remote_node_id, const char *name, char *value);
	int			 get_param(int remote_node_id, const char *name);
	int			 reset_node(int remote_node_id);

	static void busevent_signal_trampoline();

private:
	void Run() override;

	void		fill_node_info();
	int		init(uavcan::NodeID node_id, UAVCAN_DRIVER::BusEvent &bus_events);

	int		print_params(uavcan::protocol::param::GetSet::Response &resp);
	int		get_set_param(int nodeid, const char *name, uavcan::protocol::param::GetSet::Request &req);
	void 		update_params();

	void		set_setget_response(uavcan::protocol::param::GetSet::Response *resp) { _setget_response = resp; }
	void		free_setget_response(void) { _setget_response = nullptr; }

	px4::atomic_bool	_task_should_exit{false};	///< flag to indicate to tear down the CAN driver

	unsigned		_output_count{0};		///< number of actuators currently available

	static UavcanNode	*_instance;			///< singleton pointer

	uavcan_node::Allocator	 _pool_allocator;

	bool                    _node_init{false};
	uavcan::Node<>			_node;				///< library instance
	pthread_mutex_t			_node_mutex;

#if defined(CONFIG_UAVCAN_ARMING_CONTROLLER)
	UavcanArmingStatus		_arming_status_controller;
#endif
#if defined(CONFIG_UAVCAN_BEEP_CONTROLLER)
	UavcanBeepController		_beep_controller;
#endif
#if defined(CONFIG_UAVCAN_OUTPUTS_CONTROLLER)
	UavcanEscController		_esc_controller;
	UavcanMixingInterfaceESC 	_mixing_interface_esc{_node_mutex, _esc_controller};

	UavcanServoController		_servo_controller;
	UavcanMixingInterfaceServo 	_mixing_interface_servo{_node_mutex, _servo_controller};
#endif
#if defined(CONFIG_UAVCAN_HARDPOINT_CONTROLLER)
	UavcanHardpointController	_hardpoint_controller;
#endif
#if defined(CONFIG_UAVCAN_SAFETY_STATE_CONTROLLER)
	UavcanSafetyState         	_safety_state_controller;
#endif
#if defined(CONFIG_UAVCAN_RGB_CONTROLLER)
	UavcanRGBController             _rgbled_controller;
#endif

	UavcanLogMessage                _log_message_controller;

	uavcan::GlobalTimeSyncMaster	_time_sync_master;
	uavcan::GlobalTimeSyncSlave	_time_sync_slave;
	uavcan::NodeStatusMonitor	_node_status_monitor;

	uavcan::NodeInfoRetriever   _node_info_retriever;

	List<IUavcanSensorBridge *>	_sensor_bridges;		///< List of active sensor bridges

	perf_counter_t			_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")};
	perf_counter_t			_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": cycle interval")};

	void handle_time_sync(const uavcan::TimerEvent &);

	typedef uavcan::MethodBinder<UavcanNode *, void (UavcanNode::*)(const uavcan::TimerEvent &)> TimerCallback;
	uavcan::TimerEventForwarder<TimerCallback> _master_timer;

	bool				_callback_success{false};

	uavcan::protocol::param::GetSet::Response *_setget_response{nullptr};

	typedef uavcan::MethodBinder<UavcanNode *,
		void (UavcanNode::*)(const uavcan::ServiceCallResult<uavcan::protocol::param::GetSet> &)> GetSetCallback;
	typedef uavcan::MethodBinder<UavcanNode *,
		void (UavcanNode::*)(const uavcan::ServiceCallResult<uavcan::protocol::param::ExecuteOpcode> &)> ExecuteOpcodeCallback;
	typedef uavcan::MethodBinder<UavcanNode *,
		void (UavcanNode::*)(const uavcan::ServiceCallResult<uavcan::protocol::RestartNode> &)> RestartNodeCallback;

	void cb_setget(const uavcan::ServiceCallResult<uavcan::protocol::param::GetSet> &result);

	uORB::SubscriptionInterval	_parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vcmd_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _param_request_sub{ORB_ID(uavcan_parameter_request)};

	uORB::Publication<uavcan_parameter_value_s> _param_response_pub{ORB_ID(uavcan_parameter_value)};
	uORB::Publication<vehicle_command_ack_s>	_command_ack_pub{ORB_ID(vehicle_command_ack)};
	uORB::PublicationMulti<can_interface_status_s> _can_status_pub{ORB_ID(can_interface_status)};

	hrt_abstime _last_can_status_pub{0};
	orb_advert_t _can_status_pub_handles[UAVCAN_NUM_IFACES] = {nullptr};

	/*
	 * The MAVLink parameter bridge needs to know the maximum parameter index
	 * of each node so that clients can determine when parameter listings have
	 * finished. We do that by querying a node's entire parameter set whenever
	 * a parameter message is received for a node with a zero _param_count,
	 * and storing the count here. If a node doesn't respond, or doesn't have
	 * any parameters, its count will stay at zero and we'll try to query the
	 * set again next time.
	 *
	 * The node's UAVCAN ID is used as the index into the _param_counts array.
	 */
	uint8_t _param_counts[128] {};
	bool _count_in_progress{false};
	uint8_t _count_index{0};

	bool _param_in_progress{false};
	uint8_t _param_index{0};
	bool _param_list_in_progress{false};
	bool _param_list_all_nodes{false};
	uint8_t _param_list_node_id{1};

	uint32_t _param_dirty_bitmap[4] {};
	uint8_t _param_save_opcode{0};

	bool _cmd_in_progress{false};

	void cb_getset(const uavcan::ServiceCallResult<uavcan::protocol::param::GetSet> &result);
	void cb_count(const uavcan::ServiceCallResult<uavcan::protocol::param::GetSet> &result);
	void cb_opcode(const uavcan::ServiceCallResult<uavcan::protocol::param::ExecuteOpcode> &result);
	void cb_restart(const uavcan::ServiceCallResult<uavcan::protocol::RestartNode> &result);

	uavcan::ServiceClient<uavcan::protocol::param::GetSet, GetSetCallback> _param_getset_client;
	uavcan::ServiceClient<uavcan::protocol::param::ExecuteOpcode, ExecuteOpcodeCallback> _param_opcode_client;
	uavcan::ServiceClient<uavcan::protocol::RestartNode, RestartNodeCallback> _param_restartnode_client;
	void param_count(uavcan::NodeID node_id);
	void param_opcode(uavcan::NodeID node_id);

	uint8_t get_next_active_node_id(uint8_t base);
	uint8_t get_next_dirty_node_id(uint8_t base);
	void set_node_params_dirty(uint8_t node_id) { _param_dirty_bitmap[node_id >> 5] |= 1 << (node_id & 31); }
	void clear_node_params_dirty(uint8_t node_id) { _param_dirty_bitmap[node_id >> 5] &= ~(1 << (node_id & 31)); }
	bool are_node_params_dirty(uint8_t node_id) const { return bool((_param_dirty_bitmap[node_id >> 5] >> (node_id & 31)) & 1); }

	bool _check_fw{false};

	UavcanServers   *_servers{nullptr};
};
