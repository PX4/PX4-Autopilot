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

#include "Cyphal.hpp"

#include <lib/geo/geo.h>
#include <lib/version/version.h>


#ifdef CONFIG_CYPHAL_APP_DESCRIPTOR
#include "boot_app_shared.h"
/*
 * This is the AppImageDescriptor used
 * by the make_can_boot_descriptor.py tool to set
 * the application image's descriptor so that the
 * uavcan bootloader has the ability to validate the
 * image crc, size etc of this application
*/
boot_app_shared_section app_descriptor_t AppDescriptor = {
	.signature = APP_DESCRIPTOR_SIGNATURE,
	.image_crc = 0,
	.image_size = 0,
	.git_hash  = 0,
	.major_version = APP_VERSION_MAJOR,
	.minor_version = APP_VERSION_MINOR,
	.board_id = HW_VERSION_MAJOR << 8 | HW_VERSION_MINOR,
	.reserved = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff }
};
#endif

using namespace time_literals;

CyphalNode *CyphalNode::_instance;

CyphalNode::CyphalNode(uint32_t node_id, size_t capacity, size_t mtu_bytes) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan),
	_canard_handle(node_id, capacity, mtu_bytes)
{
	pthread_mutex_init(&_node_mutex, nullptr);


#ifdef CONFIG_CYPHAL_NODE_MANAGER
	_node_manager.subscribe();
#endif

#ifdef CONFIG_CYPHAL_NODE_CLIENT
	_node_client = new NodeClient(_canard_handle, _param_manager);

	_node_client->subscribe();
#endif

	_pub_manager.updateParams();

	_sub_manager.subscribe();
}

CyphalNode::~CyphalNode()
{
	if (_instance) {
		/* tell the task we want it to go away */
		_task_should_exit.store(true);
		ScheduleNow();

		unsigned i = 1000;

		do {
			/* Wait for it to exit or timeout */
			usleep(5000);

			if (--i == 0) {
				PX4_ERR("Failed to Stop Task - reboot needed");
				break;
			}

		} while (_instance);
	}

	perf_free(_cycle_perf);
	perf_free(_interval_perf);
}

int CyphalNode::start(uint32_t node_id, uint32_t bitrate)
{
	if (_instance != nullptr) {
		PX4_WARN("Already started");
		return -1;
	}

	bool can_fd = false;

	if (can_fd) {
		_instance = new CyphalNode(node_id, 8, CANARD_MTU_CAN_FD);

	} else {
		_instance = new CyphalNode(node_id, 64, CANARD_MTU_CAN_CLASSIC);
	}

	if (_instance == nullptr) {
		PX4_ERR("Out of memory");
		return -1;
	}

	// Keep the bit rate for reboots on BenginFirmware updates
	_instance->active_bitrate = bitrate;

	_instance->ScheduleOnInterval(ScheduleIntervalMs * 1000);

	return PX4_OK;
}

void CyphalNode::init()
{
	// interface init
	if (_canard_handle.init()) {
		_initialized = true;
	}

}

void CyphalNode::Run()
{
	pthread_mutex_lock(&_node_mutex);

	if (_instance != nullptr && !_initialized) {
		init();

		// return early if still not initialized
		if (!_initialized) {
			pthread_mutex_unlock(&_node_mutex);
			return;
		}
	}

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();

		// Update dynamic pub/sub objects based on Port ID params
		_pub_manager.updateParams();
		_sub_manager.updateParams();

		_mixing_output.updateParams();
	}

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	if (_canard_handle.node_id() != CANARD_NODE_ID_UNSET) {
		// send uavcan::node::Heartbeat_1_0 @ 1 Hz
		sendHeartbeat();

		// Check all publishers
		_pub_manager.update();

#ifdef CONFIG_CYPHAL_NODE_MANAGER
		_node_manager.update();
#endif
	}

#ifdef CONFIG_CYPHAL_NODE_CLIENT

	else if (_node_client != nullptr) {
		if (_canard_handle.node_id() == CANARD_NODE_ID_UNSET) {
			_node_client->update();

		} else {
			delete _node_client;
		}
	}

#endif

	_canard_handle.transmit();

	_canard_handle.receive();

	// Pop canardTx queue to send out responses to requests
	_canard_handle.transmit();

	perf_end(_cycle_perf);

	if (_instance && _task_should_exit.load()) {
		ScheduleClear();

		if (_initialized) {
			_initialized = false;
		}

		_instance = nullptr;
	}

	pthread_mutex_unlock(&_node_mutex);
}

template <typename As, typename F>
static void traverseTree(const CanardTreeNode *const root, const F &op)  // NOLINT this recursion is tightly bounded
{
	if (root != nullptr) {
		traverseTree<As, F>(root->lr[0], op);
		op(static_cast<const As *>(static_cast<const void *>(root)));
		traverseTree<As, F>(root->lr[1], op);
	}
}

void CyphalNode::print_info()
{
	pthread_mutex_lock(&_node_mutex);

	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);

	O1HeapDiagnostics heap_diagnostics = _canard_handle.getO1HeapDiagnostics();

	PX4_INFO("Heap status %zu/%zu Peak alloc %zu Peak req %zu OOM count %" PRIu64,
		 heap_diagnostics.allocated, heap_diagnostics.capacity,
		 heap_diagnostics.peak_allocated, heap_diagnostics.peak_request_size,
		 heap_diagnostics.oom_count);

	_pub_manager.printInfo();

	traverseTree<CanardRxSubscription>(_canard_handle.getRxSubscriptions(CanardTransferKindMessage),
	[&](const CanardRxSubscription * const sub) {
		if (sub->user_reference == nullptr) {
			PX4_INFO("Message port id %d", sub->port_id);

		} else {
			((UavcanBaseSubscriber *)sub->user_reference)->printInfo();
		}
	});


	traverseTree<CanardRxSubscription>(_canard_handle.getRxSubscriptions(CanardTransferKindRequest),
	[&](const CanardRxSubscription * const sub) {
		if (sub->user_reference == nullptr) {
			PX4_INFO("Service response port id %d", sub->port_id);

		} else {
			((UavcanBaseSubscriber *)sub->user_reference)->printInfo();
		}
	});

	traverseTree<CanardRxSubscription>(_canard_handle.getRxSubscriptions(CanardTransferKindResponse),
	[&](const CanardRxSubscription * const sub) {
		if (sub->user_reference == nullptr) {
			PX4_INFO("Service request port id %d", sub->port_id);

		} else {
			((UavcanBaseSubscriber *)sub->user_reference)->printInfo();
		}
	});

	_mixing_output.printInfo();

	pthread_mutex_unlock(&_node_mutex);
}

static void print_usage()
{
	PX4_INFO("usage: \n"
		 "\tuavcannode {start|status|stop}");
}

extern "C" __EXPORT int cyphal_main(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage();
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		if (CyphalNode::instance()) {
			PX4_ERR("already started");
			return 1;
		}

		// CAN bitrate
		int32_t bitrate = 0;
		param_get(param_find("CYPHAL_BAUD"), &bitrate);

		// Node ID
		int32_t node_id = 0;
		param_get(param_find("CYPHAL_ID"), &node_id);

		if (node_id == -1) {
			node_id = CANARD_NODE_ID_UNSET;
		}

		// Start
		PX4_INFO("Node ID %" PRIu32 ", bitrate %" PRIu32, node_id, bitrate);
		return CyphalNode::start(node_id, bitrate);
	}

	/* commands below require the app to be started */
	CyphalNode *const inst = CyphalNode::instance();

	if (!inst) {
		PX4_ERR("application not running");
		return 1;
	}

	if (!strcmp(argv[1], "status") || !strcmp(argv[1], "info")) {
		inst->print_info();
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		delete inst;
		return 0;
	}

	print_usage();
	return 1;
}

void CyphalNode::sendHeartbeat()
{
	if (hrt_elapsed_time(&_uavcan_node_heartbeat_last) >= 1_s) {

		uavcan_node_Heartbeat_1_0 heartbeat{};
		heartbeat.uptime = _uavcan_node_heartbeat_transfer_id; // TODO: use real uptime
		heartbeat.health.value = uavcan_node_Health_1_0_NOMINAL;
		heartbeat.mode.value = uavcan_node_Mode_1_0_OPERATIONAL;
		const hrt_abstime now = hrt_absolute_time();
		size_t payload_size = uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;

		const CanardTransferMetadata transfer_metadata = {
			.priority       = CanardPriorityNominal,
			.transfer_kind  = CanardTransferKindMessage,
			.port_id        = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
			.remote_node_id = CANARD_NODE_ID_UNSET,
			.transfer_id    = _uavcan_node_heartbeat_transfer_id++
		};

		uavcan_node_Heartbeat_1_0_serialize_(&heartbeat, _uavcan_node_heartbeat_buffer, &payload_size);

		int32_t result = _canard_handle.TxPush(now + PUBLISHER_DEFAULT_TIMEOUT_USEC,
						       &transfer_metadata,
						       payload_size,
						       &_uavcan_node_heartbeat_buffer
						      );

		if (result < 0) {
			// An error has occurred: either an argument is invalid or we've ran out of memory.
			// It is possible to statically prove that an out-of-memory will never occur for a given application if the
			// heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
			PX4_ERR("Heartbeat transmit error %" PRId32 "", result);
		}

		_uavcan_node_heartbeat_last = now;
	}
}

bool UavcanMixingInterface::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
		unsigned num_control_groups_updated)
{
	// Note: This gets called from MixingOutput from within its update() function
	// Hence, the mutex lock in UavcanMixingInterface::Run() is in effect

	/// TODO: Need esc/servo metadata / bitmask(s)
	_esc_controller.update_outputs(stop_motors, outputs, num_outputs);
	// _servo_controller.update_outputs(stop_motors, outputs, num_outputs);

	return true;
}

void UavcanMixingInterface::Run()
{
	pthread_mutex_lock(&_node_mutex);
	_mixing_output.update();
	_mixing_output.updateSubscriptions();
	pthread_mutex_unlock(&_node_mutex);
}
