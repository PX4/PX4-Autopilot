/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "Uavcan.hpp"

#include <lib/ecl/geo/geo.h>
#include <lib/version/version.h>

using namespace time_literals;

UavcanNode *UavcanNode::_instance;

O1HeapInstance *uavcan_allocator{nullptr};

static void *memAllocate(CanardInstance *const ins, const size_t amount) { return o1heapAllocate(uavcan_allocator, amount); }
static void memFree(CanardInstance *const ins, void *const pointer) { o1heapFree(uavcan_allocator, pointer); }

#if defined(__PX4_NUTTX)
# if defined(CONFIG_NET_CAN)
#  include "CanardSocketCAN.hpp"
# elif defined(CONFIG_CAN)
#  include "CanardNuttXCDev.hpp"
# endif // CONFIG_CAN
#endif // NuttX

UavcanNode::UavcanNode(CanardInterface *interface, uint32_t node_id) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan),
	_can_interface(interface)
{
	pthread_mutex_init(&_node_mutex, nullptr);

	_uavcan_heap = memalign(O1HEAP_ALIGNMENT, HeapSize);
	uavcan_allocator = o1heapInit(_uavcan_heap, HeapSize, nullptr, nullptr);

	if (uavcan_allocator == nullptr) {
		PX4_ERR("o1heapInit failed with size %d", HeapSize);
	}

	_canard_instance = canardInit(&memAllocate, &memFree);

	_canard_instance.node_id = node_id; // Defaults to anonymous; can be set up later at any point.

	bool can_fd = false;

	if (can_fd) {
		_canard_instance.mtu_bytes = CANARD_MTU_CAN_FD;

	} else {
		_canard_instance.mtu_bytes = CANARD_MTU_CAN_CLASSIC;
	}
}

UavcanNode::~UavcanNode()
{
	delete _can_interface;

	if (_instance) {
		/* tell the task we want it to go away */
		_task_should_exit.store(true);
		ScheduleNow();

		unsigned i = 10;

		do {
			/* wait 5ms - it should wake every 10ms or so worst-case */
			usleep(5000);

			if (--i == 0) {
				break;
			}

		} while (_instance);
	}

	perf_free(_cycle_perf);
	perf_free(_interval_perf);

	//delete _uavcan_heap;
}

int UavcanNode::start(uint32_t node_id, uint32_t bitrate)
{
	if (_instance != nullptr) {
		PX4_WARN("Already started");
		return -1;
	}

#if defined(__PX4_NUTTX)
# if defined(CONFIG_NET_CAN)
	CanardInterface *interface = new CanardSocketCAN();
# elif defined(CONFIG_CAN)
	CanardInterface *interface = new CanardNuttXCDev();
# endif // CONFIG_CAN
#endif // NuttX

	_instance = new UavcanNode(interface, node_id);

	if (_instance == nullptr) {
		PX4_ERR("Out of memory");
		return -1;
	}

	// Keep the bit rate for reboots on BenginFirmware updates
	_instance->active_bitrate = bitrate;

	_instance->ScheduleOnInterval(ScheduleIntervalMs * 1000);

	return PX4_OK;
}

void UavcanNode::Run()
{
	pthread_mutex_lock(&_node_mutex);

	if (!_initialized) {
		// interface init
		if (_can_interface) {
			if (_can_interface->init() == PX4_OK) {

				// Subscribe to messages uavcan.node.Heartbeat.
				canardRxSubscribe(&_canard_instance,
						  CanardTransferKindMessage,
						  uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
						  uavcan_node_Heartbeat_1_0_EXTENT_BYTES_,
						  CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
						  &_heartbeat_subscription);
                
                // Subscribe to messages uavcan.node.NodeIDAllocationData_1_0 for PNP V1
                canardRxSubscribe(&_canard_instance,
						  CanardTransferKindMessage,
						  uavcan_pnp_NodeIDAllocationData_1_0_FIXED_PORT_ID_,
						  uavcan_pnp_NodeIDAllocationData_1_0_EXTENT_BYTES_,
						  CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
						  &_pnp_v1_subscription);
                
                canardRxSubscribe(&_canard_instance,
                            CanardTransferKindResponse,
                            uavcan_register_Access_1_0_FIXED_PORT_ID_,
                            uavcan_register_Access_Response_1_0_EXTENT_BYTES_,
                            CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                            &_register_access_subscription);

                canardRxSubscribe(&_canard_instance,
                            CanardTransferKindResponse,
                            uavcan_register_List_1_0_FIXED_PORT_ID_,
                            uavcan_register_List_Response_1_0_EXTENT_BYTES_,
                            CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                            &_register_list_subscription);
                


                canardRxSubscribe(&_canard_instance,
							  CanardTransferKindMessage,
							  test_port_id,
							  reg_drone_srv_battery_Status_0_1_EXTENT_BYTES_,
							  CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
							  &_drone_srv_battery_subscription);

				_initialized = true;
			}
		}

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
	}

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);



    if (hrt_elapsed_time(&_uavcan_pnp_nodeidallocation_last) >= 1_s &&
        _node_register_setup != CANARD_NODE_ID_UNSET && 
        _node_register_request_index == _node_register_last_received_index+1){
        
        PX4_INFO("NodeID %i request register %i", _node_register_setup, _node_register_request_index);
        
        uavcan_register_List_Request_1_0 msg;
        msg.index = _node_register_request_index;
        
        uint8_t request_payload_buffer[uavcan_register_List_Request_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
        
        CanardTransfer request = {
            .timestamp_usec = hrt_absolute_time(), // Zero if transmission deadline is not limited.
            .priority       = CanardPriorityNominal,
            .transfer_kind  = CanardTransferKindRequest,
            .port_id        = uavcan_register_List_1_0_FIXED_PORT_ID_, // This is the subject-ID.
            .remote_node_id = _node_register_setup,
            .transfer_id    = _uavcan_register_list_request_transfer_id,
            .payload_size   = uavcan_register_List_Request_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
            .payload        = &request_payload_buffer,
        };

        int32_t result = uavcan_register_List_Request_1_0_serialize_(&msg, request_payload_buffer, &request.payload_size);

        if(result == 0){
            // set the data ready in the buffer and chop if needed 
            ++_uavcan_register_list_request_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
            result = canardTxPush(&_canard_instance, &request);
            
            ++_node_register_request_index;
        }
    }
    
	// send uavcan::node::Heartbeat_1_0 @ 1 Hz
	if (hrt_elapsed_time(&_uavcan_node_heartbeat_last) >= 1_s) {

		uavcan_node_Heartbeat_1_0 heartbeat{};
		heartbeat.uptime = _uavcan_node_heartbeat_transfer_id; // TODO: use real uptime
		heartbeat.health.value = uavcan_node_Health_1_0_NOMINAL;
		heartbeat.mode.value = uavcan_node_Mode_1_0_OPERATIONAL;


		CanardTransfer transfer = {
			.timestamp_usec = hrt_absolute_time(),
			.priority       = CanardPriorityNominal,
			.transfer_kind  = CanardTransferKindMessage,
			.port_id        = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
			.remote_node_id = CANARD_NODE_ID_UNSET,
			.transfer_id    = _uavcan_node_heartbeat_transfer_id++,
			.payload_size   = uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
			.payload        = &_uavcan_node_heartbeat_buffer,
		};

        uavcan_node_Heartbeat_1_0_serialize_(&heartbeat, _uavcan_node_heartbeat_buffer, &transfer.payload_size);

		int32_t result = canardTxPush(&_canard_instance, &transfer);

		if (result < 0) {
			// An error has occurred: either an argument is invalid or we've ran out of memory.
			// It is possible to statically prove that an out-of-memory will never occur for a given application if the
			// heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
			PX4_ERR("Heartbeat transmit error %d", result);
		}

		_uavcan_node_heartbeat_last = transfer.timestamp_usec;
	}

	// Transmitting
	// Look at the top of the TX queue.
	for (const CanardFrame *txf = nullptr; (txf = canardTxPeek(&_canard_instance)) != nullptr;) {
		// Check if the frame has timed out.
		if (hrt_absolute_time() > txf->timestamp_usec) { //FIXME wrong I think
			// Send the frame. Redundant interfaces may be used here.
			const int tx_res = _can_interface->transmit(*txf);

			if (tx_res < 0) {
				// Failure - drop the frame and report
				canardTxPop(&_canard_instance);

				// Deallocate the dynamic memory afterwards.
				_canard_instance.memory_free(&_canard_instance, (CanardFrame *)txf);
				PX4_ERR("Transmit error %d, frame dropped, errno '%s'", tx_res, strerror(errno));

			} else if (tx_res > 0) {
				// Success - just drop the frame
				canardTxPop(&_canard_instance);

				// Deallocate the dynamic memory afterwards.
				_canard_instance.memory_free(&_canard_instance, (CanardFrame *)txf);

			} else {
				// Timeout - just exit and try again later
				break;
			}
		}
	}

	uint8_t data[64] {};
	CanardFrame received_frame{};
	received_frame.payload = &data;

	if (_can_interface->receive(&received_frame) > 0) {

		CanardTransfer receive{};
		int32_t result = canardRxAccept(&_canard_instance, &received_frame, 0, &receive);

		if (result < 0) {
			// An error has occurred: either an argument is invalid or we've ran out of memory.
			// It is possible to statically prove that an out-of-memory will never occur for a given application if
			// the heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
			// Reception of an invalid frame is NOT an error.
			PX4_ERR("Receive error %d\n", result);

		} else if (result == 1) {
			// A transfer has been received, process it.
			PX4_INFO("received Port ID: %d", receive.port_id);

            if (receive.port_id == uavcan_pnp_NodeIDAllocationData_1_0_FIXED_PORT_ID_) {
                uavcan_pnp_NodeIDAllocationData_1_0 msg;
        
                size_t pnp_in_size_bits = receive.payload_size;
                uavcan_pnp_NodeIDAllocationData_1_0_deserialize_(&msg, (const uint8_t*)receive.payload, &pnp_in_size_bits);
                
                //TODO internal database with unique id to node ip mappings now we give an hardcoded ID back
                
                msg.allocated_node_id.count = 1;
                msg.allocated_node_id.elements[0].value = 15; // HACK hardcoded ID
                
                _uavcan_pnp_nodeidallocation_last = hrt_absolute_time();
                _node_register_request_index = 0;
                _node_register_last_received_index = -1;
                _node_register_setup = msg.allocated_node_id.elements[0].value; // This nodeID has to be configured
                
                PX4_INFO("Received NodeID allocation request assigning %i", msg.allocated_node_id.elements[0].value);
                
                uint8_t node_id_alloc_payload_buffer[uavcan_pnp_NodeIDAllocationData_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
                
                CanardTransfer transfer = {
                    .timestamp_usec = hrt_absolute_time(),      // Zero if transmission deadline is not limited.
                    .priority       = CanardPriorityNominal,
                    .transfer_kind  = CanardTransferKindMessage,
                    .port_id        = uavcan_pnp_NodeIDAllocationData_1_0_FIXED_PORT_ID_,                // This is the subject-ID.
                    .remote_node_id = CANARD_NODE_ID_UNSET,       // Messages cannot be unicast, so use UNSET.
                    .transfer_id    = _uavcan_pnp_nodeidallocation_v1_transfer_id,
                    .payload_size   = uavcan_pnp_NodeIDAllocationData_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
                    .payload        = &node_id_alloc_payload_buffer,
                };

                result = uavcan_pnp_NodeIDAllocationData_1_0_serialize_(&msg, node_id_alloc_payload_buffer, &transfer.payload_size);

                if(result == 0) {
                    // set the data ready in the buffer and chop if needed 
                    ++_uavcan_pnp_nodeidallocation_v1_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
                    result = canardTxPush(&_canard_instance, &transfer);
                }
			} if (receive.port_id == uavcan_register_List_1_0_FIXED_PORT_ID_) {
                uavcan_register_List_Response_1_0 msg;
        
                size_t register_in_size_bits = receive.payload_size;
                uavcan_register_List_Response_1_0_deserialize_(&msg, (const uint8_t*)receive.payload, &register_in_size_bits);
                
                
                
                
                
                if(strncmp((char*)msg.name.name.elements, "uavcan.pub.battery_status.id", msg.name.name.count) == 0) { //Battery status publisher
                    _node_register_setup = CANARD_NODE_ID_UNSET;
                    PX4_INFO("NodeID %i battery_status publisher set PortID to %i", receive.remote_node_id, test_port_id);
                    _node_register_last_received_index++;
                    
                    uavcan_register_Access_Request_1_0 request_msg;
                    memcpy(&request_msg.name, &msg.name, sizeof(uavcan_register_Name_1_0));
                    
                    uavcan_register_Value_1_0_select_natural16_(&request_msg.value);
                    request_msg.value.natural16.value.count = 1;
                    request_msg.value.natural16.value.elements[0] = test_port_id;
                    
                    
                    uint8_t request_payload_buffer[uavcan_register_Access_Request_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
                    
                    CanardTransfer transfer = {
                        .timestamp_usec = hrt_absolute_time(),      // Zero if transmission deadline is not limited.
                        .priority       = CanardPriorityNominal,
                        .transfer_kind  = CanardTransferKindRequest,
                        .port_id        = uavcan_register_Access_1_0_FIXED_PORT_ID_,                // This is the subject-ID.
                        .remote_node_id = receive.remote_node_id,       // Messages cannot be unicast, so use UNSET.
                        .transfer_id    = _uavcan_register_access_request_transfer_id,
                        .payload_size   = uavcan_register_Access_Request_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
                        .payload        = &request_payload_buffer,
                    };

                    result = uavcan_register_Access_Request_1_0_serialize_(&request_msg, request_payload_buffer, &transfer.payload_size);

                    if(result == 0) {
                        // set the data ready in the buffer and chop if needed 
                        ++_uavcan_register_access_request_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
                        result = canardTxPush(&_canard_instance, &transfer);
                    }
                }
                
                
			} else if (receive.port_id == test_port_id) {
				PX4_INFO("NodeID %i Battery Status msg", receive.remote_node_id);
                //TODO deserialize
                
                /*

				battery_status_s battery_status{};
				battery_status.id = bms_status.battery_id;
                                battery_status.voltage_v = bms_status.voltage;
				//battery_status.remaining = bms_status.remaining_capacity;
				battery_status.timestamp = hrt_absolute_time();
				_battery_status_pub.publish(battery_status);*/
			}

			// Deallocate the dynamic memory afterwards.
			_canard_instance.memory_free(&_canard_instance, (void *)receive.payload);
		} else {
                PX4_INFO("RX canard %d\r\n", result);
            }
	}

	perf_end(_cycle_perf);

	if (_task_should_exit.load()) {
		_can_interface->close();

		ScheduleClear();
		_instance = nullptr;
	}

	pthread_mutex_unlock(&_node_mutex);
}

void UavcanNode::print_info()
{
	pthread_mutex_lock(&_node_mutex);

	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);

	pthread_mutex_unlock(&_node_mutex);
}

static void print_usage()
{
	PX4_INFO("usage: \n"
		 "\tuavcannode {start|status|stop}");
}

extern "C" __EXPORT int uavcan_v1_main(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage();
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		if (UavcanNode::instance()) {
			PX4_ERR("already started");
			return 1;
		}

		// CAN bitrate
		int32_t bitrate = 0;
		param_get(param_find("UAVCAN_V1_BAUD"), &bitrate);

		// Node ID
		int32_t node_id = 0;
		param_get(param_find("UAVCAN_V1_ID"), &node_id);

		// Start
		PX4_INFO("Node ID %u, bitrate %u", node_id, bitrate);
		return UavcanNode::start(node_id, bitrate);
	}

	/* commands below require the app to be started */
	UavcanNode *const inst = UavcanNode::instance();

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
