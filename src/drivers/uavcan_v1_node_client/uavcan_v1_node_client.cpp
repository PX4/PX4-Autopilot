/****************************************************************************
 *
 *   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
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

#include "uavcan_v1_node_client.h"
#include <px4_platform_common/getopt.h>

UavcanNodeClient *UavcanNodeClient::_instance;
O1HeapInstance *uavcan_allocator{nullptr};

static void *memAllocate(CanardInstance *const ins, const size_t amount) { return o1heapAllocate(uavcan_allocator, amount); }
static void memFree(CanardInstance *const ins, void *const pointer) { o1heapFree(uavcan_allocator, pointer); }

int32_t set_gps_uorb_port_id(uavcan_register_Value_1_0 *value);
uavcan_register_Value_1_0 get_gps_uorb_port_id();
int32_t  set_gps_fix_port_id(uavcan_register_Value_1_0 *value);
uavcan_register_Value_1_0 get_gps_fix_port_id();
int32_t set_gps_aux_port_id(uavcan_register_Value_1_0 *value);
uavcan_register_Value_1_0 get_gps_aux_port_id();

uint64_t getMonotonicTimestampUSec(void)
{
	int ret;
	struct timespec ts;

	memset(&ts, 0, sizeof(ts));

	ret = clock_gettime(CLOCK_MONOTONIC, &ts);

	if (ret != 0) {
		PX4_ERR("clock error %i", ret);
		abort();
	}

	return ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;
}

#define UNIQUE_ID_LENGTH_BYTES                   16

static uint8_t unique_id[UNIQUE_ID_LENGTH_BYTES] = { //FIXME real HW ID
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01
};

UavcanNodeClient::UavcanNodeClient(const char can_bus_name[], bool is_can_fd)  :
	_uavcan_heap{nullptr},
	_canard_instance{0},
	_canard_socket_instance{0},
	_fd{0},
	_can_bus_name{},
	_state(UavcanNodeStates::UAVCANNODE_STATE_INIT),
	_ask_node_id_time_capture(0)
{
	if (UAVCANNODE_CLIENT_DEV_NAME_SIZE >= strlen(can_bus_name)) {
		memcpy(_can_bus_name, can_bus_name, strlen(can_bus_name));

	} else {
		PX4_ERR("CAN bus name can't be longer than %d characters",
			UAVCANNODE_CLIENT_DEV_NAME_SIZE);
	}

	_is_can_fd = is_can_fd;
}

UavcanNodeClient::~UavcanNodeClient()
{
	_canard_instance.node_id = CANARD_NODE_ID_UNSET;
}

int UavcanNodeClient::init()
{
	int ret_val = PX4_OK;

	_uavcan_heap = memalign(O1HEAP_ALIGNMENT, HeapSize);
	uavcan_allocator = o1heapInit(_uavcan_heap, HeapSize, nullptr, nullptr);

	if (uavcan_allocator == nullptr) {
		PX4_ERR("o1heapInit failed with size %d", HeapSize);
		ret_val = PX4_ERROR;
	}

	if (ret_val == PX4_OK) {

		_canard_instance = canardInit(&memAllocate, &memFree);

		if (_is_can_fd) {
			_canard_instance.mtu_bytes = CANARD_MTU_CAN_FD;

		} else {
			_canard_instance.mtu_bytes = CANARD_MTU_CAN_CLASSIC;
		}

		/* Open the CAN device for reading */
		ret_val = socketcanOpen(&_canard_socket_instance, _can_bus_name, _is_can_fd);

		if (ret_val == PX4_OK) {
			/* setup poll fd */
			_fd.fd = _canard_socket_instance.s;
			_fd.events = POLLIN;

			if (_canard_socket_instance.s < 0) {
				PX4_ERR("ERROR: open %s failed.", _can_bus_name);
				ret_val = PX4_ERROR;
			}
		}
	}

	if (ret_val == PX4_OK) {
		/* Init UAVCAN register interfaces */
		uavcan_node_GetInfo_Response_1_0 node_information; // TODO ADD INFO
		uavcan_register_interface_init(&_canard_instance, &node_information);
		uavcan_register_interface_add_entry("gnss_uorb", set_gps_uorb_port_id, get_gps_uorb_port_id);
		uavcan_register_interface_add_entry("gnss_fix", set_gps_fix_port_id, get_gps_fix_port_id);
		uavcan_register_interface_add_entry("gnss_aux", set_gps_aux_port_id, get_gps_aux_port_id);

		initPNPAllocatee(&_canard_instance, unique_id);
	}

	if (ret_val == PX4_OK) {
		_state = UavcanNodeStates::UAVCANNODE_STATE_ASK_NODE_ID;
	}

	return ret_val;

}

void UavcanNodeClient::run()
{
	while (!should_exit()) {

		switch (_state) {
		case UavcanNodeStates::UAVCANNODE_STATE_INIT:
			init();
			break;

		case UavcanNodeStates::UAVCANNODE_STATE_ASK_NODE_ID:
			ask_node_id();
			break;

		case UavcanNodeStates::UAVCANNODE_STATE_WAIT_NODE_ID:
			wait_node_id();
			break;

		case UavcanNodeStates::UAVCANNODE_STATE_UORB_BRIDGE:
			break;

		default:
			break;
		}

		process_tx_rx_once(&_canard_instance, &_canard_socket_instance, 10); //10Ms
		px4_usleep(UAVCANNODE_CLIENT_INTERVAL);
	}
}

void UavcanNodeClient::ask_node_id()
{
	if (_canard_instance.node_id == CANARD_NODE_ID_UNSET) {

		PNPAllocRequest(&_canard_instance);
		_ask_node_id_time_capture = hrt_absolute_time(); ;
		_state = UavcanNodeStates::UAVCANNODE_STATE_WAIT_NODE_ID;
	}
}

void UavcanNodeClient::wait_node_id()
{
	volatile hrt_abstime elapsed_time = hrt_elapsed_time(&_ask_node_id_time_capture);

	if (elapsed_time > UAVCANNODE_CLIENT_WAIT_NODE_ID_TIMEOUT) {

		/* Didn't received node id, try again */
		_state = UavcanNodeStates::UAVCANNODE_STATE_ASK_NODE_ID;

	} else {

		CanardNodeID node_id = PNPGetNodeID();

		if (node_id != CANARD_NODE_ID_UNSET) {

			_state = UavcanNodeStates::UAVCANNODE_STATE_UORB_BRIDGE;

			_canard_instance.node_id = node_id;

			PX4_INFO("Start node (ID: %d MTU: %d)\n",
				 _canard_instance.node_id, _canard_instance.mtu_bytes);
		}
	}
}

void UavcanNodeClient::process_tx_rx_once(CanardInstance *ins, CanardSocketInstance *sock_ins, int timeout_msec)
{
	int32_t result;

	/* Transmitting */

	for (const CanardFrame *txf = NULL; (txf = canardTxPeek(ins)) != NULL;) { // Look at the top of the TX queue.

		uint64_t current_timestamp = getMonotonicTimestampUSec();

		if (txf->timestamp_usec > current_timestamp) { // Check if the frame has timed out.

			int error = socketcanTransmit(sock_ins, txf); // Send the frame. Redundant interfaces may be used here.

			if (error == -EBUSY) {
				break; // If the driver is busy, break and retry later.
			}

		} else {
			PX4_INFO("Timeout??\n");
		}

		canardTxPop(ins);                         // Remove the frame from the queue after it's transmitted.
		ins->memory_free(ins, (CanardFrame *)txf); // Deallocate the dynamic memory afterwards.
	}


	/* Poll receive */
	if (poll(&_fd, 1, timeout_msec) <= 0) {
		return;
	}

	/* Receiving */
	CanardFrame received_frame;

	socketcanReceive(sock_ins, &received_frame);

	CanardTransfer receive;
	result = canardRxAccept(ins,
				&received_frame, // The CAN frame received from the bus.
				0,               // If the transport is not redundant, use 0.
				&receive);

	if (result < 0) {
		// An error has occurred: either an argument is invalid or we've ran out of memory.
		// It is possible to statically prove that an out-of-memory will never occur for a given application if
		// the heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
		// Reception of an invalid frame is NOT an error.
		PX4_ERR("Receive error %d\n", result);

	} else if (result == 1) {
		PX4_INFO("Receive portId %i\n", receive.port_id);

		if (receive.port_id == PNPGetPortID()) {
			PNPProcess(ins, &receive);

		} else {
			uavcan_register_interface_process(&_canard_instance, &receive);
		}

		ins->memory_free(ins, (void *)receive.payload); // Deallocate the dynamic memory afterwards.

	} else {
		// printf("RX canard %d\r\n", result);
		// Nothing to do.
		// The received frame is either invalid or it's a non-last frame of a multi-frame transfer.
		// Reception of an invalid frame is NOT reported as an error because it is not an error.
	}

}

UavcanNodeClient *UavcanNodeClient::instantiate(int argc, char *argv[])
{
	UavcanNodeClient *uavcan_node_client;

	const char *can_protocol = "";
	const char *can_bus = UAVCANNODE_CLIENT_CAN_BUS;
	bool error_flag = false;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	bool is_can_fd = false;

	while ((ch = px4_getopt(argc, argv, "b:p:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {

		case 'b':
			can_bus = myoptarg;
			PX4_INFO("CAN bus: %s\n", can_bus);
			break;

		case 'p':
			can_protocol = myoptarg;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (!strcmp(can_protocol, UAVCANNODE_CLIENT_CANFD_STR)) {
		is_can_fd = true;
		PX4_INFO("CAN FD selected\n");

	} else {
		PX4_INFO("Classical CAN selected\n");
	}

	if (error_flag) {
		uavcan_node_client = nullptr;

	} else {
		uavcan_node_client = new UavcanNodeClient(can_bus, is_can_fd);
	}

	return uavcan_node_client;
}

int UavcanNodeClient::task_spawn(int argc, char *argv[])
{
	int ret_val = PX4_OK;

	px4_main_t entry_point = (px4_main_t)&run_trampoline;

	int task_id = px4_task_spawn_cmd("uavcannode_client", SCHED_DEFAULT,
					 SCHED_PRIORITY_FAST_DRIVER, UAVCANNODE_CLIENT_STACK_SIZE,
					 entry_point, (char *const *)argv);

	if (task_id < 0) {
		task_id = -1;
		ret_val  = PX4_ERROR;
	}

	_task_id = task_id;

	return ret_val;
}

int UavcanNodeClient::custom_command(int argc, char *argv[])
{
	return 0;
}

int UavcanNodeClient::print_usage(const char *reason)
{

	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_USAGE_NAME("uavcannode_client", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("uavcannode");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	PRINT_MODULE_USAGE_PARAM_STRING('b', "can0", "can0, can1", "CAN bus", true);
	PRINT_MODULE_USAGE_PARAM_STRING('p', "can-fd", "can, can", "CAN protocol", true);

	return 0;
}
