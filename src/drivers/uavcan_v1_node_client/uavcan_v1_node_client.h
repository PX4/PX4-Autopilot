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

#pragma once

#include <sys/types.h>
#include <sys/time.h>

#include <px4_platform_common/module.h>
#include <drivers/drv_hrt.h>

#include "o1heap.h"
#include "socketcan.h"
#include <poll.h>

#include <canard.h>
#include <canard_dsdl.h>
#include "uorb_converter.h"

#include "libcancl/pnp.h"
#include "libcancl/registerinterface.h"

#define UAVCANNODE_CLIENT_DEV_NAME_SIZE	10
#define UAVCANNODE_CLIENT_CAN_BUS	"can0"
#define UAVCANNODE_CLIENT_CANFD_STR		"can-fd"

enum class UavcanNodeStates {
	UAVCANNODE_STATE_INIT = 0,
	UAVCANNODE_STATE_ASK_NODE_ID,
	UAVCANNODE_STATE_WAIT_NODE_ID,
	UAVCANNODE_STATE_UORB_CONVERTER_INIT,
	UAVCANNODE_STATE_UORB_CONVERTER_RUN
};

using namespace time_literals;

class UavcanNodeClient :  public ModuleBase<UavcanNodeClient>
{
public:

	/*
	* This memory is allocated for the 01Heap allocator used by
	* libcanard to store incoming/outcoming data
	* Current size of 8192 bytes is arbitrary, should be optimized further
	* when more nodes and messages are on the CAN bus
	*/
	static constexpr unsigned HeapSize = 8192;

	UavcanNodeClient(const char can_iface_name[], bool is_can_fd);
	virtual ~UavcanNodeClient();

	int init();

	void run() override;

	void ask_node_id();
	void wait_node_id();
	hrt_abstime random_time();
	void process_tx_once(CanardInstance *ins, CanardSocketInstance *sock_ins);
	void process_rx_once(CanardInstance *ins, CanardSocketInstance *sock_ins);

	static int32_t set_gps_uorb_port_id(uavcan_register_Value_1_0 *value);
	static uavcan_register_Value_1_0 get_gps_uorb_port_id();
	static int32_t set_gps_fix_port_id(uavcan_register_Value_1_0 *value);
	static uavcan_register_Value_1_0 get_gps_fix_port_id();
	static int32_t set_gps_aux_port_id(uavcan_register_Value_1_0 *value);
	static uavcan_register_Value_1_0 get_gps_aux_port_id();

	/** @see ModuleBase */
	static UavcanNodeClient *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	static UavcanNodeClient *instance() { return _instance; }

private:

	void *_uavcan_heap;
	static UavcanNodeClient *_instance;
	bool _is_can_fd;							///< the flag indicates if we are using classic CAN or CAN FD
	CanardInstance _canard_instance;
	CanardSocketInstance _canard_socket_instance;
	struct pollfd _fd;
	char _can_bus_name[UAVCANNODE_CLIENT_DEV_NAME_SIZE];

	UavcanNodeStates _state;
	hrt_abstime _random_wait_time;

	static const int POLL_TIMEOUT					{10};	  // 10 ms
	static const hrt_abstime TASK_INTERVAL			{100_us};
	static const uint32_t STACK_SIZE				{4096u};
	static const hrt_abstime WAIT_NODE_ID_TIMEOUT	{100000}; // 100 ms
};
