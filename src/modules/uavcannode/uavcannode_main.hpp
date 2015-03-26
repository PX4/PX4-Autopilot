/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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

#include <nuttx/config.h>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <drivers/device/device.h>

/**
 * @file uavcan_main.hpp
 *
 * Defines basic functinality of UAVCAN node.
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#define NUM_ACTUATOR_CONTROL_GROUPS_UAVCAN	1
#define UAVCAN_DEVICE_PATH	"/dev/uavcan/node"

// we add two to allow for actuator_direct and busevent
#define UAVCAN_NUM_POLL_FDS (NUM_ACTUATOR_CONTROL_GROUPS_UAVCAN+2)

/**
 * A UAVCAN node.
 */
class UavcanNode : public device::CDev
{
	static constexpr unsigned MemPoolSize        = 3200; ///< Refer to the libuavcan manual to learn why
	static constexpr unsigned RxQueueLenPerIface = 64;
	static constexpr unsigned StackSize          = 768;

public:
	typedef uavcan::Node<MemPoolSize> Node;
	typedef uavcan_stm32::CanInitHelper<RxQueueLenPerIface> CanInitHelper;

	UavcanNode(uavcan::ICanDriver &can_driver, uavcan::ISystemClock &system_clock);

	virtual		~UavcanNode();

	virtual int	ioctl(file *filp, int cmd, unsigned long arg);

	static int	start(uavcan::NodeID node_id, uint32_t bitrate);

	Node&		get_node() { return _node; }

	int		teardown();

	void		print_info();

	static UavcanNode* instance() { return _instance; }

private:
	void		fill_node_info();
	int		init(uavcan::NodeID node_id);
	void		node_spin_once();
	int		run();
	int		add_poll_fd(int fd);			///< add a fd to poll list, returning index into _poll_fds[]


	int			_task = -1;			///< handle to the OS task
	bool			_task_should_exit = false;	///< flag to indicate to tear down the CAN driver

	static UavcanNode	*_instance;			///< singleton pointer
	Node			_node;				///< library instance
	pthread_mutex_t		_node_mutex;

	pollfd			_poll_fds[UAVCAN_NUM_POLL_FDS] = {};
	unsigned		_poll_fds_num = 0;

};
