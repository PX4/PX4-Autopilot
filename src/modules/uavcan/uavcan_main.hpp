/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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

#include <uavcan_stm32/uavcan_stm32.hpp>
#include <drivers/drv_pwm_output.h>

/**
 * @file uavcan_main.hpp
 *
 * Defines basic functinality of UAVCAN node.
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

/**
 * A UAVCAN node.
 */
class UavcanNode : public device::CDev
{
	static constexpr unsigned MemPoolSize        = 10752;
	static constexpr unsigned RxQueueLenPerIface = 64;
	static constexpr unsigned StackSize          = 3000;

public:
	typedef uavcan::Node<MemPoolSize> Node;
	typedef uavcan_stm32::CanInitHelper<RxQueueLenPerIface> CanInitHelper;

	UavcanNode(uavcan::ICanDriver &can_driver, uavcan::ISystemClock &system_clock);

	static int start(uavcan::NodeID node_id, uint32_t bitrate);

	Node &getNode() { return _node; }

	static int	control_callback(uintptr_t handle,
					 uint8_t control_group,
					 uint8_t control_index,
					 float &input);
	void	subscribe();

private:
	int init(uavcan::NodeID node_id);
	int run();

	static UavcanNode	*_instance;		///< pointer to the library instance
	Node			_node;

	MixerGroup		*_mixers;

	uint32_t		_groups_required;
	uint32_t		_groups_subscribed;
	int			_control_subs[NUM_ACTUATOR_CONTROL_GROUPS];
	actuator_controls_s 	_controls[NUM_ACTUATOR_CONTROL_GROUPS];
	orb_id_t		_control_topics[NUM_ACTUATOR_CONTROL_GROUPS];
	pollfd			_poll_fds[NUM_ACTUATOR_CONTROL_GROUPS];
	unsigned		_poll_fds_num;
};
