/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
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
 * @file hardpoint.cpp
 *
 * @author Andreas Jochum <Andreas@NicaDrone.com>
 */

#include "hardpoint.hpp"
#include <systemlib/err.h>

UavcanHardpointController::UavcanHardpointController(uavcan::INode &node) :
	_node(node),
	_uavcan_pub_raw_cmd(node),
	_timer(node)
{
	_uavcan_pub_raw_cmd.setPriority(uavcan::TransferPriority::MiddleLower);
}


UavcanHardpointController::~UavcanHardpointController()
{

}

int UavcanHardpointController::init()
{
	/*
	 * Setup timer and call back function for periodic updates
	 */
	_timer.setCallback(TimerCbBinder(this, &UavcanHardpointController::periodic_update));
	return 0;
}

void UavcanHardpointController::set_command(uint8_t hardpoint_id, uint16_t command)
{
	_cmd.command = command;
	_cmd.hardpoint_id = hardpoint_id;

	/*
	 * Publish the command message to the bus
	 */
	(void)_uavcan_pub_raw_cmd.broadcast(_cmd);

	/*
	 * Start the periodic update timer after a command is set
	 */
	if (!_timer.isRunning()) {
		_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000 / MAX_RATE_HZ));
	}

}
void UavcanHardpointController::periodic_update(const uavcan::TimerEvent &)
{
	/*
	 * Broadcast command at MAX_RATE_HZ
	 */
	(void)_uavcan_pub_raw_cmd.broadcast(_cmd);
}
