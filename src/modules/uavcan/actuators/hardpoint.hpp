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
 * @file hardpoint.hpp
 *
 * @author Andreas Jochum <Andreas@NicaDrone.com>
 */

#pragma once

#include <uavcan/uavcan.hpp>
#include <uavcan/equipment/hardpoint/Command.hpp>
#include <uavcan/equipment/hardpoint/Status.hpp>
#include <systemlib/perf_counter.h>

/**
 * @brief The UavcanHardpointController class
 */

class UavcanHardpointController
{
public:
	UavcanHardpointController(uavcan::INode &node);
	~UavcanHardpointController();

	/*
	* setup periodic updater
	*/
	int init();


	/*
	 * set command
	 */
	void set_command(uint8_t hardpoint_id, uint16_t command);

private:
	/*
	 * Max update rate to avoid exessive bus traffic
	 */
	static constexpr unsigned			MAX_RATE_HZ = 1;	///< XXX make this configurable

	uavcan::equipment::hardpoint::Command		_cmd;

	void periodic_update(const uavcan::TimerEvent &);

	typedef uavcan::MethodBinder<UavcanHardpointController *, void (UavcanHardpointController::*)(const uavcan::TimerEvent &)>
	TimerCbBinder;

	pthread_mutex_t					_node_mutex;
	/*
	 * libuavcan related things
	 */
	uavcan::INode							&_node;
	uavcan::Publisher<uavcan::equipment::hardpoint::Command>	_uavcan_pub_raw_cmd;
	uavcan::TimerEventForwarder<TimerCbBinder>			_timer;

};
