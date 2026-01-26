/****************************************************************************
 *
 *   Copyright (C) 2014-2025 PX4 Development Team. All rights reserved.
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
 * @file esc.hpp
 *
 * UAVCAN <--> ORB bridge for ESC messages:
 *     uavcan.equipment.esc.RawCommand
 *     uavcan.equipment.esc.RPMCommand
 *     uavcan.equipment.esc.Status
 *     uavcan.equipment.esc.StatusExtended
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/uavcan.hpp>
#include <uavcan/equipment/esc/RawCommand.hpp>
#include <uavcan/equipment/esc/Status.hpp>
#include <uavcan/equipment/esc/StatusExtended.hpp>
#include <lib/perf/perf_counter.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/esc_report.h>
#include <uORB/topics/dronecan_node_status.h>
#include <uORB/topics/device_information.h>
#include <drivers/drv_hrt.h>
#include <drivers/uavcan/node_info.hpp>
#include "../node_info.hpp"

class UavcanEscController
{
public:
	static constexpr int MAX_ACTUATORS = esc_status_s::CONNECTED_ESC_MAX;
	static constexpr unsigned MAX_RATE_HZ = 400;
	static constexpr int16_t kMaxUavcanNodeId = 128; // UAVCAN supports up to 128 nodes (0-127)

	static_assert(uavcan::equipment::esc::RawCommand::FieldTypes::cmd::MaxSize >= MAX_ACTUATORS, "Too many actuators");


	UavcanEscController(uavcan::INode &node);
	~UavcanEscController() = default;

	int init();

	bool initialized() { return _initialized; };

	void update_outputs(uint16_t outputs[MAX_ACTUATORS], uint8_t output_array_size);
	/**
	 * Sets the number of rotors and enable timer
	 */
	void set_rotor_count(uint8_t count);

	void set_node_info_publisher(NodeInfoPublisher *publisher) { _node_info_publisher = publisher; }

	static int max_output_value() { return uavcan::equipment::esc::RawCommand::FieldTypes::cmd::RawValueType::max(); }

	esc_status_s &esc_status() { return _esc_status; }

private:
	/**
	 * ESC status message reception will be reported via this callback.
	 */
	void esc_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status> &msg);
	/**
	 * ESC extended status message reception will be stored via this callback.
	 */
	void esc_status_extended_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::StatusExtended> &msg);

	/**
	 * Checks all the ESCs freshness based on timestamp, if an ESC exceeds the timeout then is flagged offline.
	 */
	uint8_t check_escs_status();


	/**
	 * Gets failure flags for a specific ESC
	 */
	uint32_t get_failures(uint8_t esc_index);

	typedef uavcan::MethodBinder<UavcanEscController *,
		void (UavcanEscController::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status>&)> StatusCbBinder;

	typedef uavcan::MethodBinder<UavcanEscController *,
		void (UavcanEscController::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::StatusExtended>&)>
		StatusExtendedCbBinder;

	typedef uavcan::MethodBinder<UavcanEscController *,
		void (UavcanEscController::*)(const uavcan::TimerEvent &)> TimerCbBinder;

	bool _initialized{};

	esc_status_s	_esc_status{};

	uORB::PublicationMulti<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};
	uORB::Subscription _dronecan_node_status_sub{ORB_ID(dronecan_node_status)};
	uORB::Subscription _device_information_sub{ORB_ID(device_information)};

	uint8_t		_rotor_count{0};

	/*
	 * libuavcan related things
	 */
	uavcan::MonotonicTime							_prev_cmd_pub;   ///< rate limiting
	uavcan::INode								&_node;
	uavcan::Publisher<uavcan::equipment::esc::RawCommand>			_uavcan_pub_raw_cmd;
	uavcan::Subscriber<uavcan::equipment::esc::Status, StatusCbBinder>	_uavcan_sub_status;
	uavcan::Subscriber<uavcan::equipment::esc::StatusExtended, StatusExtendedCbBinder>	_uavcan_sub_status_extended;

	NodeInfoPublisher *_node_info_publisher{nullptr};
};
