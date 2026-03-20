/****************************************************************************
 *
 *   Copyright (C) 2024-2026 PX4 Development Team. All rights reserved.
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
 * @file esc_hobbywing.hpp
 *
 * DroneCAN driver for HobbyWing ESCs (com.hobbywing.esc namespace).
 *
 * HobbyWing ESCs do not include an esc_index field in their status messages.
 * Instead, a GetEscID discovery protocol maps CAN node IDs to motor slots
 * before arming. While disarmed, the FC broadcasts GetEscID (DTID 20013)
 * every 1 second; each ESC responds with [node_id, throttle_channel].
 *
 * Default ESC settings: CAN node ID = 1, bitrate = 500 kbps.
 * Change via SetID (DTID 210) and SetBaud (DTID 211) services using the
 * DroneCAN GUI Tool, or use the HobbyWing PC configuration software.
 */

#pragma once

#ifdef CONFIG_UAVCAN_HOBBYWING_ESC

#include <uavcan/uavcan.hpp>
#include <com/hobbywing/esc/RawCommand.hpp>
#include <com/hobbywing/esc/GetEscID.hpp>
#include <com/hobbywing/esc/StatusMsg1.hpp>
#include <com/hobbywing/esc/StatusMsg2.hpp>
#include <com/hobbywing/esc/StatusMsg3.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/esc_report.h>
#include <uORB/topics/actuator_armed.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <drivers/drv_hrt.h>
#include "../node_info.hpp"

/**
 * DroneCAN ESC controller for HobbyWing ESCs.
 *
 * Exposes the same public interface as UavcanEscController so it can be
 * used interchangeably by UavcanMixingInterfaceHobbyWingESC.
 */
class UavcanHobbyWingEscController
{
public:
	static constexpr int     MAX_ACTUATORS = esc_status_s::CONNECTED_ESC_MAX;
	static constexpr unsigned MAX_RATE_HZ  = 200;
	static constexpr uint8_t  SLOT_UNMAPPED = 0xFF;

	UavcanHobbyWingEscController(uavcan::INode &node);
	~UavcanHobbyWingEscController() = default;

	int  init();
	bool initialized() const { return _initialized; }

	void update_outputs(float outputs[MAX_ACTUATORS], uint8_t output_array_size);
	void set_rotor_count(uint8_t count);
	void set_node_info_publisher(NodeInfoPublisher *publisher) { _node_info_publisher = publisher; }

	static int max_output_value() { return 8191; }  ///< int14 max

	esc_status_s &esc_status() { return _esc_status; }

private:
	void handle_GetEscID(const uavcan::ReceivedDataStructure<com::hobbywing::esc::GetEscID> &msg);
	void handle_StatusMsg1(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg1> &msg);
	void handle_StatusMsg2(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg2> &msg);
	void handle_StatusMsg3(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg3> &msg);
	void get_esc_id_timer_cb(const uavcan::TimerEvent &);

	uint16_t check_escs_status();
	int8_t   node_id_to_slot_idx(uint8_t node_id) const;

	typedef uavcan::MethodBinder<UavcanHobbyWingEscController *,
		void (UavcanHobbyWingEscController::*)(const uavcan::ReceivedDataStructure<com::hobbywing::esc::GetEscID> &)>
		GetEscIDCbBinder;

	typedef uavcan::MethodBinder<UavcanHobbyWingEscController *,
		void (UavcanHobbyWingEscController::*)(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg1> &)>
		StatusMsg1CbBinder;

	typedef uavcan::MethodBinder<UavcanHobbyWingEscController *,
		void (UavcanHobbyWingEscController::*)(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg2> &)>
		StatusMsg2CbBinder;

	typedef uavcan::MethodBinder<UavcanHobbyWingEscController *,
		void (UavcanHobbyWingEscController::*)(const uavcan::ReceivedDataStructure<com::hobbywing::esc::StatusMsg3> &)>
		StatusMsg3CbBinder;

	typedef uavcan::MethodBinder<UavcanHobbyWingEscController *,
		void (UavcanHobbyWingEscController::*)(const uavcan::TimerEvent &)>
		TimerCbBinder;

	bool _initialized{false};

	/// CAN node ID -> 0-based motor slot (SLOT_UNMAPPED until GetEscID discovery)
	uint8_t _node_id_to_slot[128];

	esc_status_s _esc_status{};
	uORB::PublicationMulti<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};
	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};

	uint8_t           _rotor_count{0};
	uavcan::MonotonicTime _prev_cmd_pub;

	uavcan::INode &_node;

	// Publishers
	uavcan::Publisher<com::hobbywing::esc::RawCommand> _pub_raw_cmd;
	uavcan::Publisher<com::hobbywing::esc::GetEscID>   _pub_get_esc_id;

	// Subscribers
	uavcan::Subscriber<com::hobbywing::esc::GetEscID,   GetEscIDCbBinder>   _sub_get_esc_id;
	uavcan::Subscriber<com::hobbywing::esc::StatusMsg1, StatusMsg1CbBinder> _sub_status_msg1;
	uavcan::Subscriber<com::hobbywing::esc::StatusMsg2, StatusMsg2CbBinder> _sub_status_msg2;
	uavcan::Subscriber<com::hobbywing::esc::StatusMsg3, StatusMsg3CbBinder> _sub_status_msg3;

	/// Periodic timer: broadcasts GetEscID at 1 Hz while disarmed
	uavcan::TimerEventForwarder<TimerCbBinder> _get_esc_id_timer;

	NodeInfoPublisher *_node_info_publisher{nullptr};
};

#endif // CONFIG_UAVCAN_HOBBYWING_ESC
