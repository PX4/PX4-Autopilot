/****************************************************************************
 *
 *   Copyright (C) 2024 PX4 Development Team. All rights reserved.
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

#include "remoteid.hpp"
#include <lib/open_drone_id/open_drone_id_translations.hpp>

UavcanRemoteIDController::UavcanRemoteIDController(uavcan::INode &node) :
	ModuleParams(nullptr),
	_timer(node),
	_node(node),
	_uavcan_pub_remoteid_basicid(node)
{
}

int UavcanRemoteIDController::init()
{
	// Setup timer and call back function for periodic updates
	_timer.setCallback(TimerCbBinder(this, &UavcanRemoteIDController::periodic_update));
	_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000 / MAX_RATE_HZ));
	return 0;
}

void UavcanRemoteIDController::periodic_update(const uavcan::TimerEvent &)
{
	if (!_vehicle_status_sub.update()) {
		return;
	}

	dronecan::remoteid::BasicID basic_id {};
	// basic_id.id_or_mac // supposedly only used for drone ID data from other UAs
	basic_id.id_type = dronecan::remoteid::BasicID::ODID_ID_TYPE_SERIAL_NUMBER;
	basic_id.ua_type = static_cast<uint8_t>(open_drone_id_translations::odidTypeForMavType(
			_vehicle_status_sub.get().system_type));

	// uas_id: UAS (Unmanned Aircraft System) ID following the format specified by id_type
	// TODO: MAV_ODID_ID_TYPE_SERIAL_NUMBER needs to be ANSI/CTA-2063 format

	char uas_id[20] = {};
	board_get_px4_guid_formated((char *)(uas_id), sizeof(uas_id));
	basic_id.uas_id = uas_id;

	_uavcan_pub_remoteid_basicid.broadcast(basic_id);
}
