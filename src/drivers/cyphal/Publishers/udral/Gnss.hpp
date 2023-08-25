/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file Gnss.hpp
 *
 * Defines basic functionality of Cyphal GNSS publisher
 *
 * @author Jacob Crabill <jacob@flyvoly.com>
 */

#pragma once

// UDRAL Specification Messages
#include <reg/udral/physics/kinematics/geodetic/Point_0_1.h>

#include "../Publisher.hpp"

class UavcanGnssPublisher : public UavcanPublisher
{
public:
	UavcanGnssPublisher(CanardHandle &handle, UavcanParamManager &pmgr, uint8_t instance = 0) :
		UavcanPublisher(handle, pmgr, "udral", "gps", instance)
	{

	};

	~UavcanGnssPublisher() override = default;

	// Update the uORB Subscription and broadcast a UAVCAN message
	virtual void update() override
	{
		if (_gps_sub.updated() && _port_id != CANARD_PORT_ID_UNSET) {
			sensor_gps_s gps {};
			_gps_sub.update(&gps);
			size_t payload_size = reg_udral_physics_kinematics_geodetic_Point_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;

			reg_udral_physics_kinematics_geodetic_Point_0_1 geo {};
			geo.latitude = (int64_t)(gps.latitude_deg / 1e7);
			geo.longitude = (int64_t)(gps.longitude_deg / 1e7);
			geo.altitude = uavcan_si_unit_length_WideScalar_1_0 { .meter = gps.altitude_msl_m };

			uint8_t geo_payload_buffer[reg_udral_physics_kinematics_geodetic_Point_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

			const CanardTransferMetadata transfer_metadata = {
				.priority       = CanardPriorityNominal,
				.transfer_kind  = CanardTransferKindMessage,
				.port_id        = _port_id, // This is the subject-ID.
				.remote_node_id = CANARD_NODE_ID_UNSET,
				.transfer_id    = _transfer_id,
			};

			int32_t result = reg_udral_physics_kinematics_geodetic_Point_0_1_serialize_(&geo, geo_payload_buffer,
					 &payload_size);

			if (result == 0) {
				// set the data ready in the buffer and chop if needed
				++_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
				result = _canard_handle.TxPush(hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
							       &transfer_metadata,
							       payload_size,
							       &geo_payload_buffer);
			}
		}
	};

private:

	/// TODO: Allow >1 instance
	uORB::Subscription _gps_sub{ORB_ID(sensor_gps)};
	CanardTransferID _transfer_id_2 {0};
};
