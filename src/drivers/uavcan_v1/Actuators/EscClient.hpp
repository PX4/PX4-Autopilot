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
 * @file esc.hpp
 *
 * UAVCAN <--> ORB bridge for ESC messages:
 *     uavcan.equipment.esc.RawCommand
 *     uavcan.equipment.esc.RPMCommand
 *     uavcan.equipment.esc.Status
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <lib/perf/perf_counter.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/esc_status.h>
#include <drivers/drv_hrt.h>
#include <lib/mixer_module/mixer_module.hpp>

#include <reg/drone/service/actuator/common/sp/Vector31_0_1.h>
#include <reg/drone/service/common/Readiness_0_1.h>

class UavcanEscController
{
public:
	static constexpr int MAX_ACTUATORS = MixingOutput::MAX_ACTUATORS;
	static constexpr unsigned MAX_RATE_HZ = 200;			///< XXX make this configurable
	static constexpr uint16_t DISARMED_OUTPUT_VALUE = UINT16_MAX;

	/// TODO: derive from Publisher class to handle port-ID setting via parameter
	UavcanEscController(CanardInstance &ins, const CanardPortID &baseID) :
		_canard_instance(ins), _base_port_id(baseID) {};

	~UavcanEscController() {};

	void update_outputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs)
	{
		if (_base_port_id > 0) {
			/// TODO: Update readiness
			// reg_drone_service_common_Readiness_0_1 msg_arming;
			reg_drone_service_actuator_common_sp_Vector31_0_1 msg_sp {0};

			for (uint8_t i = 0; i < num_outputs; i++) {
				msg_sp.value[i] = outputs[i];
			}

			uint8_t esc_sp_payload_buffer[reg_drone_service_actuator_common_sp_Vector31_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

			CanardTransfer transfer = {
				.timestamp_usec = hrt_absolute_time(),      // Zero if transmission deadline is not limited.
				.priority       = CanardPriorityNominal,
				.transfer_kind  = CanardTransferKindMessage,
				.port_id        = _base_port_id,                // This is the subject-ID.
				.remote_node_id = CANARD_NODE_ID_UNSET,       // Messages cannot be unicast, so use UNSET.
				.transfer_id    = _esc_setpoint_transfer_id,
				.payload_size   = reg_drone_service_actuator_common_sp_Vector31_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
				.payload        = &esc_sp_payload_buffer,
			};

			int result = reg_drone_service_actuator_common_sp_Vector31_0_1_serialize_(&msg_sp, esc_sp_payload_buffer,
					&transfer.payload_size);

			if (result == 0) {
				// set the data ready in the buffer and chop if needed
				++_esc_setpoint_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
				result = canardTxPush(&_canard_instance, &transfer);
			}
		}
	};

	/**
	 * Sets the number of rotors
	 */
	void set_rotor_count(uint8_t count) { _rotor_count = count; }

private:
	/**
	 * ESC status message reception will be reported via this callback.
	 */
	void esc_status_sub_cb(const CanardTransfer &msg);

	CanardInstance &_canard_instance;
	CanardPortID _base_port_id {0};
	uint8_t _rotor_count {0};
	CanardTransferID _esc_setpoint_transfer_id {0};
};
