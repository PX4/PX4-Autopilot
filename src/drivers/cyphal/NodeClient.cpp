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
 * @file NodeClient.cpp
 *
 * Defines basic implementation of client UAVCAN PNP for requesting a Node ID
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#define PNP_UNIQUE_ID_SIZE 16 // 128 bit unique id

#include <crc64.h>
#include "NodeClient.hpp"

void NodeClient::callback(const CanardRxTransfer &receive)
{
	if (receive.metadata.remote_node_id != CANARD_NODE_ID_UNSET && _canard_handle.node_id() == CANARD_NODE_ID_UNSET) {

		int32_t allocated = CANARD_NODE_ID_UNSET;
		px4_guid_t px4_guid;
		board_get_px4_guid(px4_guid);

		if (_canard_handle.mtu() == CANARD_MTU_CAN_FD) {
			uavcan_pnp_NodeIDAllocationData_2_0 msg;

			size_t msg_size_in_bytes = receive.payload_size;
			uavcan_pnp_NodeIDAllocationData_2_0_deserialize_(&msg, (const uint8_t *)receive.payload,
					&msg_size_in_bytes);

			if (memcmp(msg.unique_id, px4_guid, sizeof(msg.unique_id)) == 0) {
				allocated = msg.node_id.value;
			}

		} else {
			uavcan_pnp_NodeIDAllocationData_1_0 msg;

			size_t msg_size_in_bytes = receive.payload_size;
			uavcan_pnp_NodeIDAllocationData_1_0_deserialize_(&msg, (const uint8_t *)receive.payload,
					&msg_size_in_bytes);

			if (msg.allocated_node_id.count > 0) {
				if (msg.unique_id_hash == (crc64(px4_guid, PNP_UNIQUE_ID_SIZE) & 0xFFFFFFFFFFFF)) {
					allocated = msg.allocated_node_id.elements[0].value;
				}
			}
		}

		if (allocated == CANARD_NODE_ID_UNSET) {
			return;        // UID mismatch.
		}

		if (allocated <= 0 || allocated >= (int32_t)CANARD_NODE_ID_MAX)
			// Allocated node-ID ignored because it exceeds max_node_id
		{
			return;
		}

		_canard_handle.set_node_id(allocated);

		PX4_INFO("Allocated Node ID %d", _canard_handle.node_id());

	}
}


void NodeClient::update()
{
	if (hrt_elapsed_time(&_nodealloc_request_last) >= hrt_abstime(2 *
			1000000ULL)) { // Compiler hates me here, some 1_s doesn't work

		int32_t result;

		// Allocation already done, nothing to do
		if (_canard_handle.node_id() != CANARD_NODE_ID_UNSET) {
			return;
		}

		if (_canard_handle.mtu() == CANARD_MTU_CAN_FD) {
			// NodeIDAllocationData message
			uavcan_pnp_NodeIDAllocationData_2_0 node_id_alloc_msg;
			uint8_t node_id_alloc_payload_buffer[PNP2_PAYLOAD_SIZE];
			size_t payload_size = PNP2_PAYLOAD_SIZE;

			px4_guid_t px4_guid;
			board_get_px4_guid(px4_guid);
			memcpy(node_id_alloc_msg.unique_id, px4_guid, sizeof(node_id_alloc_msg.unique_id));
			//node_id_alloc_msg.node_id.value = preffered_node_id; //FIXME preffered ID PX4 Param

			const CanardTransferMetadata transfer_metadata = {
				.priority       = CanardPriorityNominal,
				.transfer_kind  = CanardTransferKindMessage,
				.port_id        = PNP2_PORT_ID,                // This is the subject-ID.
				.remote_node_id = CANARD_NODE_ID_UNSET,       // Messages cannot be unicast, so use UNSET.
				.transfer_id    = _node_id_alloc_transfer_id,
			};

			result = uavcan_pnp_NodeIDAllocationData_2_0_serialize_(&node_id_alloc_msg, (uint8_t *)&node_id_alloc_payload_buffer,
					&payload_size);

			if (result == 0) {
				// set the data ready in the buffer and chop if needed
				++_node_id_alloc_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
				_canard_handle.TxPush(hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
						      &transfer_metadata,
						      payload_size,
						      &node_id_alloc_payload_buffer);
			}

		} else {
			// NodeIDAllocationData message
			uavcan_pnp_NodeIDAllocationData_1_0 node_id_alloc_msg;
			uavcan_pnp_NodeIDAllocationData_1_0_initialize_(&node_id_alloc_msg);
			uint8_t node_id_alloc_payload_buffer[PNP1_PAYLOAD_SIZE];
			size_t payload_size = PNP1_PAYLOAD_SIZE;

			px4_guid_t px4_guid;
			board_get_px4_guid(px4_guid);
			node_id_alloc_msg.unique_id_hash = (crc64(px4_guid, PNP_UNIQUE_ID_SIZE) & 0xFFFFFFFFFFFF);

			const CanardTransferMetadata transfer_metadata = {
				.priority       = CanardPriorityNominal,
				.transfer_kind  = CanardTransferKindMessage,
				.port_id        = PNP1_PORT_ID,                // This is the subject-ID.
				.remote_node_id = CANARD_NODE_ID_UNSET,       // Messages cannot be unicast, so use UNSET.
				.transfer_id    = _node_id_alloc_transfer_id,
			};

			result = uavcan_pnp_NodeIDAllocationData_1_0_serialize_(&node_id_alloc_msg, (uint8_t *)&node_id_alloc_payload_buffer,
					&payload_size);

			if (result == 0) {
				// set the data ready in the buffer and chop if needed
				++_node_id_alloc_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
				_canard_handle.TxPush(hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
						      &transfer_metadata,
						      payload_size,
						      &node_id_alloc_payload_buffer);
			}
		}

		_nodealloc_request_last = hrt_absolute_time();
	}
}
