/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *       Author: Ben Dyer <ben_dyer@mac.com>
 *               Pavel Kirienko <pavel.kirienko@zubax.com>
 *               David Sidrane <david_s5@nscdg.com>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "boot_config.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "chip.h"
#include "stm32.h"

#include "timer.h"
#include "uavcan.h"
#include "can.h"
#include "crc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CAN_REQUEST_TIMEOUT 1000

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: uavcan_tx_multiframe_
 ****************************************************************************/
static void uavcan_tx_multiframe_(uavcan_frame_id_t *frame_id,
				  uint8_t dest_node_id,
				  size_t message_length,
				  const uint8_t *message,
				  uint16_t initial_crc,
				  uint8_t mailbox)
{
	uint32_t i, m;
	uint16_t frame_crc;
	uint8_t payload[8];

	/* Calculate the message CRC */
	frame_crc = crc16_signature(initial_crc, message_length, message);

	/* Ensure message ID has the frame details zeroed */
	frame_id->last_frame = 0u;
	frame_id->frame_index = 0u;

	/*
	Send the message -- only prepend CRC if the message will not fit within a
	single frame
	*/
	payload[0] = dest_node_id;
	m = 1u;

	if (message_length > 7u) {
		payload[m++] = (uint8_t)frame_crc;
		payload[m++] = (uint8_t)(frame_crc >> 8u);
	}

	for (i = 0u; i < message_length; i++) {
		payload[m++] = message[i];

		if (i == message_length - 1u) {
			break;

		} else if (m == 8u) {
			can_tx(uavcan_make_service_frame_id(frame_id), 8u, payload, mailbox);
			frame_id->frame_index++;
			payload[0] = dest_node_id;
			m = 1u;
		}
	}

	/* Send the last (only?) frame */
	frame_id->last_frame = 1u;
	can_tx(uavcan_make_service_frame_id(frame_id), m, payload, mailbox);
}

/****************************************************************************
 * Name: uavcan_rx_multiframe_
 ****************************************************************************/

static can_error_t uavcan_rx_multiframe_(uint8_t node_id,
		uavcan_frame_id_t *frame_id,
		size_t *message_length,
		uint8_t *message,
		uint16_t initial_crc,
		uint32_t timeout_ms)
{
	size_t rx_length;
	size_t m;
	size_t i;
	uint32_t rx_message_id, compare_message_id, compare_mask;
	uint16_t calculated_crc;
	uint16_t message_crc;
	uint8_t payload[8];
	uint8_t got_frame;

	bl_timer_id timer = timer_allocate(modeTimeout | modeStarted, timeout_ms, 0);

	/*
	Match priority, service type ID, request/response flag, frame index.
	*/
	compare_mask = 0x3FFE03F0u;

	if (frame_id->source_node_id) {
		/* Match source node and transfer ID too */
		compare_mask |= 0x1FC07u;
	}

	frame_id->frame_index = 0;
	compare_message_id = uavcan_make_service_frame_id(frame_id);

	message_crc = 0u;
	i = 0;

	do {
		got_frame = can_rx(&rx_message_id, &rx_length, payload, MBAll);

		if (!got_frame || payload[0] != node_id ||
		    ((rx_message_id ^ compare_message_id) & compare_mask)) {
			continue;
		}

		uavcan_parse_frame_id(frame_id, rx_message_id, 0u);

		/*
		If this is the first frame, capture the actual source node ID and
		transfer ID for future comparisons
		*/
		if (frame_id->frame_index == 0u) {
			compare_message_id = uavcan_make_service_frame_id(frame_id);
			compare_mask |= 0x1FC07u;
		}

		/*
		Get the CRC and increase the service timeout if this is the first
		frame
		*/
		if (frame_id->frame_index == 0u && !frame_id->last_frame) {
			timer_restart(timer, UAVCAN_SERVICE_TIMEOUT_MS);
			message_crc = (uint16_t)(payload[1] | (payload[2] << 8u));
			m = 3u;

		} else {
			m = 1u;
		}

		/* Copy message bytes to the response */
		for (; m < rx_length && i < *message_length; m++, i++) {
			message[i] = payload[m];
		}

		/* Increment frame index for next comparison */
		compare_message_id += 0x10u;

		if (frame_id->last_frame) {
			break;
		}
	} while (!timer_expired(timer));

	timer_free(timer);

	if (!frame_id->last_frame) {
		return CAN_ERROR;

	} else {
		/* Validate CRC */
		calculated_crc = crc16_signature(initial_crc, i, message);

		*message_length = i;

		if (frame_id->frame_index == 0u || message_crc == calculated_crc) {
			return CAN_OK;

		} else {
			return CAN_ERROR;
		}
	}
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uavcan_pack_nodestatus
 *
 * Description:
 *   This function formats the data of a uavcan_nodestatus_t structure into
 *   an array of bytes.
 *
 * Input Parameters:
 *   data    - The array of bytes to populate.
 *   message - The uavcan_nodestatus_t to pack into the data
 *
 * Returned value:
 *   Number of bytes written.
 *
 ****************************************************************************/

size_t uavcan_pack_nodestatus(uint8_t *data,
			      const uavcan_nodestatus_t *message)
{
	/* No need to clear the top 4 bits of uptime_sec */
	data[0] = ((uint8_t *)&message->uptime_sec)[0];
	data[1] = ((uint8_t *)&message->uptime_sec)[1];
	data[2] = ((uint8_t *)&message->uptime_sec)[2];
	data[3] = ((uint8_t *)&message->uptime_sec)[3] | message->status_code;
	data[4] = ((uint8_t *)&message->vendor_specific_status_code)[0];
	data[5] = ((uint8_t *)&message->vendor_specific_status_code)[1];
	data[6] = message->msb_vendor_specific_status_code;
	return UAVCAN_NODESTATUS_STATUS_PACKED_SIZE;
}


/****************************************************************************
 * Name: uavcan_pack_logmessage
 *
 * Description:
 *   This function formats the data of a uavcan_logmessage_t structure into
 *   an array of bytes.
 *
 * Input Parameters:
 *   data    - The array of bytes to populate.
 *   message - The uavcan_logmessage_t to pack into the data
 *
 * Returned value:
 *   Number of bytes written.
 *
 ****************************************************************************/

size_t uavcan_pack_logmessage(uint8_t *data,
			      const uavcan_logmessage_t *message)
{
	data[0] = (uint8_t)(((message->level & 0x7u) << 5u) | 4u);
	data[1] = UAVCAN_LOGMESSAGE_SOURCE_0;
	data[2] = UAVCAN_LOGMESSAGE_SOURCE_1;
	data[3] = UAVCAN_LOGMESSAGE_SOURCE_2;
	data[4] = UAVCAN_LOGMESSAGE_SOURCE_3;
	data[5] = message->message[0];
	data[6] = message->message[1];
	return 7u;
}


/****************************************************************************
 * Name: uavcan_make_service_frame_id
 *
 * Description:
 *   This function formats the data of a uavcan_frame_id_t structure
 *   into a unit32.
 *
 * Input Parameters:
 *   frame_id - The uavcan_frame_id_t to pack.
 *
 * Returned value:
 *   A unit32 that is the frame id formed from packing the uavcan_frame_id_t.
 *
 ****************************************************************************/

uint32_t uavcan_make_service_frame_id(const uavcan_frame_id_t *frame_id)
{
	uint32_t id;

	id = PRIORITY_SERVICE << 27u;
	id |= frame_id->transfer_id & 0x7u;
	id |= frame_id->last_frame ? 0x8u : 0u;
	id |= frame_id->frame_index << 4u;
	id |= frame_id->source_node_id << 10u;
	id |= frame_id->data_type_id << 17u;
	id |= frame_id->request_not_response ? 0x4000000u : 0u;

	return id;
}

/****************************************************************************
 * Name: uavcan_parse_frame_id
 *
 * Description:
 *   This function formats the data of a uavcan frame id contained in a
 *   uint32 into uavcan_frame_id_t structure.
 *
 * Input Parameters:
 *   out_frame_id     - A pointer to a uavcan_frame_id_t parse the
 *                      frame id into.
 *   frame_id         - The frame id to parse into the
 *                      uavcan_frame_id_t
 *   expected_type_id - The expected uavcan type ID that has been
 *                      parsed into out_frame_id's data_type_id field
 *
 * Returned value:
 *   The result of comparing the data_type_id to the expected_type_id
 *   Non Zero if they match otherwise zero.
 *
 ****************************************************************************/

int uavcan_parse_frame_id(uavcan_frame_id_t *out_frame_id, uint32_t frame_id,
			  uint16_t expected_type_id)
{
	out_frame_id->transfer_id = frame_id & 0x7u;
	out_frame_id->last_frame = (frame_id & 0x8u) ? 1u : 0u;
	out_frame_id->priority = (uavcan_transferpriority_t)(frame_id >> 27u);

	if (out_frame_id->priority == PRIORITY_SERVICE) {
		out_frame_id->frame_index = (frame_id >> 4u) & 0x3Fu;
		out_frame_id->source_node_id = (frame_id >> 10u) & 0x7Fu;
		out_frame_id->data_type_id = (frame_id >> 17u) & 0x1FFu;
		out_frame_id->request_not_response =
			(frame_id & 0x4000000u) ? 1u : 0u;
		out_frame_id->broadcast_not_unicast = 0u;

	} else {
		out_frame_id->frame_index = (frame_id >> 4u) & 0xFu;
		out_frame_id->broadcast_not_unicast = (frame_id & 0x100u) ? 1u : 0u;
		out_frame_id->request_not_response = 0u;
		out_frame_id->source_node_id = (frame_id >> 9u) & 0x7Fu;
		out_frame_id->data_type_id = (frame_id >> 16u) & 0x7FFu;
	}

	return expected_type_id == out_frame_id->data_type_id;
}

/****************************************************************************
 * Name: uavcan_tx_nodestatus
 *
 * Description:
 *   This function sends a uavcan nodestatus message
 *
 * Input Parameters:
 *   node_id     - This node's node id
 *   uptime_sec  - This node's uptime in seconds.
 *   status_code - This node's current status code
 *
 * Returned value:
 *   None.
 *
 ****************************************************************************/

void uavcan_tx_nodestatus(uint8_t node_id, uint32_t uptime_sec,
			  uint8_t status_code)
{
	uavcan_nodestatus_t message;
	const uint32_t frame_id = (PRIORITY_NORMAL << 27u) |
				  (UAVCAN_NODESTATUS_DTID << 16u) | 0x108u;
	static uint8_t transfer_id;
	uint8_t payload[8];
	size_t frame_len;

	message.uptime_sec = uptime_sec;
	message.status_code = status_code;
	message.vendor_specific_status_code = 0u;
	message.msb_vendor_specific_status_code = 0u;
	frame_len = uavcan_pack_nodestatus(payload, &message);

	can_tx(frame_id | (node_id << 9u) | (transfer_id++ & 0x7u), frame_len,
	       payload, MBNodeStatus);
}

/****************************************************************************
 * Name: uavcan_tx_log_message
 *
 * Description:
 *   This functions sends uavcan logmessage type data. See uavcan/protocol.h
 *   UAVCAN_LOGMESSAGE_xxx defines.
 *
 * Input Parameters:
 *   node_id - This node's node id
 *   level   - Log Level of the logmessage DEBUG, INFO, WARN, ERROR
 *   stage   - The Stage the application is at. see UAVCAN_LOGMESSAGE_STAGE_x
 *   status  - The status of that stage. Start, Fail OK
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
void uavcan_tx_log_message(uint8_t node_id, uint8_t level, uint8_t stage,
			   uint8_t status)
{
	uavcan_logmessage_t message;
	const uint32_t frame_id = (PRIORITY_HIGH << 27u) |
				  (UAVCAN_LOGMESSAGE_DTID << 16u) | 0x108u;
	static uint8_t transfer_id;
	uint8_t payload[8];
	size_t frame_len;

	message.level = level;
	message.message[0] = stage;
	message.message[1] = status;
	frame_len = uavcan_pack_logmessage(payload, &message);
	can_tx(frame_id | (node_id << 9u) | (transfer_id++ & 0x7u), frame_len,
	       payload, MBAll);
}

/****************************************************************************
 * Name: uavcan_tx_allocation_message
 *
 * Description:
 *   This function sends a uavcan allocation message.
 *
 * Input Parameters:
 *   requested_node_id - This node's preferred node id 0 for no preference.
 *   unique_id_length  - This node's length of it's unique identifier.
 *   unique_id         - A pointer to the bytes that represent unique
 *                       identifier.
 *   unique_id_offset  - The offset equal 0 or the number of bytes in the
 *                       the last received message that matched the unique ID
 *                       field.
 *
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/

void uavcan_tx_allocation_message(uint8_t requested_node_id,
				  size_t unique_id_length,
				  const uint8_t *unique_id,
				  uint8_t unique_id_offset)
{
	uint8_t payload[8];
	const uint32_t frame_id = (PRIORITY_NORMAL << 27u) |
				  (UAVCAN_DYNAMICNODEIDALLOCATION_DTID << 16u) |
				  (1u << 8u);
	size_t i;
	size_t max_offset;
	uint8_t checksum;

	max_offset = unique_id_offset + 7u;

	if (max_offset > unique_id_length) {
		max_offset = unique_id_length;
	}

	payload[0] = (uint8_t)((requested_node_id << 1u) |
			       (unique_id_offset ? 0u : 1u));

	for (checksum = payload[0], i = 0u; i < max_offset - unique_id_offset; i++) {
		payload[i + 1u] = unique_id[unique_id_offset + i];
		checksum += unique_id[unique_id_offset + i];
	}

	can_tx(frame_id | checksum, i + 1u, payload, MBAll);
}

/****************************************************************************
 * Name: uavcan_tx_getnodeinfo_response
 *
 * Description:
 *   This function sends a uavcan getnodeinfo response to a getnodeinfo
 *   request message.
 *
 * Input Parameters:
 *   node_id      - This node's node id
 *   response     - A pointer to this node's uavcan_getnodeinfo_response_t
 *                  response data.
 *   dest_node_id - The destination node id to send the message to.
 *   transfer_id  - An incrementing count used to correlate a received
 *                  message to this transmitted message.
 *
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/

void uavcan_tx_getnodeinfo_response(uint8_t node_id,
				    uavcan_getnodeinfo_response_t *response,
				    uint8_t dest_node_id,
				    uint8_t transfer_id)
{
	/*
	This sends via mailbox 1 because it's called from SysTick. It may also
	clobber response->name because it moves the name to the end of the COA.
	*/
	uavcan_frame_id_t frame_id;
	size_t fixed_length;
	size_t contiguous_length;
	size_t packet_length;

	fixed_length = UAVCAN_NODESTATUS_STATUS_PACKED_SIZE + sizeof(uavcan_softwareversion_t) + 2u + 16u + 1u;
	contiguous_length = fixed_length +
			    response->hardware_version.certificate_of_authenticity_length;
	packet_length = contiguous_length + response->name_length;

	/* Move name so it's contiguous with the start of the packet */
	memcpy(&((uint8_t *)response)[contiguous_length], response->name, response->name_length);

	/* Set up the message ID */
	frame_id.transfer_id = transfer_id;
	frame_id.source_node_id = node_id;
	frame_id.request_not_response = 0u;
	frame_id.data_type_id = UAVCAN_GETNODEINFO_DTID;
	frame_id.priority = PRIORITY_SERVICE;

	uavcan_tx_multiframe_(&frame_id, dest_node_id, packet_length,
			      (const uint8_t *)response, UAVCAN_GETNODEINFO_CRC,
			      MBGetNodeInfo);
}

/****************************************************************************
 * Name: uavcan_rx_beginfirmwareupdate_request
 *
 * Description:
 *   This function attempts to receive a uavcan beginfirmwareupdate request
 *   message
 *
 * Input Parameters:
 *   node_id  - This node's node id
 *   request  - A pointer a uavcan_beginfirmwareupdate_request_t to
 *              receive the request data into.
 *   frame_id - A pointer to a uavcan_frame_id_t to return frame_id
 *              components in.
 *
 *
 * Returned value:
 *   CAN_OK on Success and CAN_ERROR otherwise.
 *
 ****************************************************************************/

can_error_t uavcan_rx_beginfirmwareupdate_request(uint8_t node_id,
		uavcan_beginfirmwareupdate_request_t *request,
		uavcan_frame_id_t *frame_id)
{
	size_t length;
	can_error_t status;

	length = sizeof(uavcan_beginfirmwareupdate_request_t) - 1;
	frame_id->source_node_id = 0;
	frame_id->request_not_response = 1u;
	frame_id->data_type_id = UAVCAN_BEGINFIRMWAREUPDATE_DTID;

	status = uavcan_rx_multiframe_(node_id, frame_id, &length,
				       (uint8_t *)request,
				       UAVCAN_BEGINFIRMWAREUPDATE_CRC, CAN_REQUEST_TIMEOUT);

	if (status == CAN_OK && length >= 1u) {
		request->path_length = (uint8_t)(length - 1u);
		return CAN_OK;

	} else {
		return CAN_ERROR;
	}
}

/****************************************************************************
 * Name: uavcan_tx_read_request
 *
 * Description:
 *   This function sends a uavcan read request message.
 *
 * Input Parameters:
 *   node_id      - This node's node id
 *   request      - A pointer a uavcan_read_request_t to
 *                  send.
 *   transfer_id  - An incrementing count used to correlate a received
 *                  message to this transmitted message.
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/

void uavcan_tx_read_request(uint8_t node_id,
			    const uavcan_read_request_t *request,
			    uint8_t dest_node_id,
			    uint8_t transfer_id)
{
	uavcan_frame_id_t frame_id;

	/* Set up the message ID */
	frame_id.transfer_id = transfer_id;
	frame_id.source_node_id = node_id;
	frame_id.request_not_response = 1u;
	frame_id.data_type_id = UAVCAN_READ_DTID;
	frame_id.priority = PRIORITY_SERVICE;

	uavcan_tx_multiframe_(&frame_id, dest_node_id, request->path_length + UAVCAN_FILE_READ_REQUEST_FIXED_SIZE,
			      (const uint8_t *)request, UAVCAN_READ_CRC, MBAll);
}


/****************************************************************************
 * Name: uavcan_rx_read_response
 *
 * Description:
 *   This function attempts to receive a uavcan read response message.
 *
 * Input Parameters:
 *   node_id      - This node's node id
 *   response     - A pointer a uavcan_read_response_t to receive the
 *                  response data into.
 *   dest_node_id - The remote node id to expect the message from.
 *   transfer_id  - The expected transfer_id used to correlate this response
 *                  message to the transmitted request message.
 *   timeout_ms -   The number of milliseconds to wait for a response
 *
 *
 * Returned value:
 *   CAN_OK on Success and CAN_ERROR otherwise.
 *
 ****************************************************************************/

can_error_t uavcan_rx_read_response(uint8_t node_id,
				    uavcan_read_response_t *response,
				    uint8_t dest_node_id,
				    uint8_t transfer_id,
				    uint32_t timeout_ms)
{
	uavcan_frame_id_t frame_id;
	size_t length;
	can_error_t status;

	length = sizeof(uavcan_read_response_t) - sizeof_member(uavcan_read_response_t, data_length);
	frame_id.transfer_id = transfer_id;
	frame_id.source_node_id = dest_node_id;
	frame_id.request_not_response = 0u;
	frame_id.data_type_id = UAVCAN_READ_DTID;

	status = uavcan_rx_multiframe_(node_id, &frame_id, &length,
				       (uint8_t *)response, UAVCAN_READ_CRC,
				       timeout_ms);

	if (status == CAN_OK && length >= sizeof_member(uavcan_read_response_t, error)) {
		response->data_length = (uint16_t)(length - sizeof_member(uavcan_read_response_t, error));
		return CAN_OK;

	} else {
		return CAN_ERROR;
	}
}

/****************************************************************************
 * Name: uavcan_tx_getinfo_request
 *
 * Description:
 *   This function sends a uavcan getinfo request message.
 *
 * Input Parameters:
 *   node_id      - This node's node id
 *   request      - A pointer a uavcan_getinfo_request_t to
 *                  send.
 *   dest_node_id - The destination node id to send the message to.
 *   transfer_id  - An incrementing count used to correlate a received
 *                  message to this transmitted message.
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/

void uavcan_tx_getinfo_request(uint8_t node_id,
			       const uavcan_getinfo_request_t *request,
			       uint8_t dest_node_id,
			       uint8_t transfer_id)
{
	uavcan_frame_id_t frame_id;

	/* Set up the message ID */
	frame_id.transfer_id = transfer_id;
	frame_id.source_node_id = node_id;
	frame_id.request_not_response = 1u;
	frame_id.data_type_id = UAVCAN_GETINFO_DTID;
	frame_id.priority = PRIORITY_SERVICE;

	uavcan_tx_multiframe_(&frame_id, dest_node_id, request->path_length,
			      (const uint8_t *)request, UAVCAN_GETINFO_CRC, MBAll);
}

/****************************************************************************
 * Name: uavcan_rx_getinfo_response
 *
 * Description:
 *   This function attempts to receive a uavcan getinfo response message.
 *
 * Input Parameters:
 *   node_id      - This node's node id
 *   response     - A pointer a uavcan_getinfo_response_t to receive the
 *                  response data into.
 *   dest_node_id - The remote node id to expect the message from.
 *   transfer_id  - The expected transfer_id used to correlate this response
 *                  message to the transmitted request message.
 *   timeout_ms -   The number of milliseconds to wait for a response
 *
 *
 * Returned value:
 *   CAN_OK on Success and CAN_ERROR otherwise.
 *
 ****************************************************************************/
can_error_t uavcan_rx_getinfo_response(uint8_t node_id,
				       uavcan_getinfo_response_t *response,
				       uint8_t dest_node_id,
				       uint8_t transfer_id,
				       uint32_t timeout_ms)
{
	uavcan_frame_id_t frame_id;
	size_t length;
	can_error_t status;

	length = sizeof(uavcan_getinfo_response_t);
	frame_id.transfer_id = transfer_id;
	frame_id.source_node_id = dest_node_id;
	frame_id.request_not_response = 0u;
	frame_id.data_type_id = UAVCAN_GETINFO_DTID;

	status = uavcan_rx_multiframe_(node_id, &frame_id, &length,
				       (uint8_t *)response, UAVCAN_GETINFO_CRC,
				       timeout_ms);

	if (status == CAN_OK && length == sizeof(uavcan_getinfo_response_t)) {
		return CAN_OK;

	} else {
		return CAN_ERROR;
	}
}


