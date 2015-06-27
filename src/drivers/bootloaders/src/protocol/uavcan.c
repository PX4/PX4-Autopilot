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
typedef struct packed_struct dsdl_t {
	uint16_t dtid;
	uint16_t signature_crc16;
	uint8_t intail;
	uint8_t outtail;
	uint8_t mailbox;
	uint8_t fifo;
	uint8_t packsize;
	uint8_t service: 1;
} dsdl_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Forward declaration */
extern const dsdl_t dsdl_table[SizeDSDL];
static uint8_t this_node_id = 0;

/****************************************************************************
 * Public Data
 ****************************************************************************/
uint8_t g_uavcan_priority = PriorityAllocation;

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static const dsdl_t *load_dsdl_protocol(uavcan_dsdl_t dsdl, uavcan_direction_t req_in_not_out_resp,
					uavcan_protocol_t *protocol)
{
	const dsdl_t dsdl_def =  dsdl_table[dsdl];

	/* Common initialization for Messages and services */

	protocol->msg.priority = g_uavcan_priority;
	protocol->msg.service_not_message = dsdl_def.service;

	/* Preserver the transfer_id */

	protocol->tail_init.u8 &= uavcan_protocol_mask(UavCanTransferID);

	/*
	 * Based on the direction a request in or out,
	 * Set the sot, eot flags based on if we are sending the fist frame, or
	 * only frame or expecting the the fist frame, or only frame
	 */
	protocol->tail_init.u8 |= (req_in_not_out_resp ? dsdl_def.intail : dsdl_def.outtail);


	/* Specific initialization for Messages and services */

	if (dsdl_def.service) {

		/* Specific initialization for a Services */

		protocol->ser.type_id = dsdl_def.dtid;
		protocol->ser.request_not_response = req_in_not_out_resp;


		if (req_in_not_out_resp) {

			/* If the transfer is in bound this node must be the
			 * dest_node_id
			 */

			protocol->ser.dest_node_id = this_node_id;

		} else {

			/* If the transfer is out bound this node must be the
			 * source_node_id
			 */

			protocol->ser.source_node_id = this_node_id;
		}

	} else  {

		/* Specific initialization for a Message */

		/* is it anonymous? */

		if (this_node_id == ANY_NODE_ID) {

			protocol->ana.type_id = dsdl_def.dtid;

		} else {

			protocol->msg.type_id = dsdl_def.dtid;
		}

		protocol->msg.source_node_id = this_node_id;
	}

	return &dsdl_table[dsdl];
}


/****************************************************************************
 * Name: uavcan_tx_multiframe
 *
 * Description:
 *   This function sends a uavcan service protocol transfer
 *
 * Input Parameters:
 *   protocol   - A pointer to a uavcan_protocol_t to configure the send
 *   transfer    - A pointer to the packed data of the transfer to be sent.
 *   length     - The number of bytes of data
 *   signature  - The initial value used the transfer CRC calculation.
 *   mailbox    - A can_fifo_mailbox_t MBxxx value to choose the outgoing
 *                mailbox.
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/

static void uavcan_tx_multiframe(uavcan_protocol_t *protocol,
				 const uint8_t *transfer,
				 size_t transfer_length,
				 uint16_t signature,
				 uint8_t mailbox)
{
	TODO(Check these 2 lines not needed)
	protocol->tail.sot = true;
	protocol->tail.toggle = false;

	uint8_t payload[CanPayloadLength];

	/*
	 *  Only prepend CRC if the transfer will not fit within a single frame
	 */
	uint32_t m = 0;

	if (transfer_length > MaxUserPayloadLength) {
		uint16_t transfer_crc = crc16_signature(signature, transfer_length, transfer);
		payload[PayloadOffsetCRClsb] = (uint8_t)transfer_crc;
		payload[PayloadOffsetCRCmsb] = (uint8_t)(transfer_crc >> 8u);
		m = PayloadOffsetCRCdata;
	}

	uint32_t i;

	for (i = 0; i < transfer_length; i++) {

		payload[m++] = transfer[i];

		/* Is this the last byte? */

		protocol->tail.eot = (i == (transfer_length - 1));

		/* Either end of user portion of payload or last byte */
		if (m == MaxUserPayloadLength || protocol->tail.eot) {

			uavcan_tx(protocol, payload, m, mailbox);

			/* Increment 1 bit frame sequence number */

			protocol->tail.toggle ^= true;

			m = 0;
		}
	}

}

/****************************************************************************
 * Name: uavcan_rx_multiframe
 *
 * Description:
 *   This function receives a uavcan service protocol transfer
 *
 * Input Parameters:
 *   protocol   - A pointer to a uavcan_protocol_t to configure the send
 *   transfer    - A pointer to the packed data of the transfer to be sent.
 *   length     - The number of bytes of data
 *   signature  - The initial value used the transfer CRC calculation.
 *   mailbox    - A can_fifo_mailbox_t MBxxx value to choose the outgoing
 *                mailbox.
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/

static uavcan_error_t uavcan_rx_multiframe(uavcan_protocol_t *protocol,
		uint8_t *transfer,
		size_t *transfer_length,
		uint16_t initial_crc,
		uint32_t timeout_ms, uint8_t mailbox)
{


	bl_timer_id timer = timer_allocate(modeTimeout | modeStarted, timeout_ms, 0);

	/*
	 *  Set up comparison masks
	 *  In the tail we care about the initial state sot, eot and toggle
	 *  In the CAN ID We care about the Service Type ID, RequestNotResponse,
	 *  ServiceNotMessage, and the destination id
	 *  If the source id is not ANY_NODE_ID, that that must match too.
	 */

	uint8_t  tail_compare_mask =  uavcan_protocol_mask(UavCanStartOfTransfer)
				      | uavcan_protocol_mask(UavCanEndOfTransfer)
				      | uavcan_protocol_mask(UavCanToggle);

	uint32_t  id_compare_mask =  uavcan_protocol_mask(UavCanServiceTypeID)
				     | uavcan_protocol_mask(UavCanServiceRequestNotResponse)
				     | uavcan_protocol_mask(UavCanServiceDestinationNodeID)
				     | uavcan_protocol_mask(UavCanServiceServiceNotMessage);


	if (protocol->msg.source_node_id != ANY_NODE_ID) {

		/* Match source node id */

		id_compare_mask |= uavcan_protocol_mask(UavCanServiceSourceNodeID);
	}


	uint16_t transfer_crc = 0;
	size_t i = 0;
	uavcan_protocol_t rx_protocol;

	do {
		uint8_t payload[CanPayloadLength];
		size_t rx_length;

		if (!uavcan_rx(&rx_protocol, payload, &rx_length, mailbox) ||
		    ((rx_protocol.id.u32 ^ protocol->id.u32) & id_compare_mask) ||
		    ((rx_protocol.tail_init.u8 ^ protocol->tail_init.u8) & tail_compare_mask)) {
			continue;
		}

		/*
		If this is the first frame, capture the actual source node ID and
		transfer ID for future comparisons
		*/

		size_t payload_index = 0;

		if (rx_protocol.tail.sot) {

			/*
			 * This is the start of transfer
			 */

			/* Knock down this flag because all frames after this
			 * one should not have sot
			 */
			protocol->tail.sot = false;

			/* Capture the source node id */

			protocol->ser.source_node_id = rx_protocol.ser.source_node_id;

			/* From now on match source node and transfer ID too */

			id_compare_mask |= uavcan_protocol_mask(UavCanServiceSourceNodeID);
			tail_compare_mask |= uavcan_protocol_mask(UavCanTransferID);

			timer_restart(timer, UavcanServiceTimeOutMs);

			/* if multi-frame snap crc */
			if (protocol->tail.eot == false) {

				/* Capture the frame crc */

				transfer_crc = uavcan_make_uint16(payload[PayloadOffsetCRClsb], payload[PayloadOffsetCRCmsb]);

				/* Payload index is past the crc */

				payload_index = PayloadOffsetCRCdata;

			}

		}

		/* Copy transfer bytes to the transfer */

		if (i + rx_length < *transfer_length) {
			memcpy(&transfer[i], &payload[payload_index], rx_length);
		}

		/* Increment 1 bit frame sequence number */

		protocol->tail.toggle ^= true;

		/* Is this the end of the transfer ?*/

		if (rx_protocol.tail.eot) {
			break;
		}
	} while (!timer_expired(timer));

	timer_free(timer);

	if (!rx_protocol.tail.eot) {

		return UavcanError;

	} else {

		/* Validate CRC */

		uint16_t calculated_crc = crc16_signature(initial_crc, i, transfer);

		*transfer_length = i;

		/* Is it a single frame Transfer not needed  */
		if (SingleFrameTailInit == dsdl_table[protocol->ser.type_id].intail
		    || transfer_crc == calculated_crc) {
			return UavcanOk;

		} else {
			return UavcanError;
		}
	}
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: uavcan_set_node_id
 *
 * Description:
 *   This function sets the node id to be used for out going transfers as
 *   the source node id and for incoming transfers as the destination node id
 *
 * Input Parameters:
 *   node_id    - The the node_id
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/

void uavcan_set_node_id(uint8_t node_id)
{
	this_node_id = node_id;
}

/****************************************************************************
 * Name: uavcan_pack_NodeStatus
 *
 * Description:
 *   This function formats the data of a uavcan_NodeStatus_t structure into
 *   an array of bytes.
 *
 * Input Parameters:
 *   external   -       The array of bytes to populate.
 *   internal   -       The uavcan_NodeStatus_t to pack into the external
 *                      byte representation
 *
 * Returned value:
 *   Number of bytes written.
 *
 ****************************************************************************/

size_t uavcan_pack_NodeStatus(uint8_t *external,
			      const uavcan_NodeStatus_t *internal)
{
	uint32_t uptime_sec_status_code = uavcan_ppack(internal, NodeStatus, uptime_sec) \
					  | uavcan_ppack(internal, NodeStatus, status_code);
	external[0] = ((uint8_t *)&uptime_sec_status_code)[0];
	external[1] = ((uint8_t *)&uptime_sec_status_code)[1];
	external[2] = ((uint8_t *)&uptime_sec_status_code)[2];
	external[3] = ((uint8_t *)&uptime_sec_status_code)[3];
	external[4] = ((uint8_t *)&internal->vendor_specific_status_code)[0];
	external[5] = ((uint8_t *)&internal->vendor_specific_status_code)[1];
	external[6] = internal->msb_vendor_specific_status_code;
	return PackedSizeNodeStatus;
}

/****************************************************************************
 * Name: uavcan_pack_GetNodeInfo_response
 *
 * Description:
 *   This function packs the data of a uavcan_NodeStatus_t into
 *   a uavcan_GetNodeInfo_response_t structure as array of bytes.
 *   Then it packs the uavcan_GetNodeInfo_response_t
 *
 * Input Parameters:
 *   response   The uavcan_GetNodeInfo_response_t to be packed
 *   node_status - The uavcan_NodeStatus_t that is part of the composition
 *
 * Returned value:
 *   Number of bytes written.
 *
 ****************************************************************************/

size_t uavcan_pack_GetNodeInfo_response(uavcan_GetNodeInfo_response_t *response,
					const uavcan_NodeStatus_t *node_status)
{
	uavcan_pack_NodeStatus(response->nodestatus, node_status);
	size_t contiguous_length = FixedSizeGetNodeInfo + \
				   response->hardware_version.certificate_of_authenticity_length;
	/*
	 * Move name so it's contiguous with the end of certificate_of_authenticity[length]
	 * which very well may be just after certificate_of_authenticity_length if the length
	 * of the certificate_of_authenticity is 0
	 */
	memcpy(&((uint8_t *)response)[contiguous_length], response->name, response->name_length);
	return contiguous_length + response->name_length;

}

/****************************************************************************
 * Name: uavcan_pack_LogMessage
 *
 * Description:
 *   This function formats the data of a uavcan_logmessage_t structure into
 *   an array of bytes.
 *
 * Input Parameters:
 *   external    - The array of bytes to populate.
 *   internal - The uavcan_logmessage_t to pack into the data
 *
 * Returned value:
 *   Number of bytes written.
 *
 ****************************************************************************/

static size_t uavcan_pack_LogMessage(uint8_t *external,
				     const uavcan_LogMessage_t *internal)
{
	/* Pack the 3 bit level in top bits followed by the length of source */
	external[uavcan_offset(LogMessage, level)] = uavcan_ppack(internal, LogMessage, level) \
			|  uavcan_pack(uavcan_count(LogMessage, source), LogMessage, source_length);
	memcpy(&external[uavcan_offset(LogMessage, source)], internal->source,
	       PackedSizeLogMessage - sizeof_member(uavcan_LogMessage_t, level));
	return PackedSizeLogMessage;
}
/****************************************************************************
 * Name: uavcan_rx_service
 *
 * Description:
 *
 * Input Parameters:
 *   dsdl       - An Uavcan DSDL Identifier (Auto Generated)
 *   protocol   - A pointer to a uavcan_protocol_t to configure the receive,
 *                based the dsdl for the DTID Service.
 *                If the request must come from a specific server
 *                then protocol->ser.source_node_id, should be set
 *                to that node id;
 *
 *   transfer    - A pointer to the packed data of the transfer to be sent.
 *   length     - The number of bytes of data
 *   timeout_ms - The amount of time in mS to wait for the initial transfer
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/
can_error_t uavcan_rx_service(uavcan_dsdl_t dsdl, uavcan_protocol_t *protocol,
			      uint8_t *transfer, size_t *transfer_length,
			      uint32_t timeout_ms)
{

	const dsdl_t *pdsdl = load_dsdl_protocol(dsdl, InBound, protocol);
	return uavcan_rx_multiframe(protocol, transfer, transfer_length, pdsdl->signature_crc16, timeout_ms, pdsdl->mailbox);
}

/****************************************************************************
 * Name: uavcan_tx_service_request
 *
 * Description:
 *   This helper function sends a uavcan service protocol request, it
 *   assumes the protocol object has the destination node id set.
 *
 * Input Parameters:
 *   dsdl       - An Uavcan DSDL Identifier (Auto Generated)
 *   protocol   - A pointer to a uavcan_protocol_t to configure the send,
 *                the transfer with the dest_node_id set to that of the
 *                node we are making the request of.
 *   transfer    - A pointer to the packed data of the transfer to be sent.
 *   length     - The number of bytes of data
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/

void uavcan_tx_service_request(uavcan_dsdl_t dsdl, uavcan_protocol_t *protocol,
			       const uint8_t *transfer, size_t transfer_length)
{

	const dsdl_t *pdsdl = load_dsdl_protocol(dsdl, OutBound, protocol);
	uavcan_tx_multiframe(protocol, transfer, transfer_length, pdsdl->signature_crc16,  pdsdl->mailbox);
}

/****************************************************************************
 * Name: uavcan_tx_service_response
 *
 * Description:
 *   This helper function sends a uavcan service protocol response, it
 *   assumes the protocol object is from a valid received request and will
 *   apply the incoming source node ID as the outgoing destination node id;
 *
 * Input Parameters:
 *   dsdl       - An Uavcan DSDL Identifier (Auto Generated)
 *   protocol   - A pointer to a uavcan_protocol_t to configure the send,
 *                based on a previous request.
 *   transfer    - A pointer to the packed data of the transfer to be sent.
 *   length     - The number of bytes of data
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/

void uavcan_tx_service_response(uavcan_dsdl_t dsdl, uavcan_protocol_t *protocol,
				const uint8_t *transfer, size_t length)
{

	/* Turn the Transfer around  */

	protocol->ser.dest_node_id = protocol->ser.source_node_id;
	uavcan_tx_service_request(dsdl, protocol, transfer, length);
}

/****************************************************************************
 * Name: uavcan_match
 *
 * Description:
 *   This function test the  uavcan_protocol_t object to see if
 *   it is a match with the dsdl
 *
 * Input Parameters:
 *   dsdl         -     An uavcan DSDL Identifier (Auto Generated)
 *   protocol     -     A pointer to a uavcan_protocol_t to test
 *
 *   node_id      -     The node id to match the dest_node_id
 *                      against for a service or the source node id
 *                      for a transfer. A value of ANY_NODE_ID
 *                      match Any node id;
 *
 * Returned value:
 *   Non Zero if they match otherwise zero.
 *
 ****************************************************************************/

uint8_t uavcan_match(uavcan_dsdl_t dsdl, uavcan_protocol_t *protocol,
		     uint8_t node_id)
{
	uint8_t rv = false;
	const dsdl_t *pdsdl = &dsdl_table[dsdl];

	if (pdsdl->service) {

		rv = (pdsdl->dtid == protocol->ser.type_id
		      && pdsdl->service == protocol->ser.service_not_message
		      && (node_id == ANY_NODE_ID || node_id == protocol->ser.dest_node_id)
		      && pdsdl->intail == uavcan_exclude(protocol->tail_init.u8, UavCanTransferID));

	} else {

		rv = (pdsdl->dtid == protocol->msg.type_id
		      && pdsdl->service == protocol->ser.service_not_message
		      && (node_id == ANY_NODE_ID || node_id == protocol->msg.source_node_id)
		      && pdsdl->intail == uavcan_exclude(protocol->tail_init.u8, UavCanTransferID));
	}

	return rv;

}

/****************************************************************************
 * Name: uavcan_is_anonymous
 *
 * Description:
 *   This function test the  uavcan_protocol_t object to t see if
 *   it is an anonymous frame of a given type ID
 *
 * Input Parameters:
 *   protocol         -  A pointer to a uavcan_protocol_t to test
 *   expected_type_id - The expected uavcan type ID
 *
 *
 * Returned value:
 *   Non Zero if they match otherwise zero.
 *
 ****************************************************************************/

uint8_t uavcan_is_anonymous(uavcan_protocol_t *protocol, uint16_t expected_type_id)
{
	return (protocol->ana.service_not_message == false
		&& UavcanAnonymousNodeID == protocol->ana.source_node_id
		&& protocol->ana.type_id == expected_type_id);

}

/****************************************************************************
 * Name: uavcan_is_allocation
 *
 * Description:
 *   This function test the  uavcan_protocol_t object to t see if
 *   it is a Allocation Message frame
 *
 * Input Parameters:
 *   protocol     -     A pointer to a uavcan_protocol_t to test
 *
 *
 * Returned value:
 *   Non Zero if they match otherwise zero.
 *
 ****************************************************************************/

uint8_t uavcan_is_allocation(uavcan_protocol_t *protocol)
{
	return (uavcan_is_anonymous(protocol, DTIDAllocation) ||
		uavcan_match(DTIDAllocation, protocol,  ANY_NODE_ID));
}

/****************************************************************************
 * Name: uavcan_tx
 *
 * Description:
 *   This function sends a single uavcan protocol based frame applying
 *   the tail byte.
 *
 * Input Parameters:
 *   protocol   - A pointer to a uavcan_protocol_t to configure the send
 *   frame_data - A pointer to 8 bytes of data to be sent (all 8 bytes will be
 *                loaded into the CAN transmitter but only length bytes will
 *                be sent.
 *   length     - The number of bytes of the frame_date (DLC field)
 *   mailbox    - A can_fifo_mailbox_t MBxxx value to choose the outgoing
 *                mailbox.
 *
 * Returned value:
 *   None
 ****************************************************************************/

void uavcan_tx(uavcan_protocol_t *protocol, uint8_t *frame_data,
	       size_t length, uint8_t mailbox)
{
	frame_data[length++] = protocol->tail_init.u8;
	can_tx(protocol->id.u32, length, frame_data, mailbox);
}


/****************************************************************************
 * Name: uavcan_rx
 *
 * Description:
 *   This function is called to receive a single CAN frame from the supplied
 *   fifo. It does not block if there is not available, but returns 0
 *
 * Input Parameters:
 *   protocol   - A pointer to a uavcan_protocol_t to return the CAN ID and
 *                Tail byte
 *   frame_data - A pointer to return 8 bytes of the frame's data
 *                (all 8 bytes will be read from the CAN fifo but
 *                 only length bytes will valid.
 *   length     - A pointer to return the number of bytes of the frame_data
 *                (DLC field)
 *   fifo         A can_fifo_mailbox_t fifixxx value to choose the incoming fifo.
 *
 * Returned value:
 *   The length of the data read or 0 if the fifo was empty
 *
 ****************************************************************************/

uint8_t uavcan_rx(uavcan_protocol_t *protocol, uint8_t *frame_data,
		  size_t *length,  uint8_t fifo)
{
	uint8_t rv = can_rx(&protocol->id.u32, length, frame_data, fifo);

	if (rv) {
		protocol->tail_init.u8  = frame_data[*length - 1];
	}

	return rv;
}


/****************************************************************************
 * Name: uavcan_tx_nodestatus
 *
 * Description:
 *   This function sends a uavcan nodestatus transfer
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
	static uint8_t transfer_id;

	uavcan_NodeStatus_t transfer;
	uavcan_protocol_t protocol;

	protocol.tail.transfer_id = transfer_id++;

	const dsdl_t *dsdl = load_dsdl_protocol(DSDLNodeStatus, MessageOut, &protocol);


	transfer.uptime_sec = uptime_sec;
	transfer.status_code = status_code;
	transfer.vendor_specific_status_code = 0u;
	transfer.msb_vendor_specific_status_code = 0u;

	uint8_t payload[CanPayloadLength];

	size_t frame_len = uavcan_pack_NodeStatus(payload, &transfer);

	uavcan_tx(&protocol, payload, frame_len, dsdl->mailbox);
}

/****************************************************************************
 * Name: uavcan_tx_log_message
 *
 * Description:
 *   This functions sends uavcan LogMessage type data. The Source will be
 *   taken from the application defined debug_log_source
 *
 * Input Parameters:
 *   level   - Log Level of the LogMessage Constants DEBUG, INFO, WARN, ERROR
 *   stage   - The Stage the application is at. see Aplication defined
 *             LOGMESSAGE_STAGE_x
 *   status  - The status of that stage. See Application defined
 *             LOGMESSAGE_RESULT_x
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
void uavcan_tx_log_message(uavcan_LogMessageConsts_t level, uint8_t stage,
			   uint8_t status)
{
	static uint8_t transfer_id;

	uavcan_LogMessage_t message;
	uavcan_protocol_t protocol;

	protocol.tail.transfer_id = transfer_id++;

	const dsdl_t *dsdl = load_dsdl_protocol(DSDLLogMessage, MessageOut, &protocol);


	message.level = level;
	memcpy(&message.source, debug_log_source, sizeof(message.source));
	message.text[0] = stage;
	message.text[1] = status;

	uint8_t payload[CanPayloadLength];
	size_t frame_len = uavcan_pack_LogMessage(payload, &message);
	uavcan_tx(&protocol, payload, frame_len, dsdl->mailbox);
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
 *   random             - The value to use as the discriminator of the
 *                        anonymous message
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/

void uavcan_tx_allocation_message(uint8_t requested_node_id,
				  size_t unique_id_length,
				  const uint8_t *unique_id,
				  uint8_t unique_id_offset,
				  uint16_t random)
{
	static uint8_t transfer_id;

	uavcan_protocol_t protocol;

	protocol.tail.transfer_id = transfer_id++;

	const dsdl_t *dsdl = load_dsdl_protocol(DSDLAllocation, MessageOut, &protocol);

	/* Override defaults */

	protocol.ana.discriminator = random;

	uint8_t payload[CanPayloadLength];

	size_t max_copy = MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST;
	size_t remaining = unique_id_length - unique_id_offset;

	if (max_copy > remaining) {
		max_copy = remaining;
	}

	payload[uavcan_offset(Allocation, node_id)] = uavcan_pack(requested_node_id, Allocation, node_id) |
			(unique_id_offset ? 0 : uavcan_mask(Allocation, first_part_of_unique_id));
	/*
	 * Copy in the remaining bytes of payload, either filling it
	 * or on the last chunk partially filling it
	 */
	memcpy(&payload[uavcan_offset(Allocation, unique_id)], &unique_id[unique_id_offset], max_copy);

	/* Account for the payload[0] */
	max_copy++;

	uavcan_tx(&protocol, payload, max_copy, dsdl->mailbox);

}

/****************************************************************************
 * Name: uavcan_rx_beginfirmwareupdate_request
 *
 * Description:
 *   This function attempts to receive a uavcan beginfirmwareupdate request
 *   transfer
 *
 * Input Parameters:
 *   protocol   - A pointer to a uavcan_protocol_t to return the CAN ID and
 *                Tail byte
 *   request  - A pointer a uavcan_beginfirmwareupdate_request_t to
 *              receive the request data into.
 *
 *
 * Returned value:
 *   Length of the path data on Success or a negative uavcan_error_t
 *   otherwise.
 *
 ****************************************************************************/

ssize_t uavcan_rx_beginfirmwareupdate_request(uavcan_protocol_t *protocol,
		uavcan_BeginFirmwareUpdate_request *request)
{
	static uint8_t transfer_id;


	ssize_t length = sizeof(uavcan_BeginFirmwareUpdate_request);

	protocol->tail.transfer_id = transfer_id++;
	protocol->ser.source_node_id = ANY_NODE_ID;

	uavcan_error_t status = uavcan_rx_service(DSDLBeginFirmwareUpdate, protocol,
				(uint8_t *)request,
				(size_t *) &length, UavcanServiceTimeOutMs);

	if (status == UavcanOk) {

		if (length > uavcan_length(BeginFirmwareUpdate, source_node_id)) {

			length = (length - uavcan_length(BeginFirmwareUpdate, source_node_id));

		} else {
			length = -UavcanError;
		}

	} else {
		length = -status;
	}

	return length;
}

/****************************************************************************
 * Private Data - This table is positioned here to not mess up the line
 * numbers for the debugger.
 ****************************************************************************/

#define NA 0
#define UAVCAN_DSDL_TYPE_DEF(name, dtid, service, signature, packsize, mailbox, fifo, inbound, outbound) \
	{dtid, signature, inbound, outbound, mailbox, fifo, packsize, service},
#define UAVCAN_DSDL_BIT_DEF(data_typ_name, field_name, lsb_pos, length, payload_offset, payload_length)
const dsdl_t dsdl_table[SizeDSDL] = {
#include "uavcan_dsdl_defs.h"
};
#undef UAVCAN_DSDL_TYPE_DEF
#undef UAVCAN_DSDL_BIT_DEF
#undef NA
