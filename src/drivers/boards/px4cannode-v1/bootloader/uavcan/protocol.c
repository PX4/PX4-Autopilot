#include <nuttx/config.h>
#include "board_config.h"

#include <stdint.h>
#include <stdlib.h>

#include "chip.h"
#include "stm32.h"

#include "timer.h"
#include "protocol.h"
#include "driver.h"
#include "crc.h"


#define CAN_REQUEST_TIMEOUT 1000

static void uavcan_tx_multiframe_(
    uavcan_frame_id_t *frame_id,
    uint8_t dest_node_id,
    size_t message_length,
    const uint8_t *message,
    uint16_t initial_crc,
    uint8_t mailbox
);
static can_error_t uavcan_rx_multiframe_(
    uint8_t node_id,
    uavcan_frame_id_t *frame_id,
    size_t *message_length,
    uint8_t *message,
    uint16_t initial_crc,
    uint32_t timeout_cycles
);


size_t uavcan_pack_nodestatus(
    uint8_t *data,
    const uavcan_nodestatus_t *payload
) {
    /* No need to clear the top 4 bits of uptime_sec */
    data[0] = ((uint8_t*)&payload->uptime_sec)[0];
    data[1] = ((uint8_t*)&payload->uptime_sec)[1];
    data[2] = ((uint8_t*)&payload->uptime_sec)[2];
    data[3] = ((uint8_t*)&payload->uptime_sec)[3] | payload->status_code;
    data[4] = ((uint8_t*)&payload->vendor_specific_status_code)[0];
    data[5] = ((uint8_t*)&payload->vendor_specific_status_code)[1];
    return 6u;
}


size_t uavcan_pack_logmessage(
    uint8_t *data,
    const uavcan_logmessage_t *payload
) {
    data[0] = (uint8_t)(((payload->level & 0x7u) << 5u) | 4u);
    data[1] = UAVCAN_LOGMESSAGE_SOURCE_0;
    data[2] = UAVCAN_LOGMESSAGE_SOURCE_1;
    data[3] = UAVCAN_LOGMESSAGE_SOURCE_2;
    data[4] = UAVCAN_LOGMESSAGE_SOURCE_3;
    data[5] = payload->message[0];
    data[6] = payload->message[1];
    return 7u;
}


uint32_t uavcan_make_message_id(const uavcan_frame_id_t *frame_id) {
    return (frame_id->transfer_id & 0x7u) |
           ((frame_id->last_frame ? 1u : 0u) << 3u) |
           ((frame_id->frame_index & 0x3Fu) << 4u) |
           ((frame_id->source_node_id & 0x7Fu) << 10u) |
           ((frame_id->transfer_type & 0x3u) << 17u) |
           ((frame_id->data_type_id & 0x3FFu) << 19u);
}


int uavcan_parse_message_id(uavcan_frame_id_t *frame_id, uint32_t message_id,
                            uint16_t expected_id)
{
    frame_id->transfer_id = message_id & 0x7u;
    frame_id->last_frame = (message_id & 0x8u) ? 1u : 0u;
    frame_id->frame_index = (message_id >> 4u) & 0x3Fu;
    frame_id->source_node_id = (message_id >> 10u) & 0x7Fu;
    frame_id->transfer_type = (message_id >> 17u) & 0x3u;
    frame_id->data_type_id = (message_id >> 19u) & 0x3FFu;
    return expected_id == frame_id->data_type_id;
}


void uavcan_tx_nodestatus(
    uint8_t node_id,
    uint32_t uptime_sec,
    uint8_t status_code
) {
    uavcan_nodestatus_t message;
    uavcan_frame_id_t frame_id;
    uint8_t payload[8];
    size_t frame_len;

    frame_id.transfer_id = 0;
    frame_id.last_frame = 1u;
    frame_id.frame_index = 0;
    frame_id.source_node_id = node_id;
    frame_id.transfer_type = MESSAGE_BROADCAST;
    frame_id.data_type_id = UAVCAN_NODESTATUS_DTID;

    message.uptime_sec = uptime_sec;
    message.status_code = status_code;
    message.vendor_specific_status_code = 0u;
    frame_len = uavcan_pack_nodestatus(payload, &message);

    can_tx(uavcan_make_message_id(&frame_id), frame_len, payload, MBNodeStatus);
}


void uavcan_tx_allocation_message(
    uint8_t requested_node_id,
    size_t unique_id_length,
    const uint8_t *unique_id,
    uint8_t unique_id_offset,
    uint8_t transfer_id
) {
    uint8_t payload[8];
    uavcan_frame_id_t frame_id;
    size_t i, max_offset, checksum;

    max_offset = unique_id_offset + 7u;
    if (max_offset > unique_id_length) {
        max_offset = unique_id_length;
    }

    payload[0] = (uint8_t)((requested_node_id << 1u) |
                           (unique_id_offset ? 0u : 1u));
    for (checksum = 0u, i = 0u; i < max_offset - unique_id_offset; i++) {
        payload[i + 1u] = unique_id[unique_id_offset + i];
        checksum += unique_id[unique_id_offset + i];
    }

    frame_id.transfer_id = transfer_id;
    frame_id.last_frame = 1u;
    frame_id.frame_index = (uint8_t)checksum;
    frame_id.source_node_id = 0u;
    frame_id.transfer_type = MESSAGE_BROADCAST;
    frame_id.data_type_id = UAVCAN_DYNAMICNODEIDALLOCATION_DTID;

    can_tx(uavcan_make_message_id(&frame_id), i + 1u,
           payload, MBAll);
}


void uavcan_tx_getnodeinfo_response(
    uint8_t node_id,
    uavcan_getnodeinfo_response_t *response,
    uint8_t dest_node_id,
    uint8_t transfer_id
) {
    /*
    This sends via mailbox 1 because it's called from SysTick. It may also
    clobber response->name because it moves the name to the end of the COA.
    */
    uint32_t i;
    uavcan_frame_id_t frame_id;
    size_t fixed_length, contiguous_length, packet_length;

    fixed_length = 6u + sizeof(uavcan_softwareversion_t) + 2u + 16u + 1u;
    contiguous_length = fixed_length +
        response->hardware_version.certificate_of_authenticity_length;
    packet_length = contiguous_length + response->name_length;

    /* Move name so it's contiguous with the start of the packet */
    for (i = 0u; i < response->name_length; i++) {
        ((uint8_t*)response)[contiguous_length + i] = response->name[i];
    }

    /* Set up the message ID */
    frame_id.transfer_id = transfer_id;
    frame_id.source_node_id = node_id;
    frame_id.transfer_type = SERVICE_RESPONSE;
    frame_id.data_type_id = UAVCAN_GETNODEINFO_DTID;

    uavcan_tx_multiframe_(&frame_id, dest_node_id, packet_length,
                          (const uint8_t*)response, UAVCAN_GETNODEINFO_CRC,
                          1u);
}

can_error_t uavcan_rx_beginfirmwareupdate_request(
    uint8_t node_id,
    uavcan_beginfirmwareupdate_request_t *request,
    uavcan_frame_id_t *frame_id
) {
    size_t length;
    can_error_t status;

    length = sizeof(uavcan_beginfirmwareupdate_request_t) - 1;
    frame_id->transfer_id = 0xFFu;
    frame_id->source_node_id = 0xFFu;
    frame_id->transfer_type = SERVICE_REQUEST;
    frame_id->data_type_id = UAVCAN_BEGINFIRMWAREUPDATE_DTID;

    status = uavcan_rx_multiframe_(node_id, frame_id, &length,
                                   (uint8_t*)request,
                                   UAVCAN_BEGINFIRMWAREUPDATE_CRC, CAN_REQUEST_TIMEOUT);

    if (status == CAN_OK && length >= 1u) {
        request->path_length = (uint8_t)(length - 1u);
        return CAN_OK;
    } else {
        return CAN_ERROR;
    }
}


void uavcan_tx_read_request(
    uint8_t node_id,
    const uavcan_read_request_t *request,
    uint8_t dest_node_id,
    uint8_t transfer_id
) {
    uavcan_frame_id_t frame_id;

    /* Set up the message ID */
    frame_id.transfer_id = transfer_id;
    frame_id.source_node_id = node_id;
    frame_id.transfer_type = SERVICE_REQUEST;
    frame_id.data_type_id = UAVCAN_READ_DTID;

    uavcan_tx_multiframe_(&frame_id, dest_node_id, request->path_length + 4u,
                          (const uint8_t*)request, UAVCAN_READ_CRC, MBAll);
}


can_error_t uavcan_rx_read_response(
    uint8_t node_id,
    uavcan_read_response_t *response,
    uint8_t dest_node_id,
    uint8_t transfer_id,
    uint32_t timeout_ticks
) {
    uavcan_frame_id_t frame_id;
    size_t length;
    can_error_t status;

    length = sizeof(uavcan_read_response_t) - 1;
    frame_id.transfer_id = transfer_id;
    frame_id.source_node_id = dest_node_id;
    frame_id.transfer_type = SERVICE_RESPONSE;
    frame_id.data_type_id = UAVCAN_READ_DTID;

    status = uavcan_rx_multiframe_(node_id, &frame_id, &length,
                                   (uint8_t*)response, UAVCAN_READ_CRC,
                                   timeout_ticks);

    if (status == CAN_OK && length >= 2u) {
        response->data_length = (uint8_t)(length - 2u);
        return CAN_OK;
    } else {
        return CAN_ERROR;
    }
}


void uavcan_tx_getinfo_request(
    uint8_t node_id,
    const uavcan_getinfo_request_t *request,
    uint8_t dest_node_id,
    uint8_t transfer_id
) {
    uavcan_frame_id_t frame_id;

    /* Set up the message ID */
    frame_id.transfer_id = transfer_id;
    frame_id.source_node_id = node_id;
    frame_id.transfer_type = SERVICE_REQUEST;
    frame_id.data_type_id = UAVCAN_GETINFO_DTID;

    uavcan_tx_multiframe_(&frame_id, dest_node_id, request->path_length,
                          (const uint8_t*)request, UAVCAN_GETINFO_CRC, MBAll);
}


can_error_t uavcan_rx_getinfo_response(
    uint8_t node_id,
    uavcan_getinfo_response_t *response,
    uint8_t dest_node_id,
    uint8_t transfer_id,
    uint32_t timeout_ticks
) {
    uavcan_frame_id_t frame_id;
    size_t length;
    can_error_t status;

    length = sizeof(uavcan_getinfo_response_t);
    frame_id.transfer_id = transfer_id;
    frame_id.source_node_id = dest_node_id;
    frame_id.transfer_type = SERVICE_RESPONSE;
    frame_id.data_type_id = UAVCAN_GETINFO_DTID;

    status = uavcan_rx_multiframe_(node_id, &frame_id, &length,
                                   (uint8_t*)response, UAVCAN_GETINFO_CRC,
                                   timeout_ticks);

    if (status == CAN_OK && length == sizeof(uavcan_getinfo_response_t)) {
        return CAN_OK;
    } else {
        return CAN_ERROR;
    }
}


static void uavcan_tx_multiframe_(
    uavcan_frame_id_t *frame_id,
    uint8_t dest_node_id,
    size_t message_length,
    const uint8_t *message,
    uint16_t initial_crc,
    uint8_t mailbox
) {
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
            can_tx(uavcan_make_message_id(frame_id), 8u, payload, mailbox);
            frame_id->frame_index++;
            payload[0] = dest_node_id;
            m = 1u;
        }
    }

    /* Send the last (only?) frame */
    frame_id->last_frame = 1u;
    can_tx(uavcan_make_message_id(frame_id), m, payload, mailbox);
}


static can_error_t uavcan_rx_multiframe_(uint8_t node_id,
                                         uavcan_frame_id_t *frame_id,
                                         size_t *message_length,
                                         uint8_t *message,
                                         uint16_t initial_crc,
                                         uint32_t timeout_ticks)
{
    uavcan_frame_id_t rx_id;
    size_t rx_length;
    size_t m;
    size_t i;
    uint32_t rx_message_id;
    uint16_t calculated_crc;
    uint16_t message_crc;
    uint8_t payload[8];
    uint8_t got_frame;
    uint8_t num_frames;

    bl_timer_id timer = timer_allocate(modeTimeout|modeStarted, timeout_ticks, 0);

    num_frames = 0u;
    rx_id.last_frame = 0u;
    message_crc = 0u;
    i = 0;

    do {
        rx_message_id = 0u;
        rx_length = 0u;
        got_frame = can_rx(&rx_message_id, &rx_length, payload, MBAll);
        if (!got_frame) {
            continue;
        }

        uavcan_parse_message_id(&rx_id, rx_message_id, frame_id->transfer_type);

        /*
        Skip this frame if the source node ID is wrong, or if the data type or
        transfer type do not match what we're expecting
        */
        if ((frame_id->source_node_id != 0xFFu &&
                rx_id.source_node_id != frame_id->source_node_id) ||
                rx_id.transfer_type != frame_id->transfer_type ||
                rx_id.data_type_id != frame_id->data_type_id ||
                payload[0] != node_id) {
            continue;
        }

        /*
        Get the CRC if this is the first frame of a multi-frame transfer.

        Also increase the timeout to UAVCAN_SERVICE_TIMEOUT_TICKS.
        */
        if (rx_id.frame_index == 0u) {
            num_frames = 0u;
            frame_id->transfer_id = rx_id.transfer_id;
            if (frame_id->source_node_id == 0xFFu) {
                frame_id->source_node_id = rx_id.source_node_id;
            }
        }

        /* Skip if the transfer ID is wrong */
        if ((frame_id->transfer_id ^ rx_id.transfer_id) & 0x7u) {
            continue;
        }

        /*
        Get the CRC and increase the service timeout if this is the first
        frame
        */
        if (num_frames == 0u && !rx_id.last_frame) {
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
        num_frames++;

        if (rx_id.last_frame) {
            break;
        }
    } while (!timer_expired(timer));
    timer_free(timer);
    if (!rx_id.last_frame) {
        return CAN_ERROR;
    } else {
        /* Validate CRC */
        calculated_crc = crc16_signature(initial_crc, i, message);

        *message_length = i;

        if (num_frames == 1u || message_crc == calculated_crc) {
            return CAN_OK;
        } else {
            return CAN_ERROR;
        }
    }
}
