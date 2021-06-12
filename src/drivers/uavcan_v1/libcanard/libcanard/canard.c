/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2016-2020 UAVCAN Development Team.
/// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#include "canard.h"
#include <assert.h>
#include <string.h>

// --------------------------------------------- BUILD CONFIGURATION ---------------------------------------------

/// By default, this macro resolves to the standard assert(). The user can redefine this if necessary.
/// To disable assertion checks completely, make it expand into `(void)(0)`.
#ifndef CANARD_ASSERT
// Intentional violation of MISRA: assertion macro cannot be replaced with a function definition.
#    define CANARD_ASSERT(x) assert(x)  // NOSONAR
#endif

/// This macro is needed only for testing and for library development. Do not redefine this in production.
#if defined(CANARD_CONFIG_EXPOSE_PRIVATE) && CANARD_CONFIG_EXPOSE_PRIVATE
#    define CANARD_PRIVATE
#else  // Consider defining an extra compilation option that turns this into "static inline"?
#    define CANARD_PRIVATE static
#endif

#if !defined(__STDC_VERSION__) || (__STDC_VERSION__ < 199901L)
#    error "Unsupported language: ISO C99 or a newer version is required."
#endif

// --------------------------------------------- COMMON CONSTANTS ---------------------------------------------

#define BITS_PER_BYTE 8U
#define BYTE_MAX 0xFFU

#define CAN_EXT_ID_MASK ((UINT32_C(1) << 29U) - 1U)

#define MFT_NON_LAST_FRAME_PAYLOAD_MIN 7U

#define PADDING_BYTE_VALUE 0U

#define OFFSET_PRIORITY 26U
#define OFFSET_SUBJECT_ID 8U
#define OFFSET_SERVICE_ID 14U
#define OFFSET_DST_NODE_ID 7U

#define FLAG_SERVICE_NOT_MESSAGE (UINT32_C(1) << 25U)
#define FLAG_ANONYMOUS_MESSAGE (UINT32_C(1) << 24U)
#define FLAG_REQUEST_NOT_RESPONSE (UINT32_C(1) << 24U)
#define FLAG_RESERVED_23 (UINT32_C(1) << 23U)
#define FLAG_RESERVED_07 (UINT32_C(1) << 7U)

#define TAIL_START_OF_TRANSFER 128U
#define TAIL_END_OF_TRANSFER 64U
#define TAIL_TOGGLE 32U

#define INITIAL_TOGGLE_STATE true

// --------------------------------------------- TRANSFER CRC ---------------------------------------------

typedef uint16_t TransferCRC;

#define CRC_INITIAL 0xFFFFU
#define CRC_RESIDUE 0x0000U
#define CRC_SIZE_BYTES 2U

CANARD_PRIVATE TransferCRC crcAddByte(const TransferCRC crc, const uint8_t byte);
CANARD_PRIVATE TransferCRC crcAddByte(const TransferCRC crc, const uint8_t byte)
{
    static const TransferCRC Top  = 0x8000U;
    static const TransferCRC Poly = 0x1021U;
    TransferCRC              out  = crc ^ (uint16_t)((uint16_t)(byte) << BITS_PER_BYTE);
    // Consider adding a compilation option that replaces this with a CRC table. Adds 512 bytes of ROM.
    // Do not fold this into a loop because a size-optimizing compiler won't unroll it degrading the performance.
    out = (uint16_t)((uint16_t)(out << 1U) ^ (((out & Top) != 0U) ? Poly : 0U));
    out = (uint16_t)((uint16_t)(out << 1U) ^ (((out & Top) != 0U) ? Poly : 0U));
    out = (uint16_t)((uint16_t)(out << 1U) ^ (((out & Top) != 0U) ? Poly : 0U));
    out = (uint16_t)((uint16_t)(out << 1U) ^ (((out & Top) != 0U) ? Poly : 0U));
    out = (uint16_t)((uint16_t)(out << 1U) ^ (((out & Top) != 0U) ? Poly : 0U));
    out = (uint16_t)((uint16_t)(out << 1U) ^ (((out & Top) != 0U) ? Poly : 0U));
    out = (uint16_t)((uint16_t)(out << 1U) ^ (((out & Top) != 0U) ? Poly : 0U));
    out = (uint16_t)((uint16_t)(out << 1U) ^ (((out & Top) != 0U) ? Poly : 0U));
    return out;
}

CANARD_PRIVATE TransferCRC crcAdd(const TransferCRC crc, const size_t size, const void* const data);
CANARD_PRIVATE TransferCRC crcAdd(const TransferCRC crc, const size_t size, const void* const data)
{
    CANARD_ASSERT((data != NULL) || (size == 0U));
    TransferCRC    out = crc;
    const uint8_t* p   = (const uint8_t*) data;
    for (size_t i = 0; i < size; i++)
    {
        out = crcAddByte(out, *p);
        ++p;
    }
    return out;
}

// --------------------------------------------- TRANSMISSION ---------------------------------------------

/// This is a subclass of CanardFrame. A pointer to this type can be cast to CanardFrame safely.
/// This is standard-compliant. The paragraph 6.7.2.1.15 says:
///     A pointer to a structure object, suitably converted, points to its initial member (or if that member is a
///     bit-field, then to the unit in which it resides), and vice versa. There may be unnamed padding within a
///     structure object, but not at its beginning.
typedef struct CanardInternalTxQueueItem
{
    CanardFrame                       frame;
    struct CanardInternalTxQueueItem* next;

    // Intentional violation of MISRA: this flex array is the lesser of three evils. The other two are:
    //  - Make the payload pointer point to the remainder of the allocated memory following this structure.
    //    The pointer is bad because it requires us to use pointer arithmetics.
    //  - Use a separate memory allocation for data. This is terribly wasteful (both time & memory).
    uint8_t payload_buffer[];  // NOSONAR
} CanardInternalTxQueueItem;

CANARD_PRIVATE uint32_t txMakeMessageSessionSpecifier(const CanardPortID subject_id, const CanardNodeID src_node_id);
CANARD_PRIVATE uint32_t txMakeMessageSessionSpecifier(const CanardPortID subject_id, const CanardNodeID src_node_id)
{
    CANARD_ASSERT(src_node_id <= CANARD_NODE_ID_MAX);
    CANARD_ASSERT(subject_id <= CANARD_SUBJECT_ID_MAX);
    const uint32_t tmp = subject_id | (CANARD_SUBJECT_ID_MAX + 1) | ((CANARD_SUBJECT_ID_MAX + 1) * 2);
    return src_node_id | (tmp << OFFSET_SUBJECT_ID);
}

CANARD_PRIVATE uint32_t txMakeServiceSessionSpecifier(const CanardPortID service_id,
                                                      const bool         request_not_response,
                                                      const CanardNodeID src_node_id,
                                                      const CanardNodeID dst_node_id);
CANARD_PRIVATE uint32_t txMakeServiceSessionSpecifier(const CanardPortID service_id,
                                                      const bool         request_not_response,
                                                      const CanardNodeID src_node_id,
                                                      const CanardNodeID dst_node_id)
{
    CANARD_ASSERT(src_node_id <= CANARD_NODE_ID_MAX);
    CANARD_ASSERT(dst_node_id <= CANARD_NODE_ID_MAX);
    CANARD_ASSERT(service_id <= CANARD_SERVICE_ID_MAX);
    return src_node_id | (((uint32_t) dst_node_id) << OFFSET_DST_NODE_ID) |  //
           (((uint32_t) service_id) << OFFSET_SERVICE_ID) |                  //
           (request_not_response ? FLAG_REQUEST_NOT_RESPONSE : 0U) | FLAG_SERVICE_NOT_MESSAGE;
}

/// This is the transport MTU rounded up to next full DLC minus the tail byte.
CANARD_PRIVATE size_t txGetPresentationLayerMTU(const CanardInstance* const ins);
CANARD_PRIVATE size_t txGetPresentationLayerMTU(const CanardInstance* const ins)
{
    const size_t max_index = (sizeof(CanardCANLengthToDLC) / sizeof(CanardCANLengthToDLC[0])) - 1U;
    size_t       mtu       = 0U;
    if (ins->mtu_bytes < CANARD_MTU_CAN_CLASSIC)
    {
        mtu = CANARD_MTU_CAN_CLASSIC;
    }
    else if (ins->mtu_bytes <= max_index)
    {
        mtu = CanardCANDLCToLength[CanardCANLengthToDLC[ins->mtu_bytes]];  // Round up to nearest valid length.
    }
    else
    {
        mtu = CanardCANDLCToLength[CanardCANLengthToDLC[max_index]];
    }
    return mtu - 1U;
}

CANARD_PRIVATE int32_t txMakeCANID(const CanardTransfer* const tr,
                                   const CanardNodeID          local_node_id,
                                   const size_t                presentation_layer_mtu);
CANARD_PRIVATE int32_t txMakeCANID(const CanardTransfer* const tr,
                                   const CanardNodeID          local_node_id,
                                   const size_t                presentation_layer_mtu)
{
    CANARD_ASSERT(tr != NULL);
    CANARD_ASSERT(presentation_layer_mtu > 0);
    int32_t out = -CANARD_ERROR_INVALID_ARGUMENT;
    if ((tr->transfer_kind == CanardTransferKindMessage) && (CANARD_NODE_ID_UNSET == tr->remote_node_id) &&
        (tr->port_id <= CANARD_SUBJECT_ID_MAX))
    {
        if (local_node_id <= CANARD_NODE_ID_MAX)
        {
            out = (int32_t) txMakeMessageSessionSpecifier(tr->port_id, local_node_id);
            CANARD_ASSERT(out >= 0);
        }
        else if (tr->payload_size <= presentation_layer_mtu)
        {
            CANARD_ASSERT((tr->payload != NULL) || (tr->payload_size == 0U));
            const CanardNodeID c =
                (CanardNodeID)(crcAdd(CRC_INITIAL, tr->payload_size, tr->payload) & CANARD_NODE_ID_MAX);
            const uint32_t spec = txMakeMessageSessionSpecifier(tr->port_id, c) | FLAG_ANONYMOUS_MESSAGE;
            CANARD_ASSERT(spec <= CAN_EXT_ID_MASK);
            out = (int32_t) spec;
        }
        else
        {
            out = -CANARD_ERROR_INVALID_ARGUMENT;  // Anonymous multi-frame message trs are not allowed.
        }
    }
    else if (((tr->transfer_kind == CanardTransferKindRequest) || (tr->transfer_kind == CanardTransferKindResponse)) &&
             (tr->remote_node_id <= CANARD_NODE_ID_MAX) && (tr->port_id <= CANARD_SERVICE_ID_MAX))
    {
        if (local_node_id <= CANARD_NODE_ID_MAX)
        {
            out = (int32_t) txMakeServiceSessionSpecifier(tr->port_id,
                                                          tr->transfer_kind == CanardTransferKindRequest,
                                                          local_node_id,
                                                          tr->remote_node_id);
            CANARD_ASSERT(out >= 0);
        }
        else
        {
            out = -CANARD_ERROR_INVALID_ARGUMENT;  // Anonymous service transfers are not allowed.
        }
    }
    else
    {
        out = -CANARD_ERROR_INVALID_ARGUMENT;
    }

    if (out >= 0)
    {
        const uint32_t prio = (uint32_t) tr->priority;
        if (prio <= CANARD_PRIORITY_MAX)
        {
            const uint32_t id = ((uint32_t) out) | (prio << OFFSET_PRIORITY);
            out               = (int32_t) id;
        }
        else
        {
            out = -CANARD_ERROR_INVALID_ARGUMENT;
        }
    }
    return out;
}

CANARD_PRIVATE uint8_t txMakeTailByte(const bool             start_of_transfer,
                                      const bool             end_of_transfer,
                                      const bool             toggle,
                                      const CanardTransferID transfer_id);
CANARD_PRIVATE uint8_t txMakeTailByte(const bool             start_of_transfer,
                                      const bool             end_of_transfer,
                                      const bool             toggle,
                                      const CanardTransferID transfer_id)
{
    CANARD_ASSERT(start_of_transfer ? (toggle == INITIAL_TOGGLE_STATE) : true);
    return (uint8_t)((start_of_transfer ? TAIL_START_OF_TRANSFER : 0U) | (end_of_transfer ? TAIL_END_OF_TRANSFER : 0U) |
                     (toggle ? TAIL_TOGGLE : 0U) | (transfer_id & CANARD_TRANSFER_ID_MAX));
}

/// Takes a frame payload size, returns a new size that is >=x and is rounded up to the nearest valid DLC.
CANARD_PRIVATE size_t txRoundFramePayloadSizeUp(const size_t x);
CANARD_PRIVATE size_t txRoundFramePayloadSizeUp(const size_t x)
{
    CANARD_ASSERT(x < (sizeof(CanardCANLengthToDLC) / sizeof(CanardCANLengthToDLC[0])));
    // Suppressing a false-positive out-of-bounds access error from Sonar. Its control flow analyser is misbehaving.
    const size_t y = CanardCANLengthToDLC[x];  // NOSONAR
    CANARD_ASSERT(y < (sizeof(CanardCANDLCToLength) / sizeof(CanardCANDLCToLength[0])));
    return CanardCANDLCToLength[y];
}

CANARD_PRIVATE CanardInternalTxQueueItem* txAllocateQueueItem(CanardInstance* const   ins,
                                                              const uint32_t          id,
                                                              const CanardMicrosecond deadline_usec,
                                                              const size_t            payload_size);
CANARD_PRIVATE CanardInternalTxQueueItem* txAllocateQueueItem(CanardInstance* const   ins,
                                                              const uint32_t          id,
                                                              const CanardMicrosecond deadline_usec,
                                                              const size_t            payload_size)
{
    CANARD_ASSERT(ins != NULL);
    CANARD_ASSERT(payload_size > 0U);
    CanardInternalTxQueueItem* const out =
        (CanardInternalTxQueueItem*) ins->memory_allocate(ins, sizeof(CanardInternalTxQueueItem) + payload_size);
    if (out != NULL)
    {
        out->next                  = NULL;
        out->frame.timestamp_usec  = deadline_usec;
        out->frame.payload_size    = payload_size;
        out->frame.payload         = out->payload_buffer;
        out->frame.extended_can_id = id;
    }
    return out;
}

/// Returns the element after which new elements with the specified CAN ID should be inserted.
/// Returns NULL if the element shall be inserted in the beginning of the list (i.e., no prior elements).
CANARD_PRIVATE CanardInternalTxQueueItem* txFindQueueSupremum(const CanardInstance* const ins, const uint32_t can_id);
CANARD_PRIVATE CanardInternalTxQueueItem* txFindQueueSupremum(const CanardInstance* const ins, const uint32_t can_id)
{
    CANARD_ASSERT(ins != NULL);
    CANARD_ASSERT(can_id <= CAN_EXT_ID_MASK);
    CanardInternalTxQueueItem* out = ins->_tx_queue;
    if ((NULL == out) || (out->frame.extended_can_id > can_id))
    {
        out = NULL;
    }
    else
    {
        // TODO The linear search should be replaced with O(log n) at least. Please help us here.
        while ((out != NULL) && (out->next != NULL) && (out->next->frame.extended_can_id <= can_id))
        {
            out = out->next;
        }
    }
    CANARD_ASSERT((out == NULL) || (out->frame.extended_can_id <= can_id));
    return out;
}

/// Returns the number of frames enqueued or error (i.e., =1 or <0).
CANARD_PRIVATE int32_t txPushSingleFrame(CanardInstance* const   ins,
                                         const CanardMicrosecond deadline_usec,
                                         const uint32_t          can_id,
                                         const CanardTransferID  transfer_id,
                                         const size_t            payload_size,
                                         const void* const       payload);
CANARD_PRIVATE int32_t txPushSingleFrame(CanardInstance* const   ins,
                                         const CanardMicrosecond deadline_usec,
                                         const uint32_t          can_id,
                                         const CanardTransferID  transfer_id,
                                         const size_t            payload_size,
                                         const void* const       payload)
{
    CANARD_ASSERT(ins != NULL);
    CANARD_ASSERT((payload != NULL) || (payload_size == 0));

    const size_t frame_payload_size = txRoundFramePayloadSizeUp(payload_size + 1U);
    CANARD_ASSERT(frame_payload_size > payload_size);
    const size_t padding_size = frame_payload_size - payload_size - 1U;
    CANARD_ASSERT((padding_size + payload_size + 1U) == frame_payload_size);
    int32_t out = 0;

    CanardInternalTxQueueItem* const tqi = txAllocateQueueItem(ins, can_id, deadline_usec, frame_payload_size);
    if (tqi != NULL)
    {
        if (payload_size > 0U)  // The check is needed to avoid calling memcpy() with a NULL pointer, it's an UB.
        {
            CANARD_ASSERT(payload != NULL);
            // Clang-Tidy raises an error recommending the use of memcpy_s() instead.
            // We ignore it because the safe functions are poorly supported; reliance on them may limit the portability.
            (void) memcpy(&tqi->payload_buffer[0], payload, payload_size);  // NOLINT
        }

        // Clang-Tidy raises an error recommending the use of memset_s() instead.
        // We ignore it because the safe functions are poorly supported; reliance on them may limit the portability.
        (void) memset(&tqi->payload_buffer[payload_size], PADDING_BYTE_VALUE, padding_size);  // NOLINT

        tqi->payload_buffer[frame_payload_size - 1U] = txMakeTailByte(true, true, true, transfer_id);
        CanardInternalTxQueueItem* const sup         = txFindQueueSupremum(ins, can_id);
        if (sup != NULL)
        {
            tqi->next = sup->next;
            sup->next = tqi;
        }
        else
        {
            tqi->next      = ins->_tx_queue;
            ins->_tx_queue = tqi;
        }
        out = 1;  // One frame enqueued.
    }
    else
    {
        out = -CANARD_ERROR_OUT_OF_MEMORY;
    }
    CANARD_ASSERT((out < 0) || (out == 1));
    return out;
}

/// Returns the number of frames enqueued or error.
CANARD_PRIVATE int32_t txPushMultiFrame(CanardInstance* const   ins,
                                        const size_t            presentation_layer_mtu,
                                        const CanardMicrosecond deadline_usec,
                                        const uint32_t          can_id,
                                        const CanardTransferID  transfer_id,
                                        const size_t            payload_size,
                                        const void* const       payload);
CANARD_PRIVATE int32_t txPushMultiFrame(CanardInstance* const   ins,
                                        const size_t            presentation_layer_mtu,
                                        const CanardMicrosecond deadline_usec,
                                        const uint32_t          can_id,
                                        const CanardTransferID  transfer_id,
                                        const size_t            payload_size,
                                        const void* const       payload)
{
    CANARD_ASSERT(ins != NULL);
    CANARD_ASSERT(presentation_layer_mtu > 0U);
    CANARD_ASSERT(payload_size > presentation_layer_mtu);  // Otherwise, a single-frame transfer should be used.
    CANARD_ASSERT(payload != NULL);

    int32_t out = 0;  // The number of frames enqueued or negated error.

    CanardInternalTxQueueItem* head = NULL;  // Head and tail of the linked list of frames of this transfer.
    CanardInternalTxQueueItem* tail = NULL;

    const size_t   payload_size_with_crc = payload_size + CRC_SIZE_BYTES;
    size_t         offset                = 0U;
    TransferCRC    crc                   = crcAdd(CRC_INITIAL, payload_size, payload);
    bool           start_of_transfer     = true;
    bool           toggle                = INITIAL_TOGGLE_STATE;
    const uint8_t* payload_ptr           = (const uint8_t*) payload;

    while (offset < payload_size_with_crc)
    {
        ++out;
        const size_t frame_payload_size_with_tail =
            ((payload_size_with_crc - offset) < presentation_layer_mtu)
                ? txRoundFramePayloadSizeUp((payload_size_with_crc - offset) + 1U)  // Padding in the last frame only.
                : (presentation_layer_mtu + 1U);
        CanardInternalTxQueueItem* const tqi =
            txAllocateQueueItem(ins, can_id, deadline_usec, frame_payload_size_with_tail);
        if (NULL == head)
        {
            head = tqi;
        }
        else
        {
            tail->next = tqi;
        }
        tail = tqi;
        if (NULL == tail)
        {
            break;
        }

        // Copy the payload into the frame.
        const size_t frame_payload_size = frame_payload_size_with_tail - 1U;
        size_t       frame_offset       = 0U;
        if (offset < payload_size)
        {
            size_t move_size = payload_size - offset;
            if (move_size > frame_payload_size)
            {
                move_size = frame_payload_size;
            }
            // Clang-Tidy raises an error recommending the use of memcpy_s() instead.
            // We ignore it because the safe functions are poorly supported; reliance on them may limit the portability.
            (void) memcpy(&tail->payload_buffer[0], payload_ptr, move_size);  // NOLINT
            frame_offset = frame_offset + move_size;
            offset += move_size;
            payload_ptr += move_size;
        }

        // Handle the last frame of the transfer: it is special because it also contains padding and CRC.
        if (offset >= payload_size)
        {
            // Insert padding -- only in the last frame. Don't forget to include padding into the CRC.
            while ((frame_offset + CRC_SIZE_BYTES) < frame_payload_size)
            {
                tail->payload_buffer[frame_offset] = PADDING_BYTE_VALUE;
                ++frame_offset;
                crc = crcAddByte(crc, PADDING_BYTE_VALUE);
            }

            // Insert the CRC.
            if ((frame_offset < frame_payload_size) && (offset == payload_size))
            {
                tail->payload_buffer[frame_offset] = (uint8_t)(crc >> BITS_PER_BYTE);
                ++frame_offset;
                ++offset;
            }
            if ((frame_offset < frame_payload_size) && (offset > payload_size))
            {
                tail->payload_buffer[frame_offset] = (uint8_t)(crc & BYTE_MAX);
                ++frame_offset;
                ++offset;
            }
        }

        // Finalize the frame.
        CANARD_ASSERT((frame_offset + 1U) == tail->frame.payload_size);
        tail->payload_buffer[frame_offset] =
            txMakeTailByte(start_of_transfer, offset >= payload_size_with_crc, toggle, transfer_id);
        start_of_transfer = false;
        toggle            = !toggle;
    }

    if (tail != NULL)
    {
        CANARD_ASSERT(head->next != NULL);  // This is not a single-frame transfer so at least two frames shall exist.
        CANARD_ASSERT(tail->next == NULL);  // The list shall be properly terminated.
        CanardInternalTxQueueItem* const sup = txFindQueueSupremum(ins, can_id);
        if (NULL == sup)  // Once the insertion point is located, we insert the entire frame sequence in constant time.
        {
            tail->next     = ins->_tx_queue;
            ins->_tx_queue = head;
        }
        else
        {
            tail->next = sup->next;
            sup->next  = head;
        }
    }
    else  // Failed to allocate at least one frame in the queue! Remove all frames and abort.
    {
        out = -CANARD_ERROR_OUT_OF_MEMORY;
        while (head != NULL)
        {
            CanardInternalTxQueueItem* const next = head->next;
            ins->memory_free(ins, head);
            head = next;
        }
    }

    CANARD_ASSERT((out < 0) || (out >= 2));
    return out;
}

// --------------------------------------------- RECEPTION ---------------------------------------------

#define RX_SESSIONS_PER_SUBSCRIPTION (CANARD_NODE_ID_MAX + 1U)

/// The memory requirement model provided in the documentation assumes that the maximum size of this structure never
/// exceeds 48 bytes on any conventional platform.
/// A user that needs a detailed analysis of the worst-case memory consumption may compute the size of this structure
/// for the particular platform at hand manually or by evaluating its sizeof().
/// The fields are ordered to minimize the amount of padding on all conventional platforms.
typedef struct CanardInternalRxSession
{
    CanardMicrosecond transfer_timestamp_usec;  ///< Timestamp of the last received start-of-transfer.
    size_t            total_payload_size;       ///< The payload size before the implicit truncation, including the CRC.
    size_t            payload_size;             ///< How many bytes received so far.
    uint8_t*          payload;                  ///< Dynamically allocated and handed off to the application when done.
    TransferCRC       calculated_crc;           ///< Updated with the received payload in real time.
    CanardTransferID  transfer_id;
    uint8_t           redundant_transport_index;  ///< Arbitrary value in [0, 255].
    bool              toggle;
} CanardInternalRxSession;

/// High-level transport frame model.
typedef struct
{
    CanardMicrosecond  timestamp_usec;
    CanardPriority     priority;
    CanardTransferKind transfer_kind;
    CanardPortID       port_id;
    CanardNodeID       source_node_id;
    CanardNodeID       destination_node_id;
    CanardTransferID   transfer_id;
    bool               start_of_transfer;
    bool               end_of_transfer;
    bool               toggle;
    size_t             payload_size;
    const void*        payload;
} RxFrameModel;

/// Returns truth if the frame is valid and parsed successfully. False if the frame is not a valid UAVCAN/CAN frame.
CANARD_PRIVATE bool rxTryParseFrame(const CanardFrame* const frame, RxFrameModel* const out);
CANARD_PRIVATE bool rxTryParseFrame(const CanardFrame* const frame, RxFrameModel* const out)
{
    CANARD_ASSERT(frame != NULL);
    CANARD_ASSERT(frame->extended_can_id <= CAN_EXT_ID_MASK);
    CANARD_ASSERT(out != NULL);
    bool valid = false;
    if (frame->payload_size > 0)
    {
        CANARD_ASSERT(frame->payload != NULL);
        out->timestamp_usec = frame->timestamp_usec;

        // CAN ID parsing.
        const uint32_t can_id = frame->extended_can_id;
        out->priority         = (CanardPriority)((can_id >> OFFSET_PRIORITY) & CANARD_PRIORITY_MAX);
        out->source_node_id   = (CanardNodeID)(can_id & CANARD_NODE_ID_MAX);
        if (0 == (can_id & FLAG_SERVICE_NOT_MESSAGE))
        {
            out->transfer_kind = CanardTransferKindMessage;
            out->port_id       = (CanardPortID)((can_id >> OFFSET_SUBJECT_ID) & CANARD_SUBJECT_ID_MAX);
            if ((can_id & FLAG_ANONYMOUS_MESSAGE) != 0)
            {
                out->source_node_id = CANARD_NODE_ID_UNSET;
            }
            out->destination_node_id = CANARD_NODE_ID_UNSET;
            // Reserved bits may be unreserved in the future.
            valid = (0 == (can_id & FLAG_RESERVED_23)) && (0 == (can_id & FLAG_RESERVED_07));
        }
        else
        {
            out->transfer_kind =
                ((can_id & FLAG_REQUEST_NOT_RESPONSE) != 0) ? CanardTransferKindRequest : CanardTransferKindResponse;
            out->port_id             = (CanardPortID)((can_id >> OFFSET_SERVICE_ID) & CANARD_SERVICE_ID_MAX);
            out->destination_node_id = (CanardNodeID)((can_id >> OFFSET_DST_NODE_ID) & CANARD_NODE_ID_MAX);
            // The reserved bit may be unreserved in the future. It may be used to extend the service-ID to 10 bits.
            // Per Specification, source cannot be the same as the destination.
            valid = (0 == (can_id & FLAG_RESERVED_23)) && (out->source_node_id != out->destination_node_id);
        }

        // Payload parsing.
        out->payload_size = frame->payload_size - 1U;  // Cut off the tail byte.
        out->payload      = frame->payload;

        // Tail byte parsing.
        // Intentional violation of MISRA: pointer arithmetics is required to locate the tail byte. Unavoidable.
        const uint8_t tail     = *(((const uint8_t*) out->payload) + out->payload_size);  // NOSONAR
        out->transfer_id       = tail & CANARD_TRANSFER_ID_MAX;
        out->start_of_transfer = ((tail & TAIL_START_OF_TRANSFER) != 0);
        out->end_of_transfer   = ((tail & TAIL_END_OF_TRANSFER) != 0);
        out->toggle            = ((tail & TAIL_TOGGLE) != 0);

        // Final validation.
        // Protocol version check: if SOT is set, then the toggle shall also be set.
        valid = valid && ((!out->start_of_transfer) || (INITIAL_TOGGLE_STATE == out->toggle));
        // Anonymous transfers can be only single-frame transfers.
        valid = valid &&
                ((out->start_of_transfer && out->end_of_transfer) || (CANARD_NODE_ID_UNSET != out->source_node_id));
        // Non-last frames of a multi-frame transfer shall utilize the MTU fully.
        valid = valid && ((out->payload_size >= MFT_NON_LAST_FRAME_PAYLOAD_MIN) || out->end_of_transfer);
        // A frame that is a part of a multi-frame transfer cannot be empty (tail byte not included).
        valid = valid && ((out->payload_size > 0) || (out->start_of_transfer && out->end_of_transfer));
    }
    return valid;
}

CANARD_PRIVATE void rxInitTransferFromFrame(const RxFrameModel* const frame, CanardTransfer* const out_transfer);
CANARD_PRIVATE void rxInitTransferFromFrame(const RxFrameModel* const frame, CanardTransfer* const out_transfer)
{
    CANARD_ASSERT(frame != NULL);
    CANARD_ASSERT(frame->payload != NULL);
    CANARD_ASSERT(out_transfer != NULL);
    out_transfer->timestamp_usec = frame->timestamp_usec;
    out_transfer->priority       = frame->priority;
    out_transfer->transfer_kind  = frame->transfer_kind;
    out_transfer->port_id        = frame->port_id;
    out_transfer->remote_node_id = frame->source_node_id;
    out_transfer->transfer_id    = frame->transfer_id;
    // Payload not populated.
}

/// The implementation is borrowed from the Specification.
CANARD_PRIVATE uint8_t rxComputeTransferIDDifference(const uint8_t a, const uint8_t b);
CANARD_PRIVATE uint8_t rxComputeTransferIDDifference(const uint8_t a, const uint8_t b)
{
    CANARD_ASSERT(a <= CANARD_TRANSFER_ID_MAX);
    CANARD_ASSERT(b <= CANARD_TRANSFER_ID_MAX);
    int16_t diff = (int16_t)(((int16_t) a) - ((int16_t) b));
    if (diff < 0)
    {
        const uint8_t modulo = 1U << CANARD_TRANSFER_ID_BIT_LENGTH;
        diff                 = (int16_t)(diff + (int16_t) modulo);
    }
    return (uint8_t) diff;
}

CANARD_PRIVATE int8_t rxSessionWritePayload(CanardInstance* const          ins,
                                            CanardInternalRxSession* const rxs,
                                            const size_t                   extent,
                                            const size_t                   payload_size,
                                            const void* const              payload);
CANARD_PRIVATE int8_t rxSessionWritePayload(CanardInstance* const          ins,
                                            CanardInternalRxSession* const rxs,
                                            const size_t                   extent,
                                            const size_t                   payload_size,
                                            const void* const              payload)
{
    CANARD_ASSERT(ins != NULL);
    CANARD_ASSERT(rxs != NULL);
    CANARD_ASSERT((payload != NULL) || (payload_size == 0U));
    CANARD_ASSERT(rxs->payload_size <= extent);  // This invariant is enforced by the subscription logic.
    CANARD_ASSERT(rxs->payload_size <= rxs->total_payload_size);

    rxs->total_payload_size += payload_size;

    // Allocate the payload lazily, as late as possible.
    if ((NULL == rxs->payload) && (extent > 0U))
    {
        CANARD_ASSERT(rxs->payload_size == 0);
        rxs->payload = ins->memory_allocate(ins, extent);
    }

    int8_t out = 0;
    if (rxs->payload != NULL)
    {
        // Copy the payload into the contiguous buffer. Apply the implicit truncation rule if necessary.
        size_t bytes_to_copy = payload_size;
        if ((rxs->payload_size + bytes_to_copy) > extent)
        {
            CANARD_ASSERT(rxs->payload_size <= extent);
            bytes_to_copy = extent - rxs->payload_size;
            CANARD_ASSERT((rxs->payload_size + bytes_to_copy) == extent);
            CANARD_ASSERT(bytes_to_copy < payload_size);
        }
        // This memcpy() call here is one of the two variable-complexity operations in the RX pipeline;
        // the other one is the search of the matching subscription state.
        // Excepting these two cases, the entire RX pipeline contains neither loops nor recursion.
        // Intentional violation of MISRA: indexing on a pointer. This is done to avoid pointer arithmetics.
        // Clang-Tidy raises an error recommending the use of memcpy_s() instead.
        // We ignore it because the safe functions are poorly supported; reliance on them may limit the portability.
        (void) memcpy(&rxs->payload[rxs->payload_size], payload, bytes_to_copy);  // NOLINT NOSONAR
        rxs->payload_size += bytes_to_copy;
        CANARD_ASSERT(rxs->payload_size <= extent);
    }
    else
    {
        CANARD_ASSERT(rxs->payload_size == 0);
        out = (extent > 0U) ? -CANARD_ERROR_OUT_OF_MEMORY : 0;
    }
    CANARD_ASSERT(out <= 0);
    return out;
}

CANARD_PRIVATE void rxSessionRestart(CanardInstance* const ins, CanardInternalRxSession* const rxs);
CANARD_PRIVATE void rxSessionRestart(CanardInstance* const ins, CanardInternalRxSession* const rxs)
{
    CANARD_ASSERT(ins != NULL);
    CANARD_ASSERT(rxs != NULL);
    ins->memory_free(ins, rxs->payload);  // May be NULL, which is OK.
    rxs->total_payload_size = 0U;
    rxs->payload_size       = 0U;
    rxs->payload            = NULL;
    rxs->calculated_crc     = CRC_INITIAL;
    rxs->transfer_id        = (CanardTransferID)((rxs->transfer_id + 1U) & CANARD_TRANSFER_ID_MAX);
    // The transport index is retained.
    rxs->toggle = INITIAL_TOGGLE_STATE;
}

CANARD_PRIVATE int8_t rxSessionAcceptFrame(CanardInstance* const          ins,
                                           CanardInternalRxSession* const rxs,
                                           const RxFrameModel* const      frame,
                                           const size_t                   extent,
                                           CanardTransfer* const          out_transfer);
CANARD_PRIVATE int8_t rxSessionAcceptFrame(CanardInstance* const          ins,
                                           CanardInternalRxSession* const rxs,
                                           const RxFrameModel* const      frame,
                                           const size_t                   extent,
                                           CanardTransfer* const          out_transfer)
{
    CANARD_ASSERT(ins != NULL);
    CANARD_ASSERT(rxs != NULL);
    CANARD_ASSERT(frame != NULL);
    CANARD_ASSERT(frame->payload != NULL);
    CANARD_ASSERT(frame->transfer_id <= CANARD_TRANSFER_ID_MAX);
    CANARD_ASSERT(out_transfer != NULL);

    if (frame->start_of_transfer)  // The transfer timestamp is the timestamp of its first frame.
    {
        rxs->transfer_timestamp_usec = frame->timestamp_usec;
    }

    const bool single_frame = frame->start_of_transfer && frame->end_of_transfer;
    if (!single_frame)
    {
        // Update the CRC. Observe that the implicit truncation rule may apply here: the payload may be
        // truncated, but its CRC is validated always anyway.
        rxs->calculated_crc = crcAdd(rxs->calculated_crc, frame->payload_size, frame->payload);
    }

    int8_t out = rxSessionWritePayload(ins, rxs, extent, frame->payload_size, frame->payload);
    if (out < 0)
    {
        CANARD_ASSERT(-CANARD_ERROR_OUT_OF_MEMORY == out);
        rxSessionRestart(ins, rxs);  // Out-of-memory.
    }
    else if (frame->end_of_transfer)
    {
        CANARD_ASSERT(0 == out);
        if (single_frame || (CRC_RESIDUE == rxs->calculated_crc))
        {
            out = 1;  // One transfer received, notify the application.
            rxInitTransferFromFrame(frame, out_transfer);
            out_transfer->timestamp_usec = rxs->transfer_timestamp_usec;
            out_transfer->payload_size   = rxs->payload_size;
            out_transfer->payload        = rxs->payload;

            // Cut off the CRC from the payload if it's there -- we don't want to expose it to the user.
            CANARD_ASSERT(rxs->total_payload_size >= rxs->payload_size);
            const size_t truncated_amount = rxs->total_payload_size - rxs->payload_size;
            if ((!single_frame) && (CRC_SIZE_BYTES > truncated_amount))  // Single-frame transfers don't have CRC.
            {
                CANARD_ASSERT(out_transfer->payload_size >= (CRC_SIZE_BYTES - truncated_amount));
                out_transfer->payload_size -= CRC_SIZE_BYTES - truncated_amount;
            }

            rxs->payload = NULL;  // Ownership passed over to the application, nullify to prevent freeing.
        }
        rxSessionRestart(ins, rxs);  // Successful completion.
    }
    else
    {
        rxs->toggle = !rxs->toggle;
    }
    return out;
}

/// RX session state machine update is the most intricate part of any UAVCAN transport implementation.
/// The state model used here is derived from the reference pseudocode given in the original UAVCAN v0 specification.
/// The UAVCAN/CAN v1 specification, which this library is an implementation of, does not provide any reference
/// pseudocode. Instead, it takes a higher-level, more abstract approach, where only the high-level requirements
/// are given and the particular algorithms are left to be implementation-defined. Such abstract approach is much
/// advantageous because it allows implementers to choose whatever solution works best for the specific application at
/// hand, while the wire compatibility is still guaranteed by the high-level requirements given in the specification.
CANARD_PRIVATE int8_t rxSessionUpdate(CanardInstance* const          ins,
                                      CanardInternalRxSession* const rxs,
                                      const RxFrameModel* const      frame,
                                      const uint8_t                  redundant_transport_index,
                                      const CanardMicrosecond        transfer_id_timeout_usec,
                                      const size_t                   extent,
                                      CanardTransfer* const          out_transfer);
CANARD_PRIVATE int8_t rxSessionUpdate(CanardInstance* const          ins,
                                      CanardInternalRxSession* const rxs,
                                      const RxFrameModel* const      frame,
                                      const uint8_t                  redundant_transport_index,
                                      const CanardMicrosecond        transfer_id_timeout_usec,
                                      const size_t                   extent,
                                      CanardTransfer* const          out_transfer)
{
    CANARD_ASSERT(ins != NULL);
    CANARD_ASSERT(rxs != NULL);
    CANARD_ASSERT(frame != NULL);
    CANARD_ASSERT(out_transfer != NULL);
    CANARD_ASSERT(rxs->transfer_id <= CANARD_TRANSFER_ID_MAX);
    CANARD_ASSERT(frame->transfer_id <= CANARD_TRANSFER_ID_MAX);

    const bool tid_timed_out = (frame->timestamp_usec > rxs->transfer_timestamp_usec) &&
                               ((frame->timestamp_usec - rxs->transfer_timestamp_usec) > transfer_id_timeout_usec);

    const bool not_previous_tid = rxComputeTransferIDDifference(rxs->transfer_id, frame->transfer_id) > 1;

    const bool need_restart = tid_timed_out || ((rxs->redundant_transport_index == redundant_transport_index) &&
                                                frame->start_of_transfer && not_previous_tid);

    if (need_restart)
    {
        rxs->total_payload_size        = 0U;
        rxs->payload_size              = 0U;
        rxs->calculated_crc            = CRC_INITIAL;
        rxs->transfer_id               = frame->transfer_id;
        rxs->toggle                    = INITIAL_TOGGLE_STATE;
        rxs->redundant_transport_index = redundant_transport_index;
    }

    int8_t out = 0;
    if (need_restart && (!frame->start_of_transfer))
    {
        rxSessionRestart(ins, rxs);  // SOT-miss, no point going further.
    }
    else
    {
        const bool correct_transport = (rxs->redundant_transport_index == redundant_transport_index);
        const bool correct_toggle    = (frame->toggle == rxs->toggle);
        const bool correct_tid       = (frame->transfer_id == rxs->transfer_id);
        if (correct_transport && correct_toggle && correct_tid)
        {
            out = rxSessionAcceptFrame(ins, rxs, frame, extent, out_transfer);
        }
    }
    return out;
}

CANARD_PRIVATE int8_t rxAcceptFrame(CanardInstance* const       ins,
                                    CanardRxSubscription* const subscription,
                                    const RxFrameModel* const   frame,
                                    const uint8_t               redundant_transport_index,
                                    CanardTransfer* const       out_transfer);
CANARD_PRIVATE int8_t rxAcceptFrame(CanardInstance* const       ins,
                                    CanardRxSubscription* const subscription,
                                    const RxFrameModel* const   frame,
                                    const uint8_t               redundant_transport_index,
                                    CanardTransfer* const       out_transfer)
{
    CANARD_ASSERT(ins != NULL);
    CANARD_ASSERT(subscription != NULL);
    CANARD_ASSERT(subscription->_port_id == frame->port_id);
    CANARD_ASSERT(frame != NULL);
    CANARD_ASSERT(frame->payload != NULL);
    CANARD_ASSERT(frame->transfer_id <= CANARD_TRANSFER_ID_MAX);
    CANARD_ASSERT((CANARD_NODE_ID_UNSET == frame->destination_node_id) || (ins->node_id == frame->destination_node_id));
    CANARD_ASSERT(out_transfer != NULL);

    int8_t out = 0;
    if (frame->source_node_id <= CANARD_NODE_ID_MAX)
    {
        // If such session does not exist, create it. This only makes sense if this is the first frame of a
        // transfer, otherwise, we won't be able to receive the transfer anyway so we don't bother.
        if ((NULL == subscription->_sessions[frame->source_node_id]) && frame->start_of_transfer)
        {
            CanardInternalRxSession* const rxs =
                (CanardInternalRxSession*) ins->memory_allocate(ins, sizeof(CanardInternalRxSession));
            subscription->_sessions[frame->source_node_id] = rxs;
            if (rxs != NULL)
            {
                rxs->transfer_timestamp_usec   = frame->timestamp_usec;
                rxs->total_payload_size        = 0U;
                rxs->payload_size              = 0U;
                rxs->payload                   = NULL;
                rxs->calculated_crc            = CRC_INITIAL;
                rxs->transfer_id               = frame->transfer_id;
                rxs->redundant_transport_index = redundant_transport_index;
                rxs->toggle                    = INITIAL_TOGGLE_STATE;
            }
            else
            {
                out = -CANARD_ERROR_OUT_OF_MEMORY;
            }
        }
        // There are two possible reasons why the session may not exist: 1. OOM; 2. SOT-miss.
        if (subscription->_sessions[frame->source_node_id] != NULL)
        {
            CANARD_ASSERT(out == 0);
            out = rxSessionUpdate(ins,
                                  subscription->_sessions[frame->source_node_id],
                                  frame,
                                  redundant_transport_index,
                                  subscription->_transfer_id_timeout_usec,
                                  subscription->_extent,
                                  out_transfer);
        }
    }
    else
    {
        CANARD_ASSERT(frame->source_node_id == CANARD_NODE_ID_UNSET);
        // Anonymous transfers are stateless. No need to update the state machine, just blindly accept it.
        // We have to copy the data into an allocated storage because the API expects it: the lifetime shall be
        // independent of the input data and the memory shall be free-able.
        const size_t payload_size =
            (subscription->_extent < frame->payload_size) ? subscription->_extent : frame->payload_size;
        void* const payload = ins->memory_allocate(ins, payload_size);
        if (payload != NULL)
        {
            rxInitTransferFromFrame(frame, out_transfer);
            out_transfer->payload_size = payload_size;
            out_transfer->payload      = payload;
            // Clang-Tidy raises an error recommending the use of memcpy_s() instead.
            // We ignore it because the safe functions are poorly supported; reliance on them may limit the portability.
            (void) memcpy(payload, frame->payload, payload_size);  // NOLINT
            out = 1;
        }
        else
        {
            out = -CANARD_ERROR_OUT_OF_MEMORY;
        }
    }
    return out;
}

// --------------------------------------------- PUBLIC API ---------------------------------------------

const uint8_t CanardCANDLCToLength[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
const uint8_t CanardCANLengthToDLC[65] = {
    0,  1,  2,  3,  4,  5,  6,  7,  8,                               // 0-8
    9,  9,  9,  9,                                                   // 9-12
    10, 10, 10, 10,                                                  // 13-16
    11, 11, 11, 11,                                                  // 17-20
    12, 12, 12, 12,                                                  // 21-24
    13, 13, 13, 13, 13, 13, 13, 13,                                  // 25-32
    14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,  // 33-48
    15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,  // 49-64
};

CanardInstance canardInit(const CanardMemoryAllocate memory_allocate, const CanardMemoryFree memory_free)
{
    CANARD_ASSERT(memory_allocate != NULL);
    CANARD_ASSERT(memory_free != NULL);
    const CanardInstance out = {
        .user_reference    = NULL,
        .mtu_bytes         = CANARD_MTU_CAN_FD,
        .node_id           = CANARD_NODE_ID_UNSET,
        .memory_allocate   = memory_allocate,
        .memory_free       = memory_free,
        ._rx_subscriptions = {NULL, NULL, NULL},
        ._tx_queue         = NULL,
    };
    return out;
}

int32_t canardTxPush(CanardInstance* const ins, const CanardTransfer* const transfer)
{
    int32_t out = -CANARD_ERROR_INVALID_ARGUMENT;
    if ((ins != NULL) && (transfer != NULL) && ((transfer->payload != NULL) || (0U == transfer->payload_size)))
    {
        const size_t  pl_mtu       = txGetPresentationLayerMTU(ins);
        const int32_t maybe_can_id = txMakeCANID(transfer, ins->node_id, pl_mtu);
        if (maybe_can_id >= 0)
        {
            if (transfer->payload_size <= pl_mtu)
            {
                out = txPushSingleFrame(ins,
                                        transfer->timestamp_usec,
                                        (uint32_t) maybe_can_id,
                                        transfer->transfer_id,
                                        transfer->payload_size,
                                        transfer->payload);
            }
            else
            {
                out = txPushMultiFrame(ins,
                                       pl_mtu,
                                       transfer->timestamp_usec,
                                       (uint32_t) maybe_can_id,
                                       transfer->transfer_id,
                                       transfer->payload_size,
                                       transfer->payload);
            }
        }
        else
        {
            out = maybe_can_id;
        }
    }
    return out;
}

const CanardFrame* canardTxPeek(const CanardInstance* const ins)
{
    const CanardFrame* out = NULL;
    if ((ins != NULL) && (ins->_tx_queue != NULL))
    {
        // Return pointer to the TX queue item typed as CanardFrame. Later, the application will be able to free
        // the memory allocated for the TX queue item using this pointer typed as CanardFrame. Although it may look
        // sketchy, this is actually safe and standard-compliant. The paragraph 6.7.2.1.15 of the C standard says:
        //     A pointer to a structure object, suitably converted, points to its initial member (or if that member is a
        //     bit-field, then to the unit in which it resides), and vice versa. There may be unnamed padding within a
        //     structure object, but not at its beginning.
        out = &ins->_tx_queue->frame;
        CANARD_ASSERT(((void*) out) == ((void*) ins->_tx_queue));
    }
    return out;
}

void canardTxPop(CanardInstance* const ins)
{
    if ((ins != NULL) && (ins->_tx_queue != NULL))
    {
        // The memory is NOT deallocated. The application is responsible for that.
        ins->_tx_queue = ins->_tx_queue->next;
    }
}

int8_t canardRxAccept(CanardInstance* const    ins,
                      const CanardFrame* const frame,
                      const uint8_t            redundant_transport_index,
                      CanardTransfer* const    out_transfer)
{
    int8_t out = -CANARD_ERROR_INVALID_ARGUMENT;
    if ((ins != NULL) && (out_transfer != NULL) && (frame != NULL) && (frame->extended_can_id <= CAN_EXT_ID_MASK) &&
        ((frame->payload != NULL) || (0 == frame->payload_size)))
    {
        RxFrameModel model = {0};
        if (rxTryParseFrame(frame, &model))
        {
            if ((CANARD_NODE_ID_UNSET == model.destination_node_id) || (ins->node_id == model.destination_node_id))
            {
                // Find subscription. This is the reason the function has a linear time complexity from the number of
                // subscriptions. Note also that this one of the two variable-complexity operations in the RX pipeline;
                // the other one is memcpy(). Excepting these two cases, the entire RX pipeline logic contains neither
                // loops nor recursion.
                CanardRxSubscription* sub = ins->_rx_subscriptions[(size_t) model.transfer_kind];
                while ((sub != NULL) && (sub->_port_id != model.port_id))
                {
                    sub = sub->_next;
                }

                if (sub != NULL)
                {
                    CANARD_ASSERT(sub->_port_id == model.port_id);
                    out = rxAcceptFrame(ins, sub, &model, redundant_transport_index, out_transfer);
                }
                else
                {
                    out = 0;  // No matching subscription.
                }
            }
            else
            {
                out = 0;  // Mis-addressed frame (normally it should be filtered out by the hardware).
            }
        }
        else
        {
            out = 0;  // A non-UAVCAN/CAN input frame.
        }
    }
    CANARD_ASSERT(out <= 1);
    return out;
}

int8_t canardRxSubscribe(CanardInstance* const       ins,
                         const CanardTransferKind    transfer_kind,
                         const CanardPortID          port_id,
                         const size_t                extent,
                         const CanardMicrosecond     transfer_id_timeout_usec,
                         CanardRxSubscription* const out_subscription)
{
    int8_t       out = -CANARD_ERROR_INVALID_ARGUMENT;
    const size_t tk  = (size_t) transfer_kind;
    if ((ins != NULL) && (out_subscription != NULL) && (tk < CANARD_NUM_TRANSFER_KINDS))
    {
        // Reset to the initial state. This is absolutely critical because the new payload size limit may be larger
        // than the old value; if there are any payload buffers allocated, we may overrun them because they are shorter
        // than the new payload limit. So we clear the subscription and thus ensure that no overrun may occur.
        out = canardRxUnsubscribe(ins, transfer_kind, port_id);
        if (out >= 0)
        {
            for (size_t i = 0; i < RX_SESSIONS_PER_SUBSCRIPTION; i++)
            {
                // The sessions will be created ad-hoc. Normally, for a low-jitter deterministic system,
                // we could have pre-allocated sessions here, but that requires too much memory to be feasible.
                // We could accept an extra argument that would instruct us to pre-allocate sessions here?
                out_subscription->_sessions[i] = NULL;
            }
            out_subscription->_transfer_id_timeout_usec = transfer_id_timeout_usec;
            out_subscription->_extent                   = extent;
            out_subscription->_port_id                  = port_id;
            out_subscription->_next                     = ins->_rx_subscriptions[tk];
            ins->_rx_subscriptions[tk]                  = out_subscription;
            out                                         = (out > 0) ? 0 : 1;
        }
    }
    return out;
}

int8_t canardRxUnsubscribe(CanardInstance* const    ins,
                           const CanardTransferKind transfer_kind,
                           const CanardPortID       port_id)
{
    int8_t       out = -CANARD_ERROR_INVALID_ARGUMENT;
    const size_t tk  = (size_t) transfer_kind;
    if ((ins != NULL) && (tk < CANARD_NUM_TRANSFER_KINDS))
    {
        CanardRxSubscription* prv = NULL;
        CanardRxSubscription* sub = ins->_rx_subscriptions[tk];
        while ((sub != NULL) && (sub->_port_id != port_id))
        {
            prv = sub;
            sub = sub->_next;
        }

        if (sub != NULL)
        {
            CANARD_ASSERT(sub->_port_id == port_id);
            out = 1;

            if (prv != NULL)
            {
                prv->_next = sub->_next;
            }
            else
            {
                ins->_rx_subscriptions[tk] = sub->_next;
            }

            for (size_t i = 0; i < RX_SESSIONS_PER_SUBSCRIPTION; i++)
            {
                ins->memory_free(ins, (sub->_sessions[i] != NULL) ? sub->_sessions[i]->payload : NULL);
                ins->memory_free(ins, sub->_sessions[i]);
                sub->_sessions[i] = NULL;
            }
        }
        else
        {
            out = 0;
        }
    }
    return out;
}
