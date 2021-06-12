// This software is distributed under the terms of the MIT License.
// Copyright (c) 2016-2020 UAVCAN Development Team.

#pragma once

#include "canard.h"
#include "catch.hpp"
#include <cstdarg>
#include <cstdint>
#include <limits>

/// Definitions that are not exposed by the library but that are needed for testing.
/// Please keep them in sync with the library by manually updating as necessary.
namespace exposed
{
using TransferCRC = std::uint16_t;

struct TxQueueItem final
{
    CanardFrame  frame{};
    TxQueueItem* next = nullptr;

    [[nodiscard]] auto getPayloadByte(const std::size_t offset) const -> std::uint8_t
    {
        return reinterpret_cast<const std::uint8_t*>(frame.payload)[offset];
    }

    [[nodiscard]] auto getTailByte() const
    {
        REQUIRE(frame.payload_size >= 1U);
        return getPayloadByte(frame.payload_size - 1U);
    }

    [[nodiscard]] auto isStartOfTransfer() const { return (getTailByte() & 128U) != 0; }
    [[nodiscard]] auto isEndOfTransfer() const { return (getTailByte() & 64U) != 0; }
    [[nodiscard]] auto isToggleBitSet() const { return (getTailByte() & 32U) != 0; }

    ~TxQueueItem()                   = default;
    TxQueueItem(const TxQueueItem&)  = delete;
    TxQueueItem(const TxQueueItem&&) = delete;
    auto operator=(const TxQueueItem&) -> TxQueueItem& = delete;
    auto operator=(const TxQueueItem&&) -> TxQueueItem& = delete;
};

struct RxSession
{
    CanardMicrosecond transfer_timestamp_usec   = std::numeric_limits<std::uint64_t>::max();
    std::size_t       total_payload_size        = 0U;
    std::size_t       payload_size              = 0U;
    std::uint8_t*     payload                   = nullptr;
    TransferCRC       calculated_crc            = 0U;
    CanardTransferID  transfer_id               = std::numeric_limits<std::uint8_t>::max();
    std::uint8_t      redundant_transport_index = std::numeric_limits<std::uint8_t>::max();
    bool              toggle                    = false;
};

struct RxFrameModel
{
    CanardMicrosecond   timestamp_usec      = std::numeric_limits<std::uint64_t>::max();
    CanardPriority      priority            = CanardPriorityOptional;
    CanardTransferKind  transfer_kind       = CanardTransferKindMessage;
    CanardPortID        port_id             = std::numeric_limits<std::uint16_t>::max();
    CanardNodeID        source_node_id      = CANARD_NODE_ID_UNSET;
    CanardNodeID        destination_node_id = CANARD_NODE_ID_UNSET;
    CanardTransferID    transfer_id         = std::numeric_limits<std::uint8_t>::max();
    bool                start_of_transfer   = false;
    bool                end_of_transfer     = false;
    bool                toggle              = false;
    std::size_t         payload_size        = 0U;
    const std::uint8_t* payload             = nullptr;
};

// Extern C effectively discards the outer namespaces.
extern "C" {

auto crcAdd(const std::uint16_t crc, const std::size_t size, const void* const bytes) -> std::uint16_t;

auto txMakeMessageSessionSpecifier(const std::uint16_t subject_id, const std::uint8_t src_node_id) -> std::uint32_t;
auto txMakeServiceSessionSpecifier(const std::uint16_t service_id,
                                   const bool          request_not_response,
                                   const std::uint8_t  src_node_id,
                                   const std::uint8_t  dst_node_id) -> std::uint32_t;

auto txGetPresentationLayerMTU(const CanardInstance* const ins) -> std::size_t;

auto txMakeCANID(const CanardTransfer* const transfer,
                 const std::uint8_t          local_node_id,
                 const std::size_t           presentation_layer_mtu) -> std::int32_t;

auto txMakeTailByte(const bool         start_of_transfer,
                    const bool         end_of_transfer,
                    const bool         toggle,
                    const std::uint8_t transfer_id) -> std::uint8_t;

auto txRoundFramePayloadSizeUp(const std::size_t x) -> std::size_t;

auto txFindQueueSupremum(const CanardInstance* const ins, const std::uint32_t can_id) -> TxQueueItem*;

auto rxTryParseFrame(const CanardFrame* const frame, RxFrameModel* const out_result) -> bool;

auto rxSessionWritePayload(CanardInstance* const ins,
                           RxSession* const      rxs,
                           const std::size_t     extent,
                           const std::size_t     payload_size,
                           const void* const     payload) -> std::int8_t;

void rxSessionRestart(CanardInstance* const ins, RxSession* const rxs);

auto rxSessionUpdate(CanardInstance* const     ins,
                     RxSession* const          rxs,
                     const RxFrameModel* const frame,
                     const std::uint8_t        redundant_transport_index,
                     const CanardMicrosecond   transfer_id_timeout_usec,
                     const std::size_t         extent,
                     CanardTransfer* const     out_transfer) -> std::int8_t;

auto float16Pack(const float value) -> std::uint16_t;
auto float16Unpack(const std::uint16_t value) -> float;
}
}  // namespace exposed
