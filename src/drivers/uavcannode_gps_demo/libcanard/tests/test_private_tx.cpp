// This software is distributed under the terms of the MIT License.
// Copyright (c) 2016-2020 UAVCAN Development Team.

#include "exposed.hpp"
#include "helpers.hpp"

TEST_CASE("SessionSpecifier")
{
    REQUIRE(0b000'00'0'11'1001100110011'0'1010101 ==
            exposed::txMakeMessageSessionSpecifier(0b1001100110011, 0b1010101));
    REQUIRE(0b000'11'0100110011'0101010'1010101 ==
            exposed::txMakeServiceSessionSpecifier(0b0100110011, true, 0b1010101, 0b0101010));
    REQUIRE(0b000'10'0100110011'1010101'0101010 ==
            exposed::txMakeServiceSessionSpecifier(0b0100110011, false, 0b0101010, 0b1010101));
}

TEST_CASE("txGetPresentationLayerMTU")
{
    auto ins = canardInit(&helpers::dummy_allocator::allocate, &helpers::dummy_allocator::free);
    REQUIRE(63 == exposed::txGetPresentationLayerMTU(&ins));  // This is the default.
    ins.mtu_bytes = 0;
    REQUIRE(7 == exposed::txGetPresentationLayerMTU(&ins));
    ins.mtu_bytes = 255;
    REQUIRE(63 == exposed::txGetPresentationLayerMTU(&ins));
    ins.mtu_bytes = 32;
    REQUIRE(31 == exposed::txGetPresentationLayerMTU(&ins));
    ins.mtu_bytes = 30;  // Round up.
    REQUIRE(31 == exposed::txGetPresentationLayerMTU(&ins));
}

TEST_CASE("txMakeCANID")
{
    using exposed::txMakeCANID;

    CanardTransfer            transfer{};
    std::vector<std::uint8_t> transfer_payload;

    const auto mk_transfer = [&](const CanardPriority             priority,
                                 const CanardTransferKind         kind,
                                 const std::uint16_t              port_id,
                                 const std::uint8_t               remote_node_id,
                                 const std::vector<std::uint8_t>& payload = {}) {
        transfer_payload        = payload;
        transfer.priority       = priority;
        transfer.transfer_kind  = kind;
        transfer.port_id        = port_id;
        transfer.remote_node_id = remote_node_id;
        transfer.payload        = transfer_payload.data();
        transfer.payload_size   = transfer_payload.size();
        return &transfer;
    };

    const auto crc123 = exposed::crcAdd(0xFFFFU, 3, "\x01\x02\x03");

    union PriorityAlias
    {
        std::uint8_t   bits;
        CanardPriority prio;
    };

    // MESSAGE TRANSFERS
    REQUIRE(0b000'00'0'11'1001100110011'0'1010101 ==  // Regular message.
            txMakeCANID(mk_transfer(CanardPriorityExceptional,
                                    CanardTransferKindMessage,
                                    0b1001100110011,
                                    CANARD_NODE_ID_UNSET),
                        0b1010101,
                        7U));
    REQUIRE(0b111'00'0'11'1001100110011'0'1010101 ==  // Regular message.
            txMakeCANID(mk_transfer(CanardPriorityOptional,
                                    CanardTransferKindMessage,
                                    0b1001100110011,
                                    CANARD_NODE_ID_UNSET),
                        0b1010101,
                        7U));
    REQUIRE((0b010'01'0'11'1001100110011'0'0000000U | (crc123 & CANARD_NODE_ID_MAX)) ==  // Anonymous message.
            txMakeCANID(mk_transfer(CanardPriorityFast,
                                    CanardTransferKindMessage,
                                    0b1001100110011,
                                    CANARD_NODE_ID_UNSET,
                                    {1, 2, 3}),
                        128U,  // Invalid local node-ID.
                        7U));
    REQUIRE(-CANARD_ERROR_INVALID_ARGUMENT ==  // Multi-frame anonymous messages are not allowed.
            txMakeCANID(mk_transfer(CanardPriorityImmediate,
                                    CanardTransferKindMessage,
                                    0b1001100110011,
                                    CANARD_NODE_ID_UNSET,
                                    {1, 2, 3, 4, 5, 6, 7, 8}),
                        128U,  // Invalid local node-ID is treated as anonymous/unset.
                        7U));
    REQUIRE(-CANARD_ERROR_INVALID_ARGUMENT ==  // Bad remote node-ID -- unicast messages not supported.
            txMakeCANID(mk_transfer(CanardPriorityHigh, CanardTransferKindMessage, 0b1001100110011, 123U), 0U, 7U));
    REQUIRE(
        -CANARD_ERROR_INVALID_ARGUMENT ==  // Bad subject-ID.
        txMakeCANID(mk_transfer(CanardPriorityLow, CanardTransferKindMessage, 0xFFFFU, CANARD_NODE_ID_UNSET), 0U, 7U));
    REQUIRE(-CANARD_ERROR_INVALID_ARGUMENT ==  // Bad priority.
            txMakeCANID(mk_transfer(PriorityAlias{123}.prio,
                                    CanardTransferKindMessage,
                                    0b1001100110011,
                                    CANARD_NODE_ID_UNSET),
                        0b1010101,
                        7U));

    // SERVICE TRANSFERS
    REQUIRE(0b000'11'0100110011'0101010'1010101 ==  // Request.
            txMakeCANID(mk_transfer(CanardPriorityExceptional, CanardTransferKindRequest, 0b0100110011, 0b0101010),
                        0b1010101,
                        7U));
    REQUIRE(0b111'10'0100110011'0101010'1010101 ==  // Response.
            txMakeCANID(mk_transfer(CanardPriorityOptional, CanardTransferKindResponse, 0b0100110011, 0b0101010),
                        0b1010101,
                        7U));
    REQUIRE(-CANARD_ERROR_INVALID_ARGUMENT ==  // Anonymous service transfers not permitted.
            txMakeCANID(mk_transfer(CanardPriorityExceptional, CanardTransferKindRequest, 0b0100110011, 0b0101010),
                        CANARD_NODE_ID_UNSET,
                        7U));
    REQUIRE(-CANARD_ERROR_INVALID_ARGUMENT ==  // Broadcast service transfers not permitted.
            txMakeCANID(mk_transfer(CanardPrioritySlow, CanardTransferKindResponse, 0b0100110011, CANARD_NODE_ID_UNSET),
                        0b1010101,
                        7U));
    REQUIRE(
        -CANARD_ERROR_INVALID_ARGUMENT ==  // Bad service-ID.
        txMakeCANID(mk_transfer(CanardPriorityNominal, CanardTransferKindResponse, 0xFFFFU, 0b0101010), 0b1010101, 7U));
    REQUIRE(-CANARD_ERROR_INVALID_ARGUMENT ==  // Bad priority.
            txMakeCANID(mk_transfer(PriorityAlias{123}.prio, CanardTransferKindResponse, 0b0100110011, 0b0101010),
                        0b1010101,
                        7U));
}

TEST_CASE("txMakeTailByte")
{
    using exposed::txMakeTailByte;
    REQUIRE(0b111'00000 == txMakeTailByte(true, true, true, 0U));
    REQUIRE(0b111'00000 == txMakeTailByte(true, true, true, 32U));
    REQUIRE(0b111'11111 == txMakeTailByte(true, true, true, 31U));
    REQUIRE(0b011'11111 == txMakeTailByte(false, true, true, 31U));
    REQUIRE(0b101'11110 == txMakeTailByte(true, false, true, 30U));
    REQUIRE(0b001'11101 == txMakeTailByte(false, false, true, 29U));
    REQUIRE(0b010'00001 == txMakeTailByte(false, true, false, 1U));
}

TEST_CASE("txRoundFramePayloadSizeUp")
{
    using exposed::txRoundFramePayloadSizeUp;
    REQUIRE(0 == txRoundFramePayloadSizeUp(0));
    REQUIRE(1 == txRoundFramePayloadSizeUp(1));
    REQUIRE(2 == txRoundFramePayloadSizeUp(2));
    REQUIRE(3 == txRoundFramePayloadSizeUp(3));
    REQUIRE(4 == txRoundFramePayloadSizeUp(4));
    REQUIRE(5 == txRoundFramePayloadSizeUp(5));
    REQUIRE(6 == txRoundFramePayloadSizeUp(6));
    REQUIRE(7 == txRoundFramePayloadSizeUp(7));
    REQUIRE(8 == txRoundFramePayloadSizeUp(8));
    REQUIRE(12 == txRoundFramePayloadSizeUp(9));
    REQUIRE(12 == txRoundFramePayloadSizeUp(10));
    REQUIRE(12 == txRoundFramePayloadSizeUp(11));
    REQUIRE(12 == txRoundFramePayloadSizeUp(12));
    REQUIRE(16 == txRoundFramePayloadSizeUp(13));
    REQUIRE(16 == txRoundFramePayloadSizeUp(14));
    REQUIRE(16 == txRoundFramePayloadSizeUp(15));
    REQUIRE(16 == txRoundFramePayloadSizeUp(16));
    REQUIRE(20 == txRoundFramePayloadSizeUp(17));
    REQUIRE(20 == txRoundFramePayloadSizeUp(20));
    REQUIRE(32 == txRoundFramePayloadSizeUp(30));
    REQUIRE(32 == txRoundFramePayloadSizeUp(32));
    REQUIRE(48 == txRoundFramePayloadSizeUp(40));
    REQUIRE(48 == txRoundFramePayloadSizeUp(48));
    REQUIRE(64 == txRoundFramePayloadSizeUp(50));
    REQUIRE(64 == txRoundFramePayloadSizeUp(64));
}

TEST_CASE("txFindQueueSupremum")
{
    using exposed::txFindQueueSupremum;
    using TxQueueItem = exposed::TxQueueItem;

    auto ins = canardInit(&helpers::dummy_allocator::allocate, &helpers::dummy_allocator::free);

    const auto find = [&](std::uint32_t x) -> TxQueueItem* { return txFindQueueSupremum(&ins, x); };

    REQUIRE(nullptr == find(0));
    REQUIRE(nullptr == find((1UL << 29U) - 1U));

    TxQueueItem a{};
    a.frame.extended_can_id = 1000;
    ins._tx_queue           = reinterpret_cast<CanardInternalTxQueueItem*>(&a);

    REQUIRE(nullptr == find(999));
    REQUIRE(&a == find(1000));
    REQUIRE(&a == find(1001));

    TxQueueItem b{};
    b.frame.extended_can_id = 1010;
    a.next                  = &b;

    REQUIRE(nullptr == find(999));
    REQUIRE(&a == find(1000));
    REQUIRE(&a == find(1001));
    REQUIRE(&a == find(1009));
    REQUIRE(&b == find(1010));
    REQUIRE(&b == find(1011));

    TxQueueItem c{};
    c.frame.extended_can_id = 990;
    c.next                  = &a;
    ins._tx_queue           = reinterpret_cast<CanardInternalTxQueueItem*>(&c);
    REQUIRE(reinterpret_cast<TxQueueItem*>(ins._tx_queue)->frame.extended_can_id == 990);
    REQUIRE(reinterpret_cast<TxQueueItem*>(ins._tx_queue)->next->frame.extended_can_id == 1000);
    REQUIRE(reinterpret_cast<TxQueueItem*>(ins._tx_queue)->next->next->frame.extended_can_id == 1010);
    REQUIRE(reinterpret_cast<TxQueueItem*>(ins._tx_queue)->next->next->next == nullptr);

    REQUIRE(nullptr == find(989));
    REQUIRE(&c == find(990));
    REQUIRE(&c == find(999));
    REQUIRE(&a == find(1000));
    REQUIRE(&a == find(1001));
    REQUIRE(&a == find(1009));
    REQUIRE(&b == find(1010));
    REQUIRE(&b == find(1011));
}
