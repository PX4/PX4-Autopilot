// This software is distributed under the terms of the MIT License.
// Copyright (c) 2016-2020 UAVCAN Development Team.

#include "exposed.hpp"
#include "helpers.hpp"
#include <cstring>

TEST_CASE("TxBasic0")
{
    using exposed::TxQueueItem;

    helpers::Instance ins;

    auto& alloc = ins.getAllocator();

    std::array<std::uint8_t, 1024> payload{};
    for (std::size_t i = 0; i < std::size(payload); i++)
    {
        payload.at(i) = static_cast<std::uint8_t>(i & 0xFFU);
    }

    REQUIRE(CANARD_NODE_ID_UNSET == ins.getNodeID());
    REQUIRE(CANARD_MTU_CAN_FD == ins.getMTU());
    REQUIRE(nullptr == ins.getTxQueueRoot());
    REQUIRE(0 == ins.getTxQueueLength());
    REQUIRE(0 == alloc.getNumAllocatedFragments());

    alloc.setAllocationCeiling(200);

    CanardTransfer transfer{};
    transfer.payload = payload.data();

    // Single-frame with padding.
    transfer.timestamp_usec = 1'000'000'000'000ULL;
    transfer.priority       = CanardPriorityNominal;
    transfer.transfer_kind  = CanardTransferKindMessage;
    transfer.port_id        = 321;
    transfer.remote_node_id = CANARD_NODE_ID_UNSET;
    transfer.transfer_id    = 21;
    transfer.payload_size   = 8;
    REQUIRE(1 == ins.txPush(transfer));
    REQUIRE(1 == ins.getTxQueueLength());
    REQUIRE(1 == alloc.getNumAllocatedFragments());
    REQUIRE(10 < alloc.getTotalAllocatedAmount());
    REQUIRE(80 > alloc.getTotalAllocatedAmount());
    REQUIRE(ins.getTxQueueRoot()->frame.timestamp_usec == 1'000'000'000'000ULL);
    REQUIRE(ins.getTxQueueRoot()->frame.payload_size == 12);  // Three bytes of padding.
    REQUIRE(ins.getTxQueueRoot()->getPayloadByte(0) == 0);    // Payload start.
    REQUIRE(ins.getTxQueueRoot()->getPayloadByte(1) == 1);
    REQUIRE(ins.getTxQueueRoot()->getPayloadByte(2) == 2);
    REQUIRE(ins.getTxQueueRoot()->getPayloadByte(3) == 3);
    REQUIRE(ins.getTxQueueRoot()->getPayloadByte(4) == 4);
    REQUIRE(ins.getTxQueueRoot()->getPayloadByte(5) == 5);
    REQUIRE(ins.getTxQueueRoot()->getPayloadByte(6) == 6);
    REQUIRE(ins.getTxQueueRoot()->getPayloadByte(7) == 7);   // Payload end.
    REQUIRE(ins.getTxQueueRoot()->getPayloadByte(8) == 0);   // Padding.
    REQUIRE(ins.getTxQueueRoot()->getPayloadByte(9) == 0);   // Padding.
    REQUIRE(ins.getTxQueueRoot()->getPayloadByte(10) == 0);  // Padding.
    REQUIRE(ins.getTxQueueRoot()->isStartOfTransfer());      // Tail byte at the end.
    REQUIRE(ins.getTxQueueRoot()->isEndOfTransfer());
    REQUIRE(ins.getTxQueueRoot()->isToggleBitSet());

    // Multi-frame. Priority low, inserted at the end of the TX queue.
    transfer.timestamp_usec = 1'000'000'000'100ULL;
    transfer.priority       = CanardPriorityLow;
    transfer.transfer_id    = 22;
    transfer.payload_size   = 8;
    ins.setMTU(CANARD_MTU_CAN_CLASSIC);
    ins.setNodeID(42);
    REQUIRE(2 == ins.txPush(transfer));  // 8 bytes --> 2 frames
    REQUIRE(3 == ins.getTxQueueLength());
    REQUIRE(3 == alloc.getNumAllocatedFragments());
    REQUIRE(20 < alloc.getTotalAllocatedAmount());
    REQUIRE(200 > alloc.getTotalAllocatedAmount());

    // Check the TX queue.
    {
        const auto* q = ins.getTxQueueRoot();
        REQUIRE(q != nullptr);
        REQUIRE(q->frame.timestamp_usec == 1'000'000'000'000ULL);
        REQUIRE(q->frame.payload_size == 12);
        REQUIRE(q->isStartOfTransfer());
        REQUIRE(q->isEndOfTransfer());
        REQUIRE(q->isToggleBitSet());
        q = q->next;
        REQUIRE(q != nullptr);
        REQUIRE(q->frame.timestamp_usec == 1'000'000'000'100ULL);
        REQUIRE(q->frame.payload_size == 8);
        REQUIRE(q->isStartOfTransfer());
        REQUIRE(!q->isEndOfTransfer());
        REQUIRE(q->isToggleBitSet());
        q = q->next;
        REQUIRE(q != nullptr);
        REQUIRE(q->frame.timestamp_usec == 1'000'000'000'100ULL);
        REQUIRE(q->frame.payload_size == 4);  // One leftover, two CRC, one tail.
        REQUIRE(!q->isStartOfTransfer());
        REQUIRE(q->isEndOfTransfer());
        REQUIRE(!q->isToggleBitSet());
        q = q->next;
        REQUIRE(q == nullptr);
    }

    // Single-frame, OOM.
    alloc.setAllocationCeiling(alloc.getTotalAllocatedAmount());  // Seal up the heap at this level.
    transfer.timestamp_usec = 1'000'000'000'200ULL;
    transfer.priority       = CanardPriorityLow;
    transfer.transfer_id    = 23;
    transfer.payload_size   = 1;
    REQUIRE(-CANARD_ERROR_OUT_OF_MEMORY == ins.txPush(transfer));
    REQUIRE(3 == ins.getTxQueueLength());
    REQUIRE(3 == alloc.getNumAllocatedFragments());

    // Multi-frame, first frame added successfully, then OOM. The entire transaction rejected.
    alloc.setAllocationCeiling(alloc.getTotalAllocatedAmount() + sizeof(TxQueueItem) + 10U);
    transfer.timestamp_usec = 1'000'000'000'300ULL;
    transfer.priority       = CanardPriorityHigh;
    transfer.transfer_id    = 24;
    transfer.payload_size   = 100;
    REQUIRE(-CANARD_ERROR_OUT_OF_MEMORY == ins.txPush(transfer));
    REQUIRE(3 == ins.getTxQueueLength());
    REQUIRE(3 == alloc.getNumAllocatedFragments());
    REQUIRE(20 < alloc.getTotalAllocatedAmount());
    REQUIRE(200 > alloc.getTotalAllocatedAmount());

    // Pop the queue.
    // hex(pyuavcan.transport.commons.crc.CRC16CCITT.new(list(range(8))).value)
    constexpr std::uint16_t CRC8  = 0x178DU;
    const CanardFrame*      frame = ins.txPeek();
    REQUIRE(nullptr != frame);
    REQUIRE(frame->payload_size == 12);
    REQUIRE(0 == std::memcmp(frame->payload, payload.data(), 8));
    REQUIRE(0 == reinterpret_cast<const std::uint8_t*>(frame->payload)[8]);   // Padding.
    REQUIRE(0 == reinterpret_cast<const std::uint8_t*>(frame->payload)[9]);   // Padding.
    REQUIRE(0 == reinterpret_cast<const std::uint8_t*>(frame->payload)[10]);  // Padding.
    REQUIRE((0b11100000U | 21U) == reinterpret_cast<const std::uint8_t*>(frame->payload)[11]);
    REQUIRE(frame->timestamp_usec == 1'000'000'000'000ULL);
    frame = ins.txPeek();
    REQUIRE(nullptr != frame);  // Make sure we get the same frame again.
    REQUIRE(frame->payload_size == 12);
    REQUIRE(0 == std::memcmp(frame->payload, payload.data(), 8));
    REQUIRE((0b11100000U | 21U) == reinterpret_cast<const std::uint8_t*>(frame->payload)[11]);
    REQUIRE(frame->timestamp_usec == 1'000'000'000'000ULL);
    ins.txPop();
    ins.getAllocator().deallocate(frame);
    REQUIRE(2 == ins.getTxQueueLength());
    REQUIRE(2 == alloc.getNumAllocatedFragments());
    frame = ins.txPeek();
    REQUIRE(nullptr != frame);
    REQUIRE(frame->payload_size == 8);
    REQUIRE(0 == std::memcmp(frame->payload, payload.data(), 7));
    REQUIRE((0b10100000U | 22U) == reinterpret_cast<const std::uint8_t*>(frame->payload)[7]);
    REQUIRE(frame->timestamp_usec == 1'000'000'000'100ULL);
    ins.txPop();
    ins.getAllocator().deallocate(frame);
    REQUIRE(1 == ins.getTxQueueLength());
    REQUIRE(1 == alloc.getNumAllocatedFragments());
    frame = ins.txPeek();
    REQUIRE(nullptr != frame);
    REQUIRE(frame->payload_size == 4);
    REQUIRE(0 == std::memcmp(frame->payload, payload.data() + 7U, 1));
    REQUIRE((CRC8 >> 8U) == reinterpret_cast<const std::uint8_t*>(frame->payload)[1]);
    REQUIRE((CRC8 & 0xFFU) == reinterpret_cast<const std::uint8_t*>(frame->payload)[2]);
    REQUIRE((0b01000000U | 22U) == reinterpret_cast<const std::uint8_t*>(frame->payload)[3]);
    REQUIRE(frame->timestamp_usec == 1'000'000'000'100ULL);
    ins.txPop();
    ins.getAllocator().deallocate(frame);
    REQUIRE(0 == ins.getTxQueueLength());
    REQUIRE(nullptr == ins.getTxQueueRoot());
    REQUIRE(0 == alloc.getNumAllocatedFragments());
    frame = ins.txPeek();
    REQUIRE(nullptr == frame);
    ins.txPop();  // Invocation when empty has no effect.
    ins.txPop();
    REQUIRE(0 == ins.getTxQueueLength());
    REQUIRE(nullptr == ins.getTxQueueRoot());
    REQUIRE(0 == alloc.getNumAllocatedFragments());
    frame = ins.txPeek();
    REQUIRE(nullptr == frame);

    alloc.setAllocationCeiling(1000);

    // Multi-frame, success. CRC split over the frame boundary.
    // hex(pyuavcan.transport.commons.crc.CRC16CCITT.new(list(range(61))).value)
    constexpr std::uint16_t CRC61 = 0x554EU;
    ins.setMTU(32);
    transfer.timestamp_usec = 1'000'000'001'000ULL;
    transfer.priority       = CanardPriorityFast;
    transfer.transfer_id    = 25;
    transfer.payload_size   = 31 + 30;  // CRC takes 2 bytes at the end; 3 frames: (31+1) + (30+1+1) + (1+1)
    REQUIRE(3 == ins.txPush(transfer));
    REQUIRE(3 == ins.getTxQueueLength());
    REQUIRE(3 == alloc.getNumAllocatedFragments());
    REQUIRE(40 < alloc.getTotalAllocatedAmount());
    REQUIRE(220 > alloc.getTotalAllocatedAmount());
    // Read the generated frames.
    frame = ins.txPeek();
    REQUIRE(nullptr != frame);
    REQUIRE(frame->payload_size == 32);
    REQUIRE(0 == std::memcmp(frame->payload, payload.data(), 31));
    REQUIRE((0b10100000U | 25U) == reinterpret_cast<const std::uint8_t*>(frame->payload)[31]);
    REQUIRE(frame->timestamp_usec == 1'000'000'001'000ULL);
    ins.txPop();
    ins.getAllocator().deallocate(frame);
    REQUIRE(2 == ins.getTxQueueLength());
    REQUIRE(2 == alloc.getNumAllocatedFragments());
    frame = ins.txPeek();
    REQUIRE(nullptr != frame);
    REQUIRE(frame->payload_size == 32);
    REQUIRE(0 == std::memcmp(frame->payload, payload.data() + 31U, 30));
    REQUIRE((CRC61 >> 8U) == reinterpret_cast<const std::uint8_t*>(frame->payload)[30]);
    REQUIRE((0b00000000U | 25U) == reinterpret_cast<const std::uint8_t*>(frame->payload)[31]);
    REQUIRE(frame->timestamp_usec == 1'000'000'001'000ULL);
    ins.txPop();
    ins.getAllocator().deallocate(frame);
    REQUIRE(1 == ins.getTxQueueLength());
    REQUIRE(1 == alloc.getNumAllocatedFragments());
    frame = ins.txPeek();
    REQUIRE(nullptr != frame);
    REQUIRE(frame->payload_size == 2);  // The last byte of CRC plus the tail byte.
    REQUIRE((CRC61 & 0xFFU) == reinterpret_cast<const std::uint8_t*>(frame->payload)[0]);
    REQUIRE((0b01100000U | 25U) == reinterpret_cast<const std::uint8_t*>(frame->payload)[1]);
    REQUIRE(frame->timestamp_usec == 1'000'000'001'000ULL);
    ins.txPop();
    ins.getAllocator().deallocate(frame);
    REQUIRE(0 == ins.getTxQueueLength());
    REQUIRE(0 == alloc.getNumAllocatedFragments());

    // Multi-frame, success. CRC is in the last frame->
    // hex(pyuavcan.transport.commons.crc.CRC16CCITT.new(list(range(62))).value)
    constexpr std::uint16_t CRC62 = 0xA3AEU;
    ins.setMTU(32);
    transfer.timestamp_usec = 1'000'000'002'000ULL;
    transfer.priority       = CanardPrioritySlow;
    transfer.transfer_id    = 26;
    transfer.payload_size   = 31 + 31;  // CRC takes 2 bytes at the end; 3 frames: (31+1) + (31+1) + (2+1)
    REQUIRE(3 == ins.txPush(transfer));
    REQUIRE(3 == ins.getTxQueueLength());
    REQUIRE(3 == alloc.getNumAllocatedFragments());
    REQUIRE(40 < alloc.getTotalAllocatedAmount());
    REQUIRE(220 > alloc.getTotalAllocatedAmount());
    // Read the generated frames.
    frame = ins.txPeek();
    REQUIRE(nullptr != frame);
    REQUIRE(frame->payload_size == 32);
    REQUIRE(0 == std::memcmp(frame->payload, payload.data(), 31));
    REQUIRE((0b10100000U | 26U) == reinterpret_cast<const std::uint8_t*>(frame->payload)[31]);
    REQUIRE(frame->timestamp_usec == 1'000'000'002'000ULL);
    ins.txPop();
    ins.getAllocator().deallocate(frame);
    REQUIRE(2 == ins.getTxQueueLength());
    REQUIRE(2 == alloc.getNumAllocatedFragments());
    frame = ins.txPeek();
    REQUIRE(nullptr != frame);
    REQUIRE(frame->payload_size == 32);
    REQUIRE(0 == std::memcmp(frame->payload, payload.data() + 31U, 31));
    REQUIRE((0b00000000U | 26U) == reinterpret_cast<const std::uint8_t*>(frame->payload)[31]);
    REQUIRE(frame->timestamp_usec == 1'000'000'002'000ULL);
    ins.txPop();
    ins.getAllocator().deallocate(frame);
    REQUIRE(1 == ins.getTxQueueLength());
    REQUIRE(1 == alloc.getNumAllocatedFragments());
    frame = ins.txPeek();
    REQUIRE(nullptr != frame);
    REQUIRE(frame->payload_size == 3);  // The CRC plus the tail byte.
    REQUIRE((CRC62 >> 8U) == reinterpret_cast<const std::uint8_t*>(frame->payload)[0]);
    REQUIRE((CRC62 & 0xFFU) == reinterpret_cast<const std::uint8_t*>(frame->payload)[1]);
    REQUIRE((0b01100000U | 26U) == reinterpret_cast<const std::uint8_t*>(frame->payload)[2]);
    REQUIRE(frame->timestamp_usec == 1'000'000'002'000ULL);
    ins.txPop();
    ins.getAllocator().deallocate(frame);
    REQUIRE(0 == ins.getTxQueueLength());
    REQUIRE(0 == alloc.getNumAllocatedFragments());

    // Multi-frame with padding.
    // hex(pyuavcan.transport.commons.crc.CRC16CCITT.new(list(range(112)) + [0] * 12).value)
    constexpr std::uint16_t CRC112Padding12 = 0xE7A5U;
    ins.setMTU(64);
    transfer.timestamp_usec = 1'000'000'003'000ULL;
    transfer.priority       = CanardPriorityImmediate;
    transfer.transfer_id    = 27;
    transfer.payload_size   = 112;  // 63 + 63 - 2 = 124 bytes; 124 - 112 = 12 bytes of padding.
    REQUIRE(2 == ins.txPush(transfer));
    REQUIRE(2 == ins.getTxQueueLength());
    REQUIRE(2 == alloc.getNumAllocatedFragments());
    // Read the generated frames.
    frame = ins.txPeek();
    REQUIRE(nullptr != frame);
    REQUIRE(frame->payload_size == 64);
    REQUIRE(0 == std::memcmp(frame->payload, payload.data(), 63));
    REQUIRE((0b10100000U | 27U) == reinterpret_cast<const std::uint8_t*>(frame->payload)[63]);
    REQUIRE(frame->timestamp_usec == 1'000'000'003'000ULL);
    ins.txPop();
    ins.getAllocator().deallocate(frame);
    REQUIRE(1 == ins.getTxQueueLength());
    REQUIRE(1 == alloc.getNumAllocatedFragments());
    frame = ins.txPeek();
    REQUIRE(nullptr != frame);
    REQUIRE(frame->payload_size == 64);
    REQUIRE(0 == std::memcmp(frame->payload, payload.data() + 63U, 49));
    REQUIRE(std::all_of(reinterpret_cast<const std::uint8_t*>(frame->payload) + 49,  // Check padding.
                        reinterpret_cast<const std::uint8_t*>(frame->payload) + 61,
                        [](auto x) { return x == 0U; }));
    REQUIRE((CRC112Padding12 >> 8U) == reinterpret_cast<const std::uint8_t*>(frame->payload)[61]);    // CRC
    REQUIRE((CRC112Padding12 & 0xFFU) == reinterpret_cast<const std::uint8_t*>(frame->payload)[62]);  // CRC
    REQUIRE((0b01000000U | 27U) == reinterpret_cast<const std::uint8_t*>(frame->payload)[63]);        // Tail
    REQUIRE(frame->timestamp_usec == 1'000'000'003'000ULL);
    ins.txPop();
    ins.getAllocator().deallocate(frame);
    REQUIRE(0 == ins.getTxQueueLength());
    REQUIRE(0 == alloc.getNumAllocatedFragments());

    // Single-frame empty.
    transfer.timestamp_usec = 1'000'000'004'000ULL;
    transfer.transfer_id    = 28;
    transfer.payload_size   = 0;
    transfer.payload        = nullptr;  // Null is OK if size is 0.
    REQUIRE(1 == ins.txPush(transfer));
    REQUIRE(1 == ins.getTxQueueLength());
    REQUIRE(1 == alloc.getNumAllocatedFragments());
    REQUIRE(60 > alloc.getTotalAllocatedAmount());
    REQUIRE(ins.getTxQueueRoot()->frame.timestamp_usec == 1'000'000'004'000ULL);
    REQUIRE(ins.getTxQueueRoot()->frame.payload_size == 1);
    REQUIRE(ins.getTxQueueRoot()->isStartOfTransfer());
    REQUIRE(ins.getTxQueueRoot()->isEndOfTransfer());
    REQUIRE(ins.getTxQueueRoot()->isToggleBitSet());
    frame = ins.txPeek();
    REQUIRE(nullptr != frame);
    REQUIRE(frame->payload_size == 1);
    REQUIRE((0b11100000U | 28U) == reinterpret_cast<const std::uint8_t*>(frame->payload)[0]);
    REQUIRE(frame->timestamp_usec == 1'000'000'004'000ULL);
    ins.txPop();
    ins.getAllocator().deallocate(frame);
    REQUIRE(0 == ins.getTxQueueLength());
    REQUIRE(0 == alloc.getNumAllocatedFragments());

    // Nothing left to peek at.
    frame = ins.txPeek();
    REQUIRE(nullptr == frame);

    // Invalid transfer.
    transfer.payload        = payload.data();
    transfer.timestamp_usec = 1'000'000'005'000ULL;
    transfer.transfer_kind  = CanardTransferKindMessage;
    transfer.remote_node_id = 42;
    transfer.transfer_id    = 123;
    transfer.payload_size   = 8;
    REQUIRE(-CANARD_ERROR_INVALID_ARGUMENT == ins.txPush(transfer));
    frame = ins.txPeek();
    REQUIRE(nullptr == frame);

    // Error handling.
    REQUIRE(-CANARD_ERROR_INVALID_ARGUMENT == canardTxPush(nullptr, nullptr));
    REQUIRE(-CANARD_ERROR_INVALID_ARGUMENT == canardTxPush(nullptr, &transfer));
    REQUIRE(-CANARD_ERROR_INVALID_ARGUMENT == canardTxPush(&ins.getInstance(), nullptr));
    transfer.payload_size = 1;
    transfer.payload      = nullptr;
    REQUIRE(-CANARD_ERROR_INVALID_ARGUMENT == ins.txPush(transfer));

    REQUIRE(nullptr == canardTxPeek(nullptr));

    canardTxPop(&ins.getInstance());  // No effect.
}
