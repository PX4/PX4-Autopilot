// This software is distributed under the terms of the MIT License.
// Copyright (c) 2016-2020 UAVCAN Development Team.

#include "exposed.hpp"
#include "helpers.hpp"
#include <cstring>

TEST_CASE("rxTryParseFrame")
{
    using exposed::RxFrameModel;
    using exposed::rxTryParseFrame;

    RxFrameModel model{};

    const auto parse = [&](const CanardMicrosecond          timestamp_usec,
                           const std::uint32_t              extended_can_id,
                           const std::vector<std::uint8_t>& payload) {
        static std::vector<std::uint8_t> payload_storage;
        payload_storage = payload;
        CanardFrame frame{};
        frame.timestamp_usec  = timestamp_usec;
        frame.extended_can_id = extended_can_id;
        frame.payload_size    = std::size(payload);
        frame.payload         = payload_storage.data();
        model                 = RxFrameModel{};
        return rxTryParseFrame(&frame, &model);
    };

    // MESSAGE
    REQUIRE(parse(543210U, 0U, {0, 1, 2, 3, 4, 5, 6, 7}));
    REQUIRE(model.timestamp_usec == 543210U);
    REQUIRE(model.priority == CanardPriorityExceptional);
    REQUIRE(model.transfer_kind == CanardTransferKindMessage);
    REQUIRE(model.port_id == 0U);
    REQUIRE(model.source_node_id == 0U);
    REQUIRE(model.destination_node_id == CANARD_NODE_ID_UNSET);
    REQUIRE(model.transfer_id == 7U);
    REQUIRE(!model.start_of_transfer);
    REQUIRE(!model.end_of_transfer);
    REQUIRE(!model.toggle);
    REQUIRE(model.payload_size == 7);
    REQUIRE(model.payload[0] == 0);
    REQUIRE(model.payload[1] == 1);
    REQUIRE(model.payload[2] == 2);
    REQUIRE(model.payload[3] == 3);
    REQUIRE(model.payload[4] == 4);
    REQUIRE(model.payload[5] == 5);
    REQUIRE(model.payload[6] == 6);
    // SIMILAR BUT INVALID
    REQUIRE(!parse(543210U, 0U, {}));                     // NO TAIL BYTE
    REQUIRE(!parse(543210U, 0U, {0}));                    // MFT FRAMES REQUIRE PAYLOAD
    REQUIRE(!parse(543210U, 0U, {0, 1, 2, 3, 4, 5, 6}));  // MFT NON-LAST FRAME PAYLOAD CAN'T BE SHORTER THAN 7

    // MESSAGE
    REQUIRE(parse(123456U, 0b001'00'0'11'0110011001100'0'0100111U, {0, 1, 2, 3, 4, 5, 6, 0b101'00000U | 23U}));
    REQUIRE(model.timestamp_usec == 123456U);
    REQUIRE(model.priority == CanardPriorityImmediate);
    REQUIRE(model.transfer_kind == CanardTransferKindMessage);
    REQUIRE(model.port_id == 0b0110011001100U);
    REQUIRE(model.source_node_id == 0b0100111U);
    REQUIRE(model.destination_node_id == CANARD_NODE_ID_UNSET);
    REQUIRE(model.transfer_id == 23U);
    REQUIRE(model.start_of_transfer);
    REQUIRE(!model.end_of_transfer);
    REQUIRE(model.toggle);
    REQUIRE(model.payload_size == 7);
    REQUIRE(model.payload[0] == 0);
    REQUIRE(model.payload[1] == 1);
    REQUIRE(model.payload[2] == 2);
    REQUIRE(model.payload[3] == 3);
    REQUIRE(model.payload[4] == 4);
    REQUIRE(model.payload[5] == 5);
    REQUIRE(model.payload[6] == 6);
    // SIMILAR BUT INVALID
    // NO TAIL BYTE
    REQUIRE(!parse(123456U, 0b001'00'0'11'0110011001100'0'0100111U, {}));
    // BAD TOGGLE
    REQUIRE(!parse(123456U, 0b001'00'0'11'0110011001100'0'0100111U, {0, 1, 2, 3, 4, 5, 6, 0b100'00000U | 23U}));
    // BAD RESERVED R07
    REQUIRE(!parse(123456U, 0b001'00'0'11'0110011001100'1'0100111U, {0, 1, 2, 3, 4, 5, 6, 0b101'00000U | 23U}));
    // BAD RESERVED R23
    REQUIRE(!parse(123456U, 0b001'00'1'11'0110011001100'0'0100111U, {0, 1, 2, 3, 4, 5, 6, 0b101'00000U | 23U}));
    // BAD RESERVED R07 R23
    REQUIRE(!parse(123456U, 0b001'00'1'11'0110011001100'1'0100111U, {0, 1, 2, 3, 4, 5, 6, 0b101'00000U | 23U}));
    // ANON NOT SINGLE FRAME
    REQUIRE(!parse(123456U, 0b001'01'0'11'0110011001100'0'0100111U, {0, 1, 2, 3, 4, 5, 6, 0b101'00000U | 23U}));

    // ANONYMOUS MESSAGE
    REQUIRE(parse(12345U, 0b010'01'0'00'0110011001101'0'0100111U, {0b111'00000U | 0U}));
    REQUIRE(model.timestamp_usec == 12345U);
    REQUIRE(model.priority == CanardPriorityFast);
    REQUIRE(model.transfer_kind == CanardTransferKindMessage);
    REQUIRE(model.port_id == 0b0110011001101U);
    REQUIRE(model.source_node_id == CANARD_NODE_ID_UNSET);
    REQUIRE(model.destination_node_id == CANARD_NODE_ID_UNSET);
    REQUIRE(model.transfer_id == 0U);
    REQUIRE(model.start_of_transfer);
    REQUIRE(model.end_of_transfer);
    REQUIRE(model.toggle);
    REQUIRE(model.payload_size == 0);
    // SAME BUT RESERVED 21 22 SET (and ignored)
    REQUIRE(parse(12345U, 0b010'01'0'11'0110011001101'0'0100111U, {0b111'00000U | 0U}));
    REQUIRE(model.port_id == 0b0110011001101U);
    // SIMILAR BUT INVALID
    REQUIRE(!parse(12345U, 0b010'01'0'11'0110011001100'0'0100111U, {}));                   // NO TAIL BYTE
    REQUIRE(!parse(12345U, 0b010'01'0'11'0110011001100'0'0100111U, {0b110'00000U | 0U}));  // BAD TOGGLE
    REQUIRE(!parse(12345U, 0b010'01'0'11'0110011001100'1'0100111U, {0b111'00000U | 0U}));  // BAD RESERVED 07
    REQUIRE(!parse(12345U, 0b010'01'1'11'0110011001100'0'0100111U, {0b111'00000U | 0U}));  // BAD RESERVED 23
    REQUIRE(!parse(12345U, 0b010'01'1'11'0110011001100'1'0100111U, {0b111'00000U | 0U}));  // BAD RESERVED 07 23

    // REQUEST
    REQUIRE(parse(999'999U, 0b011'11'0000110011'0011010'0100111U, {0, 1, 2, 3, 0b011'00000U | 31U}));
    REQUIRE(model.timestamp_usec == 999'999U);
    REQUIRE(model.priority == CanardPriorityHigh);
    REQUIRE(model.transfer_kind == CanardTransferKindRequest);
    REQUIRE(model.port_id == 0b0000110011U);
    REQUIRE(model.source_node_id == 0b0100111U);
    REQUIRE(model.destination_node_id == 0b0011010U);
    REQUIRE(model.transfer_id == 31U);
    REQUIRE(!model.start_of_transfer);
    REQUIRE(model.end_of_transfer);
    REQUIRE(model.toggle);
    REQUIRE(model.payload_size == 4);
    REQUIRE(model.payload[0] == 0);
    REQUIRE(model.payload[1] == 1);
    REQUIRE(model.payload[2] == 2);
    REQUIRE(model.payload[3] == 3);
    // SIMILAR BUT INVALID
    REQUIRE(!parse(999'999U, 0b011'11'0000110011'0011010'0100111U, {}));                                // NO TAIL BYTE
    REQUIRE(!parse(999'999U, 0b011'11'0000110011'0011010'0100111U, {0, 1, 2, 3, 0b110'00000U | 31U}));  // BAD TOGGLE
    REQUIRE(!parse(999'999U, 0b011'11'1000110011'0011010'0100111U, {0, 1, 2, 3, 0b011'00000U | 31U}));  // BAD RESERVED
    REQUIRE(!parse(999'999U, 0b011'11'0000110011'0100111'0100111U, {0, 1, 2, 3, 0b011'00000U | 31U}));  // SRC == DST

    // RESPONSE
    REQUIRE(parse(888'888U, 0b100'10'0000110011'0100111'0011010U, {255, 0b010'00000U | 1U}));
    REQUIRE(model.timestamp_usec == 888'888U);
    REQUIRE(model.priority == CanardPriorityNominal);
    REQUIRE(model.transfer_kind == CanardTransferKindResponse);
    REQUIRE(model.port_id == 0b0000110011U);
    REQUIRE(model.source_node_id == 0b0011010U);
    REQUIRE(model.destination_node_id == 0b0100111U);
    REQUIRE(model.transfer_id == 1U);
    REQUIRE(!model.start_of_transfer);
    REQUIRE(model.end_of_transfer);
    REQUIRE(!model.toggle);
    REQUIRE(model.payload_size == 1);
    REQUIRE(model.payload[0] == 255);
    // SIMILAR BUT INVALID
    REQUIRE(!parse(888'888U, 0b100'10'0000110011'0100111'0011010U, {}));                        // NO TAIL BYTE
    REQUIRE(!parse(888'888U, 0b100'10'0000110011'0100111'0011010U, {255, 0b100'00000U | 1U}));  // BAD TOGGLE
    REQUIRE(!parse(888'888U, 0b100'10'1000110011'0100111'0011010U, {255, 0b010'00000U | 1U}));  // BAD RESERVED
    REQUIRE(!parse(888'888U, 0b100'10'0000110011'0011010'0011010U, {255, 0b010'00000U | 1U}));  // SRC == DST
}

TEST_CASE("rxSessionWritePayload")
{
    using helpers::Instance;
    using exposed::RxSession;
    using exposed::rxSessionWritePayload;
    using exposed::rxSessionRestart;

    Instance  ins;
    RxSession rxs;
    rxs.transfer_id = 0U;

    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 0);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 0);

    // Regular write, the RX state is uninitialized so a new allocation will take place.
    REQUIRE(0 == rxSessionWritePayload(&ins.getInstance(), &rxs, 10, 5, "\x00\x01\x02\x03\x04"));
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 1);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 10);
    REQUIRE(rxs.payload_size == 5);
    REQUIRE(rxs.payload != nullptr);
    REQUIRE(rxs.payload[0] == 0);
    REQUIRE(rxs.payload[1] == 1);
    REQUIRE(rxs.payload[2] == 2);
    REQUIRE(rxs.payload[3] == 3);
    REQUIRE(rxs.payload[4] == 4);

    // Appending the pre-allocated storage.
    REQUIRE(0 == rxSessionWritePayload(&ins.getInstance(), &rxs, 10, 4, "\x05\x06\x07\x08"));
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 1);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 10);
    REQUIRE(rxs.payload_size == 9);
    REQUIRE(rxs.payload != nullptr);
    REQUIRE(rxs.payload[0] == 0);
    REQUIRE(rxs.payload[1] == 1);
    REQUIRE(rxs.payload[2] == 2);
    REQUIRE(rxs.payload[3] == 3);
    REQUIRE(rxs.payload[4] == 4);
    REQUIRE(rxs.payload[5] == 5);
    REQUIRE(rxs.payload[6] == 6);
    REQUIRE(rxs.payload[7] == 7);
    REQUIRE(rxs.payload[8] == 8);

    // Implicit truncation -- too much payload, excess ignored.
    REQUIRE(0 == rxSessionWritePayload(&ins.getInstance(), &rxs, 10, 3, "\x09\x0A\x0B"));
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 1);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 10);
    REQUIRE(rxs.payload_size == 10);
    REQUIRE(rxs.payload != nullptr);
    REQUIRE(rxs.payload[0] == 0);
    REQUIRE(rxs.payload[1] == 1);
    REQUIRE(rxs.payload[2] == 2);
    REQUIRE(rxs.payload[3] == 3);
    REQUIRE(rxs.payload[4] == 4);
    REQUIRE(rxs.payload[5] == 5);
    REQUIRE(rxs.payload[6] == 6);
    REQUIRE(rxs.payload[7] == 7);
    REQUIRE(rxs.payload[8] == 8);
    REQUIRE(rxs.payload[9] == 9);

    // Storage is already full, write ignored.
    REQUIRE(0 == rxSessionWritePayload(&ins.getInstance(), &rxs, 10, 3, "\x0C\x0D\x0E"));
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 1);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 10);
    REQUIRE(rxs.payload_size == 10);
    REQUIRE(rxs.payload != nullptr);
    REQUIRE(rxs.payload[0] == 0);
    REQUIRE(rxs.payload[1] == 1);
    REQUIRE(rxs.payload[2] == 2);
    REQUIRE(rxs.payload[3] == 3);
    REQUIRE(rxs.payload[4] == 4);
    REQUIRE(rxs.payload[5] == 5);
    REQUIRE(rxs.payload[6] == 6);
    REQUIRE(rxs.payload[7] == 7);
    REQUIRE(rxs.payload[8] == 8);
    REQUIRE(rxs.payload[9] == 9);

    // Restart frees the buffer. The transfer-ID will be incremented, too.
    rxSessionRestart(&ins.getInstance(), &rxs);
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 0);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 0);
    REQUIRE(rxs.payload_size == 0);
    REQUIRE(rxs.payload == nullptr);
    REQUIRE(rxs.calculated_crc == 0xFFFFU);
    REQUIRE(rxs.transfer_id == 1);
    REQUIRE(rxs.toggle);

    // Double restart has no effect on memory.
    rxs.calculated_crc = 0x1234U;
    rxs.transfer_id    = 23;
    rxs.toggle         = false;
    rxSessionRestart(&ins.getInstance(), &rxs);
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 0);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 0);
    REQUIRE(rxs.payload_size == 0);
    REQUIRE(rxs.payload == nullptr);
    REQUIRE(rxs.calculated_crc == 0xFFFFU);
    REQUIRE(rxs.transfer_id == 24U);
    REQUIRE(rxs.toggle);

    // Restart with a transfer-ID overflow.
    rxs.calculated_crc = 0x1234U;
    rxs.transfer_id    = 31;
    rxs.toggle         = false;
    rxSessionRestart(&ins.getInstance(), &rxs);
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 0);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 0);
    REQUIRE(rxs.payload_size == 0);
    REQUIRE(rxs.payload == nullptr);
    REQUIRE(rxs.calculated_crc == 0xFFFFU);
    REQUIRE(rxs.transfer_id == 0U);
    REQUIRE(rxs.toggle);

    // Write into a zero-capacity storage. NULL at the output.
    REQUIRE(0 == rxSessionWritePayload(&ins.getInstance(), &rxs, 0, 3, "\x00\x01\x02"));
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 0);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 0);
    REQUIRE(rxs.payload_size == 0);
    REQUIRE(rxs.payload == nullptr);

    // Write with OOM.
    ins.getAllocator().setAllocationCeiling(5);
    REQUIRE(-CANARD_ERROR_OUT_OF_MEMORY == rxSessionWritePayload(&ins.getInstance(), &rxs, 10, 3, "\x00\x01\x02"));
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 0);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 0);
    REQUIRE(rxs.payload_size == 0);
    REQUIRE(rxs.payload == nullptr);
}

TEST_CASE("rxSessionUpdate")
{
    using helpers::Instance;
    using exposed::RxSession;
    using exposed::RxFrameModel;
    using exposed::rxSessionUpdate;
    using exposed::crcAdd;

    Instance ins;
    ins.getAllocator().setAllocationCeiling(16);

    RxFrameModel frame;
    frame.timestamp_usec      = 10'000'000;
    frame.priority            = CanardPrioritySlow;
    frame.transfer_kind       = CanardTransferKindMessage;
    frame.port_id             = 2'222;
    frame.source_node_id      = 55;
    frame.destination_node_id = CANARD_NODE_ID_UNSET;
    frame.transfer_id         = 11;
    frame.start_of_transfer   = true;
    frame.end_of_transfer     = true;
    frame.toggle              = true;
    frame.payload_size        = 3;
    frame.payload             = reinterpret_cast<const uint8_t*>("\x01\x01\x01");

    RxSession rxs;
    rxs.transfer_id               = 31;
    rxs.redundant_transport_index = 1;

    CanardTransfer transfer{};

    const auto update = [&](const std::uint8_t  redundant_transport_index,
                            const std::uint64_t tid_timeout_usec,
                            const std::size_t   extent) {
        return rxSessionUpdate(&ins.getInstance(),
                               &rxs,
                               &frame,
                               redundant_transport_index,
                               tid_timeout_usec,
                               extent,
                               &transfer);
    };

    const auto crc = [](const char* const string) { return crcAdd(0xFFFF, std::strlen(string), string); };

    // Accept one transfer.
    REQUIRE(1 == update(1, 1'000'000, 16));
    REQUIRE(rxs.transfer_timestamp_usec == 10'000'000);
    REQUIRE(rxs.payload_size == 0);   // Handed over to the output transfer.
    REQUIRE(rxs.payload == nullptr);  // Handed over to the output transfer.
    REQUIRE(rxs.calculated_crc == 0xFFFF);
    REQUIRE(rxs.transfer_id == 12U);  // Incremented.
    REQUIRE(rxs.toggle);
    REQUIRE(rxs.redundant_transport_index == 1);
    REQUIRE(transfer.timestamp_usec == 10'000'000);
    REQUIRE(transfer.priority == CanardPrioritySlow);
    REQUIRE(transfer.transfer_kind == CanardTransferKindMessage);
    REQUIRE(transfer.port_id == 2'222);
    REQUIRE(transfer.remote_node_id == 55);
    REQUIRE(transfer.transfer_id == 11);
    REQUIRE(transfer.payload_size == 3);
    REQUIRE(0 == std::memcmp(transfer.payload, "\x01\x01\x01", 3));
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 1);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 16);
    ins.getAllocator().deallocate(transfer.payload);

    // Valid next transfer, wrong transport.
    frame.timestamp_usec = 10'000'100;
    frame.transfer_id    = 12;
    frame.payload        = reinterpret_cast<const uint8_t*>("\x02\x02\x02");
    REQUIRE(0 == update(2, 1'000'000, 16));
    REQUIRE(rxs.transfer_timestamp_usec == 10'000'000);
    REQUIRE(rxs.payload_size == 0);   // Handed over to the output transfer.
    REQUIRE(rxs.payload == nullptr);  // Handed over to the output transfer.
    REQUIRE(rxs.calculated_crc == 0xFFFF);
    REQUIRE(rxs.transfer_id == 12U);  // Incremented.
    REQUIRE(rxs.toggle);
    REQUIRE(rxs.redundant_transport_index == 1);

    // Correct transport.
    frame.timestamp_usec = 10'000'050;
    frame.payload        = reinterpret_cast<const uint8_t*>("\x03\x03\x03");
    REQUIRE(1 == update(1, 1'000'000, 16));
    REQUIRE(rxs.transfer_timestamp_usec == 10'000'050);
    REQUIRE(rxs.payload_size == 0);
    REQUIRE(rxs.payload == nullptr);
    REQUIRE(rxs.calculated_crc == 0xFFFF);
    REQUIRE(rxs.transfer_id == 13U);
    REQUIRE(rxs.toggle);
    REQUIRE(rxs.redundant_transport_index == 1);
    REQUIRE(transfer.timestamp_usec == 10'000'050);
    REQUIRE(transfer.priority == CanardPrioritySlow);
    REQUIRE(transfer.transfer_kind == CanardTransferKindMessage);
    REQUIRE(transfer.port_id == 2'222);
    REQUIRE(transfer.remote_node_id == 55);
    REQUIRE(transfer.transfer_id == 12);
    REQUIRE(transfer.payload_size == 3);
    REQUIRE(0 == std::memcmp(transfer.payload, "\x03\x03\x03", 3));
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 1);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 16);
    ins.getAllocator().deallocate(transfer.payload);

    // Same TID.
    frame.timestamp_usec = 10'000'200;
    frame.transfer_id    = 12;
    frame.payload        = reinterpret_cast<const uint8_t*>("\x04\x04\x04");
    REQUIRE(0 == update(1, 1'000'200, 16));
    REQUIRE(rxs.transfer_timestamp_usec == 10'000'050);
    REQUIRE(rxs.payload_size == 0);
    REQUIRE(rxs.payload == nullptr);
    REQUIRE(rxs.calculated_crc == 0xFFFF);
    REQUIRE(rxs.transfer_id == 13U);
    REQUIRE(rxs.toggle);
    REQUIRE(rxs.redundant_transport_index == 1);

    // Restart due to TID timeout, switch iface.
    frame.timestamp_usec = 20'000'000;
    frame.transfer_id    = 12;
    frame.payload        = reinterpret_cast<const uint8_t*>("\x05\x05\x05");
    REQUIRE(1 == update(0, 1'000'000, 16));
    REQUIRE(rxs.transfer_timestamp_usec == 20'000'000);
    REQUIRE(rxs.payload_size == 0);
    REQUIRE(rxs.payload == nullptr);
    REQUIRE(rxs.calculated_crc == 0xFFFF);
    REQUIRE(rxs.transfer_id == 13U);
    REQUIRE(rxs.toggle);
    REQUIRE(rxs.redundant_transport_index == 0);
    REQUIRE(transfer.timestamp_usec == 20'000'000);
    REQUIRE(transfer.priority == CanardPrioritySlow);
    REQUIRE(transfer.transfer_kind == CanardTransferKindMessage);
    REQUIRE(transfer.port_id == 2'222);
    REQUIRE(transfer.remote_node_id == 55);
    REQUIRE(transfer.transfer_id == 12);
    REQUIRE(transfer.payload_size == 3);
    REQUIRE(0 == std::memcmp(transfer.payload, "\x05\x05\x05", 3));
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 1);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 16);
    ins.getAllocator().deallocate(transfer.payload);

    // Multi-frame, first.
    frame.timestamp_usec  = 20'000'100;
    frame.transfer_id     = 13;
    frame.end_of_transfer = false;
    frame.payload_size    = 7;
    frame.payload         = reinterpret_cast<const uint8_t*>("\x06\x06\x06\x06\x06\x06\x06");
    REQUIRE(0 == update(0, 1'000'000, 16));
    REQUIRE(rxs.transfer_timestamp_usec == 20'000'100);
    REQUIRE(rxs.payload_size == 7);
    REQUIRE(0 == std::memcmp(rxs.payload, "\x06\x06\x06\x06\x06\x06\x06", 7));
    REQUIRE(rxs.calculated_crc == crc("\x06\x06\x06\x06\x06\x06\x06"));
    REQUIRE(rxs.transfer_id == 13U);
    REQUIRE(!rxs.toggle);
    REQUIRE(rxs.redundant_transport_index == 0);
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 1);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 16);

    // Multi-frame, middle.
    frame.timestamp_usec    = 20'000'200;
    frame.start_of_transfer = false;
    frame.end_of_transfer   = false;
    frame.toggle            = false;
    frame.payload           = reinterpret_cast<const uint8_t*>("\x07\x07\x07\x07\x07\x07\x07");
    REQUIRE(0 == update(0, 1'000'000, 16));
    REQUIRE(rxs.transfer_timestamp_usec == 20'000'100);
    REQUIRE(rxs.payload_size == 14);
    REQUIRE(0 == std::memcmp(rxs.payload, "\x06\x06\x06\x06\x06\x06\x06\x07\x07\x07\x07\x07\x07\x07", 14));
    REQUIRE(rxs.calculated_crc == crc("\x06\x06\x06\x06\x06\x06\x06\x07\x07\x07\x07\x07\x07\x07"));
    REQUIRE(rxs.transfer_id == 13U);
    REQUIRE(rxs.toggle);
    REQUIRE(rxs.redundant_transport_index == 0);
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 1);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 16);

    // Multi-frame, last, bad toggle.
    frame.timestamp_usec    = 20'000'300;
    frame.start_of_transfer = false;
    frame.end_of_transfer   = true;
    frame.toggle            = false;
    frame.payload           = reinterpret_cast<const uint8_t*>("\x08\x08\x08\x08");
    REQUIRE(0 == update(0, 1'000'000, 16));
    REQUIRE(rxs.transfer_timestamp_usec == 20'000'100);
    REQUIRE(rxs.payload_size == 14);
    REQUIRE(0 == std::memcmp(rxs.payload, "\x06\x06\x06\x06\x06\x06\x06\x07\x07\x07\x07\x07\x07\x07", 14));
    REQUIRE(rxs.calculated_crc == crc("\x06\x06\x06\x06\x06\x06\x06\x07\x07\x07\x07\x07\x07\x07"));
    REQUIRE(rxs.transfer_id == 13U);
    REQUIRE(rxs.toggle);
    REQUIRE(rxs.redundant_transport_index == 0);
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 1);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 16);

    // Multi-frame, last.
    frame.timestamp_usec    = 20'000'400;
    frame.start_of_transfer = false;
    frame.end_of_transfer   = true;
    frame.toggle            = true;
    frame.payload_size      = 6;  // The payload is IMPLICITLY TRUNCATED, and the CRC IS STILL VALIDATED.
    frame.payload           = reinterpret_cast<const uint8_t*>("\x09\x09\x09\x09\r\x93");
    REQUIRE(1 == update(0, 1'000'000, 16));
    REQUIRE(rxs.transfer_timestamp_usec == 20'000'100);  // First frame.
    REQUIRE(rxs.payload_size == 0);
    REQUIRE(rxs.payload == nullptr);
    REQUIRE(rxs.calculated_crc == 0xFFFF);
    REQUIRE(rxs.transfer_id == 14U);
    REQUIRE(rxs.toggle);
    REQUIRE(rxs.redundant_transport_index == 0);
    REQUIRE(transfer.timestamp_usec == 20'000'100);
    REQUIRE(transfer.priority == CanardPrioritySlow);
    REQUIRE(transfer.transfer_kind == CanardTransferKindMessage);
    REQUIRE(transfer.port_id == 2'222);
    REQUIRE(transfer.remote_node_id == 55);
    REQUIRE(transfer.transfer_id == 13);
    REQUIRE(transfer.payload_size == 16);
    REQUIRE(0 == std::memcmp(transfer.payload, "\x06\x06\x06\x06\x06\x06\x06\x07\x07\x07\x07\x07\x07\x07\x09\x09", 16));
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 1);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 16);
    ins.getAllocator().deallocate(transfer.payload);

    // Restart by TID timeout, not the first frame.
    frame.timestamp_usec    = 30'000'000;
    frame.transfer_id       = 12;  // Goes back.
    frame.start_of_transfer = false;
    frame.end_of_transfer   = false;
    frame.toggle            = true;
    frame.payload_size      = 7;
    frame.payload           = reinterpret_cast<const uint8_t*>("\x0A\x0A\x0A\x0A\x0A\x0A\x0A");
    REQUIRE(0 == update(2, 1'000'000, 16));
    REQUIRE(rxs.transfer_timestamp_usec == 20'000'100);  // No change.
    REQUIRE(rxs.payload_size == 0);
    REQUIRE(rxs.payload == nullptr);
    REQUIRE(rxs.calculated_crc == 0xFFFF);
    REQUIRE(rxs.transfer_id == 13U);
    REQUIRE(rxs.toggle);
    REQUIRE(rxs.redundant_transport_index == 2);
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 0);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 0);

    // Restart by TID mismatch.
    frame.timestamp_usec    = 20'000'200;  // Goes back.
    frame.transfer_id       = 11;          // Goes back.
    frame.start_of_transfer = true;
    frame.end_of_transfer   = false;
    frame.toggle            = true;
    frame.payload_size      = 7;
    frame.payload           = reinterpret_cast<const uint8_t*>("\x0B\x0B\x0B\x0B\x0B\x0B\x0B");
    REQUIRE(0 == update(2, 1'000'000, 16));
    REQUIRE(rxs.transfer_timestamp_usec == 20'000'200);
    REQUIRE(rxs.payload_size == 7);
    REQUIRE(0 == std::memcmp(rxs.payload, "\x0B\x0B\x0B\x0B\x0B\x0B\x0B", 7));
    REQUIRE(rxs.calculated_crc == crc("\x0B\x0B\x0B\x0B\x0B\x0B\x0B"));
    REQUIRE(rxs.transfer_id == 11U);
    REQUIRE(!rxs.toggle);
    REQUIRE(rxs.redundant_transport_index == 2);
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 1);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 16);

    // Duplicate start rejected (toggle mismatch).
    frame.timestamp_usec    = 20'000'300;
    frame.transfer_id       = 11;
    frame.start_of_transfer = true;
    frame.end_of_transfer   = true;
    frame.toggle            = true;
    frame.payload_size      = 7;
    frame.payload           = reinterpret_cast<const uint8_t*>("\x0C\x0C\x0C\x0C\x0C\x0C\x0C");
    REQUIRE(0 == update(2, 1'000'000, 16));
    REQUIRE(rxs.transfer_timestamp_usec == 20'000'200);
    REQUIRE(rxs.payload_size == 7);
    REQUIRE(0 == std::memcmp(rxs.payload, "\x0B\x0B\x0B\x0B\x0B\x0B\x0B", 7));
    REQUIRE(rxs.calculated_crc == crc("\x0B\x0B\x0B\x0B\x0B\x0B\x0B"));
    REQUIRE(rxs.transfer_id == 11U);
    REQUIRE(!rxs.toggle);
    REQUIRE(rxs.redundant_transport_index == 2);
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 1);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 16);

    // Continue & finalize.
    frame.timestamp_usec    = 20'000'400;
    frame.transfer_id       = 11;
    frame.start_of_transfer = false;
    frame.end_of_transfer   = true;
    frame.toggle            = false;
    frame.payload_size      = 5;
    frame.payload           = reinterpret_cast<const uint8_t*>("\x0D\x0D\x0DWd");  // CRC at the end.
    REQUIRE(1 == update(2, 1'000'000, 16));
    REQUIRE(rxs.transfer_timestamp_usec == 20'000'200);
    REQUIRE(rxs.payload_size == 0);
    REQUIRE(rxs.payload == nullptr);
    REQUIRE(rxs.calculated_crc == 0xFFFF);
    REQUIRE(rxs.transfer_id == 12U);
    REQUIRE(rxs.toggle);
    REQUIRE(rxs.redundant_transport_index == 2);
    REQUIRE(transfer.timestamp_usec == 20'000'200);
    REQUIRE(transfer.priority == CanardPrioritySlow);
    REQUIRE(transfer.transfer_kind == CanardTransferKindMessage);
    REQUIRE(transfer.port_id == 2'222);
    REQUIRE(transfer.remote_node_id == 55);
    REQUIRE(transfer.transfer_id == 11);
    REQUIRE(transfer.payload_size == 10);
    REQUIRE(0 == std::memcmp(transfer.payload, "\x0B\x0B\x0B\x0B\x0B\x0B\x0B\x0D\x0D\x0D", 10));
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 1);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 16);
    ins.getAllocator().deallocate(transfer.payload);

    // CRC SPLIT -- first frame.
    frame.timestamp_usec    = 30'000'000;
    frame.transfer_id       = 0;
    frame.start_of_transfer = true;
    frame.end_of_transfer   = false;
    frame.toggle            = true;
    frame.payload_size      = 8;
    frame.payload           = reinterpret_cast<const uint8_t*>("\x0E\x0E\x0E\x0E\x0E\x0E\x0E\xF7");
    REQUIRE(0 == update(0, 1'000'000, 16));
    REQUIRE(rxs.transfer_timestamp_usec == 30'000'000);
    REQUIRE(rxs.payload_size == 8);
    REQUIRE(0 == std::memcmp(rxs.payload, "\x0E\x0E\x0E\x0E\x0E\x0E\x0E\xF7", 8));
    REQUIRE(rxs.calculated_crc == crc("\x0E\x0E\x0E\x0E\x0E\x0E\x0E\xF7"));
    REQUIRE(rxs.transfer_id == 0);
    REQUIRE(!rxs.toggle);
    REQUIRE(rxs.redundant_transport_index == 0);
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 1);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 16);

    // CRC SPLIT -- second (last) frame.
    frame.timestamp_usec    = 30'000'100;
    frame.transfer_id       = 0;
    frame.start_of_transfer = false;
    frame.end_of_transfer   = true;
    frame.toggle            = false;
    frame.payload_size      = 1;
    frame.payload           = reinterpret_cast<const uint8_t*>("\xD7");
    REQUIRE(1 == update(0, 1'000'000, 16));
    REQUIRE(rxs.transfer_timestamp_usec == 30'000'000);
    REQUIRE(rxs.payload_size == 0);
    REQUIRE(rxs.payload == nullptr);
    REQUIRE(rxs.calculated_crc == 0xFFFF);
    REQUIRE(rxs.transfer_id == 1U);
    REQUIRE(rxs.toggle);
    REQUIRE(rxs.redundant_transport_index == 0);
    REQUIRE(transfer.timestamp_usec == 30'000'000);
    REQUIRE(transfer.priority == CanardPrioritySlow);
    REQUIRE(transfer.transfer_kind == CanardTransferKindMessage);
    REQUIRE(transfer.port_id == 2'222);
    REQUIRE(transfer.remote_node_id == 55);
    REQUIRE(transfer.transfer_id == 0);
    REQUIRE(transfer.payload_size == 7);  // ONE CRC BYTE BACKTRACKED!
    REQUIRE(0 == std::memcmp(transfer.payload, "\x0E\x0E\x0E\x0E\x0E\x0E\x0E", 7));
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 1);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 16);
    ins.getAllocator().deallocate(transfer.payload);

    // BAD CRC -- first frame.
    frame.timestamp_usec    = 30'001'000;
    frame.transfer_id       = 31;
    frame.start_of_transfer = true;
    frame.end_of_transfer   = false;
    frame.toggle            = true;
    frame.payload_size      = 8;
    frame.payload           = reinterpret_cast<const uint8_t*>("\x0E\x0E\x0E\x0E\x0E\x0E\x0E\xF7");
    REQUIRE(0 == update(0, 1'000'000, 16));
    REQUIRE(rxs.transfer_timestamp_usec == 30'001'000);
    REQUIRE(rxs.payload_size == 8);
    REQUIRE(0 == std::memcmp(rxs.payload, "\x0E\x0E\x0E\x0E\x0E\x0E\x0E\xF7", 8));
    REQUIRE(rxs.calculated_crc == crc("\x0E\x0E\x0E\x0E\x0E\x0E\x0E\xF7"));
    REQUIRE(rxs.transfer_id == 31U);
    REQUIRE(!rxs.toggle);
    REQUIRE(rxs.redundant_transport_index == 0);
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 1);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 16);

    // BAD CRC -- second (last) frame.
    frame.timestamp_usec    = 30'001'100;
    frame.transfer_id       = 31;
    frame.start_of_transfer = false;
    frame.end_of_transfer   = true;
    frame.toggle            = false;
    frame.payload_size      = 1;
    frame.payload           = reinterpret_cast<const uint8_t*>("\xD8");
    REQUIRE(0 == update(0, 1'000'000, 16));
    REQUIRE(rxs.transfer_timestamp_usec == 30'001'000);
    REQUIRE(rxs.payload_size == 0);
    REQUIRE(rxs.payload == nullptr);
    REQUIRE(rxs.calculated_crc == 0xFFFF);
    REQUIRE(rxs.transfer_id == 0U);
    REQUIRE(rxs.toggle);
    REQUIRE(rxs.redundant_transport_index == 0);
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 0);  // Deallocated on failure.
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 0);

    // OOM -- reset on error.
    frame.timestamp_usec    = 40'000'000;
    frame.transfer_id       = 30;
    frame.start_of_transfer = true;
    frame.end_of_transfer   = false;
    frame.toggle            = true;
    frame.payload_size      = 8;
    frame.payload           = reinterpret_cast<const uint8_t*>("\x0E\x0E\x0E\x0E\x0E\x0E\x0E\xF7");
    REQUIRE((-CANARD_ERROR_OUT_OF_MEMORY) == update(2, 1'000'000, 17));  // Exceeds the heap quota.
    REQUIRE(rxs.transfer_timestamp_usec == 40'000'000);
    REQUIRE(rxs.payload_size == 0);
    REQUIRE(rxs.payload == nullptr);
    REQUIRE(rxs.calculated_crc == 0xFFFF);
    REQUIRE(rxs.transfer_id == 31U);  // Reset.
    REQUIRE(rxs.toggle);
    REQUIRE(rxs.redundant_transport_index == 2);
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 0);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 0);
}
