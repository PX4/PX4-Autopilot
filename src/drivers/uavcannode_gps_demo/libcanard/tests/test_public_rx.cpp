// This software is distributed under the terms of the MIT License.
// Copyright (c) 2016-2020 UAVCAN Development Team.

#include "exposed.hpp"
#include "helpers.hpp"
#include <cstring>

// clang-tidy mistakenly suggests to avoid C arrays here, which is clearly an error
template <typename P, std::size_t N>
auto ensureAllNullptr(P* (&arr)[N]) -> bool  // NOLINT
{
    return std::all_of(std::begin(arr), std::end(arr), [](const auto* const x) { return x == nullptr; });
}

TEST_CASE("RxBasic0")
{
    using helpers::Instance;
    using exposed::RxSession;

    Instance              ins;
    CanardTransfer        transfer{};
    CanardRxSubscription* subscription = nullptr;

    const auto accept = [&](const std::uint8_t               redundant_transport_index,
                            const CanardMicrosecond          timestamp_usec,
                            const std::uint32_t              extended_can_id,
                            const std::vector<std::uint8_t>& payload) {
        static std::vector<std::uint8_t> payload_storage;
        payload_storage = payload;
        CanardFrame frame{};
        frame.timestamp_usec  = timestamp_usec;
        frame.extended_can_id = extended_can_id;
        frame.payload_size    = std::size(payload);
        frame.payload         = payload_storage.data();
        return ins.rxAccept(frame, redundant_transport_index, transfer, &subscription);
    };

    ins.getAllocator().setAllocationCeiling(sizeof(RxSession) + 16);  // A session and a 16-byte payload buffer.

    // No subscriptions by default.
    REQUIRE(ins.getInstance().rx_subscriptions[0] == nullptr);
    REQUIRE(ins.getInstance().rx_subscriptions[1] == nullptr);
    REQUIRE(ins.getInstance().rx_subscriptions[2] == nullptr);

    // A valid single-frame transfer for which there is no subscription.
    subscription = nullptr;
    REQUIRE(0 == accept(0, 100'000'000, 0b001'00'0'11'0110011001100'0'0100111, {0b111'00000}));
    REQUIRE(subscription == nullptr);

    // Create a message subscription.
    CanardRxSubscription sub_msg{};
    REQUIRE(1 == ins.rxSubscribe(CanardTransferKindMessage, 0b0110011001100, 32, 2'000'000, sub_msg));  // New.
    REQUIRE(0 == ins.rxSubscribe(CanardTransferKindMessage, 0b0110011001100, 16, 1'000'000, sub_msg));  // Replaced.
    REQUIRE(ins.getInstance().rx_subscriptions[0] == &sub_msg);
    REQUIRE(ins.getInstance().rx_subscriptions[0]->next == nullptr);
    REQUIRE(ins.getInstance().rx_subscriptions[0]->port_id == 0b0110011001100);
    REQUIRE(ins.getInstance().rx_subscriptions[0]->extent == 16);
    REQUIRE(ins.getInstance().rx_subscriptions[0]->transfer_id_timeout_usec == 1'000'000);
    REQUIRE(ensureAllNullptr(ins.getInstance().rx_subscriptions[0]->_sessions));
    REQUIRE(ins.getInstance().rx_subscriptions[1] == nullptr);
    REQUIRE(ins.getInstance().rx_subscriptions[2] == nullptr);

    // Create a request subscription.
    CanardRxSubscription sub_req{};
    REQUIRE(1 == ins.rxSubscribe(CanardTransferKindRequest, 0b0000110011, 20, 3'000'000, sub_req));
    REQUIRE(ins.getInstance().rx_subscriptions[0] == &sub_msg);
    REQUIRE(ins.getInstance().rx_subscriptions[1] == nullptr);
    REQUIRE(ins.getInstance().rx_subscriptions[2] == &sub_req);
    REQUIRE(ins.getInstance().rx_subscriptions[2]->next == nullptr);
    REQUIRE(ins.getInstance().rx_subscriptions[2]->port_id == 0b0000110011);
    REQUIRE(ins.getInstance().rx_subscriptions[2]->extent == 20);
    REQUIRE(ins.getInstance().rx_subscriptions[2]->transfer_id_timeout_usec == 3'000'000);
    REQUIRE(ensureAllNullptr(ins.getInstance().rx_subscriptions[2]->_sessions));

    // Create a response subscription.
    CanardRxSubscription sub_res{};
    REQUIRE(1 == ins.rxSubscribe(CanardTransferKindResponse, 0b0000111100, 10, 100'000, sub_res));
    REQUIRE(ins.getInstance().rx_subscriptions[0] == &sub_msg);
    REQUIRE(ins.getInstance().rx_subscriptions[1] == &sub_res);
    REQUIRE(ins.getInstance().rx_subscriptions[1]->next == nullptr);
    REQUIRE(ins.getInstance().rx_subscriptions[1]->port_id == 0b0000111100);
    REQUIRE(ins.getInstance().rx_subscriptions[1]->extent == 10);
    REQUIRE(ins.getInstance().rx_subscriptions[1]->transfer_id_timeout_usec == 100'000);
    REQUIRE(ensureAllNullptr(ins.getInstance().rx_subscriptions[1]->_sessions));
    REQUIRE(ins.getInstance().rx_subscriptions[2] == &sub_req);

    // Create a second response subscription.
    CanardRxSubscription sub_res2{};
    REQUIRE(1 == ins.rxSubscribe(CanardTransferKindResponse, 0b0000000000, 10, 1'000, sub_res2));
    REQUIRE(ins.getInstance().rx_subscriptions[0] == &sub_msg);
    REQUIRE(ins.getInstance().rx_subscriptions[1] == &sub_res2);
    REQUIRE(ins.getInstance().rx_subscriptions[1]->next == &sub_res);
    REQUIRE(ins.getInstance().rx_subscriptions[1]->port_id == 0b0000000000);
    REQUIRE(ins.getInstance().rx_subscriptions[1]->extent == 10);
    REQUIRE(ins.getInstance().rx_subscriptions[1]->transfer_id_timeout_usec == 1'000);
    REQUIRE(ensureAllNullptr(ins.getInstance().rx_subscriptions[1]->_sessions));
    REQUIRE(ins.getInstance().rx_subscriptions[2] == &sub_req);

    // Accepted message.
    subscription = nullptr;
    REQUIRE(1 == accept(0, 100'000'001, 0b001'00'0'11'0110011001100'0'0100111, {0b111'00000}));
    REQUIRE(subscription != nullptr);
    REQUIRE(subscription->port_id == 0b0110011001100);
    REQUIRE(transfer.timestamp_usec == 100'000'001);
    REQUIRE(transfer.priority == CanardPriorityImmediate);
    REQUIRE(transfer.transfer_kind == CanardTransferKindMessage);
    REQUIRE(transfer.port_id == 0b0110011001100);
    REQUIRE(transfer.remote_node_id == 0b0100111);
    REQUIRE(transfer.transfer_id == 0);
    REQUIRE(transfer.payload_size == 0);
    REQUIRE(0 == std::memcmp(transfer.payload, "", 0));
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 2);  // The SESSION and the PAYLOAD BUFFER.
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == (sizeof(RxSession) + 16));
    REQUIRE(ins.getInstance().rx_subscriptions[0]->_sessions[0b0100111] != nullptr);
    const auto* msg_payload = transfer.payload;  // Will need it later.

    // Provide the space for an extra session and its payload.
    ins.getAllocator().setAllocationCeiling(sizeof(RxSession) * 2 + 16 + 20);

    // Dropped request because the local node does not have a node-ID.
    subscription = nullptr;
    REQUIRE(0 == accept(0, 100'000'002, 0b011'11'0000110011'0011010'0100111, {0b111'00010}));
    REQUIRE(subscription == nullptr);

    // Dropped request because the local node has a different node-ID.
    ins.setNodeID(0b0011010);
    subscription = nullptr;
    REQUIRE(0 == accept(0, 100'000'002, 0b011'11'0000110011'0011011'0100111, {0b111'00011}));
    REQUIRE(subscription == nullptr);

    // Same request accepted now.
    subscription = nullptr;
    REQUIRE(1 == accept(0, 100'000'002, 0b011'11'0000110011'0011010'0100101, {1, 2, 3, 0b111'00100}));
    REQUIRE(subscription != nullptr);
    REQUIRE(subscription->port_id == 0b0000110011);
    REQUIRE(transfer.timestamp_usec == 100'000'002);
    REQUIRE(transfer.priority == CanardPriorityHigh);
    REQUIRE(transfer.transfer_kind == CanardTransferKindRequest);
    REQUIRE(transfer.port_id == 0b0000110011);
    REQUIRE(transfer.remote_node_id == 0b0100101);
    REQUIRE(transfer.transfer_id == 4);
    REQUIRE(transfer.payload_size == 3);
    REQUIRE(0 == std::memcmp(transfer.payload, "\x01\x02\x03", 3));
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 4);  // Two SESSIONS and two PAYLOAD BUFFERS.
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == (2 * sizeof(RxSession) + 16 + 20));
    REQUIRE(ins.getInstance().rx_subscriptions[2]->_sessions[0b0100101] != nullptr);

    // Response transfer not accepted because the local node has a different node-ID.
    // There is no dynamic memory available, but it doesn't matter because a rejection does not require allocation.
    subscription = nullptr;
    REQUIRE(0 == accept(0, 100'000'002, 0b100'10'0000110011'0100111'0011011, {10, 20, 30, 0b111'00000}));
    REQUIRE(subscription == nullptr);

    // Response transfer not accepted due to OOM -- can't allocate RX session.
    subscription = nullptr;
    REQUIRE(-CANARD_ERROR_OUT_OF_MEMORY ==
            accept(0, 100'000'003, 0b100'10'0000111100'0011010'0011011, {5, 0b111'00001}));
    REQUIRE(subscription != nullptr);  // Subscription get assigned before error code
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 4);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == (2 * sizeof(RxSession) + 16 + 20));

    // Response transfer not accepted due to OOM -- can't allocate the buffer (RX session is allocated OK).
    ins.getAllocator().setAllocationCeiling(3 * sizeof(RxSession) + 16 + 20);
    subscription = nullptr;
    REQUIRE(-CANARD_ERROR_OUT_OF_MEMORY ==
            accept(0, 100'000'003, 0b100'10'0000111100'0011010'0011011, {5, 0b111'00010}));
    REQUIRE(subscription != nullptr);  // Subscription get assigned before error code
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 5);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == (3 * sizeof(RxSession) + 16 + 20));

    // Destroy the message subscription and the buffer to free up memory.
    REQUIRE(1 == ins.rxUnsubscribe(CanardTransferKindMessage, 0b0110011001100));
    REQUIRE(0 == ins.rxUnsubscribe(CanardTransferKindMessage, 0b0110011001100));  // Repeat, nothing to do.
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 4);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == (2 * sizeof(RxSession) + 16 + 20));
    ins.getAllocator().deallocate(msg_payload);
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 3);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == (2 * sizeof(RxSession) + 20));

    // Same response accepted now. We have to keep incrementing the transfer-ID though because it's tracked.
    subscription = nullptr;
    REQUIRE(1 == accept(0, 100'000'003, 0b100'10'0000111100'0011010'0011011, {5, 0b111'00011}));
    REQUIRE(subscription != nullptr);
    REQUIRE(subscription->port_id == 0b0000111100);
    REQUIRE(transfer.timestamp_usec == 100'000'003);
    REQUIRE(transfer.priority == CanardPriorityNominal);
    REQUIRE(transfer.transfer_kind == CanardTransferKindResponse);
    REQUIRE(transfer.port_id == 0b0000111100);
    REQUIRE(transfer.remote_node_id == 0b0011011);
    REQUIRE(transfer.transfer_id == 3);
    REQUIRE(transfer.payload_size == 1);
    REQUIRE(0 == std::memcmp(transfer.payload, "\x05", 1));
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 4);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == (2 * sizeof(RxSession) + 10 + 20));
    REQUIRE(ins.getInstance().rx_subscriptions[1]->next->_sessions[0b0011011] != nullptr);

    // Bad frames shall be rejected silently.
    subscription = nullptr;
    REQUIRE(0 == accept(0, 900'000'000, 0b100'10'0000111100'0011010'0011011, {5, 0b110'00011}));
    REQUIRE(subscription == nullptr);
    subscription = nullptr;
    REQUIRE(0 == accept(0, 900'000'001, 0b100'10'0000111100'0011010'0011011, {}));
    REQUIRE(subscription == nullptr);

    // Unsubscribe.
    REQUIRE(1 == ins.rxUnsubscribe(CanardTransferKindRequest, 0b0000110011));
    REQUIRE(0 == ins.rxUnsubscribe(CanardTransferKindRequest, 0b0000110011));
    REQUIRE(1 == ins.rxUnsubscribe(CanardTransferKindResponse, 0b0000111100));
    REQUIRE(0 == ins.rxUnsubscribe(CanardTransferKindResponse, 0b0000111100));
    REQUIRE(1 == ins.rxUnsubscribe(CanardTransferKindResponse, 0b0000000000));
    REQUIRE(0 == ins.rxUnsubscribe(CanardTransferKindResponse, 0b0000000000));
}

TEST_CASE("RxAnonymous")
{
    using helpers::Instance;
    using exposed::RxSession;

    Instance              ins;
    CanardTransfer        transfer{};
    CanardRxSubscription* subscription = nullptr;

    const auto accept = [&](const std::uint8_t               redundant_transport_index,
                            const CanardMicrosecond          timestamp_usec,
                            const std::uint32_t              extended_can_id,
                            const std::vector<std::uint8_t>& payload) {
        static std::vector<std::uint8_t> payload_storage;
        payload_storage = payload;
        CanardFrame frame{};
        frame.timestamp_usec  = timestamp_usec;
        frame.extended_can_id = extended_can_id;
        frame.payload_size    = std::size(payload);
        frame.payload         = payload_storage.data();
        return ins.rxAccept(frame, redundant_transport_index, transfer, &subscription);
    };

    ins.getAllocator().setAllocationCeiling(16);

    // A valid anonymous transfer for which there is no subscription.
    subscription = nullptr;
    REQUIRE(0 == accept(0, 100'000'000, 0b001'01'0'11'0110011001100'0'0100111, {0b111'00000}));
    REQUIRE(subscription == nullptr);

    // Create a message subscription.
    CanardRxSubscription sub_msg{};
    REQUIRE(1 == ins.rxSubscribe(CanardTransferKindMessage, 0b0110011001100, 16, 2'000'000, sub_msg));  // New.

    // Accepted anonymous message.
    subscription = nullptr;
    REQUIRE(1 == accept(0,
                        100'000'001,
                        0b001'01'0'11'0110011001100'0'0100111,  //
                        {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 0b111'00000}));
    REQUIRE(subscription != nullptr);
    REQUIRE(subscription->port_id == 0b0110011001100);
    REQUIRE(transfer.timestamp_usec == 100'000'001);
    REQUIRE(transfer.priority == CanardPriorityImmediate);
    REQUIRE(transfer.transfer_kind == CanardTransferKindMessage);
    REQUIRE(transfer.port_id == 0b0110011001100);
    REQUIRE(transfer.remote_node_id == CANARD_NODE_ID_UNSET);
    REQUIRE(transfer.transfer_id == 0);
    REQUIRE(transfer.payload_size == 16);  // Truncated.
    REQUIRE(0 == std::memcmp(transfer.payload, "\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0A\x0B\x0C\x0D\x0E\x0F\x10", 0));
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 1);  // The PAYLOAD BUFFER only! No session for anons.
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 16);
    REQUIRE(ensureAllNullptr(ins.getInstance().rx_subscriptions[0]->_sessions));  // No RX states!

    // Anonymous message not accepted because OOM. The transfer shall remain unmodified by the call, so we re-check it.
    REQUIRE(-CANARD_ERROR_OUT_OF_MEMORY ==
            accept(0, 100'000'001, 0b001'01'0'11'0110011001100'0'0100111, {3, 2, 1, 0b111'00000}));
    REQUIRE(subscription != nullptr);
    REQUIRE(subscription->port_id == 0b0110011001100);
    REQUIRE(transfer.timestamp_usec == 100'000'001);
    REQUIRE(transfer.priority == CanardPriorityImmediate);
    REQUIRE(transfer.transfer_kind == CanardTransferKindMessage);
    REQUIRE(transfer.port_id == 0b0110011001100);
    REQUIRE(transfer.remote_node_id == CANARD_NODE_ID_UNSET);
    REQUIRE(transfer.transfer_id == 0);
    REQUIRE(transfer.payload_size == 16);  // Truncated.
    REQUIRE(0 == std::memcmp(transfer.payload, "\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0A\x0B\x0C\x0D\x0E\x0F\x10", 0));
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 1);  // The PAYLOAD BUFFER only! No session for anons.
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 16);
    REQUIRE(ensureAllNullptr(ins.getInstance().rx_subscriptions[0]->_sessions));  // No RX states!

    // Release the memory.
    ins.getAllocator().deallocate(transfer.payload);
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 0);
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 0);

    // Accepted anonymous message with small payload.
    subscription = nullptr;
    REQUIRE(1 == accept(0, 100'000'001, 0b001'01'0'11'0110011001100'0'0100111, {1, 2, 3, 4, 5, 6, 0b111'00000}));
    REQUIRE(subscription != nullptr);
    REQUIRE(subscription->port_id == 0b0110011001100);
    REQUIRE(transfer.timestamp_usec == 100'000'001);
    REQUIRE(transfer.priority == CanardPriorityImmediate);
    REQUIRE(transfer.transfer_kind == CanardTransferKindMessage);
    REQUIRE(transfer.port_id == 0b0110011001100);
    REQUIRE(transfer.remote_node_id == CANARD_NODE_ID_UNSET);
    REQUIRE(transfer.transfer_id == 0);
    REQUIRE(transfer.payload_size == 6);  // NOT truncated.
    REQUIRE(0 == std::memcmp(transfer.payload, "\x01\x02\x03\x04\x05\x06", 0));
    REQUIRE(ins.getAllocator().getNumAllocatedFragments() == 1);  // The PAYLOAD BUFFER only! No session for anons.
    REQUIRE(ins.getAllocator().getTotalAllocatedAmount() == 6);   // Smaller allocation.
    REQUIRE(ensureAllNullptr(ins.getInstance().rx_subscriptions[0]->_sessions));  // No RX states!
}

TEST_CASE("RxSubscriptionErrors")
{
    using helpers::Instance;
    Instance             ins;
    CanardRxSubscription sub{};

    const union
    {
        std::uint64_t      bits;
        CanardTransferKind value;
    } kind{std::numeric_limits<std::uint64_t>::max()};

    REQUIRE(-CANARD_ERROR_INVALID_ARGUMENT == canardRxSubscribe(nullptr, CanardTransferKindMessage, 0, 0, 0, &sub));
    REQUIRE(-CANARD_ERROR_INVALID_ARGUMENT == canardRxSubscribe(&ins.getInstance(), kind.value, 0, 0, 0, &sub));
    REQUIRE(-CANARD_ERROR_INVALID_ARGUMENT ==
            canardRxSubscribe(&ins.getInstance(), CanardTransferKindMessage, 0, 0, 0, nullptr));

    REQUIRE(-CANARD_ERROR_INVALID_ARGUMENT == canardRxUnsubscribe(nullptr, CanardTransferKindMessage, 0));
    REQUIRE(-CANARD_ERROR_INVALID_ARGUMENT == canardRxUnsubscribe(&ins.getInstance(), kind.value, 0));

    CanardFrame frame{};
    frame.payload_size = 1U;
    CanardTransfer transfer{};
    REQUIRE(-CANARD_ERROR_INVALID_ARGUMENT == canardRxAccept(&ins.getInstance(), &frame, 0, &transfer));
    REQUIRE(-CANARD_ERROR_INVALID_ARGUMENT == canardRxAccept(nullptr, &frame, 0, &transfer));
    REQUIRE(-CANARD_ERROR_INVALID_ARGUMENT == canardRxAccept(&ins.getInstance(), nullptr, 0, &transfer));
    REQUIRE(-CANARD_ERROR_INVALID_ARGUMENT == canardRxAccept(&ins.getInstance(), &frame, 0, nullptr));
    REQUIRE(-CANARD_ERROR_INVALID_ARGUMENT == canardRxAccept(nullptr, nullptr, 0, nullptr));
}
