/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include "transfer_test_helpers.hpp"
#include "../clock.hpp"


class TransferListenerEmulator : public IncomingTransferEmulatorBase
{
    uavcan::TransferListenerBase& target_;
    const uavcan::DataTypeDescriptor data_type_;

public:
    TransferListenerEmulator(uavcan::TransferListenerBase& target, const uavcan::DataTypeDescriptor& type,
                             uavcan::NodeID dst_node_id = 127)
        : IncomingTransferEmulatorBase(dst_node_id)
        , target_(target)
        , data_type_(type)
    { }

    void sendOneFrame(const uavcan::RxFrame& frame) { target_.handleFrame(frame); }

    Transfer makeTransfer(uavcan::TransferPriority priority, uavcan::TransferType transfer_type,
                          uint8_t source_node_id, const std::string& payload)
    {
        return IncomingTransferEmulatorBase::makeTransfer(priority, transfer_type, source_node_id, payload, data_type_);
    }
};


TEST(TransferListener, BasicMFT)
{
    const uavcan::DataTypeDescriptor type(uavcan::DataTypeKindMessage, 123, uavcan::DataTypeSignature(123456789), "A");

    static const int NUM_POOL_BLOCKS = 12;    // This number is just enough to pass the test
    uavcan::PoolAllocator<uavcan::MemPoolBlockSize * NUM_POOL_BLOCKS, uavcan::MemPoolBlockSize> pool;

    uavcan::TransferPerfCounter perf;
    TestListener<256, 1, 1> subscriber(perf, type, pool);

    /*
     * Test data
     */
    static const std::string DATA[] =
    {
        // Too long for unicast messages
        "Build a man a fire, and he'll be warm for a day. "
        "Set a man on fire, and he'll be warm for the rest of his life.",

        "123456789",

        "In the beginning there was nothing, which exploded.",

        // Too long for message transfers
        "The USSR, which they'd begun to renovate and improve at about the time when Tatarsky decided to "
        "change his profession, improved so much that it ceased to exist",

        "BEWARE JET BLAST"
    };

    for (unsigned i = 0; i < sizeof(DATA) / sizeof(DATA[0]); i++)
    {
        std::cout << "Size of test data chunk " << i << ": " << DATA[i].length() << std::endl;
    }

    TransferListenerEmulator emulator(subscriber, type);
    const Transfer transfers[] =
    {
        emulator.makeTransfer(uavcan::TransferPriorityNormal,  uavcan::TransferTypeMessageBroadcast, 1, DATA[0]),
        emulator.makeTransfer(uavcan::TransferPriorityNormal,  uavcan::TransferTypeMessageUnicast,   1, DATA[1]),   // Same NID
        emulator.makeTransfer(uavcan::TransferPriorityNormal,  uavcan::TransferTypeMessageUnicast,   2, DATA[2]),
        emulator.makeTransfer(uavcan::TransferPriorityService, uavcan::TransferTypeServiceRequest,   3, DATA[3]),
        emulator.makeTransfer(uavcan::TransferPriorityService, uavcan::TransferTypeServiceResponse,  4, DATA[4]),
    };

    /*
     * Sending concurrently
     * Expected reception order: 1, 4, 2, 0, 3
     */
    emulator.send(transfers);

    ASSERT_TRUE(subscriber.matchAndPop(transfers[1]));
    ASSERT_TRUE(subscriber.matchAndPop(transfers[4]));
    ASSERT_TRUE(subscriber.matchAndPop(transfers[2]));
    ASSERT_TRUE(subscriber.matchAndPop(transfers[0]));
    ASSERT_TRUE(subscriber.matchAndPop(transfers[3]));

    ASSERT_TRUE(subscriber.isEmpty());
}


TEST(TransferListener, CrcFailure)
{
    const uavcan::DataTypeDescriptor type(uavcan::DataTypeKindMessage, 123, uavcan::DataTypeSignature(123456789), "A");

    NullAllocator poolmgr;                                    // No dynamic memory
    uavcan::TransferPerfCounter perf;
    TestListener<256, 2, 2> subscriber(perf, type, poolmgr);  // Static buffer only, 2 entries

    /*
     * Generating transfers with damaged payload (CRC is not valid)
     */
    TransferListenerEmulator emulator(subscriber, type);
    const Transfer tr_mft = emulator.makeTransfer(uavcan::TransferPriorityNormal, uavcan::TransferTypeMessageBroadcast, 42, "123456789abcdefghik");
    const Transfer tr_sft = emulator.makeTransfer(uavcan::TransferPriorityNormal, uavcan::TransferTypeMessageUnicast, 11, "abcd");

    std::vector<uavcan::RxFrame> ser_mft = serializeTransfer(tr_mft);
    std::vector<uavcan::RxFrame> ser_sft = serializeTransfer(tr_sft);

    ASSERT_TRUE(ser_mft.size() > 1);
    ASSERT_TRUE(ser_sft.size() == 1);

    const_cast<uint8_t*>(ser_mft[1].getPayloadPtr())[1] = uint8_t(~ser_mft[1].getPayloadPtr()[1]); // CRC invalid now
    const_cast<uint8_t*>(ser_sft[0].getPayloadPtr())[2] = uint8_t(~ser_sft[0].getPayloadPtr()[2]);  // no CRC here

    /*
     * Sending and making sure that MFT was not received, but SFT was.
     */
    std::vector<std::vector<uavcan::RxFrame> > sers;
    sers.push_back(ser_mft);
    sers.push_back(ser_sft);
    sers.push_back(ser_mft);  // Ignored
    sers.push_back(ser_sft);  // Ignored

    emulator.send(sers);

    Transfer tr_sft_damaged = tr_sft;
    tr_sft_damaged.payload[2] = char(~tr_sft.payload[2]);    // Damaging the data similarly, so that it can be matched

    ASSERT_TRUE(subscriber.matchAndPop(tr_sft_damaged));
    ASSERT_TRUE(subscriber.isEmpty());
}


TEST(TransferListener, BasicSFT)
{
    const uavcan::DataTypeDescriptor type(uavcan::DataTypeKindMessage, 123, uavcan::DataTypeSignature(123456789), "A");

    NullAllocator poolmgr;                                 // No dynamic memory. At all.
    uavcan::TransferPerfCounter perf;
    TestListener<0, 0, 5> subscriber(perf, type, poolmgr); // Max buf size is 0, i.e. SFT-only

    TransferListenerEmulator emulator(subscriber, type);
    const Transfer transfers[] =
    {
        emulator.makeTransfer(uavcan::TransferPriorityNormal,  uavcan::TransferTypeMessageBroadcast, 1, "123"),
        emulator.makeTransfer(uavcan::TransferPriorityNormal,  uavcan::TransferTypeMessageUnicast,   1, "456"),   // Same NID
        emulator.makeTransfer(uavcan::TransferPriorityNormal,  uavcan::TransferTypeMessageUnicast,   2, ""),
        emulator.makeTransfer(uavcan::TransferPriorityService, uavcan::TransferTypeServiceRequest,   3, "abc"),
        emulator.makeTransfer(uavcan::TransferPriorityService, uavcan::TransferTypeServiceResponse,  4, ""),
        emulator.makeTransfer(uavcan::TransferPriorityService, uavcan::TransferTypeServiceResponse,  2, ""),      // New TT, ignored due to OOM
        emulator.makeTransfer(uavcan::TransferPriorityNormal,  uavcan::TransferTypeMessageUnicast,   2, "foo"),   // Same as 2, not ignored
        emulator.makeTransfer(uavcan::TransferPriorityNormal,  uavcan::TransferTypeMessageUnicast,   2, "123456789abc"), // Same as 2, not SFT - ignore
        emulator.makeTransfer(uavcan::TransferPriorityNormal,  uavcan::TransferTypeMessageUnicast,   2, "bar"),   // Same as 2, not ignored
    };

    emulator.send(transfers);

    ASSERT_TRUE(subscriber.matchAndPop(transfers[0]));
    ASSERT_TRUE(subscriber.matchAndPop(transfers[1]));
    ASSERT_TRUE(subscriber.matchAndPop(transfers[2]));
    ASSERT_TRUE(subscriber.matchAndPop(transfers[3]));
    ASSERT_TRUE(subscriber.matchAndPop(transfers[4]));
    ASSERT_TRUE(subscriber.matchAndPop(transfers[6]));
    ASSERT_TRUE(subscriber.matchAndPop(transfers[8]));

    ASSERT_TRUE(subscriber.isEmpty());
}


TEST(TransferListener, Cleanup)
{
    const uavcan::DataTypeDescriptor type(uavcan::DataTypeKindMessage, 123, uavcan::DataTypeSignature(123456789), "A");

    NullAllocator poolmgr;                                    // No dynamic memory
    uavcan::TransferPerfCounter perf;
    TestListener<256, 1, 2> subscriber(perf, type, poolmgr);  // Static buffer only, 1 entry

    /*
     * Generating transfers
     */
    TransferListenerEmulator emulator(subscriber, type);
    const Transfer tr_mft = emulator.makeTransfer(uavcan::TransferPriorityNormal, uavcan::TransferTypeMessageBroadcast, 42, "123456789abcdefghik");
    const Transfer tr_sft = emulator.makeTransfer(uavcan::TransferPriorityNormal, uavcan::TransferTypeMessageUnicast, 11, "abcd");

    const std::vector<uavcan::RxFrame> ser_mft = serializeTransfer(tr_mft);
    const std::vector<uavcan::RxFrame> ser_sft = serializeTransfer(tr_sft);

    ASSERT_TRUE(ser_mft.size() > 1);
    ASSERT_TRUE(ser_sft.size() == 1);

    const std::vector<uavcan::RxFrame> ser_mft_begin(ser_mft.begin(), ser_mft.begin() + 1);

    /*
     * Sending the first part and SFT
     */
    std::vector<std::vector<uavcan::RxFrame> > sers;
    sers.push_back(ser_mft_begin);      // Only the first part
    sers.push_back(ser_sft);

    emulator.send(sers);

    ASSERT_TRUE(subscriber.matchAndPop(tr_sft));
    ASSERT_TRUE(subscriber.isEmpty());

    /*
     * Cleanup with huge timestamp value will remove all entries
     */
    static_cast<uavcan::TransferListenerBase&>(subscriber).cleanup(tsMono(100000000));

    /*
     * Sending the same transfers again - they will be accepted since registres were cleared
     */
    sers.clear();
    sers.push_back(ser_mft);   // Complete transfer
    sers.push_back(ser_sft);

    emulator.send(sers);

    ASSERT_TRUE(subscriber.matchAndPop(tr_sft));
    ASSERT_TRUE(subscriber.matchAndPop(tr_mft));
    ASSERT_TRUE(subscriber.isEmpty());
}


TEST(TransferListener, MaximumTransferLength)
{
    const uavcan::DataTypeDescriptor type(uavcan::DataTypeKindMessage, 123, uavcan::DataTypeSignature(123456789), "A");

    NullAllocator poolmgr;
    uavcan::TransferPerfCounter perf;
    TestListener<uavcan::MaxPossibleTransferPayloadLen * 2, 3, 3> subscriber(perf, type, poolmgr);

    TransferListenerEmulator emulator(subscriber, type);
    const Transfer transfers[] =
    {
        emulator.makeTransfer(uavcan::TransferPriorityService, uavcan::TransferTypeServiceRequest,   1,
                              std::string(uavcan::MaxServiceTransferPayloadLen, 'z')),            // Longer
        emulator.makeTransfer(uavcan::TransferPriorityNormal, uavcan::TransferTypeMessageUnicast,   2,
                              std::string(uavcan::MaxMessageUnicastTransferPayloadLen, 'z')),     // Shorter
        emulator.makeTransfer(uavcan::TransferPriorityNormal, uavcan::TransferTypeMessageBroadcast, 3,
                              std::string(uavcan::MaxMessageBroadcastTransferPayloadLen, 'z'))    // Same as above
    };

    emulator.send(transfers);

    ASSERT_TRUE(subscriber.matchAndPop(transfers[1]));
    ASSERT_TRUE(subscriber.matchAndPop(transfers[2]));
    ASSERT_TRUE(subscriber.matchAndPop(transfers[0]));    // Service takes more frames

    ASSERT_TRUE(subscriber.isEmpty());
}


TEST(TransferListener, AnonymousTransfers)
{
    const uavcan::DataTypeDescriptor type(uavcan::DataTypeKindMessage, 123, uavcan::DataTypeSignature(123456789), "A");

    NullAllocator poolmgr;
    uavcan::TransferPerfCounter perf;
    TestListener<0, 0, 0> subscriber(perf, type, poolmgr);

    TransferListenerEmulator emulator(subscriber, type);
    const Transfer transfers[] =
    {
        emulator.makeTransfer(uavcan::TransferPriorityNormal, uavcan::TransferTypeMessageUnicast,   0, "12345678"),  // Invalid - not broadcast
        emulator.makeTransfer(uavcan::TransferPriorityNormal, uavcan::TransferTypeMessageBroadcast, 0, "12345678"),  // Valid
        emulator.makeTransfer(uavcan::TransferPriorityNormal, uavcan::TransferTypeMessageBroadcast, 0, "123456789"), // Invalid - not SFT
        emulator.makeTransfer(uavcan::TransferPriorityNormal, uavcan::TransferTypeMessageBroadcast, 0, "")           // Valid
    };

    emulator.send(transfers);

    // Nothing will be received, because anonymous transfers are disabled by default
    ASSERT_TRUE(subscriber.isEmpty());

    subscriber.allowAnonymousTransfers();

    // Re-send everything again
    emulator.send(transfers);

    // Now the anonymous transfers are enabled
    ASSERT_TRUE(subscriber.matchAndPop(transfers[1]));    // Only SFT broadcast will be accepted
    ASSERT_TRUE(subscriber.matchAndPop(transfers[3]));

    ASSERT_TRUE(subscriber.isEmpty());
}

TEST(TransferListener, Sizes)
{
    using namespace uavcan;

    std::cout << "sizeof(TransferListener<64, 1, 2>): " << sizeof(TransferListener<64, 1, 2>) << std::endl;
}
