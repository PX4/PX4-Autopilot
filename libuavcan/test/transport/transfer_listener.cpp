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

    Transfer makeTransfer(uavcan::TransferType transfer_type, uint8_t source_node_id, const std::string& payload)
    {
        return IncomingTransferEmulatorBase::makeTransfer(transfer_type, source_node_id, payload, data_type_);
    }
};


TEST(TransferListener, BasicMFT)
{
    const uavcan::DataTypeDescriptor type(uavcan::DataTypeKindMessage, 123, uavcan::DataTypeSignature(123456789), "A");

    static const int NUM_POOL_BLOCKS = 12;    // This number is just enough to pass the test
    uavcan::PoolAllocator<uavcan::MemPoolBlockSize * NUM_POOL_BLOCKS, uavcan::MemPoolBlockSize> pool;
    uavcan::PoolManager<1> poolmgr;
    poolmgr.addPool(&pool);

    uavcan::TransferPerfCounter perf;
    TestListener<256, 1, 1> subscriber(perf, type, poolmgr);

    /*
     * Test data
     */
    static const std::string DATA[] =
    {
        "123456789",

        "Build a man a fire, and he'll be warm for a day. "
        "Set a man on fire, and he'll be warm for the rest of his life.",

        "The USSR, which they'd begun to renovate and improve at about the time when Tatarsky decided to "
        "change his profession, improved so much that it ceased to exist",

        "In the beginning there was nothing, which exploded.",

        "BEWARE JET BLAST"
    };

    TransferListenerEmulator emulator(subscriber, type);
    const Transfer transfers[] =
    {
        emulator.makeTransfer(uavcan::TransferTypeMessageBroadcast, 1, DATA[0]),
        emulator.makeTransfer(uavcan::TransferTypeMessageUnicast,   1, DATA[1]),   // Same NID
        emulator.makeTransfer(uavcan::TransferTypeMessageUnicast,   2, DATA[2]),
        emulator.makeTransfer(uavcan::TransferTypeServiceRequest,   3, DATA[3]),
        emulator.makeTransfer(uavcan::TransferTypeServiceResponse,  4, DATA[4]),
    };

    /*
     * Sending concurrently
     * Expected reception order: 0, 4, 3, 1, 2
     */
    emulator.send(transfers);

    ASSERT_TRUE(subscriber.matchAndPop(transfers[0]));
    ASSERT_TRUE(subscriber.matchAndPop(transfers[4]));
    ASSERT_TRUE(subscriber.matchAndPop(transfers[3]));
    ASSERT_TRUE(subscriber.matchAndPop(transfers[1]));
    ASSERT_TRUE(subscriber.matchAndPop(transfers[2]));

    ASSERT_TRUE(subscriber.isEmpty());
}


TEST(TransferListener, CrcFailure)
{
    const uavcan::DataTypeDescriptor type(uavcan::DataTypeKindMessage, 123, uavcan::DataTypeSignature(123456789), "A");

    uavcan::PoolManager<1> poolmgr;                         // No dynamic memory
    uavcan::TransferPerfCounter perf;
    TestListener<256, 2, 2> subscriber(perf, type, poolmgr);  // Static buffer only, 2 entries

    /*
     * Generating transfers with damaged payload (CRC is not valid)
     */
    TransferListenerEmulator emulator(subscriber, type);
    const Transfer tr_mft = emulator.makeTransfer(uavcan::TransferTypeMessageBroadcast, 42, "123456789abcdefghik");
    const Transfer tr_sft = emulator.makeTransfer(uavcan::TransferTypeMessageUnicast, 11, "abcd");

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

    uavcan::PoolManager<1> poolmgr;                        // No dynamic memory. At all.
    uavcan::TransferPerfCounter perf;
    TestListener<0, 0, 5> subscriber(perf, type, poolmgr); // Max buf size is 0, i.e. SFT-only

    TransferListenerEmulator emulator(subscriber, type);
    const Transfer transfers[] =
    {
        emulator.makeTransfer(uavcan::TransferTypeMessageBroadcast, 1, "123"),
        emulator.makeTransfer(uavcan::TransferTypeMessageUnicast,   1, "456"),   // Same NID
        emulator.makeTransfer(uavcan::TransferTypeMessageUnicast,   2, ""),
        emulator.makeTransfer(uavcan::TransferTypeServiceRequest,   3, "abc"),
        emulator.makeTransfer(uavcan::TransferTypeServiceResponse,  4, ""),
        emulator.makeTransfer(uavcan::TransferTypeServiceResponse,  2, ""),      // New TT, ignored due to OOM
        emulator.makeTransfer(uavcan::TransferTypeMessageUnicast,   2, "foo"),   // Same as 2, not ignored
        emulator.makeTransfer(uavcan::TransferTypeMessageUnicast,   2, "123456789abc"), // Same as 2, not SFT - ignore
        emulator.makeTransfer(uavcan::TransferTypeMessageUnicast,   2, "bar"),   // Same as 2, not ignored
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

    uavcan::PoolManager<1> poolmgr;                           // No dynamic memory
    uavcan::TransferPerfCounter perf;
    TestListener<256, 1, 2> subscriber(perf, type, poolmgr);  // Static buffer only, 1 entry

    /*
     * Generating transfers
     */
    TransferListenerEmulator emulator(subscriber, type);
    const Transfer tr_mft = emulator.makeTransfer(uavcan::TransferTypeMessageBroadcast, 42, "123456789abcdefghik");
    const Transfer tr_sft = emulator.makeTransfer(uavcan::TransferTypeMessageUnicast, 11, "abcd");

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

    uavcan::PoolManager<1> poolmgr;
    uavcan::TransferPerfCounter perf;
    TestListener<uavcan::MaxTransferPayloadLen * 2, 2, 2> subscriber(perf, type, poolmgr);

    static const std::string DATA_OK(uavcan::MaxTransferPayloadLen, 'z');

    TransferListenerEmulator emulator(subscriber, type);
    const Transfer transfers[] =
    {
        emulator.makeTransfer(uavcan::TransferTypeMessageUnicast,   1, DATA_OK),
        emulator.makeTransfer(uavcan::TransferTypeMessageBroadcast, 1, DATA_OK)
    };

    emulator.send(transfers);

    ASSERT_TRUE(subscriber.matchAndPop(transfers[1]));    // Broadcast is shorter, so will complete first
    ASSERT_TRUE(subscriber.matchAndPop(transfers[0]));

    ASSERT_TRUE(subscriber.isEmpty());
}
