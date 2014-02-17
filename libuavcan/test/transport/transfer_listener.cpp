/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include "transfer_test_helpers.hpp"


class Emulator
{
    uavcan::TransferListenerBase& target_;
    const uavcan::DataTypeDescriptor type_;
    uint64_t ts_;
    uavcan::TransferID tid_;
    uavcan::NodeID target_node_id_;

public:
    Emulator(uavcan::TransferListenerBase& target, const uavcan::DataTypeDescriptor& type,
             uavcan::NodeID target_node_id = 127)
    : target_(target)
    , type_(type)
    , ts_(0)
    , target_node_id_(target_node_id)
    { }

    Transfer makeTransfer(uavcan::TransferType transfer_type, uint8_t source_node_id, const std::string& payload)
    {
        ts_ += 100;
        const Transfer tr(ts_, ts_ + 1000000000ul, transfer_type, tid_, source_node_id, payload);
        tid_.increment();
        return tr;
    }

    void send(const std::vector<std::vector<uavcan::RxFrame> >& sers)
    {
        unsigned int index = 0;
        while (true)
        {
            // Sending all transfers concurrently
            bool all_empty = true;
            for (std::vector<std::vector<uavcan::RxFrame> >::const_iterator it = sers.begin(); it != sers.end(); ++it)
            {
                if (it->size() <= index)
                    continue;
                all_empty = false;
                std::cout << "Emulator: Sending: " << it->at(index).toString() << std::endl;
                target_.handleFrame(it->at(index));
            }
            index++;
            if (all_empty)
                break;
        }
    }

    void send(const Transfer* transfers, unsigned int num_transfers)
    {
        std::vector<std::vector<uavcan::RxFrame> > sers;
        while (num_transfers--)
        {
            const uavcan::NodeID dnid = (transfers->transfer_type == uavcan::TRANSFER_TYPE_MESSAGE_BROADCAST)
                ? uavcan::NodeID::BROADCAST : target_node_id_;
            sers.push_back(serializeTransfer(*transfers++, dnid, type_));
        }
        send(sers);
    }

    template <int SIZE>
    void send(const Transfer (&transfers)[SIZE])
    {
        send(transfers, SIZE);
    }
};


TEST(TransferListener, BasicMFT)
{
    uavcan::DataTypeDescriptor type(uavcan::DATA_TYPE_KIND_MESSAGE, 123, uavcan::DataTypeHash());
    for (int i = 0; i < uavcan::DataTypeHash::NUM_BYTES; i++)
        type.hash.value[i] = i | (i << 4);

    static const int NUM_POOL_BLOCKS = 12;    // This number is just enough to pass the test
    uavcan::PoolAllocator<uavcan::MEM_POOL_BLOCK_SIZE * NUM_POOL_BLOCKS, uavcan::MEM_POOL_BLOCK_SIZE> pool;
    uavcan::PoolManager<1> poolmgr;
    poolmgr.addPool(&pool);

    TestSubscriber<256, 1, 1> subscriber(&type, &poolmgr);

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

    Emulator emulator(subscriber, type);
    const Transfer transfers[] =
    {
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_MESSAGE_BROADCAST, 1, DATA[0]),
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_MESSAGE_UNICAST,   1, DATA[1]),   // Same NID
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_MESSAGE_UNICAST,   2, DATA[2]),
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_SERVICE_REQUEST,   3, DATA[3]),
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_SERVICE_RESPONSE,  4, DATA[4]),
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
    uavcan::DataTypeDescriptor type(uavcan::DATA_TYPE_KIND_MESSAGE, 123, uavcan::DataTypeHash());
    for (int i = 0; i < uavcan::DataTypeHash::NUM_BYTES; i++)
        type.hash.value[i] = i | (i << 4);

    uavcan::PoolManager<1> poolmgr;                         // No dynamic memory
    TestSubscriber<256, 2, 2> subscriber(&type, &poolmgr);  // Static buffer only, 2 entries

    /*
     * Generating transfers with damaged payload (CRC is not valid)
     */
    Emulator emulator(subscriber, type);
    const Transfer tr_mft = emulator.makeTransfer(uavcan::TRANSFER_TYPE_MESSAGE_BROADCAST, 42, "123456789abcdefghik");
    const Transfer tr_sft = emulator.makeTransfer(uavcan::TRANSFER_TYPE_MESSAGE_UNICAST, 11, "abcd");

    std::vector<uavcan::RxFrame> ser_mft = serializeTransfer(tr_mft, 0, type);
    std::vector<uavcan::RxFrame> ser_sft = serializeTransfer(tr_sft, 9, type);

    ASSERT_TRUE(ser_mft.size() > 1);
    ASSERT_TRUE(ser_sft.size() == 1);

    // Fuck my brain.
    const_cast<uint8_t*>(ser_mft[1].getPayloadPtr())[1] = ~ser_mft[1].getPayloadPtr()[1];// CRC is no longer valid
    const_cast<uint8_t*>(ser_sft[0].getPayloadPtr())[2] = ~ser_sft[0].getPayloadPtr()[2];// no CRC - will be undetected

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
    tr_sft_damaged.payload[2] = ~tr_sft.payload[2];     // Damaging the data similarly, so that it can be matched

    ASSERT_TRUE(subscriber.matchAndPop(tr_sft_damaged));
    ASSERT_TRUE(subscriber.isEmpty());
}


TEST(TransferListener, BasicSFT)
{
    uavcan::DataTypeDescriptor type(uavcan::DATA_TYPE_KIND_MESSAGE, 123, uavcan::DataTypeHash());
    for (int i = 0; i < uavcan::DataTypeHash::NUM_BYTES; i++)
        type.hash.value[i] = i | (i << 4);

    uavcan::PoolManager<1> poolmgr;                         // No dynamic memory. At all.
    TestSubscriber<0, 0, 5> subscriber(&type, &poolmgr);    // Max buf size is 0, i.e. SFT-only

    Emulator emulator(subscriber, type);
    const Transfer transfers[] =
    {
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_MESSAGE_BROADCAST, 1, "123"),
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_MESSAGE_UNICAST,   1, "456"),   // Same NID
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_MESSAGE_UNICAST,   2, ""),
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_SERVICE_REQUEST,   3, "abc"),
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_SERVICE_RESPONSE,  4, ""),
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_SERVICE_RESPONSE,  2, ""),      // New TT, ignored due to OOM
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_MESSAGE_UNICAST,   2, "foo"),   // Same as 2, not ignored
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_MESSAGE_UNICAST,   2, "123456789abc"),// Same as 2, not SFT - ignore
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_MESSAGE_UNICAST,   2, "bar"),   // Same as 2, not ignored
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
    uavcan::DataTypeDescriptor type(uavcan::DATA_TYPE_KIND_MESSAGE, 123, uavcan::DataTypeHash());
    for (int i = 0; i < uavcan::DataTypeHash::NUM_BYTES; i++)
        type.hash.value[i] = i | (i << 4);

    uavcan::PoolManager<1> poolmgr;                         // No dynamic memory
    TestSubscriber<256, 1, 2> subscriber(&type, &poolmgr);  // Static buffer only, 1 entry

    /*
     * Generating transfers
     */
    Emulator emulator(subscriber, type);
    const Transfer tr_mft = emulator.makeTransfer(uavcan::TRANSFER_TYPE_MESSAGE_BROADCAST, 42, "123456789abcdefghik");
    const Transfer tr_sft = emulator.makeTransfer(uavcan::TRANSFER_TYPE_MESSAGE_UNICAST, 11, "abcd");

    const std::vector<uavcan::RxFrame> ser_mft = serializeTransfer(tr_mft, 0, type);
    const std::vector<uavcan::RxFrame> ser_sft = serializeTransfer(tr_sft, 9, type);

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
    static_cast<uavcan::TransferListenerBase&>(subscriber).cleanup(100000000);

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
    uavcan::DataTypeDescriptor type(uavcan::DATA_TYPE_KIND_MESSAGE, 123, uavcan::DataTypeHash());
    for (int i = 0; i < uavcan::DataTypeHash::NUM_BYTES; i++)
        type.hash.value[i] = i | (i << 4);

    uavcan::PoolManager<1> poolmgr;
    TestSubscriber<uavcan::MAX_TRANSFER_PAYLOAD_LEN * 2, 2, 2> subscriber(&type, &poolmgr);

    static const std::string DATA_OK(uavcan::MAX_TRANSFER_PAYLOAD_LEN, 'z');

    Emulator emulator(subscriber, type);
    const Transfer transfers[] =
    {
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_MESSAGE_UNICAST,   1, DATA_OK),
        emulator.makeTransfer(uavcan::TRANSFER_TYPE_MESSAGE_BROADCAST, 1, DATA_OK)
    };

    emulator.send(transfers);

    ASSERT_TRUE(subscriber.matchAndPop(transfers[1]));    // Broadcast is shorter, so will complete first
    ASSERT_TRUE(subscriber.matchAndPop(transfers[0]));

    ASSERT_TRUE(subscriber.isEmpty());
}
