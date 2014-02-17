/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include "transfer_test_helpers.hpp"


TEST(TransferTestHelpers, Transfer)
{
    uavcan::PoolAllocator<uavcan::MEM_POOL_BLOCK_SIZE * 8, uavcan::MEM_POOL_BLOCK_SIZE> pool;
    uavcan::PoolManager<1> poolmgr;
    poolmgr.addPool(&pool);

    uavcan::TransferBufferManager<128, 1> mgr(&poolmgr);
    uavcan::TransferBufferAccessor tba(&mgr, uavcan::TransferBufferManagerKey(0, uavcan::TRANSFER_TYPE_MESSAGE_UNICAST));

    uavcan::RxFrame frame(uavcan::Frame(123, uavcan::TRANSFER_TYPE_MESSAGE_BROADCAST, 1, 0, 0, 0, true), 0, 0, 0);
    uavcan::MultiFrameIncomingTransfer mfit(10, 1000, frame, tba);

    // Filling the buffer with data
    static const std::string TEST_DATA = "Kaneda! What do you see? Kaneda! What do you see? Kaneda! Kaneda!!!";
    ASSERT_TRUE(tba.create());
    ASSERT_EQ(TEST_DATA.length(),
              tba.access()->write(0, reinterpret_cast<const uint8_t*>(TEST_DATA.c_str()), TEST_DATA.length()));

    // Reading back
    const Transfer transfer(mfit);
    ASSERT_EQ(TEST_DATA, transfer.payload);
}


TEST(TransferTestHelpers, MFTSerialization)
{
    uavcan::DataTypeDescriptor type(uavcan::DATA_TYPE_KIND_MESSAGE, 123, uavcan::DataTypeHash());
    for (int i = 0; i < uavcan::DataTypeHash::NUM_BYTES; i++)
        type.hash.value[i] = i;

    static const std::string DATA = "To go wrong in one's own way is better than to go right in someone else's.";
    const Transfer transfer(1, 100000, uavcan::TRANSFER_TYPE_MESSAGE_UNICAST, 2, 42, DATA);

    const std::vector<uavcan::RxFrame> ser = serializeTransfer(transfer, 127, type);

    std::cout << "Serialized transfer:\n";
    for (std::vector<uavcan::RxFrame>::const_iterator it = ser.begin(); it != ser.end(); ++it)
        std::cout << "\t" << it->toString() << "\n";

    for (std::vector<uavcan::RxFrame>::const_iterator it = ser.begin(); it != ser.end(); ++it)
    {
        std::cout << "\t'";
        for (int i = 0; i < it->getPayloadLen(); i++)
        {
            uint8_t ch = it->getPayloadPtr()[i];
            if (ch < 0x20 || ch > 0x7E)
                ch = '.';
            std::cout << static_cast<char>(ch);
        }
        std::cout << "'\n";
    }
    std::cout << std::flush;
}


TEST(TransferTestHelpers, SFTSerialization)
{
    uavcan::DataTypeDescriptor type(uavcan::DATA_TYPE_KIND_MESSAGE, 123, uavcan::DataTypeHash());
    for (int i = 0; i < uavcan::DataTypeHash::NUM_BYTES; i++)
        type.hash.value[i] = i;

    {
        const Transfer transfer(1, 100000, uavcan::TRANSFER_TYPE_MESSAGE_BROADCAST, 7, 42, "Nvrfrget");
        const std::vector<uavcan::RxFrame> ser = serializeTransfer(transfer, 0, type);
        ASSERT_EQ(1, ser.size());
        std::cout << "Serialized transfer:\n\t" << ser[0].toString() << "\n";
    }
    {
        const Transfer transfer(1, 100000, uavcan::TRANSFER_TYPE_SERVICE_REQUEST, 7, 42, "7-chars");
        const std::vector<uavcan::RxFrame> ser = serializeTransfer(transfer, 127, type);
        ASSERT_EQ(1, ser.size());
        std::cout << "Serialized transfer:\n\t" << ser[0].toString() << "\n";
    }
    {
        const Transfer transfer(1, 100000, uavcan::TRANSFER_TYPE_MESSAGE_BROADCAST, 7, 42, "");
        const std::vector<uavcan::RxFrame> ser = serializeTransfer(transfer, 0, type);
        ASSERT_EQ(1, ser.size());
        std::cout << "Serialized transfer:\n\t" << ser[0].toString() << "\n";
    }
    {
        const Transfer transfer(1, 100000, uavcan::TRANSFER_TYPE_SERVICE_RESPONSE, 7, 42, "");
        const std::vector<uavcan::RxFrame> ser = serializeTransfer(transfer, 127, type);
        ASSERT_EQ(1, ser.size());
        std::cout << "Serialized transfer:\n\t" << ser[0].toString() << "\n";
    }
}
