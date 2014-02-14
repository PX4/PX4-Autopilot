/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <algorithm>
#include <gtest/gtest.h>
#include <uavcan/internal/transport/outgoing_transfer_registry.hpp>


TEST(OutgoingTransferRegistry, Basic)
{
    using uavcan::OutgoingTransferRegistryKey;
    uavcan::PoolManager<1> poolmgr;  // Empty
    uavcan::OutgoingTransferRegistry<4> otr(&poolmgr);

    otr.cleanup(1000);

    static const int NUM_KEYS = 5;
    const OutgoingTransferRegistryKey keys[NUM_KEYS] =
    {
        OutgoingTransferRegistryKey(123, uavcan::TRANSFER_TYPE_MESSAGE_UNICAST,   42),
        OutgoingTransferRegistryKey(123, uavcan::TRANSFER_TYPE_MESSAGE_BROADCAST, 0),
        OutgoingTransferRegistryKey(123, uavcan::TRANSFER_TYPE_SERVICE_REQUEST,   2),
        OutgoingTransferRegistryKey(123, uavcan::TRANSFER_TYPE_MESSAGE_UNICAST,   4),
        OutgoingTransferRegistryKey(456, uavcan::TRANSFER_TYPE_SERVICE_REQUEST,   2)
    };

    ASSERT_EQ(0, otr.accessOrCreate(keys[0], 1000000)->get());
    ASSERT_EQ(0, otr.accessOrCreate(keys[1], 1000000)->get());
    ASSERT_EQ(0, otr.accessOrCreate(keys[2], 1000000)->get());
    ASSERT_EQ(0, otr.accessOrCreate(keys[3], 1000000)->get());
    ASSERT_FALSE(otr.accessOrCreate(keys[4], 1000000));        // OOM

    /*
     * Incrementing a little
     */
    otr.accessOrCreate(keys[0], 2000000)->increment();
    otr.accessOrCreate(keys[0], 4000000)->increment();
    otr.accessOrCreate(keys[0], 3000000)->increment();
    ASSERT_EQ(3, otr.accessOrCreate(keys[0], 5000000)->get());

    otr.accessOrCreate(keys[2], 2000000)->increment();
    otr.accessOrCreate(keys[2], 3000000)->increment();
    ASSERT_EQ(2, otr.accessOrCreate(keys[2], 6000000)->get());

    otr.accessOrCreate(keys[3], 9000000)->increment();
    ASSERT_EQ(1, otr.accessOrCreate(keys[3], 4000000)->get());

    ASSERT_EQ(0, otr.accessOrCreate(keys[1], 4000000)->get());

    ASSERT_FALSE(otr.accessOrCreate(keys[4], 1000000));        // Still OOM

    /*
     * Cleaning up
     */
    otr.cleanup(4000001);    // Kills 1, 3
    ASSERT_EQ(0, otr.accessOrCreate(keys[1], 1000000)->get());
    ASSERT_EQ(0, otr.accessOrCreate(keys[3], 1000000)->get());
    otr.accessOrCreate(keys[1], 5000000)->increment();
    otr.accessOrCreate(keys[3], 5000000)->increment();

    ASSERT_EQ(3, otr.accessOrCreate(keys[0], 5000000)->get());
    ASSERT_EQ(2, otr.accessOrCreate(keys[2], 6000000)->get());

    otr.cleanup(5000001);    // Kills 1, 3 (He needs a bath, Jud. He stinks of the ground you buried him in.), 0
    ASSERT_EQ(0, otr.accessOrCreate(keys[0], 1000000)->get());
    ASSERT_EQ(0, otr.accessOrCreate(keys[1], 1000000)->get());
    ASSERT_EQ(0, otr.accessOrCreate(keys[3], 1000000)->get());

    ASSERT_EQ(2, otr.accessOrCreate(keys[2], 1000000)->get());

    otr.cleanup(5000001);    // Frees some memory for 4
    ASSERT_EQ(0, otr.accessOrCreate(keys[0], 1000000)->get());
}
