/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <string>
#include <gtest/gtest.h>
#include <uavcan/transport/transfer.hpp>


TEST(Transfer, TransferID)
{
    using uavcan::TransferID;

    // Tests below are based on this assumption
    ASSERT_EQ(8, 1 << TransferID::BitLen);

    /*
     * forwardDistance()
     */
    EXPECT_EQ(0, TransferID(0).computeForwardDistance(0));
    EXPECT_EQ(1, TransferID(0).computeForwardDistance(1));
    EXPECT_EQ(7, TransferID(0).computeForwardDistance(7));

    EXPECT_EQ(0, TransferID(7).computeForwardDistance(7));
    EXPECT_EQ(7, TransferID(7).computeForwardDistance(6));
    EXPECT_EQ(1, TransferID(7).computeForwardDistance(0));

    EXPECT_EQ(7, TransferID(7).computeForwardDistance(6));
    EXPECT_EQ(5, TransferID(0).computeForwardDistance(5));

    /*
     * Misc
     */
    EXPECT_TRUE(TransferID(2) == TransferID(2));
    EXPECT_FALSE(TransferID(2) != TransferID(2));
    EXPECT_FALSE(TransferID(2) == TransferID(0));
    EXPECT_TRUE(TransferID(2) != TransferID(0));

    TransferID tid;
    for (int i = 0; i < 999; i++)
    {
        ASSERT_EQ(i & ((1 << TransferID::BitLen) - 1), tid.get());
        const TransferID copy = tid;
        tid.increment();
        ASSERT_EQ(1, copy.computeForwardDistance(tid));
        ASSERT_EQ(7, tid.computeForwardDistance(copy));
        ASSERT_EQ(0, tid.computeForwardDistance(tid));
    }
}
