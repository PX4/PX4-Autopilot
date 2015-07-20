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
    ASSERT_EQ(32, 1 << TransferID::BitLen);

    /*
     * computeForwardDistance()
     */
    EXPECT_EQ(0, TransferID(0).computeForwardDistance(0));
    EXPECT_EQ(1, TransferID(0).computeForwardDistance(1));
    EXPECT_EQ(7, TransferID(0).computeForwardDistance(7));

    EXPECT_EQ(0, TransferID(7).computeForwardDistance(7));
    EXPECT_EQ(31,TransferID(31).computeForwardDistance(30));
    EXPECT_EQ(1, TransferID(31).computeForwardDistance(0));

    EXPECT_EQ(30,TransferID(7).computeForwardDistance(5));
    EXPECT_EQ(5, TransferID(0).computeForwardDistance(5));

    /*
     * subtracted()
     */
    EXPECT_EQ(0,  TransferID(0).subtracted(0));
    EXPECT_EQ(-1, TransferID(0).subtracted(1));
    EXPECT_EQ(-7, TransferID(0).subtracted(7));

    EXPECT_EQ(0,  TransferID(7).subtracted(7));
    EXPECT_EQ(1,  TransferID(31).subtracted(30));
    EXPECT_EQ(-1, TransferID(31).subtracted(0));

    EXPECT_EQ(2,  TransferID(7).subtracted(5));
    EXPECT_EQ(-5, TransferID(0).subtracted(5));

    EXPECT_EQ(-1, TransferID(15).subtracted(16));
    EXPECT_EQ(1,  TransferID(16).subtracted(15));

    EXPECT_EQ(-16, TransferID(10).subtracted(26));
    EXPECT_EQ(14,  TransferID(24).subtracted(10));
    EXPECT_EQ(15,  TransferID(25).subtracted(10));
    EXPECT_EQ(-16, TransferID(26).subtracted(10));
    EXPECT_EQ(-15, TransferID(27).subtracted(10));
    EXPECT_EQ(-14, TransferID(28).subtracted(10));

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
        ASSERT_EQ(31, tid.computeForwardDistance(copy));
        ASSERT_EQ(0, tid.computeForwardDistance(tid));
    }
}


TEST(Transfer, NodeID)
{
    uavcan::NodeID nid1(1);
    uavcan::NodeID nid127(127);
    uavcan::NodeID nid0(0);
    uavcan::NodeID nidx;

    ASSERT_TRUE(nid1.isUnicast());
    ASSERT_FALSE(nid1.isBroadcast());
    ASSERT_TRUE(nid1.isValid());

    ASSERT_TRUE(nid127.isUnicast());
    ASSERT_FALSE(nid127.isBroadcast());
    ASSERT_TRUE(nid127.isValid());

    ASSERT_FALSE(nid0.isUnicast());
    ASSERT_TRUE(nid0.isBroadcast());
    ASSERT_TRUE(nid0.isValid());

    ASSERT_FALSE(nidx.isUnicast());
    ASSERT_FALSE(nidx.isBroadcast());
    ASSERT_FALSE(nidx.isValid());

    /*
     * Comparison operators
     */
    ASSERT_TRUE(nid1 < nid127);
    ASSERT_TRUE(nid1 <= nid127);
    ASSERT_TRUE(nid0 < nid1);
    ASSERT_TRUE(nid0 <= nid1);

    ASSERT_FALSE(nid1 > nid127);
    ASSERT_FALSE(nid1 >= nid127);
    ASSERT_FALSE(nid0 > nid1);
    ASSERT_FALSE(nid0 >= nid1);

    ASSERT_FALSE(nid1 > uavcan::NodeID(1));
    ASSERT_TRUE(nid1 >= uavcan::NodeID(1));

    ASSERT_FALSE(nid1 == nid127);
    ASSERT_TRUE(nid127 == uavcan::NodeID(127));
}
