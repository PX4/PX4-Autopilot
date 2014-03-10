/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include "can.hpp"

TEST(CanIOManager, CanDriverMock)
{
    using uavcan::CanFrame;

    SystemClockMock clockmock;
    CanDriverMock driver(3, clockmock);

    ASSERT_EQ(3, driver.getNumIfaces());

    // All WR, no RD
    int mask_wr = 7;
    int mask_rd = 7;
    EXPECT_LT(0, driver.select(mask_wr, mask_rd, 100));
    EXPECT_EQ(7, mask_wr);
    EXPECT_EQ(0, mask_rd);

    for (int i = 0; i < 3; i++)
        driver.ifaces.at(i).writeable = false;

    // No WR, no RD
    mask_wr = 7;
    mask_rd = 7;
    EXPECT_EQ(0, driver.select(mask_wr, mask_rd, 100));
    EXPECT_EQ(0, mask_wr);
    EXPECT_EQ(0, mask_rd);
    EXPECT_EQ(100, clockmock.monotonic);
    EXPECT_EQ(100, clockmock.utc);

    // No WR, #1 RD
    const CanFrame fr1 = makeCanFrame(123, "foo", EXT);
    driver.ifaces.at(1).pushRx(fr1);
    mask_wr = 7;
    mask_rd = 6;
    EXPECT_LT(0, driver.select(mask_wr, mask_rd, 100));
    EXPECT_EQ(0, mask_wr);
    EXPECT_EQ(2, mask_rd);
    CanFrame fr2;
    uint64_t ts_monotonic, ts_utc;
    EXPECT_EQ(1, driver.getIface(1)->receive(fr2, ts_monotonic, ts_utc));
    EXPECT_EQ(fr1, fr2);
    EXPECT_EQ(100, ts_monotonic);
    EXPECT_EQ(0, ts_utc);

    // #0 WR, #1 RD, Select failure
    driver.ifaces.at(0).writeable = true;
    driver.select_failure = true;
    mask_wr = 1;
    mask_rd = 7;
    EXPECT_EQ(-1, driver.select(mask_wr, mask_rd, 100));
    EXPECT_EQ(1, mask_wr);                                 // Leaving masks unchanged - library must ignore them
    EXPECT_EQ(7, mask_rd);
}
