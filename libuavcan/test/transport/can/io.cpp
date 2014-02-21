/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include "iface_mock.hpp"


static bool rxFrameEquals(const uavcan::CanRxFrame& rxframe, const uavcan::CanFrame& frame,
                          uint64_t timestamp, int iface_index)
{
    if (static_cast<const uavcan::CanFrame&>(rxframe) != frame)
    {
        std::cout << "Frame mismatch:\n"
                  << "    " << rxframe.toString(uavcan::CanFrame::StrAligned) << "\n"
                  << "    " << frame.toString(uavcan::CanFrame::StrAligned) << std::endl;
    }
    return (static_cast<const uavcan::CanFrame&>(rxframe) == frame) &&
        (rxframe.ts_monotonic == timestamp) && (rxframe.iface_index == iface_index);
}

TEST(CanIOManager, Reception)
{
    // Memory
    uavcan::PoolAllocator<sizeof(uavcan::CanTxQueue::Entry) * 4, sizeof(uavcan::CanTxQueue::Entry)> pool;
    uavcan::PoolManager<2> poolmgr;
    poolmgr.addPool(&pool);

    // Platform interface
    SystemClockMock clockmock;
    CanDriverMock driver(2, clockmock);

    // IO Manager
    uavcan::CanIOManager iomgr(driver, poolmgr, clockmock);
    ASSERT_EQ(2, iomgr.getNumIfaces());

    /*
     * Empty, will time out
     */
    uavcan::CanRxFrame frame;
    EXPECT_EQ(0, iomgr.receive(frame, 100));
    EXPECT_EQ(100, clockmock.monotonic);
    EXPECT_EQ(100, clockmock.utc);

    /*
     * Non empty from multiple ifaces
     */
    const uavcan::CanFrame frames[2][3] = {
        { makeCanFrame(1, "a0", EXT),    makeCanFrame(99, "a1", EXT),  makeCanFrame(803, "a2", STD) },
        { makeCanFrame(6341, "b0", EXT), makeCanFrame(196, "b1", STD), makeCanFrame(73, "b2", EXT) },
    };

    clockmock.advance(10);
    driver.ifaces.at(0).pushRx(frames[0][0]);  // Timestamp 110
    driver.ifaces.at(1).pushRx(frames[1][0]);
    clockmock.advance(10);
    driver.ifaces.at(0).pushRx(frames[0][1]);  // Timestamp 120
    driver.ifaces.at(1).pushRx(frames[1][1]);
    clockmock.advance(10);
    driver.ifaces.at(0).pushRx(frames[0][2]);  // Timestamp 130
    driver.ifaces.at(1).pushRx(frames[1][2]);
    clockmock.advance(10);

    EXPECT_EQ(1, iomgr.receive(frame, 0));
    EXPECT_TRUE(rxFrameEquals(frame, frames[0][0], 110, 0));

    EXPECT_EQ(1, iomgr.receive(frame, 0));
    EXPECT_TRUE(rxFrameEquals(frame, frames[0][1], 120, 0));

    EXPECT_EQ(1, iomgr.receive(frame, 0));
    EXPECT_TRUE(rxFrameEquals(frame, frames[0][2], 130, 0));

    EXPECT_EQ(1, iomgr.receive(frame, 0));
    EXPECT_TRUE(rxFrameEquals(frame, frames[1][0], 110, 1));

    EXPECT_EQ(1, iomgr.receive(frame, 0));
    EXPECT_TRUE(rxFrameEquals(frame, frames[1][1], 120, 1));

    EXPECT_EQ(1, iomgr.receive(frame, 0));
    EXPECT_TRUE(rxFrameEquals(frame, frames[1][2], 130, 1));

    EXPECT_EQ(0, iomgr.receive(frame, 0));  // Will time out

    /*
     * Errors
     */
    driver.select_failure = true;
    EXPECT_EQ(-1, iomgr.receive(frame, 0));

    driver.select_failure = false;
    driver.ifaces.at(1).pushRx(frames[0][0]);
    driver.ifaces.at(1).rx_failure = true;
    EXPECT_EQ(-1, iomgr.receive(frame, 0));

    driver.ifaces.at(0).num_errors = 9000;
    driver.ifaces.at(1).num_errors = 100500;
    EXPECT_EQ(9000, iomgr.getNumErrors(0));
    EXPECT_EQ(100500, iomgr.getNumErrors(1));
}

TEST(CanIOManager, Transmission)
{
    using uavcan::CanIOManager;
    using uavcan::CanTxQueue;

    // Memory
    typedef uavcan::PoolAllocator<sizeof(CanTxQueue::Entry) * 4, sizeof(CanTxQueue::Entry)> Pool1;
    Pool1* ppool = new Pool1();
    Pool1& pool = *ppool;
    uavcan::PoolManager<2> poolmgr;
    poolmgr.addPool(&pool);

    // Platform interface
    SystemClockMock clockmock;
    CanDriverMock driver(2, clockmock);

    // IO Manager
    CanIOManager iomgr(driver, poolmgr, clockmock);
    ASSERT_EQ(2, iomgr.getNumIfaces());

    const int ALL_IFACES_MASK = 3;

    const uavcan::CanFrame frames[] = {
        makeCanFrame(1, "a0", EXT),    makeCanFrame(99, "a1", EXT),  makeCanFrame(803, "a2", STD)
    };

    /*
     * Simple transmission
     */
    EXPECT_EQ(2, iomgr.send(frames[0], 100, 0, ALL_IFACES_MASK, CanTxQueue::Volatile));  // To both
    EXPECT_TRUE(driver.ifaces.at(0).matchAndPopTx(frames[0], 100));
    EXPECT_TRUE(driver.ifaces.at(1).matchAndPopTx(frames[0], 100));

    EXPECT_EQ(1, iomgr.send(frames[1], 200, 100, 2, CanTxQueue::Persistent));            // To #1
    EXPECT_TRUE(driver.ifaces.at(1).matchAndPopTx(frames[1], 200));

    EXPECT_EQ(0, clockmock.monotonic);
    EXPECT_EQ(0, clockmock.utc);
    EXPECT_TRUE(driver.ifaces.at(0).tx.empty());
    EXPECT_TRUE(driver.ifaces.at(1).tx.empty());
    EXPECT_EQ(0, iomgr.getNumErrors(0));
    EXPECT_EQ(0, iomgr.getNumErrors(1));

    /*
     * TX Queue basics
     */
    EXPECT_EQ(0, pool.getNumUsedBlocks());

    // Sending to both, #0 blocked
    driver.ifaces.at(0).writeable = false;
    EXPECT_LT(0, iomgr.send(frames[0], 201, 200, ALL_IFACES_MASK, CanTxQueue::Persistent));
    EXPECT_TRUE(driver.ifaces.at(1).matchAndPopTx(frames[0], 201));
    EXPECT_EQ(200, clockmock.monotonic);
    EXPECT_EQ(200, clockmock.utc);
    EXPECT_TRUE(driver.ifaces.at(0).tx.empty());
    EXPECT_TRUE(driver.ifaces.at(1).tx.empty());
    EXPECT_EQ(1, pool.getNumUsedBlocks());          // One frame went into TX queue, and will expire soon

    // Sending to both, both blocked
    driver.ifaces.at(1).writeable = false;
    EXPECT_EQ(0, iomgr.send(frames[1], 777, 300, ALL_IFACES_MASK, CanTxQueue::Volatile));
    EXPECT_EQ(3, pool.getNumUsedBlocks());          // Total 3 frames in TX queue now

    // Sending to #0, both blocked
    EXPECT_EQ(0, iomgr.send(frames[2], 888, 400, 1, CanTxQueue::Persistent));
    EXPECT_EQ(400, clockmock.monotonic);
    EXPECT_EQ(400, clockmock.utc);
    EXPECT_TRUE(driver.ifaces.at(0).tx.empty());
    EXPECT_TRUE(driver.ifaces.at(1).tx.empty());
    EXPECT_EQ(4, pool.getNumUsedBlocks());

    // At this time TX queues are containing the following data:
    // iface 0: frames[0] (EXPIRED), frames[1], frames[2]
    // iface 1: frames[1]

    // Sending to #1, both writeable
    driver.ifaces.at(0).writeable = true;
    driver.ifaces.at(1).writeable = true;
    EXPECT_LT(0, iomgr.send(frames[0], 999, 500, 2, CanTxQueue::Persistent)); // One frame per each iface will be sent
    EXPECT_TRUE(driver.ifaces.at(0).matchAndPopTx(frames[1], 777));   // Note that frame[0] on iface #0 has expired
    EXPECT_TRUE(driver.ifaces.at(1).matchAndPopTx(frames[0], 999));   // In different order due to prioritization
    EXPECT_TRUE(driver.ifaces.at(0).tx.empty());
    EXPECT_TRUE(driver.ifaces.at(1).tx.empty());

    // Calling receive() to flush the rest two frames
    uavcan::CanRxFrame dummy_rx_frame;
    EXPECT_EQ(0, iomgr.receive(dummy_rx_frame, 0));
    EXPECT_TRUE(driver.ifaces.at(0).matchAndPopTx(frames[2], 888));
    EXPECT_TRUE(driver.ifaces.at(1).matchAndPopTx(frames[1], 777));

    // Final checks
    EXPECT_TRUE(driver.ifaces.at(0).tx.empty());
    EXPECT_TRUE(driver.ifaces.at(1).tx.empty());
    EXPECT_EQ(0, pool.getNumUsedBlocks());          // Make sure the memory was properly released
    EXPECT_EQ(1, iomgr.getNumErrors(0));            // This is because of expired frame[0]
    EXPECT_EQ(0, iomgr.getNumErrors(1));

    /*
     * TX Queue updates from receive() call
     */
    driver.ifaces.at(0).writeable = false;
    driver.ifaces.at(1).writeable = false;

    // Sending 5 frames, one will be rejected
    EXPECT_EQ(0, iomgr.send(frames[2], 2222, 1000, ALL_IFACES_MASK, CanTxQueue::Persistent));
    EXPECT_EQ(0, iomgr.send(frames[0], 3333, 1100, 2, CanTxQueue::Persistent));
    EXPECT_EQ(0, iomgr.send(frames[1], 4444, 1200, ALL_IFACES_MASK, CanTxQueue::Volatile));  // One frame kicked here

    // State checks
    EXPECT_EQ(4, pool.getNumUsedBlocks());          // TX queue is full
    EXPECT_EQ(1200, clockmock.monotonic);
    EXPECT_EQ(1200, clockmock.utc);
    EXPECT_TRUE(driver.ifaces.at(0).tx.empty());
    EXPECT_TRUE(driver.ifaces.at(1).tx.empty());

    // Preparing the driver mock for receive() call
    driver.ifaces.at(0).writeable = true;
    driver.ifaces.at(1).writeable = true;
    const uavcan::CanFrame rx_frames[] = { makeCanFrame(123, "rx0", STD), makeCanFrame(321, "rx1", EXT) };
    driver.ifaces.at(0).pushRx(rx_frames[0]);
    driver.ifaces.at(1).pushRx(rx_frames[1]);

    // This shall transmit _some_ frames now, at least one per iface (exact number can be changed - it will be OK)
    uavcan::CanRxFrame rx_frame;
    EXPECT_EQ(1, iomgr.receive(rx_frame, 0));                         // Non-blocking
    EXPECT_TRUE(rxFrameEquals(rx_frame, rx_frames[0], 1200, 0));
    EXPECT_TRUE(driver.ifaces.at(0).matchAndPopTx(frames[1], 4444));
    EXPECT_TRUE(driver.ifaces.at(1).matchAndPopTx(frames[0], 3333));

    EXPECT_EQ(1, iomgr.receive(rx_frame, 0));
    EXPECT_TRUE(rxFrameEquals(rx_frame, rx_frames[1], 1200, 1));
    EXPECT_TRUE(driver.ifaces.at(0).matchAndPopTx(frames[2], 2222));
    EXPECT_TRUE(driver.ifaces.at(1).matchAndPopTx(frames[2], 2222));  // Iface #1, frame[1] was rejected (VOLATILE)

    // State checks
    EXPECT_EQ(0, pool.getNumUsedBlocks());          // TX queue is empty
    EXPECT_EQ(1200, clockmock.monotonic);
    EXPECT_EQ(1200, clockmock.utc);
    EXPECT_TRUE(driver.ifaces.at(0).tx.empty());
    EXPECT_TRUE(driver.ifaces.at(1).tx.empty());
    EXPECT_EQ(1, iomgr.getNumErrors(0));
    EXPECT_EQ(1, iomgr.getNumErrors(1));            // This is because of rejected frame[1]

    /*
     * Error handling
     */
    // Select failure
    driver.select_failure = true;
    EXPECT_EQ(-1, iomgr.receive(rx_frame, 2000));
    EXPECT_EQ(-1, iomgr.send(frames[0], 2100, 2000, ALL_IFACES_MASK, CanTxQueue::Volatile));
    EXPECT_EQ(1200, clockmock.monotonic);
    EXPECT_EQ(1200, clockmock.utc);

    // Transmission failure
    driver.select_failure = false;
    driver.ifaces.at(0).writeable = true;
    driver.ifaces.at(1).writeable = true;
    driver.ifaces.at(0).tx_failure = true;
    driver.ifaces.at(1).tx_failure = true;
    EXPECT_GE(0, iomgr.send(frames[0], 2200, 0, ALL_IFACES_MASK, CanTxQueue::Persistent)); // Non-blocking - return < 0

    ASSERT_EQ(2, pool.getNumUsedBlocks());               // Untransmitted frames will be buffered

    // Failure removed - transmission shall proceed
    driver.ifaces.at(0).tx_failure = false;
    driver.ifaces.at(1).tx_failure = false;
    EXPECT_EQ(0, iomgr.receive(rx_frame, 2500));
    EXPECT_TRUE(driver.ifaces.at(0).matchAndPopTx(frames[0], 2200));
    EXPECT_TRUE(driver.ifaces.at(1).matchAndPopTx(frames[0], 2200));
    EXPECT_EQ(0, pool.getNumUsedBlocks());               // All transmitted
}
