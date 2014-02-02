/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <queue>
#include <vector>
#include <gtest/gtest.h>
#include <uavcan/internal/transport/can_io.hpp>
#include "../../common.hpp"


class CanIfaceMock : public uavcan::ICanIface
{
public:
    struct FrameWithTime
    {
        uavcan::CanFrame frame;
        uint64_t time;

        FrameWithTime(const uavcan::CanFrame& frame, uint64_t time)
        : frame(frame)
        , time(time)
        { }
    };

    std::queue<FrameWithTime> tx;       ///< Queue of outgoing frames (bus <-- library)
    std::queue<FrameWithTime> rx;       ///< Queue of incoming frames (bus --> library)
    bool writeable;
    bool tx_failure;
    bool rx_failure;
    uint64_t num_errors;
    SystemClockMock& clockmock;

    CanIfaceMock(SystemClockMock& clockmock)
    : writeable(true)
    , tx_failure(false)
    , rx_failure(false)
    , num_errors(0)
    , clockmock(clockmock)
    { }

    void pushRx(uavcan::CanFrame frame)
    {
        rx.push(FrameWithTime(frame, clockmock.utc));
    }

    bool matchAndPopTx(const uavcan::CanFrame& frame, uint64_t tx_deadline)
    {
        if (tx.empty())
        {
            std::cout << "Tx buffer is empty" << std::endl;
            return false;
        }
        const FrameWithTime frame_time = tx.front();
        tx.pop();
        return (frame_time.frame == frame) && (frame_time.time == tx_deadline);
    }

    int send(const uavcan::CanFrame& frame, uint64_t tx_timeout_usec)
    {
        assert(this);
        EXPECT_TRUE(writeable);        // Shall never be called when not writeable
        if (tx_failure)
            return -1;
        if (!writeable)
            return 0;
        const uint64_t monotonic_deadline = tx_timeout_usec + clockmock.monotonic;
        tx.push(FrameWithTime(frame, monotonic_deadline));
        return 1;
    }

    int receive(uavcan::CanFrame& out_frame, uint64_t& out_utc_timestamp_usec)
    {
        assert(this);
        EXPECT_TRUE(rx.size());        // Shall never be called when not readable
        if (rx_failure)
            return -1;
        if (rx.empty())
            return 0;
        const FrameWithTime frame = rx.front();
        rx.pop();
        out_frame = frame.frame;
        out_utc_timestamp_usec = frame.time;
        return 1;
    }

    // cppcheck-suppress unusedFunction
    // cppcheck-suppress functionConst
    int configureFilters(const uavcan::CanFilterConfig* filter_configs, int num_configs) { return -1; }
    // cppcheck-suppress unusedFunction
    int getNumFilters() const { return 0; }
    uint64_t getNumErrors() const { return num_errors; }
};

class CanDriverMock : public uavcan::ICanDriver
{
public:
    std::vector<CanIfaceMock> ifaces;
    SystemClockMock& clockmock;
    bool select_failure;

    CanDriverMock(int num_ifaces, SystemClockMock& clockmock)
    : ifaces(num_ifaces, CanIfaceMock(clockmock))
    , clockmock(clockmock)
    , select_failure(false)
    { }

    int select(int& inout_write_iface_mask, int& inout_read_iface_mask, uint64_t timeout_usec)
    {
        assert(this);
        std::cout << "Write/read masks: " << inout_write_iface_mask << "/" << inout_read_iface_mask << std::endl;

        if (select_failure)
            return -1;

        const int valid_iface_mask = (1 << getNumIfaces()) - 1;
        EXPECT_FALSE(inout_write_iface_mask & ~valid_iface_mask);
        EXPECT_FALSE(inout_read_iface_mask & ~valid_iface_mask);

        int out_write_mask = 0;
        int out_read_mask = 0;
        for (int i = 0; i < getNumIfaces(); i++)
        {
            const int mask = 1 << i;
            if ((inout_write_iface_mask & mask) && ifaces.at(i).writeable)
                out_write_mask |= mask;
            if ((inout_read_iface_mask & mask) && ifaces.at(i).rx.size())
                out_read_mask |= mask;
        }
        inout_write_iface_mask = out_write_mask;
        inout_read_iface_mask = out_read_mask;
        if ((out_write_mask | out_read_mask) == 0)
        {
            clockmock.advance(timeout_usec);   // Emulating timeout
            return 0;
        }
        return 1;  // This value is not being checked anyway, it just has to be greater than zero
    }

    uavcan::ICanIface* getIface(int iface_index) { return &ifaces.at(iface_index); }
    int getNumIfaces() const { return ifaces.size(); }
};

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
    uint64_t timestamp;
    EXPECT_EQ(1, driver.getIface(1)->receive(fr2, timestamp));
    EXPECT_EQ(fr1, fr2);
    EXPECT_EQ(100, timestamp);

    // #0 WR, #1 RD, Select failure
    driver.ifaces.at(0).writeable = true;
    driver.select_failure = true;
    mask_wr = 1;
    mask_rd = 7;
    EXPECT_EQ(-1, driver.select(mask_wr, mask_rd, 100));
    EXPECT_EQ(1, mask_wr);                                 // Leaving masks unchanged - library must ignore them
    EXPECT_EQ(7, mask_rd);
}

static bool rxFrameEquals(const uavcan::CanRxFrame& rxframe, const uavcan::CanFrame& frame,
                          uint64_t timestamp, int iface_index)
{
    if (rxframe.frame != frame)
    {
        std::cout << "Frame mismatch:\n"
                  << "    " << rxframe.frame.toString(uavcan::CanFrame::STR_ALIGNED) << "\n"
                  << "    " << frame.toString(uavcan::CanFrame::STR_ALIGNED) << std::endl;
    }
    return (rxframe.frame == frame) && (rxframe.timestamp == timestamp) && (rxframe.iface_index == iface_index);
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
    uavcan::CanIOManager iomgr(&driver, &poolmgr, &clockmock);
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
    CanIOManager iomgr(&driver, &poolmgr, &clockmock);
    ASSERT_EQ(2, iomgr.getNumIfaces());

    const int ALL_IFACES_MASK = 3;

    const uavcan::CanFrame frames[] = {
        makeCanFrame(1, "a0", EXT),    makeCanFrame(99, "a1", EXT),  makeCanFrame(803, "a2", STD)
    };

    /*
     * Simple transmission
     */
    EXPECT_EQ(2, iomgr.send(frames[0], 100, 0, ALL_IFACES_MASK, CanTxQueue::VOLATILE));  // To both
    EXPECT_TRUE(driver.ifaces.at(0).matchAndPopTx(frames[0], 100));
    EXPECT_TRUE(driver.ifaces.at(1).matchAndPopTx(frames[0], 100));

    EXPECT_EQ(1, iomgr.send(frames[1], 200, 100, 2, CanTxQueue::PERSISTENT));            // To #1
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
    EXPECT_LT(0, iomgr.send(frames[0], 201, 200, ALL_IFACES_MASK, CanTxQueue::PERSISTENT));
    EXPECT_TRUE(driver.ifaces.at(1).matchAndPopTx(frames[0], 201));
    EXPECT_EQ(200, clockmock.monotonic);
    EXPECT_EQ(200, clockmock.utc);
    EXPECT_TRUE(driver.ifaces.at(0).tx.empty());
    EXPECT_TRUE(driver.ifaces.at(1).tx.empty());
    EXPECT_EQ(1, pool.getNumUsedBlocks());          // One frame went into TX queue, and will expire soon

    // Sending to both, both blocked
    driver.ifaces.at(1).writeable = false;
    EXPECT_EQ(0, iomgr.send(frames[1], 777, 300, ALL_IFACES_MASK, CanTxQueue::VOLATILE));
    EXPECT_EQ(3, pool.getNumUsedBlocks());          // Total 3 frames in TX queue now

    // Sending to #0, both blocked
    EXPECT_EQ(0, iomgr.send(frames[2], 888, 400, 1, CanTxQueue::PERSISTENT));
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
    EXPECT_LT(0, iomgr.send(frames[0], 999, 500, 2, CanTxQueue::PERSISTENT));
    EXPECT_TRUE(driver.ifaces.at(0).matchAndPopTx(frames[1], 777));   // Note that frame[0] on iface #0 has expired
    EXPECT_TRUE(driver.ifaces.at(0).matchAndPopTx(frames[2], 888));
    EXPECT_TRUE(driver.ifaces.at(1).matchAndPopTx(frames[0], 999));   // In different order due to prioritization
    EXPECT_TRUE(driver.ifaces.at(1).matchAndPopTx(frames[1], 777));

    // Final checks
    ASSERT_EQ(0, driver.ifaces.at(0).tx.size());
    ASSERT_EQ(0, driver.ifaces.at(1).tx.size());
    EXPECT_EQ(0, pool.getNumUsedBlocks());          // Make sure the memory was properly released
    EXPECT_EQ(1, iomgr.getNumErrors(0));            // This is because of expired frame[0]
    EXPECT_EQ(0, iomgr.getNumErrors(1));

    /*
     * TX Queue updates from receive() call
     */
    driver.ifaces.at(0).writeable = false;
    driver.ifaces.at(1).writeable = false;

    // Sending 5 frames, one will be rejected
    EXPECT_EQ(0, iomgr.send(frames[2], 2222, 1000, ALL_IFACES_MASK, CanTxQueue::PERSISTENT));
    EXPECT_EQ(0, iomgr.send(frames[0], 3333, 1100, 2, CanTxQueue::PERSISTENT));
    EXPECT_EQ(0, iomgr.send(frames[1], 4444, 1200, ALL_IFACES_MASK, CanTxQueue::VOLATILE));  // One frame kicked here

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
    EXPECT_EQ(-1, iomgr.send(frames[0], 2100, 2000, ALL_IFACES_MASK, CanTxQueue::VOLATILE));
    EXPECT_EQ(1200, clockmock.monotonic);
    EXPECT_EQ(1200, clockmock.utc);

    // Transmission failure
    driver.select_failure = false;
    driver.ifaces.at(0).writeable = true;
    driver.ifaces.at(1).writeable = true;
    driver.ifaces.at(0).tx_failure = true;
    driver.ifaces.at(1).tx_failure = true;
    EXPECT_GE(0, iomgr.send(frames[0], 2200, 0, ALL_IFACES_MASK, CanTxQueue::PERSISTENT)); // Non-blocking - return < 0

    ASSERT_EQ(2, pool.getNumUsedBlocks());               // Untransmitted frames will be buffered

    // Failure removed - transmission shall proceed
    driver.ifaces.at(0).tx_failure = false;
    driver.ifaces.at(1).tx_failure = false;
    EXPECT_EQ(0, iomgr.receive(rx_frame, 2500));
    EXPECT_TRUE(driver.ifaces.at(0).matchAndPopTx(frames[0], 2200));
    EXPECT_TRUE(driver.ifaces.at(1).matchAndPopTx(frames[0], 2200));
    EXPECT_EQ(0, pool.getNumUsedBlocks());               // All transmitted
}
