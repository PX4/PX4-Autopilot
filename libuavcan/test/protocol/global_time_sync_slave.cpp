/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/node/publisher.hpp>
#include <uavcan/protocol/global_time_sync_slave.hpp>
#include "helpers.hpp"


TEST(GlobalTimeSyncSlave, Basic)
{
    InterlinkedTestNodesWithClockMock nodes(64, 65);

    SystemClockMock& slave_clock = nodes.clock_a;
    SystemClockMock& master_clock = nodes.clock_b;

    slave_clock.advance(1000000);
    master_clock.advance(1000000);

    master_clock.monotonic_auto_advance = slave_clock.monotonic_auto_advance = 1000;
    master_clock.preserve_utc = slave_clock.preserve_utc = true;
    slave_clock.utc = 0; // Not set yet

    uavcan::GlobalDataTypeRegistry::instance().reset();
    uavcan::DefaultDataTypeRegistrator<uavcan::protocol::GlobalTimeSync> _reg1;

    uavcan::GlobalTimeSyncSlave gtss(nodes.a);
    uavcan::Publisher<uavcan::protocol::GlobalTimeSync> gts_pub(nodes.b);

    ASSERT_LE(0, gtss.start());

    /*
     * Empty broadcast
     * The slave must only register the timestamp and adjust nothing
     */
    uavcan::protocol::GlobalTimeSync gts;
    gts.prev_utc_usec = 0;
    gts_pub.broadcast(gts);
    gts.prev_utc_usec = master_clock.utc;
    nodes.spinBoth(uavcan::MonotonicDuration::fromMSec(10));
    ASSERT_EQ(0, slave_clock.utc);
    ASSERT_EQ(1000000, master_clock.utc);
    std::cout << "Master mono=" << master_clock.monotonic << " utc=" << master_clock.utc << std::endl;
    std::cout << "Slave  mono=" << slave_clock.monotonic  << " utc=" << slave_clock.utc << std::endl;

    /*
     * Follow-up broadcast with proper time
     * Slave must adjust now
     */
    gts_pub.broadcast(gts);
    gts.prev_utc_usec = master_clock.utc;
    nodes.spinBoth(uavcan::MonotonicDuration());
    ASSERT_EQ(1000000, slave_clock.utc);
    ASSERT_EQ(1000000, master_clock.utc);
    std::cout << "Master mono=" << master_clock.monotonic << " utc=" << master_clock.utc << std::endl;
    std::cout << "Slave  mono=" << slave_clock.monotonic  << " utc=" << slave_clock.utc << std::endl;

    master_clock.utc += 1000000;
    slave_clock.utc += 1000000;

    /*
     * Next follow-up, slave is synchronized now
     * Will update
     */
    gts_pub.broadcast(gts);
    gts.prev_utc_usec = master_clock.utc;
    nodes.spinBoth(uavcan::MonotonicDuration());
    ASSERT_EQ(2000000, slave_clock.utc);
    ASSERT_EQ(2000000, master_clock.utc);

    master_clock.utc += 1000000;
    slave_clock.utc += 1000000;

    /*
     * Next follow-up, slave is synchronized now
     * Will adjust
     */
    gts_pub.broadcast(gts);
    gts.prev_utc_usec = master_clock.utc;
    nodes.spinBoth(uavcan::MonotonicDuration());
    ASSERT_EQ(3000000, slave_clock.utc);
    ASSERT_EQ(3000000, master_clock.utc);

    master_clock.utc += 1000000;
    slave_clock.utc += 1000000;
    ASSERT_EQ(4000000, slave_clock.utc);
    ASSERT_EQ(4000000, master_clock.utc);

    /*
     * Another master
     * This one has higher priority, so it will be preferred
     */
    SystemClockMock master2_clock(100);
    master2_clock.monotonic_auto_advance = 1000;
    master2_clock.preserve_utc = true;
    PairableCanDriver master2_can(master2_clock);
    master2_can.other = &nodes.can_a;
    TestNode master2_node(master2_can, master2_clock, 8);

    uavcan::Publisher<uavcan::protocol::GlobalTimeSync> gts_pub2(master2_node);

    /*
     * Update step, no adjustment yet
     */
    gts.prev_utc_usec = 0;
    gts_pub2.broadcast(gts);
    gts.prev_utc_usec = master2_clock.utc;
    nodes.spinBoth(uavcan::MonotonicDuration());
    ASSERT_EQ(4000000, slave_clock.utc);
    ASSERT_EQ(100, master2_clock.utc);

    master2_clock.utc += 1000000;

    /*
     * Adjustment
     */
    gts_pub2.broadcast(gts);
    nodes.spinBoth(uavcan::MonotonicDuration());
    ASSERT_EQ(100, slave_clock.utc);

    /*
     * Another master will be ignored now
     */
    gts.prev_utc_usec = 99999999;
    // Update
    gts_pub.broadcast(gts);
    nodes.spinBoth(uavcan::MonotonicDuration());
    ASSERT_EQ(100, slave_clock.utc);
    // Adjust
    gts_pub.broadcast(gts);
    nodes.spinBoth(uavcan::MonotonicDuration());
    ASSERT_EQ(100, slave_clock.utc);
}
