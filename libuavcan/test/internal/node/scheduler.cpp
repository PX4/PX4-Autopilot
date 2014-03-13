/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/timer.hpp>
#include <uavcan/util/method_binder.hpp>
#include "../../clock.hpp"
#include "../transport/can/can.hpp"

struct TimerCallCounter
{
    std::vector<uavcan::TimerEvent> events_a;
    std::vector<uavcan::TimerEvent> events_b;

    void callA(const uavcan::TimerEvent& ev) { events_a.push_back(ev); }
    void callB(const uavcan::TimerEvent& ev) { events_b.push_back(ev); }

    typedef uavcan::MethodBinder<TimerCallCounter*, void(TimerCallCounter::*)(const uavcan::TimerEvent&)> Binder;
};

/*
 * This test can fail on a non real time system. That's kinda sad but okay.
 */
TEST(Scheduler, Timers)
{
    uavcan::PoolAllocator<uavcan::MemPoolBlockSize * 8, uavcan::MemPoolBlockSize> pool;
    uavcan::PoolManager<1> poolmgr;
    poolmgr.addPool(&pool);

    SystemClockDriver clock_driver;
    CanDriverMock can_driver(2, clock_driver);

    uavcan::OutgoingTransferRegistry<8> out_trans_reg(poolmgr);

    uavcan::Scheduler sch(can_driver, poolmgr, clock_driver, out_trans_reg, uavcan::NodeID(1));

    /*
     * Registration
     */
    {
        TimerCallCounter tcc;
        uavcan::TimerEventForwarder<TimerCallCounter::Binder>
            a(sch, TimerCallCounter::Binder(&tcc, &TimerCallCounter::callA));
        uavcan::TimerEventForwarder<TimerCallCounter::Binder>
            b(sch, TimerCallCounter::Binder(&tcc, &TimerCallCounter::callB));

        ASSERT_EQ(0, sch.getDeadlineScheduler().getNumHandlers());

        const uavcan::MonotonicTime start_ts = clock_driver.getMonotonic();

        a.startOneShotWithDeadline(start_ts + durMono(100000));
        b.startPeriodic(durMono(1000));

        ASSERT_EQ(2, sch.getDeadlineScheduler().getNumHandlers());

        /*
         * Spinning
         */
        ASSERT_EQ(0, sch.spin(start_ts + durMono(1000000)));

        ASSERT_EQ(1, tcc.events_a.size());
        ASSERT_TRUE(areTimestampsClose(tcc.events_a[0].scheduled_time, start_ts + durMono(100000)));
        ASSERT_TRUE(areTimestampsClose(tcc.events_a[0].scheduled_time, tcc.events_a[0].real_time));

        ASSERT_LT(900, tcc.events_b.size());
        ASSERT_GT(1100, tcc.events_b.size());
        {
            uavcan::MonotonicTime next_expected_deadline = start_ts + durMono(1000);
            for (unsigned int i = 0; i < tcc.events_b.size(); i++)
            {
                ASSERT_TRUE(areTimestampsClose(tcc.events_b[i].scheduled_time, next_expected_deadline));
                ASSERT_TRUE(areTimestampsClose(tcc.events_b[i].scheduled_time, tcc.events_b[i].real_time));
                next_expected_deadline += durMono(1000);
            }
        }

        /*
         * Deinitialization
         */
        ASSERT_EQ(1, sch.getDeadlineScheduler().getNumHandlers());

        ASSERT_FALSE(a.isRunning());
        ASSERT_EQ(uavcan::MonotonicDuration::getInfinite(), a.getPeriod());

        ASSERT_TRUE(b.isRunning());
        ASSERT_EQ(1000, b.getPeriod().toUSec());
    }

    ASSERT_EQ(0, sch.getDeadlineScheduler().getNumHandlers());   // Both timers were destroyed now
    ASSERT_EQ(0, sch.spin(clock_driver.getMonotonic() + durMono(1000)));  // Spin some more without timers
}
