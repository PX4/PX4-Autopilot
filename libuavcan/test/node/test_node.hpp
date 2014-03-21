/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/node/abstract_node.hpp>
#include <memory>
#include "../transport/can/can.hpp"


struct TestNode : public uavcan::INode
{
    uavcan::PoolAllocator<uavcan::MemPoolBlockSize * 8, uavcan::MemPoolBlockSize> pool;
    uavcan::PoolManager<1> poolmgr;
    uavcan::MarshalBufferProvider<> buffer_provider;
    uavcan::OutgoingTransferRegistry<8> otr;
    uavcan::Scheduler scheduler;

    TestNode(uavcan::ICanDriver& can_driver, uavcan::ISystemClock& clock_driver, uavcan::NodeID self_node_id)
    : otr(poolmgr)
    , scheduler(can_driver, poolmgr, clock_driver, otr)
    {
        poolmgr.addPool(&pool);
        setNodeID(self_node_id);
    }

    void registerInternalFailure(const char* msg)
    {
        std::cout << "TestNode internal failure: " << msg << std::endl;
    }

    uavcan::PoolManager<1>& getAllocator() { return poolmgr; }
    uavcan::Scheduler& getScheduler() { return scheduler; }
    const uavcan::Scheduler& getScheduler() const { return scheduler; }
    uavcan::IMarshalBufferProvider& getMarshalBufferProvider() { return buffer_provider; }
};


struct PairableCanDriver : public uavcan::ICanDriver, public uavcan::ICanIface
{
    uavcan::ISystemClock& clock;
    PairableCanDriver* other;
    std::queue<uavcan::CanFrame> read_queue;

    PairableCanDriver(uavcan::ISystemClock& clock)
    : clock(clock)
    , other(NULL)
    { }

    void linkTogether(PairableCanDriver* with)
    {
        this->other = with;
        with->other = this;
    }

    uavcan::ICanIface* getIface(int iface_index)
    {
        if (iface_index == 0)
            return this;
        return NULL;
    }

    int getNumIfaces() const { return 1; }

    int select(uavcan::CanSelectMasks& inout_masks, uavcan::MonotonicTime blocking_deadline)
    {
        assert(other);
        if (inout_masks.read == 1)
            inout_masks.read = read_queue.size() ? 1 : 0;

        if (inout_masks.read || inout_masks.write)
            return 1;

        while (clock.getMonotonic() < blocking_deadline)
            usleep(1000);

        return 0;
    }

    int send(const uavcan::CanFrame& frame, uavcan::MonotonicTime)
    {
        assert(other);
        other->read_queue.push(frame);
        return 1;
    }

    int receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic, uavcan::UtcTime& out_ts_utc)
    {
        assert(other);
        assert(read_queue.size());
        out_frame = read_queue.front();
        read_queue.pop();
        out_ts_monotonic = clock.getMonotonic();
        out_ts_utc = clock.getUtc();
        return 1;
    }

    int configureFilters(const uavcan::CanFilterConfig*, int) { return -1; }
    int getNumFilters() const { return 0; }
    uint64_t getNumErrors() const { return 0; }
};


struct InterlinkedTestNodes
{
    SystemClockDriver clock;
    PairableCanDriver can_a;
    PairableCanDriver can_b;
    TestNode a;
    TestNode b;

    InterlinkedTestNodes(uavcan::NodeID nid_first, uavcan::NodeID nid_second)
    : can_a(clock)
    , can_b(clock)
    , a(can_a, clock, nid_first)
    , b(can_b, clock, nid_second)
    {
        can_a.linkTogether(&can_b);
    }

    InterlinkedTestNodes()
    : can_a(clock)
    , can_b(clock)
    , a(can_a, clock, 1)
    , b(can_b, clock, 2)
    {
        can_a.linkTogether(&can_b);
    }

    int spinBoth(uavcan::MonotonicDuration duration)
    {
        assert(duration.isPositive());
        unsigned int nspins2 = duration.toMSec() / 2 + 1;
        assert(nspins2 > 0);
        while (nspins2 --> 0)
        {
            int ret = -1;
            ret = a.spin(uavcan::MonotonicDuration::fromMSec(1));
            if (ret < 0)
                return ret;
            ret = b.spin(uavcan::MonotonicDuration::fromMSec(1));
            if (ret < 0)
                return ret;
        }
        return 0;
    }
};
