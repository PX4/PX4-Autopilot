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

    virtual void registerInternalFailure(const char* msg)
    {
        std::cout << "TestNode internal failure: " << msg << std::endl;
    }

    virtual uavcan::PoolManager<1>& getAllocator() { return poolmgr; }
    virtual uavcan::Scheduler& getScheduler() { return scheduler; }
    virtual const uavcan::Scheduler& getScheduler() const { return scheduler; }
    virtual uavcan::IMarshalBufferProvider& getMarshalBufferProvider() { return buffer_provider; }
};


struct PairableCanDriver : public uavcan::ICanDriver, public uavcan::ICanIface
{
    uavcan::ISystemClock& clock;
    PairableCanDriver* other;
    std::queue<uavcan::CanFrame> read_queue;
    std::queue<uavcan::CanFrame> loopback_queue;
    uint64_t error_count;

    PairableCanDriver(uavcan::ISystemClock& clock)
        : clock(clock)
        , other(NULL)
        , error_count(0)
    { }

    void linkTogether(PairableCanDriver* with)
    {
        this->other = with;
        with->other = this;
    }

    virtual uavcan::ICanIface* getIface(uavcan::uint8_t iface_index)
    {
        if (iface_index == 0)
        {
            return this;
        }
        return NULL;
    }

    virtual uavcan::uint8_t getNumIfaces() const { return 1; }

    virtual uavcan::int16_t select(uavcan::CanSelectMasks& inout_masks, uavcan::MonotonicTime blocking_deadline)
    {
        assert(other);
        if (inout_masks.read == 1)
        {
            inout_masks.read = (!read_queue.empty() || !loopback_queue.empty()) ? 1 : 0;
        }
        if (inout_masks.read || inout_masks.write)
        {
            return 1;
        }
        while (clock.getMonotonic() < blocking_deadline)
        {
            usleep(1000);
        }
        return 0;
    }

    virtual uavcan::int16_t send(const uavcan::CanFrame& frame, uavcan::MonotonicTime, uavcan::CanIOFlags flags)
    {
        assert(other);
        other->read_queue.push(frame);
        if (flags & uavcan::CanIOFlagLoopback)
        {
            loopback_queue.push(frame);
        }
        return 1;
    }

    virtual uavcan::int16_t receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic,
                                    uavcan::UtcTime& out_ts_utc, uavcan::CanIOFlags& out_flags)
    {
        assert(other);
        out_flags = 0;
        if (loopback_queue.empty())
        {
            assert(read_queue.size());
            out_frame = read_queue.front();
            read_queue.pop();
        }
        else
        {
            out_flags |= uavcan::CanIOFlagLoopback;
            out_frame = loopback_queue.front();
            loopback_queue.pop();
        }
        out_ts_monotonic = clock.getMonotonic();
        out_ts_utc = clock.getUtc();
        return 1;
    }

    virtual uavcan::int16_t configureFilters(const uavcan::CanFilterConfig*, uavcan::uint16_t) { return -1; }
    virtual uavcan::uint16_t getNumFilters() const { return 0; }
    virtual uavcan::uint64_t getErrorCount() const { return error_count; }
};


template <typename ClockType>
struct InterlinkedTestNodes
{
    ClockType clock_a;
    ClockType clock_b;
    PairableCanDriver can_a;
    PairableCanDriver can_b;
    TestNode a;
    TestNode b;

    InterlinkedTestNodes(uavcan::NodeID nid_first, uavcan::NodeID nid_second)
        : can_a(clock_a)
        , can_b(clock_b)
        , a(can_a, clock_a, nid_first)
        , b(can_b, clock_b, nid_second)
    {
        can_a.linkTogether(&can_b);
    }

    InterlinkedTestNodes()
        : can_a(clock_a)
        , can_b(clock_b)
        , a(can_a, clock_a, 1)
        , b(can_b, clock_b, 2)
    {
        can_a.linkTogether(&can_b);
    }

    int spinBoth(uavcan::MonotonicDuration duration)
    {
        assert(!duration.isNegative());
        unsigned nspins2 = unsigned(duration.toMSec() / 2);
        nspins2 = nspins2 ? nspins2 : 1;
        while (nspins2 --> 0)
        {
            int ret = a.spin(uavcan::MonotonicDuration::fromMSec(1));
            if (ret < 0)
            {
                return ret;
            }
            ret = b.spin(uavcan::MonotonicDuration::fromMSec(1));
            if (ret < 0)
            {
                return ret;
            }
        }
        return 0;
    }
};


typedef InterlinkedTestNodes<SystemClockDriver> InterlinkedTestNodesWithSysClock;
typedef InterlinkedTestNodes<SystemClockMock> InterlinkedTestNodesWithClockMock;
