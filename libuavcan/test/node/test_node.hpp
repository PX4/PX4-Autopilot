/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/node/abstract_node.hpp>
#include <uavcan/util/lazy_constructor.hpp>

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

