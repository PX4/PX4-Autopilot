/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/impl_constants.hpp>
#include <uavcan/node/scheduler.hpp>
#include <uavcan/node/marshal_buffer.hpp>

namespace uavcan
{

class UAVCAN_EXPORT INode
{
public:
    virtual ~INode() { }
    virtual IPoolAllocator& getAllocator() = 0;
    virtual Scheduler& getScheduler() = 0;
    virtual const Scheduler& getScheduler() const = 0;
    virtual IMarshalBufferProvider& getMarshalBufferProvider() = 0;
    virtual void registerInternalFailure(const char* msg) = 0;

    Dispatcher& getDispatcher()             { return getScheduler().getDispatcher(); }
    const Dispatcher& getDispatcher() const { return getScheduler().getDispatcher(); }

    ISystemClock& getSystemClock()         { return getScheduler().getSystemClock(); }
    MonotonicTime getMonotonicTime() const { return getScheduler().getMonotonicTime(); }
    UtcTime getUtcTime()             const { return getScheduler().getUtcTime(); }

    NodeID getNodeID() const { return getScheduler().getDispatcher().getNodeID(); }
    bool setNodeID(NodeID nid)
    {
        return getScheduler().getDispatcher().setNodeID(nid);
    }

    int spin(MonotonicTime deadline)
    {
        return getScheduler().spin(deadline);
    }

    int spin(MonotonicDuration duration)
    {
        return getScheduler().spin(getMonotonicTime() + duration);
    }
};

}
