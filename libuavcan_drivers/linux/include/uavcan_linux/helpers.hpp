/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <uavcan/uavcan.hpp>

namespace uavcan_linux
{
/**
 * Contains all drivers needed for uavcan::Node.
 */
struct DriverPack
{
    SocketCanDriver can;
    SystemClock clock;

    DriverPack(ClockAdjustmentMode clock_adjustment_mode)
        : can()
        , clock(clock_adjustment_mode)
    { }
};

typedef std::shared_ptr<DriverPack> DriverPackPtr;

typedef std::shared_ptr<uavcan::Timer> TimerPtr;

static constexpr std::size_t NodeMemPoolSize = 1024 * 512;  // One size fits all

/**
 * Wrapper for uavcan::Node with some additional convenience functions.
 */
class Node : public uavcan::Node<NodeMemPoolSize>
{
    DriverPackPtr driver_pack_;

    static void enforce(int error, const char* msg)
    {
        if (error < 0)
        {
            throw Exception(msg);
        }
    }

public:
    /**
     * Simple forwarding constructor, compatible with uavcan::Node
     */
    Node(uavcan::ICanDriver& can_driver, uavcan::ISystemClock& clock)
        : uavcan::Node<NodeMemPoolSize>(can_driver, clock)
    { }

    /**
     * Takes ownership of the driver container.
     */
    Node(DriverPackPtr driver_pack)
        : uavcan::Node<NodeMemPoolSize>(driver_pack->can, driver_pack->clock)
        , driver_pack_(driver_pack)
    { }

    template <typename DataType>
    std::shared_ptr<uavcan::Subscriber<DataType>>
    makeSubscriber(const typename uavcan::Subscriber<DataType>::Callback& cb)
    {
        std::shared_ptr<uavcan::Subscriber<DataType>> p(new uavcan::Subscriber<DataType>(*this));
        enforce(p->start(cb), "Subscriber start");
        return p;
    }

    template <typename DataType>
    std::shared_ptr<uavcan::Publisher<DataType>>
    makePublisher(uavcan::MonotonicDuration tx_timeout = uavcan::Publisher<DataType>::getDefaultTxTimeout())
    {
        std::shared_ptr<uavcan::Publisher<DataType>> p(new uavcan::Publisher<DataType>(*this));
        enforce(p->init(), "Publisher init");
        p->setTxTimeout(tx_timeout);
        return p;
    }

    template <typename DataType>
    std::shared_ptr<uavcan::ServiceServer<DataType>>
    makeServiceServer(const typename uavcan::ServiceServer<DataType>::Callback& cb)
    {
        std::shared_ptr<uavcan::ServiceServer<DataType>> p(new uavcan::ServiceServer<DataType>(*this));
        enforce(p->start(cb), "ServiceServer start");
        return p;
    }

    template <typename DataType>
    std::shared_ptr<uavcan::ServiceClient<DataType>>
    makeServiceClient(const typename uavcan::ServiceClient<DataType>::Callback& cb)
    {
        std::shared_ptr<uavcan::ServiceClient<DataType>> p(new uavcan::ServiceClient<DataType>(*this));
        enforce(p->init(), "ServiceClient init");
        p->setCallback(cb);
        return p;
    }

    TimerPtr makeTimer(uavcan::MonotonicTime deadline, const typename uavcan::Timer::Callback& cb)
    {
        TimerPtr p(new uavcan::Timer(*this));
        p->setCallback(cb);
        p->startOneShotWithDeadline(deadline);
        return p;
    }

    TimerPtr makeTimer(uavcan::MonotonicDuration period, const typename uavcan::Timer::Callback& cb)
    {
        TimerPtr p(new uavcan::Timer(*this));
        p->setCallback(cb);
        p->startPeriodic(period);
        return p;
    }
};

typedef std::shared_ptr<Node> NodePtr;

/**
 * Constructs Node with explicitly specified ClockAdjustmentMode.
 */
static inline NodePtr makeNode(const std::vector<std::string>& iface_names, ClockAdjustmentMode clock_adjustment_mode)
{
    DriverPackPtr dp(new DriverPack(clock_adjustment_mode));
    for (auto ifn : iface_names)
    {
        if (dp->can.addIface(ifn) < 0)
        {
            throw Exception("Failed to add iface " + ifn);
        }
    }
    return NodePtr(new Node(dp));
}

/**
 * This is the preferred way to make Node.
 */
static inline NodePtr makeNode(const std::vector<std::string>& iface_names)
{
    return makeNode(iface_names, SystemClock::detectPreferredClockAdjustmentMode());
}

}
