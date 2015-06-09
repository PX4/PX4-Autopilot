/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <iostream>
#include <sstream>
#include <uavcan/uavcan.hpp>
#include <uavcan/node/sub_node.hpp>

namespace uavcan_linux
{
/**
 * Default log sink will dump everything into stderr.
 * It is installed by default.
 */
class DefaultLogSink : public uavcan::ILogSink
{
    void log(const uavcan::protocol::debug::LogMessage& message) override
    {
        const auto tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        const auto tstr = std::ctime(&tt);
        std::cerr << "### UAVCAN " << tstr << message << std::endl;
    }
};

/**
 * Wrapper over uavcan::ServiceClient<> for blocking calls.
 * Blocks on uavcan::Node::spin() internally until the call is complete.
 */
template <typename DataType>
class BlockingServiceClient : public uavcan::ServiceClient<DataType>
{
    typedef uavcan::ServiceClient<DataType> Super;

    typename DataType::Response response_;
    bool call_was_successful_;

    void callback(const uavcan::ServiceCallResult<DataType>& res)
    {
        response_ = res.getResponse();
        call_was_successful_ = res.isSuccessful();
    }

    void setup()
    {
        Super::setCallback(std::bind(&BlockingServiceClient::callback, this, std::placeholders::_1));
        call_was_successful_ = false;
        response_ = typename DataType::Response();
    }

public:
    BlockingServiceClient(uavcan::INode& node)
        : uavcan::ServiceClient<DataType>(node)
        , call_was_successful_(false)
    {
        setup();
    }

    /**
     * Performs a blocking service call using default timeout (see the specs).
     * Use @ref getResponse() to get the actual response.
     * Returns negative error code.
     */
    int blockingCall(uavcan::NodeID server_node_id, const typename DataType::Request& request)
    {
        return blockingCall(server_node_id, request, Super::getDefaultRequestTimeout());
    }

    /**
     * Performs a blocking service call using the specified timeout. Please consider using default timeout instead.
     * Use @ref getResponse() to get the actual response.
     * Returns negative error code.
     */
    int blockingCall(uavcan::NodeID server_node_id, const typename DataType::Request& request,
                     uavcan::MonotonicDuration timeout)
    {
        const auto SpinDuration = uavcan::MonotonicDuration::fromMSec(2);
        setup();
        Super::setRequestTimeout(timeout);
        const int call_res = Super::call(server_node_id, request);
        if (call_res >= 0)
        {
            while (Super::hasPendingCalls())
            {
                const int spin_res = Super::getNode().spin(SpinDuration);
                if (spin_res < 0)
                {
                    return spin_res;
                }
            }
        }
        return call_res;
    }

    /**
     * Whether the last blocking call was successful.
     */
    bool wasSuccessful() const { return call_was_successful_; }

    /**
     * Use this to retrieve the response of the last blocking service call.
     * This method returns default constructed response object if the last service call was unsuccessful.
     */
    const typename DataType::Response& getResponse() const { return response_; }
};

/**
 * Contains all drivers needed for uavcan::Node.
 */
struct DriverPack
{
    SystemClock clock;
    std::shared_ptr<uavcan::ICanDriver> can;

    explicit DriverPack(ClockAdjustmentMode clock_adjustment_mode,
                        const std::shared_ptr<uavcan::ICanDriver>& can_driver)
        : clock(clock_adjustment_mode)
        , can(can_driver)
    { }

    explicit DriverPack(ClockAdjustmentMode clock_adjustment_mode,
                        const std::vector<std::string>& iface_names)
        : clock(clock_adjustment_mode)
    {
        std::shared_ptr<SocketCanDriver> socketcan(new SocketCanDriver(clock));
        can = socketcan;
        for (auto ifn : iface_names)
        {
            if (socketcan->addIface(ifn) < 0)
            {
                throw Exception("Failed to add iface " + ifn);
            }
        }
    }
};

typedef std::shared_ptr<DriverPack> DriverPackPtr;

typedef std::shared_ptr<uavcan::INode> INodePtr;

typedef std::shared_ptr<uavcan::Timer> TimerPtr;

template <typename T>
using SubscriberPtr = std::shared_ptr<uavcan::Subscriber<T>>;

template <typename T>
using PublisherPtr = std::shared_ptr<uavcan::Publisher<T>>;

template <typename T>
using ServiceServerPtr = std::shared_ptr<uavcan::ServiceServer<T>>;

template <typename T>
using ServiceClientPtr = std::shared_ptr<uavcan::ServiceClient<T>>;

template <typename T>
using BlockingServiceClientPtr = std::shared_ptr<BlockingServiceClient<T>>;

static constexpr std::size_t NodeMemPoolSize = 1024 * 512;  ///< This shall be enough for any possible use case

/**
 * Generic wrapper for node objects with some additional convenience functions.
 */
template <typename NodeType>
class NodeBase : public NodeType
{
protected:
    DriverPackPtr driver_pack_;

    static void enforce(int error, const std::string& msg)
    {
        if (error < 0)
        {
            std::ostringstream os;
            os << msg << " [" << error << "]";
            throw Exception(os.str());
        }
    }

    template <typename DataType>
    static std::string getDataTypeName()
    {
        return DataType::getDataTypeFullName();
    }

public:
    /**
     * Simple forwarding constructor, compatible with uavcan::Node.
     */
    NodeBase(uavcan::ICanDriver& can_driver, uavcan::ISystemClock& clock) :
        NodeType(can_driver, clock)
    { }

    /**
     * Takes ownership of the driver container via the shared pointer.
     */
    explicit NodeBase(DriverPackPtr driver_pack)
        : NodeType(*driver_pack->can, driver_pack->clock)
        , driver_pack_(driver_pack)
    { }

    /**
     * Allocates @ref uavcan::Subscriber in the heap using shared pointer.
     * The subscriber will be started immediately.
     * @throws uavcan_linux::Exception.
     */
    template <typename DataType>
    SubscriberPtr<DataType> makeSubscriber(const typename uavcan::Subscriber<DataType>::Callback& cb)
    {
        SubscriberPtr<DataType> p(new uavcan::Subscriber<DataType>(*this));
        enforce(p->start(cb), "Subscriber start failure " + getDataTypeName<DataType>());
        return p;
    }

    /**
     * Allocates @ref uavcan::Publisher in the heap using shared pointer.
     * The publisher will be initialized immediately.
     * @throws uavcan_linux::Exception.
     */
    template <typename DataType>
    PublisherPtr<DataType> makePublisher(uavcan::MonotonicDuration tx_timeout =
                                             uavcan::Publisher<DataType>::getDefaultTxTimeout())
    {
        PublisherPtr<DataType> p(new uavcan::Publisher<DataType>(*this));
        enforce(p->init(), "Publisher init failure " + getDataTypeName<DataType>());
        p->setTxTimeout(tx_timeout);
        return p;
    }

    /**
     * Allocates @ref uavcan::ServiceServer in the heap using shared pointer.
     * The server will be started immediately.
     * @throws uavcan_linux::Exception.
     */
    template <typename DataType>
    ServiceServerPtr<DataType> makeServiceServer(const typename uavcan::ServiceServer<DataType>::Callback& cb)
    {
        ServiceServerPtr<DataType> p(new uavcan::ServiceServer<DataType>(*this));
        enforce(p->start(cb), "ServiceServer start failure " + getDataTypeName<DataType>());
        return p;
    }

    /**
     * Allocates @ref uavcan::ServiceClient in the heap using shared pointer.
     * The service client will be initialized immediately.
     * @throws uavcan_linux::Exception.
     */
    template <typename DataType>
    ServiceClientPtr<DataType> makeServiceClient(const typename uavcan::ServiceClient<DataType>::Callback& cb)
    {
        ServiceClientPtr<DataType> p(new uavcan::ServiceClient<DataType>(*this));
        enforce(p->init(), "ServiceClient init failure " + getDataTypeName<DataType>());
        p->setCallback(cb);
        return p;
    }

    /**
     * Allocates @ref uavcan_linux::BlockingServiceClient in the heap using shared pointer.
     * The service client will be initialized immediately.
     * @throws uavcan_linux::Exception.
     */
    template <typename DataType>
    BlockingServiceClientPtr<DataType> makeBlockingServiceClient()
    {
        BlockingServiceClientPtr<DataType> p(new BlockingServiceClient<DataType>(*this));
        enforce(p->init(), "BlockingServiceClient init failure " + getDataTypeName<DataType>());
        return p;
    }

    /**
     * Allocates @ref uavcan::Timer in the heap using shared pointer.
     * The timer will be started immediately in one-shot mode.
     */
    TimerPtr makeTimer(uavcan::MonotonicTime deadline, const typename uavcan::Timer::Callback& cb)
    {
        TimerPtr p(new uavcan::Timer(*this));
        p->setCallback(cb);
        p->startOneShotWithDeadline(deadline);
        return p;
    }

    /**
     * Allocates @ref uavcan::Timer in the heap using shared pointer.
     * The timer will be started immediately in periodic mode.
     */
    TimerPtr makeTimer(uavcan::MonotonicDuration period, const typename uavcan::Timer::Callback& cb)
    {
        TimerPtr p(new uavcan::Timer(*this));
        p->setCallback(cb);
        p->startPeriodic(period);
        return p;
    }

    const DriverPackPtr& getDriverPack() const { return driver_pack_; }
    DriverPackPtr& getDriverPack() { return driver_pack_; }
};

/**
 * Wrapper for uavcan::Node with some additional convenience functions.
 * Note that this wrapper adds stderr log sink to @ref uavcan::Logger, which can be removed if needed.
 * Do not instantiate this class directly; instead use the factory functions defined below.
 */
class Node : public NodeBase<uavcan::Node<NodeMemPoolSize>>
{
    typedef NodeBase<uavcan::Node<NodeMemPoolSize>> Base;

    DefaultLogSink log_sink_;

public:
    /**
     * Simple forwarding constructor, compatible with uavcan::Node.
     */
    Node(uavcan::ICanDriver& can_driver, uavcan::ISystemClock& clock) :
        Base(can_driver, clock)
    {
        getLogger().setExternalSink(&log_sink_);
    }

    /**
     * Takes ownership of the driver container via the shared pointer.
     */
    explicit Node(DriverPackPtr driver_pack) :
        Base(driver_pack)
    {
        getLogger().setExternalSink(&log_sink_);
    }
};

/**
 * Wrapper for uavcan::SubNode with some additional convenience functions.
 * Do not instantiate this class directly; instead use the factory functions defined below.
 */
class SubNode : public NodeBase<uavcan::SubNode<NodeMemPoolSize>>
{
    typedef NodeBase<uavcan::SubNode<NodeMemPoolSize>> Base;

public:
    /**
     * Simple forwarding constructor, compatible with uavcan::Node.
     */
    SubNode(uavcan::ICanDriver& can_driver, uavcan::ISystemClock& clock) : Base(can_driver, clock) { }

    /**
     * Takes ownership of the driver container via the shared pointer.
     */
    explicit SubNode(DriverPackPtr driver_pack) : Base(driver_pack) { }
};

typedef std::shared_ptr<Node> NodePtr;
typedef std::shared_ptr<SubNode> SubNodePtr;

/**
 * Use this function to create a node instance with default SocketCAN driver.
 * It accepts the list of interface names to use for the new node, e.g. "can1", "vcan2", "slcan0".
 * Clock adjustment mode will be detected automatically unless provided explicitly.
 * @throws uavcan_linux::Exception.
 */
static inline NodePtr makeNode(const std::vector<std::string>& iface_names,
                               ClockAdjustmentMode clock_adjustment_mode =
                                   SystemClock::detectPreferredClockAdjustmentMode())
{
    DriverPackPtr dp(new DriverPack(clock_adjustment_mode, iface_names));
    return NodePtr(new Node(dp));
}

/**
 * Use this function to create a node instance with a custom driver.
 * Clock adjustment mode will be detected automatically unless provided explicitly.
 * @throws uavcan_linux::Exception.
 */
static inline NodePtr makeNode(const std::shared_ptr<uavcan::ICanDriver>& can_driver,
                               ClockAdjustmentMode clock_adjustment_mode =
                                   SystemClock::detectPreferredClockAdjustmentMode())
{
    DriverPackPtr dp(new DriverPack(clock_adjustment_mode, can_driver));
    return NodePtr(new Node(dp));
}

/**
 * Use this function to create a sub-node instance with default SocketCAN driver.
 * It accepts the list of interface names to use for the new node, e.g. "can1", "vcan2", "slcan0".
 * Clock adjustment mode will be detected automatically unless provided explicitly.
 * @throws uavcan_linux::Exception.
 */
static inline SubNodePtr makeSubNode(const std::vector<std::string>& iface_names,
                                  ClockAdjustmentMode clock_adjustment_mode =
                                      SystemClock::detectPreferredClockAdjustmentMode())
{
    DriverPackPtr dp(new DriverPack(clock_adjustment_mode, iface_names));
    return SubNodePtr(new SubNode(dp));
}

/**
 * Use this function to create a sub-node instance with a custom driver.
 * Clock adjustment mode will be detected automatically unless provided explicitly.
 * @throws uavcan_linux::Exception.
 */
static inline SubNodePtr makeSubNode(const std::shared_ptr<uavcan::ICanDriver>& can_driver,
                                     ClockAdjustmentMode clock_adjustment_mode =
                                         SystemClock::detectPreferredClockAdjustmentMode())
{
    DriverPackPtr dp(new DriverPack(clock_adjustment_mode, can_driver));
    return SubNodePtr(new SubNode(dp));
}

}
