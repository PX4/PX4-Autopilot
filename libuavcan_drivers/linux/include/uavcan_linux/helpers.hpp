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

namespace uavcan_linux
{
/**
 * Default log sink will dump everything into stderr.
 * It is installed by default.
 */
class DefaultLogSink : public uavcan::ILogSink
{
    virtual void log(const uavcan::protocol::debug::LogMessage& message)
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
        response_ = res.response;
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
            while (Super::isPending())
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
    SocketCanDriver can;

    explicit DriverPack(ClockAdjustmentMode clock_adjustment_mode)
        : clock(clock_adjustment_mode)
        , can(clock)
    { }
};

typedef std::shared_ptr<DriverPack> DriverPackPtr;

typedef std::shared_ptr<uavcan::Timer> TimerPtr;

static constexpr std::size_t NodeMemPoolSize = 1024 * 512;  ///< This shall be enough for any possible use case

/**
 * Wrapper for uavcan::Node with some additional convenience functions.
 * Note that this wrapper adds stderr log sink to @ref uavcan::Logger, which can be removed if needed.
 */
class Node : public uavcan::Node<NodeMemPoolSize>
{
    DriverPackPtr driver_pack_;
    DefaultLogSink log_sink_;

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
    Node(uavcan::ICanDriver& can_driver, uavcan::ISystemClock& clock)
        : uavcan::Node<NodeMemPoolSize>(can_driver, clock)
    {
        getLogger().setExternalSink(&log_sink_);
    }

    /**
     * Takes ownership of the driver container via the shared pointer.
     */
    explicit Node(DriverPackPtr driver_pack)
        : uavcan::Node<NodeMemPoolSize>(driver_pack->can, driver_pack->clock)
        , driver_pack_(driver_pack)
    {
        getLogger().setExternalSink(&log_sink_);
    }

    /**
     * Allocates @ref uavcan::Subscriber in the heap using shared pointer.
     * The subscriber will be started immediately.
     * @throws uavcan_linux::Exception.
     */
    template <typename DataType>
    std::shared_ptr<uavcan::Subscriber<DataType>>
    makeSubscriber(const typename uavcan::Subscriber<DataType>::Callback& cb)
    {
        std::shared_ptr<uavcan::Subscriber<DataType>> p(new uavcan::Subscriber<DataType>(*this));
        enforce(p->start(cb), "Subscriber start failure " + getDataTypeName<DataType>());
        return p;
    }

    /**
     * Allocates @ref uavcan::Publisher in the heap using shared pointer.
     * The publisher will be initialized immediately.
     * @throws uavcan_linux::Exception.
     */
    template <typename DataType>
    std::shared_ptr<uavcan::Publisher<DataType>>
    makePublisher(uavcan::MonotonicDuration tx_timeout = uavcan::Publisher<DataType>::getDefaultTxTimeout())
    {
        std::shared_ptr<uavcan::Publisher<DataType>> p(new uavcan::Publisher<DataType>(*this));
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
    std::shared_ptr<uavcan::ServiceServer<DataType>>
    makeServiceServer(const typename uavcan::ServiceServer<DataType>::Callback& cb)
    {
        std::shared_ptr<uavcan::ServiceServer<DataType>> p(new uavcan::ServiceServer<DataType>(*this));
        enforce(p->start(cb), "ServiceServer start failure " + getDataTypeName<DataType>());
        return p;
    }

    /**
     * Allocates @ref uavcan::ServiceClient in the heap using shared pointer.
     * The service client will be initialized immediately.
     * @throws uavcan_linux::Exception.
     */
    template <typename DataType>
    std::shared_ptr<uavcan::ServiceClient<DataType>>
    makeServiceClient(const typename uavcan::ServiceClient<DataType>::Callback& cb)
    {
        std::shared_ptr<uavcan::ServiceClient<DataType>> p(new uavcan::ServiceClient<DataType>(*this));
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
    std::shared_ptr<BlockingServiceClient<DataType>>
    makeBlockingServiceClient()
    {
        std::shared_ptr<BlockingServiceClient<DataType>> p(new BlockingServiceClient<DataType>(*this));
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

typedef std::shared_ptr<Node> NodePtr;

/**
 * Constructs Node with explicitly specified ClockAdjustmentMode.
 * Please consider using the overload with fewer parameters instead.
 * @throws uavcan_linux::Exception.
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
 * Use this function to create a node instance.
 * It accepts the list of interface names to use for the new node, e.g. "can1", "vcan2", "slcan0".
 * Clock adjustment mode will be detected automatically.
 * @throws uavcan_linux::Exception.
 */
static inline NodePtr makeNode(const std::vector<std::string>& iface_names)
{
    return makeNode(iface_names, SystemClock::detectPreferredClockAdjustmentMode());
}

}
