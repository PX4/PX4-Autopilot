/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/node/subscriber.hpp>
#include <uavcan/node/timer.hpp>
#include <uavcan/node/service_client.hpp>
#include <uavcan/util/method_binder.hpp>
#include "../node/test_node.hpp"


template <typename DataType>
class SubscriptionCollector : uavcan::Noncopyable
{
    typedef uavcan::ReceivedDataStructure<DataType> ReceivedDataStructType;

    void handler(const ReceivedDataStructType& msg)
    {
        this->msg.reset(new ReceivedDataStructType(msg));
    }

public:
    std::auto_ptr<ReceivedDataStructType> msg;

    typedef uavcan::MethodBinder<SubscriptionCollector*,
                                 void (SubscriptionCollector::*)(const ReceivedDataStructType&)> Binder;

    Binder bind() { return Binder(this, &SubscriptionCollector::handler); }
};


template <typename DataType>
struct SubscriberWithCollector
{
    typedef SubscriptionCollector<DataType> Collector;
    typedef uavcan::Subscriber<DataType, typename Collector::Binder> Subscriber;

    Collector collector;
    Subscriber subscriber;

    SubscriberWithCollector(uavcan::INode& node)
        : subscriber(node)
    { }

    int start() { return subscriber.start(collector.bind()); }
};


template <typename DataType>
class ServiceCallResultCollector : uavcan::Noncopyable
{
    typedef uavcan::ServiceCallResult<DataType> ResultType;

    void handler(const ResultType& result)
    {
        this->result.reset(new ResultType(result));
    }

public:
    std::auto_ptr<ResultType> result;

    typedef uavcan::MethodBinder<ServiceCallResultCollector*,
                                 void (ServiceCallResultCollector::*)(const ResultType&)> Binder;

    Binder bind() { return Binder(this, &ServiceCallResultCollector::handler); }
};


template <typename DataType>
struct ServiceClientWithCollector
{
    typedef ServiceCallResultCollector<DataType> Collector;
    typedef uavcan::ServiceClient<DataType, typename Collector::Binder> ServiceClient;

    Collector collector;
    ServiceClient client;

    ServiceClientWithCollector(uavcan::INode& node)
        : client(node)
    { }

    int call(uavcan::NodeID node_id, const typename DataType::Request& request)
    {
        client.setCallback(collector.bind());
        return client.call(node_id, request);
    }
};


struct BackgroundSpinner : uavcan::TimerBase
{
    uavcan::INode& spinning_node;

    BackgroundSpinner(uavcan::INode& spinning_node, uavcan::INode& running_node)
        : uavcan::TimerBase(running_node)
        , spinning_node(spinning_node)
    { }

    virtual void handleTimerEvent(const uavcan::TimerEvent&)
    {
        spinning_node.spin(uavcan::MonotonicDuration::fromMSec(1));
    }
};

template <typename MessageType>
static inline void emulateSingleFrameBroadcastTransfer(CanDriverMock& can, uavcan::NodeID node_id,
                                                       const MessageType& message, uavcan::TransferID tid)
{
    uavcan::StaticTransferBuffer<100> buffer;
    uavcan::BitStream bitstream(buffer);
    uavcan::ScalarCodec codec(bitstream);

    // Manual message publication
    ASSERT_LT(0, MessageType::encode(message, codec));
    ASSERT_GE(8, buffer.getMaxWritePos());

    // DataTypeID data_type_id, TransferType transfer_type, NodeID src_node_id, NodeID dst_node_id,
    // uint_fast8_t frame_index, TransferID transfer_id, bool last_frame
    uavcan::Frame frame(MessageType::DefaultDataTypeID, uavcan::TransferTypeMessageBroadcast,
                        node_id, uavcan::NodeID::Broadcast, 0, tid, true);

    ASSERT_EQ(buffer.getMaxWritePos(), frame.setPayload(buffer.getRawPtr(), buffer.getMaxWritePos()));

    uavcan::CanFrame can_frame;
    ASSERT_TRUE(frame.compile(can_frame));

    for (uint8_t i = 0; i < can.getNumIfaces(); i++)
    {
        can.ifaces.at(i).pushRx(can_frame);
    }
}
