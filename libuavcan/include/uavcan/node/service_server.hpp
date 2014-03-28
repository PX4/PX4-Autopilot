/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/impl_constants.hpp>
#include <uavcan/node/generic_publisher.hpp>
#include <uavcan/node/generic_subscriber.hpp>

#if !defined(UAVCAN_CPP_VERSION) || !defined(UAVCAN_CPP11)
# error UAVCAN_CPP_VERSION
#endif

#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
# include <functional>
#endif

namespace uavcan
{

template <typename DataType_,
#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
          typename Callback = std::function<void (const ReceivedDataStructure<typename DataType_::Request>&,
                                                  typename DataType_::Response&)>,
#else
          typename Callback = void (*)(const ReceivedDataStructure<typename DataType_::Request>&,
                                       typename DataType_::Response&),
#endif
          unsigned int NumStaticReceivers = 2,
          unsigned int NumStaticBufs = 1>
class ServiceServer : public GenericSubscriber<DataType_, typename DataType_::Request,
                                               typename TransferListenerInstantiationHelper<typename DataType_::Request,
                                                                                            NumStaticReceivers,
                                                                                            NumStaticBufs>::Type>
{
public:
    typedef DataType_ DataType;
    typedef typename DataType::Request RequestType;
    typedef typename DataType::Response ResponseType;

private:
    typedef typename TransferListenerInstantiationHelper<RequestType, NumStaticReceivers, NumStaticBufs>::Type
        TransferListenerType;
    typedef GenericSubscriber<DataType, RequestType, TransferListenerType> SubscriberType;
    typedef GenericPublisher<DataType, ResponseType> PublisherType;

    PublisherType publisher_;
    Callback callback_;
    uint32_t response_failure_count_;
    ResponseType response_;

    void handleReceivedDataStruct(ReceivedDataStructure<RequestType>& request)
    {
        assert(request.getTransferType() == TransferTypeServiceRequest);
        if (try_implicit_cast<bool>(callback_, true))
        {
            response_ = ResponseType();  // The application needs newly initialized structure
            callback_(request, response_);
        }
        else
        {
            handleFatalError("Invalid service server callback");
        }

        const int res = publisher_.publish(response_, TransferTypeServiceResponse, request.getSrcNodeID(),
                                           request.getTransferID());
        if (res < 0)
        {
            UAVCAN_TRACE("ServiceServer", "Response publication failure: %i", res);
            response_failure_count_++;
        }
    }

public:
    explicit ServiceServer(INode& node)
        : SubscriberType(node)
        , publisher_(node, getDefaultTxTimeout())
        , callback_()
        , response_failure_count_(0)
    {
        assert(getTxTimeout() == getDefaultTxTimeout());  // Making sure it is valid

        StaticAssert<DataTypeKind(DataType::DataTypeKind) == DataTypeKindService>::check();
    }

    int start(const Callback& callback)
    {
        stop();

        if (!try_implicit_cast<bool>(callback, true))
        {
            UAVCAN_TRACE("ServiceServer", "Invalid callback");
            return -ErrInvalidParam;
        }
        callback_ = callback;

        const int publisher_res = publisher_.init();
        if (publisher_res < 0)
        {
            UAVCAN_TRACE("ServiceServer", "Publisher initialization failure: %i", publisher_res);
            return publisher_res;
        }
        return SubscriberType::startAsServiceRequestListener();
    }

    using SubscriberType::stop;

    static MonotonicDuration getDefaultTxTimeout() { return MonotonicDuration::fromMSec(1000); }
    static MonotonicDuration getMinTxTimeout() { return PublisherType::getMinTxTimeout(); }
    static MonotonicDuration getMaxTxTimeout() { return PublisherType::getMaxTxTimeout(); }

    MonotonicDuration getTxTimeout() const { return publisher_.getTxTimeout(); }
    void setTxTimeout(MonotonicDuration tx_timeout) { publisher_.setTxTimeout(tx_timeout); }

    uint32_t getRequestFailureCount() const { return SubscriberType::getFailureCount(); }
    uint32_t getResponseFailureCount() const { return response_failure_count_; }
};

}
