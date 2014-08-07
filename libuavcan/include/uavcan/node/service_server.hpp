/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/build_config.hpp>
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
/**
 * Use this class to implement UAVCAN service servers.
 *
 * @tparam DataType_        Service data type.
 *
 * @tparam Callback_        Service calls will be delivered through the callback of this type, and service
 *                          response will be returned via the output parameter of the callback. Note that
 *                          the reference to service response data struct passed to the callback always points
 *                          to a default initialized response object.
 *                          Please also refer to @ref ReceivedDataStructure<>.
 *                          In C++11 mode this type defaults to std::function<>.
 *                          In C++03 mode this type defaults to a plain function pointer; use binder to
 *                          call member functions as callbacks.
 *
 * @tparam NumStaticReceivers   Number of statically allocated receiver objects. If there's more service
 *                              clients for this service, extra receivers will be allocated in the memory pool.
 *
 * @tparam NumStaticBufs        Number of statically allocated receiver buffers. If there's more concurrent
 *                              incoming transfers, extra buffers will be allocated in the memory pool.
 */
template <typename DataType_,
#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
          typename Callback_ = std::function<void (const ReceivedDataStructure<typename DataType_::Request>&,
                                                   typename DataType_::Response&)>,
#else
          typename Callback_ = void (*)(const ReceivedDataStructure<typename DataType_::Request>&,
                                        typename DataType_::Response&),
#endif
#if UAVCAN_TINY
          unsigned NumStaticReceivers = 0,
          unsigned NumStaticBufs = 0
#else
          unsigned NumStaticReceivers = 2,
          unsigned NumStaticBufs = 1
#endif
          >
class UAVCAN_EXPORT ServiceServer
    : public GenericSubscriber<DataType_, typename DataType_::Request,
                               typename TransferListenerInstantiationHelper<typename DataType_::Request,
                                                                            NumStaticReceivers, NumStaticBufs>::Type>
{
public:
    typedef DataType_ DataType;
    typedef typename DataType::Request RequestType;
    typedef typename DataType::Response ResponseType;
    typedef Callback_ Callback;

private:
    typedef typename TransferListenerInstantiationHelper<RequestType, NumStaticReceivers, NumStaticBufs>::Type
        TransferListenerType;
    typedef GenericSubscriber<DataType, RequestType, TransferListenerType> SubscriberType;
    typedef GenericPublisher<DataType, ResponseType> PublisherType;

    PublisherType publisher_;
    Callback callback_;
    uint32_t response_failure_count_;
    ResponseType response_;

    virtual void handleReceivedDataStruct(ReceivedDataStructure<RequestType>& request)
    {
        UAVCAN_ASSERT(request.getTransferType() == TransferTypeServiceRequest);
        if (try_implicit_cast<bool>(callback_, true))
        {
            response_ = ResponseType();  // The application needs newly initialized structure
            callback_(request, response_);
        }
        else
        {
            handleFatalError("Srv serv clbk");
        }

        const int res = publisher_.publish(response_, TransferTypeServiceResponse, request.getSrcNodeID(),
                                           request.getTransferID());
        if (res < 0)
        {
            UAVCAN_TRACE("ServiceServer", "Response publication failure: %i", res);
            publisher_.getNode().getDispatcher().getTransferPerfCounter().addError();
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
        UAVCAN_ASSERT(getTxTimeout() == getDefaultTxTimeout());  // Making sure it is valid

        StaticAssert<DataTypeKind(DataType::DataTypeKind) == DataTypeKindService>::check();
    }

    /**
     * Starts the server.
     * Incoming service requests will be passed to the application via the callback.
     */
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

    /**
     * Stops the server.
     */
    using SubscriberType::stop;

    static MonotonicDuration getDefaultTxTimeout() { return MonotonicDuration::fromMSec(1000); }
    static MonotonicDuration getMinTxTimeout() { return PublisherType::getMinTxTimeout(); }
    static MonotonicDuration getMaxTxTimeout() { return PublisherType::getMaxTxTimeout(); }

    MonotonicDuration getTxTimeout() const { return publisher_.getTxTimeout(); }
    void setTxTimeout(MonotonicDuration tx_timeout) { publisher_.setTxTimeout(tx_timeout); }

    /**
     * Returns the number of failed attempts to decode data structs. Generally, a failed attempt means either:
     * - Transient failure in the transport layer.
     * - Incompatible data types.
     */
    uint32_t getRequestFailureCount() const { return SubscriberType::getFailureCount(); }
    uint32_t getResponseFailureCount() const { return response_failure_count_; }
};

}
