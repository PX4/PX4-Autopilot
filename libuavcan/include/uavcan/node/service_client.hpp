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

template <typename ServiceDataType>
class UAVCAN_EXPORT ServiceResponseTransferListenerInstantiationHelper
{
    enum { DataTypeMaxByteLen = BitLenToByteLen<ServiceDataType::Response::MaxBitLen>::Result };
public:
    typedef ServiceResponseTransferListener<DataTypeMaxByteLen> Type;
};

/**
 * Object of this type will be returned to the application as a result of service call.
 * Note that application ALWAYS gets this result, even when it times out or fails because of some other reason.
 */
template <typename DataType>
struct UAVCAN_EXPORT ServiceCallResult
{
    typedef ReceivedDataStructure<typename DataType::Response> ResponseFieldType;

    enum Status { Success, ErrorTimeout };

    const Status status;              ///< Whether successful or not. Failure to decode the response causes timeout.
    NodeID server_node_id;            ///< Node ID of the server this call was addressed to.
    ResponseFieldType& response;      ///< Returned data structure. Undefined if the service call has failed.

    ServiceCallResult(Status arg_status, NodeID arg_server_node_id, ResponseFieldType& arg_response)
        : status(arg_status)
        , server_node_id(arg_server_node_id)
        , response(arg_response)
    {
        UAVCAN_ASSERT(server_node_id.isUnicast());
        UAVCAN_ASSERT((status == Success) || (status == ErrorTimeout));
    }

    /**
     * Shortcut to quickly check whether the call was successful.
     */
    bool isSuccessful() const { return status == Success; }
};

/**
 * This operator neatly prints the service call result prepended with extra data like Server Node ID.
 * The extra data will be represented as YAML comment.
 */
template <typename Stream, typename DataType>
static Stream& operator<<(Stream& s, const ServiceCallResult<DataType>& scr)
{
    s << "# Service call result [" << DataType::getDataTypeFullName() << "] "
      << (scr.isSuccessful() ? "OK" : "FAILURE")
      << " server_node_id=" << int(scr.server_node_id.get()) << "\n";
    if (scr.isSuccessful())
    {
        s << scr.response;
    }
    else
    {
        s << "# (no data)";
    }
    return s;
}

/**
 * Do not use directly.
 */
class ServiceClientBase : protected DeadlineHandler
{
protected:
    MonotonicDuration request_timeout_;
    bool pending_;

    explicit ServiceClientBase(INode& node)
        : DeadlineHandler(node.getScheduler())
        , request_timeout_(getDefaultRequestTimeout())
        , pending_(false)
    { }

    virtual ~ServiceClientBase() { }

    int prepareToCall(INode& node, const char* dtname, NodeID server_node_id, TransferID& out_transfer_id);

public:
    /**
     * Returns true if the service call is currently in progress.
     */
    bool isPending() const { return pending_; }

    /**
     * It's not recommended to override default timeouts.
     */
    static MonotonicDuration getDefaultRequestTimeout() { return MonotonicDuration::fromMSec(1000); }
    static MonotonicDuration getMinRequestTimeout() { return MonotonicDuration::fromMSec(10); }
    static MonotonicDuration getMaxRequestTimeout() { return MonotonicDuration::fromMSec(60000); }

    /**
     * Returns the service response waiting deadline, if pending.
     */
    using DeadlineHandler::getDeadline;
};

/**
 * Use this class to invoke services on remote nodes.
 *
 * @tparam DataType_        Service data type.
 *
 * @tparam Callback_        Service response will be delivered through the callback of this type.
 *                          In C++11 mode this type defaults to std::function<>.
 *                          In C++03 mode this type defaults to a plain function pointer; use binder to
 *                          call member functions as callbacks.
 */
template <typename DataType_,
#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
          typename Callback_ = std::function<void (const ServiceCallResult<DataType_>&)>
#else
          typename Callback_ = void (*)(const ServiceCallResult<DataType_>&)
#endif
          >
class UAVCAN_EXPORT ServiceClient
    : public GenericSubscriber<DataType_, typename DataType_::Response,
                               typename ServiceResponseTransferListenerInstantiationHelper<DataType_>::Type >
    , public ServiceClientBase
{
public:
    typedef DataType_ DataType;
    typedef typename DataType::Request RequestType;
    typedef typename DataType::Response ResponseType;
    typedef ServiceCallResult<DataType> ServiceCallResultType;
    typedef Callback_ Callback;

private:
    typedef ServiceClient<DataType, Callback> SelfType;
    typedef GenericPublisher<DataType, RequestType> PublisherType;
    typedef typename ServiceResponseTransferListenerInstantiationHelper<DataType>::Type TransferListenerType;
    typedef GenericSubscriber<DataType, ResponseType, TransferListenerType> SubscriberType;

    PublisherType publisher_;
    Callback callback_;

    bool isCallbackValid() const { return try_implicit_cast<bool>(callback_, true); }

    void invokeCallback(ServiceCallResultType& result);

    void handleReceivedDataStruct(ReceivedDataStructure<ResponseType>& response);

    virtual void handleDeadline(MonotonicTime);

public:
    /**
     * @param node      Node instance this client will be registered with.
     * @param callback  Callback instance. Optional, can be assigned later.
     */
    explicit ServiceClient(INode& node, const Callback& callback = Callback())
        : SubscriberType(node)
        , ServiceClientBase(node)
        , publisher_(node, getDefaultRequestTimeout())
        , callback_(callback)
    {
        setRequestTimeout(getDefaultRequestTimeout());
#if UAVCAN_DEBUG
        UAVCAN_ASSERT(getRequestTimeout() == getDefaultRequestTimeout());  // Making sure default values are OK
#endif
    }

    virtual ~ServiceClient() { cancel(); }

    /**
     * Shall be called before first use.
     * Returns negative error code.
     */
    int init()
    {
        return publisher_.init();
    }

    /**
     * Performs non-blocking service call.
     * This method transmits the service request and returns immediately.
     *
     * Service response will be delivered into the application via the callback.
     * Note that the callback will ALWAYS be called even if the service call has timed out; the
     * actual result of the call (success/failure) will be passed to the callback as well.
     *
     * If this client instance is already pending service response, it will be cancelled and the new
     * call will be performed.
     *
     * Returns negative error code.
     */
    int call(NodeID server_node_id, const RequestType& request);

    /**
     * Cancel the pending service call.
     * Does nothing if it is not pending.
     */
    void cancel();

    /**
     * Service response callback must be set prior service call.
     */
    const Callback& getCallback() const { return callback_; }
    void setCallback(const Callback& cb) { callback_ = cb; }

    /**
     * Returns the number of failed attempts to decode received response. Generally, a failed attempt means either:
     * - Transient failure in the transport layer.
     * - Incompatible data types.
     */
    uint32_t getResponseFailureCount() const { return SubscriberType::getFailureCount(); }

    /**
     * Request timeouts.
     * There is no such config as TX timeout - TX timeouts are configured automagically according to request timeouts.
     * Not recommended to change.
     */
    MonotonicDuration getRequestTimeout() const { return request_timeout_; }
    void setRequestTimeout(MonotonicDuration timeout)
    {
        timeout = max(timeout, getMinRequestTimeout());
        timeout = min(timeout, getMaxRequestTimeout());

        publisher_.setTxTimeout(timeout);
        request_timeout_ = max(timeout, publisher_.getTxTimeout());  // No less than TX timeout
    }
};

// ----------------------------------------------------------------------------

template <typename DataType_, typename Callback_>
void ServiceClient<DataType_, Callback_>::invokeCallback(ServiceCallResultType& result)
{
    if (isCallbackValid())
    {
        callback_(result);
    }
    else
    {
        handleFatalError("Srv client clbk");
    }
}

template <typename DataType_, typename Callback_>
void ServiceClient<DataType_, Callback_>::handleReceivedDataStruct(ReceivedDataStructure<ResponseType>& response)
{
    UAVCAN_ASSERT(response.getTransferType() == TransferTypeServiceResponse);
    const TransferListenerType* const listener = SubscriberType::getTransferListener();
    if (listener)
    {
        const typename TransferListenerType::ExpectedResponseParams erp = listener->getExpectedResponseParams();
        ServiceCallResultType result(ServiceCallResultType::Success, erp.src_node_id, response);
        cancel();
        invokeCallback(result);
    }
    else
    {
        UAVCAN_ASSERT(0);
        cancel();
    }
}

template <typename DataType_, typename Callback_>
void ServiceClient<DataType_, Callback_>::handleDeadline(MonotonicTime)
{
    const TransferListenerType* const listener = SubscriberType::getTransferListener();
    if (listener)
    {
        const typename TransferListenerType::ExpectedResponseParams erp = listener->getExpectedResponseParams();
        ReceivedDataStructure<ResponseType>& ref = SubscriberType::getReceivedStructStorage();
        ServiceCallResultType result(ServiceCallResultType::ErrorTimeout, erp.src_node_id, ref);

        UAVCAN_TRACE("ServiceClient", "Timeout from nid=%i, dtname=%s",
                     erp.src_node_id.get(), DataType::getDataTypeFullName());
        cancel();
        invokeCallback(result);
    }
    else
    {
        UAVCAN_ASSERT(0);
        cancel();
    }
}

template <typename DataType_, typename Callback_>
int ServiceClient<DataType_, Callback_>::call(NodeID server_node_id, const RequestType& request)
{
    cancel();
    if (!isCallbackValid())
    {
        UAVCAN_TRACE("ServiceClient", "Invalid callback");
        return -ErrInvalidParam;
    }

    /*
     * Common procedures that don't depend on the struct data type
     */
    TransferID transfer_id;
    const int prep_res =
        prepareToCall(SubscriberType::getNode(), DataType::getDataTypeFullName(), server_node_id, transfer_id);
    if (prep_res < 0)
    {
        UAVCAN_TRACE("ServiceClient", "Failed to prepare the call, error: %i", prep_res);
        cancel();
        return prep_res;
    }

    /*
     * Starting the subscriber
     */
    const int subscriber_res = SubscriberType::startAsServiceResponseListener();
    if (subscriber_res < 0)
    {
        UAVCAN_TRACE("ServiceClient", "Failed to start the subscriber, error: %i", subscriber_res);
        cancel();
        return subscriber_res;
    }

    /*
     * Configuring the listener so it will accept only the matching response
     */
    TransferListenerType* const tl = SubscriberType::getTransferListener();
    if (!tl)
    {
        UAVCAN_ASSERT(0);  // Must have been created
        cancel();
        return -ErrLogic;
    }
    const typename TransferListenerType::ExpectedResponseParams erp(server_node_id, transfer_id);
    tl->setExpectedResponseParams(erp);

    /*
     * Publishing the request
     */
    const int publisher_res = publisher_.publish(request, TransferTypeServiceRequest, server_node_id, transfer_id);
    if (!publisher_res)
    {
        cancel();
    }
    return publisher_res;
}

template <typename DataType_, typename Callback_>
void ServiceClient<DataType_, Callback_>::cancel()
{
    pending_ = false;
    SubscriberType::stop();
    DeadlineHandler::stop();
    TransferListenerType* const tl = SubscriberType::getTransferListener();
    if (tl)
    {
        tl->stopAcceptingAnything();
    }
}

}
