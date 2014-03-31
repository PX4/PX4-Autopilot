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

template <typename ServiceDataType>
class ServiceResponseTransferListenerInstantiationHelper
{
    enum { DataTypeMaxByteLen = BitLenToByteLen<ServiceDataType::Response::MaxBitLen>::Result };
public:
    typedef ServiceResponseTransferListener<DataTypeMaxByteLen> Type;
};


template <typename DataType>
struct ServiceCallResult
{
    typedef ReceivedDataStructure<typename DataType::Response> ResponseFieldType;

    enum Status { Success, ErrorTimeout };

    const Status status;
    NodeID server_node_id;
    ResponseFieldType& response;      ///< Either response contents or unspecified response structure

    ServiceCallResult(Status status, NodeID server_node_id, ResponseFieldType& response)
        : status(status)
        , server_node_id(server_node_id)
        , response(response)
    {
        assert(server_node_id.isUnicast());
        assert(status == Success || status == ErrorTimeout);
    }

    bool isSuccessful() const { return status == Success; }
};

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


template <typename DataType_,
#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
          typename Callback_ = std::function<void (const ServiceCallResult<DataType_>&)>
#else
          typename Callback_ = void (*)(const ServiceCallResult<DataType_>&)
#endif
          >
class ServiceClient
    : public GenericSubscriber<DataType_, typename DataType_::Response,
                               typename ServiceResponseTransferListenerInstantiationHelper<DataType_>::Type >
    , protected DeadlineHandler
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
    MonotonicDuration request_timeout_;
    bool pending_;

    bool isCallbackValid() const { return try_implicit_cast<bool>(callback_, true); }

    void invokeCallback(ServiceCallResultType& result)
    {
        if (isCallbackValid())
        {
            callback_(result);
        }
        else
        {
            handleFatalError("Invalid caller callback");
        }
    }

    void handleReceivedDataStruct(ReceivedDataStructure<ResponseType>& response)
    {
        assert(response.getTransferType() == TransferTypeServiceResponse);
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
            assert(0);
            cancel();
        }
    }

    void handleDeadline(MonotonicTime)
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
            assert(0);
            cancel();
        }
    }

public:
    explicit ServiceClient(INode& node, const Callback& callback = Callback())
        : SubscriberType(node)
        , DeadlineHandler(node.getScheduler())
        , publisher_(node, getDefaultRequestTimeout())
        , callback_(callback)
        , request_timeout_(getDefaultRequestTimeout())
        , pending_(false)
    {
        setRequestTimeout(getDefaultRequestTimeout());
#if UAVCAN_DEBUG
        assert(getRequestTimeout() == getDefaultRequestTimeout());  // Making sure default values are OK
#endif
    }

    virtual ~ServiceClient() { cancel(); }

    int init()
    {
        return publisher_.init();
    }

    int call(NodeID server_node_id, const RequestType& request) // TODO: Refactor, move into a non-template subclass!
    {
        /*
         * Cancelling the pending request
         */
        cancel();
        if (!isCallbackValid())
        {
            UAVCAN_TRACE("ServiceClient", "Invalid callback");
            return -ErrInvalidParam;
        }
        pending_ = true;

        /*
         * Determining the Data Type ID
         */
        GlobalDataTypeRegistry::instance().freeze();
        const DataTypeDescriptor* const descr =
            GlobalDataTypeRegistry::instance().find(DataTypeKindService, DataType::getDataTypeFullName());
        if (!descr)
        {
            UAVCAN_TRACE("ServiceClient", "Type [%s] is not registered", DataType::getDataTypeFullName());
            cancel();
            return -ErrUnknownDataType;
        }

        /*
         * Determining the Transfer ID
         */
        const OutgoingTransferRegistryKey otr_key(descr->getID(), TransferTypeServiceRequest, server_node_id);
        const MonotonicTime otr_deadline =
            SubscriberType::getNode().getMonotonicTime() + TransferSender::getDefaultMaxTransferInterval();
        TransferID* const otr_tid = SubscriberType::getNode().getDispatcher().getOutgoingTransferRegistry()
                                        .accessOrCreate(otr_key, otr_deadline);
        if (!otr_tid)
        {
            UAVCAN_TRACE("ServiceClient", "OTR access failure, dtd=%s", descr->toString().c_str());
            cancel();
            return -ErrMemory;
        }
        const TransferID transfer_id = *otr_tid;
        otr_tid->increment();

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
            assert(0);  // Must have been created
            cancel();
            return -ErrLogic;
        }
        const typename TransferListenerType::ExpectedResponseParams erp(server_node_id, transfer_id);
        tl->setExpectedResponseParams(erp);

        /*
         * Registering the deadline handler
         */
        DeadlineHandler::startWithDelay(request_timeout_);

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

    void cancel()
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

    bool isPending() const { return pending_; }

    const Callback& getCallback() const { return callback_; }
    void setCallback(const Callback& cb) { callback_ = cb; }

    uint32_t getResponseFailureCount() const { return SubscriberType::getFailureCount(); }

    /*
     * Request timeouts
     * There is no such config as TX timeout - TX timeouts are configured automagically according to request timeouts
     */
    static MonotonicDuration getDefaultRequestTimeout() { return MonotonicDuration::fromMSec(1000); }
    static MonotonicDuration getMinRequestTimeout() { return MonotonicDuration::fromMSec(10); }
    static MonotonicDuration getMaxRequestTimeout() { return MonotonicDuration::fromMSec(60000); }

    MonotonicDuration getRequestTimeout() const { return request_timeout_; }
    void setRequestTimeout(MonotonicDuration timeout)
    {
        timeout = std::max(timeout, getMinRequestTimeout());
        timeout = std::min(timeout, getMaxRequestTimeout());

        publisher_.setTxTimeout(timeout);
        request_timeout_ = std::max(timeout, publisher_.getTxTimeout());  // No less than TX timeout
    }

    using DeadlineHandler::getDeadline;
};

}
