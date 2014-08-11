/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/error.hpp>
#include <uavcan/node/abstract_node.hpp>
#include <uavcan/data_type.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/util/templates.hpp>
#include <uavcan/util/lazy_constructor.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/transport/transfer_listener.hpp>
#include <uavcan/marshal/scalar_codec.hpp>
#include <uavcan/marshal/types.hpp>

namespace uavcan
{
/**
 * This class extends the data structure with extra information obtained from the transport layer,
 * such as Source Node ID, timestamps, Transfer ID, index of the interface this transfer was picked up from, etc.
 *
 * PLEASE NOTE that since this class inherits the data structure type, subscription callbacks can accept either
 * object of this class or the data structure type directly if the extra information is not needed.
 *
 * For example, both of these callbacks can be used with the same data structure 'Foo':
 *  void first(const ReceivedDataStructure<Foo>& msg);
 *  void second(const Foo& msg);
 * In the latter case, an implicit cast will happen before the callback is invoked.
 */
template <typename DataType_>
class UAVCAN_EXPORT ReceivedDataStructure : public DataType_
{
    const IncomingTransfer* _transfer_;   ///< Such weird name is necessary to avoid clashing with DataType fields

    template <typename Ret, Ret(IncomingTransfer::*Fun) () const>
    Ret safeget() const
    {
        if (_transfer_ == NULL)
        {
            UAVCAN_ASSERT(0);
            return Ret();
        }
        return (_transfer_->*Fun)();
    }

protected:
    ReceivedDataStructure() : _transfer_(NULL) { }

    void setTransfer(const IncomingTransfer* arg_transfer)
    {
        UAVCAN_ASSERT(arg_transfer != NULL);
        _transfer_ = arg_transfer;
    }

public:
    typedef DataType_ DataType;

    MonotonicTime getMonotonicTimestamp() const
    {
        return safeget<MonotonicTime, &IncomingTransfer::getMonotonicTimestamp>();
    }
    UtcTime getUtcTimestamp()        const { return safeget<UtcTime, &IncomingTransfer::getUtcTimestamp>(); }
    TransferType getTransferType()   const { return safeget<TransferType, &IncomingTransfer::getTransferType>(); }
    TransferID getTransferID()       const { return safeget<TransferID, &IncomingTransfer::getTransferID>(); }
    NodeID getSrcNodeID()            const { return safeget<NodeID, &IncomingTransfer::getSrcNodeID>(); }
    uint8_t getIfaceIndex()          const { return safeget<uint8_t, &IncomingTransfer::getIfaceIndex>(); }
};

/**
 * This operator neatly prints the data structure prepended with extra data from the transport layer.
 * The extra data will be represented as YAML comment.
 */
template <typename Stream, typename DataType>
static Stream& operator<<(Stream& s, const ReceivedDataStructure<DataType>& rds)
{
    s << "# Received struct ts_m=" << rds.getMonotonicTimestamp()
      << " ts_utc=" << rds.getUtcTimestamp()
      << " snid=" << int(rds.getSrcNodeID().get()) << "\n";
    s << static_cast<const DataType&>(rds);
    return s;
}


class GenericSubscriberBase : Noncopyable
{
protected:
    INode& node_;
    uint32_t failure_count_;

    explicit GenericSubscriberBase(INode& node)
        : node_(node)
        , failure_count_(0)
    { }

    ~GenericSubscriberBase() { }

    int genericStart(TransferListenerBase* listener, bool (Dispatcher::*registration_method)(TransferListenerBase*));

    void stop(TransferListenerBase* listener);

public:
    /**
     * Returns the number of failed attempts to decode received message. Generally, a failed attempt means either:
     * - Transient failure in the transport layer.
     * - Incompatible data types.
     */
    uint32_t getFailureCount() const { return failure_count_; }

    INode& getNode() const { return node_; }
};

/**
 * This helper class does some compile-time magic on the transport layer machinery. For authorized personnel only.
 */
template <typename DataStruct_, unsigned NumStaticReceivers_, unsigned NumStaticBufs_>
class UAVCAN_EXPORT TransferListenerInstantiationHelper
{
    enum { DataTypeMaxByteLen = BitLenToByteLen<DataStruct_::MaxBitLen>::Result };
    enum { NeedsBuffer = int(DataTypeMaxByteLen) > int(MaxSingleFrameTransferPayloadLen) };
    enum { BufferSize = NeedsBuffer ? DataTypeMaxByteLen : 0 };
#if UAVCAN_TINY
    enum { NumStaticBufs = 0 };
    enum { NumStaticReceivers = 0 };
#else
    enum { NumStaticBufs = NeedsBuffer ? NumStaticBufs_: 0 };
    enum { NumStaticReceivers = NumStaticReceivers_ };
#endif

public:
    typedef TransferListener<BufferSize, NumStaticBufs, NumStaticReceivers> Type;
};


template <typename DataSpec, typename DataStruct, typename TransferListenerType>
class UAVCAN_EXPORT GenericSubscriber : public GenericSubscriberBase
{
    typedef GenericSubscriber<DataSpec, DataStruct, TransferListenerType> SelfType;

    // We need to break the inheritance chain here to implement lazy initialization
    class TransferForwarder : public TransferListenerType
    {
        SelfType& obj_;

        void handleIncomingTransfer(IncomingTransfer& transfer)
        {
            obj_.handleIncomingTransfer(transfer);
        }

    public:
        TransferForwarder(SelfType& obj, const DataTypeDescriptor& data_type, IPoolAllocator& allocator)
            : TransferListenerType(obj.node_.getDispatcher().getTransferPerfCounter(), data_type, allocator)
            , obj_(obj)
        { }
    };

    struct ReceivedDataStructureSpec : public ReceivedDataStructure<DataStruct>
    {
        using ReceivedDataStructure<DataStruct>::setTransfer;
    };

    LazyConstructor<TransferForwarder> forwarder_;
    ReceivedDataStructureSpec message_;

    int checkInit();

    bool decodeTransfer(IncomingTransfer& transfer);

    void handleIncomingTransfer(IncomingTransfer& transfer);

    int genericStart(bool (Dispatcher::*registration_method)(TransferListenerBase*));

protected:
    explicit GenericSubscriber(INode& node)
        : GenericSubscriberBase(node)
    { }

    virtual ~GenericSubscriber() { stop(); }

    virtual void handleReceivedDataStruct(ReceivedDataStructure<DataStruct>&) = 0;

    int startAsMessageListener()
    {
        return genericStart(&Dispatcher::registerMessageListener);
    }

    int startAsServiceRequestListener()
    {
        return genericStart(&Dispatcher::registerServiceRequestListener);
    }

    int startAsServiceResponseListener()
    {
        return genericStart(&Dispatcher::registerServiceResponseListener);
    }

    /**
     * Terminate the subscription.
     * Dispatcher core will remove this instance from the subscribers list.
     */
    void stop()
    {
        GenericSubscriberBase::stop(forwarder_);
    }

    TransferListenerType* getTransferListener() { return forwarder_; }

    /**
     * Returns the mutable reference to the temporary storage for decoded received messages.
     * Reference to this storage is used as a parameter for subscription callbacks.
     */
    ReceivedDataStructure<DataStruct>& getReceivedStructStorage() { return message_; }
};

// ----------------------------------------------------------------------------

/*
 * GenericSubscriber
 */
template <typename DataSpec, typename DataStruct, typename TransferListenerType>
int GenericSubscriber<DataSpec, DataStruct, TransferListenerType>::checkInit()
{
    if (forwarder_)
    {
        return 0;
    }
    GlobalDataTypeRegistry::instance().freeze();
    const DataTypeDescriptor* const descr =
        GlobalDataTypeRegistry::instance().find(DataTypeKind(DataSpec::DataTypeKind), DataSpec::getDataTypeFullName());
    if (!descr)
    {
        UAVCAN_TRACE("GenericSubscriber", "Type [%s] is not registered", DataSpec::getDataTypeFullName());
        return -ErrUnknownDataType;
    }
    forwarder_.template construct<SelfType&, const DataTypeDescriptor&, IPoolAllocator&>
        (*this, *descr, node_.getAllocator());
    return 0;
}

template <typename DataSpec, typename DataStruct, typename TransferListenerType>
bool GenericSubscriber<DataSpec, DataStruct, TransferListenerType>::decodeTransfer(IncomingTransfer& transfer)
{
    BitStream bitstream(transfer);
    ScalarCodec codec(bitstream);

    message_.setTransfer(&transfer);

    const int decode_res = DataStruct::decode(message_, codec);
    // We don't need the data anymore, the memory can be reused from the callback:
    transfer.release();
    if (decode_res <= 0)
    {
        UAVCAN_TRACE("GenericSubscriber", "Unable to decode the message [%i] [%s]",
                     decode_res, DataSpec::getDataTypeFullName());
        failure_count_++;
        node_.getDispatcher().getTransferPerfCounter().addError();
        return false;
    }
    return true;
}

template <typename DataSpec, typename DataStruct, typename TransferListenerType>
void GenericSubscriber<DataSpec, DataStruct, TransferListenerType>::handleIncomingTransfer(IncomingTransfer& transfer)
{
    if (decodeTransfer(transfer))
    {
        handleReceivedDataStruct(message_);
    }
}

template <typename DataSpec, typename DataStruct, typename TransferListenerType>
int GenericSubscriber<DataSpec, DataStruct, TransferListenerType>::
genericStart(bool (Dispatcher::*registration_method)(TransferListenerBase*))
{
    const int res = checkInit();
    if (res < 0)
    {
        UAVCAN_TRACE("GenericSubscriber", "Initialization failure [%s]", DataSpec::getDataTypeFullName());
        return res;
    }
    return GenericSubscriberBase::genericStart(forwarder_, registration_method);
}


}
