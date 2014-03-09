/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/scheduler.hpp>
#include <uavcan/data_type.hpp>
#include <uavcan/global_data_type_registry.hpp>
#include <uavcan/received_data_structure.hpp>
#include <uavcan/internal/debug.hpp>
#include <uavcan/internal/util.hpp>
#include <uavcan/internal/lazy_constructor.hpp>
#include <uavcan/internal/transport/transfer_listener.hpp>
#include <uavcan/internal/marshal/scalar_codec.hpp>
#include <uavcan/internal/marshal/types.hpp>

namespace uavcan
{

template <typename DataType_,
          typename Callback = void(*)(const ReceivedDataStructure<DataType_>&),
          unsigned int NumStaticBufs = 1,
          unsigned int NumStaticReceivers = NumStaticBufs + 1>
class Subscriber : Noncopyable
{
    typedef Subscriber<DataType_, Callback, NumStaticBufs, NumStaticReceivers> SelfType;

public:
    typedef DataType_ DataType;

private:
    typedef TransferListener<BitLenToByteLen<DataType::MaxBitLen>::Result,
                             NumStaticBufs ? NumStaticBufs : 1,            // TODO: add support for zero buffers
                             NumStaticReceivers ? NumStaticReceivers : 1> TransferListenerType;

    // We need to break the inheritance chain here to implement lazy initialization
    class TransferForwarder : public TransferListenerType
    {
        SelfType& obj_;

        void handleIncomingTransfer(IncomingTransfer& transfer)
        {
            obj_.handleIncomingTransfer(transfer);
        }

    public:
        TransferForwarder(SelfType& obj, const DataTypeDescriptor& data_type, IAllocator& allocator)
        : TransferListenerType(data_type, allocator)
        , obj_(obj)
        { }
    };

    struct ReceivedDataStructureSpec : public ReceivedDataStructure<DataType>
    {
        using ReceivedDataStructure<DataType>::setTransfer;
    };

    Scheduler& scheduler_;
    IAllocator& allocator_;
    Callback callback_;
    LazyConstructor<TransferForwarder> forwarder_;
    ReceivedDataStructureSpec message_;
    uint32_t failure_count_;

    bool checkInit()
    {
        if (forwarder_)
            return true;

        GlobalDataTypeRegistry::instance().freeze();

        const DataTypeDescriptor* const descr =
            GlobalDataTypeRegistry::instance().find(DataTypeKindMessage, DataType::getDataTypeFullName());
        if (!descr)
        {
            UAVCAN_TRACE("Subscriber", "Type [%s] is not registered", DataType::getDataTypeFullName());
            return false;
        }
        forwarder_.template construct<SelfType&, const DataTypeDescriptor&, IAllocator&>(*this, *descr, allocator_);
        return true;
    }

    void handleIncomingTransfer(IncomingTransfer& transfer)
    {
        assert(transfer.getTransferType() == TransferTypeMessageBroadcast ||
               transfer.getTransferType() == TransferTypeMessageUnicast);

        {
            BitStream bitstream(transfer);
            ScalarCodec codec(bitstream);

            const int decode_res = DataType::decode(message_, codec);
            // We don't need the data anymore, the memory can be reused from the callback:
            transfer.release();
            if (decode_res <= 0)
            {
                UAVCAN_TRACE("Subscriber", "Unable to decode the message [%i] [%s]",
                    decode_res, DataType::getDataTypeFullName());
                failure_count_++;
                return;
            }
        }

        message_.setTransfer(&transfer);
        if (try_implicit_cast<bool>(callback_, true))
            callback_(message_);  // Callback can accept non-const message reference and mutate it, that's OK
        else
            assert(0);
    }

public:
    Subscriber(Scheduler& scheduler, IAllocator& allocator)
    : scheduler_(scheduler)
    , allocator_(allocator)
    , callback_()
    , failure_count_(0)
    {
        StaticAssert<DataTypeKind(DataType::DataTypeKind) == DataTypeKindMessage>::check();
    }

    virtual ~Subscriber() { stop(); }

    int start(Callback callback)
    {
        stop();

        if (!try_implicit_cast<bool>(callback, true))
        {
            UAVCAN_TRACE("Subscriber", "Invalid callback");
            return -1;
        }
        callback_ = callback;

        if (!checkInit())
        {
            UAVCAN_TRACE("Subscriber", "Initialization failure [%s]", DataType::getDataTypeFullName());
            return -1;
        }

        if (!scheduler_.getDispatcher().registerMessageListener(forwarder_))
        {
            UAVCAN_TRACE("Subscriber", "Failed to register message listener [%s]", DataType::getDataTypeFullName());
            return -1;
        }
        return 1;
    }

    void stop()
    {
        if (forwarder_)
            scheduler_.getDispatcher().unregisterMessageListener(forwarder_);
    }

    Scheduler& getScheduler() const { return scheduler_; }

    uint32_t getFailureCount() const { return failure_count_; }
};

}
