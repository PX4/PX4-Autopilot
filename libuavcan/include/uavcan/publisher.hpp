/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/scheduler.hpp>
#include <uavcan/data_type.hpp>
#include <uavcan/marshal_buffer.hpp>
#include <uavcan/global_data_type_registry.hpp>
#include <uavcan/internal/debug.hpp>
#include <uavcan/internal/lazy_constructor.hpp>
#include <uavcan/internal/transport/transfer_sender.hpp>
#include <uavcan/internal/marshal/scalar_codec.hpp>
#include <uavcan/internal/marshal/types.hpp>

namespace uavcan
{

template <typename DataType_>
class Publisher
{
public:
    typedef DataType_ DataType;

private:
    enum { MinTxTimeoutUsec = 200 };

    const uint64_t max_transfer_interval_;   // TODO: memory usage can be reduced
    uint64_t tx_timeout_;
    Scheduler& scheduler_;
    IMarshalBufferProvider& buffer_provider_;
    LazyConstructor<TransferSender> sender_;

    bool checkInit()
    {
        if (sender_)
            return true;

        GlobalDataTypeRegistry::instance().freeze();

        const DataTypeDescriptor* const descr =
            GlobalDataTypeRegistry::instance().find(DataTypeKindMessage, DataType::getDataTypeFullName());
        if (!descr)
        {
            UAVCAN_TRACE("Publisher", "Type [%s] is not registered", DataType::getDataTypeFullName());
            return false;
        }
        sender_.construct<Dispatcher&, const DataTypeDescriptor&, CanTxQueue::Qos, uint64_t>
            (scheduler_.getDispatcher(), *descr, CanTxQueue::Volatile, max_transfer_interval_);
        return true;
    }

    uint64_t getTxDeadline() const { return scheduler_.getMonotonicTimestamp() + tx_timeout_; }

    IMarshalBuffer* getBuffer()
    {
        return buffer_provider_.getBuffer(BitLenToByteLen<DataType::MaxBitLen>::Result);
    }

    int genericSend(const DataType& message, TransferType transfer_type, NodeID dst_node_id,
                    uint64_t monotonic_blocking_deadline)
    {
        if (!checkInit())
            return -1;

        IMarshalBuffer* const buf = getBuffer();
        if (!buf)
            return -1;

        BitStream bitstream(*buf);
        ScalarCodec codec(bitstream);
        const int encode_res = DataType::encode(message, codec);
        if (encode_res <= 0)
        {
            assert(0);   // Impossible, internal error
            return -1;
        }

        return sender_->send(buf->getDataPtr(), buf->getDataLength(), getTxDeadline(),
                             monotonic_blocking_deadline, transfer_type, dst_node_id);
    }

public:
    Publisher(Scheduler& scheduler, IMarshalBufferProvider& buffer_provider, uint64_t tx_timeout_usec,
              uint64_t max_transfer_interval = TransferSender::DefaultMaxTransferInterval)
    : max_transfer_interval_(max_transfer_interval)
    , tx_timeout_(tx_timeout_usec)
    , scheduler_(scheduler)
    , buffer_provider_(buffer_provider)
    {
        setTxTimeout(tx_timeout_usec);
        StaticAssert<DataTypeKind(DataType::DataTypeKind) == DataTypeKindMessage>::check();
    }

    int broadcast(const DataType& message, uint64_t monotonic_blocking_deadline = 0)
    {
        return genericSend(message, TransferTypeMessageBroadcast, NodeID::Broadcast, monotonic_blocking_deadline);
    }

    int unicast(const DataType& message, NodeID dst_node_id, uint64_t monotonic_blocking_deadline = 0)
    {
        if (!dst_node_id.isUnicast())
        {
            assert(0);
            return -1;
        }
        return genericSend(message, TransferTypeMessageUnicast, dst_node_id, monotonic_blocking_deadline);
    }

    uint64_t getTxTimeout() const { return tx_timeout_; }
    void setTxTimeout(uint64_t usec)
    {
        tx_timeout_ = std::max(usec, uint64_t(MinTxTimeoutUsec));
    }

    Scheduler& getScheduler() const { return scheduler_; }
};

}
