/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/internal/node/scheduler.hpp>
#include <uavcan/data_type.hpp>
#include <uavcan/internal/node/marshal_buffer.hpp>
#include <uavcan/global_data_type_registry.hpp>
#include <uavcan/util/lazy_constructor.hpp>
#include <uavcan/internal/debug.hpp>
#include <uavcan/internal/transport/transfer_sender.hpp>
#include <uavcan/internal/marshal/scalar_codec.hpp>
#include <uavcan/internal/marshal/types.hpp>

namespace uavcan
{

/**
 * Generic publisher, suitable for messages and services.
 * DataSpec - data type specification class
 * DataStruct - instantiable class
 */
template <typename DataSpec, typename DataStruct>
class GenericPublisher
{
public:
    enum { DefaultTxTimeoutUsec = 2500 };    // 2500 ms --> 400Hz max
    enum { MinTxTimeoutUsec = 200 };

private:
    enum { Qos = (DataTypeKind(DataSpec::DataTypeKind) == DataTypeKindMessage) ?
                  CanTxQueue::Volatile : CanTxQueue::Persistent };

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
            GlobalDataTypeRegistry::instance().find(DataTypeKind(DataSpec::DataTypeKind),
                                                    DataSpec::getDataTypeFullName());
        if (!descr)
        {
            UAVCAN_TRACE("GenericPublisher", "Type [%s] is not registered", DataSpec::getDataTypeFullName());
            return false;
        }
        sender_.template construct<Dispatcher&, const DataTypeDescriptor&, CanTxQueue::Qos, uint64_t>
            (scheduler_.getDispatcher(), *descr, CanTxQueue::Qos(Qos), max_transfer_interval_);
        return true;
    }

    uint64_t getTxDeadline() const { return scheduler_.getMonotonicTimestamp() + tx_timeout_; }

    IMarshalBuffer* getBuffer()
    {
        return buffer_provider_.getBuffer(BitLenToByteLen<DataStruct::MaxBitLen>::Result);
    }

    int genericPublish(const DataStruct& message, TransferType transfer_type, NodeID dst_node_id,
                       TransferID* tid, uint64_t monotonic_blocking_deadline)
    {
        if (!checkInit())
            return -1;

        IMarshalBuffer* const buf = getBuffer();
        if (!buf)
            return -1;

        {
            BitStream bitstream(*buf);
            ScalarCodec codec(bitstream);
            const int encode_res = DataStruct::encode(message, codec);
            if (encode_res <= 0)
            {
                assert(0);   // Impossible, internal error
                return -1;
            }
        }
        if (tid)
        {
            return sender_->send(buf->getDataPtr(), buf->getDataLength(), getTxDeadline(),
                                 monotonic_blocking_deadline, transfer_type, dst_node_id, *tid);
        }
        else
        {
            return sender_->send(buf->getDataPtr(), buf->getDataLength(), getTxDeadline(),
                                 monotonic_blocking_deadline, transfer_type, dst_node_id);
        }
    }

protected:
    GenericPublisher(Scheduler& scheduler, IMarshalBufferProvider& buffer_provider,
                     uint64_t max_transfer_interval = TransferSender::DefaultMaxTransferInterval)
    : max_transfer_interval_(max_transfer_interval)
    , tx_timeout_(DefaultTxTimeoutUsec)
    , scheduler_(scheduler)
    , buffer_provider_(buffer_provider)
    { }

    ~GenericPublisher() { }

    int publish(const DataStruct& message, TransferType transfer_type, NodeID dst_node_id,
                uint64_t monotonic_blocking_deadline = 0)
    {
        return genericPublish(message, transfer_type, dst_node_id, NULL, monotonic_blocking_deadline);
    }

    int publish(const DataStruct& message, TransferType transfer_type, NodeID dst_node_id, TransferID tid,
                uint64_t monotonic_blocking_deadline = 0)
    {
        return genericPublish(message, transfer_type, dst_node_id, &tid, monotonic_blocking_deadline);
    }

public:
    uint64_t getTxTimeout() const { return tx_timeout_; }
    void setTxTimeout(uint64_t usec)
    {
        tx_timeout_ = std::max(usec, uint64_t(MinTxTimeoutUsec));
    }

    Scheduler& getScheduler() const { return scheduler_; }
};

}
