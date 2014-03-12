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
    enum { Qos = (DataTypeKind(DataSpec::DataTypeKind) == DataTypeKindMessage) ?
                  CanTxQueue::Volatile : CanTxQueue::Persistent };

    const MonotonicDuration max_transfer_interval_;   // TODO: memory usage can be reduced
    MonotonicDuration tx_timeout_;
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
        sender_.template construct<Dispatcher&, const DataTypeDescriptor&, CanTxQueue::Qos, MonotonicDuration>
            (scheduler_.getDispatcher(), *descr, CanTxQueue::Qos(Qos), max_transfer_interval_);
        return true;
    }

    MonotonicTime getTxDeadline() const { return scheduler_.getMonotonicTimestamp() + tx_timeout_; }

    IMarshalBuffer* getBuffer()
    {
        return buffer_provider_.getBuffer(BitLenToByteLen<DataStruct::MaxBitLen>::Result);
    }

    int genericPublish(const DataStruct& message, TransferType transfer_type, NodeID dst_node_id,
                       TransferID* tid, MonotonicTime blocking_deadline)
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
                                 blocking_deadline, transfer_type, dst_node_id, *tid);
        }
        else
        {
            return sender_->send(buf->getDataPtr(), buf->getDataLength(), getTxDeadline(),
                                 blocking_deadline, transfer_type, dst_node_id);
        }
    }

public:
    GenericPublisher(Scheduler& scheduler, IMarshalBufferProvider& buffer_provider,
                     MonotonicDuration max_transfer_interval = TransferSender::getDefaultMaxTransferInterval())
    : max_transfer_interval_(max_transfer_interval)
    , tx_timeout_(getDefaultTxTimeout())
    , scheduler_(scheduler)
    , buffer_provider_(buffer_provider)
    { }

    ~GenericPublisher() { }

    int publish(const DataStruct& message, TransferType transfer_type, NodeID dst_node_id,
                MonotonicTime blocking_deadline = MonotonicTime())
    {
        return genericPublish(message, transfer_type, dst_node_id, NULL, blocking_deadline);
    }

    int publish(const DataStruct& message, TransferType transfer_type, NodeID dst_node_id, TransferID tid,
                MonotonicTime blocking_deadline = MonotonicTime())
    {
        return genericPublish(message, transfer_type, dst_node_id, &tid, blocking_deadline);
    }

    static MonotonicDuration getDefaultTxTimeout() { return MonotonicDuration::fromUSec(2500); }// 2500ms --> 400Hz max
    static MonotonicDuration getMinTxTimeout() { return MonotonicDuration::fromUSec(200); }

    MonotonicDuration getTxTimeout() const { return tx_timeout_; }
    void setTxTimeout(MonotonicDuration tx_timeout)
    {
        tx_timeout_ = std::max(tx_timeout, getMinTxTimeout());
    }

    Scheduler& getScheduler() const { return scheduler_; }
};

}
