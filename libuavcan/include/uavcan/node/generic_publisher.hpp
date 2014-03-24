/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/node/abstract_node.hpp>
#include <uavcan/data_type.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/util/lazy_constructor.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/transport/transfer_sender.hpp>
#include <uavcan/marshal/scalar_codec.hpp>
#include <uavcan/marshal/types.hpp>

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
    enum
    {
        Qos = (DataTypeKind(DataSpec::DataTypeKind) == DataTypeKindMessage) ?
              CanTxQueue::Volatile : CanTxQueue::Persistent
    };

    const MonotonicDuration max_transfer_interval_;   // TODO: memory usage can be reduced
    MonotonicDuration tx_timeout_;
    INode& node_;
    LazyConstructor<TransferSender> sender_;

    bool checkInit()
    {
        if (sender_)
        {
            return true;
        }

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
            (node_.getDispatcher(), *descr, CanTxQueue::Qos(Qos), max_transfer_interval_);
        return true;
    }

    MonotonicTime getTxDeadline() const { return node_.getMonotonicTime() + tx_timeout_; }

    IMarshalBuffer* getBuffer()
    {
        return node_.getMarshalBufferProvider().getBuffer(BitLenToByteLen<DataStruct::MaxBitLen>::Result);
    }

    int genericPublish(const DataStruct& message, TransferType transfer_type, NodeID dst_node_id,
                       TransferID* tid, MonotonicTime blocking_deadline)
    {
        if (!checkInit())
        {
            return -1;
        }

        IMarshalBuffer* const buf = getBuffer();
        if (!buf)
        {
            return -1;
        }

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
    GenericPublisher(INode& node, MonotonicDuration tx_timeout,
                     MonotonicDuration max_transfer_interval = TransferSender::getDefaultMaxTransferInterval())
        : max_transfer_interval_(max_transfer_interval)
        , tx_timeout_(tx_timeout)
        , node_(node)
    {
        setTxTimeout(tx_timeout);
#if UAVCAN_DEBUG
        assert(getTxTimeout() == tx_timeout);  // Making sure default values are OK
#endif
    }

    ~GenericPublisher() { }

    int init()
    {
        return checkInit() ? 0 : -1;
    }

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

    TransferSender* getTransferSender()
    {
        checkInit();
        return sender_.isConstructed() ? static_cast<TransferSender*>(sender_) : NULL;
    }

    static MonotonicDuration getMinTxTimeout() { return MonotonicDuration::fromUSec(200); }
    static MonotonicDuration getMaxTxTimeout() { return MonotonicDuration::fromMSec(60000); }

    MonotonicDuration getTxTimeout() const { return tx_timeout_; }
    void setTxTimeout(MonotonicDuration tx_timeout)
    {
        tx_timeout = std::max(tx_timeout, getMinTxTimeout());
        tx_timeout = std::min(tx_timeout, getMaxTxTimeout());
        tx_timeout_ = tx_timeout;
    }

    INode& getNode() const { return node_; }
};

}
