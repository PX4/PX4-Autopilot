/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/error.hpp>
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

class GenericPublisherBase : Noncopyable
{
    const MonotonicDuration max_transfer_interval_;   // TODO: memory usage can be reduced
    MonotonicDuration tx_timeout_;
    INode& node_;
    LazyConstructor<TransferSender> sender_;

protected:
    GenericPublisherBase(INode& node, MonotonicDuration tx_timeout,
                         MonotonicDuration max_transfer_interval)
        : max_transfer_interval_(max_transfer_interval)
        , tx_timeout_(tx_timeout)
        , node_(node)
    {
        setTxTimeout(tx_timeout);
#if UAVCAN_DEBUG
        UAVCAN_ASSERT(getTxTimeout() == tx_timeout);  // Making sure default values are OK
#endif
    }

    ~GenericPublisherBase() { }

    bool isInited() const;

    int doInit(DataTypeKind dtkind, const char* dtname, CanTxQueue::Qos qos);

    MonotonicTime getTxDeadline() const;

    IMarshalBuffer* getBuffer(unsigned byte_len);

    int genericPublish(const IMarshalBuffer& buffer, TransferType transfer_type, NodeID dst_node_id,
                       TransferID* tid, MonotonicTime blocking_deadline);

    TransferSender* getTransferSender();

public:
    static MonotonicDuration getMinTxTimeout() { return MonotonicDuration::fromUSec(200); }
    static MonotonicDuration getMaxTxTimeout() { return MonotonicDuration::fromMSec(60000); }

    MonotonicDuration getTxTimeout() const { return tx_timeout_; }
    void setTxTimeout(MonotonicDuration tx_timeout);

    INode& getNode() const { return node_; }
};

/**
 * Generic publisher, suitable for messages and services.
 * DataSpec - data type specification class
 * DataStruct - instantiable class
 */
template <typename DataSpec, typename DataStruct>
class UAVCAN_EXPORT GenericPublisher : public GenericPublisherBase
{
    enum
    {
        Qos = (DataTypeKind(DataSpec::DataTypeKind) == DataTypeKindMessage) ?
              CanTxQueue::Volatile : CanTxQueue::Persistent
    };

    int checkInit();

    int doEncode(const DataStruct& message, IMarshalBuffer& buffer) const;

    int genericPublish(const DataStruct& message, TransferType transfer_type, NodeID dst_node_id,
                       TransferID* tid, MonotonicTime blocking_deadline);

public:
    /**
     * @param max_transfer_interval     Maximum expected time interval between subsequent publications. Leave default.
     */
    GenericPublisher(INode& node, MonotonicDuration tx_timeout,
                     MonotonicDuration max_transfer_interval = TransferSender::getDefaultMaxTransferInterval())
        : GenericPublisherBase(node, tx_timeout, max_transfer_interval)
    { }

    ~GenericPublisher() { }

    /**
     * Init method can be called prior first publication, but it's not necessary
     * because the publisher can be automatically initialized ad-hoc.
     */
    int init()
    {
        return checkInit();
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
        (void)checkInit();
        return GenericPublisherBase::getTransferSender();
    }
};

// ----------------------------------------------------------------------------

template <typename DataSpec, typename DataStruct>
int GenericPublisher<DataSpec, DataStruct>::checkInit()
{
    if (isInited())
    {
        return 0;
    }
    return doInit(DataTypeKind(DataSpec::DataTypeKind), DataSpec::getDataTypeFullName(), CanTxQueue::Qos(Qos));
}

template <typename DataSpec, typename DataStruct>
int GenericPublisher<DataSpec, DataStruct>::doEncode(const DataStruct& message, IMarshalBuffer& buffer) const
{
    BitStream bitstream(buffer);
    ScalarCodec codec(bitstream);
    const int encode_res = DataStruct::encode(message, codec);
    if (encode_res <= 0)
    {
        UAVCAN_ASSERT(0);   // Impossible, internal error
        return -ErrInvalidMarshalData;
    }
    return encode_res;
}

template <typename DataSpec, typename DataStruct>
int GenericPublisher<DataSpec, DataStruct>::genericPublish(const DataStruct& message, TransferType transfer_type,
                                                           NodeID dst_node_id, TransferID* tid,
                                                           MonotonicTime blocking_deadline)
{
    const int res = checkInit();
    if (res < 0)
    {
        return res;
    }
    IMarshalBuffer* const buf = getBuffer(BitLenToByteLen<DataStruct::MaxBitLen>::Result);
    if (!buf)
    {
        return -ErrMemory;
    }
    const int encode_res = doEncode(message, *buf);
    if (encode_res < 0)
    {
        return encode_res;
    }
    return GenericPublisherBase::genericPublish(*buf, transfer_type, dst_node_id, tid, blocking_deadline);
}

}
