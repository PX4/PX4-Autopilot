/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/internal/node/generic_publisher.hpp>

namespace uavcan
{

template <typename DataType_>
class Publisher : public GenericPublisher<DataType_, DataType_>
{
    typedef GenericPublisher<DataType_, DataType_> BaseType;

public:
    typedef DataType_ DataType;

    Publisher(Scheduler& scheduler, IMarshalBufferProvider& buffer_provider,
              uint64_t tx_timeout_usec = BaseType::DefaultTxTimeoutUsec,
              uint64_t max_transfer_interval = TransferSender::DefaultMaxTransferInterval)
    : BaseType(scheduler, buffer_provider, max_transfer_interval)
    {
        BaseType::setTxTimeout(tx_timeout_usec);
        StaticAssert<DataTypeKind(DataType::DataTypeKind) == DataTypeKindMessage>::check();
    }

    int broadcast(const DataType& message)
    {
        return publish(message, TransferTypeMessageBroadcast, NodeID::Broadcast);
    }

    int unicast(const DataType& message, NodeID dst_node_id)
    {
        if (!dst_node_id.isUnicast())
        {
            assert(0);
            return -1;
        }
        return publish(message, TransferTypeMessageUnicast, dst_node_id);
    }
};

}
