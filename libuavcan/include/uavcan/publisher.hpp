/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/internal/node/generic_publisher.hpp>

namespace uavcan
{

template <typename DataType_>
class Publisher : protected GenericPublisher<DataType_, DataType_>
{
    typedef GenericPublisher<DataType_, DataType_> BaseType;

public:
    typedef DataType_ DataType;

    Publisher(Scheduler& scheduler, IMarshalBufferProvider& buffer_provider,
              MonotonicDuration tx_timeout = BaseType::getDefaultTxTimeout(),
              MonotonicDuration max_transfer_interval = TransferSender::getDefaultMaxTransferInterval())
    : BaseType(scheduler, buffer_provider, max_transfer_interval)
    {
        BaseType::setTxTimeout(tx_timeout);
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

    using BaseType::getDefaultTxTimeout;
    using BaseType::getMinTxTimeout;
    using BaseType::getTxTimeout;
    using BaseType::setTxTimeout;
    using BaseType::getScheduler;
};

}
