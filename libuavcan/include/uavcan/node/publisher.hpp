/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/node/generic_publisher.hpp>

namespace uavcan
{
/**
 * Use this class to publish messages to the bus (broadcast, unicast, or both).
 *
 * @tparam DataType_    Message data type
 */
template <typename DataType_>
class UAVCAN_EXPORT Publisher : protected GenericPublisher<DataType_, DataType_>
{
    typedef GenericPublisher<DataType_, DataType_> BaseType;

public:
    typedef DataType_ DataType; ///< Message data type

    /**
     * @param node          Node instance this publisher will be registered with.
     *
     * @param tx_timeout    If CAN frames of this message are not delivered to the bus
     *                      in this amount of time, they will be discarded. Default value
     *                      is good enough for rather high-frequency, high-priority messages.
     *
     * @param max_transfer_interval     Maximum expected transfer interval. It's absolutely safe
     *                                  to leave default value here. It just defines how soon the
     *                                  Transfer ID  tracking objects associated with this message type
     *                                  will be garbage collected by the library if it's no longer
     *                                  being published.
     */
    explicit Publisher(INode& node, MonotonicDuration tx_timeout = getDefaultTxTimeout(),
                       MonotonicDuration max_transfer_interval = TransferSender::getDefaultMaxTransferInterval())
        : BaseType(node, tx_timeout, max_transfer_interval)
    {
#if UAVCAN_DEBUG
        UAVCAN_ASSERT(getTxTimeout() == tx_timeout);  // Making sure default values are OK
#endif
        StaticAssert<DataTypeKind(DataType::DataTypeKind) == DataTypeKindMessage>::check();
    }

    /**
     * Broadcast the message.
     * Returns negative error code.
     */
    int broadcast(const DataType& message)
    {
        return BaseType::publish(message, TransferTypeMessageBroadcast, NodeID::Broadcast);
    }

    /**
     * Warning: You probably don't want to use this method; it's for advanced use cases like
     * e.g. network time synchronization. Use the overloaded method with fewer arguments instead.
     * This overload allows to explicitly specify Transfer ID.
     * Returns negative error code.
     */
    int broadcast(const DataType& message, TransferID tid)
    {
        return BaseType::publish(message, TransferTypeMessageBroadcast, NodeID::Broadcast, tid);
    }

    /**
     * Unicast the message to the specified destination Node ID.
     * Returns negative error code.
     */
    int unicast(const DataType& message, NodeID dst_node_id)
    {
        if (!dst_node_id.isUnicast())
        {
            UAVCAN_ASSERT(0);
            return -ErrInvalidParam;
        }
        return BaseType::publish(message, TransferTypeMessageUnicast, dst_node_id);
    }

    static MonotonicDuration getDefaultTxTimeout() { return MonotonicDuration::fromMSec(10); }

    /**
     * Init method can be called prior first publication, but it's not necessary
     * because the publisher can be automatically initialized ad-hoc.
     */
    using BaseType::init;

    using BaseType::getTransferSender;
    using BaseType::getMinTxTimeout;
    using BaseType::getMaxTxTimeout;
    using BaseType::getTxTimeout;
    using BaseType::setTxTimeout;
    using BaseType::getNode;
};

}
