/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/node/generic_publisher.hpp>

namespace uavcan
{

bool GenericPublisherBase::isInited() const
{
    return bool(sender_);
}

int GenericPublisherBase::doInit(DataTypeKind dtkind, const char* dtname, CanTxQueue::Qos qos)
{
    if (isInited())
    {
        return 0;
    }

    GlobalDataTypeRegistry::instance().freeze();

    const DataTypeDescriptor* const descr = GlobalDataTypeRegistry::instance().find(dtkind, dtname);
    if (!descr)
    {
        UAVCAN_TRACE("GenericPublisher", "Type [%s] is not registered", dtname);
        return -ErrUnknownDataType;
    }
    sender_.construct<Dispatcher&, const DataTypeDescriptor&, CanTxQueue::Qos, MonotonicDuration>
        (node_.getDispatcher(), *descr, qos, max_transfer_interval_);
    return 0;
}

MonotonicTime GenericPublisherBase::getTxDeadline() const
{
    return node_.getMonotonicTime() + tx_timeout_;
}

IMarshalBuffer* GenericPublisherBase::getBuffer(unsigned byte_len)
{
    return node_.getMarshalBufferProvider().getBuffer(byte_len);
}

int GenericPublisherBase::genericPublish(const IMarshalBuffer& buffer, TransferType transfer_type, NodeID dst_node_id,
                                         TransferID* tid, MonotonicTime blocking_deadline)
{
    if (tid)
    {
        return sender_->send(buffer.getDataPtr(), buffer.getDataLength(), getTxDeadline(),
                             blocking_deadline, transfer_type, dst_node_id, *tid);
    }
    else
    {
        return sender_->send(buffer.getDataPtr(), buffer.getDataLength(), getTxDeadline(),
                             blocking_deadline, transfer_type, dst_node_id);
    }
}

TransferSender* GenericPublisherBase::getTransferSender()
{
    return sender_.isConstructed() ? static_cast<TransferSender*>(sender_) : NULL;
}

void GenericPublisherBase::setTxTimeout(MonotonicDuration tx_timeout)
{
    tx_timeout = max(tx_timeout, getMinTxTimeout());
    tx_timeout = min(tx_timeout, getMaxTxTimeout());
    tx_timeout_ = tx_timeout;
}

}
