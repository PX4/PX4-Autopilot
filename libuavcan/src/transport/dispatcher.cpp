/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cassert>
#include <uavcan/transport/dispatcher.hpp>
#include <uavcan/debug.hpp>

namespace uavcan
{
/*
 * LoopbackFrameListenerBase
 */
void LoopbackFrameListenerBase::startListening()
{
    dispatcher_.getLoopbackFrameListenerRegistry().add(this);
}

void LoopbackFrameListenerBase::stopListening()
{
    dispatcher_.getLoopbackFrameListenerRegistry().remove(this);
}

bool LoopbackFrameListenerBase::isListening() const
{
    return dispatcher_.getLoopbackFrameListenerRegistry().doesExist(this);
}

/*
 * LoopbackFrameListenerRegistry
 */
void LoopbackFrameListenerRegistry::add(LoopbackFrameListenerBase* listener)
{
    assert(listener);
    listeners_.insert(listener);
}

void LoopbackFrameListenerRegistry::remove(LoopbackFrameListenerBase* listener)
{
    assert(listener);
    listeners_.remove(listener);
}

bool LoopbackFrameListenerRegistry::doesExist(const LoopbackFrameListenerBase* listener) const
{
    assert(listener);
    const LoopbackFrameListenerBase* p = listeners_.get();
    while (p)
    {
        if (p == listener)
            return true;
        p = p->getNextListNode();
    }
    return false;
}

void LoopbackFrameListenerRegistry::invokeListeners(RxFrame& frame)
{
    LoopbackFrameListenerBase* p = listeners_.get();
    while (p)
    {
        LoopbackFrameListenerBase* const next = p->getNextListNode();
        p->handleLoopbackFrame(frame);
        p = next;
    }
}

/*
 * Dispatcher::ListenerRegister
 */
bool Dispatcher::ListenerRegister::add(TransferListenerBase* listener, Mode mode)
{
    if (mode == UniqueListener)
    {
        TransferListenerBase* p = list_.get();
        while (p)
        {
            if (p->getDataTypeDescriptor().getID() == listener->getDataTypeDescriptor().getID())
                return false;
            p = p->getNextListNode();
        }
    }
    // Objective is to arrange entries by Data Type ID in ascending order from root.
    list_.insertBefore(listener, DataTypeIDInsertionComparator(listener->getDataTypeDescriptor().getID()));
    return true;
}

void Dispatcher::ListenerRegister::remove(TransferListenerBase* listener)
{
    list_.remove(listener);
}

bool Dispatcher::ListenerRegister::exists(DataTypeID dtid) const
{
    TransferListenerBase* p = list_.get();
    while (p)
    {
        if (p->getDataTypeDescriptor().getID() == dtid)
            return true;
        p = p->getNextListNode();
    }
    return false;
}

void Dispatcher::ListenerRegister::cleanup(MonotonicTime ts)
{
    TransferListenerBase* p = list_.get();
    while (p)
    {
        p->cleanup(ts);
        p = p->getNextListNode();
    }
}

void Dispatcher::ListenerRegister::handleFrame(const RxFrame& frame)
{
    TransferListenerBase* p = list_.get();
    while (p)
    {
        if (p->getDataTypeDescriptor().getID() == frame.getDataTypeID())
            p->handleFrame(frame);
        else if (p->getDataTypeDescriptor().getID() < frame.getDataTypeID())  // Listeners are ordered by data type id!
            break;
        p = p->getNextListNode();
    }
}

/*
 * Dispatcher
 */
void Dispatcher::handleFrame(const CanRxFrame& can_frame)
{
    RxFrame frame;
    if (!frame.parse(can_frame))
    {
        UAVCAN_TRACE("Dispatcher", "Invalid CAN frame received: %s", can_frame.toString().c_str());
        return;
    }

    if ((frame.getDstNodeID() != NodeID::Broadcast) &&
        (frame.getDstNodeID() != getNodeID()))
    {
        return;
    }

    switch (frame.getTransferType())
    {
    case TransferTypeMessageBroadcast:
    case TransferTypeMessageUnicast:
        lmsg_.handleFrame(frame);
        break;

    case TransferTypeServiceRequest:
        lsrv_req_.handleFrame(frame);
        break;

    case TransferTypeServiceResponse:
        lsrv_resp_.handleFrame(frame);
        break;

    default:
        assert(0);
    }
}

void Dispatcher::handleLoopbackFrame(const CanRxFrame& can_frame)
{
    RxFrame frame;
    if (!frame.parse(can_frame))
    {
        UAVCAN_TRACE("Dispatcher", "Invalid loopback CAN frame: %s", can_frame.toString().c_str());
        assert(0);  // No way!
        return;
    }
    assert(frame.getSrcNodeID() == getNodeID());
    loopback_listeners_.invokeListeners(frame);
}

int Dispatcher::spin(MonotonicTime deadline)
{
    int num_frames_processed = 0;
    do
    {
        CanIOFlags flags = 0;
        CanRxFrame frame;
        const int res = canio_.receive(frame, deadline, flags);
        if (res < 0)
            return res;
        if (res > 0)
        {
            if (flags & CanIOFlagLoopback)
            {
                handleLoopbackFrame(frame);
            }
            else
            {
                num_frames_processed++;
                handleFrame(frame);
            }
        }
    }
    while (sysclock_.getMonotonic() < deadline);

    return num_frames_processed;
}

int Dispatcher::send(const Frame& frame, MonotonicTime tx_deadline, MonotonicTime blocking_deadline,
                     CanTxQueue::Qos qos, CanIOFlags flags, uint8_t iface_mask)
{
    if (frame.getSrcNodeID() != getNodeID())
    {
        assert(0);
        return -1;
    }

    CanFrame can_frame;
    if (!frame.compile(can_frame))
    {
        UAVCAN_TRACE("Dispatcher", "Unable to send: frame is malformed: %s", frame.toString().c_str());
        assert(0);
        return -1;
    }
    return canio_.send(can_frame, tx_deadline, blocking_deadline, iface_mask, qos, flags);
}

void Dispatcher::cleanup(MonotonicTime ts)
{
    outgoing_transfer_reg_.cleanup(ts);
    lmsg_.cleanup(ts);
    lsrv_req_.cleanup(ts);
    lsrv_resp_.cleanup(ts);
}

bool Dispatcher::registerMessageListener(TransferListenerBase* listener)
{
    if (listener->getDataTypeDescriptor().getKind() != DataTypeKindMessage)
    {
        assert(0);
        return false;
    }
    return lmsg_.add(listener, ListenerRegister::ManyListeners);       // Multiple subscribers are OK
}

bool Dispatcher::registerServiceRequestListener(TransferListenerBase* listener)
{
    if (listener->getDataTypeDescriptor().getKind() != DataTypeKindService)
    {
        assert(0);
        return false;
    }
    return lsrv_req_.add(listener, ListenerRegister::UniqueListener);  // Only one server per data type
}

bool Dispatcher::registerServiceResponseListener(TransferListenerBase* listener)
{
    if (listener->getDataTypeDescriptor().getKind() != DataTypeKindService)
    {
        assert(0);
        return false;
    }
    return lsrv_resp_.add(listener, ListenerRegister::ManyListeners);  // Multiple callers may call same srv
}

void Dispatcher::unregisterMessageListener(TransferListenerBase* listener)
{
    lmsg_.remove(listener);
}

void Dispatcher::unregisterServiceRequestListener(TransferListenerBase* listener)
{
    lsrv_req_.remove(listener);
}

void Dispatcher::unregisterServiceResponseListener(TransferListenerBase* listener)
{
    lsrv_resp_.remove(listener);
}

bool Dispatcher::hasSubscriber(DataTypeID dtid) const
{
    return lmsg_.exists(dtid);
}

bool Dispatcher::hasPublisher(DataTypeID dtid) const
{
    return outgoing_transfer_reg_.exists(dtid, TransferTypeMessageBroadcast)
        || outgoing_transfer_reg_.exists(dtid, TransferTypeMessageUnicast);
}

bool Dispatcher::hasServer(DataTypeID dtid) const
{
    return lsrv_req_.exists(dtid);
}

bool Dispatcher::setNodeID(NodeID nid)
{
    if (nid.isUnicast() && !self_node_id_.isValid())
    {
        self_node_id_ = nid;
        return true;
    }
    return false;
}

}
