/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cassert>
#include <uavcan/transport/dispatcher.hpp>
#include <uavcan/debug.hpp>

namespace uavcan
{
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
        (frame.getDstNodeID() != getSelfNodeID()))
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

int Dispatcher::spin(MonotonicTime deadline)
{
    int num_frames_processed = 0;
    do
    {
        CanRxFrame frame;
        const int res = canio_.receive(frame, deadline);
        if (res < 0)
            return res;
        if (res > 0)
        {
            num_frames_processed++;
            handleFrame(frame);
        }
    }
    while (sysclock_.getMonotonic() < deadline);

    return num_frames_processed;
}

int Dispatcher::send(const Frame& frame, MonotonicTime tx_deadline, MonotonicTime blocking_deadline,
                     CanTxQueue::Qos qos)
{
    if (frame.getSrcNodeID() != getSelfNodeID())
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
    const int iface_mask = (1 << canio_.getNumIfaces()) - 1;

    return canio_.send(can_frame, tx_deadline, blocking_deadline, iface_mask, qos);
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

}
