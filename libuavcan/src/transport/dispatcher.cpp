/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cassert>
#include <uavcan/internal/transport/dispatcher.hpp>
#include <uavcan/internal/debug.hpp>

namespace uavcan
{
/*
 * Dispatcher::ListenerRegister
 */
bool Dispatcher::ListenerRegister::add(TransferListenerBase* listener, Mode mode)
{
    if (mode == UNIQUE_LISTENER)
    {
        TransferListenerBase* p = list_.get();
        while (p)
        {
            if (p->getDataTypeDescriptor().id == listener->getDataTypeDescriptor().id)
                return false;
            p = p->getNextListNode();
        }
    }
    // Objective is to arrange entries by Data Type ID in ascending order from root.
    list_.insertBefore(listener, DataTypeIDInsertionComparator(listener->getDataTypeDescriptor().id));
    return true;
}

void Dispatcher::ListenerRegister::remove(TransferListenerBase* listener)
{
    list_.remove(listener);
}

void Dispatcher::ListenerRegister::cleanup(uint64_t ts_monotonic)
{
    TransferListenerBase* p = list_.get();
    while (p)
    {
        p->cleanup(ts_monotonic);
        p = p->getNextListNode();
    }
}

void Dispatcher::ListenerRegister::handleFrame(const RxFrame& frame)
{
    TransferListenerBase* p = list_.get();
    while (p)
    {
        if (p->getDataTypeDescriptor().id == frame.getDataTypeID())
            p->handleFrame(frame);
        else if (p->getDataTypeDescriptor().id < frame.getDataTypeID())  // Listeners are ordered by data type id!
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

    if ((frame.getDstNodeID() != NodeID::BROADCAST) &&
        (frame.getDstNodeID() != getSelfNodeID()))
    {
        return;
    }

    switch (frame.getTransferType())
    {
    case TRANSFER_TYPE_MESSAGE_BROADCAST:
    case TRANSFER_TYPE_MESSAGE_UNICAST:
        lmsg_.handleFrame(frame);
        break;

    case TRANSFER_TYPE_SERVICE_REQUEST:
        lsrv_req_.handleFrame(frame);
        break;

    case TRANSFER_TYPE_SERVICE_RESPONSE:
        lsrv_resp_.handleFrame(frame);
        break;

    default:
        assert(0);
    }
}

int Dispatcher::spin(uint64_t monotonic_deadline)
{
    int num_frames_processed = 0;
    do
    {
        CanRxFrame frame;
        const int res = canio_.receive(frame, monotonic_deadline);
        if (res < 0)
            return res;
        if (res > 0)
        {
            num_frames_processed++;
            handleFrame(frame);
        }
    }
    while (sysclock_.getMonotonicMicroseconds() < monotonic_deadline);

    return num_frames_processed;
}

int Dispatcher::send(const Frame& frame, uint64_t monotonic_tx_deadline, uint64_t monotonic_blocking_deadline,
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

    return canio_.send(can_frame, monotonic_tx_deadline, monotonic_blocking_deadline, iface_mask, qos);
}

void Dispatcher::cleanup(uint64_t ts_monotonic)
{
    outgoing_transfer_reg_.cleanup(ts_monotonic);
    lmsg_.cleanup(ts_monotonic);
    lsrv_req_.cleanup(ts_monotonic);
    lsrv_resp_.cleanup(ts_monotonic);
}

bool Dispatcher::registerMessageListener(TransferListenerBase* listener)
{
    if (listener->getDataTypeDescriptor().kind != DATA_TYPE_KIND_MESSAGE)
    {
        assert(0);
        return false;
    }
    return lmsg_.add(listener, ListenerRegister::MANY_LISTENERS);       // Multiple subscribers are OK
}

bool Dispatcher::registerServiceRequestListener(TransferListenerBase* listener)
{
    if (listener->getDataTypeDescriptor().kind != DATA_TYPE_KIND_SERVICE)
    {
        assert(0);
        return false;
    }
    return lsrv_req_.add(listener, ListenerRegister::UNIQUE_LISTENER);  // Only one server per data type
}

bool Dispatcher::registerServiceResponseListener(TransferListenerBase* listener)
{
    if (listener->getDataTypeDescriptor().kind != DATA_TYPE_KIND_SERVICE)
    {
        assert(0);
        return false;
    }
    return lsrv_resp_.add(listener, ListenerRegister::MANY_LISTENERS);  // Multiple callers may call same srv
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
