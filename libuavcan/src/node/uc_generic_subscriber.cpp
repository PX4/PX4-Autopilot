/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/node/generic_subscriber.hpp>

namespace uavcan
{

int GenericSubscriberBase::genericStart(TransferListenerBase* listener,
                                        bool (Dispatcher::*registration_method)(TransferListenerBase*))
{
    if (listener == NULL)
    {
        UAVCAN_ASSERT(0);
        return -ErrLogic;
    }
    stop(listener);
    if (!(node_.getDispatcher().*registration_method)(listener))
    {
        UAVCAN_TRACE("GenericSubscriber", "Failed to register transfer listener");
        return -ErrInvalidTransferListener;
    }
    return 0;
}

void GenericSubscriberBase::stop(TransferListenerBase* listener)
{
    if (listener != NULL)
    {
        node_.getDispatcher().unregisterMessageListener(listener);
        node_.getDispatcher().unregisterServiceRequestListener(listener);
        node_.getDispatcher().unregisterServiceResponseListener(listener);
    }
}

}
