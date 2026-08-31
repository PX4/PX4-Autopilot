/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/node/generic_subscriber.hpp>

namespace uavcan
{

int GenericSubscriberBase::genericStart(TransferListener* listener,
                                        bool (Dispatcher::*registration_method)(TransferListener*))
{
    if (listener == UAVCAN_NULLPTR)
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

bool GenericSubscriberBase::decodeTransfer(IncomingTransfer& transfer, void* rx_struct,
                                           int (*decode_fn)(void* rx_struct, ScalarCodec& codec))
{
    /*
     * Decoding into the temporary storage
     */
    BitStream bitstream(transfer);
    ScalarCodec codec(bitstream);

    const int decode_res = decode_fn(rx_struct, codec);

    // We don't need the data anymore, the memory can be reused from the callback:
    transfer.release();

    if (decode_res <= 0)
    {
        UAVCAN_TRACE("GenericSubscriber", "Unable to decode the message [%i]", decode_res);
        failure_count_++;
        node_.getDispatcher().getTransferPerfCounter().addError();
        return false;
    }
    return true;
}

void GenericSubscriberBase::stop(TransferListener* listener)
{
    if (listener != UAVCAN_NULLPTR)
    {
        node_.getDispatcher().unregisterMessageListener(listener);
        node_.getDispatcher().unregisterServiceRequestListener(listener);
        node_.getDispatcher().unregisterServiceResponseListener(listener);
    }
}

}
