/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <uavcan/internal/node/generic_subscriber.hpp>

namespace uavcan
{

template <typename DataType_,
          typename Callback = void(*)(const ReceivedDataStructure<DataType_>&),
          unsigned int NumStaticReceivers = 2,
          unsigned int NumStaticBufs = 1>
class Subscriber : public GenericSubscriber<DataType_, DataType_,
                                            typename TransferListenerInstantiationHelper<DataType_,
                                                                                         NumStaticReceivers,
                                                                                         NumStaticBufs>::Type>
{
    typedef typename TransferListenerInstantiationHelper<DataType_, NumStaticReceivers, NumStaticBufs>::Type
        TransferListenerType;
    typedef GenericSubscriber<DataType_, DataType_, TransferListenerType> BaseType;

    Callback callback_;

    void handleReceivedDataStruct(ReceivedDataStructure<DataType_>& msg)
    {
        if (try_implicit_cast<bool>(callback_, true))
            callback_(msg);
        else
            handleFatalError("Invalid subscriber callback");
    }

public:
    typedef DataType_ DataType;

    Subscriber(Scheduler& scheduler, IAllocator& allocator)
    : BaseType(scheduler, allocator)
    , callback_()
    {
        StaticAssert<DataTypeKind(DataType::DataTypeKind) == DataTypeKindMessage>::check();
    }

    int start(const Callback& callback)
    {
        stop();

        if (!try_implicit_cast<bool>(callback, true))
        {
            UAVCAN_TRACE("Subscriber", "Invalid callback");
            return -1;
        }
        callback_ = callback;

        return BaseType::startAsMessageListener();
    }

    using BaseType::stop;
    using BaseType::getFailureCount;
};

}
