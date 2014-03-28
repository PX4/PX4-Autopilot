/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <uavcan/impl_constants.hpp>
#include <uavcan/node/generic_subscriber.hpp>

#if !defined(UAVCAN_CPP_VERSION) || !defined(UAVCAN_CPP11)
# error UAVCAN_CPP_VERSION
#endif

#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
# include <functional>
#endif

namespace uavcan
{

template <typename DataType_,
#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
          typename Callback = std::function<void (const ReceivedDataStructure<DataType_>&)>,
#else
          typename Callback = void (*)(const ReceivedDataStructure<DataType_>&),
#endif
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
        {
            callback_(msg);
        }
        else
        {
            handleFatalError("Invalid subscriber callback");
        }
    }

public:
    typedef DataType_ DataType;

    explicit Subscriber(INode& node)
        : BaseType(node)
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
            return -ErrInvalidParam;
        }
        callback_ = callback;

        return BaseType::startAsMessageListener();
    }

    using BaseType::stop;
    using BaseType::getFailureCount;
};

}
