/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <uavcan/debug.hpp>
#include <uavcan/node/subscriber.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/protocol/Panic.hpp>

namespace uavcan
{
/**
 * Callback prototype:
 *   void (const ReceivedDataStructure<protocol::Panic>&)
 * Or:
 *   void (const protocol::Panic&)
 * The listener can be stopped from the callback.
 */
template <typename Callback = void (*)(const ReceivedDataStructure<protocol::Panic>&)>
class PanicListener : Noncopyable
{
    typedef MethodBinder<PanicListener*, void (PanicListener::*)(const ReceivedDataStructure<protocol::Panic>&)>
        PanicMsgCallback;

    Subscriber<protocol::Panic, PanicMsgCallback> sub_;
    MonotonicTime prev_msg_timestamp_;
    Callback callback_;
    uint8_t num_subsequent_msgs_;

    void invokeCallback(const ReceivedDataStructure<protocol::Panic>& msg)
    {
        if (try_implicit_cast<bool>(callback_, true))
        {
            callback_(msg);
        }
        else
        {
            assert(0);       // This is a logic error because normally we shouldn't start with an invalid callback
            sub_.getNode().registerInternalFailure("PanicListener invalid callback");
        }
    }

    void handleMsg(const ReceivedDataStructure<protocol::Panic>& msg)
    {
        UAVCAN_TRACE("PanicListener", "Received panic from snid=%i reason=%s",
                     int(msg.getSrcNodeID().get()), msg.reason_text.c_str());
        if (prev_msg_timestamp_.isZero())
        {
            num_subsequent_msgs_ = 1;
            prev_msg_timestamp_ = msg.getMonotonicTimestamp();
        }
        else
        {
            const MonotonicDuration diff = msg.getMonotonicTimestamp() - prev_msg_timestamp_;
            assert(diff.isPositive() || diff.isZero());
            if (diff.toMSec() > protocol::Panic::MAX_INTERVAL_MS)
            {
                num_subsequent_msgs_ = 1;
            }
            else
            {
                num_subsequent_msgs_++;
            }
            prev_msg_timestamp_ = msg.getMonotonicTimestamp();
            if (num_subsequent_msgs_ >= protocol::Panic::MIN_MESSAGES)
            {
                num_subsequent_msgs_ = protocol::Panic::MIN_MESSAGES;
                invokeCallback(msg);                      // The application can stop us from the callback
            }
        }
    }

public:
    explicit PanicListener(INode& node)
        : sub_(node)
        , callback_()
        , num_subsequent_msgs_(0)
    { }

    int start(const Callback& callback)
    {
        stop();
        if (!try_implicit_cast<bool>(callback, true))
        {
            UAVCAN_TRACE("PanicListener", "Invalid callback");
            return -1;
        }
        callback_ = callback;
        return sub_.start(PanicMsgCallback(this, &PanicListener::handleMsg));
    }

    void stop()
    {
        sub_.stop();
        num_subsequent_msgs_ = 0;
        prev_msg_timestamp_ = MonotonicTime();
    }
};

}
