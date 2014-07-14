/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/node/publisher.hpp>
#include <uavcan/node/timer.hpp>
#include <uavcan/protocol/Panic.hpp>

namespace uavcan
{
/**
 * Helper for broadcasting the message uavcan.protocol.Panic.
 */
class UAVCAN_EXPORT PanicBroadcaster : private TimerBase
{
    Publisher<protocol::Panic> pub_;
    protocol::Panic msg_;

    void publishOnce();

    virtual void handleTimerEvent(const TimerEvent&);

public:
    explicit PanicBroadcaster(INode& node)
        : TimerBase(node)
        , pub_(node)
    {
        pub_.setTxTimeout(MonotonicDuration::fromMSec(protocol::Panic::BROADCASTING_INTERVAL_MS - 10));
    }

    /**
     * Begin broadcasting at the standard interval.
     * This method does not block and can't fail.
     */
    void panic(const char* short_reason);

    /**
     * Stop broadcasting immediately.
     */
    void dontPanic();    // Where's my towel

    bool isPanicking() const;

    const typename protocol::Panic::FieldTypes::reason_text& getReason() const;
};

}
