/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/protocol/panic_broadcaster.hpp>
#include <uavcan/debug.hpp>

namespace uavcan
{

void PanicBroadcaster::publishOnce()
{
    const int res = pub_.broadcast(msg_);
    if (res < 0)
    {
        pub_.getNode().registerInternalFailure("Panic pub failed");
    }
}

void PanicBroadcaster::handleTimerEvent(const TimerEvent&)
{
    publishOnce();
}

void PanicBroadcaster::panic(const char* short_reason)
{
    msg_.reason_text.clear();
    const char* p = short_reason;
    while (p && *p)
    {
        if (msg_.reason_text.size() == msg_.reason_text.capacity())
        {
            break;
        }
        msg_.reason_text.push_back(protocol::Panic::FieldTypes::reason_text::ValueType(*p));
        p++;
    }

    UAVCAN_TRACE("PanicBroadcaster", "Panicking with reason '%s'", getReason().c_str());

    publishOnce();
    startPeriodic(MonotonicDuration::fromMSec(protocol::Panic::BROADCASTING_INTERVAL_MS));
}

void PanicBroadcaster::dontPanic()
{
    stop();
    msg_.reason_text.clear();
}

bool PanicBroadcaster::isPanicking() const
{
    return isRunning();
}

const typename protocol::Panic::FieldTypes::reason_text& PanicBroadcaster::getReason() const
{
    return msg_.reason_text;
}

}
