/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/internal/can_io.hpp>

class SystemClockMock : public uavcan::ISystemClock
{
public:
    uint64_t monotonic;
    uint64_t utc;

    SystemClockMock()
    : monotonic(0)
    , utc(0)
    { }

    void advance(uint64_t usec)
    {
        monotonic += usec;
        utc += usec;
    }

    uint64_t getMonotonicMicroseconds() const
    {
        assert(this);
        return monotonic;
    }

    uint64_t getUtcMicroseconds() const
    {
        assert(this);
        return utc;
    }

    void adjustUtcMicroseconds(uint64_t new_timestamp_usec, int64_t offset_usec)
    {
        assert(0);
    }
};

enum FrameType { STD, EXT };
static uavcan::CanFrame makeFrame(uint32_t id, const std::string& str_data, FrameType type)
{
    id |= (type == EXT) ? uavcan::CanFrame::FLAG_EFF : 0;
    return uavcan::CanFrame(id, reinterpret_cast<const uint8_t*>(str_data.c_str()), str_data.length());
}

namespace
{

int getQueueLength(uavcan::CanTxQueue& queue)
{
    const uavcan::CanTxQueue::Entry* p = queue.peek();
    int length = 0;
    while (p)
    {
        length++;
        p = p->getNextListNode();
    }
    return length;
}

bool isInQueue(uavcan::CanTxQueue& queue, const uavcan::CanFrame& frame)
{
    const uavcan::CanTxQueue::Entry* p = queue.peek();
    while (p)
    {
        if (frame == p->frame)
            return true;
        p = p->getNextListNode();
    }
    return false;
}

}
