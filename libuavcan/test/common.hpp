/*
 * Copyright (C) 2014 <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <uavcan/can_driver.hpp>
#include <uavcan/system_clock.hpp>

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
static uavcan::CanFrame makeCanFrame(uint32_t id, const std::string& str_data, FrameType type)
{
    id |= (type == EXT) ? uavcan::CanFrame::FLAG_EFF : 0;
    return uavcan::CanFrame(id, reinterpret_cast<const uint8_t*>(str_data.c_str()), str_data.length());
}
