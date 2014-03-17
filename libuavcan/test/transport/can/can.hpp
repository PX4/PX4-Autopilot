/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <queue>
#include <vector>
#include <gtest/gtest.h>
#include <uavcan/transport/can_io.hpp>
#include <uavcan/transport/frame.hpp>
#include <uavcan/can_driver.hpp>
#include <uavcan/system_clock.hpp>
#include "../../clock.hpp"


class CanIfaceMock : public uavcan::ICanIface
{
public:
    struct FrameWithTime
    {
        uavcan::CanFrame frame;
        uavcan::MonotonicTime time;

        FrameWithTime(const uavcan::CanFrame& frame, uavcan::MonotonicTime time)
        : frame(frame)
        , time(time)
        { }

        FrameWithTime(const uavcan::CanFrame& frame, uint64_t time_usec)
        : frame(frame)
        , time(uavcan::MonotonicTime::fromUSec(time_usec))
        { }
    };

    std::queue<FrameWithTime> tx;       ///< Queue of outgoing frames (bus <-- library)
    std::queue<FrameWithTime> rx;       ///< Queue of incoming frames (bus --> library)
    bool writeable;
    bool tx_failure;
    bool rx_failure;
    uint64_t num_errors;
    uavcan::ISystemClock& iclock;

    CanIfaceMock(uavcan::ISystemClock& iclock)
    : writeable(true)
    , tx_failure(false)
    , rx_failure(false)
    , num_errors(0)
    , iclock(iclock)
    { }

    void pushRx(const uavcan::CanFrame& frame)
    {
        rx.push(FrameWithTime(frame, iclock.getMonotonic()));
    }

    void pushRx(const uavcan::RxFrame& frame)
    {
        uavcan::CanFrame can_frame;
        EXPECT_TRUE(frame.compile(can_frame));
        rx.push(FrameWithTime(can_frame, frame.getMonotonicTimestamp()));
    }

    bool matchAndPopTx(const uavcan::CanFrame& frame, uavcan::MonotonicTime tx_deadline)
    {
        if (tx.empty())
        {
            std::cout << "Tx buffer is empty" << std::endl;
            return false;
        }
        const FrameWithTime frame_time = tx.front();
        tx.pop();
        return (frame_time.frame == frame) && (frame_time.time == tx_deadline);
    }

    bool matchAndPopTx(const uavcan::CanFrame& frame, uint64_t tx_deadline_usec)
    {
        return matchAndPopTx(frame, uavcan::MonotonicTime::fromUSec(tx_deadline_usec));
    }

    uavcan::CanFrame popTxFrame()
    {
        if (tx.empty())
        {
            std::cout << "Tx buffer is empty" << std::endl;
            std::abort();
        }
        const FrameWithTime frame_time = tx.front();
        tx.pop();
        return frame_time.frame;
    }

    int send(const uavcan::CanFrame& frame, uavcan::MonotonicTime tx_deadline)
    {
        assert(this);
        EXPECT_TRUE(writeable);        // Shall never be called when not writeable
        if (tx_failure)
            return -1;
        if (!writeable)
            return 0;
        tx.push(FrameWithTime(frame, tx_deadline));
        return 1;
    }

    int receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic, uavcan::UtcTime& out_ts_utc)
    {
        assert(this);
        EXPECT_TRUE(rx.size());        // Shall never be called when not readable
        if (rx_failure)
            return -1;
        if (rx.empty())
            return 0;
        const FrameWithTime frame = rx.front();
        rx.pop();
        out_frame = frame.frame;
        out_ts_monotonic = frame.time;
        out_ts_utc = uavcan::UtcTime();
        return 1;
    }

    // cppcheck-suppress unusedFunction
    // cppcheck-suppress functionConst
    int configureFilters(const uavcan::CanFilterConfig*, int) { return -1; }
    // cppcheck-suppress unusedFunction
    int getNumFilters() const { return 0; }
    uint64_t getNumErrors() const { return num_errors; }
};

class CanDriverMock : public uavcan::ICanDriver
{
public:
    std::vector<CanIfaceMock> ifaces;
    uavcan::ISystemClock& iclock;
    bool select_failure;

    CanDriverMock(int num_ifaces, uavcan::ISystemClock& iclock)
    : ifaces(num_ifaces, CanIfaceMock(iclock))
    , iclock(iclock)
    , select_failure(false)
    { }

    int select(int& inout_write_iface_mask, int& inout_read_iface_mask, uavcan::MonotonicTime deadline)
    {
        assert(this);
        //std::cout << "Write/read masks: " << inout_write_iface_mask << "/" << inout_read_iface_mask << std::endl;

        if (select_failure)
            return -1;

        const int valid_iface_mask = (1 << getNumIfaces()) - 1;
        EXPECT_FALSE(inout_write_iface_mask & ~valid_iface_mask);
        EXPECT_FALSE(inout_read_iface_mask & ~valid_iface_mask);

        int out_write_mask = 0;
        int out_read_mask = 0;
        for (int i = 0; i < getNumIfaces(); i++)
        {
            const int mask = 1 << i;
            if ((inout_write_iface_mask & mask) && ifaces.at(i).writeable)
                out_write_mask |= mask;
            if ((inout_read_iface_mask & mask) && ifaces.at(i).rx.size())
                out_read_mask |= mask;
        }
        inout_write_iface_mask = out_write_mask;
        inout_read_iface_mask = out_read_mask;
        if ((out_write_mask | out_read_mask) == 0)
        {
            const uavcan::MonotonicTime ts = iclock.getMonotonic();
            const uavcan::MonotonicDuration diff = deadline - ts;
            SystemClockMock* const mock = dynamic_cast<SystemClockMock*>(&iclock);
            if (mock)
            {
                if (diff.isPositive())
                    mock->advance(diff.toUSec());   // Emulating timeout
            }
            else
            {
                if (diff.isPositive())
                    usleep(diff.toUSec());
            }
            return 0;
        }
        return 1;  // This value is not being checked anyway, it just has to be greater than zero
    }

    uavcan::ICanIface* getIface(int iface_index) { return &ifaces.at(iface_index); }
    int getNumIfaces() const { return ifaces.size(); }
};

enum FrameType { STD, EXT };
inline uavcan::CanFrame makeCanFrame(uint32_t id, const std::string& str_data, FrameType type)
{
    id |= (type == EXT) ? uavcan::CanFrame::FlagEFF : 0;
    return uavcan::CanFrame(id, reinterpret_cast<const uint8_t*>(str_data.c_str()), str_data.length());
}
