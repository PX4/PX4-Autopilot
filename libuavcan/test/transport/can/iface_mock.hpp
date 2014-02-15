/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <queue>
#include <vector>
#include <gtest/gtest.h>
#include <uavcan/internal/transport/can_io.hpp>
#include "../../common.hpp"


class CanIfaceMock : public uavcan::ICanIface
{
public:
    struct FrameWithTime
    {
        uavcan::CanFrame frame;
        uint64_t time;

        FrameWithTime(const uavcan::CanFrame& frame, uint64_t time)
        : frame(frame)
        , time(time)
        { }
    };

    std::queue<FrameWithTime> tx;       ///< Queue of outgoing frames (bus <-- library)
    std::queue<FrameWithTime> rx;       ///< Queue of incoming frames (bus --> library)
    bool writeable;
    bool tx_failure;
    bool rx_failure;
    uint64_t num_errors;
    SystemClockMock& clockmock;

    CanIfaceMock(SystemClockMock& clockmock)
    : writeable(true)
    , tx_failure(false)
    , rx_failure(false)
    , num_errors(0)
    , clockmock(clockmock)
    { }

    void pushRx(uavcan::CanFrame frame)
    {
        rx.push(FrameWithTime(frame, clockmock.utc));
    }

    bool matchAndPopTx(const uavcan::CanFrame& frame, uint64_t tx_deadline)
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

    int send(const uavcan::CanFrame& frame, uint64_t tx_timeout_usec)
    {
        assert(this);
        EXPECT_TRUE(writeable);        // Shall never be called when not writeable
        if (tx_failure)
            return -1;
        if (!writeable)
            return 0;
        const uint64_t monotonic_deadline = tx_timeout_usec + clockmock.monotonic;
        tx.push(FrameWithTime(frame, monotonic_deadline));
        return 1;
    }

    int receive(uavcan::CanFrame& out_frame, uint64_t& out_ts_monotonic_usec, uint64_t& out_ts_utc_usec)
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
        out_ts_monotonic_usec = frame.time;
        out_ts_utc_usec = 0;
        return 1;
    }

    // cppcheck-suppress unusedFunction
    // cppcheck-suppress functionConst
    int configureFilters(const uavcan::CanFilterConfig* filter_configs, int num_configs) { return -1; }
    // cppcheck-suppress unusedFunction
    int getNumFilters() const { return 0; }
    uint64_t getNumErrors() const { return num_errors; }
};

class CanDriverMock : public uavcan::ICanDriver
{
public:
    std::vector<CanIfaceMock> ifaces;
    SystemClockMock& clockmock;
    bool select_failure;

    CanDriverMock(int num_ifaces, SystemClockMock& clockmock)
    : ifaces(num_ifaces, CanIfaceMock(clockmock))
    , clockmock(clockmock)
    , select_failure(false)
    { }

    int select(int& inout_write_iface_mask, int& inout_read_iface_mask, uint64_t timeout_usec)
    {
        assert(this);
        std::cout << "Write/read masks: " << inout_write_iface_mask << "/" << inout_read_iface_mask << std::endl;

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
            clockmock.advance(timeout_usec);   // Emulating timeout
            return 0;
        }
        return 1;  // This value is not being checked anyway, it just has to be greater than zero
    }

    uavcan::ICanIface* getIface(int iface_index) { return &ifaces.at(iface_index); }
    int getNumIfaces() const { return ifaces.size(); }
};
