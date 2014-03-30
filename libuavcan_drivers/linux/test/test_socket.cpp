/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <iostream>
#include <cerrno>
#include <uavcan_linux/uavcan_linux.hpp>

#define ASSERT(x) if (!(x)) { throw std::runtime_error(#x); }

static uavcan::CanFrame makeFrame(std::uint32_t id, const std::string& data)
{
    return uavcan::CanFrame(id, reinterpret_cast<const std::uint8_t*>(data.c_str()), data.length());
}

static uavcan::MonotonicTime tsMonoOffsetMs(std::int64_t ms)
{
    return uavcan_linux::SystemClock().getMonotonic() + uavcan::MonotonicDuration::fromMSec(ms);
}

static void testNonexistentIface()
{
    const int sock1 = uavcan_linux::SocketCanIface::openSocket("noif9");
    ASSERT(sock1 < 0);
    const int sock2 = uavcan_linux::SocketCanIface::openSocket("verylongifacenameverylongifacenameverylongifacename");
    ASSERT(sock2 < 0);
}

static void testSocketRxTx(const std::string& iface_name)
{
    const int sock1 = uavcan_linux::SocketCanIface::openSocket(iface_name);
    const int sock2 = uavcan_linux::SocketCanIface::openSocket(iface_name);
    ASSERT(sock1 >= 0 && sock2 >= 0);

    uavcan_linux::SocketCanIface if1(sock1);
    uavcan_linux::SocketCanIface if2(sock2);

    /*
     * Sending two frames, one of which must be returned back
     */
    ASSERT(1 == if1.send(makeFrame(123, "if1-1"), tsMonoOffsetMs(100), 0));
    ASSERT(1 == if1.send(makeFrame(456, "if1-2"), tsMonoOffsetMs(100), uavcan::CanIOFlagLoopback));
    ASSERT(if1.hasPendingTx());
    if1.poll(true, true);    // Reads confirmation for the first, writes the second
    if1.poll(true, true);    // Reads confirmation for the second and stores it in RX queue, writes nothing
    ASSERT(0 == if1.getErrorCount());
    ASSERT(!if1.hasPendingTx());
    ASSERT(if1.hasReadyRx());       // Second loopback

    /*
     * Second iface, same thing
     */
    ASSERT(1 == if2.send(makeFrame(321, "if2-1"), tsMonoOffsetMs(100), 0));
    ASSERT(1 == if2.send(makeFrame(654, "if2-2"), tsMonoOffsetMs(100), uavcan::CanIOFlagLoopback));
    ASSERT(1 == if2.send(makeFrame(1, "discard"), tsMonoOffsetMs(0), uavcan::CanIOFlagLoopback));  // Will timeout
    ASSERT(if2.hasPendingTx());
    if2.poll(true, true);    // Reads confirmation for the first, writes the second
    if2.poll(true, true);    // Reads confirmation for the second and stores it in RX queue, writes nothing
    ASSERT(1 == if2.getErrorCount());  // One timed out
    ASSERT(!if2.hasPendingTx());
    ASSERT(if2.hasReadyRx());

    /*
     * No-op
     */
    if1.poll(true, true);
    if2.poll(true, true);

    uavcan::CanFrame frame;
    uavcan::MonotonicTime ts_mono;
    uavcan::UtcTime ts_utc;
    uavcan::CanIOFlags flags = 0;

    const uavcan_linux::SystemClock clock(uavcan_linux::ClockAdjustmentMode::PerDriverPrivate);

    /*
     * Read first
     */
    ASSERT(1 == if1.receive(frame, ts_mono, ts_utc, flags));
    ASSERT(frame == makeFrame(321, "if2-1"));
    ASSERT(flags == 0);
    ASSERT(!ts_mono.isZero());
    ASSERT(!ts_utc.isZero());
    ASSERT((clock.getMonotonic() - ts_mono).getAbs().toMSec() < 10);
    ASSERT((clock.getUtc() - ts_utc).getAbs().toMSec() < 10);

    ASSERT(1 == if1.receive(frame, ts_mono, ts_utc, flags));
    ASSERT(frame == makeFrame(456, "if1-2"));
    ASSERT(flags == uavcan::CanIOFlagLoopback);
    ASSERT((clock.getMonotonic() - ts_mono).getAbs().toMSec() < 10);
    ASSERT((clock.getUtc() - ts_utc).getAbs().toMSec() < 10);

    ASSERT(1 == if1.receive(frame, ts_mono, ts_utc, flags));
    ASSERT(frame == makeFrame(654, "if2-2"));
    ASSERT(flags == 0);
    ASSERT((clock.getMonotonic() - ts_mono).getAbs().toMSec() < 10);
    ASSERT((clock.getUtc() - ts_utc).getAbs().toMSec() < 10);

    ASSERT(0 == if1.receive(frame, ts_mono, ts_utc, flags));
    ASSERT(!if1.hasPendingTx());
    ASSERT(!if1.hasReadyRx());

    /*
     * Read second
     */
    ASSERT(1 == if2.receive(frame, ts_mono, ts_utc, flags));
    ASSERT(frame == makeFrame(123, "if1-1"));
    ASSERT(flags == 0);
    ASSERT((clock.getMonotonic() - ts_mono).getAbs().toMSec() < 10);
    ASSERT((clock.getUtc() - ts_utc).getAbs().toMSec() < 10);

    ASSERT(1 == if2.receive(frame, ts_mono, ts_utc, flags));
    ASSERT(frame == makeFrame(456, "if1-2"));
    ASSERT(flags == 0);
    ASSERT((clock.getMonotonic() - ts_mono).getAbs().toMSec() < 10);
    ASSERT((clock.getUtc() - ts_utc).getAbs().toMSec() < 10);

    ASSERT(1 == if2.receive(frame, ts_mono, ts_utc, flags));
    ASSERT(frame == makeFrame(654, "if2-2"));
    ASSERT(flags == uavcan::CanIOFlagLoopback);
    ASSERT((clock.getMonotonic() - ts_mono).getAbs().toMSec() < 10);
    ASSERT((clock.getUtc() - ts_utc).getAbs().toMSec() < 10);

    ASSERT(0 == if2.receive(frame, ts_mono, ts_utc, flags));
    ASSERT(!if2.hasPendingTx());
    ASSERT(!if2.hasReadyRx());
}

static void testSocketFilters(const std::string& iface_name)
{
    using uavcan::CanFrame;

    const int sock1 = uavcan_linux::SocketCanIface::openSocket(iface_name);
    const int sock2 = uavcan_linux::SocketCanIface::openSocket(iface_name);
    ASSERT(sock1 >= 0 && sock2 >= 0);

    uavcan_linux::SocketCanIface if1(sock1);
    uavcan_linux::SocketCanIface if2(sock2);

    /*
     * Configuring filters
     */
    uavcan::CanFilterConfig fcs[3];
    // STD/EXT 123
    fcs[0].id = 123;
    fcs[0].mask = CanFrame::MaskExtID;
    // Only EXT 456789
    fcs[1].id = 456789 | CanFrame::FlagEFF;
    fcs[1].mask = CanFrame::MaskExtID | CanFrame::FlagEFF;
    // Only STD 0
    fcs[2].id = 0;
    fcs[2].mask = CanFrame::MaskExtID | CanFrame::FlagEFF;

    ASSERT(0 == if2.configureFilters(fcs, 3));

    /*
     * Sending data from 1 to 2, making sure only filtered data will be accepted
     */
    const auto EFF = CanFrame::FlagEFF;
    ASSERT(1 == if1.send(makeFrame(123,          "1"), tsMonoOffsetMs(100), 0)); // Accept 0
    ASSERT(1 == if1.send(makeFrame(123 | EFF,    "2"), tsMonoOffsetMs(100), 0)); // Accept 0
    ASSERT(1 == if1.send(makeFrame(456,          "3"), tsMonoOffsetMs(100), 0)); // Drop
    ASSERT(1 == if1.send(makeFrame(456789,       "4"), tsMonoOffsetMs(100), 0)); // Drop
    ASSERT(1 == if1.send(makeFrame(456789 | EFF, "5"), tsMonoOffsetMs(100), 0)); // Accept 1
    ASSERT(1 == if1.send(makeFrame(0,            "6"), tsMonoOffsetMs(100), 0)); // Accept 2
    ASSERT(1 == if1.send(makeFrame(EFF,          "7"), tsMonoOffsetMs(100), 0)); // Drop

    for (int i = 0; i < 7; i++)
    {
        if1.poll(true, true);
        if2.poll(true, false);
    }
    ASSERT(!if1.hasPendingTx());
    ASSERT(!if1.hasReadyRx());
    ASSERT(0 == if1.getErrorCount());
    ASSERT(if2.hasReadyRx());

    /*
     * Checking RX on 2
     * Notice how the frames were reordered according to CAN bus arbitration rules
     */
    uavcan::CanFrame frame;
    uavcan::MonotonicTime ts_mono;
    uavcan::UtcTime ts_utc;
    uavcan::CanIOFlags flags = 0;

    ASSERT(1 == if2.receive(frame, ts_mono, ts_utc, flags));
    ASSERT(frame == makeFrame(0, "6"));
    ASSERT(flags == 0);

    ASSERT(1 == if2.receive(frame, ts_mono, ts_utc, flags));
    ASSERT(frame == makeFrame(123 | EFF, "2"));

    ASSERT(1 == if2.receive(frame, ts_mono, ts_utc, flags));
    ASSERT(frame == makeFrame(456789 | EFF, "5"));

    ASSERT(1 == if2.receive(frame, ts_mono, ts_utc, flags));
    ASSERT(frame == makeFrame(123, "1"));

    ASSERT(!if2.hasReadyRx());
}

int main(int argc, const char** argv)
{
    if (argc < 2)
    {
        std::cout << "Usage:\n\t" << argv[0] << " <can-iface-name>" << std::endl;
        return 1;
    }

    testNonexistentIface();
    testSocketRxTx(argv[1]);
    testSocketFilters(argv[1]);

    return 0;
}
