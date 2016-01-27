/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <iostream>
#include <vector>
#include <cerrno>
#include <uavcan_linux/uavcan_linux.hpp>
#include "debug.hpp"

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
    ENFORCE(sock1 < 0);
    const int sock2 = uavcan_linux::SocketCanIface::openSocket("verylongifacenameverylongifacenameverylongifacename");
    ENFORCE(sock2 < 0);
}

static void testSocketRxTx(const std::string& iface_name)
{
    const int sock1 = uavcan_linux::SocketCanIface::openSocket(iface_name);
    const int sock2 = uavcan_linux::SocketCanIface::openSocket(iface_name);
    ENFORCE(sock1 >= 0 && sock2 >= 0);

    /*
     * Clocks will have some offset from the true system time
     * SocketCAN driver must handle this correctly
     */
    uavcan_linux::SystemClock clock_impl(uavcan_linux::ClockAdjustmentMode::PerDriverPrivate);
    clock_impl.adjustUtc(uavcan::UtcDuration::fromMSec(100000));
    const uavcan_linux::SystemClock& clock = clock_impl;

    uavcan_linux::SocketCanIface if1(clock, sock1);
    uavcan_linux::SocketCanIface if2(clock, sock2);

    /*
     * Sending two frames, one of which must be returned back
     */
    ENFORCE(1 == if1.send(makeFrame(123, "if1-1"), tsMonoOffsetMs(100), 0));
    ENFORCE(1 == if1.send(makeFrame(456, "if1-2"), tsMonoOffsetMs(100), uavcan::CanIOFlagLoopback));
    if1.poll(true, true);
    if1.poll(true, true);
    ENFORCE(0 == if1.getErrorCount());
    ENFORCE(!if1.hasReadyTx());
    ENFORCE(if1.hasReadyRx());       // Second loopback

    /*
     * Second iface, same thing
     */
    ENFORCE(1 == if2.send(makeFrame(321, "if2-1"), tsMonoOffsetMs(100), 0));
    ENFORCE(1 == if2.send(makeFrame(654, "if2-2"), tsMonoOffsetMs(100), uavcan::CanIOFlagLoopback));
    ENFORCE(1 == if2.send(makeFrame(1, "discard"), tsMonoOffsetMs(-1), uavcan::CanIOFlagLoopback));  // Will timeout
    if2.poll(true, true);
    if2.poll(true, true);
    ENFORCE(1 == if2.getErrorCount());  // One timed out
    ENFORCE(!if2.hasReadyTx());
    ENFORCE(if2.hasReadyRx());

    /*
     * No-op
     */
    if1.poll(true, true);
    if2.poll(true, true);

    uavcan::CanFrame frame;
    uavcan::MonotonicTime ts_mono;
    uavcan::UtcTime ts_utc;
    uavcan::CanIOFlags flags = 0;

    /*
     * Read first
     */
    ENFORCE(1 == if1.receive(frame, ts_mono, ts_utc, flags));
    ENFORCE(frame == makeFrame(456, "if1-2"));
    ENFORCE(flags == uavcan::CanIOFlagLoopback);
    ENFORCE((clock.getMonotonic() - ts_mono).getAbs().toMSec() < 10);
    ENFORCE((clock.getUtc() - ts_utc).getAbs().toMSec() < 10);

    ENFORCE(1 == if1.receive(frame, ts_mono, ts_utc, flags));
    ENFORCE(frame == makeFrame(321, "if2-1"));
    ENFORCE(flags == 0);
    ENFORCE(!ts_mono.isZero());
    ENFORCE(!ts_utc.isZero());
    ENFORCE((clock.getMonotonic() - ts_mono).getAbs().toMSec() < 10);
    ENFORCE((clock.getUtc() - ts_utc).getAbs().toMSec() < 10);

    ENFORCE(1 == if1.receive(frame, ts_mono, ts_utc, flags));
    ENFORCE(frame == makeFrame(654, "if2-2"));
    ENFORCE(flags == 0);
    ENFORCE((clock.getMonotonic() - ts_mono).getAbs().toMSec() < 10);
    ENFORCE((clock.getUtc() - ts_utc).getAbs().toMSec() < 10);

    ENFORCE(0 == if1.receive(frame, ts_mono, ts_utc, flags));
    ENFORCE(!if1.hasReadyTx());
    ENFORCE(!if1.hasReadyRx());

    /*
     * Read second
     */
    ENFORCE(1 == if2.receive(frame, ts_mono, ts_utc, flags));
    ENFORCE(frame == makeFrame(123, "if1-1"));
    ENFORCE(flags == 0);
    ENFORCE((clock.getMonotonic() - ts_mono).getAbs().toMSec() < 10);
    ENFORCE((clock.getUtc() - ts_utc).getAbs().toMSec() < 10);

    ENFORCE(1 == if2.receive(frame, ts_mono, ts_utc, flags));
    ENFORCE(frame == makeFrame(456, "if1-2"));
    ENFORCE(flags == 0);
    ENFORCE((clock.getMonotonic() - ts_mono).getAbs().toMSec() < 10);
    ENFORCE((clock.getUtc() - ts_utc).getAbs().toMSec() < 10);

    ENFORCE(1 == if2.receive(frame, ts_mono, ts_utc, flags));
    ENFORCE(frame == makeFrame(654, "if2-2"));
    ENFORCE(flags == uavcan::CanIOFlagLoopback);
    ENFORCE((clock.getMonotonic() - ts_mono).getAbs().toMSec() < 10);
    ENFORCE((clock.getUtc() - ts_utc).getAbs().toMSec() < 10);

    ENFORCE(0 == if2.receive(frame, ts_mono, ts_utc, flags));
    ENFORCE(!if2.hasReadyTx());
    ENFORCE(!if2.hasReadyRx());
}

static void testSocketFilters(const std::string& iface_name)
{
    using uavcan::CanFrame;

    const int sock1 = uavcan_linux::SocketCanIface::openSocket(iface_name);
    const int sock2 = uavcan_linux::SocketCanIface::openSocket(iface_name);
    ENFORCE(sock1 >= 0 && sock2 >= 0);

    /*
     * Clocks will have some offset from the true system time
     * SocketCAN driver must handle this correctly
     */
    uavcan_linux::SystemClock clock_impl(uavcan_linux::ClockAdjustmentMode::PerDriverPrivate);
    clock_impl.adjustUtc(uavcan::UtcDuration::fromMSec(-1000));
    const uavcan_linux::SystemClock& clock = clock_impl;

    uavcan_linux::SocketCanIface if1(clock, sock1);
    uavcan_linux::SocketCanIface if2(clock, sock2);

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

    ENFORCE(0 == if2.configureFilters(fcs, 3));

    /*
     * Sending data from 1 to 2, making sure only filtered data will be accepted
     */
    const auto EFF = CanFrame::FlagEFF;
    ENFORCE(1 == if1.send(makeFrame(123,          "1"), tsMonoOffsetMs(100), 0)); // Accept 0
    ENFORCE(1 == if1.send(makeFrame(123 | EFF,    "2"), tsMonoOffsetMs(100), 0)); // Accept 0
    ENFORCE(1 == if1.send(makeFrame(456,          "3"), tsMonoOffsetMs(100), 0)); // Drop
    ENFORCE(1 == if1.send(makeFrame(456789,       "4"), tsMonoOffsetMs(100), 0)); // Drop
    ENFORCE(1 == if1.send(makeFrame(456789 | EFF, "5"), tsMonoOffsetMs(100), 0)); // Accept 1
    ENFORCE(1 == if1.send(makeFrame(0,            "6"), tsMonoOffsetMs(100), 0)); // Accept 2
    ENFORCE(1 == if1.send(makeFrame(EFF,          "7"), tsMonoOffsetMs(100), 0)); // Drop

    for (int i = 0; i < 7; i++)
    {
        if1.poll(true, true);
        if2.poll(true, false);
    }
    ENFORCE(!if1.hasReadyTx());
    ENFORCE(!if1.hasReadyRx());
    ENFORCE(0 == if1.getErrorCount());
    ENFORCE(if2.hasReadyRx());

    /*
     * Checking RX on 2
     */
    uavcan::CanFrame frame;
    uavcan::MonotonicTime ts_mono;
    uavcan::UtcTime ts_utc;
    uavcan::CanIOFlags flags = 0;

    ENFORCE(1 == if2.receive(frame, ts_mono, ts_utc, flags));
    ENFORCE(frame == makeFrame(123, "1"));

    ENFORCE(1 == if2.receive(frame, ts_mono, ts_utc, flags));
    ENFORCE(frame == makeFrame(123 | EFF, "2"));

    ENFORCE(1 == if2.receive(frame, ts_mono, ts_utc, flags));
    ENFORCE(frame == makeFrame(456789 | EFF, "5"));

    ENFORCE(1 == if2.receive(frame, ts_mono, ts_utc, flags));
    ENFORCE(frame == makeFrame(0, "6"));
    ENFORCE(flags == 0);

    ENFORCE(!if2.hasReadyRx());
}

static void testDriver(const std::vector<std::string>& iface_names)
{
    /*
     * Clocks will have some offset from the true system time
     * SocketCAN driver must handle this correctly
     */
    uavcan_linux::SystemClock clock_impl(uavcan_linux::ClockAdjustmentMode::PerDriverPrivate);
    clock_impl.adjustUtc(uavcan::UtcDuration::fromMSec(9000000));
    const uavcan_linux::SystemClock& clock = clock_impl;

    uavcan_linux::SocketCanDriver driver(clock);
    for (auto ifn : iface_names)
    {
        std::cout << "Adding iface " << ifn << std::endl;
        ENFORCE(0 == driver.addIface(ifn));
    }

    ENFORCE(-1 == driver.addIface("noif9"));
    ENFORCE(-1 == driver.addIface("noif9"));
    ENFORCE(-1 == driver.addIface("noif9"));

    ENFORCE(driver.getNumIfaces() == iface_names.size());
    ENFORCE(nullptr == driver.getIface(255));
    ENFORCE(nullptr == driver.getIface(driver.getNumIfaces()));

    const uavcan::CanFrame* pending_tx[uavcan::MaxCanIfaces] = {};

    const unsigned AllIfacesMask = (1 << driver.getNumIfaces()) - 1;

    /*
     * Send, no loopback
     */
    std::cout << "select() 1" << std::endl;
    uavcan::CanSelectMasks masks;        // Driver provides masks for all available events
    ENFORCE(driver.getNumIfaces() == driver.select(masks, pending_tx, tsMonoOffsetMs(1000)));
    ENFORCE(masks.read == 0);
    ENFORCE(masks.write == AllIfacesMask);

    for (int i = 0; i < driver.getNumIfaces(); i++)
    {
        ENFORCE(1 == driver.getIface(i)->send(makeFrame(123, std::to_string(i)), tsMonoOffsetMs(10), 0));
    }

    std::cout << "select() 2" << std::endl;
    ENFORCE(driver.getNumIfaces() == driver.select(masks, pending_tx, tsMonoOffsetMs(1000)));
    ENFORCE(masks.read == 0);
    ENFORCE(masks.write == AllIfacesMask);

    /*
     * Send with loopback
     */
    for (int i = 0; i < driver.getNumIfaces(); i++)
    {
        ENFORCE(1 == driver.getIface(i)->send(makeFrame(456, std::to_string(i)), tsMonoOffsetMs(10),
                                              uavcan::CanIOFlagLoopback));
        ENFORCE(1 == driver.getIface(i)->send(makeFrame(789, std::to_string(i)), tsMonoOffsetMs(-1), // Will timeout
                                              uavcan::CanIOFlagLoopback));
    }

    std::cout << "select() 3" << std::endl;
    ENFORCE(driver.getNumIfaces() == driver.select(masks, pending_tx, tsMonoOffsetMs(1000)));
    ENFORCE(masks.read == AllIfacesMask);
    ENFORCE(masks.write == AllIfacesMask);

    /*
     * Receive loopback
     */
    for (int i = 0; i < driver.getNumIfaces(); i++)
    {
        uavcan::CanFrame frame;
        uavcan::MonotonicTime ts_mono;
        uavcan::UtcTime ts_utc;
        uavcan::CanIOFlags flags = 0;
        ENFORCE(1 == driver.getIface(i)->receive(frame, ts_mono, ts_utc, flags));
        ENFORCE(frame == makeFrame(456, std::to_string(i)));
        ENFORCE(flags == uavcan::CanIOFlagLoopback);
        ENFORCE((clock.getMonotonic() - ts_mono).getAbs().toMSec() < 10);
        ENFORCE((clock.getUtc() - ts_utc).getAbs().toMSec() < 10);

        ENFORCE(!driver.getIface(i)->hasReadyTx());
        ENFORCE(!driver.getIface(i)->hasReadyRx());
    }

    std::cout << "select() 4" << std::endl;
    masks.write = 0;
    ENFORCE(driver.getNumIfaces() == driver.select(masks, pending_tx, tsMonoOffsetMs(1000)));
    ENFORCE(masks.read == 0);
    ENFORCE(masks.write == AllIfacesMask);

    std::cout << "exit" << std::endl;

    /*
     * Error checks
     */
    for (int i = 0; i < driver.getNumIfaces(); i++)
    {
        for (auto kv : driver.getIface(i)->getErrors())
        {
            switch (kv.first)
            {
            case uavcan_linux::SocketCanError::SocketReadFailure:
            case uavcan_linux::SocketCanError::SocketWriteFailure:
            {
                ENFORCE(kv.second == 0);
                break;
            }
            case uavcan_linux::SocketCanError::TxTimeout:
            {
                ENFORCE(kv.second == 1);  // One timed out frame from the above
                break;
            }
            default:
            {
                ENFORCE(false);
                break;
            }
            }
        }
    }
}

int main(int argc, const char** argv)
{
    try
    {
        if (argc < 2)
        {
            std::cerr << "Usage:\n\t" << argv[0] << " <can-iface-name-1> [can-iface-name-N...]" << std::endl;
            return 1;
        }

        std::vector<std::string> iface_names;
        for (int i = 1; i < argc; i++)
        {
            iface_names.emplace_back(argv[i]);
        }

        testNonexistentIface();
        testSocketRxTx(iface_names[0]);
        testSocketFilters(iface_names[0]);

        testDriver(iface_names);

        return 0;
    }
    catch (const std::exception& ex)
    {
        std::cerr << "Exception: " << ex.what() << std::endl;
        return 1;
    }
}
