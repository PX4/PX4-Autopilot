/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cstdio>
#include <algorithm>
#include <board.hpp>
#include <chip.h>
#include <uavcan_lpc11c24/uavcan_lpc11c24.hpp>
#include <uavcan/protocol/global_time_sync_slave.hpp>
#include <uavcan/protocol/logger.hpp>

namespace
{

static constexpr unsigned NodeMemoryPoolSize = 2800;

/**
 * This is a compact, reentrant and thread-safe replacement to standard llto().
 * It returns the string by value, no extra storage is needed.
 */
typename uavcan::MakeString<22>::Type intToString(long long n)
{
    char buf[24] = {};
    const short sign = (n < 0) ? -1 : 1;
    if (sign < 0)
    {
        n = -n;
    }
    unsigned pos = 0;
    do
    {
        buf[pos++] = char(n % 10 + '0');
    }
    while ((n /= 10) > 0);
    if (sign < 0)
    {
        buf[pos++] = '-';
    }
    buf[pos] = '\0';
    for (unsigned i = 0, j = pos - 1U; i < j; i++, j--)
    {
        std::swap(buf[i], buf[j]);
    }
    return static_cast<const char*>(buf);
}

uavcan::Node<NodeMemoryPoolSize>& getNode()
{
    static uavcan::Node<NodeMemoryPoolSize> node(uavcan_lpc11c24::CanDriver::instance(),
                                                 uavcan_lpc11c24::SystemClock::instance());
    return node;
}

uavcan::GlobalTimeSyncSlave& getTimeSyncSlave()
{
    static uavcan::GlobalTimeSyncSlave tss(getNode());
    return tss;
}

uavcan::Logger& getLogger()
{
    static uavcan::Logger logger(getNode());
    return logger;
}

#if __GNUC__
__attribute__((noinline))
#endif
void init()
{
    board::resetWatchdog();
    board::syslog("Boot\r\n");

    board::setErrorLed(false);
    board::setStatusLed(true);

    /*
     * Configuring the clock - this must be done before the CAN controller is initialized
     */
    uavcan_lpc11c24::clock::init();

    /*
     * Configuring the CAN controller
     */
    std::uint32_t bit_rate = 0;
    while (bit_rate == 0)
    {
        board::syslog("CAN bitrate detection...\r\n");
        bit_rate = uavcan_lpc11c24::CanDriver::detectBitRate(&board::resetWatchdog);
    }
    board::syslog("CAN bitrate: ");
    board::syslog(intToString(bit_rate).c_str());
    board::syslog("\r\n");

    if (uavcan_lpc11c24::CanDriver::instance().init(bit_rate) < 0)
    {
        board::die();
    }
    board::syslog("CAN init ok\r\n");

    board::resetWatchdog();

    /*
     * Configuring the node
     */
    getNode().setName("org.uavcan.lpc11c24_test");

    uavcan::protocol::SoftwareVersion swver;
    swver.major = FW_VERSION_MAJOR;
    swver.minor = FW_VERSION_MINOR;
    swver.vcs_commit = GIT_HASH;
    swver.optional_field_flags = swver.OPTIONAL_FIELD_FLAG_VCS_COMMIT;
    getNode().setSoftwareVersion(swver);

    uavcan::protocol::HardwareVersion hwver;
    std::uint8_t uid[board::UniqueIDSize] = {};
    board::readUniqueID(uid);
    std::copy(std::begin(uid), std::end(uid), std::begin(hwver.unique_id));
    getNode().setHardwareVersion(hwver);

    board::resetWatchdog();

    /*
     * Performing dynamic node ID allocation
     */
    getNode().setNodeID(72);  // TODO

    /*
     * Starting the node
     */
    while (getNode().start() < 0)
    {
    }

    board::resetWatchdog();

    while (getTimeSyncSlave().start() < 0)
    {
    }

    while (getLogger().init() < 0)
    {
    }
    getLogger().setLevel(uavcan::protocol::debug::LogLevel::DEBUG);

    board::resetWatchdog();
}

}

int main()
{
    init();

    getNode().setModeOperational();

    uavcan::MonotonicTime prev_log_at;

    while (true)
    {
        //const int res = getNode().spin(uavcan::MonotonicDuration::fromMSec(25));
        const int res = getNode().spinOnce();
        board::setErrorLed(res < 0);
        board::setStatusLed(uavcan_lpc11c24::CanDriver::instance().hadActivity());

        if (uavcan_lpc11c24::CanDriver::instance().hasEmptyTx())
        {
            // Abort test
            static unsigned value = 0;
            const std::uint8_t payload[] = {
                0,
                0,
                0,
                0,
                std::uint8_t(value >> 24),
                std::uint8_t(value >> 16),
                std::uint8_t(value >> 8),
                std::uint8_t(value >> 0),
            };
            value++;
            (void)uavcan_lpc11c24::CanDriver::instance().send(uavcan::CanFrame(123U | uavcan::CanFrame::FlagEFF,
                                                                               payload, sizeof(payload)),
                                                              uavcan::MonotonicTime::getMax(),
                                                              uavcan::CanIOFlagAbortOnError);
        }

        const auto ts = uavcan_lpc11c24::clock::getMonotonic();
        if ((ts - prev_log_at).toMSec() >= 1000)
        {
            prev_log_at = ts;
            // CAN bus off state monitoring
            board::syslog("CAN bus off: ");
            board::syslog(intToString(int(uavcan_lpc11c24::CanDriver::instance().isInBusOffState())).c_str());
            board::syslog("\r\n");
            // CAN error counter, for debugging purposes
            board::syslog("CAN errors: ");
            board::syslog(intToString(static_cast<long long>(uavcan_lpc11c24::CanDriver::instance().getErrorCount())).c_str());
            board::syslog(" ");
            board::syslog(intToString(uavcan_lpc11c24::CanDriver::instance().getRxQueueOverflowCount()).c_str());
            board::syslog("\r\n");
            // We don't want to use formatting functions provided by libuavcan because they rely on std::snprintf()
            // hence we need to construct the message manually:
            uavcan::protocol::debug::LogMessage logmsg;
            logmsg.level.value = uavcan::protocol::debug::LogLevel::INFO;
            logmsg.source = "app";
            logmsg.text = intToString(uavcan_lpc11c24::clock::getPrevUtcAdjustment().toUSec()).c_str();
            (void)getLogger().log(logmsg);
        }

        board::resetWatchdog();
    }
}
