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

typedef uavcan::Node<3136> Node;

Node& getNode()
{
    static Node node(uavcan_lpc11c24::CanDriver::instance(), uavcan_lpc11c24::SystemClock::instance());
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
__attribute__((noreturn))
#endif
void die()
{
    while (true) { }
}

#if __GNUC__
__attribute__((noinline))
#endif
void init()
{
    board::resetWatchdog();

    if (uavcan_lpc11c24::CanDriver::instance().init(1000000) < 0)
    {
        die();
    }

    board::resetWatchdog();

    getNode().setNodeID(72);
    getNode().setName("org.uavcan.lpc11c24_test");

    uavcan::protocol::SoftwareVersion swver;
    swver.major = FW_VERSION_MAJOR;
    swver.minor = FW_VERSION_MINOR;
    swver.vcs_commit = GIT_HASH;
    swver.optional_field_mask = swver.OPTIONAL_FIELD_MASK_VCS_COMMIT;
    getNode().setSoftwareVersion(swver);

    uavcan::protocol::HardwareVersion hwver;
    std::uint8_t uid[board::UniqueIDSize] = {};
    board::readUniqueID(uid);
    std::copy(std::begin(uid), std::end(uid), std::begin(hwver.unique_id));
    getNode().setHardwareVersion(hwver);

    board::resetWatchdog();

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

void reverse(char* s)
{
    for (int i = 0, j = int(std::strlen(s)) - 1; i < j; i++, j--)
    {
        const char c = s[i];
        s[i] = s[j];
        s[j] = c;
    }
}

void lltoa(long long n, char buf[24])
{
    const short sign = (n < 0) ? -1 : 1;
    if (sign < 0)
    {
        n = -n;
    }
    unsigned i = 0;
    do
    {
        buf[i++] = char(n % 10 + '0');
    }
    while ((n /= 10) > 0);
    if (sign < 0)
    {
        buf[i++] = '-';
    }
    buf[i] = '\0';
    reverse(buf);
}

}

int main()
{
    init();

    getNode().setStatusOk();

    uavcan::MonotonicTime prev_log_at;

    while (true)
    {
        const int res = getNode().spin(uavcan::MonotonicDuration::fromMSec(25));
        board::setErrorLed(res < 0);
        board::setStatusLed(uavcan_lpc11c24::CanDriver::instance().hadActivity());

        const auto ts = uavcan_lpc11c24::clock::getMonotonic();
        if ((ts - prev_log_at).toMSec() >= 1000)
        {
            prev_log_at = ts;

            // We don't want to use formatting functions provided by libuavcan because they rely on std::snprintf()
            char buf[24];
            lltoa(uavcan_lpc11c24::clock::getPrevUtcAdjustment().toUSec(), buf);
            buf[sizeof(buf) - 1] = '\0';

            // ...hence we need to construct the message manually:
            uavcan::protocol::debug::LogMessage logmsg;
            logmsg.level.value = uavcan::protocol::debug::LogLevel::INFO;
            logmsg.source = "app";
            logmsg.text = buf;
            (void)getLogger().log(logmsg);
        }

        board::resetWatchdog();
    }
}
