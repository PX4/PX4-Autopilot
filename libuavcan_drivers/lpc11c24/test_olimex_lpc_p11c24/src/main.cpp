/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <board.hpp>
#include <chip.h>
#include <uavcan_lpc11c24/uavcan_lpc11c24.hpp>
#include <uavcan/protocol/global_time_sync_slave.hpp>
#include <uavcan/protocol/dynamic_node_id_client.hpp>
#include <uavcan/protocol/logger.hpp>

/*
 * GCC 4.9 cannot generate a working binary with higher optimization levels, although
 *         rest of the firmware can be compiled with -Os.
 * GCC 4.8 and earlier don't work at all on this firmware.
 */
#if __GNUC__
# pragma GCC optimize 1
#endif

/**
 * This function re-defines the standard ::rand(), which is used by the class uavcan::DynamicNodeIDClient.
 * Redefinition is normally not needed, but GCC 4.9 tends to generate broken binaries if it is not redefined.
 */
int rand()
{
    static int x = 1;
    x = x * 48271 % 2147483647;
    return x;
}

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

uavcan::NodeID performDynamicNodeIDAllocation()
{
    uavcan::DynamicNodeIDClient client(getNode());

    const int client_start_res = client.start(getNode().getHardwareVersion().unique_id);
    if (client_start_res < 0)
    {
        board::die();
    }

    while (!client.isAllocationComplete())
    {
        board::resetWatchdog();
        (void)getNode().spin(uavcan::MonotonicDuration::fromMSec(100));
    }

    return client.getAllocatedNodeID();
}

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
        board::syslog("CAN auto bitrate...\r\n");
        bit_rate = uavcan_lpc11c24::CanDriver::detectBitRate(&board::resetWatchdog);
    }
    board::syslog("Bitrate: ");
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
     * Starting the node and performing dynamic node ID allocation
     */
    if (getNode().start() < 0)
    {
        board::die();
    }

    board::syslog("Node ID allocation...\r\n");

    getNode().setNodeID(performDynamicNodeIDAllocation());

    board::syslog("Node ID ");
    board::syslog(intToString(getNode().getNodeID().get()).c_str());
    board::syslog("\r\n");

    board::resetWatchdog();

    /*
     * Example filter configuration.
     * Can be removed safely.
     */
    {
        constexpr unsigned NumFilters = 3;
        uavcan::CanFilterConfig filters[NumFilters];

        // Acepting all service transfers addressed to us
        filters[0].id   = (unsigned(getNode().getNodeID().get()) << 8) | (1U << 7) | uavcan::CanFrame::FlagEFF;
        filters[0].mask = 0x7F80 | uavcan::CanFrame::FlagEFF;

        // Accepting time sync messages
        filters[1].id   = (4U << 8) | uavcan::CanFrame::FlagEFF;
        filters[1].mask = 0xFFFF80 | uavcan::CanFrame::FlagEFF;

        // Accepting zero CAN ID (just for the sake of testing)
        filters[2].id   = 0 | uavcan::CanFrame::FlagEFF;
        filters[2].mask = uavcan::CanFrame::MaskExtID | uavcan::CanFrame::FlagEFF;

        if (uavcan_lpc11c24::CanDriver::instance().configureFilters(filters, NumFilters) < 0)
        {
            board::syslog("Filter init failed\r\n");
            board::die();
        }
    }

    /*
     * Initializing other libuavcan-related objects
     */
    if (getTimeSyncSlave().start() < 0)
    {
        board::die();
    }

    if (getLogger().init() < 0)
    {
        board::die();
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
        const int res = getNode().spin(uavcan::MonotonicDuration::fromMSec(25));
        board::setErrorLed(res < 0);
        board::setStatusLed(uavcan_lpc11c24::CanDriver::instance().hadActivity());

        const auto ts = uavcan_lpc11c24::clock::getMonotonic();
        if ((ts - prev_log_at).toMSec() >= 1000)
        {
            prev_log_at = ts;

            /*
             * CAN bus off state monitoring
             */
            if (uavcan_lpc11c24::CanDriver::instance().isInBusOffState())
            {
                board::syslog("CAN BUS OFF\r\n");
            }

            /*
             * CAN error counter, for debugging purposes
             */
            board::syslog("CAN errors: ");
            board::syslog(intToString(static_cast<long long>(uavcan_lpc11c24::CanDriver::instance().getErrorCount())).c_str());
            board::syslog(" ");
            board::syslog(intToString(uavcan_lpc11c24::CanDriver::instance().getRxQueueOverflowCount()).c_str());
            board::syslog("\r\n");

            /*
             * We don't want to use formatting functions provided by libuavcan because they rely on std::snprintf(),
             * so we need to construct the message manually:
             */
            uavcan::protocol::debug::LogMessage logmsg;
            logmsg.level.value = uavcan::protocol::debug::LogLevel::INFO;
            logmsg.source = "app";
            logmsg.text = intToString(uavcan_lpc11c24::clock::getPrevUtcAdjustment().toUSec()).c_str();
            (void)getLogger().log(logmsg);
        }

        board::resetWatchdog();
    }
}
