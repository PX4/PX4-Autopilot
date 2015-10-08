/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <unistd.h>
#include <zubax_chibios/sys/sys.h>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <uavcan/protocol/global_time_sync_slave.hpp>
#include "board/board.hpp"

namespace app
{
namespace
{

uavcan_stm32::CanInitHelper<128> can;

typedef uavcan::Node<16384> Node;

uavcan::LazyConstructor<Node> node_;

Node& getNode()
{
    if (!node_.isConstructed())
    {
        node_.construct<uavcan::ICanDriver&, uavcan::ISystemClock&>(can.driver, uavcan_stm32::SystemClock::instance());
    }
    return *node_;
}

void init()
{
    board::init();

    int res = 0;
    do
    {
        ::sleep(1);
        ::lowsyslog("CAN auto bit rate detection...\n");

        std::uint32_t bitrate = 0;
        res = can.init([]() { ::usleep(can.getRecommendedListeningDelay().toUSec()); }, bitrate);
        if (res >= 0)
        {
            ::lowsyslog("CAN inited at %u bps\n", unsigned(bitrate));
        }
    }
    while (res < 0);
}

class : public chibios_rt::BaseStaticThread<8192>
{
    void configureNodeInfo()
    {
        Node& node = app::getNode();

        node.setNodeID(64);
        node.setName("org.uavcan.stm32_test_stm32f107");

        /*
         * Software version
         * TODO: Fill other fields too
         */
        uavcan::protocol::SoftwareVersion swver;

        swver.vcs_commit = GIT_HASH;
        swver.optional_field_flags = swver.OPTIONAL_FIELD_FLAG_VCS_COMMIT;

        node.setSoftwareVersion(swver);

        lowsyslog("Git commit hash: 0x%08x\n", GIT_HASH);

        /*
         * Hardware version
         * TODO: Fill other fields too
         */
        uavcan::protocol::HardwareVersion hwver;

        std::uint8_t uid[board::UniqueIDSize] = {};
        board::readUniqueID(uid);
        std::copy(std::begin(uid), std::end(uid), std::begin(hwver.unique_id));

        node.setHardwareVersion(hwver);

        lowsyslog("UDID:");
        for (auto b : hwver.unique_id)
        {
            lowsyslog(" %02x", unsigned(b));
        }
        lowsyslog("\n");
    }

public:
    msg_t main()
    {
        /*
         * Setting up the node parameters
         */
        configureNodeInfo();

        Node& node = app::getNode();

        /*
         * Initializing the UAVCAN node - this may take a while
         */
        while (true)
        {
            // Calling start() multiple times is OK - only the first successfull call will be effective
            int res = node.start();

            if (res < 0)
            {
                lowsyslog("Node initialization failure: %i, will try agin soon\n", res);
            }
            else
            {
                break;
            }
            ::sleep(3);
        }

        /*
         * Time synchronizer
         */
        static uavcan::GlobalTimeSyncSlave time_sync_slave(node);
        {
            const int res = time_sync_slave.start();
            if (res < 0)
            {
                board::die(res);
            }
        }

        /*
         * Main loop
         */
        lowsyslog("UAVCAN node started\n");
        node.setModeOperational();
        while (true)
        {
            const int spin_res = node.spin(uavcan::MonotonicDuration::fromMSec(5000));
            if (spin_res < 0)
            {
                lowsyslog("Spin failure: %i\n", spin_res);
            }

            lowsyslog("Time sync master: %u\n", unsigned(time_sync_slave.getMasterNodeID().get()));

            lowsyslog("Memory usage: free=%u used=%u worst=%u\n",
                      node.getAllocator().getNumFreeBlocks(),
                      node.getAllocator().getNumUsedBlocks(),
                      node.getAllocator().getPeakNumUsedBlocks());

            lowsyslog("CAN errors: %lu %lu\n",
                      static_cast<unsigned long>(can.driver.getIface(0)->getErrorCount()),
                      static_cast<unsigned long>(can.driver.getIface(1)->getErrorCount()));

#if !UAVCAN_TINY
            node.getLogger().setLevel(uavcan::protocol::debug::LogLevel::INFO);
            node.logInfo("app", "UTC %* sec, %* corr, %* jumps",
                         uavcan_stm32::clock::getUtc().toMSec() / 1000,
                         uavcan_stm32::clock::getUtcSpeedCorrectionPPM(),
                         uavcan_stm32::clock::getUtcAjdustmentJumpCount());
#endif
        }
        return msg_t();
    }
} uavcan_node_thread;

}
}

int main()
{
    app::init();

    lowsyslog("Starting the UAVCAN thread\n");
    app::uavcan_node_thread.start(LOWPRIO);

    while (true)
    {
        for (int i = 0; i < 200; i++)
        {
            board::setLed(app::can.driver.hadActivity());
            ::usleep(25000);
        }

        const uavcan::UtcTime utc = uavcan_stm32::clock::getUtc();
        lowsyslog("UTC %lu sec   Rate corr: %fPPM   Jumps: %lu   Locked: %i\n",
                  static_cast<unsigned long>(utc.toMSec() / 1000),
                  uavcan_stm32::clock::getUtcRateCorrectionPPM(),
                  uavcan_stm32::clock::getUtcJumpCount(),
                  int(uavcan_stm32::clock::isUtcLocked()));
    }
}
