//
// Multicopter mission test.
//
// Author: Julian Oes <julian@oes.ch>

#define CATCH_CONFIG_RUNNER
#include "catch2/catch.hpp"

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <iostream>
#include <string>
#include "autopilot_tester.h"

using namespace mavsdk;

static std::string connection_url{"udp://"};

static void usage(const std::string& bin_name);
static void remove_argv(int& argc, char** argv, int pos);

int main(int argc, char** argv)
{
    for (int i = 0; i < argc; ++i) {
        const std::string argv_string(argv[i]);

        if (argv_string == "-h") {
            usage(argv[0]);
        }

        if (argv_string == "--url") {
            if (argc > i + 1) {
                connection_url = argv[i+1];
                remove_argv(argc, argv, i);
                remove_argv(argc, argv, i);
            } else {
                std::cerr << "No connection URL supplied" << std::endl;
                usage(argv[0]);
                return -1;
            }
        }
    }

    Catch::Session session;
    const int catch_ret = session.applyCommandLine(argc, argv);
    if (catch_ret != 0) {
        return catch_ret;
    }
    return session.run();
}

void usage(const std::string& bin_name)
{
    std::cout << std::endl
              << "Usage : " << bin_name << " [--url CONNECTION_URL] [catch2 arguments]" << std::endl
              << "Connection URL format should be :" << std::endl
              << " For TCP : tcp://[server_host][:server_port]" << std::endl
              << " For UDP : udp://[bind_host][:bind_port]" << std::endl
              << " For Serial : serial:///path/to/serial/dev[:baudrate]" << std::endl
              << "For example, to connect to the simulator use URL: udp://:14540" << std::endl
              << std::endl;
}

void remove_argv(int& argc, char** argv, int pos)
{
    for (int i = pos; i+1 < argc; ++i) {
        argv[i] = argv[i+1];
    }
    argv[--argc] = nullptr;
}

TEST_CASE("We can takeoff and land", "[multicopter]")
{
    AutopilotTester tester;
    tester.connect(connection_url);
    tester.wait_until_ready();
    tester.arm();
    tester.takeoff();
    tester.wait_until_hovering();
    tester.land();
    tester.wait_until_disarmed();
}

TEST_CASE("We can fly a square mission and do RTL", "[multicopter]")
{
    AutopilotTester tester;
    tester.connect(connection_url);
    tester.wait_until_ready();

    AutopilotTester::MissionOptions mission_options;
    mission_options.rtl_at_end = true;
    tester.prepare_square_mission(mission_options);

    tester.arm();
    tester.execute_mission();
    tester.wait_until_hovering();
    tester.wait_until_disarmed();
}