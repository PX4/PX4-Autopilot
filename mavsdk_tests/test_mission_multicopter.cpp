//
// Multicopter mission test.
//
// Author: Julian Oes <julian@oes.ch>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <iostream>
#include <string>
#include "autopilot_tester.h"


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