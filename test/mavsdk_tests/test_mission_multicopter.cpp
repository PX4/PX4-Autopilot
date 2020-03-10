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


TEST_CASE("Takeoff and Land", "[multicopter][vtol]")
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

TEST_CASE("Fly square Multicopter Missions", "[multicopter][vtol]")
{
    AutopilotTester tester;
    tester.connect(connection_url);
    tester.wait_until_ready();

    SECTION("Mission including RTL") {
        AutopilotTester::MissionOptions mission_options;
        mission_options.rtl_at_end = true;
        tester.prepare_square_mission(mission_options);
        tester.arm();
        tester.execute_mission();
        tester.wait_until_disarmed();
    }

    SECTION("Mission with manual RTL") {
        AutopilotTester::MissionOptions mission_options;
        mission_options.rtl_at_end = false;
        tester.prepare_square_mission(mission_options);
        tester.arm();
        tester.execute_mission();
        tester.wait_until_hovering();
        tester.execute_rtl();
        tester.wait_until_disarmed();
    }
}
