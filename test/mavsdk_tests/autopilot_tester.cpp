#include "autopilot_tester.h"
#include <iostream>
#include <future>

using namespace mavsdk;
using namespace mavsdk::geometry;

std::string connection_url {"udp://"};

void AutopilotTester::connect(const std::string uri)
{
    ConnectionResult ret = _mavsdk.add_any_connection(uri);
    REQUIRE(ret == ConnectionResult::SUCCESS);

    std::cout << "Waiting for system connect" << std::endl;
    REQUIRE(poll_condition_with_timeout(
        [this]() { return _mavsdk.is_connected(); }, std::chrono::seconds(25)));

    auto& system = _mavsdk.system();

    _telemetry.reset(new Telemetry(system));
    _action.reset(new Action(system));
    _mission.reset(new Mission(system));
}

void AutopilotTester::wait_until_ready()
{
    std::cout << "Waiting for system to be ready" << std::endl;
    CHECK(poll_condition_with_timeout(
        [this]() { return _telemetry->health_all_ok(); }, std::chrono::seconds(20)));
}

void AutopilotTester::set_takeoff_altitude(const float altitude_m)
{
    CHECK(Action::Result::SUCCESS == _action->set_takeoff_altitude(altitude_m));
    const auto result = _action->get_takeoff_altitude();
    CHECK(result.first == Action::Result::SUCCESS);
    CHECK(result.second == Approx(altitude_m));
}

void AutopilotTester::arm()
{
    const auto result = _action->arm();
    REQUIRE(result == Action::Result::SUCCESS);
}

void AutopilotTester::takeoff()
{
    const auto result = _action->takeoff();
    REQUIRE(result == Action::Result::SUCCESS);
}

void AutopilotTester::land()
{
    const auto result = _action->land();
    REQUIRE(result == Action::Result::SUCCESS);
}

void AutopilotTester::transition_to_fixedwing()
{
    const auto result = _action->transition_to_fixedwing();
    REQUIRE(result == Action::Result::SUCCESS);
}

void AutopilotTester::transition_to_multicopter()
{
    const auto result = _action->transition_to_multicopter();
    REQUIRE(result == Action::Result::SUCCESS);
}

void AutopilotTester::wait_until_disarmed()
{
    REQUIRE(poll_condition_with_timeout(
        [this]() { return !_telemetry->armed(); }, std::chrono::seconds(60)));
}

void AutopilotTester::wait_until_hovering()
{
    REQUIRE(poll_condition_with_timeout(
        [this]() { return _telemetry->landed_state() == Telemetry::LandedState::IN_AIR; }, std::chrono::seconds(10)));
}

void AutopilotTester::prepare_square_mission(MissionOptions mission_options)
{
    const auto ct = _get_coordinate_transformation();

    std::vector<std::shared_ptr<MissionItem>> mission_items {};
    mission_items.push_back(_create_mission_item({mission_options.leg_length_m, 0.}, mission_options, ct));
    mission_items.push_back(_create_mission_item({mission_options.leg_length_m, mission_options.leg_length_m}, mission_options, ct));
    mission_items.push_back(_create_mission_item({0., mission_options.leg_length_m}, mission_options, ct));

    _mission->set_return_to_launch_after_mission(mission_options.rtl_at_end);

    std::promise<void> prom;
    auto fut = prom.get_future();

    _mission->upload_mission_async(mission_items, [&prom](Mission::Result result) {
        REQUIRE(Mission::Result::SUCCESS == result);
        prom.set_value();
    });

    REQUIRE(fut.wait_for(std::chrono::seconds(2)) == std::future_status::ready);
}

void AutopilotTester::execute_mission()
{
    std::promise<void> prom;
    auto fut = prom.get_future();

    _mission->start_mission_async([&prom](Mission::Result result) {
        REQUIRE(Mission::Result::SUCCESS == result);
        prom.set_value();
    });

    // TODO: Adapt time limit based on mission size, flight speed, sim speed factor, etc.

    REQUIRE(poll_condition_with_timeout(
        [this]() { return _mission->mission_finished(); }, std::chrono::seconds(60)));

    REQUIRE(fut.wait_for(std::chrono::seconds(1)) == std::future_status::ready);
}

CoordinateTransformation AutopilotTester::_get_coordinate_transformation()
{
    const auto home = _telemetry->home_position();
    REQUIRE(std::isfinite(home.latitude_deg));
    REQUIRE(std::isfinite(home.longitude_deg));
    REQUIRE(std::isfinite(home.longitude_deg));
    return CoordinateTransformation({home.latitude_deg, home.longitude_deg});
}

std::shared_ptr<MissionItem>  AutopilotTester::_create_mission_item(
    const CoordinateTransformation::LocalCoordinate& local_coordinate,
    const MissionOptions& mission_options,
    const CoordinateTransformation& ct)
{
    auto mission_item = std::make_shared<MissionItem>();
    const auto pos_north = ct.global_from_local(local_coordinate);
    mission_item->set_position(pos_north.latitude_deg, pos_north.longitude_deg);
    mission_item->set_relative_altitude(mission_options.relative_altitude_m);
    return mission_item;
}

void AutopilotTester::execute_rtl()
{
    REQUIRE(Action::Result::SUCCESS == _action->return_to_launch());
}
