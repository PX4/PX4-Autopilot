#pragma once

#include <mavsdk/mavsdk.h>
#include <mavsdk/geometry.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include "catch2/catch.hpp"
#include <chrono>
#include <memory>
#include <thread>

extern std::string connection_url;

class AutopilotTester {
public:
    struct MissionOptions {
        double leg_length_m {20.0};
        double relative_altitude_m {10.0};
        bool rtl_at_end {false};
    };

    void connect(const std::string uri);
    void wait_until_ready();
    void set_takeoff_altitude(const float altitude_m);
    void arm();
    void takeoff();
    void land();
    void transition_to_fixedwing();
    void transition_to_multicopter();
    void wait_until_disarmed();
    void wait_until_hovering();
    void prepare_square_mission(MissionOptions mission_options);
    void execute_mission();
    void execute_rtl();

private:
    mavsdk::geometry::CoordinateTransformation _get_coordinate_transformation();
    std::shared_ptr<mavsdk::MissionItem> _create_mission_item(
        const mavsdk::geometry::CoordinateTransformation::LocalCoordinate& local_coordinate,
        const MissionOptions& mission_options,
        const mavsdk::geometry::CoordinateTransformation& ct);

    mavsdk::Mavsdk _mavsdk{};
    std::unique_ptr<mavsdk::Telemetry> _telemetry{};
    std::unique_ptr<mavsdk::Action> _action{};
    std::unique_ptr<mavsdk::Mission> _mission{};
};

template<typename Rep, typename Period>
bool poll_condition_with_timeout(
    std::function<bool()> fun, std::chrono::duration<Rep, Period> duration)
{
    // We need millisecond resolution for sleeping.
    const std::chrono::milliseconds duration_ms(duration);

    unsigned iteration = 0;
    while (!fun()) {
        std::this_thread::sleep_for(duration_ms / 10);
        if (iteration++ >= 10) {
            return false;
        }
    }
    return true;
}
