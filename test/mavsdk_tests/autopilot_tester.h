#pragma once

#include <mavsdk/mavsdk.h>
#include <mavsdk/geometry.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include "catch2/catch.hpp"
#include <chrono>
#include <memory>
#include <thread>

extern std::string connection_url;

using namespace mavsdk;
using namespace mavsdk::geometry;

class AutopilotTester {
public:
    struct MissionOptions {
        double leg_length_m {20.0};
        double relative_altitude_m {10.0};
        bool rtl_at_end {false};
    };

    void connect(const std::string uri);
    void wait_until_ready();
    void wait_until_ready_local_position_only();
    void store_home();
    void check_home_within(float acceptance_radius_m);
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
    void offboard_goto(const Offboard::PositionNEDYaw& target, float acceptance_radius_m = 0.3f,
                       std::chrono::seconds timeout_duration = std::chrono::seconds(60));
    void offboard_land();
    void request_ground_truth();


private:
    mavsdk::geometry::CoordinateTransformation get_coordinate_transformation();
    std::shared_ptr<mavsdk::MissionItem> create_mission_item(
        const mavsdk::geometry::CoordinateTransformation::LocalCoordinate& local_coordinate,
        const MissionOptions& mission_options,
        const mavsdk::geometry::CoordinateTransformation& ct);
    Telemetry::GroundTruth get_ground_truth_position();

    bool ground_truth_horizontal_position_close_to(const Telemetry::GroundTruth& target_pos, float acceptance_radius_m);
    bool estimated_position_close_to(const Offboard::PositionNEDYaw& target_position, float acceptance_radius_m);
    bool estimated_horizontal_position_close_to(const Offboard::PositionNEDYaw& target_pos, float acceptance_radius_m);

    mavsdk::Mavsdk _mavsdk{};
    std::unique_ptr<mavsdk::Telemetry> _telemetry{};
    std::unique_ptr<mavsdk::Action> _action{};
    std::unique_ptr<mavsdk::Mission> _mission{};
    std::unique_ptr<mavsdk::Offboard> _offboard{};

    Telemetry::GroundTruth _home{NAN, NAN, NAN};
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

inline float sq(float x) { return x * x; };
