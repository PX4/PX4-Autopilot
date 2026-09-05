#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/debug_vect.h>
#include <matrix/math.hpp>

extern "C" __EXPORT int complementary_filter_main(int argc, char *argv[]);

// Notice the added 'public px4::ScheduledWorkItem' here:
class ComplementaryFilter : public ModuleBase<ComplementaryFilter>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    ComplementaryFilter(int example_param, bool example_flag);
    ~ComplementaryFilter() override = default;

    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();

private:
    void Run() override; // Added override for the scheduled work item

    // uORB Subscriptions and Publications
    uORB::Subscription _sensor_sub{ORB_ID(sensor_combined)};
    uORB::Publication<debug_vect_s> _debug_pub{ORB_ID(debug_vect)};

    // Filter variables
    float _roll_prev{0.0f};
    float _pitch_prev{0.0f};
    uint64_t _last_timestamp{0};
    
    // Tuning parameter tau
    const float _tau = 0.98f; 
};