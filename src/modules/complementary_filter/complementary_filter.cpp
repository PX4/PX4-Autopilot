#include "complementary_filter.hpp"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <cmath>

int ComplementaryFilter::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }
    PRINT_MODULE_DESCRIPTION("Custom Complementary Filter for Roll and Pitch estimation");
    PRINT_MODULE_USAGE_NAME("complementary_filter", "estimator");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}

int ComplementaryFilter::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int ComplementaryFilter::task_spawn(int argc, char *argv[])
{
    ComplementaryFilter *instance = new ComplementaryFilter(0, false);
    if (instance) {
        _object.store(instance); // Using the atomic store we fixed earlier
        _task_id = task_id_is_work_queue;
        instance->ScheduleOnInterval(4000); 
        return PX4_OK;
    } else {
        PX4_ERR("alloc failed");
        return -1;
    }
}

// Notice the constructor now initializes the ScheduledWorkItem queue:
ComplementaryFilter::ComplementaryFilter(int example_param, bool example_flag)
    : ModuleParams(nullptr),
      ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl) 
{
}

bool ComplementaryFilter::init()
{
    ScheduleOnInterval(4000); 
    return true;
}

void ComplementaryFilter::Run()
{
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    sensor_combined_s sensor_data;
    if (_sensor_sub.update(&sensor_data)) {
        
        float dt = 0.004f; 
        if (_last_timestamp != 0) {
            dt = (sensor_data.timestamp - _last_timestamp) / 1e6f;
        }
        _last_timestamp = sensor_data.timestamp;

        float ax = sensor_data.accelerometer_m_s2[0];
        float ay = sensor_data.accelerometer_m_s2[1];
        float az = sensor_data.accelerometer_m_s2[2];
        float wx = sensor_data.gyro_rad[0];
        float wy = sensor_data.gyro_rad[1];

        float roll_acc = atan2f(ay, -az);
        float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az));

        float roll = _tau * (_roll_prev + wx * dt) + (1.0f - _tau) * roll_acc;
        float pitch = _tau * (_pitch_prev + wy * dt) + (1.0f - _tau) * pitch_acc;

        _roll_prev = roll;
        _pitch_prev = pitch;

        debug_vect_s debug_msg{};
        debug_msg.timestamp = hrt_absolute_time();
        strncpy(debug_msg.name, "comp_filter", sizeof(debug_msg.name));
        debug_msg.x = roll;
        debug_msg.y = pitch;
        debug_msg.z = 0.0f;
        
        _debug_pub.publish(debug_msg);
    }
}

int complementary_filter_main(int argc, char *argv[])
{
    return ComplementaryFilter::main(argc, argv);
}