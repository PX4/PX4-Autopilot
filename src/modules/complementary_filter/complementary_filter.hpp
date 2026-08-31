#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_imu.h>   // Calibrated, synchronized IMU data (same source EKF2 uses)
#include <uORB/topics/debug_vect.h>
#include <string.h>

extern "C" {
    #include "Subsystem.h"             // Simulink-generated complementary filter
}

class ComplementaryFilter
{
public:
    ComplementaryFilter();
    ~ComplementaryFilter();

    // Main task logic (the per-cycle loop lives in the .cpp)
    void run();

    // Task control
    int  start();
    void stop();
    bool is_running() const { return _task_is_running; }

    // Trampoline for PX4 task spawning (bridges the C-style task
    // entry point PX4 expects to this C++ instance's run() method)
    static int task_spawn_trampoline(int argc, char *argv[]);

private:
    bool _task_should_exit{false};
    bool _task_is_running{false};
    int  _control_task{-1};

    // uORB subscriber (input) and publisher (output)
    int             _imu_sub{-1};
    orb_advert_t    _debug_pub{nullptr};
    struct debug_vect_s _debug_data{};

    // Simulink model state — this is the filter's persistent memory
    // between cycles (equivalent to "previous_angle" for roll/pitch,
    // held inside the generated DWork struct's Unit Delay states)
    RT_MODEL_Subsystem_T _subsystem_model{};
    DW_Subsystem_T       _subsystem_dwork{};

    real_T _accel[3]{0.0, 0.0, 0.0};
    real_T _gyro[3]{0.0, 0.0, 0.0};
    real_T _tau{0.98};                 // complementary filter weighting coefficient
    real_T _roll_filtered{0.0};
    real_T _pitch_filtered{0.0};
};
