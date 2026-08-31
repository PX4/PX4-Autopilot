#include "complementary_filter.hpp"

// Global pointer to the current instance for the trampoline
static ComplementaryFilter *instance = nullptr;

ComplementaryFilter::ComplementaryFilter()
{
    // Initialize Simulink model — sets up initial internal state
    _subsystem_model.dwork = &_subsystem_dwork;
    Subsystem_initialize(&_subsystem_model, _accel, _gyro, &_tau, &_roll_filtered, &_pitch_filtered);

    // Initialize the debug message once (name field doesn't change per cycle)
    memset(&_debug_data, 0, sizeof(_debug_data));
  strlcpy(_debug_data.name, "COMP_FILT", sizeof(_debug_data.name));
    _debug_data.name[sizeof(_debug_data.name) - 1] = '\0';
}

ComplementaryFilter::~ComplementaryFilter()
{
    Subsystem_terminate(&_subsystem_model);
    if (_imu_sub >= 0) {
        orb_unsubscribe(_imu_sub);
    }
}

int ComplementaryFilter::task_spawn_trampoline(int argc, char *argv[])
{
    if (instance) {
        instance->run();
    }
    return 0;
}

int ComplementaryFilter::start()
{
    if (_task_is_running) {
        PX4_WARN("Complementary Filter is already running");
        return -1;
    }

    _task_should_exit = false;
    _control_task = px4_task_spawn_cmd(
        "complementary_filter",
        SCHED_DEFAULT,
        SCHED_PRIORITY_DEFAULT,
        2048,
        (px4_main_t)&ComplementaryFilter::task_spawn_trampoline,
        nullptr
    );

    if (_control_task < 0) {
        PX4_ERR("Task start failed");
        return -1;
    }

    return 0;
}

void ComplementaryFilter::stop()
{
    _task_should_exit = true;
}

void ComplementaryFilter::run()
{
    _task_is_running = true;
    PX4_INFO("Complementary Filter started on vehicle_imu (calibrated data).");

    _imu_sub = orb_subscribe(ORB_ID(vehicle_imu));

    px4_pollfd_struct_t fds[] = {
        { .fd = _imu_sub, .events = POLLIN }
    };

    while (!_task_should_exit) {

        // Block up to 10ms waiting for a new IMU sample. NOTE: for the
        // integration inside the generated filter to stay accurate, the
        // vehicle_imu publish rate should match the Ts used when the
        // Simulink model was configured for code generation.
        int poll_ret = px4_poll(fds, 1, 10);

        if (poll_ret > 0 && (fds[0].revents & POLLIN)) {
            struct vehicle_imu_s imu_data;
            orb_copy(ORB_ID(vehicle_imu), _imu_sub, &imu_data);

            // Guard against division by zero on the very first sample.
            if (imu_data.delta_angle_dt > 0 && imu_data.delta_velocity_dt > 0) {

                float dt_angle    = imu_data.delta_angle_dt * 1e-6f;
                float dt_velocity = imu_data.delta_velocity_dt * 1e-6f;

                float gyro_x = imu_data.delta_angle[0] / dt_angle;
                float gyro_y = imu_data.delta_angle[1] / dt_angle;
                float gyro_z = imu_data.delta_angle[2] / dt_angle;

                float accel_x = imu_data.delta_velocity[0] / dt_velocity;
                float accel_y = imu_data.delta_velocity[1] / dt_velocity;
                float accel_z = imu_data.delta_velocity[2] / dt_velocity;

                // Sign flip to match the filter's frame convention
                _accel[0] = -accel_x;
                _accel[1] = -accel_y;
                _accel[2] = -accel_z;

                _gyro[0] = -gyro_x;
                _gyro[1] = -gyro_y;
                _gyro[2] = gyro_z;   // NOTE: not negated, unlike the other 5 axes — confirm this is intentional

                // Simulink step function — runs the filter for this cycle
                Subsystem_step(&_subsystem_model, _accel, _gyro, _tau, &_roll_filtered, &_pitch_filtered);

                _debug_data.timestamp = hrt_absolute_time();
                _debug_data.x = (float)_roll_filtered;
                _debug_data.y = (float)_pitch_filtered;
                _debug_data.z = 0.0f;

                if (_debug_pub == nullptr) {
                    _debug_pub = orb_advertise(ORB_ID(debug_vect), &_debug_data);
                } else {
                    orb_publish(ORB_ID(debug_vect), _debug_pub, &_debug_data);
                }
            }
        }
    }

    PX4_INFO("Complementary Filter stopped.");
    _task_is_running = false;
}

// =========================================================================
// Main Entry Point for the PX4 Module
// =========================================================================
extern "C" __EXPORT int complementary_filter_main(int argc, char *argv[])
{
    if (argc < 2) {
        PX4_WARN("usage: complementary_filter {start|stop|status}");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {
        if (instance != nullptr) {
            PX4_WARN("already running");
            return 1;
        }
        instance = new ComplementaryFilter();
        if (instance == nullptr) {
            PX4_ERR("Memory allocation failed");
            return 1;
        }
        instance->start();
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        if (instance == nullptr) {
            PX4_WARN("not running");
            return 1;
        }
        instance->stop();
        // Wait briefly for the task thread to exit cleanly before deleting
        px4_usleep(20000);
        delete instance;
        instance = nullptr;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (instance != nullptr && instance->is_running()) {
            PX4_INFO("running");
        } else {
            PX4_INFO("not running");
        }
        return 0;
    }

    PX4_WARN("unrecognized command");
    return 1;
}
