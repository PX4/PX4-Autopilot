#include "interceptor_control.hpp"
#include <px4_platform_common/log.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

using namespace matrix;

InterceptorControl::InterceptorControl() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
    parameters_update();
    _vehicle_local_position_sub.subscribe();
    _vehicle_angular_velocity_sub.subscribe();
    _sensor_accel_sub.subscribe();
    _actuator_servos_pub.advertise();
}

InterceptorControl::~InterceptorControl()
{
    // Cleanup if necessary
}

bool InterceptorControl::init()
{
    ScheduleOnInterval(10_ms);  // Runs every 10 milliseconds
    return true;
}

void InterceptorControl::set_accel_commands(float Ay, float Az)
{
	_accel_commands.Ay_cmd = Ay;
	_accel_commands.Az_cmd = Az;
	_accel_commands.valid = true;
}

bool InterceptorControl::get_accel_command(matrix::Vector3f &vec)
{
	if (!_accel_commands.valid) {
	    return false;
	}
	vec(0) = 0.0f;  // Assuming X acceleration is not used
	vec(1) = _accel_commands.Ay_cmd;
	vec(2) = _accel_commands.Az_cmd;
	return true;
}

void InterceptorControl::Run()
{
    if (should_exit()) {
        exit_and_cleanup();
        return;
    }

    Vector3f acceleration_commands;
    if (!get_accel_command(acceleration_commands)) {
        PX4_WARN("Failed to get acceleration commands.");
        return;
    }

    vehicle_local_position_s local_pos;
    if (!_vehicle_local_position_sub.update(&local_pos)) {
        PX4_WARN("Failed to get missile position/velocity.");
        return;
    }

    vehicle_angular_velocity_s angular_velocity;
    if (!_vehicle_angular_velocity_sub.update(&angular_velocity)) {
        PX4_WARN("Failed to get angular velocity.");
        return;
    }
    Vector3f gyro_latest(angular_velocity.xyz[0], angular_velocity.xyz[1], angular_velocity.xyz[2]);

    sensor_accel_s accel_data;
    if (!_sensor_accel_sub.update(&accel_data)) {
        PX4_WARN("Failed to get accelerometer data.");
        return;
    }
    Vector3f acc_latest(accel_data.x, accel_data.y, accel_data.z);

//     float pitch_rate_commanded;
//     normalAccController.step(acceleration_commands.z, acc_latest.data(), pitch_rate_commanded);

    float pitch_rate_body = gyro_latest(1);  // Index 1 corresponds to Y-axis

    float speed_magnitude = sqrtf(local_pos.vx * local_pos.vx + local_pos.vy * local_pos.vy + local_pos.vz * local_pos.vz);
    float elevator_deflection;
    InterceptorControl::_pitch_damper.step(0.0f, pitch_rate_body, elevator_deflection);

    float elevator_PWM = math::constrain((elevator_deflection * 500.0f / 7.0f) + 1500.0f, 1000.0f, 2000.0f);

//     float yaw_rate_command;
//     lateralAccController.step(acc_latest.y, acceleration_commands.y, gyro_latest.z, yaw_rate_command, plane.aparm.kp_lac, plane.aparm.ki_lac, plane.aparm.kd_lac);

    float rudder_deflection;
    _yaw_damper.step(0.0f, gyro_latest(2), speed_magnitude, rudder_deflection);

    float rudder_PWM = math::constrain((rudder_deflection * 500.0f / 7.0f) + 1500.0f, 1000.0f, 2000.0f);


    actuator_servos_s actuators{};
    actuators.timestamp = hrt_absolute_time();
    // Assign the servo values (elevator and rudder)
    actuators.control[0] = elevator_PWM;  // Servo index 0
    actuators.control[1] = rudder_PWM;    // Servo index 1
    // Publish to actuator_servos topic
    _actuator_servos_pub.publish(actuators);

    PX4_INFO("Pitch rate: %.4f, Pitch rate cmd: %.4f, Elevator: %.2f, Speed: %.2f", static_cast<double>(pitch_rate_body), static_cast<double>(0.0f), static_cast<double>(elevator_deflection), static_cast<double>(speed_magnitude));
}

void InterceptorControl::parameters_update() {
    // Update parameters if needed
}

int InterceptorControl::main(int argc, char *argv[])
{
    if (argc < 2) {
        print_usage("Missing command");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {
        if (_object.load() != nullptr) {
            PX4_WARN("Already running");
            return 1;
        }

        _object.store(new InterceptorControl());

        if (_object.load() == nullptr) {
            PX4_ERR("Allocation failed");
            return 1;
        }

        if (!_object.load()->init()) {
            delete _object.load();
            _object.store(nullptr);
            PX4_ERR("Init failed");
            return 1;
        }

        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        if (_object.load() == nullptr) {
            PX4_WARN("Not running");
            return 1;
        }
        delete _object.load();
        _object.store(nullptr);
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        PX4_INFO(_object.load() ? "Running" : "Not running");
        return 0;
    }

    return print_usage("Unknown command");
}

int InterceptorControl::custom_command(int argc, char *argv[])
{
    return print_usage("Unknown command");
}

int InterceptorControl::print_usage(const char *reason)
{
    PRINT_MODULE_USAGE_NAME("interceptor_control", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop the module");
    return 0;
}

extern "C" __EXPORT int interceptor_control_main(int argc, char *argv[])
{
    return InterceptorControl::main(argc, argv);
}
