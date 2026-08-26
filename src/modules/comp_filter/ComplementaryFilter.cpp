#include "ComplementaryFilter.hpp"

using math::constrain;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

#define GYRO_ANGS_WEIGHT 0.998f

CompFilter::CompFilter(const px4::wq_config_t &config):
	ModuleBase(), ScheduledWorkItem(MODULE_NAME, config)
{

}

bool CompFilter::init()
	{
		if (!_sensor_combined_sub.registerCallback()) {
			PX4_ERR("Failed to register sensor_combined callback");
			return false;
		}

		// Schedule the first run immediately to get things moving
		ScheduleNow();
		return true;
	}


void CompFilter::Run()
{
	if (should_exit()) {
        _sensor_combined_sub.unregisterCallback();
        exit_and_cleanup();
        return;
    }

	imuSample imu_sample_new {};
	sensor_combined_s sensor_combined;


	if (_sensor_combined_sub.update(&sensor_combined)) {
		imu_sample_new.time_us = sensor_combined.timestamp;
		imu_sample_new.dt = sensor_combined.gyro_integral_dt * 1.e-6f;
		imu_sample_new.gyro = Vector3f{sensor_combined.gyro_rad};
		imu_sample_new.acc = Vector3f{sensor_combined.accelerometer_m_s2};

		updateGyroAngs(imu_sample_new.gyro, imu_sample_new.dt);
		updateAccAngs(imu_sample_new.acc);

		_angles = GYRO_ANGS_WEIGHT * _gyro_angles + (1.f - GYRO_ANGS_WEIGHT) * _acc_angles;

		// 1. Create a blank message struct
		comp_filter_status_s status_msg{};
		
		// 2. Set the timestamp
		status_msg.timestamp = hrt_absolute_time(); 

		// 3. Copy your matrix::Vector3f data into the message arrays
		_pure_gyro_angles.copyTo(status_msg.gyro_angles);
		_acc_angles.copyTo(status_msg.acc_angles);
		_angles.copyTo(status_msg.comp_angles);

		// publish the message
		_status_pub.publish(status_msg);
	}

	

}

void CompFilter::updateGyroAngs(const Vector3f &gyro,const float &dt)
{
    float phi = _angles(0);
    float theta = _angles(1);
    // float psi = _angles(2);
    float p = gyro(0);
    float q = gyro(1);
    float r = gyro(2);
    float phi_dot = p + q*sinf(phi)*tanf(theta) + cosf(phi)*tanf(theta)*r;
    float theta_dot = cosf(phi)*q - sinf(phi)*r;
    float psi_dot = (sinf(phi)/cosf(theta))*q + (cosf(phi)/cosf(theta))*r;
    Vector3f gyro_rotated({phi_dot,theta_dot,psi_dot});
	_gyro_angles = _angles + gyro_rotated * dt;
	_pure_gyro_angles = _pure_gyro_angles + gyro_rotated * dt;
}

void CompFilter::updateAccAngs(const Vector3f &acc)
{
	_acc_angles(0) = atan2f(-acc(1), -acc(2));
	_acc_angles(1) = atan2f(acc(0), sqrtf(acc(1) * acc(1) + acc(2) * acc(2)));
	_acc_angles(2) = 0.f;
}

CompFilter::~CompFilter()
{
    // 1. Ensure the uORB callback is unregistered so the scheduler 
    // doesn't try to wake up a destroyed object.
    _sensor_combined_sub.unregisterCallback();

    // 2. If you added any performance counters (perf_alloc) later, 
    // you would free them here using perf_free().
}


// --- ModuleBase Required Functions ---

CompFilter *CompFilter::instantiate(int argc, char *argv[])
{
    CompFilter *instance = new CompFilter(px4::wq_configurations::INS0);
    if (instance) {
        // ... you could parse CLI arguments here if you wanted to ...
    }
    return instance;
}

int CompFilter::task_spawn(int argc, char *argv[])
{
    CompFilter *instance = instantiate(argc, argv);

    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;

        if (instance->init()) {
            return PX4_OK;
        } else {
            PX4_ERR("init failed");
            instance->request_stop();
            instance->ScheduleNow();
            return PX4_ERROR;
        }
    }
    return PX4_ERROR;
}

int CompFilter::custom_command(int argc, char *argv[])
{
    return print_usage("Unrecognized command");
}

int CompFilter::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }
    PRINT_MODULE_DESCRIPTION("Complementary Filter for IMU data");
    PRINT_MODULE_USAGE_NAME("comp_filter", "system");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}

// This macro generates the "comp_filter_main" function for you automatically!
// It connects your class to the PX4 terminal.
extern "C" __EXPORT int comp_filter_main(int argc, char *argv[])
{
    return CompFilter::main(argc, argv);
}
