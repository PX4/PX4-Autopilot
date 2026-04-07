#include "MulticopterDataAnalysis.hpp"
#include <px4_platform_common/events.h>

using namespace time_literals;

ModuleBase::Descriptor MulticopterDataAnalysis::desc{task_spawn, custom_command, print_usage};

MulticopterDataAnalysis::MulticopterRateControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_vehicle_thrust_setpoint_pub(vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_mc) : ORB_ID(vehicle_thrust_setpoint)),
	_vehicle_torque_setpoint_pub(vtol ? ORB_ID(vehicle_torque_setpoint_virtual_mc) : ORB_ID(vehicle_torque_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
	_controller_status_pub.advertise();
}

MulticopterDataAnalysis::~MulticopterDataAnalysis()
{
	perf_free(_loop_perf);
}

bool MulticopterDataAnalysis::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void MulticopterDataAnalysis::Run()
{   
    // Check on this
    if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

    perf_begin(_loop_perf);

    // get the latest data from the topics we're subscribed to (if updated)
    if (_parameter_update_sub.updated()) {
        // clear update        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);
    }

    //check if multicopter is is in the air


    // if in the air, log the data we want to analyze




    perf_end(_loop_perf);

}

void MulticopterDataAnalysis::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int MulticopterDataAnalysis::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
        MulticopterDataAnalysis is a module for analyzing multicopter flight data. It subscribes to various topics related to the multicopter's state and control, and can be used to log data);
        )DESCR_STR");
    }

void MulticopterDataAnalysis::checkTopicUpdates(){
    // get the latest data from the topics we're subscribed to (if updated) and store them in member variables for analysis
    if (_vehicle_control_mode_sub.updated()) {
        _vehicle_control_mode_sub.copy(&_vehicle_control_mode);
    }
}

    




