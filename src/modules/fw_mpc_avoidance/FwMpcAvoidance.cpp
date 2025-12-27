/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team.
 *
 ****************************************************************************/

#include "FwMpcAvoidance.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>

using matrix::Quatf;
using matrix::Vector3f;

const matrix::Vector3f FwMpcDynamics::_I_diag{0.02f, 0.02f, 0.04f};
const matrix::SquareMatrix<float, 3> FwMpcDynamics::_I = matrix::diag(FwMpcDynamics::_I_diag);
const matrix::SquareMatrix<float, 3> FwMpcDynamics::_I_inv = matrix::diag(matrix::Vector3f{1.f / 0.02f, 1.f / 0.02f, 1.f / 0.04f});

FwMpcAvoidance::FwMpcAvoidance() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

bool FwMpcAvoidance::init()
{
	if (!_lpos_sub.registerCallback()) {
		PX4_ERR("vehicle_local_position callback registration failed");
		return false;
	}

	_last_run = hrt_absolute_time();
	ScheduleNow();
	return true;
}

void FwMpcAvoidance::parameters_update()
{
	if (_param_update_sub.updated()) {
		parameter_update_s p{};
		_param_update_sub.copy(&p);
		updateParams();
	}
}

void FwMpcAvoidance::step_internal_model(const float dt)
{
	vehicle_attitude_s att{};
	vehicle_angular_velocity_s rates{};
	vehicle_local_position_s lpos{};
	wind_s wind{};

	if (_att_sub.copy(&att) && _rates_sub.copy(&rates) && _lpos_sub.copy(&lpos)) {
		FwMpcDynamics::State s{};
		s.q_nb = Quatf(att.q);
		s.omega_B = Vector3f{rates.xyz[0], rates.xyz[1], rates.xyz[2]};
		s.velocity_N = Vector3f{lpos.vx, lpos.vy, lpos.vz};
		s.position_N = Vector3f{lpos.x, lpos.y, lpos.z};
		_dynamics.reset(s);
	}

	_wind_sub.copy(&wind);
	Vector3f wind_B{wind.windspeed_north, wind.windspeed_east, 0.0f};
	// In absence of MPC solution, use zero moments and zero thrust; this keeps state integration bounded.
	_dynamics.propagate(Vector3f{}, Vector3f{}, wind_B, dt);
}

void FwMpcAvoidance::Run()
{
	if (should_exit()) {
		_lpos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	parameters_update();

	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain((now - _last_run) * 1e-6f, 0.001f, 0.1f);
	_last_run = now;

	fixed_wing_lateral_setpoint_s lat_sp{};
	fixed_wing_longitudinal_setpoint_s lon_sp{};

	const bool have_lat = _lat_sp_sub.copy(&lat_sp);
	const bool have_lon = _lon_sp_sub.copy(&lon_sp);

	if (_param_fw_mpc_avoid_en.get()) {
		// Placeholder: integrate internal model so MPC rollout code can be added here later.
		step_internal_model(math::max(dt, _param_fw_mpc_avoid_dt.get()));
		// TODO: plug MPC optimizer and override lat_sp / lon_sp when collision predicted.
	}

	// Passthrough (or future override) publisher.
	if (have_lat) {
		_lat_sp_pub.publish(lat_sp);
	}

	if (have_lon) {
		_lon_sp_pub.publish(lon_sp);
	}

	ScheduleDelayed(20000); // 20 ms
}

int FwMpcAvoidance::task_spawn(int argc, char *argv[])
{
	FwMpcAvoidance *instance = new FwMpcAvoidance();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("allocation failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int FwMpcAvoidance::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FwMpcAvoidance::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_USAGE_NAME("fw_mpc_avoidance", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	return 0;
}

extern "C" __EXPORT int fw_mpc_avoidance_main(int argc, char *argv[])
{
	return FwMpcAvoidance::main(argc, argv);
}
