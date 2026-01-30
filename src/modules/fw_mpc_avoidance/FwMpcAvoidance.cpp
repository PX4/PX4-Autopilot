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
	_controller.configure(_param_fw_mpc_avoid_dt.get(), FwMpcController::kMaxHorizon);
	const matrix::Vector3f I_diag{_param_sih_ixx.get(), _param_sih_iyy.get(), _param_sih_izz.get()};
	_controller.set_vehicle_params(_param_sih_mass.get(), I_diag, _param_sih_kdv.get(), _param_sih_kdw.get());

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

		if (_controller.configure(_param_fw_mpc_avoid_dt.get(), FwMpcController::kMaxHorizon)) {
			_mpc_ready = false;

		} else {
			PX4_ERR("fw mpc config failed");
		}

		const matrix::Vector3f I_diag{_param_sih_ixx.get(), _param_sih_iyy.get(), _param_sih_izz.get()};
		_controller.set_vehicle_params(_param_sih_mass.get(), I_diag, _param_sih_kdv.get(), _param_sih_kdw.get());
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
	vehicle_local_position_setpoint_s lpos_sp{};

	bool have_lat = _lat_sp_sub.copy(&lat_sp);
	bool have_lon = _lon_sp_sub.copy(&lon_sp);
	const bool have_goal = _lpos_sp_sub.copy(&lpos_sp);

	if (_param_fw_mpc_avoid_en.get()) {
		vehicle_attitude_s att{};
		vehicle_angular_velocity_s rates{};
		vehicle_local_position_s lpos{};

		const bool have_state = _att_sub.copy(&att) && _rates_sub.copy(&rates) && _lpos_sub.copy(&lpos);

		if (have_state && have_goal) {
			const matrix::Quatf q(att.q);
			const matrix::Dcmf R_nb{q};
			const matrix::Vector3f vel_N{lpos.vx, lpos.vy, lpos.vz};
			const matrix::Vector3f vel_B = R_nb.transpose() * vel_N;
			const matrix::Eulerf euler(q);

			FwMpcController::StateVec x_now{};
			x_now(0) = vel_B(0);
			x_now(1) = vel_B(1);
			x_now(2) = vel_B(2);
			x_now(3) = rates.xyz[0];
			x_now(4) = rates.xyz[1];
			x_now(5) = rates.xyz[2];
			x_now(6) = euler.phi();
			x_now(7) = euler.theta();
			x_now(8) = euler.psi();
			x_now(9) = lpos.x;
			x_now(10) = lpos.y;
			x_now(11) = -lpos.z; // up

			const matrix::Vector3f goal_up{lpos_sp.x, lpos_sp.y, -lpos_sp.z};

			if (!_mpc_ready) {
				_controller.initTrim(13.f, x_now(11), goal_up);
				_mpc_ready = true;
			}

			FwMpcController::ControlVec u_cmd{};
			FwMpcController::StateVec x_pred{};
			const float V_cruise = math::max(vel_N.norm(), 8.f);

			if (_controller.step(x_now, goal_up, V_cruise, false, u_cmd, x_pred)) {
				const float phi_cmd = x_pred(6);
				const float theta_cmd = x_pred(7);
				const float throttle_norm = math::constrain(u_cmd(3) / _controller.limits().u_max(3), 0.f, 1.f);

				lat_sp.timestamp = now;
				lat_sp.course = NAN;
				lat_sp.airspeed_direction = NAN;
				lat_sp.lateral_acceleration = CONSTANTS_ONE_G * tanf(phi_cmd);

				lon_sp.timestamp = now;
				lon_sp.altitude = NAN;
				lon_sp.height_rate = NAN;
				lon_sp.equivalent_airspeed = NAN;
				lon_sp.pitch_direct = theta_cmd;
				lon_sp.throttle_direct = throttle_norm;
				have_lat = true;
				have_lon = true;
			}

		} else {
			// Fallback: integrate internal model to keep nominal state bounded.
			step_internal_model(math::max(dt, _param_fw_mpc_avoid_dt.get()));
		}
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
