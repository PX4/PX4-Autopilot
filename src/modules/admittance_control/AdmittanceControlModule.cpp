/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file AdmittanceControlModule.cpp
 * @brief RLS parameter identification and external wrench estimator
 *
 * @author Pedro Mendes <pmen817@aucklanduni.ac.nz>
 */

#include "AdmittanceControlModule.hpp"

AdmittanceControlModule::AdmittanceControlModule() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	updateParams();
}

AdmittanceControlModule::~AdmittanceControlModule()
{
	perf_free(_cycle_perf);
}

bool AdmittanceControlModule::init()
{
	// execute Run() on every rls_wrench_estimator publication
	if (!_rls_wrench_estimator_sub.registerCallback()) {
		PX4_ERR("rls_wrench_estimator callback registration failed");
		return false;
	}

	//limit to 100Hz
	_rls_wrench_estimator_sub.set_interval_us(10_ms);

	return true;
}

void AdmittanceControlModule::updateParams()
{
	ModuleParams::updateParams();

	BellParameters params_bell;

	params_bell.A[0] = _param_adm_ctr_ax.get();
	params_bell.A[1] = _param_adm_ctr_ay.get();
	params_bell.A[2] = _param_adm_ctr_az.get();
	params_bell.A[3] = _param_adm_ctr_ayaw.get();

	params_bell.B1[0] = _param_adm_ctr_b1x.get();
	params_bell.B1[1] = _param_adm_ctr_b1y.get();
	params_bell.B1[2] = _param_adm_ctr_b1z.get();
	params_bell.B1[3] = _param_adm_ctr_b1yaw.get();

	params_bell.B2[0] = _param_adm_ctr_b2x.get();
	params_bell.B2[1] = _param_adm_ctr_b2y.get();
	params_bell.B2[2] = _param_adm_ctr_b2z.get();
	params_bell.B2[3] = _param_adm_ctr_b2yaw.get();

	params_bell.B3[0] = _param_adm_ctr_b3x.get();
	params_bell.B3[1] = _param_adm_ctr_b3y.get();
	params_bell.B3[2] = _param_adm_ctr_b3z.get();
	params_bell.B3[3] = _param_adm_ctr_b3yaw.get();

	params_bell.M_min[0] = _param_adm_ctr_mminx.get();
	params_bell.M_min[1] = _param_adm_ctr_mminy.get();
	params_bell.M_min[2] = _param_adm_ctr_mminz.get();
	params_bell.M_min[3] = _param_adm_ctr_mminyaw.get();

	params_bell.K_min[0] = _param_adm_ctr_kminx.get();
	params_bell.K_min[1] = _param_adm_ctr_kminy.get();
	params_bell.K_min[2] = _param_adm_ctr_kminz.get();
	params_bell.K_min[3] = _param_adm_ctr_kminyaw.get();

	params_bell.M_max[0] = _param_adm_ctr_mmaxx.get();
	params_bell.M_max[1] = _param_adm_ctr_mmaxy.get();
	params_bell.M_max[2] = _param_adm_ctr_mmaxz.get();
	params_bell.M_max[3] = _param_adm_ctr_mmaxyaw.get();

	params_bell.K_max[0] = _param_adm_ctr_kmaxx.get();
	params_bell.K_max[1] = _param_adm_ctr_kmaxy.get();
	params_bell.K_max[2] = _param_adm_ctr_kmaxz.get();
	params_bell.K_max[3] = _param_adm_ctr_kmaxyaw.get();

	params_bell.lpf_sat_factor = _param_adm_ctr_lpf.get();


	_control.initialize(params_bell);
}

void AdmittanceControlModule::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (!_rls_wrench_estimator_sub.updated()) {
		return;
	}

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}



	//all inputs required for each step
	if (!_actuator_outputs_sub.updated() || !_vehicle_attitude_setpoint_sub.updated()) {
		return;
	}

	perf_begin(_cycle_perf);

	rls_wrench_estimator_s wrench;
	actuator_outputs_s actuator_outputs;
	vehicle_attitude_setpoint_s v_att_sp;
	vehicle_local_position_setpoint_s setpoint;


	_sp_updated = _trajectory_setpoint_sub.updated();

	_finite = copyAndCheckAllFinite(wrench, actuator_outputs, v_att_sp, setpoint);

	const float dt = (wrench.timestamp - _timestamp_last) * 1e-6f;
	_timestamp_last = wrench.timestamp;

	vehicle_local_position_setpoint_s admittance_sp{};


	if (_debug_vect_sub.updated()) {
		debug_vect_s flags_vect;
		_debug_vect_sub.copy(&flags_vect);
		_admittance_flag = (flags_vect.y > 0.5f);
		_target_dist = (flags_vect.z);

		_debug_timestamp_last = hrt_absolute_time();
	}

	if (hrt_elapsed_time(&_debug_timestamp_last) > 1_s) {
		_admittance_flag = false; //timeout case external link lost
		_target_dist = 99;
	}

	// Guard against too small (< 1ms) and too large (> 1000ms) dt's.
	if (_finite && _admittance_flag && (dt > 0.001f) && (dt < 1.f)) {

		Vector<float, 4> We = zeros<float, 4, 1>();

		// //Saturate and Deadzone
		We(0) = math::constrain(((fabsf(wrench.fe[0]) > _param_adm_ctr_dzx.get()) ? (wrench.fe[0]) : (0.f)), -_param_adm_ctr_sax.get(), _param_adm_ctr_sax.get());
		We(1) = math::constrain(((fabsf(wrench.fe[1]) > _param_adm_ctr_dzy.get()) ? (wrench.fe[1]) : (0.f)), -_param_adm_ctr_say.get(), _param_adm_ctr_say.get());
		We(2) = math::constrain(((fabsf(wrench.fe[2]) > _param_adm_ctr_dzz.get()) ? (wrench.fe[2]) : (0.f)), -_param_adm_ctr_saz.get(), _param_adm_ctr_saz.get());
		We(3) = math::constrain(((fabsf(wrench.me[2]) > _param_adm_ctr_dzw.get()) ? (wrench.me[2]) : (0.f)), -_param_adm_ctr_saw.get(), _param_adm_ctr_saw.get());

		float pwm[8] = {
			actuator_outputs.output[0],
			actuator_outputs.output[1],
			actuator_outputs.output[2],
			actuator_outputs.output[3],
			actuator_outputs.output[4],
			actuator_outputs.output[5],
			actuator_outputs.output[6],
			actuator_outputs.output[7]
		};

		if (_param_rls_n_rotors.get() <= 4) {
			pwm[4] = 0.f;
			pwm[5] = 0.f;
			pwm[6] = 0.f;
			pwm[7] = 0.f;
		}

		const matrix::Vector<float, 8> output =  matrix::Vector<float, 8>(pwm);
		const matrix::Quatf q{v_att_sp.q_d};

		if (!_valid) {
			_control.reset(setpoint);
		}

		_control.update(dt, We, output, _target_dist, q, setpoint);

		admittance_sp = _control.getSetpoints();
		_valid = true;


	} else {
		admittance_sp = setpoint;
		_valid = false;
	}

	perf_end(_cycle_perf);

	//Only publish when new setpoints are updated;
	if (_sp_updated) {
		_admittance_setpoint_pub.publish(admittance_sp);
	}

}

bool AdmittanceControlModule::copyAndCheckAllFinite(rls_wrench_estimator_s &wrench, actuator_outputs_s &actuator_outputs,
						vehicle_attitude_setpoint_s &v_att_sp, vehicle_local_position_setpoint_s &sp)
{

	_rls_wrench_estimator_sub.copy(&wrench);

	if (!(PX4_ISFINITE(wrench.fe[0]) && PX4_ISFINITE(wrench.fe[1]) && PX4_ISFINITE(wrench.fe[2])
		&& PX4_ISFINITE(wrench.me[0]) && PX4_ISFINITE(wrench.me[1]) && PX4_ISFINITE(wrench.me[2]))) {

		return false;
	}


	_vehicle_attitude_setpoint_sub.copy(&v_att_sp);

	if (!(PX4_ISFINITE(v_att_sp.q_d[0]) && PX4_ISFINITE(v_att_sp.q_d[1]) && PX4_ISFINITE(v_att_sp.q_d[2]) && PX4_ISFINITE(v_att_sp.q_d[3]))) {

		return false;
	}


	_actuator_outputs_sub.copy(&actuator_outputs);

	for (int i = 0; i < 8; i++) {
		if (!PX4_ISFINITE(actuator_outputs.output[i])) {return false;}
	}


	_trajectory_setpoint_sub.copy(&sp);

	if (!(PX4_ISFINITE(sp.x) && PX4_ISFINITE(sp.y) && PX4_ISFINITE(sp.z) && PX4_ISFINITE(sp.yaw))) {
		return false;
	}

	if (!(PX4_ISFINITE(sp.vx) && PX4_ISFINITE(sp.vy) && PX4_ISFINITE(sp.vz) && PX4_ISFINITE(sp.yawspeed))) {
		sp.vx = 0.f;
		sp.vy = 0.f;
		sp.vz = 0.f;
		sp.yawspeed = 0.f;
	}

	if (!(PX4_ISFINITE(sp.acceleration[0]) && PX4_ISFINITE(sp.acceleration[1]) && PX4_ISFINITE(sp.acceleration[2]))) {
		sp.acceleration[0] = 0.f;
		sp.acceleration[1] = 0.f;
		sp.acceleration[2] = 0.f;
	}

	return true;
}



int AdmittanceControlModule::task_spawn(int argc, char *argv[])
{
	AdmittanceControlModule *instance = new AdmittanceControlModule();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int AdmittanceControlModule::print_status()
{
	perf_print_counter(_cycle_perf);
	return 0;
}

int AdmittanceControlModule::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int AdmittanceControlModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Admittance Controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("admittance_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int admittance_control_main(int argc, char *argv[])
{
	return AdmittanceControlModule::main(argc, argv);
}
