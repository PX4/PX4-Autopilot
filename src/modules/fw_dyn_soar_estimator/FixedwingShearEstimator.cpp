/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include "FixedwingShearEstimator.hpp"

using math::constrain;
using math::max;
using math::min;
using math::radians;
using matrix::Matrix;
using matrix::Vector3f;
using matrix::Vector;
using matrix::wrap_pi;
using matrix::geninv;



FixedwingShearEstimator::FixedwingShearEstimator() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::test1),
    _soaring_estimator_shear_pub(ORB_ID(soaring_estimator_shear)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	// limit to 10 Hz
	_soaring_controller_wind_sub.set_interval_ms(100.f);

	/* fetch initial parameter values */
	parameters_update();
}

FixedwingShearEstimator::~FixedwingShearEstimator()
{
	perf_free(_loop_perf);
}

bool
FixedwingShearEstimator::init()
{
	if (!_soaring_controller_wind_sub.registerCallback()) {
		PX4_ERR("vehicle position callback registration failed!");
		return false;
	}
    PX4_INFO("Starting FW_DYN_SOAR_ESTIMATOR");

    // normalization variables
    _unit_v = 1.f;
    _unit_h = 1.f;
    _unit_a = 1.f;

    parameters_update();

    // init horizontal wind field
    for (uint i=0; i<6; i++){
        _X_prior_horizontal(i) = 0.f;
        _X_posterior_horizontal(i) = 0.f;
        for (uint j=0; j<6; j++){
            _A_horizontal(i,j) = 0.f;
        }
    }
    _P_prior_horizontal = _Q_horizontal;
    _P_posterior_horizontal = _Q_horizontal;

    // init vertical wind field
    for (uint i=0; i<_dim_vertical; i++){
        _X_prior_vertical(i) = 0.f;
        _X_posterior_vertical(i) = 0.f;
        for (uint j=0; j<_dim_vertical; j++){
            _A_vertical(i,j) = 0.f;
        }
    }
    _P_prior_vertical = _Q_vertical;
    _P_posterior_vertical = _Q_vertical;


    //
    _X_prior_horizontal(4) = _init_height;
    _X_posterior_horizontal(4) = _init_height;
    _X_prior_horizontal(5) = 0.5f/_unit_a;
    _X_posterior_horizontal(5) = 0.5f/_unit_a;

    // init time
    _last_run = hrt_absolute_time();

    // init reset counter
    _reset_counter = 0;

    return true;
}

int
FixedwingShearEstimator::parameters_update()
{
	updateParams();

	// update params...
    _Q_horizontal.setZero();
    _Q_horizontal(0,0) = powf(_param_sigma_q_vel.get()/_unit_v,2);
    _Q_horizontal(1,1) = powf(_param_sigma_q_vel.get()/_unit_v,2);
    _Q_horizontal(2,2) = powf(_param_sigma_q_vel.get()/_unit_v,2);
    _Q_horizontal(3,3) = powf(_param_sigma_q_vel.get()/_unit_v,2);
    _Q_horizontal(4,4) = powf(_param_sigma_q_h.get()/_unit_h,2);
    _Q_horizontal(5,5) = powf(_param_sigma_q_a.get()/_unit_a,2);

    _R_horizontal.setZero();
    _R_horizontal(0,0) = powf(_param_sigma_r_vel.get()/_unit_v,2);
    _R_horizontal(1,1) = powf(_param_sigma_r_vel.get()/_unit_v,2);

    for (int i=0;i<(int)_dim_vertical;i++){
        _Q_vertical(i,i) = powf(_param_sigma_q_vel.get(),2);
    }

    _R_vertical(0,0) = powf(_param_sigma_r_vel.get(),2);

    _init_height = _param_init_h.get()/_unit_h;

	return PX4_OK;
}

void
FixedwingShearEstimator::reset_filter()
{
    // reset all states of the filter to some initial guess.

    // reset horizontal wind state
    for (uint i=0;i<6;i++){
        _X_prior_horizontal(i) = 0.0f;
        _X_posterior_horizontal(i) = 0.0f;
    }
    _P_prior_horizontal = _Q_horizontal;
    _P_posterior_horizontal = _Q_horizontal;

    // reset vertical wind state
    for (uint i=0;i<_dim_vertical;i++){
        _X_prior_vertical(i) = 0.0f;
        _X_posterior_vertical(i) = 0.0f;
    }
    _P_prior_vertical = _Q_vertical;
    _P_posterior_vertical = _Q_vertical;

    // set height to enable convergence
    _X_prior_horizontal(4) = _init_height;
    _X_posterior_horizontal(4) = _init_height;

    // increment counter
    _reset_counter += 1;
}

void
FixedwingShearEstimator::perform_prior_update()
{
    // get time since last run
    float dt = (float)(hrt_absolute_time() - _last_run)/1000000.f;
    _last_run = hrt_absolute_time();
    if (dt>10.f) {
        dt = 10.f;
    }

    // perform prior update assuming trivial dynamics of the wind field (mean field stays the same)
    _X_prior_horizontal = _X_posterior_horizontal;
    _X_prior_vertical = _X_posterior_vertical;
    _P_prior_horizontal = _P_posterior_horizontal + dt*_Q_horizontal;
    _P_prior_vertical = _P_posterior_vertical + dt*_Q_vertical;
}

void
FixedwingShearEstimator::perform_posterior_update(float height, Vector3f wind)
{
    //
    bool error = false;

    // first fill the horizontal observation matrix:
    float vx = _X_prior_horizontal(0);
    float vy = _X_prior_horizontal(1);
    //float bx = _X_prior_horizontal(1);
    //float by = _X_prior_horizontal(3);
    float h = _X_prior_horizontal(4);
    float a = _X_prior_horizontal(5);

    _H_horizontal.setZero();
    _H_horizontal(0,0) = 1.f/(1.f + expf(-(height-h)*a));
    _H_horizontal(1,1) = 1.f/(1.f + expf(-(height-h)*a));
    _H_horizontal(0,2) = 1.f;
    _H_horizontal(1,3) = 1.f;
    _H_horizontal(0,4) = -((a*vx*expf(-(height-h)*a)))/powf((1.f + expf(-(height-h)*a)),2.f);
    _H_horizontal(1,4) = -((a*vy*expf(-(height-h)*a)))/powf((1.f + expf(-(height-h)*a)),2.f);
    _H_horizontal(0,5) = -((vx*(h-height))*expf(-(height-h)*a))/powf((1.f + expf(-(height-h)*a)),2.f);
    _H_horizontal(1,5) = -((vy*(h-height))*expf(-(height-h)*a))/powf((1.f + expf(-(height-h)*a)),2.f);

    // then fill the vertical observation matrix
    for (uint i=0;i<_dim_vertical;i++){
        _H_vertical(0,i) = powf(height,i);
    }

    // compute Kalman gain matrix for horizontal wind states
    Matrix<float, 6, 2> tmp1_horizontal = _P_prior_horizontal*_H_horizontal.T();
    Matrix<float, 2, 2> tmp2_horizontal = _H_horizontal*(_P_prior_horizontal*_H_horizontal.T()) + _R_horizontal;
    float determinant_horizontal = tmp2_horizontal(0,0)*tmp2_horizontal(1,1) - tmp2_horizontal(1,0)*tmp2_horizontal(0,1);
    if (fabs(determinant_horizontal)>0.000001f){
        _K_horizontal = tmp1_horizontal*geninv(tmp2_horizontal);
    }
    else{
        PX4_WARN("singular horizontal matrix, resetting filter");
        error = true;
    }

    // then fill the vertical observation matrix
    for (uint i=0;i<_dim_vertical;i++){
        _H_vertical(0,i) = powf(height-h,i);
    }

    // compute Kalman gain matrix for vertical wind states
    Matrix<float, _dim_vertical, 1> tmp1_vertical = _P_prior_vertical*_H_vertical.T();
    Matrix<float, 1, 1> tmp2_vertical = _H_vertical*_P_prior_vertical*_H_vertical.T() + _R_vertical;
    Matrix<float, 1, 1> inv_vertical;
    float determinant_vertical = tmp2_vertical(0,0);
    if (fabs(determinant_vertical)>0.000001f){
        inv_vertical(0,0) = 1.f / determinant_vertical;
        _K_vertical = tmp1_vertical*inv_vertical;
    }
    else{
        PX4_WARN("singular vertical matrix, resetting filter");
        error = true;
    }

    if (error) {
        reset_filter();
    }
    else {
        // perform horizontal update
        Vector<float, 2> z_expected_horizontal;
        Vector<float, 2> wind_horizontal;
        Matrix<float, 6, 6> identity_1;
        z_expected_horizontal(0) = _X_prior_horizontal(0)/(1.f+expf(-_X_prior_horizontal(5)*(_current_height-_X_prior_horizontal(4)))) + _X_prior_horizontal(2);
        z_expected_horizontal(1) = _X_prior_horizontal(1)/(1.f+expf(-_X_prior_horizontal(5)*(_current_height-_X_prior_horizontal(4)))) + _X_prior_horizontal(3);
        z_expected_horizontal.print();
        wind_horizontal(0) = _current_wind(0);
        wind_horizontal(1) = _current_wind(1);
        identity_1.setIdentity();
        //z_expected_horizontal.print();
        //wind_horizontal.print();
        //(_K_horizontal*(wind_horizontal - z_expected_horizontal)).print();
        _X_posterior_horizontal = _X_prior_horizontal + _K_horizontal*(wind_horizontal - z_expected_horizontal);
        _P_posterior_horizontal = (identity_1 - _K_horizontal*_H_horizontal)*_P_prior_horizontal;

        // perform vertical update
        Vector<float, 1> z_expected_vertical = (Vector<float, 1>) (_H_vertical*_X_prior_vertical);
        Vector<float, 1> wind_vertical;
        Matrix<float, _dim_vertical, _dim_vertical> identity_2;
        wind_vertical(0) = wind(2);
        identity_2.setIdentity();
        _X_posterior_vertical = _X_prior_vertical + _K_vertical*(wind_vertical - z_expected_vertical);
        _P_posterior_vertical = (identity_2 - _K_vertical*_H_vertical)*_P_prior_vertical;
    }

    // find the correct sign of params (parametrization is not unique)
    /*
    if (_X_posterior_horizontal(5)<0.f) {
        _X_posterior_horizontal(0) *= -1.f;
        _X_posterior_horizontal(1) *= -1.f;
        _X_posterior_horizontal(5) *= -1.f;
    }
    */

}

void
FixedwingShearEstimator::Run()
{
    if (should_exit()) {
		_soaring_controller_wind_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

    perf_begin(_loop_perf);

    // only run controller if wind info changed
    soaring_controller_wind_s soaring_controller_wind;
	if (_soaring_controller_wind_sub.update(&soaring_controller_wind))
    {   
        // only update parameters if they changed
		bool params_updated = _parameter_update_sub.updated();

		// check for parameter updates
		if (params_updated) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();
			parameters_update();
		}
        // get current measurement
        _current_wind = Vector3f(soaring_controller_wind.wind_estimate_filtered)/_unit_v;
        _current_height = Vector3f(soaring_controller_wind.position)(2)/_unit_h;
        //_current_wind(0) = 0.f;
        //_current_wind(1) = -(10.f/_unit_v)/(1+expf(-1.f/_unit_a*(_current_height- 110.f/_unit_h)));
        //_current_wind(2) = 0.f;

        if (true) {
            // prior update
            perform_prior_update();

            // posterior update
            perform_posterior_update(_current_height, _current_wind);


            
            // check if filter diverges 
            // maybe reset filters...
            if (sqrtf(_P_posterior_horizontal(4,4))>=10.f) {
                PX4_WARN("large height uncertainty, resetting filter");
                reset_filter();
            }
        }

        // publish shear params
        // ========================================
        // publish controller position in ENU frame
        // ========================================
        _soaring_estimator_shear.timestamp = hrt_absolute_time();
        _soaring_estimator_shear.vx = _X_posterior_horizontal(0)*_unit_v;
        _soaring_estimator_shear.vy = _X_posterior_horizontal(1)*_unit_v;
        _soaring_estimator_shear.bx = _X_posterior_horizontal(2)*_unit_v;
        _soaring_estimator_shear.by = _X_posterior_horizontal(3)*_unit_v;
        _soaring_estimator_shear.h = _X_posterior_horizontal(4)*_unit_h;
        _soaring_estimator_shear.a = _X_posterior_horizontal(5)*_unit_a;
        _soaring_estimator_shear.sigma_vx = sqrtf(_P_posterior_horizontal(0,0))*_unit_v;
        _soaring_estimator_shear.sigma_vy = sqrtf(_P_posterior_horizontal(1,1))*_unit_v;
        _soaring_estimator_shear.sigma_bx = sqrtf(_P_posterior_horizontal(2,2))*_unit_v;
        _soaring_estimator_shear.sigma_by = sqrtf(_P_posterior_horizontal(3,3))*_unit_v;
        _soaring_estimator_shear.sigma_h = sqrtf(_P_posterior_horizontal(4,4))*_unit_h;
        _soaring_estimator_shear.sigma_a = sqrtf(_P_posterior_horizontal(5,5))*_unit_a;
        _soaring_estimator_shear.reset_counter = _reset_counter;
        _soaring_estimator_shear_pub.publish(_soaring_estimator_shear);
    }


}


int FixedwingShearEstimator::task_spawn(int argc, char *argv[])
{
	FixedwingShearEstimator *instance = new FixedwingShearEstimator();

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


int FixedwingShearEstimator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FixedwingShearEstimator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_dyn_soar_estimator is the fixed wing shear estimator for dynamic soaring.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_dyn_soar_estimator", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_dyn_soar_estimator_main(int argc, char *argv[])
{
	return FixedwingShearEstimator::main(argc, argv);
}



