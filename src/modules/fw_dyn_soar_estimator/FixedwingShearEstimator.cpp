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



FixedwingShearEstimator::FixedwingShearEstimator() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::test1),
    _soaring_estimator_shear_pub(ORB_ID(soaring_estimator_shear))
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
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("vehicle position callback registration failed!");
		return false;
	}
    PX4_INFO("Starting FW_DYN_SOAR_ESTIMATOR");
	return true;
}

int
FixedwingShearEstimator::parameters_update()
{
	updateParams();

	// update params...
    _Q_horizontal(0,0) = powf(_param_sigma_q_vel.get(),2);
    _Q_horizontal(1,1) = powf(_param_sigma_q_vel.get(),2);
    _Q_horizontal(2,2) = powf(_param_sigma_q_vel.get(),2);
    _Q_horizontal(3,3) = powf(_param_sigma_q_vel.get(),2);
    _Q_horizontal(4,4) = powf(_param_sigma_q_h.get(),2);
    _Q_horizontal(5,5) = powf(_param_sigma_q_a.get(),2);

    _R_horizontal(0,0) = powf(_param_sigma_r_vel.get(),2);
    _R_horizontal(1,1) = powf(_param_sigma_r_vel.get(),2);

    for (int i=0;i<_dim_vertical;i++){
        _Q_vertical(i,i) = powf(_param_sigma_q_vel.get(),2);
    }

    _R_vertical = powf(_param_sigma_r_vel.get(),2);

	return PX4_OK;
}

void
FixedwingShearEstimator::reset_filter()
{
    // reset all states of the filter to some initial guess.

    // reset horizontal wind state
    for (int i=0;i<6;i++){
        _X_prior_horizontal(i,i) = 0.0f;
        _X_posterior_horizontal(i,i) = 0.0f;
        _P_prior_horizontal(i,i) = 1.0f;
        _P_posterior_horizontal(i,i) = 1.0f;
    }

    // reset vertical wind state
    for (int i=0;i<_dim_vertical;i++){
        _X_prior_vertical(i,i) = 0.0f;
        _X_posterior_vertical(i,i) = 0.0f;
        _P_prior_vertical(i,i) = 1.0f;
        _P_posterior_vertical(i,i) = 1.0f;
    }
}

void
FixedwingShearEstimator::perform_prior_update()
{

}

void
FixedwingShearEstimator::perform_posterior_update()
{
    
}

void
FixedwingPositionINDIControl::Run()
{
    if (should_exit()) {
		_soaring_controller_wind_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

    perf_begin(_loop_perf);

    // only run controller if wind info changed
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
        _current_wind = Vector3f(soaring_controller_wind.wind_estimate_filtered);
        _current_height = Vector3f(soaring_controller_wind.position)(2);

        // prior update
        perform_prior_update();

        // posterior update
        perform_posterior_update(_current_height, _current_wind);

        // check if filter diverges 
        // maybe reset filters...

        // publish shear params
    }


}


