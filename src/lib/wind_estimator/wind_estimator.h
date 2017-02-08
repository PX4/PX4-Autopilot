/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file wind_estimator.h
 * A wind and airspeed scale estimator.
 */

#include <matrix/matrix/math.hpp>

class WindEstimator
{
public:
	WindEstimator();

	~WindEstimator();

	void update(float dt);

	void fuse_airspeed(float true_airspeed, float velI[3], float velIvar[2]);

	void fuse_beta(float velI[3], float q_att[4]);

	void get_wind(float wind[2])
	{
		wind[0] = _state(w_n);
		wind[1] = _state(w_e);
	}

	bool is_estimate_valid() {return _initialised;}

	float get_tas_scale() {return _state(tas);}

	float get_tas_innov() {return _tas_innov;}

	float get_tas_innov_var() {return _tas_innov_var;}

	float get_beta_innov() {return _beta_innov;}

	float get_beta_innov_var() {return _beta_innov_var;}

	void get_wind_var(float wind_var[2])
	{
		wind_var[0] = _P(0, 0);
		wind_var[1] = _P(1, 1);
	}

	void set_wind_p_noise(float wind_sigma) {_wind_p_var = wind_sigma * wind_sigma;}

	void set_tas_scale_p_noise(float tas_scale_sigma) {_tas_scale_p_var = tas_scale_sigma * tas_scale_sigma;}

	void set_tas_noise(float tas_sigma) {_tas_var = tas_sigma * tas_sigma;}

	void set_beta_noise(float beta_var) {_beta_var = beta_var * beta_var;}


private:
	enum {w_n = 0, w_e, tas};	// enum which can be used to access state.

	matrix::Vector3f _state;		// state vector
	matrix::Matrix<float, 3, 3> _P;		// state covariance matrix

	float _tas_innov;	// true airspeed innovation
	float _tas_innov_var;	// true airspeed innovation variance

	float _beta_innov;	// sideslip innovation
	float _beta_innov_var;	// sideslip innovation variance

	bool _initialised = false;	// True: filter has been initialised

	float _wind_p_var;	// wind process noise variance
	float _tas_scale_p_var;	// true airspeed scale process noise variance
	float _tas_var;		// true airspeed measurement noise variance
	float _beta_var;	// sideslip measurement noise variance

	// initialise state and state covariance matrix
	bool initialise(float velI[3], float velIvar[2], float tas_meas);

	void run_sanity_checks();
};