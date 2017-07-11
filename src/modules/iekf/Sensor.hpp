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

#pragma once

#include <matrix/math.hpp>

using namespace matrix;

const float betaWarn = 1.0f;

/**
 * Class to hold data for sensors and manage
 * timestamps etc. There should be one for each
 * uniquely sent and timestamped message.
 */
class Sensor
{
public:
	Sensor(const char *name, float betaMax, float condMax, float rateMax) :
		_timestamp(0),
		_beta(0),
		_cond(1),
		_name(name),
		_betaMax(betaMax),
		_condMax(condMax),
		_rateMax(rateMax)
	{
	}

	/**
	 * Check if ready and return dt
	 */
	bool ready(uint64_t timestampNew, float &dt)
	{
		// return if no new data
		if (timestampNew == _timestamp) {
			return false;
		}

		dt = (timestampNew - _timestamp) / 1.0e6f;
		_timestamp = timestampNew;

		if (dt <= 0 || ((1.0f / dt) > _rateMax)) {
			return false;
		}

		return true;
	}
	inline void setBeta(float beta)
	{
		_beta = beta;
	}
	inline void setRateMax(float rateMax)
	{
		_rateMax = rateMax;
	}
	inline float getBeta()
	{
		return _beta;
	}
	inline bool faulted()
	{
		return _beta > _betaMax;
	}
	inline bool poorlyConditioned()
	{
		return _cond > _condMax;
	}
	inline bool shouldCorrect()
	{
		return !faulted() && !poorlyConditioned();
	}
	void setCorrectionInfo(float beta, float cond)
	{
		_beta = beta;
		_cond = cond;

		if (beta > betaWarn) {
			PX4_WARN("%s fault: beta %10.4f", _name, double(beta));
		}

		if (beta > _betaMax) {
			PX4_INFO("beyond beta max, not correcting");
		};

		if (cond > _condMax) {
			PX4_WARN("%s poorly conditioned %10.4f", _name, double(cond));
		}
	}

	template<class Type, size_t n_x, size_t n_y>
	void kalmanCorrectCond(
		const SquareMatrix<Type, n_x> &P,
		const Matrix<Type, n_y, n_x> &H,
		const Matrix<Type, n_y, n_y> &R,
		const Matrix<Type, n_y, 1> &r,
		Vector<Type, n_x> &dx,
		SquareMatrix<Type, n_x> &dP,
		SquareMatrix<Type, n_y> &S
	)
	{
		//PX4_INFO("R");
		//R.print();
		dx.setZero();
		dP.setZero();
		// tmp = S  = H * P * H^T + R
		S = H * P * H.T() + R;
		//PX4_INFO("S");
		//tmp.print();
		// tmp = L = cholesky(S)
		SquareMatrix<Type, n_y> tmp = cholesky(S);
		//PX4_INFO("L");
		//tmp.print();
		Vector<Type, n_y> d = tmp.diag();
		//PX4_INFO("diag");
		//d.print();
		// tmp = L_I = inv(L)
		tmp = inv(tmp);
		// tmp = inv(S) = L_I^T*L_I
		tmp = tmp.T() * tmp;
		float cond = d.max() / d.min();
		float beta = (r.T() * (tmp * r))(0, 0) / BETA_TABLE[n_y];
		Matrix<Type, n_x, n_y> K = P * H.T() * tmp;
		setCorrectionInfo(beta, cond);

		dP = -K * H * P;

		// use jospeh form to make sure it stays positive definite
		//SquareMatrix<Type, n_x> tmp2 = eye<float, Xe::n>() - K * H;
		//dP = tmp2 * P * tmp2.T();
		//dP += K * R * K.T();
		//dP -= P;
		dx = K * r;
	}
	float &getRateMax()
	{
		return _rateMax;
	}

private:
	uint64_t _timestamp; // time of last read
	float _beta; // if > 1, indicates fault
	float _cond; // condition number, ratio of
	// eigen values of innovation matrix
	// S = H*P*H^T + R
	const char *_name;
	float _betaMax;
	float _condMax;
	float _rateMax;
};

