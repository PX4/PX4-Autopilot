/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file polycomp.h
 *
 * A polynomial compensation class
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#pragma once

#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>

class PolyComp
{
public:
	PolyComp();
	virtual ~PolyComp();

	/**
	 * polynomial temperature correction on a given axis
	 *
	 * @param axis  axis (0/1/2) which needs to be compensated
	 * @param input raw data of axis
	 * @param temp	temperature at which the measurement is taken
	 * @return		scaled and temperature compensated data
	 */
	float			get(unsigned axis, float input, float temp);

	void			set_coeffs(struct accel_scale &_scale);
	void			set_coeffs(struct gyro_scale &_scale);
	void			set_coeffs(struct mag_scale &_scale);

private:
	static const unsigned n_axes = 3;
	float _x3[n_axes];		/**< x^3 term of polynomial */
	float _x2[n_axes];		/**< x^2 term of polynomial */
	float _x1[n_axes];		/**< x^1 term of polynomial */
	float _offsets[n_axes];		/**< x^0 / offset term of polynomial */
	float _scales[n_axes];		/**< linear scale error */
	float _cal_temp;		/**< temperature at which no compensation is needed */
	float _min_temp;		/**< minimum temperature with valid compensation data */
	float _max_temp;		/**< maximum temperature with valid compensation data */

	/* we don't want this class to be copied */
	PolyComp(const PolyComp &);
	PolyComp operator=(const PolyComp &);
};

PolyComp::PolyComp() :
	_x3{0},
	_x2{0},
	_x1{0},
	_offsets{0},
	_cal_temp(0),
	_min_temp(0),
	_max_temp(0)
{
}

PolyComp::~PolyComp()
{
}

void
PolyComp::set_coeffs(struct accel_scale &_scale)
{
	for (unsigned i = 0; i < n_axes; i++) {
		_x3[i] = _scale.x3_temp[i];
		_x2[i] = _scale.x2_temp[i];
		_x1[i] = _scale.x1_temp[i];
	}

	_offsets[0] = _scale.x_offset;
	_offsets[1] = _scale.y_offset;
	_offsets[2] = _scale.z_offset;
	_scales[0] = _scale.x_scale;
	_scales[1] = _scale.y_scale;
	_scales[2] = _scale.z_scale;
	_min_temp = _scale.min_temp;
	_max_temp = _scale.max_temp;
	_cal_temp = _scale.cal_temp;
}

void
PolyComp::set_coeffs(struct gyro_scale &_scale)
{
	for (unsigned i = 0; i < n_axes; i++) {
		_x3[i] = _scale.x3_temp[i];
		_x2[i] = _scale.x2_temp[i];
		_x1[i] = _scale.x1_temp[i];
	}

	_offsets[0] = _scale.x_offset;
	_offsets[1] = _scale.y_offset;
	_offsets[2] = _scale.z_offset;
	_scales[0] = _scale.x_scale;
	_scales[1] = _scale.y_scale;
	_scales[2] = _scale.z_scale;
	_min_temp = _scale.min_temp;
	_max_temp = _scale.max_temp;
	_cal_temp = _scale.cal_temp;
}

void
PolyComp::set_coeffs(struct mag_scale &_scale)
{
	for (unsigned i = 0; i < n_axes; i++) {
		_x3[i] = _scale.x3_temp[i];
		_x2[i] = _scale.x2_temp[i];
		_x1[i] = _scale.x1_temp[i];
	}

	_offsets[0] = _scale.x_offset;
	_offsets[1] = _scale.y_offset;
	_offsets[2] = _scale.z_offset;
	_scales[0] = _scale.x_scale;
	_scales[1] = _scale.y_scale;
	_scales[2] = _scale.z_scale;
	_min_temp = _scale.min_temp;
	_max_temp = _scale.max_temp;
	_cal_temp = _scale.cal_temp;
}

float
PolyComp::get(unsigned axis, float input, float temp)
{
	if (temp < _min_temp) {
		temp = _min_temp;
	}

	if (temp > _max_temp) {
		temp = _max_temp;
	}

	float ret = input;
	/* compensate input with temperature polynomial */
	ret -= _x3[axis] * (temp * temp * temp) + _x2[axis] * (temp * temp) + _x1[axis] * temp;
	/* adjust compensation with respect to the temperature which does not need compensation */
	/* TODO: offset is constant and can be calculated in advance in 'set_coeffs' */
	ret += _x3[axis] * (_cal_temp * _cal_temp * _cal_temp) + _x2[axis] * (_cal_temp * _cal_temp) + _x1[axis] * _cal_temp;
	/* compensate offset (constant bias) */
	//ret -= _offsets[axis];
	//ret *= _scales[axis];
	return ret;
}
