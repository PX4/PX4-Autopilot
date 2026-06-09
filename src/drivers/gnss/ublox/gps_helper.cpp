/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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

#include "gps_helper.h"
#include <math.h>

#ifndef M_PI
#define M_PI		3.141592653589793238462643383280
#endif


/**
 * @file gps_helper.cpp
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <julian@oes.ch>
 */

GPSHelper::GPSHelper(GPSCallbackPtr callback, void *callback_user) :
	_callback(callback),
	_callback_user(callback_user)
{
}

void
GPSHelper::resetUpdateRates()
{
	_rate_count_vel = 0;
	_rate_count_lat_lon = 0;
	_interval_rate_start = gps_absolute_time();
}

void
GPSHelper::storeUpdateRates()
{
	_rate_vel = _rate_count_vel / (((float)(gps_absolute_time() - _interval_rate_start)) / 1000000.0f);
	_rate_lat_lon = _rate_count_lat_lon / (((float)(gps_absolute_time() - _interval_rate_start)) / 1000000.0f);
}

void GPSHelper::ECEF2lla(double ecef_x, double ecef_y, double ecef_z, double &latitude, double &longitude,
			 float &altitude)
{
	// WGS84 ellipsoid constants
	constexpr double a = 6378137.; // radius
	constexpr double e = 8.1819190842622e-2;  // eccentricity

	constexpr double asq = a * a;
	constexpr double esq = e * e;

	double x = ecef_x;
	double y = ecef_y;
	double z = ecef_z;

	double b = sqrt(asq * (1. - esq));
	double bsq = b * b;
	double ep = sqrt((asq - bsq) / bsq);
	double p = sqrt(x * x + y * y);
	double th = atan2(a * z, b * p);

	longitude = atan2(y, x);
	double sin_th = sin(th);
	double cos_th = cos(th);
	latitude = atan2(z + ep * ep * b * sin_th * sin_th * sin_th, p - esq * a * cos_th * cos_th * cos_th);
	double sin_lat = sin(latitude);
	double N = a / sqrt(1. - esq * sin_lat * sin_lat);
	altitude = (float)(p / cos(latitude) - N);

	// rad to deg
	longitude *= 180. / M_PI;
	latitude *= 180. / M_PI;

	// correction for altitude near poles left out.
}
