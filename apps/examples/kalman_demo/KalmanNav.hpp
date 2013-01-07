/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file KalmanNav.hpp
 *
 * kalman filter navigation code
 */

#pragma once

//#define MATRIX_ASSERT
//#define VECTOR_ASSERT

#include <nuttx/config.h>

#include <mathlib/mathlib.h>
#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>
#include <controllib/block/UOrbSubscription.hpp>
#include <controllib/block/UOrbPublication.hpp>

#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/parameter_update.h>

#include <drivers/drv_hrt.h>
#include <poll.h>
#include <unistd.h>

class KalmanNav : public control::SuperBlock
{
public:
	KalmanNav(SuperBlock *parent, const char *name);
	virtual ~KalmanNav() {};
	void update();
	virtual void updatePublications();
	void predictFast(float dt);
	void predictSlow(float dt);
	void correctAtt();
	void correctGps();
	virtual void updateParams();
protected:
	math::Matrix F;
	math::Matrix G;
	math::Matrix P;
	math::Matrix V;
	math::Matrix HAtt;
	math::Matrix RAtt;
	math::Matrix HGps;
	math::Matrix RGps;
	math::Dcm C_nb;
	math::Quaternion q;
	control::UOrbSubscription<sensor_combined_s> _sensors;
	control::UOrbSubscription<vehicle_gps_position_s> _gps;
	control::UOrbSubscription<parameter_update_s> _param_update;
	control::UOrbPublication<vehicle_global_position_s> _pos;
	control::UOrbPublication<vehicle_attitude_s> _att;
	uint64_t _pubTimeStamp;
	uint64_t _fastTimeStamp;
	uint64_t _slowTimeStamp;
	uint64_t _attTimeStamp;
	uint64_t _outTimeStamp;
	uint16_t _navFrames;
	float fN, fE, fD;
	// states
	enum {PHI = 0, THETA, PSI, VN, VE, VD, LAT, LON, ALT};
	float phi, theta, psi;
	float vN, vE, vD;
	double lat, lon, alt;
	control::BlockParam<float> _vGyro;
	control::BlockParam<float> _vAccel;
	control::BlockParam<float> _rMag;
	control::BlockParam<float> _rGpsV;
	control::BlockParam<float> _rGpsGeo;
	control::BlockParam<float> _rGpsAlt;
	control::BlockParam<float> _rAccel;
	int32_t getLatDegE7() { return int32_t(lat * 1.0e7 * M_RAD_TO_DEG); }
	void setLatDegE7(int32_t val) { lat = val / 1.0e7 / M_RAD_TO_DEG; }
	int32_t getLonDegE7() { return int32_t(lon * 1.0e7 * M_RAD_TO_DEG); }
	void setLonDegE7(int32_t val) { lon = val / 1.0e7 / M_RAD_TO_DEG; }
	int32_t getAltE3() { return int32_t(alt * 1.0e3); }
	void setAltE3(int32_t val) { alt = double(val) / 1.0e3; }
};
