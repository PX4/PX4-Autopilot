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

/**
 * Kalman filter navigation class
 * http://en.wikipedia.org/wiki/Extended_Kalman_filter
 * Discrete-time extended Kalman filter
 */
class KalmanNav : public control::SuperBlock
{
public:
	/**
	 * Constructor
	 */
	KalmanNav(SuperBlock *parent, const char *name);

	/**
	 * Deconstuctor
	 */

	virtual ~KalmanNav() {};
	/**
	 * The main callback function for the class
	 */
	void update();


	/**
	 * Publication update
	 */
	virtual void updatePublications();

	/**
	 * State prediction
	 * Continuous, non-linear
	 */
	int predictState(float dt);

	/**
	 * State covariance prediction
	 * Continuous, linear
	 */
	int predictStateCovariance(float dt);

	/**
	 * Attitude correction
	 */
	int correctAtt();

	/**
	 * Position correction
	 */
	int correctPos();

	/**
	 * Overloaded update parameters
	 */
	virtual void updateParams();
protected:
	// kalman filter
	math::Matrix F;             /**< Jacobian(f,x), where dx/dt = f(x,u) */
	math::Matrix G;             /**< noise shaping matrix for gyro/accel */
	math::Matrix P;             /**< state covariance matrix */
	math::Matrix P0;            /**< initial state covariance matrix */
	math::Matrix V;             /**< gyro/ accel noise matrix */
	math::Matrix HAtt;          /**< attitude measurement matrix */
	math::Matrix RAtt;          /**< attitude measurement noise matrix */
	math::Matrix HPos;          /**< position measurement jacobian matrix */
	math::Matrix RPos;          /**< position measurement noise matrix */
	// attitude
	math::Dcm C_nb;             /**< direction cosine matrix from body to nav frame */
	math::Quaternion q;         /**< quaternion from body to nav frame */
	// subscriptions
	control::UOrbSubscription<sensor_combined_s> _sensors;          /**< sensors sub. */
	control::UOrbSubscription<vehicle_gps_position_s> _gps;         /**< gps sub. */
	control::UOrbSubscription<parameter_update_s> _param_update;    /**< parameter update sub. */
	// publications
	control::UOrbPublication<vehicle_global_position_s> _pos;       /**< position pub. */
	control::UOrbPublication<vehicle_attitude_s> _att;              /**< attitude pub. */
	// time stamps
	uint64_t _pubTimeStamp;     /**< output data publication time stamp */
	uint64_t _predictTimeStamp; /**< prediction time stamp */
	uint64_t _attTimeStamp;     /**< attitude correction time stamp */
	uint64_t _outTimeStamp;     /**< output time stamp */
	// frame count
	uint16_t _navFrames;        /**< navigation frames completed in output cycle */
	// miss counts
	uint16_t _miss;         	/**< number of times fast prediction loop missed */
	// accelerations
	float fN, fE, fD;           /**< navigation frame acceleration */
	// states
	enum {PHI = 0, THETA, PSI, VN, VE, VD, LAT, LON, ALT};  /**< state enumeration */
	float phi, theta, psi;                  /**< 3-2-1 euler angles */
	float vN, vE, vD;                       /**< navigation velocity, m/s */
	double lat, lon, alt;                   /**< lat, lon, alt, radians */
	// parameters
	control::BlockParam<float> _vGyro;      /**< gyro process noise */
	control::BlockParam<float> _vAccel;     /**< accelerometer process noise  */
	control::BlockParam<float> _rMag;       /**< magnetometer measurement noise  */
	control::BlockParam<float> _rGpsVel;    /**< gps velocity measurement noise */
	control::BlockParam<float> _rGpsPos;    /**< gps position measurement noise */
	control::BlockParam<float> _rGpsAlt;    /**< gps altitude measurement noise */
	control::BlockParam<float> _rPressAlt;  /**< press altitude measurement noise */
	control::BlockParam<float> _rAccel;     /**< accelerometer measurement noise */
	control::BlockParam<float> _magDip;     /**< magnetic inclination with level */
	control::BlockParam<float> _magDec;     /**< magnetic declination, clockwise rotation */
	control::BlockParam<float> _g;          /**< gravitational constant */
	control::BlockParam<float> _faultPos;   /**< fault detection threshold for position */
	control::BlockParam<float> _faultAtt;   /**< fault detection threshold for attitude */
	// status
	bool _attitudeInitialized;
	bool _positionInitialized;
	uint16_t _attitudeInitCounter;
	// accessors
	int32_t getLatDegE7() { return int32_t(lat * 1.0e7 * M_RAD_TO_DEG); }
	void setLatDegE7(int32_t val) { lat = val / 1.0e7 / M_RAD_TO_DEG; }
	int32_t getLonDegE7() { return int32_t(lon * 1.0e7 * M_RAD_TO_DEG); }
	void setLonDegE7(int32_t val) { lon = val / 1.0e7 / M_RAD_TO_DEG; }
	int32_t getAltE3() { return int32_t(alt * 1.0e3); }
	void setAltE3(int32_t val) { alt = double(val) / 1.0e3; }
};
