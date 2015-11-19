/****************************************************************************
 *
 *   Copyright (c) 2013 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file estimator_base.h
 * Definition of base class for attitude estimators
 *
 * @author Roman Bast <bastroman@gmail.com>
 *
 */

#include <stdint.h>
#include <matrix/matrix/math.hpp>
#include <lib/geo/geo.h>
#include "RingBuffer.h"

struct gps_message {
	uint64_t time_usec;
	int32_t lat;			// Latitude in 1E-7 degrees
	int32_t lon;			// Longitude in 1E-7 degrees
	int32_t alt;			// Altitude in 1E-3 meters (millimeters) above MSL
	uint8_t fix_type;		// 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time
	float eph;			// GPS HDOP horizontal dilution of position in m
	float epv;			// GPS VDOP horizontal dilution of position in m
	uint64_t time_usec_vel;	// Timestamp for velocity informations
	float vel_m_s;			// GPS ground speed (m/s)
	float vel_ned[3];		// GPS ground speed NED
	bool vel_ned_valid;		// GPS ground speed is valid
};

class EstimatorBase
{
public:
	EstimatorBase();
	~EstimatorBase();

	// set delta angle imu data
	void setIMUData(uint64_t time_usec, uint64_t delta_ang_dt, uint64_t delta_vel_dt, float *delta_ang, float *delta_vel);

	// set magnetometer data
	void setMagData(uint64_t time_usec, float *data);

	// set gps data
	void setGpsData(uint64_t time_usec, struct gps_message *gps);

	// set baro data
	void setBaroData(uint64_t time_usec, float *data);

	// set airspeed data
	void setAirspeedData(uint64_t time_usec, float *data);

	// set range data
	void setRangeData(uint64_t time_usec, float *data);

	// set optical flow data
	void setOpticalFlowData(uint64_t time_usec, float *data);

protected:

	typedef matrix::Vector<float, 2> Vector2f;
	typedef matrix::Vector<float, 3> Vector3f;
	typedef matrix::Quaternion<float> Quaternion;
	typedef matrix::Matrix<float, 3, 3> Matrix3f;

	struct stateSample {
		Vector3f    ang_error;
		Vector3f    vel;
		Vector3f    pos;
		Vector3f    gyro_bias;
		Vector3f    gyro_scale;
		float       accel_z_bias;
		Vector3f    mag_I;
		Vector3f    mag_B;
		Vector2f    wind_vel;
		Quaternion  quat_nominal;
	} _state;

	struct outputSample {
		Quaternion  quat_nominal;
		Vector3f    vel;
		Vector3f    pos;
	};

	struct imuSample {
		Vector3f    delta_ang;
		Vector3f    delta_vel;
		float       delta_ang_dt;
		float       delta_vel_dt;
		uint32_t    time_us;
	};

	struct gpsSample {
		Vector2f    pos;
		float       hgt;
		Vector3f    vel;
		uint32_t    time_us;
	};

	struct magSample {
		Vector3f    mag;
		uint32_t    time_us;
	};

	struct baroSample {
		float       hgt;
		uint32_t    time_us;
	};

	struct rangeSample {
		float       rng;
		uint32_t    time_us;
	};

	struct airspeedSample {
		float       airspeed;
		uint32_t    time_us;
	};

	struct flowSample {
		Vector2f    flowRadXY;
		Vector2f    flowRadXYcomp;
		uint32_t    time_us;
	};

	struct {
		uint32_t mag_delay_ms;
		uint32_t baro_delay_ms;
		uint32_t gps_delay_ms;
		uint32_t airspeed_delay_ms;
		float 	requiredEph;
		float 	requiredEpv;
	} _params;


	static const uint8_t OBS_BUFFER_LENGTH = 5;
	static const uint8_t IMU_BUFFER_LENGTH = 25;
	static const unsigned FILTER_UPDATE_PERRIOD_MS = 10;

	float _dt_imu_avg;
	uint32_t _imu_time_last;

	imuSample _imu_sample_delayed;
	imuSample _imu_down_sampled;
	Quaternion
	_q_down_sampled;

	magSample _mag_sample_delayed;
	baroSample _baro_sample_delayed;
	gpsSample _gps_sample_delayed;
	rangeSample _range_sample_delayed;
	airspeedSample _airspeed_sample_delayed;
	flowSample _flow_sample_delayed;

	struct map_projection_reference_s _posRef;
	float _gps_alt_ref;


	uint32_t _imu_ticks;

	bool _imu_updated;
	bool _start_predict_enabled;
	bool _initialised;
	bool _gps_initialised;
	bool _gps_speed_valid;

	RingBuffer<imuSample> _imu_buffer;
	RingBuffer<gpsSample> _gps_buffer;
	RingBuffer<magSample> _mag_buffer;
	RingBuffer<baroSample> _baro_buffer;
	RingBuffer<rangeSample> _range_buffer;
	RingBuffer<airspeedSample> _airspeed_buffer;
	RingBuffer<flowSample> 	_flow_buffer;
	RingBuffer<outputSample> _output_buffer;

	uint64_t _time_last_imu;
	uint64_t _time_last_gps;
	uint64_t _time_last_mag;
	uint64_t _time_last_baro;
	uint64_t _time_last_range;
	uint64_t _time_last_airspeed;


	void initialiseVariables(uint64_t timestamp);

	void initialiseGPS(struct gps_message *gps);

	bool gps_is_good(struct gps_message *gps);

public:
	void printIMU(struct imuSample *data);
	void printStoredIMU();
	void printQuaternion(Quaternion &q);
	void print_imu_avg_time();
	void printMag(struct magSample *data);
	void printStoredMag();
	void printBaro(struct baroSample *data);
	void printStoredBaro();
	void printGps(struct gpsSample *data);
	void printStoredGps();
};

