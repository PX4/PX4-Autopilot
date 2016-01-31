/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * @author Roman Bast <bapstroman@gmail.com>
 *
 */

#include <stdint.h>
#include <matrix/matrix/math.hpp>
#include <lib/geo/geo.h>
#include "RingBuffer.h"

#include "common.h"

using namespace estimator;
class EstimatorInterface
{

public:
	EstimatorInterface();
	~EstimatorInterface();

	virtual bool init(uint64_t timestamp) = 0;
	virtual bool update() = 0;

	// gets the innovations of velocity and position measurements
	// 0-2 vel, 3-5 pos
	virtual void get_vel_pos_innov(float vel_pos_innov[6]) = 0;

	// gets the innovations of the earth magnetic field measurements
	virtual void get_mag_innov(float mag_innov[3]) = 0;

	// gets the innovations of the heading measurement
	virtual void get_heading_innov(float *heading_innov) = 0;

	// gets the innovation variances of velocity and position measurements
	// 0-2 vel, 3-5 pos
	virtual void get_vel_pos_innov_var(float vel_pos_innov_var[6]) = 0;

	// gets the innovation variances of the earth magnetic field measurements
	virtual void get_mag_innov_var(float mag_innov_var[3]) = 0;

	// gets the innovation variance of the heading measurement
	virtual void get_heading_innov_var(float *heading_innov_var) = 0;

	virtual void get_state_delayed(float *state) = 0;

	virtual void get_covariances(float *covariances) = 0;

    // get the ekf WGS-84 origin positoin and height and the system time it was last set
    virtual void get_ekf_origin(uint64_t *origin_time, map_projection_reference_s *origin_pos, float *origin_alt) = 0;


    // ask estimator for sensor data collection decision and do any preprocessing if required, returns true if not defined
    virtual bool collect_gps(uint64_t time_usec, struct gps_message *gps) { return true; }

    virtual bool collect_imu(imuSample &imu) { return true; }

    virtual bool collect_mag(uint64_t time_usec, float *data) { return true; }

    virtual bool collect_baro(uint64_t time_usec, float *data) { return true; }

    virtual bool collect_airspeed(uint64_t time_usec, float *data) { return true; }

    virtual bool collect_range(uint64_t time_usec, float *data) { return true; }

    virtual bool collect_opticalflow(uint64_t time_usec, float *data) { return true; }

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

	// return a address to the parameters struct
	// in order to give access to the application
	parameters *getParamHandle() {return &_params;}

    // set vehicle arm status data
    void set_arm_status(bool data){ _vehicle_armed = data; }

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

	bool position_is_valid();


	void copy_quaternion(float *quat)
	{
		for (unsigned i = 0; i < 4; i++) {
			quat[i] = _output_new.quat_nominal(i);
		}
	}
	void copy_velocity(float *vel)
	{
		for (unsigned i = 0; i < 3; i++) {
			vel[i] = _output_new.vel(i);
		}
	}
	void copy_position(float *pos)
	{
		for (unsigned i = 0; i < 3; i++) {
			pos[i] = _output_new.pos(i);
		}
	}
	void copy_timestamp(uint64_t *time_us)
	{
		*time_us = _imu_time_last;
	}

protected:

	parameters _params;		// filter parameters

	static const uint8_t OBS_BUFFER_LENGTH = 10;
	static const uint8_t IMU_BUFFER_LENGTH = 30;
	static const unsigned FILTER_UPDATE_PERRIOD_MS = 10;

	float _dt_imu_avg;
	uint64_t _imu_time_last;

	imuSample _imu_sample_delayed;

	magSample _mag_sample_delayed;
	baroSample _baro_sample_delayed;
	gpsSample _gps_sample_delayed;
	rangeSample _range_sample_delayed;
	airspeedSample _airspeed_sample_delayed;
	flowSample _flow_sample_delayed;

	outputSample _output_sample_delayed;
	outputSample _output_new;
	imuSample _imu_sample_new;

    uint64_t _imu_ticks;

	bool _imu_updated = false;
	bool _initialised = false;
    bool _vehicle_armed = false;     // vehicle arm status used to turn off functionality used on the ground

    bool _NED_origin_initialised = false;
    bool _gps_speed_valid = false;
    struct map_projection_reference_s _pos_ref = {};    // Contains WGS-84 position latitude and longitude (radians)

	bool _mag_healthy = false;		// computed by mag innovation test
    float _yaw_test_ratio;          // yaw innovation consistency check ratio
    float _mag_test_ratio[3];       // magnetometer XYZ innovation consistency check ratios

    float _vel_pos_test_ratio[6];   // velocity and position innovation consistency check ratios

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


	fault_status_t _fault_status;
	bool initialise_interface(uint64_t timestamp);
	void unallocate_buffers();
};
