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
 * @file estimator_interface.h
 * Definition of base class for attitude estimators
 *
 * @author Roman Bast <bapstroman@gmail.com>
 *
 */

#include <stdint.h>
#include <matrix/matrix/math.hpp>
#include "RingBuffer.h"
#include "geo.h"
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

	// gets the innovation of airspeed measurement
 	virtual void get_airspeed_innov(float *airspeed_innov) = 0;

	// gets the innovations of the heading measurement
	virtual void get_heading_innov(float *heading_innov) = 0;

	// gets the innovation variances of velocity and position measurements
	// 0-2 vel, 3-5 pos
	virtual void get_vel_pos_innov_var(float vel_pos_innov_var[6]) = 0;

	// gets the innovation variances of the earth magnetic field measurements
	virtual void get_mag_innov_var(float mag_innov_var[3]) = 0;

	// gets the innovation variance of the airspeed measurement
 	virtual void get_airspeed_innov_var(float *get_airspeed_innov_var) = 0;

	// gets the innovation variance of the heading measurement
	virtual void get_heading_innov_var(float *heading_innov_var) = 0;

	virtual void get_state_delayed(float *state) = 0;

	virtual void get_covariances(float *covariances) = 0;

	// get the ekf WGS-84 origin position and height and the system time it was last set
	virtual void get_vel_var(Vector3f &vel_var) = 0;
	virtual void get_pos_var(Vector3f &pos_var) = 0;

	// gets the innovation variance of the flow measurement
	virtual void get_flow_innov_var(float flow_innov_var[2]) = 0;

	// gets the innovation of the flow measurement
	virtual void get_flow_innov(float flow_innov[2]) = 0;

	// gets the innovation variance of the HAGL measurement
	virtual void get_hagl_innov_var(float *flow_innov_var) = 0;

	// gets the innovation of the HAGL measurement
	virtual void get_hagl_innov(float *flow_innov_var) = 0;

	// get the ekf WGS-84 origin positoin and height and the system time it was last set
	virtual void get_ekf_origin(uint64_t *origin_time, map_projection_reference_s *origin_pos, float *origin_alt) = 0;

	// get the 1-sigma horizontal and vertical position uncertainty of the ekf WGS-84 position
	virtual void get_ekf_accuracy(float *ekf_eph, float *ekf_epv, bool *dead_reckoning) = 0;

	// ask estimator for sensor data collection decision and do any preprocessing if required, returns true if not defined
	virtual bool collect_gps(uint64_t time_usec, struct gps_message *gps) { return true; }

	virtual bool collect_imu(imuSample &imu) { return true; }

	virtual bool collect_mag(uint64_t time_usec, float *data) { return true; }

	virtual bool collect_baro(uint64_t time_usec, float *data) { return true; }

	virtual bool collect_airspeed(uint64_t time_usec, float *data) { return true; }

	virtual bool collect_range(uint64_t time_usec, float *data) { return true; }

	virtual bool collect_opticalflow(uint64_t time_usec, flow_message *flow) { return true; }

	// set delta angle imu data
	void setIMUData(uint64_t time_usec, uint64_t delta_ang_dt, uint64_t delta_vel_dt, float *delta_ang, float *delta_vel);

	// set magnetometer data
	void setMagData(uint64_t time_usec, float *data);
	//void setMagData(uint64_t time_usec, struct magSample *mag);

	// set gps data
	void setGpsData(uint64_t time_usec, struct gps_message *gps);

	// set baro data
	void setBaroData(uint64_t time_usec, float *data);

	// set airspeed data
	void setAirspeedData(uint64_t time_usec, float *data);

	// set range data
	void setRangeData(uint64_t time_usec, float *data);

	// set optical flow data
	void setOpticalFlowData(uint64_t time_usec, flow_message *flow);

	// return a address to the parameters struct
	// in order to give access to the application
	parameters *getParamHandle() {return &_params;}

	// set vehicle arm status data
	void set_arm_status(bool data) { _vehicle_armed = data; }

	// set vehicle landed status data
	void set_in_air_status(bool in_air) {_in_air = in_air;}

	// return true if the global position estimate is valid
	virtual bool global_position_is_valid() = 0;

	// return true if the estimate is valid
	// return the estimated terrain vertical position relative to the NED origin
	virtual bool get_terrain_vert_pos(float *ret) = 0;

	// return true if the local position estimate is valid
	bool local_position_is_valid();


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
		*time_us = _time_last_imu;
	}

	// Copy the magnetic declination that we wish to save to the EKF2_MAG_DECL parameter for the next startup
	void copy_mag_decl_deg(float *val)
	{
		*val = _mag_declination_to_save_deg;
	}

protected:

	parameters _params;		// filter parameters

	static const uint8_t OBS_BUFFER_LENGTH = 10;	// defines how many measurement samples we can buffer
	static const uint8_t IMU_BUFFER_LENGTH = 30;	// defines how many imu samples we can buffer
	static const unsigned FILTER_UPDATE_PERRIOD_MS = 10;	// ekf prediction period in milliseconds

	float _dt_imu_avg;	// average imu update period in s

	imuSample _imu_sample_delayed;	// captures the imu sample on the delayed time horizon

	// measurement samples capturing measurements on the delayed time horizon
	magSample _mag_sample_delayed;
	baroSample _baro_sample_delayed;
	gpsSample _gps_sample_delayed;
	rangeSample _range_sample_delayed;
	airspeedSample _airspeed_sample_delayed;
	flowSample _flow_sample_delayed;

	outputSample _output_sample_delayed;	// filter output on the delayed time horizon
	outputSample _output_new;	// filter output on the non-delayed time horizon
	imuSample _imu_sample_new;	// imu sample capturing the newest imu data

	uint64_t _imu_ticks;	// counter for imu updates

	bool _imu_updated;      // true if the ekf should update (completed downsampling process)
	bool _initialised;      // true if the ekf interface instance (data buffering) is initialized
	bool _vehicle_armed;    // vehicle arm status used to turn off functionality used on the ground
	bool _in_air;           // we assume vehicle is in the air, set by the given landing detector

	bool _NED_origin_initialised = false;
	bool _gps_speed_valid = false;
	float _gps_origin_eph = 0.0f; // horizontal position uncertainty of the GPS origin
	float _gps_origin_epv = 0.0f; // vertical position uncertainty of the GPS origin
	struct map_projection_reference_s _pos_ref = {};    // Contains WGS-84 position latitude and longitude (radians)

	bool _mag_healthy;              // computed by mag innovation test
	float _yaw_test_ratio;          // yaw innovation consistency check ratio
	float _mag_test_ratio[3];       // magnetometer XYZ innovation consistency check ratios
	float _vel_pos_test_ratio[6];   // velocity and position innovation consistency check ratios
	float _tas_test_ratio;			// tas innovation consistency check ratio

	// data buffer instances
	RingBuffer<imuSample> _imu_buffer;
	RingBuffer<gpsSample> _gps_buffer;
	RingBuffer<magSample> _mag_buffer;
	RingBuffer<baroSample> _baro_buffer;
	RingBuffer<rangeSample> _range_buffer;
	RingBuffer<airspeedSample> _airspeed_buffer;
	RingBuffer<flowSample> 	_flow_buffer;
	RingBuffer<outputSample> _output_buffer;

	uint64_t _time_last_imu;	// timestamp of last imu sample in microseconds
	uint64_t _time_last_gps;	// timestamp of last gps measurement in microseconds
	uint64_t _time_last_mag;	// timestamp of last magnetometer measurement in microseconds
	uint64_t _time_last_baro;	// timestamp of last barometer measurement in microseconds
	uint64_t _time_last_range;	// timestamp of last range measurement in microseconds
	uint64_t _time_last_airspeed;	// timestamp of last airspeed measurement in microseconds
	uint64_t _time_last_optflow;

	fault_status_t _fault_status;

	// allocate data buffers and intialise interface variables
	bool initialise_interface(uint64_t timestamp);

	// free buffer memory
	void unallocate_buffers();

	float _mag_declination_gps;         // magnetic declination returned by the geo library using the last valid GPS position (rad)
	float _mag_declination_to_save_deg; // magnetic declination to save to EKF2_MAG_DECL (deg)

	// this is the current status of the filter control modes
	filter_control_status_u _control_status;

	// this is the previous status of the filter control modes - used to detect mode transitions
	filter_control_status_u _control_status_prev;

};
