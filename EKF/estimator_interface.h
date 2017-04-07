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
#include "mathlib.h"

using namespace estimator;
class EstimatorInterface
{

public:
	EstimatorInterface();
	~EstimatorInterface() = default;

	virtual bool init(uint64_t timestamp) = 0;
	virtual bool update() = 0;

	// gets the innovations of velocity and position measurements
	// 0-2 vel, 3-5 pos
	virtual void get_vel_pos_innov(float vel_pos_innov[6]) = 0;

	// gets the innovations of the earth magnetic field measurements
	virtual void get_mag_innov(float mag_innov[3]) = 0;

	// gets the innovation of airspeed measurement
 	virtual void get_airspeed_innov(float *airspeed_innov) = 0;

	// gets the innovation of the synthetic sideslip measurement
	virtual void get_beta_innov(float *beta_innov) = 0;

	// gets the innovations of the heading measurement
	virtual void get_heading_innov(float *heading_innov) = 0;

	// gets the innovation variances of velocity and position measurements
	// 0-2 vel, 3-5 pos
	virtual void get_vel_pos_innov_var(float vel_pos_innov_var[6]) = 0;

	// gets the innovation variances of the earth magnetic field measurements
	virtual void get_mag_innov_var(float mag_innov_var[3]) = 0;

	// gets the innovation variance of the airspeed measurement
 	virtual void get_airspeed_innov_var(float *get_airspeed_innov_var) = 0;

	// gets the innovation variance of the synthetic sideslip measurement
	virtual void get_beta_innov_var(float *get_beta_innov_var) = 0;

	// gets the innovation variance of the heading measurement
	virtual void get_heading_innov_var(float *heading_innov_var) = 0;

	virtual void get_state_delayed(float *state) = 0;

	virtual void get_wind_velocity(float *wind) = 0;

	virtual void get_covariances(float *covariances) = 0;

	// gets the variances for the NED velocity states
	virtual void get_vel_var(Vector3f &vel_var) = 0;

	// gets the variances for the NED position states
	virtual void get_pos_var(Vector3f &pos_var) = 0;

	// gets the innovation variance of the flow measurement
	virtual void get_flow_innov_var(float flow_innov_var[2]) = 0;

	// gets the innovation of the flow measurement
	virtual void get_flow_innov(float flow_innov[2]) = 0;

	// gets the innovation variance of the HAGL measurement
	virtual void get_hagl_innov_var(float *flow_innov_var) = 0;

	// gets the innovation of the HAGL measurement
	virtual void get_hagl_innov(float *flow_innov_var) = 0;

	// return an array containing the output predictor angular, velocity and position tracking
	// error magnitudes (rad), (m/s), (m)
	virtual void get_output_tracking_error(float error[3]) = 0;

	/*
	Returns  following IMU vibration metrics in the following array locations
	0 : Gyro delta angle coning metric = filtered length of (delta_angle x prev_delta_angle)
	1 : Gyro high frequency vibe = filtered length of (delta_angle - prev_delta_angle)
	2 : Accel high frequency vibe = filtered length of (delta_velocity - prev_delta_velocity)
	*/
	virtual void get_imu_vibe_metrics(float vibe[3]) = 0;

	// get the ekf WGS-84 origin position and height and the system time it was last set
	// return true if the origin is valid
	virtual bool get_ekf_origin(uint64_t *origin_time, map_projection_reference_s *origin_pos, float *origin_alt) = 0;

	// get the 1-sigma horizontal and vertical position uncertainty of the ekf WGS-84 position
	virtual void get_ekf_gpos_accuracy(float *ekf_eph, float *ekf_epv, bool *dead_reckoning) = 0;

	// get the 1-sigma horizontal and vertical position uncertainty of the ekf local position
	virtual void get_ekf_lpos_accuracy(float *ekf_eph, float *ekf_epv, bool *dead_reckoning) = 0;

	// get the 1-sigma horizontal and vertical velocity uncertainty
	virtual void get_ekf_vel_accuracy(float *ekf_evh, float *ekf_evv, bool *dead_reckoning) = 0;

	// ask estimator for sensor data collection decision and do any preprocessing if required, returns true if not defined
	virtual bool collect_gps(uint64_t time_usec, struct gps_message *gps) { return true; }

	// accumulate and downsample IMU data to the EKF prediction rate
	virtual bool collect_imu(imuSample &imu) { return true; }

	// set delta angle imu data
	void setIMUData(uint64_t time_usec, uint64_t delta_ang_dt, uint64_t delta_vel_dt, float (&delta_ang)[3], float (&delta_vel)[3]);

	// set magnetometer data
	void setMagData(uint64_t time_usec, float (&data)[3]);

	// set gps data
	void setGpsData(uint64_t time_usec, struct gps_message *gps);

	// set baro data
	void setBaroData(uint64_t time_usec, float data);

	// set airspeed data
	void setAirspeedData(uint64_t time_usec, float true_airspeed, float eas2tas);

	// set range data
	void setRangeData(uint64_t time_usec, float data);

	// set optical flow data
	void setOpticalFlowData(uint64_t time_usec, flow_message *flow);

	// set external vision position and attitude data
	void setExtVisionData(uint64_t time_usec, ext_vision_message *evdata);

	// return a address to the parameters struct
	// in order to give access to the application
	parameters *getParamHandle() {return &_params;}

	// set vehicle landed status data
	void set_in_air_status(bool in_air) {_control_status.flags.in_air = in_air;}

	// set flag if synthetic sideslip measurement should be fused
	void set_fuse_beta_flag(bool fuse_beta) {_control_status.flags.fuse_beta = fuse_beta;}

	// return true if the global position estimate is valid
	virtual bool global_position_is_valid() = 0;

	// return true if the EKF is dead reckoning the position using inertial data only
	virtual bool inertial_dead_reckoning() = 0;

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
	// get the velocity of the body frame origin in local NED earth frame
	void get_velocity(float *vel)
	{
		// calculate the average angular rate across the last IMU update
		Vector3f ang_rate = _imu_sample_new.delta_ang * (1.0f/_imu_sample_new.delta_ang_dt);
		// calculate the velocity of the relative to the body origin
		// Note % operator has been overloaded to performa cross product
		Vector3f vel_imu_rel_body = cross_product(ang_rate , _params.imu_pos_body);
		// rotate the relative velocty into earth frame and subtract from the EKF velocity
		// (which is at the IMU) to get velocity of the body origin
		Vector3f vel_earth = _output_new.vel - _R_to_earth_now * vel_imu_rel_body;
		// copy to output
		for (unsigned i = 0; i < 3; i++) {
			vel[i] = vel_earth(i);
		}
	}
	// get the position of the body frame origin in local NED earth frame
	void get_position(float *pos)
	{
		// rotate the position of the IMU relative to the boy origin into earth frame
		Vector3f pos_offset_earth = _R_to_earth_now * _params.imu_pos_body;
		// subtract from the EKF position (which is at the IMU) to get position at the body origin
		for (unsigned i = 0; i < 3; i++) {
			pos[i] = _output_new.pos(i) - pos_offset_earth(i);
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

	virtual void get_accel_bias(float bias[3]) = 0;
	virtual void get_gyro_bias(float bias[3]) = 0;

	// get EKF mode status
	void get_control_mode(uint16_t *val)
	{
		*val = _control_status.value;
	}

	// get EKF internal fault status
	void get_filter_fault_status(uint16_t *val)
	{
		*val = _fault_status.value;
	}

	// get GPS check status
	virtual void get_gps_check_status(uint16_t *val) = 0;

	// return the amount the local vertical position changed in the last reset and the number of reset events
	virtual void get_posD_reset(float *delta, uint8_t *counter) = 0;

	// return the amount the local vertical velocity changed in the last reset and the number of reset events
	virtual void get_velD_reset(float *delta, uint8_t *counter) = 0;

	// return the amount the local horizontal position changed in the last reset and the number of reset events
	virtual void get_posNE_reset(float delta[2], uint8_t *counter) = 0;

	// return the amount the local horizontal velocity changed in the last reset and the number of reset events
	virtual void get_velNE_reset(float delta[2], uint8_t *counter) = 0;

	// return the amount the quaternion has changed in the last reset and the number of reset events
	virtual void get_quat_reset(float delta_quat[4], uint8_t *counter) = 0;

	// get EKF innovation consistency check status information comprising of:
	// status - a bitmask integer containing the pass/fail status for each EKF measurement innovation consistency check
	// Innovation Test Ratios - these are the ratio of the innovation to the acceptance threshold.
	// A value > 1 indicates that the sensor measurement has exceeded the maximum acceptable level and has been rejected by the EKF
	// Where a measurement type is a vector quantity, eg magnetoemter, GPS position, etc, the maximum value is returned.
	virtual void get_innovation_test_status(uint16_t *status, float *mag, float *vel, float *pos, float *hgt, float *tas, float *hagl) = 0;

	// return a bitmask integer that describes which state estimates can be used for flight control
	virtual void get_ekf_soln_status(uint16_t *status) = 0;

protected:

	parameters _params;		// filter parameters

	/*
	 OBS_BUFFER_LENGTH defines how many observations (non-IMU measurements) we can buffer
	 which sets the maximum frequency at which we can process non-IMU measurements. Measurements that
	 arrive too soon after the previous measurement will not be processed.
	 max freq (Hz) = (OBS_BUFFER_LENGTH - 1) / (IMU_BUFFER_LENGTH * FILTER_UPDATE_PERIOD_MS * 0.001)
	 This can be adjusted to match the max sensor data rate plus some margin for jitter.
	*/
	uint8_t _obs_buffer_length;
	/*
	IMU_BUFFER_LENGTH defines how many IMU samples we buffer which sets the time delay from current time to the
	EKF fusion time horizon and therefore the maximum sensor time offset relative to the IMU that we can compensate for.
	max sensor time offet (msec) =  IMU_BUFFER_LENGTH * FILTER_UPDATE_PERIOD_MS
	This can be adjusted to a value that is FILTER_UPDATE_PERIOD_MS longer than the maximum observation time delay.
	*/
	uint8_t _imu_buffer_length;
	static const unsigned FILTER_UPDATE_PERIOD_MS = 12;	// ekf prediction period in milliseconds - this should ideally be an integer multiple of the IMU time delta

	unsigned _min_obs_interval_us; // minimum time interval between observations that will guarantee data is not lost (usec)

	float _dt_imu_avg;	// average imu update period in s

	imuSample _imu_sample_delayed;	// captures the imu sample on the delayed time horizon

	// measurement samples capturing measurements on the delayed time horizon
	magSample _mag_sample_delayed;
	baroSample _baro_sample_delayed;
	gpsSample _gps_sample_delayed;
	rangeSample _range_sample_delayed;
	airspeedSample _airspeed_sample_delayed;
	flowSample _flow_sample_delayed;
	extVisionSample _ev_sample_delayed;

	outputSample _output_sample_delayed;	// filter output on the delayed time horizon
	outputSample _output_new;	// filter output on the non-delayed time horizon
	imuSample _imu_sample_new;	// imu sample capturing the newest imu data
	Matrix3f _R_to_earth_now; // rotation matrix from body to earth frame at current time

	uint64_t _imu_ticks;	// counter for imu updates

	bool _imu_updated;      // true if the ekf should update (completed downsampling process)
	bool _initialised;      // true if the ekf interface instance (data buffering) is initialized

	bool _NED_origin_initialised;
	bool _gps_speed_valid;
	float _gps_origin_eph; // horizontal position uncertainty of the GPS origin
	float _gps_origin_epv; // vertical position uncertainty of the GPS origin
	struct map_projection_reference_s _pos_ref;    // Contains WGS-84 position latitude and longitude (radians) of the EKF origin
	struct map_projection_reference_s _gps_pos_prev;    // Contains WGS-84 position latitude and longitude (radians) of the previous GPS message
	float _gps_alt_prev;	// height from the previous GPS message (m)

	// innovation consistency check monitoring ratios
	float _yaw_test_ratio;          // yaw innovation consistency check ratio
	float _mag_test_ratio[3];       // magnetometer XYZ innovation consistency check ratios
	float _vel_pos_test_ratio[6];   // velocity and position innovation consistency check ratios
	float _tas_test_ratio;		// tas innovation consistency check ratio
	float _terr_test_ratio;		// height above terrain measurement innovation consistency check ratio
	float _beta_test_ratio;		// sideslip innovation consistency check ratio
	innovation_fault_status_u _innov_check_fail_status{};

	bool _is_dead_reckoning;	// true if we are no longer fusing measurements that constrain horizontal velocity drift

	// IMU vibration monitoring
	Vector3f _delta_ang_prev;	// delta angle from the previous IMU measurement
	Vector3f _delta_vel_prev;	// delta velocity from the previous IMU measurement
	float _vibe_metrics[3];		// IMU vibration metrics
					// [0] Level of coning vibration in the IMU delta angles (rad^2)
					// [1] high frequency vibraton level in the IMU delta angle data (rad)
					// [2] high frequency vibration level in the IMU delta velocity data (m/s)

	// data buffer instances
	RingBuffer<imuSample> _imu_buffer;
	RingBuffer<gpsSample> _gps_buffer;
	RingBuffer<magSample> _mag_buffer;
	RingBuffer<baroSample> _baro_buffer;
	RingBuffer<rangeSample> _range_buffer;
	RingBuffer<airspeedSample> _airspeed_buffer;
	RingBuffer<flowSample> 	_flow_buffer;
	RingBuffer<extVisionSample> _ext_vision_buffer;
	RingBuffer<outputSample> _output_buffer;

	uint64_t _time_last_imu;	// timestamp of last imu sample in microseconds
	uint64_t _time_last_gps;	// timestamp of last gps measurement in microseconds
	uint64_t _time_last_mag;	// timestamp of last magnetometer measurement in microseconds
	uint64_t _time_last_baro;	// timestamp of last barometer measurement in microseconds
	uint64_t _time_last_range;	// timestamp of last range measurement in microseconds
	uint64_t _time_last_airspeed;	// timestamp of last airspeed measurement in microseconds
	uint64_t _time_last_ext_vision; // timestamp of last external vision measurement in microseconds
	uint64_t _time_last_optflow;

	fault_status_u _fault_status{};

	// allocate data buffers and intialise interface variables
	bool initialise_interface(uint64_t timestamp);

	// free buffer memory
	void unallocate_buffers();

	float _mag_declination_gps;         // magnetic declination returned by the geo library using the last valid GPS position (rad)
	float _mag_declination_to_save_deg; // magnetic declination to save to EKF2_MAG_DECL (deg)

	// this is the current status of the filter control modes
	filter_control_status_u _control_status{};

	// this is the previous status of the filter control modes - used to detect mode transitions
	filter_control_status_u _control_status_prev{};

	// perform a vector cross product
	Vector3f cross_product(const Vector3f &vecIn1, const Vector3f &vecIn2);

	// calculate the inverse rotation matrix from a quaternion rotation
	Matrix3f quat_to_invrotmat(const Quaternion& quat);

};
