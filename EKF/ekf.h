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
 * @file ekf.h
 * Class for core functions for ekf attitude and position estimator.
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "estimator_interface.h"
#include "geo.h"

class Ekf : public EstimatorInterface
{
public:

	Ekf();
	~Ekf();

	// initialise variables to sane values (also interface class)
	bool init(uint64_t timestamp);

	// should be called every time new data is pushed into the filter
	bool update();

	// gets the innovations of velocity and position measurements
	// 0-2 vel, 3-5 pos
	void get_vel_pos_innov(float vel_pos_innov[6]);

	// gets the innovations of the earth magnetic field measurements
	void get_mag_innov(float mag_innov[3]);

	// gets the innovations of the heading measurement
	void get_heading_innov(float *heading_innov);

	// gets the innovation variances of velocity and position measurements
	// 0-2 vel, 3-5 pos
	void get_vel_pos_innov_var(float vel_pos_innov_var[6]);

	// gets the innovation variances of the earth magnetic field measurements
	void get_mag_innov_var(float mag_innov_var[3]);

	// gets the innovations of airspeed measurement
	void get_airspeed_innov(float *airspeed_innov);

	// gets the innovation variance of the airspeed measurement
	void get_airspeed_innov_var(float *airspeed_innov_var);

	// gets the innovation variance of the heading measurement
	void get_heading_innov_var(float *heading_innov_var);

	// gets the innovation variance of the flow measurement
	void get_flow_innov_var(float flow_innov_var[2]);

	// gets the innovation of the flow measurement
	void get_flow_innov(float flow_innov[2]);

	// gets the innovation variance of the HAGL measurement
	void get_hagl_innov_var(float *hagl_innov_var);

	// gets the innovation of the HAGL measurement
	void get_hagl_innov(float *hagl_innov);

	// get the state vector at the delayed time horizon
	void get_state_delayed(float *state);

	// get the diagonal elements of the covariance matrix
	void get_covariances(float *covariances);

	// ask estimator for sensor data collection decision and do any preprocessing if required, returns true if not defined
	bool collect_gps(uint64_t time_usec, struct gps_message *gps);
	bool collect_imu(imuSample &imu);

	// get the ekf WGS-84 origin position and height and the system time it was last set
	void get_ekf_origin(uint64_t *origin_time, map_projection_reference_s *origin_pos, float *origin_alt);

	// get the 1-sigma horizontal and vertical position uncertainty of the ekf WGS-84 position
	void get_ekf_accuracy(float *ekf_eph, float *ekf_epv, bool *dead_reckoning);

	void get_vel_var(Vector3f &vel_var);

	void get_pos_var(Vector3f &pos_var);

	// return true if the global position estimate is valid
	bool global_position_is_valid();

	// return true if the etimate is valid
	// return the estimated terrain vertical position relative to the NED origin
	bool get_terrain_vert_pos(float *ret);

private:

	static const uint8_t _k_num_states = 24;
	static constexpr float _k_earth_rate = 0.000072921f;

	stateSample _state;		// state struct of the ekf running at the delayed time horizon

	bool _filter_initialised;	// true when the EKF sttes and covariances been initialised
	bool _earth_rate_initialised;	// true when we know the earth rotatin rate (requires GPS)

	bool _fuse_height;		// baro height data should be fused
	bool _fuse_pos;			// gps position data should be fused
	bool _fuse_hor_vel;		// gps horizontal velocity measurement should be fused
	bool _fuse_vert_vel;		// gps vertical velocity measurement should be fused
	bool _fuse_flow;		// flow measurement should be fused
	bool _fuse_hagl_data;		// if true then range data will be fused to estimate terrain height

	uint64_t _time_last_fake_gps;	// last time in us at which we have faked gps measurement for static mode

	uint64_t _time_last_pos_fuse;   // time the last fusion of horizontal position measurements was performed (usec)
	uint64_t _time_last_vel_fuse;   // time the last fusion of velocity measurements was performed (usec)
	uint64_t _time_last_hgt_fuse;   // time the last fusion of height measurements was performed (usec)
	uint64_t _time_last_of_fuse;    // time the last fusion of optical flow measurements were performed (usec)
	Vector2f _last_known_posNE;     // last known local NE position vector (m)
	float _last_disarmed_posD;      // vertical position recorded at arming (m)

	Vector3f _earth_rate_NED;	// earth rotation vector (NED) in rad/s

	matrix::Dcm<float> _R_prev;	// transformation matrix from earth frame to body frame of previous ekf step

	float P[_k_num_states][_k_num_states];	// state covariance matrix
	float KH[_k_num_states][_k_num_states]; // intermediate variable for the covariance update
	float KHP[_k_num_states][_k_num_states]; // intermediate variable for the covariance update

	float _vel_pos_innov[6];	// innovations: 0-2 vel,  3-5 pos
	float _vel_pos_innov_var[6];	// innovation variances: 0-2 vel, 3-5 pos

	float _mag_innov[3];		// earth magnetic field innovations
	float _mag_innov_var[3];	// earth magnetic field innovation variance

	float _airspeed_innov;		// airspeed measurement innovation
	float _airspeed_innov_var;  // airspeed measurement innovation variance

	float _heading_innov;		// heading measurement innovation
	float _heading_innov_var;	// heading measurement innovation variance

	Vector3f _tilt_err_vec;         // Vector of the most recent attitude error correction from velocity and position fusion
	float _tilt_err_length_filt;    // filtered length of _tilt_err_vec

	// optical flow processing
	float _flow_innov[2];		// flow measurement innovation
	float _flow_innov_var[2];	// flow innovation variance
	Vector2f _flow_gyro_bias;	// bias errors in optical flow sensor rate gyro outputs
	Vector2f _imu_del_ang_of;	// bias corrected XY delta angle measurements accumulated across the same time frame as the optical flow rates
	float _delta_time_of;		// time in sec that _imu_del_ang_of was accumulated over

	float _mag_declination;		// magnetic declination used by reset and fusion functions (rad)

	// complementary filter states
	Vector3f _delta_angle_corr;	// delta angle correction vector
	Vector3f _delta_vel_corr;	// delta velocity correction vector
	Vector3f _vel_corr;		// velocity correction vector
	imuSample _imu_down_sampled;	// down sampled imu data (sensor rate -> filter update rate)
	Quaternion _q_down_sampled;	// down sampled quaternion (tracking delta angles between ekf update steps)

	// variables used for the GPS quality checks
	float _gpsDriftVelN;		// GPS north position derivative (m/s)
	float _gpsDriftVelE;		// GPS east position derivative (m/s)
	float _gps_drift_velD;		// GPS down position derivative (m/s)
	float _gps_velD_diff_filt;	// GPS filtered Down velocity (m/s)
	float _gps_velN_filt;		// GPS filtered North velocity (m/s)
	float _gps_velE_filt;		// GPS filtered East velocity (m/s)
	uint64_t _last_gps_fail_us;	// last system time in usec that the GPS failed it's checks

	// Variables used to publish the WGS-84 location of the EKF local NED origin
	uint64_t _last_gps_origin_time_us;  // time the origin was last set (uSec)
	float _gps_alt_ref;		// WGS-84 height (m)

	// Variables used to initialise the filter states
	uint32_t _hgt_counter;		// number of height samples taken
	float _hgt_filt_state;		// filtered height measurement
	uint32_t _mag_counter;		// number of magnetometer samples taken
	uint64_t _time_last_mag;	// measurement time of last magnetomter sample
	Vector3f _mag_filt_state;	// filtered magnetometer measurement
	Vector3f _delVel_sum;		// summed delta velocity
	float _hgt_sensor_offset;	// height that needs to be subtracted from the primary height sensor so that it reads zero height at the origin (m)

	gps_check_fail_status_u _gps_check_fail_status;

	// Terrain height state estimation
	float _terrain_vpos;		// estimated vertical position of the terrain underneath the vehicle in local NED frame (m)
	float _terrain_var;		// variance of terrain position estimate (m^2)
	float _hagl_innov;		// innovation of the last height above terrain measurement (m)
	float _hagl_innov_var;		// innovation variance for the last height above terrain measurement (m^2)
	uint64_t _time_last_hagl_fuse;	// last system time in usec that the hagl measurement failed it's checks
	bool _terrain_initialised;	// true when the terrain estimator has been intialised

	// height sensor fault status
	bool _baro_hgt_faulty;		// true if valid baro data is unavailable for use
	bool _gps_hgt_faulty;		// true if valid gps height data is unavailable for use
	bool _rng_hgt_faulty;		// true if valid rnage finder height data is unavailable for use
	int _primary_hgt_source;	// priary source of height data set at initialisation

	float _baro_hgt_offset;		// number of metres the baro height origin is above the local NED origin (m)

	// update the real time complementary filter states. This includes the prediction
	// and the correction step
	void calculateOutputStates();

	// initialise filter states of both the delayed ekf and the real time complementary filter
	bool initialiseFilter(void);

	// initialise ekf covariance matrix
	void initialiseCovariance();

	// predict ekf state
	void predictState();

	// predict ekf covariance
	void predictCovariance();

	// ekf sequential fusion of magnetometer measurements
	void fuseMag();

	// fuse the first euler angle from either a 321 or 312 rotation sequence as the observation (currently measures yaw using the magnetometer)
	void fuseHeading();

	// fuse projecton of magnetometer onto horizontal plane
	void fuseMag2D();

	// fuse magnetometer declination measurement
	void fuseDeclination();

	// fuse airspeed measurement
	void fuseAirspeed();

	// fuse velocity and position measurements (also barometer height)
	void fuseVelPosHeight();

	// reset velocity states of the ekf
	bool resetVelocity();

	// fuse optical flow line of sight rate measurements
	void fuseOptFlow();

	// calculate optical flow bias errors
	void calcOptFlowBias();

	// initialise the terrain vertical position estimator
	// return true if the initialisation is successful
	bool initHagl();

	// predict the terrain vertical position state and variance
	void predictHagl();

	// update the terrain vertical position estimate using a height above ground measurement from the range finder
	void fuseHagl();

	// reset the heading and magnetic field states using the declination and magnetometer measurements
	// return true if successful
	bool resetMagHeading(Vector3f &mag_init);

	// calculate the magnetic declination to be used by the alignment and fusion processing
	void calcMagDeclination();

	// reset position states of the ekf (only vertical position)
	bool resetPosition();

	// reset height state of the ekf
	void resetHeight();

	void makeCovSymetrical();

	// limit the diagonal of the covariance matrix
	void limitCov();

	// make ekf covariance matrix symmetric
	void makeSymmetrical();

	// constrain the ekf states
	void constrainStates();

	// generic function which will perform a fusion step given a kalman gain K
	// and a scalar innovation value
	void fuse(float *K, float innovation);

	// calculate the earth rotation vector from a given latitude
	void calcEarthRateNED(Vector3f &omega, double lat_rad) const;

	// return true id the GPS quality is good enough to set an origin and start aiding
	bool gps_is_good(struct gps_message *gps);

	// Control the filter fusion modes
	void controlFusionModes();

	// Determine if we are airborne or motors are armed
	void calculateVehicleStatus();

	// return the square of two floating point numbers - used in auto coded sections
	inline float sq(float var)
	{
		return var * var;
	}

	// zero the specified range of rows in the state covariance matrix
	void zeroRows(float (&cov_mat)[_k_num_states][_k_num_states], uint8_t first, uint8_t last);

	// zero the specified range of columns in the state covariance matrix
	void zeroCols(float (&cov_mat)[_k_num_states][_k_num_states], uint8_t first, uint8_t last);
};
