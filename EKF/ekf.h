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

	Ekf() = default;
	~Ekf() = default;

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

	// gets the innovations of synthetic sideslip measurement
	void get_beta_innov(float *beta_innov);

	// gets the innovation variance of the synthetic sideslip measurement
	void get_beta_innov_var(float *beta_innov_var);

	// gets the innovation variance of the heading measurement
	void get_heading_innov_var(float *heading_innov_var);

	// gets the innovation variance of the flow measurement
	void get_flow_innov_var(float flow_innov_var[2]);

	// gets the innovation of the flow measurement
	void get_flow_innov(float flow_innov[2]);

	// gets the innovation variance of the drag specific force measurement
	void get_drag_innov_var(float drag_innov_var[2]);

	// gets the innovation of the drag specific force measurement
	void get_drag_innov(float drag_innov[2]);

	// gets the innovation variance of the HAGL measurement
	void get_hagl_innov_var(float *hagl_innov_var);

	// gets the innovation of the HAGL measurement
	void get_hagl_innov(float *hagl_innov);

	// get the state vector at the delayed time horizon
	void get_state_delayed(float *state);

	// get the wind velocity in m/s
	void get_wind_velocity(float *wind);

	// get the diagonal elements of the covariance matrix
	void get_covariances(float *covariances);

	// ask estimator for sensor data collection decision and do any preprocessing if required, returns true if not defined
	bool collect_gps(uint64_t time_usec, struct gps_message *gps);
	bool collect_imu(imuSample &imu);

	// get the ekf WGS-84 origin position and height and the system time it was last set
	// return true if the origin is valid
	bool get_ekf_origin(uint64_t *origin_time, map_projection_reference_s *origin_pos, float *origin_alt);

	// get the 1-sigma horizontal and vertical position uncertainty of the ekf WGS-84 position
	void get_ekf_gpos_accuracy(float *ekf_eph, float *ekf_epv, bool *dead_reckoning);

	// get the 1-sigma horizontal and vertical position uncertainty of the ekf local position
	void get_ekf_lpos_accuracy(float *ekf_eph, float *ekf_epv, bool *dead_reckoning);

	// get the 1-sigma horizontal and vertical velocity uncertainty
	void get_ekf_vel_accuracy(float *ekf_evh, float *ekf_evv, bool *dead_reckoning);

	void get_vel_var(Vector3f &vel_var);

	void get_pos_var(Vector3f &pos_var);

	// return an array containing the output predictor angular, velocity and position tracking
	// error magnitudes (rad), (m/s), (m)
	void get_output_tracking_error(float error[3]);

	/*
	Returns  following IMU vibration metrics in the following array locations
	0 : Gyro delta angle coning metric = filtered length of (delta_angle x prev_delta_angle)
	1 : Gyro high frequency vibe = filtered length of (delta_angle - prev_delta_angle)
	2 : Accel high frequency vibe = filtered length of (delta_velocity - prev_delta_velocity)
	*/
	void get_imu_vibe_metrics(float vibe[3]);

	// return true if the global position estimate is valid
	bool global_position_is_valid();

	// return true if the EKF is dead reckoning the position using inertial data only
	bool inertial_dead_reckoning();

	// return true if the etimate is valid
	// return the estimated terrain vertical position relative to the NED origin
	bool get_terrain_vert_pos(float *ret);

	// get the accerometer bias in m/s/s
	void get_accel_bias(float bias[3]);

	// get the gyroscope bias in rad/s
	void get_gyro_bias(float bias[3]);

	// get GPS check status
	void get_gps_check_status(uint16_t *val);

	// return the amount the local vertical position changed in the last reset and the number of reset events
	void get_posD_reset(float *delta, uint8_t *counter) {*delta = _state_reset_status.posD_change; *counter = _state_reset_status.posD_counter;}

	// return the amount the local vertical velocity changed in the last reset and the number of reset events
	void get_velD_reset(float *delta, uint8_t *counter) {*delta = _state_reset_status.velD_change; *counter = _state_reset_status.velD_counter;}

	// return the amount the local horizontal position changed in the last reset and the number of reset events
	void get_posNE_reset(float delta[2], uint8_t *counter)
	{
		memcpy(delta, &_state_reset_status.posNE_change._data[0], sizeof(_state_reset_status.posNE_change._data));
		*counter = _state_reset_status.posNE_counter;
	}

	// return the amount the local horizontal velocity changed in the last reset and the number of reset events
	void get_velNE_reset(float delta[2], uint8_t *counter)
	{
		memcpy(delta, &_state_reset_status.velNE_change._data[0], sizeof(_state_reset_status.velNE_change._data));
		*counter = _state_reset_status.velNE_counter;
	}

	// return the amount the quaternion has changed in the last reset and the number of reset events
	void get_quat_reset(float delta_quat[4], uint8_t *counter)
	{
		memcpy(delta_quat, &_state_reset_status.quat_change._data[0], sizeof(_state_reset_status.quat_change._data));
		*counter = _state_reset_status.quat_counter;
	}

	// get EKF innovation consistency check status information comprising of:
	// status - a bitmask integer containing the pass/fail status for each EKF measurement innovation consistency check
	// Innovation Test Ratios - these are the ratio of the innovation to the acceptance threshold.
	// A value > 1 indicates that the sensor measurement has exceeded the maximum acceptable level and has been rejected by the EKF
	// Where a measurement type is a vector quantity, eg magnetoemter, GPS position, etc, the maximum value is returned.
	void get_innovation_test_status(uint16_t *status, float *mag, float *vel, float *pos, float *hgt, float *tas, float *hagl);

	// return a bitmask integer that describes which state estimates can be used for flight control
	void get_ekf_soln_status(uint16_t *status);

private:

	static constexpr uint8_t _k_num_states{24};
	static constexpr float _k_earth_rate{0.000072921f};
	static constexpr float _gravity_mss{9.80665f};

	// reset event monitoring
	// structure containing velocity, position, height and yaw reset information
	struct {
		uint8_t velNE_counter;	// number of horizontal position reset events (allow to wrap if count exceeds 255)
		uint8_t velD_counter;	// number of vertical velocity reset events (allow to wrap if count exceeds 255)
		uint8_t posNE_counter;	// number of horizontal position reset events (allow to wrap if count exceeds 255)
		uint8_t posD_counter;	// number of vertical position reset events (allow to wrap if count exceeds 255)
		uint8_t quat_counter;	// number of quaternion reset events (allow to wrap if count exceeds 255)
		Vector2f velNE_change;  // North East velocity change due to last reset (m)
		float velD_change;	// Down velocity change due to last reset (m/s)
		Vector2f posNE_change;	// North, East position change due to last reset (m)
		float posD_change;	// Down position change due to last reset (m)
		Quaternion quat_change;	// quaternion delta due to last reset - multiply pre-reset quaternion by this to get post-reset quaternion
	} _state_reset_status{};

	float _dt_ekf_avg{0.001f * FILTER_UPDATE_PERIOD_MS};		// average update rate of the ekf
	float _dt_update{0.01f};		// delta time since last ekf update. This time can be used for filters
						// which run at the same rate as the Ekf::update() function

	stateSample _state{};		// state struct of the ekf running at the delayed time horizon

	bool _filter_initialised{false};	// true when the EKF sttes and covariances been initialised
	bool _earth_rate_initialised{false};	// true when we know the earth rotatin rate (requires GPS)

	bool _fuse_height{false};		// baro height data should be fused
	bool _fuse_pos{false};			// gps position data should be fused
	bool _fuse_hor_vel{false};		// gps horizontal velocity measurement should be fused
	bool _fuse_vert_vel{false};		// gps vertical velocity measurement should be fused

	// booleans true when fresh sensor data is available at the fusion time horizon
	bool _gps_data_ready{false};
	bool _mag_data_ready{false};
	bool _baro_data_ready{false};
	bool _range_data_ready{false};
	bool _flow_data_ready{false};
	bool _ev_data_ready{false};
	bool _tas_data_ready{false};

	uint64_t _time_last_fake_gps{0};	// last time in us at which we have faked gps measurement for static mode

	uint64_t _time_last_pos_fuse{0};   // time the last fusion of horizontal position measurements was performed (usec)
	uint64_t _time_last_vel_fuse{0};   // time the last fusion of velocity measurements was performed (usec)
	uint64_t _time_last_hgt_fuse{0};   // time the last fusion of height measurements was performed (usec)
	uint64_t _time_last_of_fuse{0};    // time the last fusion of optical flow measurements were performed (usec)
	uint64_t _time_last_arsp_fuse{0};	// time the last fusion of airspeed measurements were performed (usec)
	uint64_t _time_last_beta_fuse{0};	// time the last fusion of synthetic sideslip measurements were performed (usec)
	Vector2f _last_known_posNE;     // last known local NE position vector (m)
	float _last_disarmed_posD{0.0f};      // vertical position recorded at arming (m)
	float _imu_collection_time_adj{0.0f};	// the amount of time the IMU collection needs to be advanced to meet the target set by FILTER_UPDATE_PERIOD_MS (sec)

	uint64_t _time_acc_bias_check{0};	// last time the  accel bias check passed (usec)

	Vector3f _earth_rate_NED;	// earth rotation vector (NED) in rad/s

	matrix::Dcm<float> _R_to_earth;	// transformation matrix from body frame to earth frame from last EKF predition

	// used by magnetometer fusion mode selection
	Vector2f _accel_lpf_NE;			// Low pass filtered horizontal earth frame acceleration (m/s**2)
	float _yaw_delta_ef{0.0f};		// Recent change in yaw angle measured about the earth frame D axis (rad)
	float _yaw_rate_lpf_ef{0.0f};		// Filtered angular rate about earth frame D axis (rad/sec)
	bool _mag_bias_observable{false};	// true when there is enough rotation to make magnetometer bias errors observable
	bool _yaw_angle_observable{false};	// true when there is enough horizontal acceleration to make yaw observable
	uint64_t _time_yaw_started{0};		// last system time in usec that a yaw rotation moaneouvre was detected

	float P[_k_num_states][_k_num_states] {};	// state covariance matrix

	float _vel_pos_innov[6] {};	// innovations: 0-2 vel,  3-5 pos
	float _vel_pos_innov_var[6] {};	// innovation variances: 0-2 vel, 3-5 pos

	float _mag_innov[3] {};		// earth magnetic field innovations
	float _mag_innov_var[3] {};	// earth magnetic field innovation variance

	float _airspeed_innov{0.0f};		// airspeed measurement innovation
	float _airspeed_innov_var{0.0f};	// airspeed measurement innovation variance

	float _beta_innov{0.0f};		// synthetic sideslip measurement innovation
	float _beta_innov_var{0.0f};	// synthetic sideslip measurement innovation variance

	float _drag_innov[2] {};		// multirotor drag measurement innovation
	float _drag_innov_var[2] {};	// multirotor drag measurement innovation variance

	float _heading_innov{0.0f};		// heading measurement innovation
	float _heading_innov_var{0.0f};	// heading measurement innovation variance

	// optical flow processing
	float _flow_innov[2] {};		// flow measurement innovation
	float _flow_innov_var[2] {};	// flow innovation variance
	Vector3f _flow_gyro_bias;	// bias errors in optical flow sensor rate gyro outputs
	Vector3f _imu_del_ang_of;	// bias corrected delta angle measurements accumulated across the same time frame as the optical flow rates
	float _delta_time_of{0.0f};		// time in sec that _imu_del_ang_of was accumulated over

	float _mag_declination{0.0f};		// magnetic declination used by reset and fusion functions (rad)

	// output predictor states
	Vector3f _delta_angle_corr;	// delta angle correction vector
	imuSample _imu_down_sampled{};	// down sampled imu data (sensor rate -> filter update rate)
	Quaternion _q_down_sampled;	// down sampled quaternion (tracking delta angles between ekf update steps)
	Vector3f _vel_err_integ;	// integral of velocity tracking error
	Vector3f _pos_err_integ;	// integral of position tracking error
	float _output_tracking_error[3] {}; // contains the magnitude of the angle, velocity and position track errors (rad, m/s, m)

	// variables used for the GPS quality checks
	float _gpsDriftVelN{0.0f};		// GPS north position derivative (m/s)
	float _gpsDriftVelE{0.0f};		// GPS east position derivative (m/s)
	float _gps_drift_velD{0.0f};		// GPS down position derivative (m/s)
	float _gps_velD_diff_filt{0.0f};	// GPS filtered Down velocity (m/s)
	float _gps_velN_filt{0.0f};		// GPS filtered North velocity (m/s)
	float _gps_velE_filt{0.0f};		// GPS filtered East velocity (m/s)
	uint64_t _last_gps_fail_us{0};	// last system time in usec that the GPS failed it's checks

	// Variables used to publish the WGS-84 location of the EKF local NED origin
	uint64_t _last_gps_origin_time_us{0};  // time the origin was last set (uSec)
	float _gps_alt_ref{0.0f};		// WGS-84 height (m)

	// Variables used to initialise the filter states
	uint32_t _hgt_counter{0};		// number of height samples read during initialisation
	float _rng_filt_state{0.0f};		// filtered height measurement
	uint32_t _mag_counter{0};		// number of magnetometer samples read during initialisation
	uint32_t _ev_counter{0};		// number of exgernal vision samples read during initialisation
	uint64_t _time_last_mag{0};	// measurement time of last magnetomter sample
	Vector3f _mag_filt_state;	// filtered magnetometer measurement
	Vector3f _delVel_sum;		// summed delta velocity
	float _hgt_sensor_offset{0.0f};	// set as necessary if desired to maintain the same height after a height reset (m)
	float _baro_hgt_offset{0.0f};		// baro height reading at the local NED origin (m)

	// Variables used to control activation of post takeoff functionality
	float _last_on_ground_posD{0.0f};	// last vertical position when the in_air status was false (m)
	bool _flt_mag_align_complete{true};	// true when the in-flight mag field alignment has been completed
	uint64_t _time_last_movement{0};	// last system time in usec that sufficient movement to use 3-axis magnetometer fusion was detected
	float _saved_mag_variance[6] {};	// magnetic field state variances that have been saved for use at the next initialisation (Ga**2)

	gps_check_fail_status_u _gps_check_fail_status{};

	// variables used to inhibit accel bias learning
	bool _accel_bias_inhibit{false};	// true when the accel bias learning is being inhibited
	float _accel_mag_filt{0.0f};		// acceleration magnitude after application of a decaying envelope filter (m/sec**2)
	float _ang_rate_mag_filt{0.0f};	// angular rate magnitude after application of a decaying envelope filter (rad/sec)
	Vector3f _prev_dvel_bias_var;	// saved delta velocity XYZ bias variances (m/sec)**2

	// Terrain height state estimation
	float _terrain_vpos{0.0f};		// estimated vertical position of the terrain underneath the vehicle in local NED frame (m)
	float _terrain_var{1e4f};		// variance of terrain position estimate (m^2)
	float _hagl_innov{0.0f};		// innovation of the last height above terrain measurement (m)
	float _hagl_innov_var{0.0f};		// innovation variance for the last height above terrain measurement (m^2)
	uint64_t _time_last_hagl_fuse;		// last system time in usec that the hagl measurement failed it's checks
	bool _terrain_initialised{false};	// true when the terrain estimator has been intialised
	float _sin_tilt_rng{0.0f};		// sine of the range finder tilt rotation about the Y body axis
	float _cos_tilt_rng{0.0f};		// cosine of the range finder tilt rotation about the Y body axis
	float _R_rng_to_earth_2_2{0.0f};	// 2,2 element of the rotation matrix from sensor frame to earth frame
	bool _range_data_continuous{false};	// true when we are receiving range finder data faster than a 2Hz average
	float _dt_last_range_update_filt_us{0.0f};	// filtered value of the delta time elapsed since the last range measurement came into
							// the filter (microseconds)

	// height sensor fault status
	bool _baro_hgt_faulty{false};		// true if valid baro data is unavailable for use
	bool _gps_hgt_faulty{false};		// true if valid gps height data is unavailable for use
	bool _rng_hgt_faulty{false};		// true if valid rnage finder height data is unavailable for use
	int _primary_hgt_source{VDIST_SENSOR_BARO};	// priary source of height data set at initialisation

	// imu fault status
	uint64_t _time_bad_vert_accel{0};	// last time a bad vertical accel was detected (usec)
	uint64_t _time_good_vert_accel{0};	// last time a good vertical accel was detected (usec)
	bool _bad_vert_accel_detected{false};	// true when bad vertical accelerometer data has been detected

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

	// fuse magnetometer declination measurement
	void fuseDeclination();

	// fuse airspeed measurement
	void fuseAirspeed();

	// fuse synthetic zero sideslip measurement
	void fuseSideslip();

	// fuse body frame drag specific forces for multi-rotor wind estimation
	void fuseDrag();

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

	// run the terrain estimator
	void runTerrainEstimator();

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

	// modify output filter to match the the EKF state at the fusion time horizon
	void alignOutputFilter();

	// limit the diagonal of the covariance matrix
	void fixCovarianceErrors();

	// make ekf covariance matrix symmetric between a nominated state indexe range
	void makeSymmetrical(float (&cov_mat)[_k_num_states][_k_num_states], uint8_t first, uint8_t last);

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

	// control fusion of external vision observations
	void controlExternalVisionFusion();

	// control fusion of optical flow observtions
	void controlOpticalFlowFusion();

	// control fusion of GPS observations
	void controlGpsFusion();

	// control fusion of magnetometer observations
	void controlMagFusion();

	// control fusion of range finder observations
	void controlRangeFinderFusion();

	// control fusion of air data observations
	void controlAirDataFusion();

	// control fusion of synthetic sideslip observations
	void controlBetaFusion();

	// control fusion of multi-rotor drag specific force observations
	void controlDragFusion();

	// control fusion of pressure altitude observations
	void controlBaroFusion();

	// control fusion of velocity and position observations
	void controlVelPosFusion();

	// control for height sensor timeouts, sensor changes and state resets
	void controlHeightSensorTimeouts();

	// return the square of two floating point numbers - used in auto coded sections
	inline float sq(float var)
	{
		return var * var;
	}

	// zero the specified range of rows in the state covariance matrix
	void zeroRows(float (&cov_mat)[_k_num_states][_k_num_states], uint8_t first, uint8_t last);

	// zero the specified range of columns in the state covariance matrix
	void zeroCols(float (&cov_mat)[_k_num_states][_k_num_states], uint8_t first, uint8_t last);

	// calculate the measurement variance for the optical flow sensor
	float calcOptFlowMeasVar();

	// rotate quaternion covariances into variances for an equivalent rotation vector
	Vector3f calcRotVecVariances();

	// initialise the quaternion covariances using rotation vector variances
	void initialiseQuatCovariances(Vector3f &rot_vec_var);

	// perform a limited reset of the magnetic field state covariances
	void resetMagCovariance();

	// perform a limited reset of the wind state covariances
	void resetWindCovariance();

	// perform a reset of the wind states
	void resetWindStates();

	// check that the range finder data is continuous
	void checkRangeDataContinuity();

};
