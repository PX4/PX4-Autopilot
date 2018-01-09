/****************************************************************************
 *
 *   Copyright (c) 2015-2017 PX4 Development Team. All rights reserved.
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
 * @file ekf2_main.cpp
 * Implementation of the attitude and position estimator.
 *
 * @author Roman Bapst
 */

#include <cfloat>

#include <controllib/block/BlockParam.hpp>
#include <controllib/blocks.hpp>
#include <drivers/drv_hrt.h>
#include <ecl/EKF/ekf.h>
#include <mathlib/mathlib.h>
#include <px4_defines.h>
#include <px4_module.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/ekf2_innovations.h>
#include <uORB/topics/ekf2_timestamps.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_bias.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/landing_target_pose.h>

using control::BlockParamFloat;
using control::BlockParamExtFloat;
using control::BlockParamInt;
using control::BlockParamExtInt;

using math::constrain;

extern "C" __EXPORT int ekf2_main(int argc, char *argv[]);

class Ekf2 final : public control::SuperBlock, public ModuleBase<Ekf2>
{
public:
	Ekf2();
	~Ekf2() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static Ekf2 *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	void set_replay_mode(bool replay) { _replay_mode = replay; }

	static void	task_main_trampoline(int argc, char *argv[]);

	int print_status() override;

private:
	int getRangeSubIndex(const int *subs); ///< get subscription index of first downward-facing range sensor

	bool 	_replay_mode = false;			///< true when we use replay data from a log

	// time slip monitoring
	uint64_t _integrated_time_us = 0;	///< integral of gyro delta time from start (uSec)
	uint64_t _start_time_us = 0;		///< system time at EKF start (uSec)
	int64_t _last_time_slip_us = 0;		///< Last time slip (uSec)

	// Initialise time stamps used to send sensor data to the EKF and for logging
	uint64_t _timestamp_mag_us = 0;		///< magnetomer data timestamp (uSec)
	uint64_t _timestamp_balt_us = 0;	///< pressure altitude data timestamp (uSec)
	uint8_t _invalid_mag_id_count = 0;	///< number of times an invalid magnetomer device ID has been detected

	// Used to down sample magnetometer data
	float _mag_data_sum[3] = {};			///< summed magnetometer readings (Gauss)
	uint64_t _mag_time_sum_ms = 0;		///< summed magnetoemter time stamps (mSec)
	uint8_t _mag_sample_count = 0;		///< number of magnetometer measurements summed during downsampling
	uint32_t _mag_time_ms_last_used =
		0;	///< time stamp of the last averaged magnetometer measurement sent to the EKF (mSec)

	// Used to down sample barometer data
	float _balt_data_sum = 0.0f;			///< summed pressure altitude readings (m)
	uint64_t _balt_time_sum_ms = 0;		///< summed pressure altitude time stamps (mSec)
	uint8_t _balt_sample_count = 0;		///< number of barometric altitude measurements summed
	uint32_t _balt_time_ms_last_used =
		0;	///< time stamp of the last averaged barometric altitude measurement sent to the EKF (mSec)

	// Used to check, save and use learned magnetometer biases
	hrt_abstime _last_magcal_us = 0;	///< last time the EKF was operating a mode that estimates magnetomer biases (uSec)
	hrt_abstime _total_cal_time_us = 0;	///< accumulated calibration time since the last save

	float _last_valid_mag_cal[3] = {};	///< last valid XYZ magnetometer bias estimates (mGauss)
	bool _valid_cal_available[3] = {};	///< true when an unsaved valid calibration for the XYZ magnetometer bias is available
	float _last_valid_variance[3] = {};	///< variances for the last valid magnetometer XYZ bias estimates (mGauss**2)

	// Used to filter velocity innovations during pre-flight checks
	bool _preflt_horiz_fail = false;	///< true if preflight horizontal innovation checks are failed
	bool _preflt_vert_fail = false;		///< true if preflight vertical innovation checks are failed
	bool _preflt_fail = false;		///< true if any preflight innovation checks are failed
	Vector2f _vel_ne_innov_lpf = {};	///< Preflight low pass filtered NE axis velocity innovations (m/sec)
	float _vel_d_innov_lpf = {};		///< Preflight low pass filtered D axis velocity innovations (m/sec)
	float _hgt_innov_lpf = 0.0f;		///< Preflight low pass filtered height innovation (m)
	float _yaw_innov_magnitude_lpf = 0.0f;	///< Preflight low pass filtered yaw innovation magntitude (rad)

	static constexpr float _innov_lpf_tau_inv = 0.2f;	///< Preflight low pass filter time constant inverse (1/sec)
	static constexpr float _vel_innov_test_lim =
		0.5f;	///< Maximum permissible velocity innovation to pass pre-flight checks (m/sec)
	static constexpr float _hgt_innov_test_lim =
		1.5f;	///< Maximum permissible height innovation to pass pre-flight checks (m)
	static constexpr float _nav_yaw_innov_test_lim =
		0.25f;	///< Maximum permissible yaw innovation to pass pre-flight checks when aiding inertial nav using NE frame observations (rad)
	static constexpr float _yaw_innov_test_lim =
		0.52f;	///< Maximum permissible yaw innovation to pass pre-flight checks when not aiding inertial nav using NE frame observations (rad)
	const float _vel_innov_spike_lim = 2.0f * _vel_innov_test_lim;	///< preflight velocity innovation spike limit (m/sec)
	const float _hgt_innov_spike_lim = 2.0f * _hgt_innov_test_lim;	///< preflight position innovation spike limit (m)

	orb_advert_t _att_pub{nullptr};
	orb_advert_t _wind_pub{nullptr};
	orb_advert_t _estimator_status_pub{nullptr};
	orb_advert_t _estimator_innovations_pub{nullptr};
	orb_advert_t _ekf2_timestamps_pub{nullptr};
	orb_advert_t _sensor_bias_pub{nullptr};

	uORB::Publication<vehicle_local_position_s> _vehicle_local_position_pub;
	uORB::Publication<vehicle_global_position_s> _vehicle_global_position_pub;

	// Used to correct baro data for positional errors
	Vector3f _vel_body_wind = {};	// XYZ velocity relative to wind in body frame (m/s)

	Ekf _ekf;

	parameters *_params;	///< pointer to ekf parameter struct (located in _ekf class instance)

	BlockParamExtInt
	_obs_dt_min_ms;	///< Maximmum time delay of any sensor used to increse buffer length to handle large timing jitter (mSec)
	BlockParamExtFloat _mag_delay_ms;	///< magnetometer measurement delay relative to the IMU (mSec)
	BlockParamExtFloat _baro_delay_ms;	///< barometer height measurement delay relative to the IMU (mSec)
	BlockParamExtFloat _gps_delay_ms;	///< GPS measurement delay relative to the IMU (mSec)
	BlockParamExtFloat
	_flow_delay_ms;	///< optical flow measurement delay relative to the IMU (mSec) - this is to the middle of the optical flow integration interval
	BlockParamExtFloat _rng_delay_ms;	///< range finder measurement delay relative to the IMU (mSec)
	BlockParamExtFloat _airspeed_delay_ms;	///< airspeed measurement delay relative to the IMU (mSec)
	BlockParamExtFloat _ev_delay_ms;	///< off-board vision measurement delay relative to the IMU (mSec)
	BlockParamExtFloat _auxvel_delay_ms;	///< auxillary velocity measurement delay relative to the IMU (mSec)

	BlockParamExtFloat _gyro_noise;	///< IMU angular rate noise used for covariance prediction (rad/sec)
	BlockParamExtFloat _accel_noise;	///< IMU acceleration noise use for covariance prediction (m/sec**2)

	// process noise
	BlockParamExtFloat _gyro_bias_p_noise;	///< process noise for IMU rate gyro bias prediction (rad/sec**2)
	BlockParamExtFloat _accel_bias_p_noise;///< process noise for IMU accelerometer bias prediction (m/sec**3)
	BlockParamExtFloat _mage_p_noise;	///< process noise for earth magnetic field prediction (Gauss/sec)
	BlockParamExtFloat _magb_p_noise;	///< process noise for body magnetic field prediction (Gauss/sec)
	BlockParamExtFloat _wind_vel_p_noise;	///< process noise for wind velocity prediction (m/sec**2)
	BlockParamExtFloat _terrain_p_noise;	///< process noise for terrain offset (m/sec)
	BlockParamExtFloat
	_terrain_gradient;	///< gradient of terrain used to estimate process noise due to changing position (m/m)

	BlockParamExtFloat _gps_vel_noise;	///< minimum allowed observation noise for gps velocity fusion (m/sec)
	BlockParamExtFloat _gps_pos_noise;	///< minimum allowed observation noise for gps position fusion (m)
	BlockParamExtFloat _pos_noaid_noise;	///< observation noise for non-aiding position fusion (m)
	BlockParamExtFloat _baro_noise;	///< observation noise for barometric height fusion (m)
	BlockParamExtFloat _baro_innov_gate;	///< barometric height innovation consistency gate size (STD)
	BlockParamExtFloat _posNE_innov_gate;	///< GPS horizontal position innovation consistency gate size (STD)
	BlockParamExtFloat _vel_innov_gate;	///< GPS velocity innovation consistency gate size (STD)
	BlockParamExtFloat _tas_innov_gate;	///< True Airspeed innovation consistency gate size (STD)

	// control of magnetometer fusion
	BlockParamExtFloat _mag_heading_noise;	///< measurement noise used for simple heading fusion (rad)
	BlockParamExtFloat _mag_noise;		///< measurement noise used for 3-axis magnetoemeter fusion (Gauss)
	BlockParamExtFloat _eas_noise;		///< measurement noise used for airspeed fusion (m/sec)
	BlockParamExtFloat _beta_innov_gate; ///< synthetic sideslip innovation consistency gate size (STD)
	BlockParamExtFloat _beta_noise;	///< synthetic sideslip noise (rad)
	BlockParamExtFloat _mag_declination_deg;///< magnetic declination (degrees)
	BlockParamExtFloat _heading_innov_gate;///< heading fusion innovation consistency gate size (STD)
	BlockParamExtFloat _mag_innov_gate;	///< magnetometer fusion innovation consistency gate size (STD)
	BlockParamExtInt _mag_decl_source;	///< bitmask used to control the handling of declination data
	BlockParamExtInt _mag_fuse_type;	///< integer used to specify the type of magnetometer fusion used
	BlockParamExtFloat _mag_acc_gate;	///< integer used to specify the type of magnetometer fusion used
	BlockParamExtFloat _mag_yaw_rate_gate;	///< yaw rate threshold used by mode select logic (rad/sec)

	BlockParamExtInt _gps_check_mask;	///< bitmask used to control which GPS quality checks are used
	BlockParamExtFloat _requiredEph;	///< maximum acceptable horiz position error (m)
	BlockParamExtFloat _requiredEpv;	///< maximum acceptable vert position error (m)
	BlockParamExtFloat _requiredSacc;	///< maximum acceptable speed error (m/s)
	BlockParamExtInt _requiredNsats;	///< minimum acceptable satellite count
	BlockParamExtFloat _requiredGDoP;	///< maximum acceptable geometric dilution of precision
	BlockParamExtFloat _requiredHdrift;	///< maximum acceptable horizontal drift speed (m/s)
	BlockParamExtFloat _requiredVdrift;	///< maximum acceptable vertical drift speed (m/s)

	// measurement source control
	BlockParamExtInt
	_fusion_mode;		///< bitmasked integer that selects which of the GPS and optical flow aiding sources will be used
	BlockParamExtInt _vdist_sensor_type;	///< selects the primary source for height data

	// range finder fusion
	BlockParamExtFloat _range_noise;	///< observation noise for range finder measurements (m)
	BlockParamExtFloat _range_noise_scaler; ///< scale factor from range to range noise (m/m)
	BlockParamExtFloat _range_innov_gate;	///< range finder fusion innovation consistency gate size (STD)
	BlockParamExtFloat _rng_gnd_clearance;	///< minimum valid value for range when on ground (m)
	BlockParamExtFloat _rng_pitch_offset;	///< range sensor pitch offset (rad)
	BlockParamExtInt _rng_aid;		///< enables use of a range finder even if primary height source is not range finder
	BlockParamExtFloat _rng_aid_hor_vel_max;	///< maximum allowed horizontal velocity for range aid (m/s)
	BlockParamExtFloat _rng_aid_height_max;	///< maximum allowed absolute altitude (AGL) for range aid (m)
	BlockParamExtFloat _rng_aid_innov_gate;	///< gate size used for innovation consistency checks for range aid fusion (STD)

	// vision estimate fusion
	BlockParamFloat _ev_pos_noise;	///< default position observation noise for exernal vision measurements (m)
	BlockParamFloat _ev_ang_noise;	///< default angular observation noise for exernal vision measurements (rad)
	BlockParamExtFloat _ev_innov_gate;	///< external vision position innovation consistency gate size (STD)

	// optical flow fusion
	BlockParamExtFloat _flow_noise;	///< best quality observation noise for optical flow LOS rate measurements (rad/sec)
	BlockParamExtFloat
	_flow_noise_qual_min;	///< worst quality observation noise for optical flow LOS rate measurements (rad/sec)
	BlockParamExtInt _flow_qual_min;	///< minimum acceptable quality integer from  the flow sensor
	BlockParamExtFloat _flow_innov_gate;	///< optical flow fusion innovation consistency gate size (STD)
	BlockParamExtFloat _flow_rate_max;	///< maximum valid optical flow rate (rad/sec)

	// sensor positions in body frame
	BlockParamExtFloat _imu_pos_x;		///< X position of IMU in body frame (m)
	BlockParamExtFloat _imu_pos_y;		///< Y position of IMU in body frame (m)
	BlockParamExtFloat _imu_pos_z;		///< Z position of IMU in body frame (m)
	BlockParamExtFloat _gps_pos_x;		///< X position of GPS antenna in body frame (m)
	BlockParamExtFloat _gps_pos_y;		///< Y position of GPS antenna in body frame (m)
	BlockParamExtFloat _gps_pos_z;		///< Z position of GPS antenna in body frame (m)
	BlockParamExtFloat _rng_pos_x;		///< X position of range finder in body frame (m)
	BlockParamExtFloat _rng_pos_y;		///< Y position of range finder in body frame (m)
	BlockParamExtFloat _rng_pos_z;		///< Z position of range finder in body frame (m)
	BlockParamExtFloat _flow_pos_x;	///< X position of optical flow sensor focal point in body frame (m)
	BlockParamExtFloat _flow_pos_y;	///< Y position of optical flow sensor focal point in body frame (m)
	BlockParamExtFloat _flow_pos_z;	///< Z position of optical flow sensor focal point in body frame (m)
	BlockParamExtFloat _ev_pos_x;		///< X position of VI sensor focal point in body frame (m)
	BlockParamExtFloat _ev_pos_y;		///< Y position of VI sensor focal point in body frame (m)
	BlockParamExtFloat _ev_pos_z;		///< Z position of VI sensor focal point in body frame (m)

	// control of airspeed and sideslip fusion
	BlockParamFloat
	_arspFusionThreshold; 	///< A value of zero will disabled airspeed fusion. Any positive value sets the minimum airspeed which will be used (m/sec)
	BlockParamInt _fuseBeta;		///< Controls synthetic sideslip fusion, 0 disables, 1 enables

	// output predictor filter time constants
	BlockParamExtFloat _tau_vel;		///< time constant used by the output velocity complementary filter (sec)
	BlockParamExtFloat _tau_pos;		///< time constant used by the output position complementary filter (sec)

	// IMU switch on bias paameters
	BlockParamExtFloat _gyr_bias_init;	///< 1-sigma gyro bias uncertainty at switch on (rad/sec)
	BlockParamExtFloat _acc_bias_init;	///< 1-sigma accelerometer bias uncertainty at switch on (m/sec**2)
	BlockParamExtFloat _ang_err_init;	///< 1-sigma tilt error after initial alignment using gravity vector (rad)

	// EKF saved XYZ magnetometer bias values
	BlockParamFloat _mag_bias_x;		///< X magnetometer bias (mGauss)
	BlockParamFloat _mag_bias_y;		///< Y magnetometer bias (mGauss)
	BlockParamFloat _mag_bias_z;		///< Z magnetometer bias (mGauss)
	BlockParamInt _mag_bias_id;		///< ID of the magnetometer sensor used to learn the bias values
	BlockParamFloat
	_mag_bias_saved_variance; ///< Assumed error variance of previously saved magnetometer bias estimates (mGauss**2)
	BlockParamFloat _mag_bias_alpha;	///< maximum fraction of the learned magnetometer bias that is saved at each disarm

	// EKF accel bias learning control
	BlockParamExtFloat _acc_bias_lim;	///< Accelerometer bias learning limit (m/s**2)
	BlockParamExtFloat _acc_bias_learn_acc_lim;	///< Maximum IMU accel magnitude that allows IMU bias learning (m/s**2)
	BlockParamExtFloat
	_acc_bias_learn_gyr_lim;	///< Maximum IMU gyro angular rate magnitude that allows IMU bias learning (m/s**2)
	BlockParamExtFloat _acc_bias_learn_tc;	///< Time constant used to inhibit IMU delta velocity bias learning (sec)

	// Multi-rotor drag specific force fusion
	BlockParamExtFloat _drag_noise;	///< observation noise variance for drag specific force measurements (m/sec**2)**2
	BlockParamExtFloat _bcoef_x;		///< ballistic coefficient along the X-axis (kg/m**2)
	BlockParamExtFloat _bcoef_y;		///< ballistic coefficient along the Y-axis (kg/m**2)

	// Corrections for static pressure position error where Ps_error = Ps_meas - Ps_truth
	// Coef = Ps_error / Pdynamic, where Pdynamic = 1/2 * density * TAS**2
	BlockParamFloat _aspd_max;		///< upper limit on airspeed used for correction  (m/s**2)
	BlockParamFloat _K_pstatic_coef_xp;	///< static pressure position error coefficient along the positive X body axis
	BlockParamFloat _K_pstatic_coef_xn;	///< static pressure position error coefficient along the negative X body axis
	BlockParamFloat _K_pstatic_coef_y;	///< static pressure position error coefficient along the Y body axis
	BlockParamFloat _K_pstatic_coef_z;	///< static pressure position error coefficient along the Z body axis

	BlockParamInt _airspeed_disabled;	///< airspeed mode parameter

};

Ekf2::Ekf2():
	SuperBlock(nullptr, "EKF2"),
	_vehicle_local_position_pub(ORB_ID(vehicle_local_position), -1, &getPublications()),
	_vehicle_global_position_pub(ORB_ID(vehicle_global_position), -1, &getPublications()),
	_params(_ekf.getParamHandle()),
	_obs_dt_min_ms(this, "MIN_OBS_DT", true, _params->sensor_interval_min_ms),
	_mag_delay_ms(this, "MAG_DELAY", true, _params->mag_delay_ms),
	_baro_delay_ms(this, "BARO_DELAY", true, _params->baro_delay_ms),
	_gps_delay_ms(this, "GPS_DELAY", true, _params->gps_delay_ms),
	_flow_delay_ms(this, "OF_DELAY", true, _params->flow_delay_ms),
	_rng_delay_ms(this, "RNG_DELAY", true, _params->range_delay_ms),
	_airspeed_delay_ms(this, "ASP_DELAY", true, _params->airspeed_delay_ms),
	_ev_delay_ms(this, "EV_DELAY", true, _params->ev_delay_ms),
	_auxvel_delay_ms(this, "AVEL_DELAY", true, _params->auxvel_delay_ms),
	_gyro_noise(this, "GYR_NOISE", true, _params->gyro_noise),
	_accel_noise(this, "ACC_NOISE", true, _params->accel_noise),
	_gyro_bias_p_noise(this, "GYR_B_NOISE", true, _params->gyro_bias_p_noise),
	_accel_bias_p_noise(this, "ACC_B_NOISE", true, _params->accel_bias_p_noise),
	_mage_p_noise(this, "MAG_E_NOISE", true, _params->mage_p_noise),
	_magb_p_noise(this, "MAG_B_NOISE", true, _params->magb_p_noise),
	_wind_vel_p_noise(this, "WIND_NOISE", true, _params->wind_vel_p_noise),
	_terrain_p_noise(this, "TERR_NOISE", true, _params->terrain_p_noise),
	_terrain_gradient(this, "TERR_GRAD", true, _params->terrain_gradient),
	_gps_vel_noise(this, "GPS_V_NOISE", true, _params->gps_vel_noise),
	_gps_pos_noise(this, "GPS_P_NOISE", true, _params->gps_pos_noise),
	_pos_noaid_noise(this, "NOAID_NOISE", true, _params->pos_noaid_noise),
	_baro_noise(this, "BARO_NOISE", true, _params->baro_noise),
	_baro_innov_gate(this, "BARO_GATE", true, _params->baro_innov_gate),
	_posNE_innov_gate(this, "GPS_P_GATE", true, _params->posNE_innov_gate),
	_vel_innov_gate(this, "GPS_V_GATE", true, _params->vel_innov_gate),
	_tas_innov_gate(this, "TAS_GATE", true, _params->tas_innov_gate),
	_mag_heading_noise(this, "HEAD_NOISE", true, _params->mag_heading_noise),
	_mag_noise(this, "MAG_NOISE", true, _params->mag_noise),
	_eas_noise(this, "EAS_NOISE", true, _params->eas_noise),
	_beta_innov_gate(this, "BETA_GATE", true, _params->beta_innov_gate),
	_beta_noise(this, "BETA_NOISE", true, _params->beta_noise),
	_mag_declination_deg(this, "MAG_DECL", true, _params->mag_declination_deg),
	_heading_innov_gate(this, "HDG_GATE", true, _params->heading_innov_gate),
	_mag_innov_gate(this, "MAG_GATE", true, _params->mag_innov_gate),
	_mag_decl_source(this, "DECL_TYPE", true, _params->mag_declination_source),
	_mag_fuse_type(this, "MAG_TYPE", true, _params->mag_fusion_type),
	_mag_acc_gate(this, "MAG_ACCLIM", true, _params->mag_acc_gate),
	_mag_yaw_rate_gate(this, "MAG_YAWLIM", true, _params->mag_yaw_rate_gate),
	_gps_check_mask(this, "GPS_CHECK", true, _params->gps_check_mask),
	_requiredEph(this, "REQ_EPH", true, _params->req_hacc),
	_requiredEpv(this, "REQ_EPV", true, _params->req_vacc),
	_requiredSacc(this, "REQ_SACC", true, _params->req_sacc),
	_requiredNsats(this, "REQ_NSATS", true, _params->req_nsats),
	_requiredGDoP(this, "REQ_GDOP", true, _params->req_gdop),
	_requiredHdrift(this, "REQ_HDRIFT", true, _params->req_hdrift),
	_requiredVdrift(this, "REQ_VDRIFT", true, _params->req_vdrift),
	_fusion_mode(this, "AID_MASK", true, _params->fusion_mode),
	_vdist_sensor_type(this, "HGT_MODE", true, _params->vdist_sensor_type),
	_range_noise(this, "RNG_NOISE", true, _params->range_noise),
	_range_noise_scaler(this, "RNG_SFE", true, _params->range_noise_scaler),
	_range_innov_gate(this, "RNG_GATE", true, _params->range_innov_gate),
	_rng_gnd_clearance(this, "MIN_RNG", true, _params->rng_gnd_clearance),
	_rng_pitch_offset(this, "RNG_PITCH", true, _params->rng_sens_pitch),
	_rng_aid(this, "RNG_AID", true, _params->range_aid),
	_rng_aid_hor_vel_max(this, "RNG_A_VMAX", true, _params->max_vel_for_range_aid),
	_rng_aid_height_max(this, "RNG_A_HMAX", true, _params->max_hagl_for_range_aid),
	_rng_aid_innov_gate(this, "RNG_A_IGATE", true, _params->range_aid_innov_gate),
	_ev_pos_noise(this, "EVP_NOISE"),
	_ev_ang_noise(this, "EVA_NOISE"),
	_ev_innov_gate(this, "EV_GATE", true, _params->ev_innov_gate),
	_flow_noise(this, "OF_N_MIN", true, _params->flow_noise),
	_flow_noise_qual_min(this, "OF_N_MAX", true, _params->flow_noise_qual_min),
	_flow_qual_min(this, "OF_QMIN", true, _params->flow_qual_min),
	_flow_innov_gate(this, "OF_GATE", true, _params->flow_innov_gate),
	_flow_rate_max(this, "OF_RMAX", true, _params->flow_rate_max),
	_imu_pos_x(this, "IMU_POS_X", true, _params->imu_pos_body(0)),
	_imu_pos_y(this, "IMU_POS_Y", true, _params->imu_pos_body(1)),
	_imu_pos_z(this, "IMU_POS_Z", true, _params->imu_pos_body(2)),
	_gps_pos_x(this, "GPS_POS_X", true, _params->gps_pos_body(0)),
	_gps_pos_y(this, "GPS_POS_Y", true, _params->gps_pos_body(1)),
	_gps_pos_z(this, "GPS_POS_Z", true, _params->gps_pos_body(2)),
	_rng_pos_x(this, "RNG_POS_X", true, _params->rng_pos_body(0)),
	_rng_pos_y(this, "RNG_POS_Y", true, _params->rng_pos_body(1)),
	_rng_pos_z(this, "RNG_POS_Z", true, _params->rng_pos_body(2)),
	_flow_pos_x(this, "OF_POS_X", true, _params->flow_pos_body(0)),
	_flow_pos_y(this, "OF_POS_Y", true, _params->flow_pos_body(1)),
	_flow_pos_z(this, "OF_POS_Z", true, _params->flow_pos_body(2)),
	_ev_pos_x(this, "EV_POS_X", true, _params->ev_pos_body(0)),
	_ev_pos_y(this, "EV_POS_Y", true, _params->ev_pos_body(1)),
	_ev_pos_z(this, "EV_POS_Z", true, _params->ev_pos_body(2)),
	_arspFusionThreshold(this, "ARSP_THR"),
	_fuseBeta(this, "FUSE_BETA"),
	_tau_vel(this, "TAU_VEL", true, _params->vel_Tau),
	_tau_pos(this, "TAU_POS", true, _params->pos_Tau),
	_gyr_bias_init(this, "GBIAS_INIT", true, _params->switch_on_gyro_bias),
	_acc_bias_init(this, "ABIAS_INIT", true, _params->switch_on_accel_bias),
	_ang_err_init(this, "ANGERR_INIT", true, _params->initial_tilt_err),
	_mag_bias_x(this, "MAGBIAS_X"),
	_mag_bias_y(this, "MAGBIAS_Y"),
	_mag_bias_z(this, "MAGBIAS_Z"),
	_mag_bias_id(this, "MAGBIAS_ID"),
	_mag_bias_saved_variance(this, "MAGB_VREF"),
	_mag_bias_alpha(this, "MAGB_K"),
	_acc_bias_lim(this, "ABL_LIM", true, _params->acc_bias_lim),
	_acc_bias_learn_acc_lim(this, "ABL_ACCLIM", true, _params->acc_bias_learn_acc_lim),
	_acc_bias_learn_gyr_lim(this, "ABL_GYRLIM", true, _params->acc_bias_learn_gyr_lim),
	_acc_bias_learn_tc(this, "ABL_TAU", true, _params->acc_bias_learn_tc),
	_drag_noise(this, "DRAG_NOISE", true, _params->drag_noise),
	_bcoef_x(this, "BCOEF_X", true, _params->bcoef_x),
	_bcoef_y(this, "BCOEF_Y", true, _params->bcoef_y),
	_aspd_max(this, "ASPD_MAX"),
	_K_pstatic_coef_xp(this, "PCOEF_XP"),
	_K_pstatic_coef_xn(this, "PCOEF_XN"),
	_K_pstatic_coef_y(this, "PCOEF_Y"),
	_K_pstatic_coef_z(this, "PCOEF_Z"),
	// non EKF2 parameters
	_airspeed_disabled(this, "FW_ARSP_MODE", false)
{
}

int Ekf2::print_status()
{
	PX4_INFO("local position OK %s", (_ekf.local_position_is_valid()) ? "yes" : "no");
	PX4_INFO("global position OK %s", (_ekf.global_position_is_valid()) ? "yes" : "no");
	PX4_INFO("time slip: %" PRId64 " us", _last_time_slip_us);
	return 0;
}

void Ekf2::run()
{
	// subscribe to relevant topics
	int sensors_sub = orb_subscribe(ORB_ID(sensor_combined));
	int gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	int airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	int params_sub = orb_subscribe(ORB_ID(parameter_update));
	int optical_flow_sub = orb_subscribe(ORB_ID(optical_flow));
	int ev_pos_sub = orb_subscribe(ORB_ID(vehicle_vision_position));
	int ev_att_sub = orb_subscribe(ORB_ID(vehicle_vision_attitude));
	int vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	int status_sub = orb_subscribe(ORB_ID(vehicle_status));
	int sensor_selection_sub = orb_subscribe(ORB_ID(sensor_selection));
	int sensor_baro_sub = orb_subscribe(ORB_ID(sensor_baro));
	int landing_target_pose_sub = orb_subscribe(ORB_ID(landing_target_pose));

	bool imu_bias_reset_request = false;

	// because we can have several distance sensor instances with different orientations
	int range_finder_subs[ORB_MULTI_MAX_INSTANCES];
	int range_finder_sub_index = -1; // index for downward-facing range finder subscription

	for (unsigned i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		range_finder_subs[i] = orb_subscribe_multi(ORB_ID(distance_sensor), i);
	}

	px4_pollfd_struct_t fds[1] = {};
	fds[0].fd = sensors_sub;
	fds[0].events = POLLIN;

	// initialise parameter cache
	updateParams();

	// initialize data structures outside of loop
	// because they will else not always be
	// properly populated
	sensor_combined_s sensors = {};
	vehicle_gps_position_s gps = {};
	airspeed_s airspeed = {};
	optical_flow_s optical_flow = {};
	distance_sensor_s range_finder = {};
	vehicle_land_detected_s vehicle_land_detected = {};
	vehicle_local_position_s ev_pos = {};
	vehicle_attitude_s ev_att = {};
	vehicle_status_s vehicle_status = {};
	sensor_selection_s sensor_selection = {};
	sensor_baro_s sensor_baro = {};
	sensor_baro.pressure = 1013.5f; // initialise pressure to sea level
	landing_target_pose_s landing_target_pose = {};

	while (!should_exit()) {
		int ret = px4_poll(fds, sizeof(fds) / sizeof(fds[0]), 1000);

		if (!(fds[0].revents & POLLIN)) {
			// no new data
			continue;
		}

		if (ret < 0) {
			// Poll error, sleep and try again
			usleep(10000);
			continue;

		} else if (ret == 0) {
			// Poll timeout or no new data, do nothing
			continue;
		}

		bool params_updated = false;
		orb_check(params_sub, &params_updated);

		if (params_updated) {
			// read from param to clear updated flag
			parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), params_sub, &update);
			updateParams();
		}

		bool gps_updated = false;
		bool airspeed_updated = false;
		bool baro_updated = false;
		bool sensor_selection_updated = false;
		bool optical_flow_updated = false;
		bool range_finder_updated = false;
		bool vehicle_land_detected_updated = false;
		bool vision_position_updated = false;
		bool vision_attitude_updated = false;
		bool vehicle_status_updated = false;
		bool landing_target_pose_updated = false;

		orb_copy(ORB_ID(sensor_combined), sensors_sub, &sensors);
		// update all other topics if they have new data

		orb_check(status_sub, &vehicle_status_updated);

		if (vehicle_status_updated) {
			orb_copy(ORB_ID(vehicle_status), status_sub, &vehicle_status);
		}

		orb_check(gps_sub, &gps_updated);

		if (gps_updated) {
			orb_copy(ORB_ID(vehicle_gps_position), gps_sub, &gps);
		}

		// Do not attempt to use airspeed if use has been disabled by the user.
		if (_airspeed_disabled.get() == 0) {
			orb_check(airspeed_sub, &airspeed_updated);
		}

		if (airspeed_updated) {
			orb_copy(ORB_ID(airspeed), airspeed_sub, &airspeed);
		}

		orb_check(sensor_baro_sub, &baro_updated);

		if (baro_updated) {
			orb_copy(ORB_ID(sensor_baro), sensor_baro_sub, &sensor_baro);
		}

		orb_check(sensor_selection_sub, &sensor_selection_updated);

		// Always update sensor selction first time through if time stamp is non zero
		if (sensor_selection_updated || (sensor_selection.timestamp == 0)) {
			sensor_selection_s sensor_selection_prev = sensor_selection;
			orb_copy(ORB_ID(sensor_selection), sensor_selection_sub, &sensor_selection);

			if ((sensor_selection_prev.timestamp > 0) && (sensor_selection.timestamp > sensor_selection_prev.timestamp)) {
				if (sensor_selection.accel_device_id != sensor_selection_prev.accel_device_id) {
					PX4_WARN("accel id changed, resetting IMU bias");
					imu_bias_reset_request = true;
				}

				if (sensor_selection.gyro_device_id != sensor_selection_prev.gyro_device_id) {
					PX4_WARN("gyro id changed, resetting IMU bias");
					imu_bias_reset_request = true;
				}
			}
		}

		// attempt reset until successful
		if (imu_bias_reset_request) {
			imu_bias_reset_request = !_ekf.reset_imu_bias();
		}

		orb_check(optical_flow_sub, &optical_flow_updated);

		if (optical_flow_updated) {
			orb_copy(ORB_ID(optical_flow), optical_flow_sub, &optical_flow);
		}

		if (range_finder_sub_index >= 0) {
			orb_check(range_finder_subs[range_finder_sub_index], &range_finder_updated);

			if (range_finder_updated) {
				orb_copy(ORB_ID(distance_sensor), range_finder_subs[range_finder_sub_index], &range_finder);

				// check if distance sensor is within working boundaries
				if (range_finder.min_distance >= range_finder.current_distance ||
				    range_finder.max_distance <= range_finder.current_distance) {
					// use rng_gnd_clearance if on ground
					if (_ekf.get_in_air_status()) {
						range_finder_updated = false;

					} else {
						range_finder.current_distance = _rng_gnd_clearance.get();
					}
				}
			}

		} else {
			range_finder_sub_index = getRangeSubIndex(range_finder_subs);
		}

		orb_check(ev_pos_sub, &vision_position_updated);

		if (vision_position_updated) {
			orb_copy(ORB_ID(vehicle_vision_position), ev_pos_sub, &ev_pos);
		}

		orb_check(ev_att_sub, &vision_attitude_updated);

		if (vision_attitude_updated) {
			orb_copy(ORB_ID(vehicle_vision_attitude), ev_att_sub, &ev_att);
		}

		// in replay mode we are getting the actual timestamp from the sensor topic
		hrt_abstime now = 0;

		if (_replay_mode) {
			now = sensors.timestamp;

		} else {
			now = hrt_absolute_time();
		}

		// push imu data into estimator
		float gyro_integral[3];
		float gyro_dt = sensors.gyro_integral_dt / 1.e6f;
		gyro_integral[0] = sensors.gyro_rad[0] * gyro_dt;
		gyro_integral[1] = sensors.gyro_rad[1] * gyro_dt;
		gyro_integral[2] = sensors.gyro_rad[2] * gyro_dt;

		float accel_integral[3];
		float accel_dt = sensors.accelerometer_integral_dt / 1.e6f;
		accel_integral[0] = sensors.accelerometer_m_s2[0] * accel_dt;
		accel_integral[1] = sensors.accelerometer_m_s2[1] * accel_dt;
		accel_integral[2] = sensors.accelerometer_m_s2[2] * accel_dt;

		_ekf.setIMUData(now, sensors.gyro_integral_dt, sensors.accelerometer_integral_dt, gyro_integral, accel_integral);

		// read mag data
		if (sensors.magnetometer_timestamp_relative == sensor_combined_s::RELATIVE_TIMESTAMP_INVALID) {
			// set a zero timestamp to let the ekf replay program know that this data is not valid
			_timestamp_mag_us = 0;

		} else {
			if ((sensors.timestamp + sensors.magnetometer_timestamp_relative) != _timestamp_mag_us) {
				_timestamp_mag_us = sensors.timestamp + sensors.magnetometer_timestamp_relative;

				// Reset learned bias parameters if there has been a persistant change in magnetometer ID
				// Do not reset parmameters when armed to prevent potential time slips casued by parameter set
				// and notification events
				// Check if there has been a persistant change in magnetometer ID
				if (sensor_selection.mag_device_id != 0 && sensor_selection.mag_device_id != _mag_bias_id.get()) {
					if (_invalid_mag_id_count < 200) {
						_invalid_mag_id_count++;
					}

				} else {
					if (_invalid_mag_id_count > 0) {
						_invalid_mag_id_count--;
					}
				}

				if ((vehicle_status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) && (_invalid_mag_id_count > 100)) {
					// the sensor ID used for the last saved mag bias is not confirmed to be the same as the current sensor ID
					// this means we need to reset the learned bias values to zero
					_mag_bias_x.set(0.f);
					_mag_bias_x.commit_no_notification();
					_mag_bias_y.set(0.f);
					_mag_bias_y.commit_no_notification();
					_mag_bias_z.set(0.f);
					_mag_bias_z.commit_no_notification();
					_mag_bias_id.set(sensor_selection.mag_device_id);
					_mag_bias_id.commit();

					_invalid_mag_id_count = 0;

					PX4_INFO("Mag sensor ID changed to %i", _mag_bias_id.get());
				}

				// If the time last used by the EKF is less than specified, then accumulate the
				// data and push the average when the specified interval is reached.
				_mag_time_sum_ms += _timestamp_mag_us / 1000;
				_mag_sample_count++;
				_mag_data_sum[0] += sensors.magnetometer_ga[0];
				_mag_data_sum[1] += sensors.magnetometer_ga[1];
				_mag_data_sum[2] += sensors.magnetometer_ga[2];
				uint32_t mag_time_ms = _mag_time_sum_ms / _mag_sample_count;

				if (mag_time_ms - _mag_time_ms_last_used > _params->sensor_interval_min_ms) {
					const float mag_sample_count_inv = 1.0f / _mag_sample_count;
					// calculate mean of measurements and correct for learned bias offsets
					float mag_data_avg_ga[3] = {_mag_data_sum[0] *mag_sample_count_inv - _mag_bias_x.get(),
								    _mag_data_sum[1] *mag_sample_count_inv - _mag_bias_y.get(),
								    _mag_data_sum[2] *mag_sample_count_inv - _mag_bias_z.get()
								   };

					_ekf.setMagData(1000 * (uint64_t)mag_time_ms, mag_data_avg_ga);

					_mag_time_ms_last_used = mag_time_ms;
					_mag_time_sum_ms = 0;
					_mag_sample_count = 0;
					_mag_data_sum[0] = 0.0f;
					_mag_data_sum[1] = 0.0f;
					_mag_data_sum[2] = 0.0f;
				}
			}
		}

		// read baro data
		if (sensors.baro_timestamp_relative == sensor_combined_s::RELATIVE_TIMESTAMP_INVALID) {
			// set a zero timestamp to let the ekf replay program know that this data is not valid
			_timestamp_balt_us = 0;

		} else {
			if ((sensors.timestamp + sensors.baro_timestamp_relative) != _timestamp_balt_us) {
				_timestamp_balt_us = sensors.timestamp + sensors.baro_timestamp_relative;

				// If the time last used by the EKF is less than specified, then accumulate the
				// data and push the average when the specified interval is reached.
				_balt_time_sum_ms += _timestamp_balt_us / 1000;
				_balt_sample_count++;
				_balt_data_sum += sensors.baro_alt_meter;
				uint32_t balt_time_ms = _balt_time_sum_ms / _balt_sample_count;

				if (balt_time_ms - _balt_time_ms_last_used > (uint32_t)_params->sensor_interval_min_ms) {
					// take mean across sample period
					float balt_data_avg = _balt_data_sum / (float)_balt_sample_count;

					// estimate air density assuming typical 20degC ambient temperature
					const float pressure_to_density = 100.0f / (CONSTANTS_AIR_GAS_CONST * (20.0f - CONSTANTS_ABSOLUTE_NULL_CELSIUS));
					const float rho = pressure_to_density * sensor_baro.pressure;
					_ekf.set_air_density(rho);

					// calculate static pressure error = Pmeas - Ptruth
					// model position error sensitivity as a body fixed ellipse with different scale in the positive and negtive X direction
					float max_airspeed_sq = _aspd_max.get();
					max_airspeed_sq *= max_airspeed_sq;
					float K_pstatic_coef_x;

					if (_vel_body_wind(0) >= 0.0f) {
						K_pstatic_coef_x = _K_pstatic_coef_xp.get();

					} else {
						K_pstatic_coef_x = _K_pstatic_coef_xn.get();
					}

					float pstatic_err = 0.5f * rho * (K_pstatic_coef_x * fminf(_vel_body_wind(0) * _vel_body_wind(0), max_airspeed_sq) +
									  _K_pstatic_coef_y.get() * fminf(_vel_body_wind(1) * _vel_body_wind(1), max_airspeed_sq) +
									  _K_pstatic_coef_z.get() * fminf(_vel_body_wind(2) * _vel_body_wind(2), max_airspeed_sq));

					// correct baro measurement using pressure error estimate and assuming sea level gravity
					balt_data_avg += pstatic_err / (rho * CONSTANTS_ONE_G);

					// push to estimator
					_ekf.setBaroData(1000 * (uint64_t)balt_time_ms, balt_data_avg);

					_balt_time_ms_last_used = balt_time_ms;
					_balt_time_sum_ms = 0;
					_balt_sample_count = 0;
					_balt_data_sum = 0.0f;
				}
			}
		}

		// read gps data if available
		if (gps_updated) {
			struct gps_message gps_msg;
			gps_msg.time_usec = gps.timestamp;
			gps_msg.lat = gps.lat;
			gps_msg.lon = gps.lon;
			gps_msg.alt = gps.alt;
			gps_msg.fix_type = gps.fix_type;
			gps_msg.eph = gps.eph;
			gps_msg.epv = gps.epv;
			gps_msg.sacc = gps.s_variance_m_s;
			gps_msg.vel_m_s = gps.vel_m_s;
			gps_msg.vel_ned[0] = gps.vel_n_m_s;
			gps_msg.vel_ned[1] = gps.vel_e_m_s;
			gps_msg.vel_ned[2] = gps.vel_d_m_s;
			gps_msg.vel_ned_valid = gps.vel_ned_valid;
			gps_msg.nsats = gps.satellites_used;
			//TODO: add gdop to gps topic
			gps_msg.gdop = 0.0f;

			_ekf.setGpsData(gps.timestamp, &gps_msg);
		}

		// only set airspeed data if condition for airspeed fusion are met
		bool fuse_airspeed = airspeed_updated && !vehicle_status.is_rotary_wing
				     && (_arspFusionThreshold.get() > FLT_EPSILON)
				     && (airspeed.true_airspeed_m_s > _arspFusionThreshold.get());

		if (fuse_airspeed) {
			const float eas2tas = airspeed.true_airspeed_m_s / airspeed.indicated_airspeed_m_s;
			_ekf.setAirspeedData(airspeed.timestamp, airspeed.true_airspeed_m_s, eas2tas);
		}

		if (vehicle_status_updated) {
			// only fuse synthetic sideslip measurements if conditions are met
			bool fuse_beta = !vehicle_status.is_rotary_wing && (_fuseBeta.get() == 1);
			_ekf.set_fuse_beta_flag(fuse_beta);

			// let the EKF know if the vehicle motion is that of a fixed wing (forward flight only relative to wind)
			_ekf.set_is_fixed_wing(!vehicle_status.is_rotary_wing);
		}

		if (optical_flow_updated) {
			flow_message flow;
			flow.flowdata(0) = optical_flow.pixel_flow_x_integral;
			flow.flowdata(1) = optical_flow.pixel_flow_y_integral;
			flow.quality = optical_flow.quality;
			flow.gyrodata(0) = optical_flow.gyro_x_rate_integral;
			flow.gyrodata(1) = optical_flow.gyro_y_rate_integral;
			flow.gyrodata(2) = optical_flow.gyro_z_rate_integral;
			flow.dt = optical_flow.integration_timespan;

			if (PX4_ISFINITE(optical_flow.pixel_flow_y_integral) &&
			    PX4_ISFINITE(optical_flow.pixel_flow_x_integral)) {
				_ekf.setOpticalFlowData(optical_flow.timestamp, &flow);
			}
		}

		if (range_finder_updated) {
			_ekf.setRangeData(range_finder.timestamp, range_finder.current_distance);
		}

		// get external vision data
		// if error estimates are unavailable, use parameter defined defaults
		if (vision_position_updated || vision_attitude_updated) {
			ext_vision_message ev_data;
			ev_data.posNED(0) = ev_pos.x;
			ev_data.posNED(1) = ev_pos.y;
			ev_data.posNED(2) = ev_pos.z;
			matrix::Quatf q(ev_att.q);
			ev_data.quat = q;

			// position measurement error from parameters. TODO : use covariances from topic
			ev_data.posErr = fmaxf(_ev_pos_noise.get(), fmaxf(ev_pos.eph, ev_pos.epv));
			ev_data.angErr = _ev_ang_noise.get();

			// only set data if all positions and velocities are valid
			if (ev_pos.xy_valid && ev_pos.z_valid && ev_pos.v_xy_valid && ev_pos.v_z_valid) {
				// use timestamp from external computer, clocks are synchronized when using MAVROS
				_ekf.setExtVisionData(vision_position_updated ? ev_pos.timestamp : ev_att.timestamp, &ev_data);
			}
		}

		orb_check(vehicle_land_detected_sub, &vehicle_land_detected_updated);

		if (vehicle_land_detected_updated) {
			orb_copy(ORB_ID(vehicle_land_detected), vehicle_land_detected_sub, &vehicle_land_detected);
			_ekf.set_in_air_status(!vehicle_land_detected.landed);
		}

		// use the landing target pose estimate as another source of velocity data
		orb_check(landing_target_pose_sub, &landing_target_pose_updated);

		if (landing_target_pose_updated) {
			orb_copy(ORB_ID(landing_target_pose), landing_target_pose_sub, &landing_target_pose);

			// we can only use the landing target if it has a fixed position and  a valid velocity estimate
			if (landing_target_pose.is_static && landing_target_pose.rel_vel_valid) {
				// velocity of vehicle relative to target has opposite sign to target relative to vehicle
				float velocity[2];
				velocity[0] = -landing_target_pose.vx_rel;
				velocity[1] = -landing_target_pose.vy_rel;
				float variance[2];
				variance[0] = landing_target_pose.cov_vx_rel;
				variance[1] = landing_target_pose.cov_vy_rel;
				_ekf.setAuxVelData(landing_target_pose.timestamp, velocity, variance);
			}
		}

		// run the EKF update and output
		const bool updated = _ekf.update();

		if (updated) {

			// integrate time to monitor time slippage
			if (_start_time_us == 0) {
				_start_time_us = now;
				_last_time_slip_us = 0;

			} else if (_start_time_us > 0) {
				_integrated_time_us += sensors.gyro_integral_dt;
				_last_time_slip_us = (now - _start_time_us) - _integrated_time_us;
			}

			matrix::Quatf q;
			_ekf.copy_quaternion(q.data());

			// In-run bias estimates
			float gyro_bias[3];
			_ekf.get_gyro_bias(gyro_bias);

			{
				// generate vehicle attitude quaternion data
				vehicle_attitude_s att;
				att.timestamp = now;

				q.copyTo(att.q);
				_ekf.get_quat_reset(&att.delta_q_reset[0], &att.quat_reset_counter);

				att.rollspeed = sensors.gyro_rad[0] - gyro_bias[0];
				att.pitchspeed = sensors.gyro_rad[1] - gyro_bias[1];
				att.yawspeed = sensors.gyro_rad[2] - gyro_bias[2];

				// publish vehicle attitude data
				if (_att_pub == nullptr) {
					_att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

				} else {
					orb_publish(ORB_ID(vehicle_attitude), _att_pub, &att);
				}
			}

			// generate vehicle local position data
			vehicle_local_position_s &lpos = _vehicle_local_position_pub.get();

			lpos.timestamp = now;

			// Position of body origin in local NED frame
			float position[3];
			_ekf.get_position(position);
			const float lpos_x_prev = lpos.x;
			const float lpos_y_prev = lpos.y;
			lpos.x = (_ekf.local_position_is_valid()) ? position[0] : 0.0f;
			lpos.y = (_ekf.local_position_is_valid()) ? position[1] : 0.0f;
			lpos.z = position[2];

			// Velocity of body origin in local NED frame (m/s)
			float velocity[3];
			_ekf.get_velocity(velocity);
			lpos.vx = velocity[0];
			lpos.vy = velocity[1];
			lpos.vz = velocity[2];

			// vertical position time derivative (m/s)
			_ekf.get_pos_d_deriv(&lpos.z_deriv);

			// Acceleration of body origin in local NED frame
			float vel_deriv[3];
			_ekf.get_vel_deriv_ned(vel_deriv);
			lpos.ax = vel_deriv[0];
			lpos.ay = vel_deriv[1];
			lpos.az = vel_deriv[2];

			// TODO: better status reporting
			lpos.xy_valid = _ekf.local_position_is_valid() && !_preflt_horiz_fail;
			lpos.z_valid = !_preflt_vert_fail;
			lpos.v_xy_valid = _ekf.local_position_is_valid() && !_preflt_horiz_fail;
			lpos.v_z_valid = !_preflt_vert_fail;

			// Position of local NED origin in GPS / WGS84 frame
			map_projection_reference_s ekf_origin;
			uint64_t origin_time;

			// true if position (x,y,z) has a valid WGS-84 global reference (ref_lat, ref_lon, alt)
			const bool ekf_origin_valid = _ekf.get_ekf_origin(&origin_time, &ekf_origin, &lpos.ref_alt);
			lpos.xy_global = ekf_origin_valid;
			lpos.z_global = ekf_origin_valid;

			if (ekf_origin_valid && (origin_time > lpos.ref_timestamp)) {
				lpos.ref_timestamp = origin_time;
				lpos.ref_lat = ekf_origin.lat_rad * 180.0 / M_PI; // Reference point latitude in degrees
				lpos.ref_lon = ekf_origin.lon_rad * 180.0 / M_PI; // Reference point longitude in degrees
			}

			// The rotation of the tangent plane vs. geographical north
			matrix::Eulerf euler(q);
			lpos.yaw = euler.psi();

			lpos.dist_bottom_valid = _ekf.get_terrain_valid();

			float terrain_vpos;
			_ekf.get_terrain_vert_pos(&terrain_vpos);
			lpos.dist_bottom = terrain_vpos - lpos.z; // Distance to bottom surface (ground) in meters

			// constrain the distance to ground to _rng_gnd_clearance
			if (lpos.dist_bottom < _rng_gnd_clearance.get()) {
				lpos.dist_bottom = _rng_gnd_clearance.get();
			}

			lpos.dist_bottom_rate = -lpos.vz; // Distance to bottom surface (ground) change rate

			bool dead_reckoning;
			_ekf.get_ekf_lpos_accuracy(&lpos.eph, &lpos.epv, &dead_reckoning);
			_ekf.get_ekf_vel_accuracy(&lpos.evh, &lpos.evv, &dead_reckoning);

			// get state reset information of position and velocity
			_ekf.get_posD_reset(&lpos.delta_z, &lpos.z_reset_counter);
			_ekf.get_velD_reset(&lpos.delta_vz, &lpos.vz_reset_counter);
			_ekf.get_posNE_reset(&lpos.delta_xy[0], &lpos.xy_reset_counter);
			_ekf.get_velNE_reset(&lpos.delta_vxy[0], &lpos.vxy_reset_counter);

			// publish vehicle local position data
			_vehicle_local_position_pub.update();

			if (_ekf.global_position_is_valid() && !_preflt_fail) {
				// generate and publish global position data
				vehicle_global_position_s &global_pos = _vehicle_global_position_pub.get();

				global_pos.timestamp = now;

				if (fabsf(lpos_x_prev - lpos.x) > FLT_EPSILON || fabsf(lpos_y_prev - lpos.y) > FLT_EPSILON) {
					map_projection_reproject(&ekf_origin, lpos.x, lpos.y, &global_pos.lat, &global_pos.lon);
				}

				global_pos.lat_lon_reset_counter = lpos.xy_reset_counter;

				global_pos.alt = -lpos.z + lpos.ref_alt; // Altitude AMSL in meters

				// global altitude has opposite sign of local down position
				global_pos.delta_alt = -lpos.delta_z;

				global_pos.vel_n = lpos.vx; // Ground north velocity, m/s
				global_pos.vel_e = lpos.vy; // Ground east velocity, m/s
				global_pos.vel_d = lpos.vz; // Ground downside velocity, m/s

				global_pos.pos_d_deriv = lpos.z_deriv; // vertical position time derivative, m/s

				global_pos.yaw = lpos.yaw; // Yaw in radians -PI..+PI.

				_ekf.get_ekf_gpos_accuracy(&global_pos.eph, &global_pos.epv, &global_pos.dead_reckoning);
				global_pos.evh = lpos.evh;
				global_pos.evv = lpos.evv;

				global_pos.terrain_alt_valid = lpos.dist_bottom_valid;

				if (global_pos.terrain_alt_valid) {
					global_pos.terrain_alt = lpos.ref_alt - terrain_vpos; // Terrain altitude in m, WGS84

				} else {
					global_pos.terrain_alt = 0.0f; // Terrain altitude in m, WGS84
				}

				global_pos.pressure_alt = sensors.baro_alt_meter; // Pressure altitude AMSL (m)

				global_pos.dead_reckoning = _ekf.inertial_dead_reckoning(); // True if this position is estimated through dead-reckoning

				_vehicle_global_position_pub.update();
			}

			{
				// publish all corrected sensor readings and bias estimates after mag calibration is updated above
				float accel_bias[3];
				_ekf.get_accel_bias(accel_bias);

				sensor_bias_s bias;

				bias.timestamp = now;

				bias.gyro_x = sensors.gyro_rad[0] - gyro_bias[0];
				bias.gyro_y = sensors.gyro_rad[1] - gyro_bias[1];
				bias.gyro_z = sensors.gyro_rad[2] - gyro_bias[2];

				bias.accel_x = sensors.accelerometer_m_s2[0] - accel_bias[0];
				bias.accel_y = sensors.accelerometer_m_s2[1] - accel_bias[1];
				bias.accel_z = sensors.accelerometer_m_s2[2] - accel_bias[2];

				bias.mag_x = sensors.magnetometer_ga[0] - (_last_valid_mag_cal[0] / 1000.0f); // mGauss -> Gauss
				bias.mag_y = sensors.magnetometer_ga[1] - (_last_valid_mag_cal[1] / 1000.0f); // mGauss -> Gauss
				bias.mag_z = sensors.magnetometer_ga[2] - (_last_valid_mag_cal[2] / 1000.0f); // mGauss -> Gauss

				bias.gyro_x_bias = gyro_bias[0];
				bias.gyro_y_bias = gyro_bias[1];
				bias.gyro_z_bias = gyro_bias[2];

				bias.accel_x_bias = accel_bias[0];
				bias.accel_y_bias = accel_bias[1];
				bias.accel_z_bias = accel_bias[2];

				bias.mag_x_bias = _last_valid_mag_cal[0];
				bias.mag_y_bias = _last_valid_mag_cal[1];
				bias.mag_z_bias = _last_valid_mag_cal[2];

				if (_sensor_bias_pub == nullptr) {
					_sensor_bias_pub = orb_advertise(ORB_ID(sensor_bias), &bias);

				} else {
					orb_publish(ORB_ID(sensor_bias), _sensor_bias_pub, &bias);
				}
			}
		}

		// publish estimator status
		estimator_status_s status;
		status.timestamp = now;
		_ekf.get_state_delayed(status.states);
		_ekf.get_covariances(status.covariances);
		_ekf.get_gps_check_status(&status.gps_check_fail_flags);
		_ekf.get_control_mode(&status.control_mode_flags);
		_ekf.get_filter_fault_status(&status.filter_fault_flags);
		_ekf.get_innovation_test_status(&status.innovation_check_flags, &status.mag_test_ratio,
						&status.vel_test_ratio, &status.pos_test_ratio,
						&status.hgt_test_ratio, &status.tas_test_ratio,
						&status.hagl_test_ratio, &status.beta_test_ratio);

		status.pos_horiz_accuracy = _vehicle_local_position_pub.get().eph;
		status.pos_vert_accuracy = _vehicle_local_position_pub.get().epv;
		_ekf.get_ekf_soln_status(&status.solution_status_flags);
		_ekf.get_imu_vibe_metrics(status.vibe);
		status.time_slip = _last_time_slip_us / 1e6f;
		status.nan_flags = 0.0f; // unused
		status.health_flags = 0.0f; // unused
		status.timeout_flags = 0.0f; // unused
		status.pre_flt_fail = _preflt_fail;

		if (_estimator_status_pub == nullptr) {
			_estimator_status_pub = orb_advertise(ORB_ID(estimator_status), &status);

		} else {
			orb_publish(ORB_ID(estimator_status), _estimator_status_pub, &status);
		}

		if (updated) {
			{
				/* Check and save learned magnetometer bias estimates */

				// Check if conditions are OK to for learning of magnetometer bias values
				if (!vehicle_land_detected.landed && // not on ground
				    (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) && // vehicle is armed
				    (status.filter_fault_flags == 0) && // there are no filter faults
				    (status.control_mode_flags & (1 << 5))) { // the EKF is operating in the correct mode

					if (_last_magcal_us == 0) {
						_last_magcal_us = now;

					} else {
						_total_cal_time_us += now - _last_magcal_us;
						_last_magcal_us = now;
					}

				} else if (status.filter_fault_flags != 0) {
					// if a filter fault has occurred, assume previous learning was invalid and do not
					// count it towards total learning time.
					_total_cal_time_us = 0;

					for (bool &cal_available : _valid_cal_available) {
						cal_available = false;
					}
				}

				// Start checking mag bias estimates when we have accumulated sufficient calibration time
				if (_total_cal_time_us > 120 * 1000 * 1000ULL) {
					// we have sufficient accumulated valid flight time to form a reliable bias estimate
					// check that the state variance for each axis is within a range indicating filter convergence
					const float max_var_allowed = 100.0f * _mag_bias_saved_variance.get();
					const float min_var_allowed = 0.01f * _mag_bias_saved_variance.get();

					// Declare all bias estimates invalid if any variances are out of range
					bool all_estimates_invalid = false;

					for (uint8_t axis_index = 0; axis_index <= 2; axis_index++) {
						if (status.covariances[axis_index + 19] < min_var_allowed
						    || status.covariances[axis_index + 19] > max_var_allowed) {
							all_estimates_invalid = true;
						}
					}

					// Store valid estimates and their associated variances
					if (!all_estimates_invalid) {
						for (uint8_t axis_index = 0; axis_index <= 2; axis_index++) {
							_last_valid_mag_cal[axis_index] = status.states[axis_index + 19];
							_valid_cal_available[axis_index] = true;
							_last_valid_variance[axis_index] = status.covariances[axis_index + 19];
						}
					}
				}

				// Check and save the last valid calibration when we are disarmed
				if ((vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY)
				    && (status.filter_fault_flags == 0)
				    && (sensor_selection.mag_device_id == _mag_bias_id.get())) {

					BlockParamFloat *mag_biases[] = { &_mag_bias_x, &_mag_bias_y, &_mag_bias_z };

					for (uint8_t axis_index = 0; axis_index <= 2; axis_index++) {
						if (_valid_cal_available[axis_index]) {

							// calculate weighting using ratio of variances and update stored bias values
							const float weighting = constrain(_mag_bias_saved_variance.get() / (_mag_bias_saved_variance.get() +
											  _last_valid_variance[axis_index]), 0.0f, _mag_bias_alpha.get());
							const float mag_bias_saved = mag_biases[axis_index]->get();

							_last_valid_mag_cal[axis_index] = weighting * _last_valid_mag_cal[axis_index] + mag_bias_saved;

							mag_biases[axis_index]->set(_last_valid_mag_cal[axis_index]);
							mag_biases[axis_index]->commit_no_notification();

							_valid_cal_available[axis_index] = false;
						}
					}

					// reset to prevent data being saved too frequently
					_total_cal_time_us = 0;
				}

				{
					// Velocity of body origin in local NED frame (m/s)
					float velocity[3];
					_ekf.get_velocity(velocity);

					matrix::Quatf q;
					_ekf.copy_quaternion(q.data());

					// Calculate wind-compensated velocity in body frame
					Vector3f v_wind_comp(velocity);
					matrix::Dcmf R_to_body(q.inversed());

					float velNE_wind[2];
					_ekf.get_wind_velocity(velNE_wind);

					v_wind_comp(0) -= velNE_wind[0];
					v_wind_comp(1) -= velNE_wind[1];
					_vel_body_wind = R_to_body * v_wind_comp; // TODO: move this elsewhere

					// Publish wind estimate
					wind_estimate_s wind_estimate;
					wind_estimate.timestamp = now;
					wind_estimate.windspeed_north = velNE_wind[0];
					wind_estimate.windspeed_east = velNE_wind[1];
					wind_estimate.variance_north = status.covariances[22];
					wind_estimate.variance_east = status.covariances[23];

					if (_wind_pub == nullptr) {
						_wind_pub = orb_advertise(ORB_ID(wind_estimate), &wind_estimate);

					} else {
						orb_publish(ORB_ID(wind_estimate), _wind_pub, &wind_estimate);
					}
				}
			}

			{
				// publish estimator innovation data
				ekf2_innovations_s innovations;
				innovations.timestamp = now;
				_ekf.get_vel_pos_innov(&innovations.vel_pos_innov[0]);
				_ekf.get_aux_vel_innov(&innovations.aux_vel_innov[0]);
				_ekf.get_mag_innov(&innovations.mag_innov[0]);
				_ekf.get_heading_innov(&innovations.heading_innov);
				_ekf.get_airspeed_innov(&innovations.airspeed_innov);
				_ekf.get_beta_innov(&innovations.beta_innov);
				_ekf.get_flow_innov(&innovations.flow_innov[0]);
				_ekf.get_hagl_innov(&innovations.hagl_innov);
				_ekf.get_drag_innov(&innovations.drag_innov[0]);

				_ekf.get_vel_pos_innov_var(&innovations.vel_pos_innov_var[0]);
				_ekf.get_mag_innov_var(&innovations.mag_innov_var[0]);
				_ekf.get_heading_innov_var(&innovations.heading_innov_var);
				_ekf.get_airspeed_innov_var(&innovations.airspeed_innov_var);
				_ekf.get_beta_innov_var(&innovations.beta_innov_var);
				_ekf.get_flow_innov_var(&innovations.flow_innov_var[0]);
				_ekf.get_hagl_innov_var(&innovations.hagl_innov_var);
				_ekf.get_drag_innov_var(&innovations.drag_innov_var[0]);

				_ekf.get_output_tracking_error(&innovations.output_tracking_error[0]);

				// calculate noise filtered velocity innovations which are used for pre-flight checking
				if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
					// calculate coefficients for LPF applied to innovation sequences
					float alpha = constrain(sensors.accelerometer_integral_dt / 1.e6f * _innov_lpf_tau_inv, 0.0f, 1.0f);
					float beta = 1.0f - alpha;

					// filter the velocity and innvovations
					_vel_ne_innov_lpf(0) = beta * _vel_ne_innov_lpf(0) + alpha * constrain(innovations.vel_pos_innov[0],
							       -_vel_innov_spike_lim, _vel_innov_spike_lim);
					_vel_ne_innov_lpf(1) = beta * _vel_ne_innov_lpf(1) + alpha * constrain(innovations.vel_pos_innov[1],
							       -_vel_innov_spike_lim, _vel_innov_spike_lim);
					_vel_d_innov_lpf = beta * _vel_d_innov_lpf + alpha * constrain(innovations.vel_pos_innov[2],
							   -_vel_innov_spike_lim, _vel_innov_spike_lim);

					// set the max allowed yaw innovaton depending on whether we are not aiding navigation using
					// observations in the NE reference frame.
					filter_control_status_u _ekf_control_mask;
					_ekf.get_control_mode(&_ekf_control_mask.value);
					bool doing_ne_aiding = _ekf_control_mask.flags.gps ||  _ekf_control_mask.flags.ev_pos;

					float yaw_test_limit;

					if (doing_ne_aiding) {
						// use a smaller tolerance when doing NE inertial frame aiding
						yaw_test_limit = _nav_yaw_innov_test_lim;

					} else {
						// use a larger tolerance when not doing NE inertial frame aiding
						yaw_test_limit = _yaw_innov_test_lim;
					}

					// filter the yaw innovations using a decaying envelope filter to prevent innovation sign changes due to angle wrapping allowinging large innvoations to pass checks after filtering.
					_yaw_innov_magnitude_lpf = fmaxf(beta * _yaw_innov_magnitude_lpf,
									 fminf(fabsf(innovations.heading_innov), 2.0f * yaw_test_limit));

					_hgt_innov_lpf = beta * _hgt_innov_lpf + alpha * constrain(innovations.vel_pos_innov[5], -_hgt_innov_spike_lim,
							 _hgt_innov_spike_lim);

					// check the yaw and horizontal velocity innovations
					float vel_ne_innov_length = sqrtf(innovations.vel_pos_innov[0] * innovations.vel_pos_innov[0] +
									  innovations.vel_pos_innov[1] * innovations.vel_pos_innov[1]);
					_preflt_horiz_fail = (_vel_ne_innov_lpf.norm() > _vel_innov_test_lim)
							     || (vel_ne_innov_length > 2.0f * _vel_innov_test_lim)
							     || (_yaw_innov_magnitude_lpf > yaw_test_limit);

					// check the vertical velocity and position innovations
					_preflt_vert_fail = (fabsf(_vel_d_innov_lpf) > _vel_innov_test_lim)
							    || (fabsf(innovations.vel_pos_innov[2]) > 2.0f * _vel_innov_test_lim)
							    || (fabsf(_hgt_innov_lpf) > _hgt_innov_test_lim);

					// master pass-fail status
					_preflt_fail = _preflt_horiz_fail || _preflt_vert_fail;

				} else {
					_vel_ne_innov_lpf.zero();
					_vel_d_innov_lpf = 0.0f;
					_hgt_innov_lpf = 0.0f;
					_preflt_horiz_fail = false;
					_preflt_vert_fail = false;
					_preflt_fail = false;
				}

				if (_estimator_innovations_pub == nullptr) {
					_estimator_innovations_pub = orb_advertise(ORB_ID(ekf2_innovations), &innovations);

				} else {
					orb_publish(ORB_ID(ekf2_innovations), _estimator_innovations_pub, &innovations);
				}
			}

		} else if (_replay_mode) {
			// in replay mode we have to tell the replay module not to wait for an update
			// we do this by publishing an attitude with zero timestamp
			vehicle_attitude_s att;
			att.timestamp = now;

			if (_att_pub == nullptr) {
				_att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

			} else {
				orb_publish(ORB_ID(vehicle_attitude), _att_pub, &att);
			}
		}

		{
			// publish ekf2_timestamps (using 0.1 ms relative timestamps)
			ekf2_timestamps_s ekf2_timestamps;
			ekf2_timestamps.timestamp = sensors.timestamp;

			if (gps_updated) {
				// divide individually to get consistent rounding behavior
				ekf2_timestamps.gps_timestamp_rel = (int16_t)((int64_t)gps.timestamp / 100 - (int64_t)ekf2_timestamps.timestamp / 100);

			} else {
				ekf2_timestamps.gps_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
			}

			if (optical_flow_updated) {
				ekf2_timestamps.optical_flow_timestamp_rel = (int16_t)((int64_t)optical_flow.timestamp / 100 -
						(int64_t)ekf2_timestamps.timestamp / 100);

			} else {
				ekf2_timestamps.optical_flow_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
			}

			if (range_finder_updated) {
				ekf2_timestamps.distance_sensor_timestamp_rel = (int16_t)((int64_t)range_finder.timestamp / 100 -
						(int64_t)ekf2_timestamps.timestamp / 100);

			} else {
				ekf2_timestamps.distance_sensor_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
			}

			if (airspeed_updated) {
				ekf2_timestamps.airspeed_timestamp_rel = (int16_t)((int64_t)airspeed.timestamp / 100 -
						(int64_t)ekf2_timestamps.timestamp / 100);

			} else {
				ekf2_timestamps.airspeed_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
			}

			if (vision_position_updated) {
				ekf2_timestamps.vision_position_timestamp_rel = (int16_t)((int64_t)ev_pos.timestamp / 100 -
						(int64_t)ekf2_timestamps.timestamp / 100);

			} else {
				ekf2_timestamps.vision_position_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
			}

			if (vision_attitude_updated) {
				ekf2_timestamps.vision_attitude_timestamp_rel = (int16_t)((int64_t)ev_att.timestamp / 100 -
						(int64_t)ekf2_timestamps.timestamp / 100);

			} else {
				ekf2_timestamps.vision_attitude_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
			}

			if (_ekf2_timestamps_pub == nullptr) {
				_ekf2_timestamps_pub = orb_advertise(ORB_ID(ekf2_timestamps), &ekf2_timestamps);

			} else {
				orb_publish(ORB_ID(ekf2_timestamps), _ekf2_timestamps_pub, &ekf2_timestamps);
			}
		}
	}

	orb_unsubscribe(sensors_sub);
	orb_unsubscribe(gps_sub);
	orb_unsubscribe(airspeed_sub);
	orb_unsubscribe(params_sub);
	orb_unsubscribe(optical_flow_sub);
	orb_unsubscribe(ev_pos_sub);
	orb_unsubscribe(ev_att_sub);
	orb_unsubscribe(vehicle_land_detected_sub);
	orb_unsubscribe(status_sub);
	orb_unsubscribe(sensor_selection_sub);
	orb_unsubscribe(sensor_baro_sub);
	orb_unsubscribe(landing_target_pose_sub);

	for (unsigned i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		orb_unsubscribe(range_finder_subs[i]);
		range_finder_subs[i] = -1;
	}
}

int Ekf2::getRangeSubIndex(const int *subs)
{
	for (unsigned i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		bool updated = false;
		orb_check(subs[i], &updated);

		if (updated) {
			distance_sensor_s report;
			orb_copy(ORB_ID(distance_sensor), subs[i], &report);

			// only use the first instace which has the correct orientation
			if (report.orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING) {
				PX4_INFO("Found range finder with instance %d", i);
				return i;
			}
		}
	}

	return -1;
}

Ekf2 *Ekf2::instantiate(int argc, char *argv[])
{
	Ekf2 *instance = new Ekf2();

	if (instance) {
		if (argc >= 2 && !strcmp(argv[1], "-r")) {
			instance->set_replay_mode(true);
		}
	}

	return instance;
}

int Ekf2::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Ekf2::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Attitude and position estimator using an Extended Kalman Filter. It is used for Multirotors and Fixed-Wing.

The documentation can be found on the [tuning_the_ecl_ekf](https://dev.px4.io/en/tutorials/tuning_the_ecl_ekf.html) page.

ekf2 can be started in replay mode (`-r`): in this mode it does not access the system time, but only uses the
timestamps from the sensor topics.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ekf2", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('r', "Enable replay mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int Ekf2::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("ekf2",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_ESTIMATOR,
				      6600,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

int ekf2_main(int argc, char *argv[])
{
	return Ekf2::main(argc, argv);
}
