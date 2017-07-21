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

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_module.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <float.h>

#include <arch/board/board.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <platforms/px4_defines.h>
#include <drivers/drv_hrt.h>
#include <controllib/blocks.hpp>

#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/ekf2_innovations.h>
#include <uORB/topics/ekf2_replay.h>
#include <uORB/topics/ekf2_timestamps.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_selection.h>

#include <ecl/EKF/ekf.h>


extern "C" __EXPORT int ekf2_main(int argc, char *argv[]);


class Ekf2 : public control::SuperBlock, public ModuleBase<Ekf2>
{
public:
	Ekf2();

	~Ekf2();

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

	void 	set_replay_mode(bool replay) {_replay_mode = replay;};

	static void	task_main_trampoline(int argc, char *argv[]);

	int print_status() override;

private:
	static constexpr float _dt_max = 0.02;	///< minimum allowed arrival time between non-IMU sensor readings  (sec)
	bool 	_replay_mode;			///< true when we use replay data from a log
	int32_t _publish_replay_mode;		///< set to 1 if we should publish replay messages for logging

	float	_default_ev_pos_noise = 0.05f;	///< external vision position noise used when an invalid value is supplied (m)
	float	_default_ev_ang_noise = 0.05f;	///< external vision angle noise used when an invalid value is supplied (rad)

	// time slip monitoring
	uint64_t integrated_time_us = 0;	///< integral of gyro delta time from start (uSec)
	uint64_t start_time_us = 0;		///< system time at EKF start (uSec)

	// Initialise time stamps used to send sensor data to the EKF and for logging
	uint64_t _timestamp_mag_us = 0;		///< magnetomer data timestamp (uSec)
	uint64_t _timestamp_balt_us = 0;	///< pressure altitude data timestamp (uSec)
	uint8_t _invalid_mag_id_count = 0;	///< number of times an invalid magnetomer device ID has been detected

	// Used to down sample magnetometer data
	float _mag_data_sum[3];			///< summed magnetometer readings (Gauss)
	uint64_t _mag_time_sum_ms = 0;		///< summed magnetoemter time stamps (mSec)
	uint8_t _mag_sample_count = 0;		///< number of magnetometer measurements summed during downsampling
	uint32_t _mag_time_ms_last_used =
		0;	///< time stamp of the last averaged magnetometer measurement sent to the EKF (mSec)

	// Used to down sample barometer data
	float _balt_data_sum;			///< summed pressure altitude readings (m)
	uint64_t _balt_time_sum_ms = 0;		///< summed pressure altitude time stamps (mSec)
	uint8_t _balt_sample_count = 0;		///< number of barometric altitude measurements summed
	uint32_t _balt_time_ms_last_used =
		0;	///< time stamp of the last averaged barometric altitude measurement sent to the EKF (mSec)

	float _acc_hor_filt = 0.0f; 	///< low-pass filtered horizontal acceleration (m/sec**2)

	// Used to check, save and use learned magnetometer biases
	hrt_abstime _last_magcal_us = 0;	///< last time the EKF was operating a mode that estimates magnetomer biases (uSec)
	hrt_abstime _total_cal_time_us = 0;	///< accumulated calibration time since the last save
	hrt_abstime _last_time_slip_us = 0;	///< Last time slip (uSec)
	float _last_valid_mag_cal[3] = {};	///< last valid XYZ magnetometer bias estimates (mGauss)
	bool _valid_cal_available[3] = {};	///< true when an unsaved valid calibration for the XYZ magnetometer bias is available
	float _last_valid_variance[3] = {};	///< variances for the last valid magnetometer XYZ bias estimates (mGauss**2)

	// Used to filter velocity innovations during pre-flight checks
	Vector3f _vel_innov_lpf_ned = {};	///< Preflight low pass filtered velocity innovations (m/sec)
	float _hgt_innov_lpf = 0.0f;		///< Preflight low pass filtered height innovation (m)
	const float _innov_lpf_tau_inv = 0.2f;	///< Preflight low pass filter time constant inverse (1/sec)
	const float _vel_innov_test_lim = 0.5f;	///< Maximum permissible velocity innovation to pass pre-flight checks (m/sec)
	const float _hgt_innov_test_lim = 1.5f;	///< Maximum permissible height innovation to pass pre-flight checks (m)
	const float _vel_innov_spike_lim = 2.0f * _vel_innov_test_lim;	///< preflight velocity innovation spike limit (m/sec)
	const float _hgt_innov_spike_lim = 2.0f * _hgt_innov_test_lim;	///< preflight position innovation spike limit (m)
	bool _vel_innov_preflt_fail = false;	///< true if the norm of the filtered innovation vector is too large before flight

	orb_advert_t _att_pub;
	orb_advert_t _lpos_pub;
	orb_advert_t _control_state_pub;
	orb_advert_t _vehicle_global_position_pub;
	orb_advert_t _wind_pub;
	orb_advert_t _estimator_status_pub;
	orb_advert_t _estimator_innovations_pub;
	orb_advert_t _replay_pub;
	orb_advert_t _ekf2_timestamps_pub;

	/* Low pass filter for attitude rates */
	math::LowPassFilter2p _lp_roll_rate;	///< Low pass filter applied to roll rates published on the control_state message
	math::LowPassFilter2p _lp_pitch_rate;	///< Low pass filter applied to pitch rates published on the control_state message
	math::LowPassFilter2p _lp_yaw_rate;	///< Low pass filter applied to yaw rates published on the control_state message

	Ekf _ekf;

	parameters *_params;	///< pointer to ekf parameter struct (located in _ekf class instance)

	control::BlockParamExtInt
	_obs_dt_min_ms;	///< Maximmum time delay of any sensor used to increse buffer length to handle large timing jitter (mSec)
	control::BlockParamExtFloat _mag_delay_ms;	///< magnetometer measurement delay relative to the IMU (mSec)
	control::BlockParamExtFloat _baro_delay_ms;	///< barometer height measurement delay relative to the IMU (mSec)
	control::BlockParamExtFloat _gps_delay_ms;	///< GPS measurement delay relative to the IMU (mSec)
	control::BlockParamExtFloat
	_flow_delay_ms;	///< optical flow measurement delay relative to the IMU (mSec) - this is to the middle of the optical flow integration interval
	control::BlockParamExtFloat _rng_delay_ms;	///< range finder measurement delay relative to the IMU (mSec)
	control::BlockParamExtFloat _airspeed_delay_ms;	///< airspeed measurement delay relative to the IMU (mSec)
	control::BlockParamExtFloat _ev_delay_ms;	///< off-board vision measurement delay relative to the IMU (mSec)

	control::BlockParamExtFloat _gyro_noise;	///< IMU angular rate noise used for covariance prediction (rad/sec)
	control::BlockParamExtFloat _accel_noise;	///< IMU acceleration noise use for covariance prediction (m/sec**2)

	// process noise
	control::BlockParamExtFloat _gyro_bias_p_noise;	///< process noise for IMU rate gyro bias prediction (rad/sec**2)
	control::BlockParamExtFloat _accel_bias_p_noise;///< process noise for IMU accelerometer bias prediction (m/sec**3)
	control::BlockParamExtFloat _mage_p_noise;	///< process noise for earth magnetic field prediction (Gauss/sec)
	control::BlockParamExtFloat _magb_p_noise;	///< process noise for body magnetic field prediction (Gauss/sec)
	control::BlockParamExtFloat _wind_vel_p_noise;	///< process noise for wind velocity prediction (m/sec**2)
	control::BlockParamExtFloat _terrain_p_noise;	///< process noise for terrain offset (m/sec)
	control::BlockParamExtFloat
	_terrain_gradient;	///< gradient of terrain used to estimate process noise due to changing position (m/m)

	control::BlockParamExtFloat _gps_vel_noise;	///< minimum allowed observation noise for gps velocity fusion (m/sec)
	control::BlockParamExtFloat _gps_pos_noise;	///< minimum allowed observation noise for gps position fusion (m)
	control::BlockParamExtFloat _pos_noaid_noise;	///< observation noise for non-aiding position fusion (m)
	control::BlockParamExtFloat _baro_noise;	///< observation noise for barometric height fusion (m)
	control::BlockParamExtFloat _baro_innov_gate;	///< barometric height innovation consistency gate size (STD)
	control::BlockParamExtFloat _posNE_innov_gate;	///< GPS horizontal position innovation consistency gate size (STD)
	control::BlockParamExtFloat _vel_innov_gate;	///< GPS velocity innovation consistency gate size (STD)
	control::BlockParamExtFloat _tas_innov_gate;	///< True Airspeed innovation consistency gate size (STD)

	// control of magnetometer fusion
	control::BlockParamExtFloat _mag_heading_noise;	///< measurement noise used for simple heading fusion (rad)
	control::BlockParamExtFloat _mag_noise;		///< measurement noise used for 3-axis magnetoemeter fusion (Gauss)
	control::BlockParamExtFloat _eas_noise;		///< measurement noise used for airspeed fusion (m/sec)
	control::BlockParamExtFloat _beta_noise;	///< synthetic sideslip noise (rad)
	control::BlockParamExtFloat _mag_declination_deg;///< magnetic declination (degrees)
	control::BlockParamExtFloat _heading_innov_gate;///< heading fusion innovation consistency gate size (STD)
	control::BlockParamExtFloat _mag_innov_gate;	///< magnetometer fusion innovation consistency gate size (STD)
	control::BlockParamExtInt _mag_decl_source;	///< bitmask used to control the handling of declination data
	control::BlockParamExtInt _mag_fuse_type;	///< integer used to specify the type of magnetometer fusion used
	control::BlockParamExtFloat _mag_acc_gate;	///< integer used to specify the type of magnetometer fusion used
	control::BlockParamExtFloat _mag_yaw_rate_gate;	///< yaw rate threshold used by mode select logic (rad/sec)

	control::BlockParamExtInt _gps_check_mask;	///< bitmask used to control which GPS quality checks are used
	control::BlockParamExtFloat _requiredEph;	///< maximum acceptable horiz position error (m)
	control::BlockParamExtFloat _requiredEpv;	///< maximum acceptable vert position error (m)
	control::BlockParamExtFloat _requiredSacc;	///< maximum acceptable speed error (m/s)
	control::BlockParamExtInt _requiredNsats;	///< minimum acceptable satellite count
	control::BlockParamExtFloat _requiredGDoP;	///< maximum acceptable geometric dilution of precision
	control::BlockParamExtFloat _requiredHdrift;	///< maximum acceptable horizontal drift speed (m/s)
	control::BlockParamExtFloat _requiredVdrift;	///< maximum acceptable vertical drift speed (m/s)
	control::BlockParamExtInt _param_record_replay_msg;	///< turns on recording of ekf2 replay messages

	// measurement source control
	control::BlockParamExtInt
	_fusion_mode;		///< bitmasked integer that selects which of the GPS and optical flow aiding sources will be used
	control::BlockParamExtInt _vdist_sensor_type;	///< selects the primary source for height data

	// range finder fusion
	control::BlockParamExtFloat _range_noise;	///< observation noise for range finder measurements (m)
	control::BlockParamExtFloat _range_noise_scaler; ///< scale factor from range to range noise (m/m)
	control::BlockParamExtFloat _range_innov_gate;	///< range finder fusion innovation consistency gate size (STD)
	control::BlockParamExtFloat _rng_gnd_clearance;	///< minimum valid value for range when on ground (m)
	control::BlockParamExtFloat _rng_pitch_offset;	///< range sensor pitch offset (rad)
	control::BlockParamExtInt
	_rng_aid;		///< enables use of a range finder even if primary height source is not range finder (EKF2_HGT_MODE != 2)
	control::BlockParamExtFloat _rng_aid_hor_vel_max;	///< maximum allowed horizontal velocity for range aid (m/s)
	control::BlockParamExtFloat _rng_aid_height_max;	///< maximum allowed absolute altitude (AGL) for range aid (m)
	control::BlockParamExtFloat
	_rng_aid_innov_gate;	///< gate size used for innovation consistency checks for range aid fusion (STD)

	// vision estimate fusion
	control::BlockParamExtFloat _ev_pos_noise;	///< default position observation noise for exernal vision measurements (m)
	control::BlockParamExtFloat _ev_ang_noise;	///< default angular observation noise for exernal vision measurements (rad)
	control::BlockParamExtFloat _ev_innov_gate;	///< external vision position innovation consistency gate size (STD)

	// optical flow fusion
	control::BlockParamExtFloat
	_flow_noise;	///< best quality observation noise for optical flow LOS rate measurements (rad/sec)
	control::BlockParamExtFloat
	_flow_noise_qual_min;	///< worst quality observation noise for optical flow LOS rate measurements (rad/sec)
	control::BlockParamExtInt _flow_qual_min;	///< minimum acceptable quality integer from  the flow sensor
	control::BlockParamExtFloat _flow_innov_gate;	///< optical flow fusion innovation consistency gate size (STD)
	control::BlockParamExtFloat _flow_rate_max;	///< maximum valid optical flow rate (rad/sec)

	// sensor positions in body frame
	control::BlockParamExtFloat _imu_pos_x;		///< X position of IMU in body frame (m)
	control::BlockParamExtFloat _imu_pos_y;		///< Y position of IMU in body frame (m)
	control::BlockParamExtFloat _imu_pos_z;		///< Z position of IMU in body frame (m)
	control::BlockParamExtFloat _gps_pos_x;		///< X position of GPS antenna in body frame (m)
	control::BlockParamExtFloat _gps_pos_y;		///< Y position of GPS antenna in body frame (m)
	control::BlockParamExtFloat _gps_pos_z;		///< Z position of GPS antenna in body frame (m)
	control::BlockParamExtFloat _rng_pos_x;		///< X position of range finder in body frame (m)
	control::BlockParamExtFloat _rng_pos_y;		///< Y position of range finder in body frame (m)
	control::BlockParamExtFloat _rng_pos_z;		///< Z position of range finder in body frame (m)
	control::BlockParamExtFloat _flow_pos_x;	///< X position of optical flow sensor focal point in body frame (m)
	control::BlockParamExtFloat _flow_pos_y;	///< Y position of optical flow sensor focal point in body frame (m)
	control::BlockParamExtFloat _flow_pos_z;	///< Z position of optical flow sensor focal point in body frame (m)
	control::BlockParamExtFloat _ev_pos_x;		///< X position of VI sensor focal point in body frame (m)
	control::BlockParamExtFloat _ev_pos_y;		///< Y position of VI sensor focal point in body frame (m)
	control::BlockParamExtFloat _ev_pos_z;		///< Z position of VI sensor focal point in body frame (m)

	// control of airspeed and sideslip fusion
	control::BlockParamFloat
	_arspFusionThreshold; 	///< A value of zero will disabled airspeed fusion. Any positive value sets the minimum airspeed which will be used (m/sec)
	control::BlockParamInt _fuseBeta;		///< Controls synthetic sideslip fusion, 0 disables, 1 enables

	// output predictor filter time constants
	control::BlockParamExtFloat _tau_vel;		///< time constant used by the output velocity complementary filter (sec)
	control::BlockParamExtFloat _tau_pos;		///< time constant used by the output position complementary filter (sec)

	// IMU switch on bias paameters
	control::BlockParamExtFloat _gyr_bias_init;	///< 1-sigma gyro bias uncertainty at switch on (rad/sec)
	control::BlockParamExtFloat _acc_bias_init;	///< 1-sigma accelerometer bias uncertainty at switch on (m/sec**2)
	control::BlockParamExtFloat _ang_err_init;	///< 1-sigma tilt error after initial alignment using gravity vector (rad)

	// airspeed mode parameter
	control::BlockParamInt _airspeed_mode;

	// EKF saved XYZ magnetometer bias values
	control::BlockParamFloat _mag_bias_x;		///< X magnetometer bias (mGauss)
	control::BlockParamFloat _mag_bias_y;		///< Y magnetometer bias (mGauss)
	control::BlockParamFloat _mag_bias_z;		///< Z magnetometer bias (mGauss)
	control::BlockParamInt _mag_bias_id;		///< ID of the magnetometer sensor used to learn the bias values
	control::BlockParamFloat
	_mag_bias_saved_variance; ///< Assumed error variance of previously saved magnetometer bias estimates (mGauss**2)
	control::BlockParamFloat
	_mag_bias_alpha;	///< maximum fraction of the learned magnetometer bias that is saved at each disarm

};

Ekf2::Ekf2():
	SuperBlock(nullptr, "EKF"),
	_replay_mode(false),
	_publish_replay_mode(0),
	_att_pub(nullptr),
	_lpos_pub(nullptr),
	_control_state_pub(nullptr),
	_vehicle_global_position_pub(nullptr),
	_wind_pub(nullptr),
	_estimator_status_pub(nullptr),
	_estimator_innovations_pub(nullptr),
	_replay_pub(nullptr),
	_ekf2_timestamps_pub(nullptr),
	_lp_roll_rate(250.0f, 30.0f),
	_lp_pitch_rate(250.0f, 30.0f),
	_lp_yaw_rate(250.0f, 20.0f),
	_params(_ekf.getParamHandle()),
	_obs_dt_min_ms(this, "EKF2_MIN_OBS_DT", false, _params->sensor_interval_min_ms),
	_mag_delay_ms(this, "EKF2_MAG_DELAY", false, _params->mag_delay_ms),
	_baro_delay_ms(this, "EKF2_BARO_DELAY", false, _params->baro_delay_ms),
	_gps_delay_ms(this, "EKF2_GPS_DELAY", false, _params->gps_delay_ms),
	_flow_delay_ms(this, "EKF2_OF_DELAY", false, _params->flow_delay_ms),
	_rng_delay_ms(this, "EKF2_RNG_DELAY", false, _params->range_delay_ms),
	_airspeed_delay_ms(this, "EKF2_ASP_DELAY", false, _params->airspeed_delay_ms),
	_ev_delay_ms(this, "EKF2_EV_DELAY", false, _params->ev_delay_ms),
	_gyro_noise(this, "EKF2_GYR_NOISE", false, _params->gyro_noise),
	_accel_noise(this, "EKF2_ACC_NOISE", false, _params->accel_noise),
	_gyro_bias_p_noise(this, "EKF2_GYR_B_NOISE", false, _params->gyro_bias_p_noise),
	_accel_bias_p_noise(this, "EKF2_ACC_B_NOISE", false, _params->accel_bias_p_noise),
	_mage_p_noise(this, "EKF2_MAG_E_NOISE", false, _params->mage_p_noise),
	_magb_p_noise(this, "EKF2_MAG_B_NOISE", false, _params->magb_p_noise),
	_wind_vel_p_noise(this, "EKF2_WIND_NOISE", false, _params->wind_vel_p_noise),
	_terrain_p_noise(this, "EKF2_TERR_NOISE", false, _params->terrain_p_noise),
	_terrain_gradient(this, "EKF2_TERR_GRAD", false, _params->terrain_gradient),
	_gps_vel_noise(this, "EKF2_GPS_V_NOISE", false, _params->gps_vel_noise),
	_gps_pos_noise(this, "EKF2_GPS_P_NOISE", false, _params->gps_pos_noise),
	_pos_noaid_noise(this, "EKF2_NOAID_NOISE", false, _params->pos_noaid_noise),
	_baro_noise(this, "EKF2_BARO_NOISE", false, _params->baro_noise),
	_baro_innov_gate(this, "EKF2_BARO_GATE", false, _params->baro_innov_gate),
	_posNE_innov_gate(this, "EKF2_GPS_P_GATE", false, _params->posNE_innov_gate),
	_vel_innov_gate(this, "EKF2_GPS_V_GATE", false, _params->vel_innov_gate),
	_tas_innov_gate(this, "EKF2_TAS_GATE", false, _params->tas_innov_gate),
	_mag_heading_noise(this, "EKF2_HEAD_NOISE", false, _params->mag_heading_noise),
	_mag_noise(this, "EKF2_MAG_NOISE", false, _params->mag_noise),
	_eas_noise(this, "EKF2_EAS_NOISE", false, _params->eas_noise),
	_beta_noise(this, "EKF2_BETA_NOISE", false, _params->beta_noise),
	_mag_declination_deg(this, "EKF2_MAG_DECL", false, _params->mag_declination_deg),
	_heading_innov_gate(this, "EKF2_HDG_GATE", false, _params->heading_innov_gate),
	_mag_innov_gate(this, "EKF2_MAG_GATE", false, _params->mag_innov_gate),
	_mag_decl_source(this, "EKF2_DECL_TYPE", false, _params->mag_declination_source),
	_mag_fuse_type(this, "EKF2_MAG_TYPE", false, _params->mag_fusion_type),
	_mag_acc_gate(this, "EKF2_MAG_ACCLIM", false, _params->mag_acc_gate),
	_mag_yaw_rate_gate(this, "EKF2_MAG_YAWLIM", false, _params->mag_yaw_rate_gate),
	_gps_check_mask(this, "EKF2_GPS_CHECK", false, _params->gps_check_mask),
	_requiredEph(this, "EKF2_REQ_EPH", false, _params->req_hacc),
	_requiredEpv(this, "EKF2_REQ_EPV", false, _params->req_vacc),
	_requiredSacc(this, "EKF2_REQ_SACC", false, _params->req_sacc),
	_requiredNsats(this, "EKF2_REQ_NSATS", false, _params->req_nsats),
	_requiredGDoP(this, "EKF2_REQ_GDOP", false, _params->req_gdop),
	_requiredHdrift(this, "EKF2_REQ_HDRIFT", false, _params->req_hdrift),
	_requiredVdrift(this, "EKF2_REQ_VDRIFT", false, _params->req_vdrift),
	_param_record_replay_msg(this, "EKF2_REC_RPL", false, _publish_replay_mode),
	_fusion_mode(this, "EKF2_AID_MASK", false, _params->fusion_mode),
	_vdist_sensor_type(this, "EKF2_HGT_MODE", false, _params->vdist_sensor_type),
	_range_noise(this, "EKF2_RNG_NOISE", false, _params->range_noise),
	_range_noise_scaler(this, "EKF2_RNG_SFE", false, _params->range_noise_scaler),
	_range_innov_gate(this, "EKF2_RNG_GATE", false, _params->range_innov_gate),
	_rng_gnd_clearance(this, "EKF2_MIN_RNG", false, _params->rng_gnd_clearance),
	_rng_pitch_offset(this, "EKF2_RNG_PITCH", false, _params->rng_sens_pitch),
	_rng_aid(this, "EKF2_RNG_AID", false, _params->range_aid),
	_rng_aid_hor_vel_max(this, "EKF2_RNG_A_VMAX", false, _params->max_vel_for_range_aid),
	_rng_aid_height_max(this, "EKF2_RNG_A_HMAX", false, _params->max_hagl_for_range_aid),
	_rng_aid_innov_gate(this, "EKF2_RNG_A_IGATE", false, _params->range_aid_innov_gate),
	_ev_pos_noise(this, "EKF2_EVP_NOISE", false, _default_ev_pos_noise),
	_ev_ang_noise(this, "EKF2_EVA_NOISE", false, _default_ev_ang_noise),
	_ev_innov_gate(this, "EKF2_EV_GATE", false, _params->ev_innov_gate),
	_flow_noise(this, "EKF2_OF_N_MIN", false, _params->flow_noise),
	_flow_noise_qual_min(this, "EKF2_OF_N_MAX", false, _params->flow_noise_qual_min),
	_flow_qual_min(this, "EKF2_OF_QMIN", false, _params->flow_qual_min),
	_flow_innov_gate(this, "EKF2_OF_GATE", false, _params->flow_innov_gate),
	_flow_rate_max(this, "EKF2_OF_RMAX", false, _params->flow_rate_max),
	_imu_pos_x(this, "EKF2_IMU_POS_X", false, _params->imu_pos_body(0)),
	_imu_pos_y(this, "EKF2_IMU_POS_Y", false, _params->imu_pos_body(1)),
	_imu_pos_z(this, "EKF2_IMU_POS_Z", false, _params->imu_pos_body(2)),
	_gps_pos_x(this, "EKF2_GPS_POS_X", false, _params->gps_pos_body(0)),
	_gps_pos_y(this, "EKF2_GPS_POS_Y", false, _params->gps_pos_body(1)),
	_gps_pos_z(this, "EKF2_GPS_POS_Z", false, _params->gps_pos_body(2)),
	_rng_pos_x(this, "EKF2_RNG_POS_X", false, _params->rng_pos_body(0)),
	_rng_pos_y(this, "EKF2_RNG_POS_Y", false, _params->rng_pos_body(1)),
	_rng_pos_z(this, "EKF2_RNG_POS_Z", false, _params->rng_pos_body(2)),
	_flow_pos_x(this, "EKF2_OF_POS_X", false, _params->flow_pos_body(0)),
	_flow_pos_y(this, "EKF2_OF_POS_Y", false, _params->flow_pos_body(1)),
	_flow_pos_z(this, "EKF2_OF_POS_Z", false, _params->flow_pos_body(2)),
	_ev_pos_x(this, "EKF2_EV_POS_X", false, _params->ev_pos_body(0)),
	_ev_pos_y(this, "EKF2_EV_POS_Y", false, _params->ev_pos_body(1)),
	_ev_pos_z(this, "EKF2_EV_POS_Z", false, _params->ev_pos_body(2)),
	_arspFusionThreshold(this, "EKF2_ARSP_THR", false),
	_fuseBeta(this, "EKF2_FUSE_BETA", false),
	_tau_vel(this, "EKF2_TAU_VEL", false, _params->vel_Tau),
	_tau_pos(this, "EKF2_TAU_POS", false, _params->pos_Tau),
	_gyr_bias_init(this, "EKF2_GBIAS_INIT", false, _params->switch_on_gyro_bias),
	_acc_bias_init(this, "EKF2_ABIAS_INIT", false, _params->switch_on_accel_bias),
	_ang_err_init(this, "EKF2_ANGERR_INIT", false, _params->initial_tilt_err),
	_airspeed_mode(this, "FW_ARSP_MODE", false),
	_mag_bias_x(this, "EKF2_MAGBIAS_X", false),
	_mag_bias_y(this, "EKF2_MAGBIAS_Y", false),
	_mag_bias_z(this, "EKF2_MAGBIAS_Z", false),
	_mag_bias_id(this, "EKF2_MAGBIAS_ID", false),
	_mag_bias_saved_variance(this, "EKF2_MAGB_VREF", false),
	_mag_bias_alpha(this, "EKF2_MAGB_K", false)

{

}

Ekf2::~Ekf2()
{

}

int Ekf2::print_status()
{
	PX4_INFO("local position OK %s", (_ekf.local_position_is_valid()) ? "yes" : "no");
	PX4_INFO("global position OK %s", (_ekf.global_position_is_valid()) ? "yes" : "no");
	PX4_INFO("time slip: %" PRIu64 " us", _last_time_slip_us);
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
	int range_finder_sub = orb_subscribe(ORB_ID(distance_sensor));
	int ev_pos_sub = orb_subscribe(ORB_ID(vehicle_vision_position));
	int ev_att_sub = orb_subscribe(ORB_ID(vehicle_vision_attitude));
	int vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	int status_sub = orb_subscribe(ORB_ID(vehicle_status));
	int sensor_selection_sub = orb_subscribe(ORB_ID(sensor_selection));

	px4_pollfd_struct_t fds[2] = {};
	fds[0].fd = sensors_sub;
	fds[0].events = POLLIN;
	fds[1].fd = params_sub;
	fds[1].events = POLLIN;

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

	while (!should_exit()) {
		int ret = px4_poll(fds, sizeof(fds) / sizeof(fds[0]), 1000);

		if (ret < 0) {
			// Poll error, sleep and try again
			usleep(10000);
			continue;

		} else if (ret == 0) {
			// Poll timeout or no new data, do nothing
			continue;
		}

		if (fds[1].revents & POLLIN) {
			// read from param to clear updated flag
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), params_sub, &update);
			updateParams();

			// fetch sensor data in next loop
			continue;

		} else if (!(fds[0].revents & POLLIN)) {
			// no new data
			continue;
		}

		bool gps_updated = false;
		bool airspeed_updated = false;
		bool optical_flow_updated = false;
		bool range_finder_updated = false;
		bool vehicle_land_detected_updated = false;
		bool vision_position_updated = false;
		bool vision_attitude_updated = false;
		bool vehicle_status_updated = false;

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

		orb_check(airspeed_sub, &airspeed_updated);

		if (airspeed_updated) {
			orb_copy(ORB_ID(airspeed), airspeed_sub, &airspeed);
		}

		orb_check(optical_flow_sub, &optical_flow_updated);

		if (optical_flow_updated) {
			orb_copy(ORB_ID(optical_flow), optical_flow_sub, &optical_flow);
		}

		orb_check(range_finder_sub, &range_finder_updated);

		if (range_finder_updated) {
			orb_copy(ORB_ID(distance_sensor), range_finder_sub, &range_finder);

			if (range_finder.min_distance > range_finder.current_distance
			    || range_finder.max_distance < range_finder.current_distance) {
				range_finder_updated = false;
			}
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
		_ekf.setIMUData(now, sensors.gyro_integral_dt, sensors.accelerometer_integral_dt,
				gyro_integral, accel_integral);

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
				orb_copy(ORB_ID(sensor_selection), sensor_selection_sub, &sensor_selection);

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
					float mag_sample_count_inv = 1.0f / (float)_mag_sample_count;
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
					float balt_data_avg = _balt_data_sum / (float)_balt_sample_count;
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
			struct gps_message gps_msg = {};
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
			//TODO add gdop to gps topic
			gps_msg.gdop = 0.0f;

			_ekf.setGpsData(gps.timestamp, &gps_msg);

		}

		// only set airspeed data if condition for airspeed fusion are met
		bool fuse_airspeed = airspeed_updated && !vehicle_status.is_rotary_wing
				     && _arspFusionThreshold.get() <= airspeed.true_airspeed_m_s && _arspFusionThreshold.get() >= 0.1f;

		if (fuse_airspeed) {
			float eas2tas = airspeed.true_airspeed_m_s / airspeed.indicated_airspeed_m_s;
			_ekf.setAirspeedData(airspeed.timestamp, airspeed.true_airspeed_m_s, eas2tas);
		}

		// only fuse synthetic sideslip measurements if conditions are met
		bool fuse_beta = !vehicle_status.is_rotary_wing && _fuseBeta.get();
		_ekf.set_fuse_beta_flag(fuse_beta);

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
			ev_data.posErr = _default_ev_pos_noise;
			ev_data.angErr = _default_ev_ang_noise;

			// use timestamp from external computer, clocks are synchronized when using MAVROS
			_ekf.setExtVisionData(vision_position_updated ? ev_pos.timestamp : ev_att.timestamp, &ev_data);
		}

		orb_check(vehicle_land_detected_sub, &vehicle_land_detected_updated);

		if (vehicle_land_detected_updated) {
			orb_copy(ORB_ID(vehicle_land_detected), vehicle_land_detected_sub, &vehicle_land_detected);
			_ekf.set_in_air_status(!vehicle_land_detected.landed);
		}

		// run the EKF update and output
		if (_ekf.update()) {

			// integrate time to monitor time slippage
			if (start_time_us == 0) {
				start_time_us = now;

			} else if (start_time_us > 0) {
				integrated_time_us += sensors.gyro_integral_dt;
			}

			matrix::Quaternion<float> q;
			_ekf.copy_quaternion(q.data());

			float velocity[3];
			_ekf.get_velocity(velocity);

			float pos_d_deriv;
			_ekf.get_pos_d_deriv(&pos_d_deriv);

			float gyro_rad[3];

			{
				// generate control state data
				control_state_s ctrl_state = {};
				float gyro_bias[3] = {};
				_ekf.get_gyro_bias(gyro_bias);
				ctrl_state.timestamp = now;
				gyro_rad[0] = sensors.gyro_rad[0] - gyro_bias[0];
				gyro_rad[1] = sensors.gyro_rad[1] - gyro_bias[1];
				gyro_rad[2] = sensors.gyro_rad[2] - gyro_bias[2];
				ctrl_state.roll_rate = _lp_roll_rate.apply(gyro_rad[0]);
				ctrl_state.pitch_rate = _lp_pitch_rate.apply(gyro_rad[1]);
				ctrl_state.yaw_rate = _lp_yaw_rate.apply(gyro_rad[2]);
				ctrl_state.roll_rate_bias = gyro_bias[0];
				ctrl_state.pitch_rate_bias = gyro_bias[1];
				ctrl_state.yaw_rate_bias = gyro_bias[2];

				// Velocity in body frame
				Vector3f v_n(velocity);
				matrix::Dcm<float> R_to_body(q.inversed());
				Vector3f v_b = R_to_body * v_n;
				ctrl_state.x_vel = v_b(0);
				ctrl_state.y_vel = v_b(1);
				ctrl_state.z_vel = v_b(2);


				// Local Position NED
				float position[3];
				_ekf.get_position(position);
				ctrl_state.x_pos = position[0];
				ctrl_state.y_pos = position[1];
				ctrl_state.z_pos = position[2];

				// Attitude quaternion
				q.copyTo(ctrl_state.q);

				_ekf.get_quat_reset(&ctrl_state.delta_q_reset[0], &ctrl_state.quat_reset_counter);

				// Acceleration data
				matrix::Vector<float, 3> acceleration(sensors.accelerometer_m_s2);

				float accel_bias[3];
				_ekf.get_accel_bias(accel_bias);
				ctrl_state.x_acc = acceleration(0) - accel_bias[0];
				ctrl_state.y_acc = acceleration(1) - accel_bias[1];
				ctrl_state.z_acc = acceleration(2) - accel_bias[2];

				// compute lowpass filtered horizontal acceleration
				acceleration = R_to_body.transpose() * acceleration;
				_acc_hor_filt = 0.95f * _acc_hor_filt + 0.05f * sqrtf(acceleration(0) * acceleration(0) +
						acceleration(1) * acceleration(1));
				ctrl_state.horz_acc_mag = _acc_hor_filt;

				ctrl_state.airspeed_valid = false;

				// use estimated velocity for airspeed estimate
				if (_airspeed_mode.get() == control_state_s::AIRSPD_MODE_MEAS) {
					// use measured airspeed
					if (PX4_ISFINITE(airspeed.indicated_airspeed_m_s) && now - airspeed.timestamp < 1e6
					    && airspeed.timestamp > 0) {
						ctrl_state.airspeed = airspeed.indicated_airspeed_m_s;
						ctrl_state.airspeed_valid = true;
					}

				} else if (_airspeed_mode.get() == control_state_s::AIRSPD_MODE_EST) {
					if (_ekf.local_position_is_valid()) {
						ctrl_state.airspeed = sqrtf(velocity[0] * velocity[0] + velocity[1] * velocity[1] + velocity[2] * velocity[2]);
						ctrl_state.airspeed_valid = true;
					}

				} else if (_airspeed_mode.get() == control_state_s::AIRSPD_MODE_DISABLED) {
					// do nothing, airspeed has been declared as non-valid above, controllers
					// will handle this assuming always trim airspeed
				}

				// publish control state data
				if (_control_state_pub == nullptr) {
					_control_state_pub = orb_advertise(ORB_ID(control_state), &ctrl_state);

				} else {
					orb_publish(ORB_ID(control_state), _control_state_pub, &ctrl_state);
				}
			}


			{
				// generate vehicle attitude quaternion data
				struct vehicle_attitude_s att = {};
				att.timestamp = now;

				q.copyTo(att.q);

				att.rollspeed = gyro_rad[0];
				att.pitchspeed = gyro_rad[1];
				att.yawspeed = gyro_rad[2];

				// publish vehicle attitude data
				if (_att_pub == nullptr) {
					_att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

				} else {
					orb_publish(ORB_ID(vehicle_attitude), _att_pub, &att);
				}
			}

			// generate vehicle local position data
			struct vehicle_local_position_s lpos = {};
			float pos[3] = {};

			lpos.timestamp = now;

			// Position of body origin in local NED frame
			_ekf.get_position(pos);
			lpos.x = (_ekf.local_position_is_valid()) ? pos[0] : 0.0f;
			lpos.y = (_ekf.local_position_is_valid()) ? pos[1] : 0.0f;
			lpos.z = pos[2];

			// Velocity of body origin in local NED frame (m/s)
			lpos.vx = velocity[0];
			lpos.vy = velocity[1];
			lpos.vz = velocity[2];
			lpos.z_deriv = pos_d_deriv; // vertical position time derivative (m/s)

			// TODO: better status reporting
			lpos.xy_valid = _ekf.local_position_is_valid() && !_vel_innov_preflt_fail;
			lpos.z_valid = !_vel_innov_preflt_fail;
			lpos.v_xy_valid = _ekf.local_position_is_valid() && !_vel_innov_preflt_fail;
			lpos.v_z_valid = !_vel_innov_preflt_fail;

			// Position of local NED origin in GPS / WGS84 frame
			struct map_projection_reference_s ekf_origin = {};
			// true if position (x,y,z) has a valid WGS-84 global reference (ref_lat, ref_lon, alt)
			lpos.xy_global = lpos.z_global = _ekf.get_ekf_origin(&lpos.ref_timestamp, &ekf_origin, &lpos.ref_alt);
			lpos.ref_lat = ekf_origin.lat_rad * 180.0 / M_PI; // Reference point latitude in degrees
			lpos.ref_lon = ekf_origin.lon_rad * 180.0 / M_PI; // Reference point longitude in degrees

			// The rotation of the tangent plane vs. geographical north
			matrix::Eulerf euler(q);
			lpos.yaw = euler.psi();

			float terrain_vpos;
			lpos.dist_bottom_valid = _ekf.get_terrain_valid();
			_ekf.get_terrain_vert_pos(&terrain_vpos);
			lpos.dist_bottom = terrain_vpos - pos[2]; // Distance to bottom surface (ground) in meters

			// constrain the distance to ground to _params->rng_gnd_clearance
			if (lpos.dist_bottom < _params->rng_gnd_clearance) {
				lpos.dist_bottom = _params->rng_gnd_clearance;
			}

			lpos.dist_bottom_rate = -velocity[2]; // Distance to bottom surface (ground) change rate
			lpos.surface_bottom_timestamp = now; // Time when new bottom surface found

			bool dead_reckoning;
			_ekf.get_ekf_lpos_accuracy(&lpos.eph, &lpos.epv, &dead_reckoning);
			_ekf.get_ekf_vel_accuracy(&lpos.evh, &lpos.evv, &dead_reckoning);

			// get state reset information of position and velocity
			_ekf.get_posD_reset(&lpos.delta_z, &lpos.z_reset_counter);
			_ekf.get_velD_reset(&lpos.delta_vz, &lpos.vz_reset_counter);
			_ekf.get_posNE_reset(&lpos.delta_xy[0], &lpos.xy_reset_counter);
			_ekf.get_velNE_reset(&lpos.delta_vxy[0], &lpos.vxy_reset_counter);

			// publish vehicle local position data
			if (_lpos_pub == nullptr) {
				_lpos_pub = orb_advertise(ORB_ID(vehicle_local_position), &lpos);

			} else {
				orb_publish(ORB_ID(vehicle_local_position), _lpos_pub, &lpos);
			}

			if (_ekf.global_position_is_valid() && !_vel_innov_preflt_fail) {
				// generate and publish global position data
				struct vehicle_global_position_s global_pos = {};

				global_pos.timestamp = now;
				global_pos.time_utc_usec = gps.time_utc_usec; // GPS UTC timestamp in microseconds

				double est_lat, est_lon, lat_pre_reset, lon_pre_reset;
				map_projection_reproject(&ekf_origin, lpos.x, lpos.y, &est_lat, &est_lon);
				global_pos.lat = est_lat; // Latitude in degrees
				global_pos.lon = est_lon; // Longitude in degrees
				map_projection_reproject(&ekf_origin, lpos.x - lpos.delta_xy[0], lpos.y - lpos.delta_xy[1], &lat_pre_reset,
							 &lon_pre_reset);
				global_pos.delta_lat_lon[0] = est_lat - lat_pre_reset;
				global_pos.delta_lat_lon[1] = est_lon - lon_pre_reset;
				global_pos.lat_lon_reset_counter = lpos.xy_reset_counter;

				global_pos.alt = -pos[2] + lpos.ref_alt; // Altitude AMSL in meters
				_ekf.get_posD_reset(&global_pos.delta_alt, &global_pos.alt_reset_counter);
				// global altitude has opposite sign of local down position
				global_pos.delta_alt *= -1.0f;

				global_pos.vel_n = velocity[0]; // Ground north velocity, m/s
				global_pos.vel_e = velocity[1]; // Ground east velocity, m/s
				global_pos.vel_d = velocity[2]; // Ground downside velocity, m/s
				global_pos.pos_d_deriv = pos_d_deriv; // vertical position time derivative, m/s

				global_pos.yaw = euler.psi(); // Yaw in radians -PI..+PI.

				_ekf.get_ekf_gpos_accuracy(&global_pos.eph, &global_pos.epv, &global_pos.dead_reckoning);
				global_pos.evh = lpos.evh;
				global_pos.evv = lpos.evv;

				if (lpos.dist_bottom_valid) {
					global_pos.terrain_alt = lpos.ref_alt - terrain_vpos; // Terrain altitude in m, WGS84
					global_pos.terrain_alt_valid = true; // Terrain altitude estimate is valid

				} else {
					global_pos.terrain_alt = 0.0f; // Terrain altitude in m, WGS84
					global_pos.terrain_alt_valid = false; // Terrain altitude estimate is valid
				}

				global_pos.dead_reckoning = _ekf.inertial_dead_reckoning(); // True if this position is estimated through dead-reckoning

				global_pos.pressure_alt = sensors.baro_alt_meter; // Pressure altitude AMSL (m)

				if (_vehicle_global_position_pub == nullptr) {
					_vehicle_global_position_pub = orb_advertise(ORB_ID(vehicle_global_position), &global_pos);

				} else {
					orb_publish(ORB_ID(vehicle_global_position), _vehicle_global_position_pub, &global_pos);
				}
			}

			// publish estimator status
			{
				struct estimator_status_s status = {};
				status.timestamp = now;
				_ekf.get_state_delayed(status.states);
				_ekf.get_covariances(status.covariances);
				_ekf.get_gps_check_status(&status.gps_check_fail_flags);
				_ekf.get_control_mode(&status.control_mode_flags);
				_ekf.get_filter_fault_status(&status.filter_fault_flags);
				_ekf.get_innovation_test_status(&status.innovation_check_flags, &status.mag_test_ratio,
								&status.vel_test_ratio, &status.pos_test_ratio,
								&status.hgt_test_ratio, &status.tas_test_ratio,
								&status.hagl_test_ratio);

				status.pos_horiz_accuracy = lpos.eph;
				status.pos_vert_accuracy = lpos.epv;
				_ekf.get_ekf_soln_status(&status.solution_status_flags);
				_ekf.get_imu_vibe_metrics(status.vibe);

				// monitor time slippage
				if (start_time_us != 0 && now > start_time_us) {
					status.time_slip = (float)(1e-6 * ((double)(now - start_time_us) - (double) integrated_time_us));
					_last_time_slip_us = (now - start_time_us) - integrated_time_us;

				} else {
					status.time_slip = 0.0f;
				}

				if (_estimator_status_pub == nullptr) {
					_estimator_status_pub = orb_advertise(ORB_ID(estimator_status), &status);

				} else {
					orb_publish(ORB_ID(estimator_status), _estimator_status_pub, &status);
				}

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
					memset(_valid_cal_available, false, sizeof(_valid_cal_available));
				}

				// Start checking mag bias estimates when we have accumulated sufficient calibration time
				if (_total_cal_time_us > 120 * 1000 * 1000ULL) {
					// we have sufficient accumulated valid flight time to form a reliable bias estimate
					// check that the state variance for each axis is within a range indicating filter convergence
					float max_var_allowed = 100.0f * _mag_bias_saved_variance.get();
					float min_var_allowed = 0.01f * _mag_bias_saved_variance.get();

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
					control::BlockParamFloat *mag_biases[] = { &_mag_bias_x, &_mag_bias_y, &_mag_bias_z };

					for (uint8_t axis_index = 0; axis_index <= 2; axis_index++) {
						if (_valid_cal_available[axis_index]) {
							// calculate weighting using ratio of variances and update stored bias values
							float weighting = _mag_bias_saved_variance.get() / (_mag_bias_saved_variance.get() +
									  _last_valid_variance[axis_index]);
							weighting = math::constrain(weighting, 0.0f, _mag_bias_alpha.get());
							float mag_bias_saved = mag_biases[axis_index]->get();
							_last_valid_mag_cal[axis_index] = weighting * _last_valid_mag_cal[axis_index] + mag_bias_saved;
							mag_biases[axis_index]->set(_last_valid_mag_cal[axis_index]);
							mag_biases[axis_index]->commit_no_notification();

							_valid_cal_available[axis_index] = false;
						}
					}

					// reset to prevent data being saved too frequently
					_total_cal_time_us = 0;
				}

				// Publish wind estimate
				struct wind_estimate_s wind_estimate = {};
				wind_estimate.timestamp = now;
				wind_estimate.windspeed_north = status.states[22];
				wind_estimate.windspeed_east = status.states[23];
				wind_estimate.covariance_north = status.covariances[22];
				wind_estimate.covariance_east = status.covariances[23];

				if (_wind_pub == nullptr) {
					_wind_pub = orb_advertise(ORB_ID(wind_estimate), &wind_estimate);

				} else {
					orb_publish(ORB_ID(wind_estimate), _wind_pub, &wind_estimate);
				}
			}

			// publish estimator innovation data
			{
				struct ekf2_innovations_s innovations = {};
				innovations.timestamp = now;
				_ekf.get_vel_pos_innov(&innovations.vel_pos_innov[0]);
				_ekf.get_mag_innov(&innovations.mag_innov[0]);
				_ekf.get_heading_innov(&innovations.heading_innov);
				_ekf.get_airspeed_innov(&innovations.airspeed_innov);
				_ekf.get_beta_innov(&innovations.beta_innov);
				_ekf.get_flow_innov(&innovations.flow_innov[0]);
				_ekf.get_hagl_innov(&innovations.hagl_innov);

				_ekf.get_vel_pos_innov_var(&innovations.vel_pos_innov_var[0]);
				_ekf.get_mag_innov_var(&innovations.mag_innov_var[0]);
				_ekf.get_heading_innov_var(&innovations.heading_innov_var);
				_ekf.get_airspeed_innov_var(&innovations.airspeed_innov_var);
				_ekf.get_beta_innov_var(&innovations.beta_innov_var);
				_ekf.get_flow_innov_var(&innovations.flow_innov_var[0]);
				_ekf.get_hagl_innov_var(&innovations.hagl_innov_var);

				_ekf.get_output_tracking_error(&innovations.output_tracking_error[0]);

				// calculate noise filtered velocity innovations which are used for pre-flight checking
				if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
					float alpha = math::constrain(sensors.accelerometer_integral_dt / 1.e6f * _innov_lpf_tau_inv, 0.0f, 1.0f);
					float beta = 1.0f - alpha;
					_vel_innov_lpf_ned(0) = beta * _vel_innov_lpf_ned(0) + alpha * math::constrain(innovations.vel_pos_innov[0],
								-_vel_innov_spike_lim, _vel_innov_spike_lim);
					_vel_innov_lpf_ned(1) = beta * _vel_innov_lpf_ned(1) + alpha * math::constrain(innovations.vel_pos_innov[1],
								-_vel_innov_spike_lim, _vel_innov_spike_lim);
					_vel_innov_lpf_ned(2) = beta * _vel_innov_lpf_ned(2) + alpha * math::constrain(innovations.vel_pos_innov[2],
								-_vel_innov_spike_lim, _vel_innov_spike_lim);
					_hgt_innov_lpf = beta * _hgt_innov_lpf + alpha * math::constrain(innovations.vel_pos_innov[5], -_hgt_innov_spike_lim,
							 _hgt_innov_spike_lim);
					_vel_innov_preflt_fail = ((_vel_innov_lpf_ned.norm() > _vel_innov_test_lim)
								  || (fabsf(_hgt_innov_lpf) > _hgt_innov_test_lim));

				} else {
					_vel_innov_lpf_ned.zero();
					_hgt_innov_lpf = 0.0f;
					_vel_innov_preflt_fail = false;
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
			struct vehicle_attitude_s att = {};
			att.timestamp = now;

			if (_att_pub == nullptr) {
				_att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

			} else {
				orb_publish(ORB_ID(vehicle_attitude), _att_pub, &att);
			}
		}

		// publish ekf2_timestamps (using 0.1 ms relative timestamps)
		{
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


		// publish replay message if in replay mode
		bool publish_replay_message = (bool)_param_record_replay_msg.get();

		if (publish_replay_message) {
			struct ekf2_replay_s replay = {};
			replay.timestamp = now;
			replay.gyro_integral_dt = sensors.gyro_integral_dt;
			replay.accelerometer_integral_dt = sensors.accelerometer_integral_dt;
			replay.magnetometer_timestamp = _timestamp_mag_us;
			replay.baro_timestamp = _timestamp_balt_us;
			memcpy(replay.gyro_rad, sensors.gyro_rad, sizeof(replay.gyro_rad));
			memcpy(replay.accelerometer_m_s2, sensors.accelerometer_m_s2, sizeof(replay.accelerometer_m_s2));
			memcpy(replay.magnetometer_ga, sensors.magnetometer_ga, sizeof(replay.magnetometer_ga));
			replay.baro_alt_meter = sensors.baro_alt_meter;

			// only write gps data if we had a gps update.
			if (gps_updated) {
				replay.time_usec = gps.timestamp;
				replay.lat = gps.lat;
				replay.lon = gps.lon;
				replay.alt = gps.alt;
				replay.fix_type = gps.fix_type;
				replay.nsats = gps.satellites_used;
				replay.eph = gps.eph;
				replay.epv = gps.epv;
				replay.sacc = gps.s_variance_m_s;
				replay.vel_m_s = gps.vel_m_s;
				replay.vel_n_m_s = gps.vel_n_m_s;
				replay.vel_e_m_s = gps.vel_e_m_s;
				replay.vel_d_m_s = gps.vel_d_m_s;
				replay.vel_ned_valid = gps.vel_ned_valid;

			} else {
				// this will tell the logging app not to bother logging any gps replay data
				replay.time_usec = 0;
			}

			if (optical_flow_updated) {
				replay.flow_timestamp = optical_flow.timestamp;
				replay.flow_pixel_integral[0] = optical_flow.pixel_flow_x_integral;
				replay.flow_pixel_integral[1] = optical_flow.pixel_flow_y_integral;
				replay.flow_gyro_integral[0] = optical_flow.gyro_x_rate_integral;
				replay.flow_gyro_integral[1] = optical_flow.gyro_y_rate_integral;
				replay.flow_time_integral = optical_flow.integration_timespan;
				replay.flow_quality = optical_flow.quality;

			} else {
				replay.flow_timestamp = 0;
			}

			if (range_finder_updated) {
				replay.rng_timestamp = range_finder.timestamp;
				replay.range_to_ground = range_finder.current_distance;

			} else {
				replay.rng_timestamp = 0;
			}

			if (airspeed_updated) {
				replay.asp_timestamp = airspeed.timestamp;
				replay.indicated_airspeed_m_s = airspeed.indicated_airspeed_m_s;
				replay.true_airspeed_m_s = airspeed.true_airspeed_m_s;

			} else {
				replay.asp_timestamp = 0;
			}

			if (vision_position_updated || vision_attitude_updated) {
				replay.ev_timestamp = vision_position_updated ? ev_pos.timestamp : ev_att.timestamp;
				replay.pos_ev[0] = ev_pos.x;
				replay.pos_ev[1] = ev_pos.y;
				replay.pos_ev[2] = ev_pos.z;
				replay.quat_ev[0] = ev_att.q[0];
				replay.quat_ev[1] = ev_att.q[1];
				replay.quat_ev[2] = ev_att.q[2];
				replay.quat_ev[3] = ev_att.q[3];
				// TODO : switch to covariances from topic later
				replay.pos_err = _default_ev_pos_noise;
				replay.ang_err = _default_ev_ang_noise;

			} else {
				replay.ev_timestamp = 0;
			}

			if (_replay_pub == nullptr) {
				_replay_pub = orb_advertise(ORB_ID(ekf2_replay), &replay);

			} else {
				orb_publish(ORB_ID(ekf2_replay), _replay_pub, &replay);
			}
		}
	}

	orb_unsubscribe(sensors_sub);
	orb_unsubscribe(gps_sub);
	orb_unsubscribe(airspeed_sub);
	orb_unsubscribe(params_sub);
	orb_unsubscribe(optical_flow_sub);
	orb_unsubscribe(range_finder_sub);
	orb_unsubscribe(ev_pos_sub);
	orb_unsubscribe(vehicle_land_detected_sub);
	orb_unsubscribe(status_sub);
	orb_unsubscribe(sensor_selection_sub);
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
				      SCHED_PRIORITY_MAX - 5,
				      5900,
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
