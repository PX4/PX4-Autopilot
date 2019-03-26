/****************************************************************************
 *
 *   Copyright (c) 2015-2018 PX4 Development Team. All rights reserved.
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

#include <float.h>

#include <drivers/drv_hrt.h>
#include <lib/ecl/EKF/ekf.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <px4_defines.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/ekf2_innovations.h>
#include <uORB/topics/ekf2_timestamps.h>
#include <uORB/topics/ekf_gps_position.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/ekf_gps_drift.h>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_bias.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wind_estimate.h>

// defines used to specify the mask position for use of different accuracy metrics in the GPS blending algorithm
#define BLEND_MASK_USE_SPD_ACC      1
#define BLEND_MASK_USE_HPOS_ACC     2
#define BLEND_MASK_USE_VPOS_ACC     4

// define max number of GPS receivers supported and 0 base instance used to access virtual 'blended' GPS solution
#define GPS_MAX_RECEIVERS 2
#define GPS_BLENDED_INSTANCE 2

using math::constrain;
using namespace time_literals;

extern "C" __EXPORT int ekf2_main(int argc, char *argv[]);

class Ekf2 final : public ModuleBase<Ekf2>, public ModuleParams
{
public:
	Ekf2();
	~Ekf2() override;

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

	template<typename Param>
	void update_mag_bias(Param &mag_bias_param, int axis_index);
	template<typename Param>
	bool update_mag_decl(Param &mag_decl_param);
	bool publish_attitude(const sensor_combined_s &sensors, const hrt_abstime &now);
	bool publish_wind_estimate(const hrt_abstime &timestamp);

	const Vector3f get_vel_body_wind();

	/*
	 * Update the internal state estimate for a blended GPS solution that is a weighted average of the phsyical
	 * receiver solutions. This internal state cannot be used directly by estimators because if physical receivers
	 * have significant position differences, variation in receiver estimated accuracy will cause undesirable
	 * variation in the position solution.
	*/
	bool blend_gps_data();

	/*
	 * Calculate internal states used to blend GPS data from multiple receivers using weightings calculated
	 * by calc_blend_weights()
	 * States are written to _gps_state and _gps_blended_state class variables
	 */
	void update_gps_blend_states();

	/*
	 * The location in _gps_blended_state will move around as the relative accuracy changes.
	 * To mitigate this effect a low-pass filtered offset from each GPS location to the blended location is
	 * calculated.
	*/
	void update_gps_offsets();

	/*
	 * Apply the steady state physical receiver offsets calculated by update_gps_offsets().
	*/
	void apply_gps_offsets();

	/*
	 Calculate GPS output that is a blend of the offset corrected physical receiver data
	*/
	void calc_gps_blend_output();

	/*
	 * Calculate filtered WGS84 height from estimated AMSL height
	 */
	float filter_altitude_ellipsoid(float amsl_hgt);

	bool 	_replay_mode = false;			///< true when we use replay data from a log

	// time slip monitoring
	uint64_t _integrated_time_us = 0;	///< integral of gyro delta time from start (uSec)
	uint64_t _start_time_us = 0;		///< system time at EKF start (uSec)
	int64_t _last_time_slip_us = 0;		///< Last time slip (uSec)

	perf_counter_t _perf_update_data;
	perf_counter_t _perf_ekf_update;

	// Initialise time stamps used to send sensor data to the EKF and for logging
	uint8_t _invalid_mag_id_count = 0;	///< number of times an invalid magnetomer device ID has been detected

	// Used to down sample magnetometer data
	float _mag_data_sum[3] = {};			///< summed magnetometer readings (Gauss)
	uint64_t _mag_time_sum_ms = 0;		///< summed magnetoemter time stamps (mSec)
	uint8_t _mag_sample_count = 0;		///< number of magnetometer measurements summed during downsampling
	int32_t _mag_time_ms_last_used = 0;	///< time stamp of the last averaged magnetometer measurement sent to the EKF (mSec)

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

	// Used to control saving of mag declination to be used on next startup
	bool _mag_decl_saved = false;	///< true when the magnetic declination has been saved

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

	// set pose/velocity as invalid if standard deviation is bigger than max_std_dev
	// TODO: the user should be allowed to set these values by a parameter
	static constexpr float ep_max_std_dev = 100.0f;	///< Maximum permissible standard deviation for estimated position
	static constexpr float eo_max_std_dev = 100.0f;	///< Maximum permissible standard deviation for estimated orientation
	//static constexpr float ev_max_std_dev = 100.0f;	///< Maximum permissible standard deviation for estimated velocity

	// GPS blending and switching
	gps_message _gps_state[GPS_MAX_RECEIVERS] {}; ///< internal state data for the physical GPS
	gps_message _gps_blended_state{};		///< internal state data for the blended GPS
	gps_message _gps_output[GPS_MAX_RECEIVERS + 1] {}; ///< output state data for the physical and blended GPS
	Vector2f _NE_pos_offset_m[GPS_MAX_RECEIVERS] = {}; ///< Filtered North,East position offset from GPS instance to blended solution in _output_state.location (m)
	float _hgt_offset_mm[GPS_MAX_RECEIVERS] = {};	///< Filtered height offset from GPS instance relative to blended solution in _output_state.location (mm)
	Vector3f _blended_antenna_offset = {};		///< blended antenna offset
	float _blend_weights[GPS_MAX_RECEIVERS] = {};	///< blend weight for each GPS. The blend weights must sum to 1.0 across all instances.
	uint64_t _time_prev_us[GPS_MAX_RECEIVERS] = {};	///< the previous value of time_us for that GPS instance - used to detect new data.
	uint8_t _gps_best_index = 0;			///< index of the physical receiver with the lowest reported error
	uint8_t _gps_select_index = 0;			///< 0 = GPS1, 1 = GPS2, 2 = blended
	uint8_t _gps_time_ref_index =
		0;		///< index of the receiver that is used as the timing reference for the blending update
	uint8_t _gps_oldest_index = 0;			///< index of the physical receiver with the oldest data
	uint8_t _gps_newest_index = 0;			///< index of the physical receiver with the newest data
	uint8_t _gps_slowest_index = 0;			///< index of the physical receiver with the slowest update rate
	float _gps_dt[GPS_MAX_RECEIVERS] = {};		///< average time step in seconds.
	bool  _gps_new_output_data = false;		///< true if there is new output data for the EKF

	int32_t _gps_alttitude_ellipsoid[GPS_MAX_RECEIVERS] {};	///< altitude in 1E-3 meters (millimeters) above ellipsoid
	uint64_t _gps_alttitude_ellipsoid_previous_timestamp[GPS_MAX_RECEIVERS] {}; ///< storage for previous timestamp to compute dt
	float   _wgs84_hgt_offset = 0;  ///< height offset between AMSL and WGS84

	int _airdata_sub{ -1};
	int _airspeed_sub{ -1};
	int _ev_odom_sub{ -1};
	int _landing_target_pose_sub{ -1};
	int _magnetometer_sub{ -1};
	int _optical_flow_sub{ -1};
	int _params_sub{ -1};
	int _sensor_selection_sub{ -1};
	int _sensors_sub{ -1};
	int _status_sub{ -1};
	int _vehicle_land_detected_sub{ -1};

	// because we can have several distance sensor instances with different orientations
	int _range_finder_subs[ORB_MULTI_MAX_INSTANCES] {};
	int _range_finder_sub_index = -1; // index for downward-facing range finder subscription

	// because we can have multiple GPS instances
	int _gps_subs[GPS_MAX_RECEIVERS] {};
	int _gps_orb_instance{ -1};

	orb_advert_t _att_pub{nullptr};
	orb_advert_t _wind_pub{nullptr};
	orb_advert_t _estimator_status_pub{nullptr};
	orb_advert_t _ekf_gps_drift_pub{nullptr};
	orb_advert_t _estimator_innovations_pub{nullptr};
	orb_advert_t _ekf2_timestamps_pub{nullptr};
	orb_advert_t _sensor_bias_pub{nullptr};
	orb_advert_t _blended_gps_pub{nullptr};

	uORB::Publication<vehicle_local_position_s> _vehicle_local_position_pub;
	uORB::Publication<vehicle_global_position_s> _vehicle_global_position_pub;
	uORB::Publication<vehicle_odometry_s> _vehicle_odometry_pub;

	Ekf _ekf;

	parameters *_params;	///< pointer to ekf parameter struct (located in _ekf class instance)

	DEFINE_PARAMETERS(
		(ParamExtInt<px4::params::EKF2_MIN_OBS_DT>)
		_param_ekf2_min_obs_dt,	///< Maximum time delay of any sensor used to increase buffer length to handle large timing jitter (mSec)
		(ParamExtFloat<px4::params::EKF2_MAG_DELAY>)
		_param_ekf2_mag_delay,	///< magnetometer measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_BARO_DELAY>)
		_param_ekf2_baro_delay,	///< barometer height measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_GPS_DELAY>)
		_param_ekf2_gps_delay,	///< GPS measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_OF_DELAY>)
		_param_ekf2_of_delay,	///< optical flow measurement delay relative to the IMU (mSec) - this is to the middle of the optical flow integration interval
		(ParamExtFloat<px4::params::EKF2_RNG_DELAY>)
		_param_ekf2_rng_delay,	///< range finder measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_ASP_DELAY>)
		_param_ekf2_asp_delay,	///< airspeed measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_EV_DELAY>)
		_param_ekf2_ev_delay,	///< off-board vision measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_AVEL_DELAY>)
		_param_ekf2_avel_delay,	///< auxillary velocity measurement delay relative to the IMU (mSec)

		(ParamExtFloat<px4::params::EKF2_GYR_NOISE>)
		_param_ekf2_gyr_noise,	///< IMU angular rate noise used for covariance prediction (rad/sec)
		(ParamExtFloat<px4::params::EKF2_ACC_NOISE>)
		_param_ekf2_acc_noise,	///< IMU acceleration noise use for covariance prediction (m/sec**2)

		// process noise
		(ParamExtFloat<px4::params::EKF2_GYR_B_NOISE>)
		_param_ekf2_gyr_b_noise,	///< process noise for IMU rate gyro bias prediction (rad/sec**2)
		(ParamExtFloat<px4::params::EKF2_ACC_B_NOISE>)
		_param_ekf2_acc_b_noise,///< process noise for IMU accelerometer bias prediction (m/sec**3)
		(ParamExtFloat<px4::params::EKF2_MAG_E_NOISE>)
		_param_ekf2_mag_e_noise,	///< process noise for earth magnetic field prediction (Gauss/sec)
		(ParamExtFloat<px4::params::EKF2_MAG_B_NOISE>)
		_param_ekf2_mag_b_noise,	///< process noise for body magnetic field prediction (Gauss/sec)
		(ParamExtFloat<px4::params::EKF2_WIND_NOISE>)
		_param_ekf2_wind_noise,	///< process noise for wind velocity prediction (m/sec**2)
		(ParamExtFloat<px4::params::EKF2_TERR_NOISE>) _param_ekf2_terr_noise,	///< process noise for terrain offset (m/sec)
		(ParamExtFloat<px4::params::EKF2_TERR_GRAD>)
		_param_ekf2_terr_grad,	///< gradient of terrain used to estimate process noise due to changing position (m/m)

		(ParamExtFloat<px4::params::EKF2_GPS_V_NOISE>)
		_param_ekf2_gps_v_noise,	///< minimum allowed observation noise for gps velocity fusion (m/sec)
		(ParamExtFloat<px4::params::EKF2_GPS_P_NOISE>)
		_param_ekf2_gps_p_noise,	///< minimum allowed observation noise for gps position fusion (m)
		(ParamExtFloat<px4::params::EKF2_NOAID_NOISE>)
		_param_ekf2_noaid_noise,	///< observation noise for non-aiding position fusion (m)
		(ParamExtFloat<px4::params::EKF2_BARO_NOISE>)
		_param_ekf2_baro_noise,	///< observation noise for barometric height fusion (m)
		(ParamExtFloat<px4::params::EKF2_BARO_GATE>)
		_param_ekf2_baro_gate,	///< barometric height innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_GND_EFF_DZ>)
		_param_ekf2_gnd_eff_dz,	///< barometric deadzone range for negative innovations (m)
		(ParamExtFloat<px4::params::EKF2_GND_MAX_HGT>)
		_param_ekf2_gnd_max_hgt,	///< maximum height above the ground level for expected negative baro innovations (m)
		(ParamExtFloat<px4::params::EKF2_GPS_P_GATE>)
		_param_ekf2_gps_p_gate,	///< GPS horizontal position innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_GPS_V_GATE>)
		_param_ekf2_gps_v_gate,	///< GPS velocity innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_TAS_GATE>)
		_param_ekf2_tas_gate,	///< True Airspeed innovation consistency gate size (STD)

		// control of magnetometer fusion
		(ParamExtFloat<px4::params::EKF2_HEAD_NOISE>)
		_param_ekf2_head_noise,	///< measurement noise used for simple heading fusion (rad)
		(ParamExtFloat<px4::params::EKF2_MAG_NOISE>)
		_param_ekf2_mag_noise,		///< measurement noise used for 3-axis magnetoemeter fusion (Gauss)
		(ParamExtFloat<px4::params::EKF2_EAS_NOISE>)
		_param_ekf2_eas_noise,		///< measurement noise used for airspeed fusion (m/sec)
		(ParamExtFloat<px4::params::EKF2_BETA_GATE>)
		_param_ekf2_beta_gate, ///< synthetic sideslip innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_BETA_NOISE>) _param_ekf2_beta_noise,	///< synthetic sideslip noise (rad)
		(ParamExtFloat<px4::params::EKF2_MAG_DECL>) _param_ekf2_mag_decl,///< magnetic declination (degrees)
		(ParamExtFloat<px4::params::EKF2_HDG_GATE>)
		_param_ekf2_hdg_gate,///< heading fusion innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_MAG_GATE>)
		_param_ekf2_mag_gate,	///< magnetometer fusion innovation consistency gate size (STD)
		(ParamExtInt<px4::params::EKF2_DECL_TYPE>)
		_param_ekf2_decl_type,	///< bitmask used to control the handling of declination data
		(ParamExtInt<px4::params::EKF2_MAG_TYPE>)
		_param_ekf2_mag_type,	///< integer used to specify the type of magnetometer fusion used
		(ParamExtFloat<px4::params::EKF2_MAG_ACCLIM>)
		_param_ekf2_mag_acclim,	///< integer used to specify the type of magnetometer fusion used
		(ParamExtFloat<px4::params::EKF2_MAG_YAWLIM>)
		_param_ekf2_mag_yawlim,	///< yaw rate threshold used by mode select logic (rad/sec)

		(ParamExtInt<px4::params::EKF2_GPS_CHECK>)
		_param_ekf2_gps_check,	///< bitmask used to control which GPS quality checks are used
		(ParamExtFloat<px4::params::EKF2_REQ_EPH>) _param_ekf2_req_eph,	///< maximum acceptable horiz position error (m)
		(ParamExtFloat<px4::params::EKF2_REQ_EPV>) _param_ekf2_req_epv,	///< maximum acceptable vert position error (m)
		(ParamExtFloat<px4::params::EKF2_REQ_SACC>) _param_ekf2_req_sacc,	///< maximum acceptable speed error (m/s)
		(ParamExtInt<px4::params::EKF2_REQ_NSATS>) _param_ekf2_req_nsats,	///< minimum acceptable satellite count
		(ParamExtFloat<px4::params::EKF2_REQ_GDOP>)
		_param_ekf2_req_gdop,	///< maximum acceptable geometric dilution of precision
		(ParamExtFloat<px4::params::EKF2_REQ_HDRIFT>)
		_param_ekf2_req_hdrift,	///< maximum acceptable horizontal drift speed (m/s)
		(ParamExtFloat<px4::params::EKF2_REQ_VDRIFT>) _param_ekf2_req_vdrift,	///< maximum acceptable vertical drift speed (m/s)

		// measurement source control
		(ParamExtInt<px4::params::EKF2_AID_MASK>)
		_param_ekf2_aid_mask,		///< bitmasked integer that selects which of the GPS and optical flow aiding sources will be used
		(ParamExtInt<px4::params::EKF2_HGT_MODE>) _param_ekf2_hgt_mode,	///< selects the primary source for height data
		(ParamExtInt<px4::params::EKF2_NOAID_TOUT>)
		_param_ekf2_noaid_tout,	///< maximum lapsed time from last fusion of measurements that constrain drift before the EKF will report the horizontal nav solution invalid (uSec)

		// range finder fusion
		(ParamExtFloat<px4::params::EKF2_RNG_NOISE>)
		_param_ekf2_rng_noise,	///< observation noise for range finder measurements (m)
		(ParamExtFloat<px4::params::EKF2_RNG_SFE>) _param_ekf2_rng_sfe, ///< scale factor from range to range noise (m/m)
		(ParamExtFloat<px4::params::EKF2_RNG_GATE>)
		_param_ekf2_rng_gate,	///< range finder fusion innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_MIN_RNG>) _param_ekf2_min_rng,	///< minimum valid value for range when on ground (m)
		(ParamExtFloat<px4::params::EKF2_RNG_PITCH>) _param_ekf2_rng_pitch,	///< range sensor pitch offset (rad)
		(ParamExtInt<px4::params::EKF2_RNG_AID>)
		_param_ekf2_rng_aid,		///< enables use of a range finder even if primary height source is not range finder
		(ParamExtFloat<px4::params::EKF2_RNG_A_VMAX>)
		_param_ekf2_rng_a_vmax,	///< maximum allowed horizontal velocity for range aid (m/s)
		(ParamExtFloat<px4::params::EKF2_RNG_A_HMAX>)
		_param_ekf2_rng_a_hmax,	///< maximum allowed absolute altitude (AGL) for range aid (m)
		(ParamExtFloat<px4::params::EKF2_RNG_A_IGATE>)
		_param_ekf2_rng_a_igate,	///< gate size used for innovation consistency checks for range aid fusion (STD)

		// vision estimate fusion
		(ParamFloat<px4::params::EKF2_EVP_NOISE>)
		_param_ekf2_evp_noise,	///< default position observation noise for exernal vision measurements (m)
		(ParamFloat<px4::params::EKF2_EVA_NOISE>)
		_param_ekf2_eva_noise,	///< default angular observation noise for exernal vision measurements (rad)
		(ParamExtFloat<px4::params::EKF2_EV_GATE>)
		_param_ekf2_ev_gate,	///< external vision position innovation consistency gate size (STD)

		// optical flow fusion
		(ParamExtFloat<px4::params::EKF2_OF_N_MIN>)
		_param_ekf2_of_n_min,	///< best quality observation noise for optical flow LOS rate measurements (rad/sec)
		(ParamExtFloat<px4::params::EKF2_OF_N_MAX>)
		_param_ekf2_of_n_max,	///< worst quality observation noise for optical flow LOS rate measurements (rad/sec)
		(ParamExtInt<px4::params::EKF2_OF_QMIN>)
		_param_ekf2_of_qmin,	///< minimum acceptable quality integer from  the flow sensor
		(ParamExtFloat<px4::params::EKF2_OF_GATE>)
		_param_ekf2_of_gate,	///< optical flow fusion innovation consistency gate size (STD)

		// sensor positions in body frame
		(ParamExtFloat<px4::params::EKF2_IMU_POS_X>) _param_ekf2_imu_pos_x,		///< X position of IMU in body frame (m)
		(ParamExtFloat<px4::params::EKF2_IMU_POS_Y>) _param_ekf2_imu_pos_y,		///< Y position of IMU in body frame (m)
		(ParamExtFloat<px4::params::EKF2_IMU_POS_Z>) _param_ekf2_imu_pos_z,		///< Z position of IMU in body frame (m)
		(ParamExtFloat<px4::params::EKF2_GPS_POS_X>) _param_ekf2_gps_pos_x,		///< X position of GPS antenna in body frame (m)
		(ParamExtFloat<px4::params::EKF2_GPS_POS_Y>) _param_ekf2_gps_pos_y,		///< Y position of GPS antenna in body frame (m)
		(ParamExtFloat<px4::params::EKF2_GPS_POS_Z>) _param_ekf2_gps_pos_z,		///< Z position of GPS antenna in body frame (m)
		(ParamExtFloat<px4::params::EKF2_RNG_POS_X>) _param_ekf2_rng_pos_x,		///< X position of range finder in body frame (m)
		(ParamExtFloat<px4::params::EKF2_RNG_POS_Y>) _param_ekf2_rng_pos_y,		///< Y position of range finder in body frame (m)
		(ParamExtFloat<px4::params::EKF2_RNG_POS_Z>) _param_ekf2_rng_pos_z,		///< Z position of range finder in body frame (m)
		(ParamExtFloat<px4::params::EKF2_OF_POS_X>)
		_param_ekf2_of_pos_x,	///< X position of optical flow sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_OF_POS_Y>)
		_param_ekf2_of_pos_y,	///< Y position of optical flow sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_OF_POS_Z>)
		_param_ekf2_of_pos_z,	///< Z position of optical flow sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_EV_POS_X>)
		_param_ekf2_ev_pos_x,		///< X position of VI sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_EV_POS_Y>)
		_param_ekf2_ev_pos_y,		///< Y position of VI sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_EV_POS_Z>)
		_param_ekf2_ev_pos_z,		///< Z position of VI sensor focal point in body frame (m)

		// control of airspeed and sideslip fusion
		(ParamFloat<px4::params::EKF2_ARSP_THR>)
		_param_ekf2_arsp_thr, 	///< A value of zero will disabled airspeed fusion. Any positive value sets the minimum airspeed which will be used (m/sec)
		(ParamInt<px4::params::EKF2_FUSE_BETA>)
		_param_ekf2_fuse_beta,		///< Controls synthetic sideslip fusion, 0 disables, 1 enables

		// output predictor filter time constants
		(ParamExtFloat<px4::params::EKF2_TAU_VEL>)
		_param_ekf2_tau_vel,		///< time constant used by the output velocity complementary filter (sec)
		(ParamExtFloat<px4::params::EKF2_TAU_POS>)
		_param_ekf2_tau_pos,		///< time constant used by the output position complementary filter (sec)

		// IMU switch on bias parameters
		(ParamExtFloat<px4::params::EKF2_GBIAS_INIT>)
		_param_ekf2_gbias_init,	///< 1-sigma gyro bias uncertainty at switch on (rad/sec)
		(ParamExtFloat<px4::params::EKF2_ABIAS_INIT>)
		_param_ekf2_abias_init,	///< 1-sigma accelerometer bias uncertainty at switch on (m/sec**2)
		(ParamExtFloat<px4::params::EKF2_ANGERR_INIT>)
		_param_ekf2_angerr_init,	///< 1-sigma tilt error after initial alignment using gravity vector (rad)

		// EKF saved XYZ magnetometer bias values
		(ParamFloat<px4::params::EKF2_MAGBIAS_X>) _param_ekf2_magbias_x,		///< X magnetometer bias (mGauss)
		(ParamFloat<px4::params::EKF2_MAGBIAS_Y>) _param_ekf2_magbias_y,		///< Y magnetometer bias (mGauss)
		(ParamFloat<px4::params::EKF2_MAGBIAS_Z>) _param_ekf2_magbias_z,		///< Z magnetometer bias (mGauss)
		(ParamInt<px4::params::EKF2_MAGBIAS_ID>)
		_param_ekf2_magbias_id,		///< ID of the magnetometer sensor used to learn the bias values
		(ParamFloat<px4::params::EKF2_MAGB_VREF>)
		_param_ekf2_magb_vref, ///< Assumed error variance of previously saved magnetometer bias estimates (mGauss**2)
		(ParamFloat<px4::params::EKF2_MAGB_K>)
		_param_ekf2_magb_k,	///< maximum fraction of the learned magnetometer bias that is saved at each disarm

		// EKF accel bias learning control
		(ParamExtFloat<px4::params::EKF2_ABL_LIM>) _param_ekf2_abl_lim,	///< Accelerometer bias learning limit (m/s**2)
		(ParamExtFloat<px4::params::EKF2_ABL_ACCLIM>)
		_param_ekf2_abl_acclim,	///< Maximum IMU accel magnitude that allows IMU bias learning (m/s**2)
		(ParamExtFloat<px4::params::EKF2_ABL_GYRLIM>)
		_param_ekf2_abl_gyrlim,	///< Maximum IMU gyro angular rate magnitude that allows IMU bias learning (m/s**2)
		(ParamExtFloat<px4::params::EKF2_ABL_TAU>)
		_param_ekf2_abl_tau,	///< Time constant used to inhibit IMU delta velocity bias learning (sec)

		// Multi-rotor drag specific force fusion
		(ParamExtFloat<px4::params::EKF2_DRAG_NOISE>)
		_param_ekf2_drag_noise,	///< observation noise variance for drag specific force measurements (m/sec**2)**2
		(ParamExtFloat<px4::params::EKF2_BCOEF_X>) _param_ekf2_bcoef_x,		///< ballistic coefficient along the X-axis (kg/m**2)
		(ParamExtFloat<px4::params::EKF2_BCOEF_Y>) _param_ekf2_bcoef_y,		///< ballistic coefficient along the Y-axis (kg/m**2)

		// Corrections for static pressure position error where Ps_error = Ps_meas - Ps_truth
		// Coef = Ps_error / Pdynamic, where Pdynamic = 1/2 * density * TAS**2
		(ParamFloat<px4::params::EKF2_ASPD_MAX>)
		_param_ekf2_aspd_max,		///< upper limit on airspeed used for correction  (m/s**2)
		(ParamFloat<px4::params::EKF2_PCOEF_XP>)
		_param_ekf2_pcoef_xp,	///< static pressure position error coefficient along the positive X body axis
		(ParamFloat<px4::params::EKF2_PCOEF_XN>)
		_param_ekf2_pcoef_xn,	///< static pressure position error coefficient along the negative X body axis
		(ParamFloat<px4::params::EKF2_PCOEF_YP>)
		_param_ekf2_pcoef_yp,	///< static pressure position error coefficient along the positive Y body axis
		(ParamFloat<px4::params::EKF2_PCOEF_YN>)
		_param_ekf2_pcoef_yn,	///< static pressure position error coefficient along the negative Y body axis
		(ParamFloat<px4::params::EKF2_PCOEF_Z>)
		_param_ekf2_pcoef_z,	///< static pressure position error coefficient along the Z body axis

		// GPS blending
		(ParamInt<px4::params::EKF2_GPS_MASK>)
		_param_ekf2_gps_mask,	///< mask defining when GPS accuracy metrics are used to calculate the blend ratio
		(ParamFloat<px4::params::EKF2_GPS_TAU>)
		_param_ekf2_gps_tau,		///< time constant controlling how rapidly the offset used to bring GPS solutions together is allowed to change (sec)

		// Test used to determine if the vehicle is static or moving
		(ParamExtFloat<px4::params::EKF2_MOVE_TEST>)
		_param_ekf2_move_test	///< scaling applied to IMU data thresholds used to determine if the vehicle is static or moving.

	)

};

Ekf2::Ekf2():
	ModuleParams(nullptr),
	_perf_update_data(perf_alloc_once(PC_ELAPSED, "EKF2 data acquisition")),
	_perf_ekf_update(perf_alloc_once(PC_ELAPSED, "EKF2 update")),
	_vehicle_local_position_pub(ORB_ID(vehicle_local_position)),
	_vehicle_global_position_pub(ORB_ID(vehicle_global_position)),
	_vehicle_odometry_pub(ORB_ID(vehicle_odometry)),
	_params(_ekf.getParamHandle()),
	_param_ekf2_min_obs_dt(_params->sensor_interval_min_ms),
	_param_ekf2_mag_delay(_params->mag_delay_ms),
	_param_ekf2_baro_delay(_params->baro_delay_ms),
	_param_ekf2_gps_delay(_params->gps_delay_ms),
	_param_ekf2_of_delay(_params->flow_delay_ms),
	_param_ekf2_rng_delay(_params->range_delay_ms),
	_param_ekf2_asp_delay(_params->airspeed_delay_ms),
	_param_ekf2_ev_delay(_params->ev_delay_ms),
	_param_ekf2_avel_delay(_params->auxvel_delay_ms),
	_param_ekf2_gyr_noise(_params->gyro_noise),
	_param_ekf2_acc_noise(_params->accel_noise),
	_param_ekf2_gyr_b_noise(_params->gyro_bias_p_noise),
	_param_ekf2_acc_b_noise(_params->accel_bias_p_noise),
	_param_ekf2_mag_e_noise(_params->mage_p_noise),
	_param_ekf2_mag_b_noise(_params->magb_p_noise),
	_param_ekf2_wind_noise(_params->wind_vel_p_noise),
	_param_ekf2_terr_noise(_params->terrain_p_noise),
	_param_ekf2_terr_grad(_params->terrain_gradient),
	_param_ekf2_gps_v_noise(_params->gps_vel_noise),
	_param_ekf2_gps_p_noise(_params->gps_pos_noise),
	_param_ekf2_noaid_noise(_params->pos_noaid_noise),
	_param_ekf2_baro_noise(_params->baro_noise),
	_param_ekf2_baro_gate(_params->baro_innov_gate),
	_param_ekf2_gnd_eff_dz(_params->gnd_effect_deadzone),
	_param_ekf2_gnd_max_hgt(_params->gnd_effect_max_hgt),
	_param_ekf2_gps_p_gate(_params->posNE_innov_gate),
	_param_ekf2_gps_v_gate(_params->vel_innov_gate),
	_param_ekf2_tas_gate(_params->tas_innov_gate),
	_param_ekf2_head_noise(_params->mag_heading_noise),
	_param_ekf2_mag_noise(_params->mag_noise),
	_param_ekf2_eas_noise(_params->eas_noise),
	_param_ekf2_beta_gate(_params->beta_innov_gate),
	_param_ekf2_beta_noise(_params->beta_noise),
	_param_ekf2_mag_decl(_params->mag_declination_deg),
	_param_ekf2_hdg_gate(_params->heading_innov_gate),
	_param_ekf2_mag_gate(_params->mag_innov_gate),
	_param_ekf2_decl_type(_params->mag_declination_source),
	_param_ekf2_mag_type(_params->mag_fusion_type),
	_param_ekf2_mag_acclim(_params->mag_acc_gate),
	_param_ekf2_mag_yawlim(_params->mag_yaw_rate_gate),
	_param_ekf2_gps_check(_params->gps_check_mask),
	_param_ekf2_req_eph(_params->req_hacc),
	_param_ekf2_req_epv(_params->req_vacc),
	_param_ekf2_req_sacc(_params->req_sacc),
	_param_ekf2_req_nsats(_params->req_nsats),
	_param_ekf2_req_gdop(_params->req_gdop),
	_param_ekf2_req_hdrift(_params->req_hdrift),
	_param_ekf2_req_vdrift(_params->req_vdrift),
	_param_ekf2_aid_mask(_params->fusion_mode),
	_param_ekf2_hgt_mode(_params->vdist_sensor_type),
	_param_ekf2_noaid_tout(_params->valid_timeout_max),
	_param_ekf2_rng_noise(_params->range_noise),
	_param_ekf2_rng_sfe(_params->range_noise_scaler),
	_param_ekf2_rng_gate(_params->range_innov_gate),
	_param_ekf2_min_rng(_params->rng_gnd_clearance),
	_param_ekf2_rng_pitch(_params->rng_sens_pitch),
	_param_ekf2_rng_aid(_params->range_aid),
	_param_ekf2_rng_a_vmax(_params->max_vel_for_range_aid),
	_param_ekf2_rng_a_hmax(_params->max_hagl_for_range_aid),
	_param_ekf2_rng_a_igate(_params->range_aid_innov_gate),
	_param_ekf2_ev_gate(_params->ev_innov_gate),
	_param_ekf2_of_n_min(_params->flow_noise),
	_param_ekf2_of_n_max(_params->flow_noise_qual_min),
	_param_ekf2_of_qmin(_params->flow_qual_min),
	_param_ekf2_of_gate(_params->flow_innov_gate),
	_param_ekf2_imu_pos_x(_params->imu_pos_body(0)),
	_param_ekf2_imu_pos_y(_params->imu_pos_body(1)),
	_param_ekf2_imu_pos_z(_params->imu_pos_body(2)),
	_param_ekf2_gps_pos_x(_params->gps_pos_body(0)),
	_param_ekf2_gps_pos_y(_params->gps_pos_body(1)),
	_param_ekf2_gps_pos_z(_params->gps_pos_body(2)),
	_param_ekf2_rng_pos_x(_params->rng_pos_body(0)),
	_param_ekf2_rng_pos_y(_params->rng_pos_body(1)),
	_param_ekf2_rng_pos_z(_params->rng_pos_body(2)),
	_param_ekf2_of_pos_x(_params->flow_pos_body(0)),
	_param_ekf2_of_pos_y(_params->flow_pos_body(1)),
	_param_ekf2_of_pos_z(_params->flow_pos_body(2)),
	_param_ekf2_ev_pos_x(_params->ev_pos_body(0)),
	_param_ekf2_ev_pos_y(_params->ev_pos_body(1)),
	_param_ekf2_ev_pos_z(_params->ev_pos_body(2)),
	_param_ekf2_tau_vel(_params->vel_Tau),
	_param_ekf2_tau_pos(_params->pos_Tau),
	_param_ekf2_gbias_init(_params->switch_on_gyro_bias),
	_param_ekf2_abias_init(_params->switch_on_accel_bias),
	_param_ekf2_angerr_init(_params->initial_tilt_err),
	_param_ekf2_abl_lim(_params->acc_bias_lim),
	_param_ekf2_abl_acclim(_params->acc_bias_learn_acc_lim),
	_param_ekf2_abl_gyrlim(_params->acc_bias_learn_gyr_lim),
	_param_ekf2_abl_tau(_params->acc_bias_learn_tc),
	_param_ekf2_drag_noise(_params->drag_noise),
	_param_ekf2_bcoef_x(_params->bcoef_x),
	_param_ekf2_bcoef_y(_params->bcoef_y),
	_param_ekf2_move_test(_params->is_moving_scaler)
{
	_airdata_sub = orb_subscribe(ORB_ID(vehicle_air_data));
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	_ev_odom_sub = orb_subscribe(ORB_ID(vehicle_visual_odometry));
	_landing_target_pose_sub = orb_subscribe(ORB_ID(landing_target_pose));
	_magnetometer_sub = orb_subscribe(ORB_ID(vehicle_magnetometer));
	_optical_flow_sub = orb_subscribe(ORB_ID(optical_flow));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_sensor_selection_sub = orb_subscribe(ORB_ID(sensor_selection));
	_sensors_sub = orb_subscribe(ORB_ID(sensor_combined));
	_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));

	for (unsigned i = 0; i < GPS_MAX_RECEIVERS; i++) {
		_gps_subs[i] = orb_subscribe_multi(ORB_ID(vehicle_gps_position), i);
	}

	for (unsigned i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		_range_finder_subs[i] = orb_subscribe_multi(ORB_ID(distance_sensor), i);
	}

	// initialise parameter cache
	updateParams();
}

Ekf2::~Ekf2()
{
	perf_free(_perf_update_data);
	perf_free(_perf_ekf_update);

	orb_unsubscribe(_airdata_sub);
	orb_unsubscribe(_airspeed_sub);
	orb_unsubscribe(_ev_odom_sub);
	orb_unsubscribe(_landing_target_pose_sub);
	orb_unsubscribe(_magnetometer_sub);
	orb_unsubscribe(_optical_flow_sub);
	orb_unsubscribe(_params_sub);
	orb_unsubscribe(_sensor_selection_sub);
	orb_unsubscribe(_sensors_sub);
	orb_unsubscribe(_status_sub);
	orb_unsubscribe(_vehicle_land_detected_sub);

	for (unsigned i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		orb_unsubscribe(_range_finder_subs[i]);
	}

	for (unsigned i = 0; i < GPS_MAX_RECEIVERS; i++) {
		orb_unsubscribe(_gps_subs[i]);
	}
}

int Ekf2::print_status()
{
	PX4_INFO("local position: %s", (_ekf.local_position_is_valid()) ? "valid" : "invalid");
	PX4_INFO("global position: %s", (_ekf.global_position_is_valid()) ? "valid" : "invalid");

	PX4_INFO("time slip: %" PRId64 " us", _last_time_slip_us);

	perf_print_counter(_perf_update_data);
	perf_print_counter(_perf_ekf_update);

	return 0;
}

template<typename Param>
void Ekf2::update_mag_bias(Param &mag_bias_param, int axis_index)
{
	if (_valid_cal_available[axis_index]) {

		// calculate weighting using ratio of variances and update stored bias values
		const float weighting = constrain(_param_ekf2_magb_vref.get() / (_param_ekf2_magb_vref.get() +
						  _last_valid_variance[axis_index]), 0.0f, _param_ekf2_magb_k.get());
		const float mag_bias_saved = mag_bias_param.get();

		_last_valid_mag_cal[axis_index] = weighting * _last_valid_mag_cal[axis_index] + mag_bias_saved;

		mag_bias_param.set(_last_valid_mag_cal[axis_index]);
		mag_bias_param.commit_no_notification();

		_valid_cal_available[axis_index] = false;
	}
}

template<typename Param>
bool Ekf2::update_mag_decl(Param &mag_decl_param)
{
	// update stored declination value
	float declination_deg;

	if (_ekf.get_mag_decl_deg(&declination_deg)) {
		mag_decl_param.set(declination_deg);
		mag_decl_param.commit_no_notification();
		return true;
	}

	return false;
}

void Ekf2::run()
{
	bool imu_bias_reset_request = false;

	px4_pollfd_struct_t fds[1] = {};
	fds[0].fd = _sensors_sub;
	fds[0].events = POLLIN;

	// initialize data structures outside of loop
	// because they will else not always be
	// properly populated
	sensor_combined_s sensors = {};
	vehicle_land_detected_s vehicle_land_detected = {};
	vehicle_status_s vehicle_status = {};
	sensor_selection_s sensor_selection = {};

	while (!should_exit()) {
		int ret = px4_poll(fds, sizeof(fds) / sizeof(fds[0]), 1000);

		if (!(fds[0].revents & POLLIN)) {
			// no new data
			continue;
		}

		if (ret < 0) {
			// Poll error, sleep and try again
			px4_usleep(10000);
			continue;

		} else if (ret == 0) {
			// Poll timeout or no new data, do nothing
			continue;
		}

		perf_begin(_perf_update_data);

		bool params_updated = false;
		orb_check(_params_sub, &params_updated);

		if (params_updated) {
			// read from param to clear updated flag
			parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);
			updateParams();
		}

		orb_copy(ORB_ID(sensor_combined), _sensors_sub, &sensors);

		// ekf2_timestamps (using 0.1 ms relative timestamps)
		ekf2_timestamps_s ekf2_timestamps = {};
		ekf2_timestamps.timestamp = sensors.timestamp;

		ekf2_timestamps.airspeed_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
		ekf2_timestamps.distance_sensor_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
		ekf2_timestamps.gps_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
		ekf2_timestamps.optical_flow_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
		ekf2_timestamps.vehicle_air_data_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
		ekf2_timestamps.vehicle_magnetometer_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
		ekf2_timestamps.visual_odometry_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;

		// update all other topics if they have new data

		bool vehicle_status_updated = false;

		orb_check(_status_sub, &vehicle_status_updated);

		if (vehicle_status_updated) {
			if (orb_copy(ORB_ID(vehicle_status), _status_sub, &vehicle_status) == PX4_OK) {
				// only fuse synthetic sideslip measurements if conditions are met
				_ekf.set_fuse_beta_flag(!vehicle_status.is_rotary_wing && (_param_ekf2_fuse_beta.get() == 1));

				// let the EKF know if the vehicle motion is that of a fixed wing (forward flight only relative to wind)
				_ekf.set_is_fixed_wing(!vehicle_status.is_rotary_wing);
			}
		}

		bool sensor_selection_updated = false;

		orb_check(_sensor_selection_sub, &sensor_selection_updated);

		// Always update sensor selction first time through if time stamp is non zero
		if (sensor_selection_updated || (sensor_selection.timestamp == 0)) {
			sensor_selection_s sensor_selection_prev = sensor_selection;

			if (orb_copy(ORB_ID(sensor_selection), _sensor_selection_sub, &sensor_selection) == PX4_OK) {
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
		}

		// attempt reset until successful
		if (imu_bias_reset_request) {
			imu_bias_reset_request = !_ekf.reset_imu_bias();
		}

		const hrt_abstime now = sensors.timestamp;

		// push imu data into estimator
		imuSample imu_sample_new;
		imu_sample_new.time_us = now;
		imu_sample_new.delta_ang_dt = sensors.gyro_integral_dt * 1.e-6f;
		imu_sample_new.delta_ang = Vector3f{sensors.gyro_rad} * imu_sample_new.delta_ang_dt;
		imu_sample_new.delta_vel_dt = sensors.accelerometer_integral_dt * 1.e-6f;
		imu_sample_new.delta_vel = Vector3f{sensors.accelerometer_m_s2} * imu_sample_new.delta_vel_dt;

		_ekf.setIMUData(imu_sample_new);

		// publish attitude immediately (uses quaternion from output predictor)
		publish_attitude(sensors, now);

		// read mag data
		bool magnetometer_updated = false;
		orb_check(_magnetometer_sub, &magnetometer_updated);

		if (magnetometer_updated) {
			vehicle_magnetometer_s magnetometer;

			if (orb_copy(ORB_ID(vehicle_magnetometer), _magnetometer_sub, &magnetometer) == PX4_OK) {
				// Reset learned bias parameters if there has been a persistant change in magnetometer ID
				// Do not reset parmameters when armed to prevent potential time slips casued by parameter set
				// and notification events
				// Check if there has been a persistant change in magnetometer ID
				if (sensor_selection.mag_device_id != 0 && sensor_selection.mag_device_id != (uint32_t)_param_ekf2_magbias_id.get()) {
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
					_param_ekf2_magbias_x.set(0.f);
					_param_ekf2_magbias_x.commit_no_notification();
					_param_ekf2_magbias_y.set(0.f);
					_param_ekf2_magbias_y.commit_no_notification();
					_param_ekf2_magbias_z.set(0.f);
					_param_ekf2_magbias_z.commit_no_notification();
					_param_ekf2_magbias_id.set(sensor_selection.mag_device_id);
					_param_ekf2_magbias_id.commit();

					_invalid_mag_id_count = 0;

					PX4_INFO("Mag sensor ID changed to %i", _param_ekf2_magbias_id.get());
				}

				// If the time last used by the EKF is less than specified, then accumulate the
				// data and push the average when the specified interval is reached.
				_mag_time_sum_ms += magnetometer.timestamp / 1000;
				_mag_sample_count++;
				_mag_data_sum[0] += magnetometer.magnetometer_ga[0];
				_mag_data_sum[1] += magnetometer.magnetometer_ga[1];
				_mag_data_sum[2] += magnetometer.magnetometer_ga[2];
				int32_t mag_time_ms = _mag_time_sum_ms / _mag_sample_count;

				if ((mag_time_ms - _mag_time_ms_last_used) > _params->sensor_interval_min_ms) {
					const float mag_sample_count_inv = 1.0f / _mag_sample_count;
					// calculate mean of measurements and correct for learned bias offsets
					float mag_data_avg_ga[3] = {_mag_data_sum[0] *mag_sample_count_inv - _param_ekf2_magbias_x.get(),
								    _mag_data_sum[1] *mag_sample_count_inv - _param_ekf2_magbias_y.get(),
								    _mag_data_sum[2] *mag_sample_count_inv - _param_ekf2_magbias_z.get()
								   };

					_ekf.setMagData(1000 * (uint64_t)mag_time_ms, mag_data_avg_ga);

					_mag_time_ms_last_used = mag_time_ms;
					_mag_time_sum_ms = 0;
					_mag_sample_count = 0;
					_mag_data_sum[0] = 0.0f;
					_mag_data_sum[1] = 0.0f;
					_mag_data_sum[2] = 0.0f;
				}

				ekf2_timestamps.vehicle_magnetometer_timestamp_rel = (int16_t)((int64_t)magnetometer.timestamp / 100 -
						(int64_t)ekf2_timestamps.timestamp / 100);
			}
		}

		// read baro data
		bool airdata_updated = false;
		orb_check(_airdata_sub, &airdata_updated);

		if (airdata_updated) {
			vehicle_air_data_s airdata;

			if (orb_copy(ORB_ID(vehicle_air_data), _airdata_sub, &airdata) == PX4_OK) {
				// If the time last used by the EKF is less than specified, then accumulate the
				// data and push the average when the specified interval is reached.
				_balt_time_sum_ms += airdata.timestamp / 1000;
				_balt_sample_count++;
				_balt_data_sum += airdata.baro_alt_meter;
				uint32_t balt_time_ms = _balt_time_sum_ms / _balt_sample_count;

				if (balt_time_ms - _balt_time_ms_last_used > (uint32_t)_params->sensor_interval_min_ms) {
					// take mean across sample period
					float balt_data_avg = _balt_data_sum / (float)_balt_sample_count;

					_ekf.set_air_density(airdata.rho);

					// calculate static pressure error = Pmeas - Ptruth
					// model position error sensitivity as a body fixed ellipse with a different scale in the positive and
					// negative X and Y directions
					const Vector3f vel_body_wind = get_vel_body_wind();

					float K_pstatic_coef_x;

					if (vel_body_wind(0) >= 0.0f) {
						K_pstatic_coef_x = _param_ekf2_pcoef_xp.get();

					} else {
						K_pstatic_coef_x = _param_ekf2_pcoef_xn.get();
					}

					float K_pstatic_coef_y;

					if (vel_body_wind(1) >= 0.0f) {
						K_pstatic_coef_y = _param_ekf2_pcoef_yp.get();

					} else {
						K_pstatic_coef_y = _param_ekf2_pcoef_yn.get();
					}

					const float max_airspeed_sq = _param_ekf2_aspd_max.get() * _param_ekf2_aspd_max.get();
					const float x_v2 = fminf(vel_body_wind(0) * vel_body_wind(0), max_airspeed_sq);
					const float y_v2 = fminf(vel_body_wind(1) * vel_body_wind(1), max_airspeed_sq);
					const float z_v2 = fminf(vel_body_wind(2) * vel_body_wind(2), max_airspeed_sq);

					const float pstatic_err = 0.5f * airdata.rho *
								  (K_pstatic_coef_x * x_v2) + (K_pstatic_coef_y * y_v2) + (_param_ekf2_pcoef_z.get() * z_v2);

					// correct baro measurement using pressure error estimate and assuming sea level gravity
					balt_data_avg += pstatic_err / (airdata.rho * CONSTANTS_ONE_G);

					// push to estimator
					_ekf.setBaroData(1000 * (uint64_t)balt_time_ms, balt_data_avg);

					_balt_time_ms_last_used = balt_time_ms;
					_balt_time_sum_ms = 0;
					_balt_sample_count = 0;
					_balt_data_sum = 0.0f;
				}

				ekf2_timestamps.vehicle_air_data_timestamp_rel = (int16_t)((int64_t)airdata.timestamp / 100 -
						(int64_t)ekf2_timestamps.timestamp / 100);
			}
		}

		// read gps1 data if available
		bool gps1_updated = false;
		orb_check(_gps_subs[0], &gps1_updated);

		if (gps1_updated) {
			vehicle_gps_position_s gps;

			if (orb_copy(ORB_ID(vehicle_gps_position), _gps_subs[0], &gps) == PX4_OK) {
				_gps_state[0].time_usec = gps.timestamp;
				_gps_state[0].lat = gps.lat;
				_gps_state[0].lon = gps.lon;
				_gps_state[0].alt = gps.alt;
				_gps_state[0].yaw = gps.heading;
				_gps_state[0].yaw_offset = gps.heading_offset;
				_gps_state[0].fix_type = gps.fix_type;
				_gps_state[0].eph = gps.eph;
				_gps_state[0].epv = gps.epv;
				_gps_state[0].sacc = gps.s_variance_m_s;
				_gps_state[0].vel_m_s = gps.vel_m_s;
				_gps_state[0].vel_ned[0] = gps.vel_n_m_s;
				_gps_state[0].vel_ned[1] = gps.vel_e_m_s;
				_gps_state[0].vel_ned[2] = gps.vel_d_m_s;
				_gps_state[0].vel_ned_valid = gps.vel_ned_valid;
				_gps_state[0].nsats = gps.satellites_used;
				//TODO: add gdop to gps topic
				_gps_state[0].gdop = 0.0f;
				_gps_alttitude_ellipsoid[0] = gps.alt_ellipsoid;

				ekf2_timestamps.gps_timestamp_rel = (int16_t)((int64_t)gps.timestamp / 100 - (int64_t)ekf2_timestamps.timestamp / 100);
			}
		}

		// check for second GPS receiver data
		bool gps2_updated = false;
		orb_check(_gps_subs[1], &gps2_updated);

		if (gps2_updated) {
			vehicle_gps_position_s gps;

			if (orb_copy(ORB_ID(vehicle_gps_position), _gps_subs[1], &gps) == PX4_OK) {
				_gps_state[1].time_usec = gps.timestamp;
				_gps_state[1].lat = gps.lat;
				_gps_state[1].lon = gps.lon;
				_gps_state[1].alt = gps.alt;
				_gps_state[1].yaw = gps.heading;
				_gps_state[1].yaw_offset = gps.heading_offset;
				_gps_state[1].fix_type = gps.fix_type;
				_gps_state[1].eph = gps.eph;
				_gps_state[1].epv = gps.epv;
				_gps_state[1].sacc = gps.s_variance_m_s;
				_gps_state[1].vel_m_s = gps.vel_m_s;
				_gps_state[1].vel_ned[0] = gps.vel_n_m_s;
				_gps_state[1].vel_ned[1] = gps.vel_e_m_s;
				_gps_state[1].vel_ned[2] = gps.vel_d_m_s;
				_gps_state[1].vel_ned_valid = gps.vel_ned_valid;
				_gps_state[1].nsats = gps.satellites_used;
				//TODO: add gdop to gps topic
				_gps_state[1].gdop = 0.0f;
				_gps_alttitude_ellipsoid[1] = gps.alt_ellipsoid;
			}
		}

		if ((_param_ekf2_gps_mask.get() == 0) && gps1_updated) {
			// When GPS blending is disabled we always use the first receiver instance
			_ekf.setGpsData(_gps_state[0].time_usec, _gps_state[0]);

		} else if ((_param_ekf2_gps_mask.get() > 0) && (gps1_updated || gps2_updated)) {
			// blend dual receivers if available

			// calculate blending weights
			if (!blend_gps_data()) {
				// handle case where the blended states cannot be updated
				if (_gps_state[0].fix_type > _gps_state[1].fix_type) {
					// GPS 1 has the best fix status so use that
					_gps_select_index = 0;

				} else if (_gps_state[1].fix_type > _gps_state[0].fix_type) {
					// GPS 2 has the best fix status so use that
					_gps_select_index = 1;

				} else if (_gps_select_index == 2) {
					// use last receiver we received data from
					if (gps1_updated) {
						_gps_select_index = 0;

					} else if (gps2_updated) {
						_gps_select_index = 1;
					}
				}

				// Only use selected receiver data if it has been updated
				if ((gps1_updated && _gps_select_index == 0) || (gps2_updated && _gps_select_index == 1)) {
					_gps_new_output_data = true;

				} else {
					_gps_new_output_data = false;
				}
			}

			if (_gps_new_output_data) {
				// correct the _gps_state data for steady state offsets and write to _gps_output
				apply_gps_offsets();

				// calculate a blended output from the offset corrected receiver data
				if (_gps_select_index == 2) {
					calc_gps_blend_output();
				}

				// write selected GPS to EKF
				_ekf.setGpsData(_gps_output[_gps_select_index].time_usec, _gps_output[_gps_select_index]);

				// log blended solution as a third GPS instance
				ekf_gps_position_s gps;
				gps.timestamp = _gps_output[_gps_select_index].time_usec;
				gps.lat = _gps_output[_gps_select_index].lat;
				gps.lon = _gps_output[_gps_select_index].lon;
				gps.alt = _gps_output[_gps_select_index].alt;
				gps.fix_type = _gps_output[_gps_select_index].fix_type;
				gps.eph = _gps_output[_gps_select_index].eph;
				gps.epv = _gps_output[_gps_select_index].epv;
				gps.s_variance_m_s = _gps_output[_gps_select_index].sacc;
				gps.vel_m_s = _gps_output[_gps_select_index].vel_m_s;
				gps.vel_n_m_s = _gps_output[_gps_select_index].vel_ned[0];
				gps.vel_e_m_s = _gps_output[_gps_select_index].vel_ned[1];
				gps.vel_d_m_s = _gps_output[_gps_select_index].vel_ned[2];
				gps.vel_ned_valid = _gps_output[_gps_select_index].vel_ned_valid;
				gps.satellites_used = _gps_output[_gps_select_index].nsats;
				gps.heading = _gps_output[_gps_select_index].yaw;
				gps.heading_offset = _gps_output[_gps_select_index].yaw_offset;
				gps.selected = _gps_select_index;

				// Publish to the EKF blended GPS topic
				orb_publish_auto(ORB_ID(ekf_gps_position), &_blended_gps_pub, &gps, &_gps_orb_instance, ORB_PRIO_LOW);

				// clear flag to avoid re-use of the same data
				_gps_new_output_data = false;
			}
		}

		bool airspeed_updated = false;
		orb_check(_airspeed_sub, &airspeed_updated);

		if (airspeed_updated) {
			airspeed_s airspeed;

			if (orb_copy(ORB_ID(airspeed), _airspeed_sub, &airspeed) == PX4_OK) {
				// only set airspeed data if condition for airspeed fusion are met
				if ((_param_ekf2_arsp_thr.get() > FLT_EPSILON) && (airspeed.true_airspeed_m_s > _param_ekf2_arsp_thr.get())) {

					const float eas2tas = airspeed.true_airspeed_m_s / airspeed.indicated_airspeed_m_s;
					_ekf.setAirspeedData(airspeed.timestamp, airspeed.true_airspeed_m_s, eas2tas);
				}

				ekf2_timestamps.airspeed_timestamp_rel = (int16_t)((int64_t)airspeed.timestamp / 100 -
						(int64_t)ekf2_timestamps.timestamp / 100);
			}
		}

		bool optical_flow_updated = false;

		orb_check(_optical_flow_sub, &optical_flow_updated);

		if (optical_flow_updated) {
			optical_flow_s optical_flow;

			if (orb_copy(ORB_ID(optical_flow), _optical_flow_sub, &optical_flow) == PX4_OK) {
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

				// Save sensor limits reported by the optical flow sensor
				_ekf.set_optical_flow_limits(optical_flow.max_flow_rate, optical_flow.min_ground_distance,
							     optical_flow.max_ground_distance);

				ekf2_timestamps.optical_flow_timestamp_rel = (int16_t)((int64_t)optical_flow.timestamp / 100 -
						(int64_t)ekf2_timestamps.timestamp / 100);
			}
		}

		if (_range_finder_sub_index >= 0) {
			bool range_finder_updated = false;

			orb_check(_range_finder_subs[_range_finder_sub_index], &range_finder_updated);

			if (range_finder_updated) {
				distance_sensor_s range_finder;

				if (orb_copy(ORB_ID(distance_sensor), _range_finder_subs[_range_finder_sub_index], &range_finder) == PX4_OK) {
					// check distance sensor data quality
					// TODO - move this check inside the ecl library
					if (range_finder.signal_quality == 0) {
						// use rng_gnd_clearance if on ground
						if (_ekf.get_in_air_status()) {
							range_finder_updated = false;

						} else {
							range_finder.current_distance = _param_ekf2_min_rng.get();
						}
					}

					if (range_finder_updated) { _ekf.setRangeData(range_finder.timestamp, range_finder.current_distance); }

					// Save sensor limits reported by the rangefinder
					_ekf.set_rangefinder_limits(range_finder.min_distance, range_finder.max_distance);

					ekf2_timestamps.distance_sensor_timestamp_rel = (int16_t)((int64_t)range_finder.timestamp / 100 -
							(int64_t)ekf2_timestamps.timestamp / 100);
				}
			}

		} else {
			_range_finder_sub_index = getRangeSubIndex(_range_finder_subs);
		}

		// get external vision data
		// if error estimates are unavailable, use parameter defined defaults
		bool visual_odometry_updated = false;
		orb_check(_ev_odom_sub, &visual_odometry_updated);

		if (visual_odometry_updated) {
			// copy both attitude & position, we need both to fill a single ext_vision_message
			vehicle_odometry_s ev_odom;
			orb_copy(ORB_ID(vehicle_visual_odometry), _ev_odom_sub, &ev_odom);

			ext_vision_message ev_data;

			// check for valid position data
			if (PX4_ISFINITE(ev_odom.x) && PX4_ISFINITE(ev_odom.y) && PX4_ISFINITE(ev_odom.z)) {
				ev_data.posNED(0) = ev_odom.x;
				ev_data.posNED(1) = ev_odom.y;
				ev_data.posNED(2) = ev_odom.z;

				// position measurement error from parameters
				if (PX4_ISFINITE(ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_X_VARIANCE])) {
					ev_data.posErr = fmaxf(_param_ekf2_evp_noise.get(),
							       sqrtf(fmaxf(ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_X_VARIANCE],
									   ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_Y_VARIANCE])));
					ev_data.hgtErr = fmaxf(_param_ekf2_evp_noise.get(),
							       sqrtf(ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_Z_VARIANCE]));

				} else {
					ev_data.posErr = _param_ekf2_evp_noise.get();
					ev_data.hgtErr = _param_ekf2_evp_noise.get();
				}
			}

			// check for valid orientation data
			if (PX4_ISFINITE(ev_odom.q[0])) {
				ev_data.quat = matrix::Quatf(ev_odom.q);

				// orientation measurement error from parameters
				if (PX4_ISFINITE(ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_ROLL_VARIANCE])) {
					ev_data.angErr = fmaxf(_param_ekf2_eva_noise.get(),
							       sqrtf(fmaxf(ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_ROLL_VARIANCE],
									   fmaxf(ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_PITCH_VARIANCE],
											   ev_odom.pose_covariance[ev_odom.COVARIANCE_MATRIX_YAW_VARIANCE]))));

				} else {
					ev_data.angErr = _param_ekf2_eva_noise.get();
				}
			}

			// only set data if all positions and orientation are valid
			if (ev_data.posErr < ep_max_std_dev && ev_data.angErr < eo_max_std_dev) {
				// use timestamp from external computer, clocks are synchronized when using MAVROS
				_ekf.setExtVisionData(ev_odom.timestamp, &ev_data);
			}

			ekf2_timestamps.visual_odometry_timestamp_rel = (int16_t)((int64_t)ev_odom.timestamp / 100 -
					(int64_t)ekf2_timestamps.timestamp / 100);
		}

		bool vehicle_land_detected_updated = false;
		orb_check(_vehicle_land_detected_sub, &vehicle_land_detected_updated);

		if (vehicle_land_detected_updated) {
			if (orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &vehicle_land_detected) == PX4_OK) {
				_ekf.set_in_air_status(!vehicle_land_detected.landed);
			}
		}

		// use the landing target pose estimate as another source of velocity data
		bool landing_target_pose_updated = false;
		orb_check(_landing_target_pose_sub, &landing_target_pose_updated);

		if (landing_target_pose_updated) {
			landing_target_pose_s landing_target_pose;

			if (orb_copy(ORB_ID(landing_target_pose), _landing_target_pose_sub, &landing_target_pose) == PX4_OK) {
				// we can only use the landing target if it has a fixed position and  a valid velocity estimate
				if (landing_target_pose.is_static && landing_target_pose.rel_vel_valid) {
					// velocity of vehicle relative to target has opposite sign to target relative to vehicle
					float velocity[2] = { -landing_target_pose.vx_rel, -landing_target_pose.vy_rel};
					float variance[2] = {landing_target_pose.cov_vx_rel, landing_target_pose.cov_vy_rel};
					_ekf.setAuxVelData(landing_target_pose.timestamp, velocity, variance);
				}
			}
		}

		perf_end(_perf_update_data);

		// run the EKF update and output
		perf_begin(_perf_ekf_update);
		const bool updated = _ekf.update();
		perf_end(_perf_ekf_update);

		// integrate time to monitor time slippage
		if (_start_time_us == 0) {
			_start_time_us = now;
			_last_time_slip_us = 0;

		} else if (_start_time_us > 0) {
			_integrated_time_us += sensors.gyro_integral_dt;
			_last_time_slip_us = (now - _start_time_us) - _integrated_time_us;
		}

		if (updated) {

			filter_control_status_u control_status;
			_ekf.get_control_mode(&control_status.value);

			// only publish position after successful alignment
			if (control_status.flags.tilt_align) {
				// generate vehicle local position data
				vehicle_local_position_s &lpos = _vehicle_local_position_pub.get();

				// generate vehicle odometry data
				vehicle_odometry_s &odom = _vehicle_odometry_pub.get();

				lpos.timestamp = now;
				odom.timestamp = lpos.timestamp;

				odom.local_frame = odom.LOCAL_FRAME_NED;

				// Position of body origin in local NED frame
				float position[3];
				_ekf.get_position(position);
				const float lpos_x_prev = lpos.x;
				const float lpos_y_prev = lpos.y;
				lpos.x = (_ekf.local_position_is_valid()) ? position[0] : 0.0f;
				lpos.y = (_ekf.local_position_is_valid()) ? position[1] : 0.0f;
				lpos.z = position[2];

				// Vehicle odometry position
				odom.x = lpos.x;
				odom.y = lpos.y;
				odom.z = lpos.z;

				// Velocity of body origin in local NED frame (m/s)
				float velocity[3];
				_ekf.get_velocity(velocity);
				lpos.vx = velocity[0];
				lpos.vy = velocity[1];
				lpos.vz = velocity[2];

				// Vehicle odometry linear velocity
				odom.vx = lpos.vx;
				odom.vy = lpos.vy;
				odom.vz = lpos.vz;

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
				matrix::Quatf q;
				_ekf.copy_quaternion(q.data());

				lpos.yaw = matrix::Eulerf(q).psi();

				// Vehicle odometry quaternion
				q.copyTo(odom.q);

				// Vehicle odometry angular rates
				float gyro_bias[3];
				_ekf.get_gyro_bias(gyro_bias);
				odom.rollspeed = sensors.gyro_rad[0] - gyro_bias[0];
				odom.pitchspeed = sensors.gyro_rad[1] - gyro_bias[1];
				odom.yawspeed = sensors.gyro_rad[2] - gyro_bias[2];

				lpos.dist_bottom_valid = _ekf.get_terrain_valid();

				float terrain_vpos;
				_ekf.get_terrain_vert_pos(&terrain_vpos);
				lpos.dist_bottom = terrain_vpos - lpos.z; // Distance to bottom surface (ground) in meters

				// constrain the distance to ground to _rng_gnd_clearance
				if (lpos.dist_bottom < _param_ekf2_min_rng.get()) {
					lpos.dist_bottom = _param_ekf2_min_rng.get();
				}

				// update ground effect flag based on terrain estimation
				if (lpos.dist_bottom_valid && lpos.dist_bottom < _param_ekf2_gnd_max_hgt.get()) {
					_ekf.set_gnd_effect_flag(true);
				}

				// update ground effect flag based on land detector state
				else if (vehicle_land_detected_updated && _param_ekf2_gnd_eff_dz.get() > 0.0f) {
					_ekf.set_gnd_effect_flag(vehicle_land_detected.in_ground_effect);
				}

				lpos.dist_bottom_rate = -lpos.vz; // Distance to bottom surface (ground) change rate

				_ekf.get_ekf_lpos_accuracy(&lpos.eph, &lpos.epv);
				_ekf.get_ekf_vel_accuracy(&lpos.evh, &lpos.evv);

				// get state reset information of position and velocity
				_ekf.get_posD_reset(&lpos.delta_z, &lpos.z_reset_counter);
				_ekf.get_velD_reset(&lpos.delta_vz, &lpos.vz_reset_counter);
				_ekf.get_posNE_reset(&lpos.delta_xy[0], &lpos.xy_reset_counter);
				_ekf.get_velNE_reset(&lpos.delta_vxy[0], &lpos.vxy_reset_counter);

				// get control limit information
				_ekf.get_ekf_ctrl_limits(&lpos.vxy_max, &lpos.vz_max, &lpos.hagl_min, &lpos.hagl_max);

				// convert NaN to INFINITY
				if (!PX4_ISFINITE(lpos.vxy_max)) {
					lpos.vxy_max = INFINITY;
				}

				if (!PX4_ISFINITE(lpos.vz_max)) {
					lpos.vz_max = INFINITY;
				}

				if (!PX4_ISFINITE(lpos.hagl_min)) {
					lpos.hagl_min = INFINITY;
				}

				if (!PX4_ISFINITE(lpos.hagl_max)) {
					lpos.hagl_max = INFINITY;
				}

				// Get covariances to vehicle odometry
				float covariances[24];
				_ekf.covariances_diagonal().copyTo(covariances);

				// get the covariance matrix size
				const size_t POS_URT_SIZE = sizeof(odom.pose_covariance) / sizeof(odom.pose_covariance[0]);
				const size_t VEL_URT_SIZE = sizeof(odom.velocity_covariance) / sizeof(odom.velocity_covariance[0]);

				// initially set pose covariances to 0
				for (size_t i = 0; i < POS_URT_SIZE; i++) {
					odom.pose_covariance[i] = 0.0;
				}

				// set the position variances
				odom.pose_covariance[odom.COVARIANCE_MATRIX_X_VARIANCE] = covariances[7];
				odom.pose_covariance[odom.COVARIANCE_MATRIX_Y_VARIANCE] = covariances[8];
				odom.pose_covariance[odom.COVARIANCE_MATRIX_Z_VARIANCE] = covariances[9];

				// TODO: implement propagation from quaternion covariance to Euler angle covariance
				// by employing the covariance law

				// initially set velocity covariances to 0
				for (size_t i = 0; i < VEL_URT_SIZE; i++) {
					odom.velocity_covariance[i] = 0.0;
				}

				// set the linear velocity variances
				odom.velocity_covariance[odom.COVARIANCE_MATRIX_VX_VARIANCE] = covariances[4];
				odom.velocity_covariance[odom.COVARIANCE_MATRIX_VY_VARIANCE] = covariances[5];
				odom.velocity_covariance[odom.COVARIANCE_MATRIX_VZ_VARIANCE] = covariances[6];

				// publish vehicle local position data
				_vehicle_local_position_pub.update();

				// publish vehicle odometry data
				_vehicle_odometry_pub.update();

				if (_ekf.global_position_is_valid() && !_preflt_fail) {
					// generate and publish global position data
					vehicle_global_position_s &global_pos = _vehicle_global_position_pub.get();

					global_pos.timestamp = now;

					if (fabsf(lpos_x_prev - lpos.x) > FLT_EPSILON || fabsf(lpos_y_prev - lpos.y) > FLT_EPSILON) {
						map_projection_reproject(&ekf_origin, lpos.x, lpos.y, &global_pos.lat, &global_pos.lon);
					}

					global_pos.lat_lon_reset_counter = lpos.xy_reset_counter;

					global_pos.alt = -lpos.z + lpos.ref_alt; // Altitude AMSL in meters
					global_pos.alt_ellipsoid = filter_altitude_ellipsoid(global_pos.alt);

					// global altitude has opposite sign of local down position
					global_pos.delta_alt = -lpos.delta_z;

					global_pos.vel_n = lpos.vx; // Ground north velocity, m/s
					global_pos.vel_e = lpos.vy; // Ground east velocity, m/s
					global_pos.vel_d = lpos.vz; // Ground downside velocity, m/s

					global_pos.yaw = lpos.yaw; // Yaw in radians -PI..+PI.

					_ekf.get_ekf_gpos_accuracy(&global_pos.eph, &global_pos.epv);

					global_pos.terrain_alt_valid = lpos.dist_bottom_valid;

					if (global_pos.terrain_alt_valid) {
						global_pos.terrain_alt = lpos.ref_alt - terrain_vpos; // Terrain altitude in m, WGS84

					} else {
						global_pos.terrain_alt = 0.0f; // Terrain altitude in m, WGS84
					}

					global_pos.dead_reckoning = _ekf.inertial_dead_reckoning(); // True if this position is estimated through dead-reckoning

					_vehicle_global_position_pub.update();
				}
			}

			{
				// publish all corrected sensor readings and bias estimates after mag calibration is updated above
				sensor_bias_s bias;

				bias.timestamp = now;

				// In-run bias estimates
				float gyro_bias[3];
				_ekf.get_gyro_bias(gyro_bias);
				bias.gyro_x_bias = gyro_bias[0];
				bias.gyro_y_bias = gyro_bias[1];
				bias.gyro_z_bias = gyro_bias[2];

				float accel_bias[3];
				_ekf.get_accel_bias(accel_bias);
				bias.accel_x_bias = accel_bias[0];
				bias.accel_y_bias = accel_bias[1];
				bias.accel_z_bias = accel_bias[2];

				bias.mag_x_bias = _last_valid_mag_cal[0];
				bias.mag_y_bias = _last_valid_mag_cal[1];
				bias.mag_z_bias = _last_valid_mag_cal[2];

				// TODO: remove from sensor_bias?
				bias.accel_x = sensors.accelerometer_m_s2[0] - accel_bias[0];
				bias.accel_y = sensors.accelerometer_m_s2[1] - accel_bias[1];
				bias.accel_z = sensors.accelerometer_m_s2[2] - accel_bias[2];

				if (_sensor_bias_pub == nullptr) {
					_sensor_bias_pub = orb_advertise(ORB_ID(sensor_bias), &bias);

				} else {
					orb_publish(ORB_ID(sensor_bias), _sensor_bias_pub, &bias);
				}
			}

			// publish estimator status
			estimator_status_s status;
			status.timestamp = now;
			_ekf.get_state_delayed(status.states);
			status.n_states = 24;
			_ekf.covariances_diagonal().copyTo(status.covariances);
			_ekf.get_gps_check_status(&status.gps_check_fail_flags);
			// only report enabled GPS check failures (the param indexes are shifted by 1 bit, because they don't include
			// the GPS Fix bit, which is always checked)
			status.gps_check_fail_flags &= ((uint16_t)_params->gps_check_mask << 1) | 1;
			status.control_mode_flags = control_status.value;
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
			status.health_flags = 0.0f; // unused
			status.timeout_flags = 0.0f; // unused
			status.pre_flt_fail = _preflt_fail;

			if (_estimator_status_pub == nullptr) {
				_estimator_status_pub = orb_advertise(ORB_ID(estimator_status), &status);

			} else {
				orb_publish(ORB_ID(estimator_status), _estimator_status_pub, &status);
			}

			// publish GPS drift data only when updated to minimise overhead
			float gps_drift[3];
			bool blocked;

			if (_ekf.get_gps_drift_metrics(gps_drift, &blocked)) {
				ekf_gps_drift_s drift_data;
				drift_data.timestamp = now;
				drift_data.hpos_drift_rate = gps_drift[0];
				drift_data.vpos_drift_rate = gps_drift[1];
				drift_data.hspd = gps_drift[2];
				drift_data.blocked = blocked;

				if (_ekf_gps_drift_pub == nullptr) {
					_ekf_gps_drift_pub = orb_advertise(ORB_ID(ekf_gps_drift), &drift_data);

				} else {
					orb_publish(ORB_ID(ekf_gps_drift), _ekf_gps_drift_pub, &drift_data);
				}
			}

			{
				/* Check and save learned magnetometer bias estimates */

				// Check if conditions are OK for learning of magnetometer bias values
				if (!vehicle_land_detected.landed && // not on ground
				    (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) && // vehicle is armed
				    !status.filter_fault_flags && // there are no filter faults
				    control_status.flags.mag_3D) { // the EKF is operating in the correct mode

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
				if (_total_cal_time_us > 120_s) {
					// we have sufficient accumulated valid flight time to form a reliable bias estimate
					// check that the state variance for each axis is within a range indicating filter convergence
					const float max_var_allowed = 100.0f * _param_ekf2_magb_vref.get();
					const float min_var_allowed = 0.01f * _param_ekf2_magb_vref.get();

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
				    && (sensor_selection.mag_device_id == (uint32_t)_param_ekf2_magbias_id.get())) {

					update_mag_bias(_param_ekf2_magbias_x, 0);
					update_mag_bias(_param_ekf2_magbias_y, 1);
					update_mag_bias(_param_ekf2_magbias_z, 2);

					// reset to prevent data being saved too frequently
					_total_cal_time_us = 0;
				}

			}

			publish_wind_estimate(now);

			if (!_mag_decl_saved && (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY)) {
				_mag_decl_saved = update_mag_decl(_param_ekf2_mag_decl);
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
					bool doing_ne_aiding = control_status.flags.gps ||  control_status.flags.ev_pos;

					float yaw_test_limit;

					if (doing_ne_aiding && vehicle_status.is_rotary_wing) {
						// use a smaller tolerance when doing NE inertial frame aiding as a rotary wing
						// vehicle which cannot use GPS course to realign heading in flight
						yaw_test_limit = _nav_yaw_innov_test_lim;

					} else {
						// use a larger tolerance when not doing NE inertial frame aiding or
						// if a fixed wing vehicle which can realign heading using GPS course
						yaw_test_limit = _yaw_innov_test_lim;
					}

					// filter the yaw innovations
					_yaw_innov_magnitude_lpf = beta * _yaw_innov_magnitude_lpf + alpha * constrain(innovations.heading_innov,
								   -2.0f * yaw_test_limit, 2.0f * yaw_test_limit);

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

		}

		// publish ekf2_timestamps
		if (_ekf2_timestamps_pub == nullptr) {
			_ekf2_timestamps_pub = orb_advertise(ORB_ID(ekf2_timestamps), &ekf2_timestamps);

		} else {
			orb_publish(ORB_ID(ekf2_timestamps), _ekf2_timestamps_pub, &ekf2_timestamps);
		}
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

bool Ekf2::publish_attitude(const sensor_combined_s &sensors, const hrt_abstime &now)
{
	if (_ekf.attitude_valid()) {
		// generate vehicle attitude quaternion data
		vehicle_attitude_s att;
		att.timestamp = now;

		const Quatf q{_ekf.calculate_quaternion()};
		q.copyTo(att.q);

		_ekf.get_quat_reset(&att.delta_q_reset[0], &att.quat_reset_counter);

		// In-run bias estimates
		float gyro_bias[3];
		_ekf.get_gyro_bias(gyro_bias);
		att.rollspeed = sensors.gyro_rad[0] - gyro_bias[0];
		att.pitchspeed = sensors.gyro_rad[1] - gyro_bias[1];
		att.yawspeed = sensors.gyro_rad[2] - gyro_bias[2];

		int instance;
		orb_publish_auto(ORB_ID(vehicle_attitude), &_att_pub, &att, &instance, ORB_PRIO_HIGH);

		return true;

	}  else if (_replay_mode) {
		// in replay mode we have to tell the replay module not to wait for an update
		// we do this by publishing an attitude with zero timestamp
		vehicle_attitude_s att = {};

		int instance;
		orb_publish_auto(ORB_ID(vehicle_attitude), &_att_pub, &att, &instance, ORB_PRIO_HIGH);
	}

	return false;
}

bool Ekf2::publish_wind_estimate(const hrt_abstime &timestamp)
{
	if (_ekf.get_wind_status()) {
		float velNE_wind[2];
		_ekf.get_wind_velocity(velNE_wind);

		float wind_var[2];
		_ekf.get_wind_velocity_var(wind_var);

		// Publish wind estimate
		wind_estimate_s wind_estimate;
		wind_estimate.timestamp = timestamp;
		wind_estimate.windspeed_north = velNE_wind[0];
		wind_estimate.windspeed_east = velNE_wind[1];
		wind_estimate.variance_north = wind_var[0];
		wind_estimate.variance_east = wind_var[1];

		int instance;
		orb_publish_auto(ORB_ID(wind_estimate), &_wind_pub, &wind_estimate, &instance, ORB_PRIO_DEFAULT);

		return true;
	}

	return false;
}

const Vector3f Ekf2::get_vel_body_wind()
{
	// Used to correct baro data for positional errors

	matrix::Quatf q;
	_ekf.copy_quaternion(q.data());
	matrix::Dcmf R_to_body(q.inversed());

	// Calculate wind-compensated velocity in body frame
	// Velocity of body origin in local NED frame (m/s)
	float velocity[3];
	_ekf.get_velocity(velocity);

	float velNE_wind[2];
	_ekf.get_wind_velocity(velNE_wind);

	Vector3f v_wind_comp = {velocity[0] - velNE_wind[0], velocity[1] - velNE_wind[1], velocity[2]};

	return R_to_body * v_wind_comp;
}

bool Ekf2::blend_gps_data()
{
	// zero the blend weights
	memset(&_blend_weights, 0, sizeof(_blend_weights));

	/*
	 * If both receivers have the same update rate, use the oldest non-zero time.
	 * If two receivers with different update rates are used, use the slowest.
	 * If time difference is excessive, use newest to prevent a disconnected receiver
	 * from blocking updates.
	 */

	// Calculate the time step for each receiver with some filtering to reduce the effects of jitter
	// Find the largest and smallest time step.
	float dt_max = 0.0f;
	float dt_min = 0.3f;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		float raw_dt = 0.001f * (float)(_gps_state[i].time_usec - _time_prev_us[i]);

		if (raw_dt > 0.0f && raw_dt < 0.3f) {
			_gps_dt[i] = 0.1f * raw_dt + 0.9f * _gps_dt[i];
		}

		if (_gps_dt[i] > dt_max) {
			dt_max = _gps_dt[i];
			_gps_slowest_index = i;
		}

		if (_gps_dt[i] < dt_min) {
			dt_min = _gps_dt[i];
		}
	}

	// Find the receiver that is last be updated
	uint64_t max_us = 0; // newest non-zero system time of arrival of a GPS message
	uint64_t min_us = -1; // oldest non-zero system time of arrival of a GPS message

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		// Find largest and smallest times
		if (_gps_state[i].time_usec > max_us) {
			max_us = _gps_state[i].time_usec;
			_gps_newest_index = i;
		}

		if ((_gps_state[i].time_usec < min_us) && (_gps_state[i].time_usec > 0)) {
			min_us = _gps_state[i].time_usec;
			_gps_oldest_index = i;
		}
	}

	if ((max_us - min_us) > 300000) {
		// A receiver has timed out so fall out of blending
		return false;
	}

	/*
	 * If the largest dt is less than 20% greater than the smallest, then we have  receivers
	 * running at the same rate then we wait until we have two messages with an arrival time
	 * difference that is less than 50% of the smallest time step and use the time stamp from
	 * the newest data.
	 * Else we have two receivers at different update rates and use the slowest receiver
	 * as the timing reference.
	 */

	if ((dt_max - dt_min) < 0.2f * dt_min) {
		// both receivers assumed to be running at the same rate
		if ((max_us - min_us) < (uint64_t)(5e5f * dt_min)) {
			// data arrival within a short time window enables the two measurements to be blended
			_gps_time_ref_index = _gps_newest_index;
			_gps_new_output_data = true;
		}

	} else {
		// both receivers running at different rates
		_gps_time_ref_index = _gps_slowest_index;

		if (_gps_state[_gps_time_ref_index].time_usec > _time_prev_us[_gps_time_ref_index]) {
			// blend data at the rate of the slower receiver
			_gps_new_output_data = true;
		}
	}

	if (_gps_new_output_data) {
		_gps_blended_state.time_usec = _gps_state[_gps_time_ref_index].time_usec;

		// calculate the sum squared speed accuracy across all GPS sensors
		float speed_accuracy_sum_sq = 0.0f;

		if (_param_ekf2_gps_mask.get() & BLEND_MASK_USE_SPD_ACC) {
			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
				if (_gps_state[i].fix_type >= 3 && _gps_state[i].sacc > 0.0f) {
					speed_accuracy_sum_sq += _gps_state[i].sacc * _gps_state[i].sacc;

				} else {
					// not all receivers support this metric so set it to zero and don't use it
					speed_accuracy_sum_sq = 0.0f;
					break;
				}
			}
		}

		// calculate the sum squared horizontal position accuracy across all GPS sensors
		float horizontal_accuracy_sum_sq = 0.0f;

		if (_param_ekf2_gps_mask.get() & BLEND_MASK_USE_HPOS_ACC) {
			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
				if (_gps_state[i].fix_type >= 2 && _gps_state[i].eph > 0.0f) {
					horizontal_accuracy_sum_sq += _gps_state[i].eph * _gps_state[i].eph;

				} else {
					// not all receivers support this metric so set it to zero and don't use it
					horizontal_accuracy_sum_sq = 0.0f;
					break;
				}
			}
		}

		// calculate the sum squared vertical position accuracy across all GPS sensors
		float vertical_accuracy_sum_sq = 0.0f;

		if (_param_ekf2_gps_mask.get() & BLEND_MASK_USE_VPOS_ACC) {
			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
				if (_gps_state[i].fix_type >= 3 && _gps_state[i].epv > 0.0f) {
					vertical_accuracy_sum_sq += _gps_state[i].epv * _gps_state[i].epv;

				} else {
					// not all receivers support this metric so set it to zero and don't use it
					vertical_accuracy_sum_sq = 0.0f;
					break;
				}
			}
		}

		// Check if we can do blending using reported accuracy
		bool can_do_blending = (horizontal_accuracy_sum_sq > 0.0f || vertical_accuracy_sum_sq > 0.0f
					|| speed_accuracy_sum_sq > 0.0f);

		// if we can't do blending using reported accuracy, return false and hard switch logic will be used instead
		if (!can_do_blending) {
			return false;
		}

		float sum_of_all_weights = 0.0f;

		// calculate a weighting using the reported speed accuracy
		float spd_blend_weights[GPS_MAX_RECEIVERS] = {};

		if (speed_accuracy_sum_sq > 0.0f && (_param_ekf2_gps_mask.get() & BLEND_MASK_USE_SPD_ACC)) {
			// calculate the weights using the inverse of the variances
			float sum_of_spd_weights = 0.0f;

			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
				if (_gps_state[i].fix_type >= 3 && _gps_state[i].sacc >= 0.001f) {
					spd_blend_weights[i] = 1.0f / (_gps_state[i].sacc * _gps_state[i].sacc);
					sum_of_spd_weights += spd_blend_weights[i];
				}
			}

			// normalise the weights
			if (sum_of_spd_weights > 0.0f) {
				for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
					spd_blend_weights[i] = spd_blend_weights[i] / sum_of_spd_weights;
				}

				sum_of_all_weights += 1.0f;
			}
		}

		// calculate a weighting using the reported horizontal position
		float hpos_blend_weights[GPS_MAX_RECEIVERS] = {};

		if (horizontal_accuracy_sum_sq > 0.0f && (_param_ekf2_gps_mask.get() & BLEND_MASK_USE_HPOS_ACC)) {
			// calculate the weights using the inverse of the variances
			float sum_of_hpos_weights = 0.0f;

			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
				if (_gps_state[i].fix_type >= 2 && _gps_state[i].eph >= 0.001f) {
					hpos_blend_weights[i] = horizontal_accuracy_sum_sq / (_gps_state[i].eph * _gps_state[i].eph);
					sum_of_hpos_weights += hpos_blend_weights[i];
				}
			}

			// normalise the weights
			if (sum_of_hpos_weights > 0.0f) {
				for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
					hpos_blend_weights[i] = hpos_blend_weights[i] / sum_of_hpos_weights;
				}

				sum_of_all_weights += 1.0f;
			}
		}

		// calculate a weighting using the reported vertical position accuracy
		float vpos_blend_weights[GPS_MAX_RECEIVERS] = {};

		if (vertical_accuracy_sum_sq > 0.0f && (_param_ekf2_gps_mask.get() & BLEND_MASK_USE_VPOS_ACC)) {
			// calculate the weights using the inverse of the variances
			float sum_of_vpos_weights = 0.0f;

			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
				if (_gps_state[i].fix_type >= 3 && _gps_state[i].epv >= 0.001f) {
					vpos_blend_weights[i] = vertical_accuracy_sum_sq / (_gps_state[i].epv * _gps_state[i].epv);
					sum_of_vpos_weights += vpos_blend_weights[i];
				}
			}

			// normalise the weights
			if (sum_of_vpos_weights > 0.0f) {
				for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
					vpos_blend_weights[i] = vpos_blend_weights[i] / sum_of_vpos_weights;
				}

				sum_of_all_weights += 1.0f;
			};
		}

		// calculate an overall weight
		for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
			_blend_weights[i] = (hpos_blend_weights[i] + vpos_blend_weights[i] + spd_blend_weights[i]) / sum_of_all_weights;
		}

		// With updated weights we can calculate a blended GPS solution and
		// offsets for each physical receiver
		update_gps_blend_states();
		update_gps_offsets();
		_gps_select_index = 2;

	}

	return true;
}

/*
 * Update the internal state estimate for a blended GPS solution that is a weighted average of the phsyical receiver solutions
 * with weights are calculated in calc_gps_blend_weights(). This internal state cannot be used directly by estimators
 * because if physical receivers have significant position differences,  variation in receiver estimated accuracy will
 * cause undesirable variation in the position solution.
*/
void Ekf2::update_gps_blend_states()
{
	// initialise the blended states so we can accumulate the results using the weightings for each GPS receiver.
	_gps_blended_state.time_usec = 0;
	_gps_blended_state.lat = 0;
	_gps_blended_state.lon = 0;
	_gps_blended_state.alt = 0;
	_gps_blended_state.fix_type = 0;
	_gps_blended_state.eph = FLT_MAX;
	_gps_blended_state.epv = FLT_MAX;
	_gps_blended_state.sacc = FLT_MAX;
	_gps_blended_state.vel_m_s = 0.0f;
	_gps_blended_state.vel_ned[0] = 0.0f;
	_gps_blended_state.vel_ned[1] = 0.0f;
	_gps_blended_state.vel_ned[2] = 0.0f;
	_gps_blended_state.vel_ned_valid = true;
	_gps_blended_state.nsats = 0;
	_gps_blended_state.gdop = FLT_MAX;

	_blended_antenna_offset.zero();

	// combine the the GPS states into a blended solution using the weights calculated in calc_blend_weights()
	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		// blend the timing data
		_gps_blended_state.time_usec += (uint64_t)((double)_gps_state[i].time_usec * (double)_blend_weights[i]);

		// use the highest status
		if (_gps_state[i].fix_type > _gps_blended_state.fix_type) {
			_gps_blended_state.fix_type = _gps_state[i].fix_type;
		}

		// calculate a blended average speed and velocity vector
		_gps_blended_state.vel_m_s += _gps_state[i].vel_m_s * _blend_weights[i];
		_gps_blended_state.vel_ned[0] += _gps_state[i].vel_ned[0] * _blend_weights[i];
		_gps_blended_state.vel_ned[1] += _gps_state[i].vel_ned[1] * _blend_weights[i];
		_gps_blended_state.vel_ned[2] += _gps_state[i].vel_ned[2] * _blend_weights[i];

		// Assume blended error magnitude, DOP and sat count is equal to the best value from contributing receivers
		// If any receiver contributing has an invalid velocity, then report blended velocity as invalid
		if (_blend_weights[i] > 0.0f) {

			if (_gps_state[i].eph > 0.0f
			    && _gps_state[i].eph < _gps_blended_state.eph) {
				_gps_blended_state.eph = _gps_state[i].eph;
			}

			if (_gps_state[i].epv > 0.0f
			    && _gps_state[i].epv < _gps_blended_state.epv) {
				_gps_blended_state.epv = _gps_state[i].epv;
			}

			if (_gps_state[i].sacc > 0.0f
			    && _gps_state[i].sacc < _gps_blended_state.sacc) {
				_gps_blended_state.sacc = _gps_state[i].sacc;
			}

			if (_gps_state[i].gdop > 0
			    && _gps_state[i].gdop < _gps_blended_state.gdop) {
				_gps_blended_state.gdop = _gps_state[i].gdop;
			}

			if (_gps_state[i].nsats > 0
			    && _gps_state[i].nsats > _gps_blended_state.nsats) {
				_gps_blended_state.nsats = _gps_state[i].nsats;
			}

			if (!_gps_state[i].vel_ned_valid) {
				_gps_blended_state.vel_ned_valid = false;
			}

		}

		// TODO read parameters for individual GPS antenna positions and blend
		// Vector3f temp_antenna_offset = _antenna_offset[i];
		// temp_antenna_offset *= _blend_weights[i];
		// _blended_antenna_offset += temp_antenna_offset;

	}

	/*
	 * Calculate the instantaneous weighted average location using  available GPS instances and store in  _gps_state.
	 * This is statistically the most likely location, but may not be stable enough for direct use by the EKF.
	*/

	// Use the GPS with the highest weighting as the reference position
	float best_weight = 0.0f;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		if (_blend_weights[i] > best_weight) {
			best_weight = _blend_weights[i];
			_gps_best_index = i;
			_gps_blended_state.lat = _gps_state[i].lat;
			_gps_blended_state.lon = _gps_state[i].lon;
			_gps_blended_state.alt = _gps_state[i].alt;
		}
	}

	// Convert each GPS position to a local NEU offset relative to the reference position
	Vector2f blended_NE_offset_m;
	blended_NE_offset_m.zero();
	float blended_alt_offset_mm = 0.0f;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		if ((_blend_weights[i] > 0.0f) && (i != _gps_best_index)) {
			// calculate the horizontal offset
			Vector2f horiz_offset{};
			get_vector_to_next_waypoint((_gps_blended_state.lat / 1.0e7),
						    (_gps_blended_state.lon / 1.0e7), (_gps_state[i].lat / 1.0e7), (_gps_state[i].lon / 1.0e7),
						    &horiz_offset(0), &horiz_offset(1));

			// sum weighted offsets
			blended_NE_offset_m += horiz_offset * _blend_weights[i];

			// calculate vertical offset
			float vert_offset = (float)(_gps_state[i].alt - _gps_blended_state.alt);

			// sum weighted offsets
			blended_alt_offset_mm += vert_offset * _blend_weights[i];
		}
	}

	// Add the sum of weighted offsets to the reference position to obtain the blended position
	double lat_deg_now = (double)_gps_blended_state.lat * 1.0e-7;
	double lon_deg_now = (double)_gps_blended_state.lon * 1.0e-7;
	double lat_deg_res, lon_deg_res;
	add_vector_to_global_position(lat_deg_now, lon_deg_now, blended_NE_offset_m(0), blended_NE_offset_m(1), &lat_deg_res,
				      &lon_deg_res);
	_gps_blended_state.lat = (int32_t)(1.0E7 * lat_deg_res);
	_gps_blended_state.lon = (int32_t)(1.0E7 * lon_deg_res);
	_gps_blended_state.alt += (int32_t)blended_alt_offset_mm;

	// Take GPS heading from the highest weighted receiver that is publishing a valid .heading value
	uint8_t gps_best_yaw_index = 0;
	best_weight = 0.0f;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		if (PX4_ISFINITE(_gps_state[i].yaw) && (_blend_weights[i] > best_weight)) {
			best_weight = _blend_weights[i];
			gps_best_yaw_index = i;
		}
	}

	_gps_blended_state.yaw = _gps_state[gps_best_yaw_index].yaw;
	_gps_blended_state.yaw_offset = _gps_state[gps_best_yaw_index].yaw_offset;
}

/*
 * The location in _gps_blended_state will move around as the relative accuracy changes.
 * To mitigate this effect a low-pass filtered offset from each GPS location to the blended location is
 * calculated.
*/
void Ekf2::update_gps_offsets()
{

	// Calculate filter coefficients to be applied to the offsets for each GPS position and height offset
	// Increase the filter time constant proportional to the inverse of the weighting
	// A weighting of 1 will make the offset adjust the slowest, a weighting of 0 will make it adjust with zero filtering
	float alpha[GPS_MAX_RECEIVERS] = {};
	float omega_lpf = 1.0f / fmaxf(_param_ekf2_gps_tau.get(), 1.0f);

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		if (_gps_state[i].time_usec - _time_prev_us[i] > 0) {
			// calculate the filter coefficient that achieves the time constant specified by the user adjustable parameter
			float min_alpha = constrain(omega_lpf * 1e-6f * (float)(_gps_state[i].time_usec - _time_prev_us[i]),
						    0.0f, 1.0f);

			// scale the filter coefficient so that time constant is inversely proprtional to weighting
			if (_blend_weights[i] > min_alpha) {
				alpha[i] = min_alpha / _blend_weights[i];

			} else {
				alpha[i] = 1.0f;
			}

			_time_prev_us[i] = _gps_state[i].time_usec;
		}
	}

	// Calculate a filtered position delta for each GPS relative to the blended solution state
	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		Vector2f offset;
		get_vector_to_next_waypoint((_gps_state[i].lat / 1.0e7), (_gps_state[i].lon / 1.0e7),
					    (_gps_blended_state.lat / 1.0e7), (_gps_blended_state.lon / 1.0e7), &offset(0), &offset(1));
		_NE_pos_offset_m[i] = offset * alpha[i] + _NE_pos_offset_m[i] * (1.0f - alpha[i]);
		_hgt_offset_mm[i] = (float)(_gps_blended_state.alt - _gps_state[i].alt) *  alpha[i] +
				    _hgt_offset_mm[i] * (1.0f - alpha[i]);
	}

	// calculate offset limits from the largest difference between receivers
	Vector2f max_ne_offset{};
	float max_alt_offset = 0;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		for (uint8_t j = i; j < GPS_MAX_RECEIVERS; j++) {
			if (i != j) {
				Vector2f offset;
				get_vector_to_next_waypoint((_gps_state[i].lat / 1.0e7), (_gps_state[i].lon / 1.0e7),
							    (_gps_state[j].lat / 1.0e7), (_gps_state[j].lon / 1.0e7), &offset(0), &offset(1));
				max_ne_offset(0) = fmaxf(max_ne_offset(0), fabsf(offset(0)));
				max_ne_offset(1) = fmaxf(max_ne_offset(1), fabsf(offset(1)));
				max_alt_offset = fmaxf(max_alt_offset, fabsf((float)(_gps_state[i].alt - _gps_state[j].alt)));
			}
		}
	}

	// apply offset limits
	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		_NE_pos_offset_m[i](0) = constrain(_NE_pos_offset_m[i](0), -max_ne_offset(0), max_ne_offset(0));
		_NE_pos_offset_m[i](1) = constrain(_NE_pos_offset_m[i](1), -max_ne_offset(1), max_ne_offset(1));
		_hgt_offset_mm[i] = constrain(_hgt_offset_mm[i], -max_alt_offset, max_alt_offset);
	}

}


/*
 * Apply the steady state physical receiver offsets calculated by update_gps_offsets().
*/
void Ekf2::apply_gps_offsets()
{
	// calculate offset corrected output for each physical GPS.
	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		// Add the sum of weighted offsets to the reference position to obtain the blended position
		double lat_deg_now = (double)_gps_state[i].lat * 1.0e-7;
		double lon_deg_now = (double)_gps_state[i].lon * 1.0e-7;
		double lat_deg_res, lon_deg_res;
		add_vector_to_global_position(lat_deg_now, lon_deg_now, _NE_pos_offset_m[i](0), _NE_pos_offset_m[i](1), &lat_deg_res,
					      &lon_deg_res);
		_gps_output[i].lat = (int32_t)(1.0E7 * lat_deg_res);
		_gps_output[i].lon = (int32_t)(1.0E7 * lon_deg_res);
		_gps_output[i].alt = _gps_state[i].alt + (int32_t)_hgt_offset_mm[i];

		// other receiver data is used uncorrected
		_gps_output[i].time_usec	= _gps_state[i].time_usec;
		_gps_output[i].fix_type		= _gps_state[i].fix_type;
		_gps_output[i].vel_m_s		= _gps_state[i].vel_m_s;
		_gps_output[i].vel_ned[0]	= _gps_state[i].vel_ned[0];
		_gps_output[i].vel_ned[1]	= _gps_state[i].vel_ned[1];
		_gps_output[i].vel_ned[2]	= _gps_state[i].vel_ned[2];
		_gps_output[i].eph		= _gps_state[i].eph;
		_gps_output[i].epv		= _gps_state[i].epv;
		_gps_output[i].sacc		= _gps_state[i].sacc;
		_gps_output[i].gdop		= _gps_state[i].gdop;
		_gps_output[i].nsats		= _gps_state[i].nsats;
		_gps_output[i].vel_ned_valid	= _gps_state[i].vel_ned_valid;
		_gps_output[i].yaw		= _gps_state[i].yaw;
		_gps_output[i].yaw_offset	= _gps_state[i].yaw_offset;

	}
}

/*
 Calculate GPS output that is a blend of the offset corrected physical receiver data
*/
void Ekf2::calc_gps_blend_output()
{
	// Convert each GPS position to a local NEU offset relative to the reference position
	// which is defined as the positon of the blended solution calculated from non offset corrected data
	Vector2f blended_NE_offset_m;
	blended_NE_offset_m.zero();
	float blended_alt_offset_mm = 0.0f;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		if (_blend_weights[i] > 0.0f) {
			// calculate the horizontal offset
			Vector2f horiz_offset{};
			get_vector_to_next_waypoint((_gps_blended_state.lat / 1.0e7),
						    (_gps_blended_state.lon / 1.0e7),
						    (_gps_output[i].lat / 1.0e7),
						    (_gps_output[i].lon / 1.0e7),
						    &horiz_offset(0),
						    &horiz_offset(1));

			// sum weighted offsets
			blended_NE_offset_m += horiz_offset * _blend_weights[i];

			// calculate vertical offset
			float vert_offset = (float)(_gps_output[i].alt - _gps_blended_state.alt);

			// sum weighted offsets
			blended_alt_offset_mm += vert_offset * _blend_weights[i];
		}
	}

	// Add the sum of weighted offsets to the reference position to obtain the blended position
	double lat_deg_now = (double)_gps_blended_state.lat * 1.0e-7;
	double lon_deg_now = (double)_gps_blended_state.lon * 1.0e-7;
	double lat_deg_res, lon_deg_res;
	add_vector_to_global_position(lat_deg_now, lon_deg_now, blended_NE_offset_m(0), blended_NE_offset_m(1), &lat_deg_res,
				      &lon_deg_res);
	_gps_output[GPS_BLENDED_INSTANCE].lat = (int32_t)(1.0E7 * lat_deg_res);
	_gps_output[GPS_BLENDED_INSTANCE].lon = (int32_t)(1.0E7 * lon_deg_res);
	_gps_output[GPS_BLENDED_INSTANCE].alt = _gps_blended_state.alt + (int32_t)blended_alt_offset_mm;

	// Copy remaining data from internal states to output
	_gps_output[GPS_BLENDED_INSTANCE].time_usec	= _gps_blended_state.time_usec;
	_gps_output[GPS_BLENDED_INSTANCE].fix_type	= _gps_blended_state.fix_type;
	_gps_output[GPS_BLENDED_INSTANCE].vel_m_s	= _gps_blended_state.vel_m_s;
	_gps_output[GPS_BLENDED_INSTANCE].vel_ned[0]	= _gps_blended_state.vel_ned[0];
	_gps_output[GPS_BLENDED_INSTANCE].vel_ned[1]	= _gps_blended_state.vel_ned[1];
	_gps_output[GPS_BLENDED_INSTANCE].vel_ned[2]	= _gps_blended_state.vel_ned[2];
	_gps_output[GPS_BLENDED_INSTANCE].eph		= _gps_blended_state.eph;
	_gps_output[GPS_BLENDED_INSTANCE].epv		= _gps_blended_state.epv;
	_gps_output[GPS_BLENDED_INSTANCE].sacc		= _gps_blended_state.sacc;
	_gps_output[GPS_BLENDED_INSTANCE].gdop		= _gps_blended_state.gdop;
	_gps_output[GPS_BLENDED_INSTANCE].nsats		= _gps_blended_state.nsats;
	_gps_output[GPS_BLENDED_INSTANCE].vel_ned_valid	= _gps_blended_state.vel_ned_valid;
	_gps_output[GPS_BLENDED_INSTANCE].yaw		= _gps_blended_state.yaw;
	_gps_output[GPS_BLENDED_INSTANCE].yaw_offset	= _gps_blended_state.yaw_offset;

}

float Ekf2::filter_altitude_ellipsoid(float amsl_hgt)
{

	float height_diff = static_cast<float>(_gps_alttitude_ellipsoid[0]) * 1e-3f - amsl_hgt;

	if (_gps_alttitude_ellipsoid_previous_timestamp[0] == 0) {

		_wgs84_hgt_offset = height_diff;
		_gps_alttitude_ellipsoid_previous_timestamp[0] = _gps_state[0].time_usec;

	} else if (_gps_state[0].time_usec != _gps_alttitude_ellipsoid_previous_timestamp[0]) {

		// apply a 10 second first order low pass filter to baro offset
		float dt = 1e-6f * static_cast<float>(_gps_state[0].time_usec - _gps_alttitude_ellipsoid_previous_timestamp[0]);
		_gps_alttitude_ellipsoid_previous_timestamp[0] = _gps_state[0].time_usec;
		float offset_rate_correction = 0.1f * (height_diff - _wgs84_hgt_offset);
		_wgs84_hgt_offset += dt * math::constrain(offset_rate_correction, -0.1f, 0.1f);
	}

	return amsl_hgt + _wgs84_hgt_offset;
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

The documentation can be found on the [ECL/EKF Overview & Tuning](https://docs.px4.io/en/advanced_config/tuning_the_ecl_ekf.html) page.

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
