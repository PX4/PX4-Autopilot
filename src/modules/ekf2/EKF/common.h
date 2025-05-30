/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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
 * @file common.h
 * Definition of base class for attitude estimators
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Siddharth Bharat Purohit <siddharthbharatpurohit@gmail.com>
 *
 */
#ifndef EKF_COMMON_H
#define EKF_COMMON_H

#include <cstdint>

#include <matrix/math.hpp>
#include <mathlib/math/Utilities.hpp>

namespace estimator
{

using matrix::AxisAnglef;
using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Matrix3f;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;
using matrix::wrap_pi;

using math::Utilities::getEulerYaw;
using math::Utilities::quatToInverseRotMat;
using math::Utilities::shouldUse321RotationSequence;
using math::Utilities::sq;
using math::Utilities::updateYawInRotMat;

// maximum sensor intervals in usec
static constexpr uint64_t BARO_MAX_INTERVAL     =
	200e3;  ///< Maximum allowable time interval between pressure altitude measurements (uSec)
static constexpr uint64_t EV_MAX_INTERVAL       =
	200e3;  ///< Maximum allowable time interval between external vision system measurements (uSec)
static constexpr uint64_t GNSS_MAX_INTERVAL     =
	500e3;  ///< Maximum allowable time interval between GNSS measurements (uSec)
static constexpr uint64_t GNSS_YAW_MAX_INTERVAL =
	1500e3; ///< Maximum allowable time interval between GNSS yaw measurements (uSec)
static constexpr uint64_t MAG_MAX_INTERVAL      =
	500e3;  ///< Maximum allowable time interval between magnetic field measurements (uSec)

// bad accelerometer detection and mitigation
static constexpr uint64_t BADACC_PROBATION =
	3e6; ///< Period of time that accel data declared bad must continuously pass checks to be declared good again (uSec)
static constexpr float BADACC_BIAS_PNOISE =
	4.9f;  ///< The delta velocity process noise is set to this when accel data is declared bad (m/sec**2)

// ground effect compensation
static constexpr uint64_t GNDEFFECT_TIMEOUT =
	10e6; ///< Maximum period of time that ground effect protection will be active after it was last turned on (uSec)

enum class PositionFrame : uint8_t {
	LOCAL_FRAME_NED = 0,
	LOCAL_FRAME_FRD = 1,
};

enum class VelocityFrame : uint8_t {
	LOCAL_FRAME_NED = 0,
	LOCAL_FRAME_FRD = 1,
	BODY_FRAME_FRD  = 2
};

#if defined(CONFIG_EKF2_MAGNETOMETER)
enum GeoDeclinationMask : uint8_t {
	// Bit locations for mag_declination_source
	USE_GEO_DECL  = (1 << 0), ///< set to true to use the declination from the geo library when the GPS position becomes available, set to false to always use the EKF2_MAG_DECL value
	SAVE_GEO_DECL = (1 << 1) ///< set to true to set the EKF2_MAG_DECL parameter to the value returned by the geo library
};

enum MagFuseType : uint8_t {
	// Integer definitions for mag_fusion_type
	AUTO    = 0,   	///< The selection of either heading or 3D magnetometer fusion will be automatic
	HEADING = 1,   	///< Simple yaw angle fusion will always be used. This is less accurate, but less affected by earth field distortions. It should not be used for pitch angles outside the range from -60 to +60 deg
	NONE    = 5,   	///< Do not use magnetometer under any circumstance.
	INIT    = 6     ///< Use the mag for heading initialization only.
};
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_TERRAIN)
enum class TerrainFusionMask : uint8_t {
	TerrainFuseRangeFinder = (1 << 0),
	TerrainFuseOpticalFlow = (1 << 1)
};
#endif // CONFIG_EKF2_TERRAIN

enum class HeightSensor : uint8_t {
	BARO  = 0,
	GNSS  = 1,
	RANGE = 2,
	EV    = 3,
	UNKNOWN  = 4
};

enum class PositionSensor : uint8_t {
	UNKNOWN = 0,
	GNSS    = 1,
	EV      = 2,
};

enum class ImuCtrl : uint8_t {
	GyroBias      = (1 << 0),
	AccelBias     = (1 << 1),
	GravityVector = (1 << 2),
};

enum class GnssCtrl : uint8_t {
	HPOS  = (1 << 0),
	VPOS  = (1 << 1),
	VEL  = (1 << 2),
	YAW  = (1 << 3)
};

enum class RngCtrl : uint8_t {
	DISABLED    = 0,
	CONDITIONAL = 1,
	ENABLED     = 2
};

enum class EvCtrl : uint8_t {
	HPOS = (1 << 0),
	VPOS = (1 << 1),
	VEL  = (1 << 2),
	YAW  = (1 << 3)
};

enum class MagCheckMask : uint8_t {
	STRENGTH    = (1 << 0),
	INCLINATION = (1 << 1),
	FORCE_WMM   = (1 << 2)
};

enum class FlowGyroSource : uint8_t {
	Auto     = 0,
	Internal = 1
};

struct imuSample {
	uint64_t    time_us{};                ///< timestamp of the measurement (uSec)
	Vector3f    delta_ang{};              ///< delta angle in body frame (integrated gyro measurements) (rad)
	Vector3f    delta_vel{};              ///< delta velocity in body frame (integrated accelerometer measurements) (m/sec)
	float       delta_ang_dt{};           ///< delta angle integration period (sec)
	float       delta_vel_dt{};           ///< delta velocity integration period (sec)
	bool        delta_vel_clipping[3] {}; ///< true (per axis) if this sample contained any accelerometer clipping
};

struct gnssSample {
	uint64_t    time_us{};    ///< timestamp of the measurement (uSec)
	double      lat{};        ///< latitude (degrees)
	double      lon{};        ///< longitude (degrees)
	float       alt{};        ///< GNSS altitude above MSL (m)
	Vector3f    vel{};        ///< NED earth frame GNSS velocity measurement (m/sec)
	float       hacc{};       ///< 1-std horizontal position error (m)
	float       vacc{};       ///< 1-std vertical position error (m)
	float       sacc{};       ///< 1-std speed error (m/sec)
	uint8_t     fix_type{};   ///< 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time
	uint8_t     nsats{};      ///< number of satellites used
	float       pdop{};       ///< position dilution of precision
	float       yaw{};        ///< yaw angle. NaN if not set (used for dual antenna GPS), (rad, [-PI, PI])
	float       yaw_acc{};    ///< 1-std yaw error (rad)
	float       yaw_offset{}; ///< Heading/Yaw offset for dual antenna GPS - refer to description for GPS_YAW_OFFSET
	bool        spoofed{};    ///< true if GNSS data is spoofed
};

struct magSample {
	uint64_t    time_us{};  ///< timestamp of the measurement (uSec)
	Vector3f    mag{};      ///< NED magnetometer body frame measurements (Gauss)
	bool        reset{false}; ///< magnetometer changed (different sensor or calibration change)
};

struct baroSample {
	uint64_t    time_us{};  ///< timestamp of the measurement (uSec)
	float       hgt{};      ///< pressure altitude above sea level (m)
	bool        reset{false};
};

struct airspeedSample {
	uint64_t    time_us{};          ///< timestamp of the measurement (uSec)
	float       true_airspeed{};    ///< true airspeed measurement (m/sec)
	float       eas2tas{};          ///< equivalent to true airspeed factor
};

struct flowSample {
	uint64_t    time_us{};   ///< timestamp of the integration period midpoint (uSec)
	Vector2f    flow_rate{}; ///< measured angular rate of the image about the X and Y body axes (rad/s), RH rotation is positive
	Vector3f    gyro_rate{}; ///< measured angular rate of the inertial frame about the body axes obtained from rate gyro measurements (rad/s), RH rotation is positive
	uint8_t     quality{};   ///< quality indicator between 0 and 255
};

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
struct extVisionSample {
	uint64_t    time_us{};     ///< timestamp of the measurement (uSec)
	Vector3f    pos{};         ///< XYZ position in external vision's local reference frame (m) - Z must be aligned with down axis
	Vector3f    vel{};         ///< FRD velocity in reference frame defined in vel_frame variable (m/sec) - Z must be aligned with down axis
	Quatf       quat{};        ///< quaternion defining rotation from body to earth frame
	Vector3f    position_var{};    ///< XYZ position variances (m**2)
	Vector3f    velocity_var{};    ///< XYZ velocity variances ((m/sec)**2)
	Vector3f    orientation_var{}; ///< orientation variance (rad**2)
	PositionFrame pos_frame = PositionFrame::LOCAL_FRAME_FRD;
	VelocityFrame vel_frame = VelocityFrame::BODY_FRAME_FRD;
	uint8_t     reset_counter{};
	int8_t     quality{};     ///< quality indicator between 0 and 100
};
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_DRAG_FUSION)
struct dragSample {
	uint64_t    time_us{};     ///< timestamp of the measurement (uSec)
	Vector2f    accelXY{};     ///< measured specific force along the X and Y body axes (m/sec**2)
};
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_AUXVEL)
struct auxVelSample {
	uint64_t    time_us{};     ///< timestamp of the measurement (uSec)
	Vector2f    vel{};         ///< measured NE velocity relative to the local origin (m/sec)
	Vector2f    velVar{};      ///< estimated error variance of the NE velocity (m/sec)**2
};
#endif // CONFIG_EKF2_AUXVEL

struct systemFlagUpdate {
	uint64_t time_us{};
	bool at_rest{false};
	bool in_air{true};
	bool is_fixed_wing{false};
	bool gnd_effect{false};
	bool constant_pos{false};
};

struct parameters {

	int32_t filter_update_interval_us{10000}; ///< filter update interval in microseconds

	int32_t imu_ctrl{static_cast<int32_t>(ImuCtrl::GyroBias) | static_cast<int32_t>(ImuCtrl::AccelBias)};

	float velocity_limit{100.f};           ///< velocity state limit (m/s)

	// measurement source control
	int32_t height_sensor_ref{static_cast<int32_t>(HeightSensor::BARO)};
	int32_t position_sensor_ref{static_cast<int32_t>(PositionSensor::GNSS)};

	float delay_max_ms{110.f};              ///< maximum time delay of all the aiding sensors. Sets the size of the observation buffers. (mSec)

	// input noise
	float gyro_noise{1.5e-2f};              ///< IMU angular rate noise used for covariance prediction (rad/sec)
	float accel_noise{3.5e-1f};             ///< IMU acceleration noise use for covariance prediction (m/sec**2)

	// process noise
	float gyro_bias_p_noise{1.0e-3f};       ///< process noise for IMU rate gyro bias prediction (rad/sec**2)
	float accel_bias_p_noise{1.0e-2f};      ///< process noise for IMU accelerometer bias prediction (m/sec**3)

#if defined(CONFIG_EKF2_WIND)
	const float initial_wind_uncertainty {1.0f};    ///< 1-sigma initial uncertainty in wind velocity (m/sec)
	float wind_vel_nsd{1.0e-2f};        ///< process noise spectral density for wind velocity prediction (m/sec**2/sqrt(Hz))
	const float wind_vel_nsd_scaler{0.5f};      ///< scaling of wind process noise with vertical velocity
#endif // CONFIG_EKF2_WIND

	// initialization errors
	float switch_on_gyro_bias{0.1f};        ///< 1-sigma gyro bias uncertainty at switch on (rad/sec)
	float switch_on_accel_bias{0.2f};       ///< 1-sigma accelerometer bias uncertainty at switch on (m/sec**2)
	float initial_tilt_err{0.1f};           ///< 1-sigma tilt error after initial alignment using gravity vector (rad)

#if defined(CONFIG_EKF2_BAROMETER)
	int32_t baro_ctrl {1};
	float baro_delay_ms{0.0f};              ///< barometer height measurement delay relative to the IMU (mSec)
	float baro_noise{2.0f};                 ///< observation noise for barometric height fusion (m)
	float baro_bias_nsd{0.13f};             ///< process noise for barometric height bias estimation (m/s/sqrt(Hz))
	float baro_innov_gate{5.0f};            ///< barometric and GPS height innovation consistency gate size (STD)

	float gnd_effect_deadzone{5.0f};        ///< Size of deadzone applied to negative baro innovations when ground effect compensation is active (m)
	float gnd_effect_max_hgt{0.5f};         ///< Height above ground at which baro ground effect becomes insignificant (m)

# if defined(CONFIG_EKF2_BARO_COMPENSATION)
	// static barometer pressure position error coefficient along body axes
	float static_pressure_coef_xp{0.0f};    // (-)
	float static_pressure_coef_xn{0.0f};    // (-)
	float static_pressure_coef_yp{0.0f};    // (-)
	float static_pressure_coef_yn{0.0f};    // (-)
	float static_pressure_coef_z{0.0f};     // (-)

	// upper limit on airspeed used for correction  (m/s**2)
	float max_correction_airspeed{20.0f};
# endif // CONFIG_EKF2_BARO_COMPENSATION
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_GNSS)
	int32_t gnss_ctrl {static_cast<int32_t>(GnssCtrl::HPOS) | static_cast<int32_t>(GnssCtrl::VEL)};
	float gps_delay_ms{110.0f};             ///< GPS measurement delay relative to the IMU (mSec)

	Vector3f gps_pos_body{};                ///< xyz position of the GPS antenna in body frame (m)

	// position and velocity fusion
	float gps_vel_noise{0.5f};           ///< minimum allowed observation noise for gps velocity fusion (m/sec)
	float gps_pos_noise{0.5f};              ///< minimum allowed observation noise for gps position fusion (m)
	float gps_hgt_bias_nsd{0.13f};          ///< process noise for gnss height bias estimation (m/s/sqrt(Hz))
	float gps_pos_innov_gate{5.0f};         ///< GPS horizontal position innovation consistency gate size (STD)
	float gps_vel_innov_gate{5.0f};         ///< GPS velocity innovation consistency gate size (STD)

	// these parameters control the strictness of GPS quality checks used to determine if the GPS is
	// good enough to set a local origin and commence aiding
	int32_t gps_check_mask{21};             ///< bitmask used to control which GPS quality checks are used
	float req_hacc{5.0f};                   ///< maximum acceptable horizontal position error (m)
	float req_vacc{8.0f};                   ///< maximum acceptable vertical position error (m)
	float req_sacc{1.0f};                   ///< maximum acceptable speed error (m/s)
	int32_t req_nsats{6};                   ///< minimum acceptable satellite count
	float req_pdop{2.0f};                   ///< maximum acceptable position dilution of precision
	float req_hdrift{0.3f};                 ///< maximum acceptable horizontal drift speed (m/s)
	float req_vdrift{0.5f};                 ///< maximum acceptable vertical drift speed (m/s)

# if defined(CONFIG_EKF2_GNSS_YAW)
	// GNSS heading fusion
	float gnss_heading_noise{0.1f};          ///< measurement noise standard deviation used for GNSS heading fusion (rad)
# endif // CONFIG_EKF2_GNSS_YAW

	// Parameters used to control when yaw is reset to the EKF-GSF yaw estimator value
	float EKFGSF_tas_default{15.0f};                ///< default airspeed value assumed during fixed wing flight if no airspeed measurement available (m/s)
	const unsigned EKFGSF_reset_delay{1000000};     ///< Number of uSec of bad innovations on main filter in immediate post-takeoff phase before yaw is reset to EKF-GSF value
	const float EKFGSF_yaw_err_max{0.262f};         ///< Composite yaw 1-sigma uncertainty threshold used to check for convergence (rad)

#endif // CONFIG_EKF2_GNSS

	float pos_noaid_noise{10.0f};           ///< observation noise for non-aiding position fusion (m)

	float heading_innov_gate{2.6f};         ///< heading fusion innovation consistency gate size (STD)
	float mag_heading_noise{3.0e-1f};       ///< measurement noise used for simple heading fusion (rad)

#if defined(CONFIG_EKF2_MAGNETOMETER)
	float mag_delay_ms {0.0f};              ///< magnetometer measurement delay relative to the IMU (mSec)

	float mage_p_noise{1.0e-3f};            ///< process noise for earth magnetic field prediction (Gauss/sec)
	float magb_p_noise{1.0e-4f};            ///< process noise for body magnetic field prediction (Gauss/sec)

	// magnetometer fusion
	float mag_noise{5.0e-2f};               ///< measurement noise used for 3-axis magnetometer fusion (Gauss)
	float mag_declination_deg{0.0f};        ///< magnetic declination (degrees)
	float mag_innov_gate{3.0f};             ///< magnetometer fusion innovation consistency gate size (STD)
	int32_t mag_declination_source{3};      ///< bitmask used to control the handling of declination data
	int32_t mag_fusion_type{0};             ///< integer used to specify the type of magnetometer fusion used
	float mag_acc_gate{0.5f};               ///< when in auto select mode, heading fusion will be used when manoeuvre accel is lower than this (m/sec**2)

	// compute synthetic magnetomter Z value if possible
	int32_t synthesize_mag_z{0};
	int32_t mag_check{0};
	float mag_check_strength_tolerance_gs{0.2f};
	float mag_check_inclination_tolerance_deg{20.f};
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_AIRSPEED)
	// airspeed fusion
	float airspeed_delay_ms{100.0f};        ///< airspeed measurement delay relative to the IMU (mSec)
	float tas_innov_gate{5.0f};             ///< True Airspeed innovation consistency gate size (STD)
	float eas_noise{1.4f};                  ///< EAS measurement noise standard deviation used for airspeed fusion (m/s)
	float arsp_thr{2.0f};                   ///< Airspeed fusion threshold. A value of zero will deactivate airspeed fusion
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	// synthetic sideslip fusion
	int32_t beta_fusion_enabled{0};
	float beta_innov_gate{5.0f};            ///< synthetic sideslip innovation consistency gate size in standard deviation (STD)
	float beta_noise{0.3f};                 ///< synthetic sideslip noise (rad)
	const float beta_avg_ft_us{150000.0f};  ///< The average time between synthetic sideslip measurements (uSec)
#endif // CONFIG_EKF2_SIDESLIP

#if defined(CONFIG_EKF2_TERRAIN)
	float terrain_p_noise {5.0f};           ///< process noise for terrain offset (m/sec)
	float terrain_gradient{0.5f};           ///< gradient of terrain used to estimate process noise due to changing position (m/m)
	const float terrain_timeout{10.f};      ///< maximum time for invalid bottom distance measurements before resetting terrain estimate (s)
#endif // CONFIG_EKF2_TERRAIN

#if defined(CONFIG_EKF2_TERRAIN) || defined(CONFIG_EKF2_OPTICAL_FLOW) || defined(CONFIG_EKF2_RANGE_FINDER)
	float rng_gnd_clearance {0.1f};         ///< minimum valid value for range when on ground (m)
#endif // CONFIG_EKF2_TERRAIN || CONFIG_EKF2_OPTICAL_FLOW || CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// range finder fusion
	int32_t rng_ctrl{static_cast<int32_t>(RngCtrl::CONDITIONAL)};

	float range_delay_ms{5.0f};             ///< range finder measurement delay relative to the IMU (mSec)
	float range_noise{0.1f};                ///< observation noise for range finder measurements (m)
	float range_innov_gate{5.0f};           ///< range finder fusion innovation consistency gate size (STD)
	float rng_sens_pitch{0.0f};             ///< Pitch offset of the range sensor (rad). Sensor points out along Z axis when offset is zero. Positive rotation is RH about Y axis.
	float range_noise_scaler{0.0f};         ///< scaling from range measurement to noise (m/m)
	float max_hagl_for_range_aid{5.0f};     ///< maximum height above ground for which we allow to use the range finder as height source (if rng_control == 1)
	float max_vel_for_range_aid{1.0f};      ///< maximum ground velocity for which we allow to use the range finder as height source (if rng_control == 1)
	float range_aid_innov_gate{1.0f};       ///< gate size used for innovation consistency checks for range aid fusion
	float range_valid_quality_s{1.0f};      ///< minimum duration during which the reported range finder signal quality needs to be non-zero in order to be declared valid (s)
	float range_cos_max_tilt{0.7071f};      ///< cosine of the maximum tilt angle from the vertical that permits use of range finder and flow data
	float range_kin_consistency_gate{1.0f}; ///< gate size used by the range finder kinematic consistency check
	float rng_fog{0.f};                 	///< max distance which a blocked range sensor measures (fog, dirt) [m]

	Vector3f rng_pos_body{};                ///< xyz position of range sensor in body frame (m)
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// vision position fusion
	int32_t ev_ctrl{0};
	float ev_delay_ms{175.0f};              ///< off-board vision measurement delay relative to the IMU (mSec)

	float ev_vel_noise{0.1f};               ///< minimum allowed observation noise for EV velocity fusion (m/sec)
	float ev_pos_noise{0.1f};               ///< minimum allowed observation noise for EV position fusion (m)
	float ev_att_noise{0.1f};               ///< minimum allowed observation noise for EV attitude fusion (rad/sec)
	int32_t ev_quality_minimum{0};          ///< vision minimum acceptable quality integer
	float ev_vel_innov_gate{3.0f};          ///< vision velocity fusion innovation consistency gate size (STD)
	float ev_pos_innov_gate{5.0f};          ///< vision position fusion innovation consistency gate size (STD)
	float ev_hgt_bias_nsd{0.13f};           ///< process noise for vision height bias estimation (m/s/sqrt(Hz))

	Vector3f ev_pos_body{};                 ///< xyz position of VI-sensor focal point in body frame (m)
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	// gravity fusion
	float gravity_noise{1.0f};              ///< accelerometer measurement gaussian noise (m/s**2)
#endif // CONFIG_EKF2_GRAVITY_FUSION

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	int32_t flow_ctrl {0};
	int32_t flow_gyro_src {static_cast<int32_t>(FlowGyroSource::Auto)};
	float flow_delay_ms{5.0f};              ///< optical flow measurement delay relative to the IMU (mSec) - this is to the middle of the optical flow integration interval

	// optical flow fusion
	float flow_noise{0.15f};                ///< observation noise for optical flow LOS rate measurements (rad/sec)
	float flow_noise_qual_min{0.5f};        ///< observation noise for optical flow LOS rate measurements when flow sensor quality is at the minimum useable (rad/sec)
	int32_t flow_qual_min{1};               ///< minimum acceptable quality integer from  the flow sensor
	int32_t flow_qual_min_gnd{0};           ///< minimum acceptable quality integer from  the flow sensor when on ground
	float flow_innov_gate{3.0f};            ///< optical flow fusion innovation consistency gate size (STD)

	Vector3f flow_pos_body{};               ///< xyz position of range sensor focal point in body frame (m)
#endif // CONFIG_EKF2_OPTICAL_FLOW

	// XYZ offset of sensors in body axes (m)
	Vector3f imu_pos_body{};                ///< xyz position of IMU in body frame (m)

	// accel bias learning control
	float acc_bias_lim{0.4f};               ///< maximum accel bias magnitude (m/sec**2)
	float acc_bias_learn_acc_lim{25.0f};    ///< learning is disabled if the magnitude of the IMU acceleration vector is greater than this (m/sec**2)
	float acc_bias_learn_gyr_lim{3.0f};     ///< learning is disabled if the magnitude of the IMU angular rate vector is greater than this (rad/sec)
	float acc_bias_learn_tc{0.5f};          ///< time constant used to control the decaying envelope filters applied to the accel and gyro magnitudes (sec)

	float gyro_bias_lim{0.4f};              ///< maximum gyro bias magnitude (rad/sec)

	const unsigned reset_timeout_max{7'000'000};      ///< maximum time we allow horizontal inertial dead reckoning before attempting to reset the states to the measurement or change _control_status if the data is unavailable (uSec)
	const unsigned no_aid_timeout_max{1'000'000};     ///< maximum lapsed time from last fusion of a measurement that constrains horizontal velocity drift before the EKF will determine that the sensor is no longer contributing to aiding (uSec)
	const unsigned hgt_fusion_timeout_max{5'000'000}; ///< maximum time we allow height fusion to fail before attempting a reset or stopping the fusion aiding (uSec)

	int32_t valid_timeout_max{5'000'000};     ///< amount of time spent inertial dead reckoning before the estimator reports the state estimates as invalid (uSec)

#if defined(CONFIG_EKF2_DRAG_FUSION)
	// multi-rotor drag specific force fusion
	int32_t drag_ctrl{0};
	float drag_noise{2.5f};                 ///< observation noise variance for drag specific force measurements (m/sec**2)**2
	float bcoef_x{100.0f};                  ///< bluff body drag ballistic coefficient for the X-axis (kg/m**2)
	float bcoef_y{100.0f};                  ///< bluff body drag ballistic coefficient for the Y-axis (kg/m**2)
	float mcoef{0.1f};                      ///< rotor momentum drag coefficient for the X and Y axes (1/s)
#endif // CONFIG_EKF2_DRAG_FUSION

	// control of accel error detection and mitigation (IMU clipping)
	const float vert_innov_test_lim{3.0f};          ///< Number of standard deviations of vertical vel/pos innovations allowed before triggering a vertical acceleration failure
	const float vert_innov_test_min{1.0f};          ///< Minimum number of standard deviations of vertical vel/pos innovations required to trigger a vertical acceleration failure
	const int bad_acc_reset_delay_us{500000};       ///< Continuous time that the vertical position and velocity innovation test must fail before the states are reset (uSec)

#if defined(CONFIG_EKF2_AUXVEL)
	// auxiliary velocity fusion
	float auxvel_delay_ms{5.0f};            ///< auxiliary velocity measurement delay relative to the IMU (mSec)
	const float auxvel_noise{0.5f};         ///< minimum observation noise, uses reported noise if greater (m/s)
	const float auxvel_gate{5.0f};          ///< velocity fusion innovation consistency gate size (STD)
#endif // CONFIG_EKF2_AUXVEL

};

union fault_status_u {
	struct {
		bool bad_mag_x         : 1; ///< 0 - true if the fusion of the magnetometer X-axis has encountered a numerical error
		bool bad_mag_y         : 1; ///< 1 - true if the fusion of the magnetometer Y-axis has encountered a numerical error
		bool bad_mag_z         : 1; ///< 2 - true if the fusion of the magnetometer Z-axis has encountered a numerical error
		bool bad_hdg           : 1; ///< 3 - true if the fusion of the heading angle has encountered a numerical error
		bool bad_mag_decl      : 1; ///< 4 - true if the fusion of the magnetic declination has encountered a numerical error
		bool bad_airspeed      : 1; ///< 5 - true if fusion of the airspeed has encountered a numerical error
bool bad_sideslip      :
		1; ///< 6 - true if fusion of the synthetic sideslip constraint has encountered a numerical error
		bool bad_optflow_X     : 1; ///< 7 - true if fusion of the optical flow X axis has encountered a numerical error
		bool bad_optflow_Y     : 1; ///< 8 - true if fusion of the optical flow Y axis has encountered a numerical error
		bool __UNUSED          : 1; ///< 9 -
		bool bad_acc_vertical  : 1; ///< 10 - true if bad vertical accelerometer data has been detected
		bool bad_acc_clipping  : 1; ///< 11 - true if delta velocity data contains clipping (asymmetric railing)
	} flags;
	uint32_t value;
};

// define structure used to communicate innovation test failures
union innovation_fault_status_u {
	struct {
		bool reject_hor_vel   : 1; ///< 0 - true if horizontal velocity observations have been rejected
		bool reject_ver_vel   : 1; ///< 1 - true if vertical velocity observations have been rejected
		bool reject_hor_pos   : 1; ///< 2 - true if horizontal position observations have been rejected
		bool reject_ver_pos   : 1; ///< 3 - true if true if vertical position observations have been rejected
		bool reject_mag_x     : 1; ///< 4 - true if the X magnetometer observation has been rejected
		bool reject_mag_y     : 1; ///< 5 - true if the Y magnetometer observation has been rejected
		bool reject_mag_z     : 1; ///< 6 - true if the Z magnetometer observation has been rejected
		bool reject_yaw       : 1; ///< 7 - true if the yaw observation has been rejected
		bool reject_airspeed  : 1; ///< 8 - true if the airspeed observation has been rejected
		bool reject_sideslip  : 1; ///< 9 - true if the synthetic sideslip observation has been rejected
		bool reject_hagl      : 1; ///< 10 - unused
		bool reject_optflow_X : 1; ///< 11 - true if the X optical flow observation has been rejected
		bool reject_optflow_Y : 1; ///< 12 - true if the Y optical flow observation has been rejected
	} flags;
	uint16_t value;
};

// publish the status of various GPS quality checks
union gps_check_fail_status_u {
	struct {
		uint16_t fix    : 1; ///< 0 - true if the fix type is insufficient (no 3D solution)
		uint16_t nsats  : 1; ///< 1 - true if number of satellites used is insufficient
		uint16_t pdop   : 1; ///< 2 - true if position dilution of precision is insufficient
		uint16_t hacc   : 1; ///< 3 - true if reported horizontal accuracy is insufficient
		uint16_t vacc   : 1; ///< 4 - true if reported vertical accuracy is insufficient
		uint16_t sacc   : 1; ///< 5 - true if reported speed accuracy is insufficient
		uint16_t hdrift : 1; ///< 6 - true if horizontal drift is excessive (can only be used when stationary on ground)
		uint16_t vdrift : 1; ///< 7 - true if vertical drift is excessive (can only be used when stationary on ground)
		uint16_t hspeed : 1; ///< 8 - true if horizontal speed is excessive (can only be used when stationary on ground)
		uint16_t vspeed : 1; ///< 9 - true if vertical speed error is excessive
		uint16_t spoofed: 1; ///< 10 - true if the GNSS data is spoofed
	} flags;
	uint16_t value;
};

// bitmask containing filter control status
union filter_control_status_u {
	struct {
		uint64_t tilt_align              : 1; ///< 0 - true if the filter tilt alignment is complete
		uint64_t yaw_align               : 1; ///< 1 - true if the filter yaw alignment is complete
		uint64_t gnss_pos                : 1; ///< 2 - true if GNSS position measurement fusion is intended
		uint64_t opt_flow                : 1; ///< 3 - true if optical flow measurements fusion is intended
		uint64_t mag_hdg                 : 1; ///< 4 - true if a simple magnetic yaw heading fusion is intended
		uint64_t mag_3D                  : 1; ///< 5 - true if 3-axis magnetometer measurement fusion is intended
		uint64_t mag_dec                 : 1; ///< 6 - true if synthetic magnetic declination measurements fusion is intended
		uint64_t in_air                  : 1; ///< 7 - true when the vehicle is airborne
		uint64_t wind                    : 1; ///< 8 - true when wind velocity is being estimated
		uint64_t baro_hgt                : 1; ///< 9 - true when baro data is being fused
uint64_t rng_hgt                 :
		1; ///< 10 - true when range finder data is being fused for height aiding
		uint64_t gps_hgt                 : 1; ///< 11 - true when GPS altitude is being fused
		uint64_t ev_pos                  : 1; ///< 12 - true when local position data fusion from external vision is intended
		uint64_t ev_yaw                  : 1; ///< 13 - true when yaw data from external vision measurements fusion is intended
		uint64_t ev_hgt                  : 1; ///< 14 - true when height data from external vision measurements is being fused
		uint64_t fuse_beta               : 1; ///< 15 - true when synthetic sideslip measurements are being fused
		uint64_t mag_field_disturbed     : 1; ///< 16 - true when the mag field does not match the expected strength
		uint64_t fixed_wing              : 1; ///< 17 - true when the vehicle is operating as a fixed wing vehicle
uint64_t mag_fault               :
		1; ///< 18 - true when the magnetometer has been declared faulty and is no longer being used
		uint64_t fuse_aspd               : 1; ///< 19 - true when airspeed measurements are being fused
uint64_t gnd_effect              :
		1; ///< 20 - true when protection from ground effect induced static pressure rise is active
uint64_t rng_stuck               :
		1; ///< 21 - true when rng data wasn't ready for more than 10s and new rng values haven't changed enough
uint64_t gnss_yaw                 :
		1; ///< 22 - true when yaw (not ground course) data fusion from a GPS receiver is intended
		uint64_t mag_aligned_in_flight   : 1; ///< 23 - true when the in-flight mag field alignment has been completed
uint64_t ev_vel                  :
		1; ///< 24 - true when local frame velocity data fusion from external vision measurements is intended
uint64_t synthetic_mag_z         :
		1; ///< 25 - true when we are using a synthesized measurement for the magnetometer Z component
		uint64_t vehicle_at_rest         : 1; ///< 26 - true when the vehicle is at rest
uint64_t gnss_yaw_fault           :
		1; ///< 27 - true when the GNSS heading has been declared faulty and is no longer being used
uint64_t rng_fault               :
		1; ///< 28 - true when the range finder has been declared faulty and is no longer being used
uint64_t inertial_dead_reckoning :
		1; ///< 29 - true if we are no longer fusing measurements that constrain horizontal velocity drift
		uint64_t wind_dead_reckoning     : 1; ///< 30 - true if we are navigationg reliant on wind relative measurements
		uint64_t rng_kin_consistent      : 1; ///< 31 - true when the range finder kinematic consistency check is passing
		uint64_t fake_pos                : 1; ///< 32 - true when fake position measurements are being fused
		uint64_t fake_hgt                : 1; ///< 33 - true when fake height measurements are being fused
		uint64_t gravity_vector          : 1; ///< 34 - true when gravity vector measurements are being fused
uint64_t mag                     :
		1; ///< 35 - true if 3-axis magnetometer measurement fusion (mag states only) is intended
uint64_t ev_yaw_fault            :
		1; ///< 36 - true when the EV heading has been declared faulty and is no longer being used
uint64_t mag_heading_consistent  :
		1; ///< 37 - true when the heading obtained from mag data is declared consistent with the filter
		uint64_t aux_gpos                : 1; ///< 38 - true if auxiliary global position measurement fusion is intended
		uint64_t rng_terrain             : 1; ///< 39 - true if we are fusing range finder data for terrain
		uint64_t opt_flow_terrain        : 1; ///< 40 - true if we are fusing flow data for terrain
		uint64_t valid_fake_pos          : 1; ///< 41 - true if a valid constant position is being fused
		uint64_t constant_pos            : 1; ///< 42 - true if the vehicle is at a constant position
		uint64_t baro_fault              : 1; ///< 43 - true when the baro has been declared faulty and is no longer being used
		uint64_t gnss_vel                : 1; ///< 44 - true if GNSS velocity measurement fusion is intended
	} flags;
	uint64_t value;
};

// define structure used to communicate information events
union information_event_status_u {
	struct {
		bool gps_checks_passed          : 1; ///< 0 - true when gps quality checks are passing passed
		bool reset_vel_to_gps           : 1; ///< 1 - true when the velocity states are reset to the gps measurement
		bool reset_vel_to_flow          : 1; ///< 2 - true when the velocity states are reset using the optical flow measurement
		bool reset_vel_to_vision        : 1; ///< 3 - true when the velocity states are reset to the vision system measurement
		bool reset_vel_to_zero          : 1; ///< 4  - true when the velocity states are reset to zero
		bool reset_pos_to_last_known    : 1; ///< 5 - true when the position states are reset to the last known position
		bool reset_pos_to_gps           : 1; ///< 6 - true when the position states are reset to the gps measurement
		bool reset_pos_to_vision        : 1; ///< 7 - true when the position states are reset to the vision system measurement
bool starting_gps_fusion        :
		1; ///< 8 - true when the filter starts using gps measurements to correct the state estimates
bool starting_vision_pos_fusion :
		1; ///< 9 - true when the filter starts using vision system position measurements to correct the state estimates
bool starting_vision_vel_fusion :
		1; ///< 10 - true when the filter starts using vision system velocity measurements to correct the state estimates
bool starting_vision_yaw_fusion :
		1; ///< 11 - true when the filter starts using vision system yaw  measurements to correct the state estimates
bool yaw_aligned_to_imu_gps     :
		1; ///< 12 - true when the filter resets the yaw to an estimate derived from IMU and GPS data
		bool reset_hgt_to_baro          : 1; ///< 13 - true when the vertical position state is reset to the baro measurement
		bool reset_hgt_to_gps           : 1; ///< 14 - true when the vertical position state is reset to the gps measurement
		bool reset_hgt_to_rng           : 1; ///< 15 - true when the vertical position state is reset to the rng measurement
		bool reset_hgt_to_ev            : 1; ///< 16 - true when the vertical position state is reset to the ev measurement
bool reset_pos_to_ext_obs       :
		1; ///< 17 - true when horizontal position was reset to an external observation while deadreckoning
		bool reset_wind_to_ext_obs 	: 1; ///< 18 - true when wind states were reset to an external observation
	} flags;
	uint32_t value;
};

}
#endif // !EKF_COMMON_H
