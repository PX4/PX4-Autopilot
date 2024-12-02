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
 * @file ekf.h
 * Class for core functions for ekf attitude and position estimator.
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#ifndef EKF_EKF_H
#define EKF_EKF_H

#include "estimator_interface.h"

#if defined(CONFIG_EKF2_GNSS)
# include "yaw_estimator/EKFGSF_yaw.h"
#endif // CONFIG_EKF2_GNSS

#include "bias_estimator/bias_estimator.hpp"
#include "bias_estimator/height_bias_estimator.hpp"
#include "bias_estimator/position_bias_estimator.hpp"

#include <ekf_derivation/generated/state.h>

#include <uORB/topics/estimator_aid_source1d.h>
#include <uORB/topics/estimator_aid_source2d.h>
#include <uORB/topics/estimator_aid_source3d.h>

#include "aid_sources/ZeroGyroUpdate.hpp"
#include "aid_sources/ZeroVelocityUpdate.hpp"

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION)
# include "aid_sources/aux_global_position/aux_global_position.hpp"
#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION

enum class Likelihood { LOW, MEDIUM, HIGH };
class ExternalVisionVel;

class Ekf final : public EstimatorInterface
{
public:
	typedef matrix::Vector<float, State::size> VectorState;
	typedef matrix::SquareMatrix<float, State::size> SquareMatrixState;

	Ekf()
	{
		reset();
	};

	virtual ~Ekf() = default;

	// initialise variables to sane values (also interface class)
	bool init(uint64_t timestamp) override;

	void print_status();

	// should be called every time new data is pushed into the filter
	bool update();

	const StateSample &state() const { return _state; }

#if defined(CONFIG_EKF2_BAROMETER)
	const auto &aid_src_baro_hgt() const { return _aid_src_baro_hgt; }
	const BiasEstimator::status &getBaroBiasEstimatorStatus() const { return _baro_b_est.getStatus(); }
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_TERRAIN)
	// terrain estimate
	bool isTerrainEstimateValid() const { return _terrain_valid; }

	// get the estimated terrain vertical position relative to the NED origin
	float getTerrainVertPos() const { return _state.terrain + getEkfGlobalOriginAltitude(); };
	float getHagl() const { return _state.terrain + _gpos.altitude(); }

	// get the terrain variance
	float getTerrainVariance() const { return P(State::terrain.idx, State::terrain.idx); }

#endif // CONFIG_EKF2_TERRAIN

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// range height
	const auto &aid_src_rng_hgt() const { return _aid_src_rng_hgt; }

	float getHaglRateInnov() const { return _rng_consistency_check.getInnov(); }
	float getHaglRateInnovVar() const { return _rng_consistency_check.getInnovVar(); }
	float getHaglRateInnovRatio() const { return _rng_consistency_check.getSignedTestRatioLpf(); }
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	const auto &aid_src_optical_flow() const { return _aid_src_optical_flow; }

	const Vector2f &getFlowVelBody() const { return _flow_vel_body; }
	Vector2f getFlowVelNE() const { return Vector2f(_R_to_earth * Vector3f(getFlowVelBody()(0), getFlowVelBody()(1), 0.f)); }

	const Vector2f &getFilteredFlowVelBody() const { return _flow_vel_body_lpf.getState(); }
	Vector2f getFilteredFlowVelNE() const { return Vector2f(_R_to_earth * Vector3f(getFilteredFlowVelBody()(0), getFilteredFlowVelBody()(1), 0.f)); }

	const Vector2f &getFlowCompensated() const { return _flow_rate_compensated; }
	const Vector2f &getFlowUncompensated() const { return _flow_sample_delayed.flow_rate; }

	const Vector3f getFlowGyro() const { return _flow_sample_delayed.gyro_rate; }
	const Vector3f &getFlowGyroBias() const { return _flow_gyro_bias; }
	const Vector3f &getFlowRefBodyRate() const { return _ref_body_rate; }
#endif // CONFIG_EKF2_OPTICAL_FLOW

	float getHeadingInnov() const;
	float getHeadingInnovVar() const;
	float getHeadingInnovRatio() const;

#if defined(CONFIG_EKF2_DRAG_FUSION)
	const auto &aid_src_drag() const { return _aid_src_drag; }
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	const auto &aid_src_gravity() const { return _aid_src_gravity; }
#endif // CONFIG_EKF2_GRAVITY_FUSION

#if defined(CONFIG_EKF2_WIND)
	// get the wind velocity in m/s
	const Vector2f &getWindVelocity() const { return _state.wind_vel; };
	Vector2f getWindVelocityVariance() const { return getStateVariance<State::wind_vel>(); }

	/**
	* @brief Resets the wind states to an external observation
	*
	* @param wind_speed The wind speed in m/s
	* @param wind_direction The azimuth (from true north) to where the wind is heading in radians
	* @param wind_speed_accuracy The 1 sigma accuracy of the wind speed estimate in m/s
	* @param wind_direction_accuracy The 1 sigma accuracy of the wind direction estimate in radians
	*/
	void resetWindToExternalObservation(float wind_speed, float wind_direction, float wind_speed_accuracy,
					    float wind_direction_accuracy);
#endif // CONFIG_EKF2_WIND

	template <const IdxDof &S>
	matrix::Vector<float, S.dof>getStateVariance() const { return P.slice<S.dof, S.dof>(S.idx, S.idx).diag(); } // calling getStateCovariance().diag() uses more flash space

	template <const IdxDof &S>
	matrix::SquareMatrix<float, S.dof>getStateCovariance() const { return P.slice<S.dof, S.dof>(S.idx, S.idx); }

	// get the full covariance matrix
	const matrix::SquareMatrix<float, State::size> &covariances() const { return P; }
	float stateCovariance(unsigned r, unsigned c) const { return P(r, c); }

	// get the diagonal elements of the covariance matrix
	matrix::Vector<float, State::size> covariances_diagonal() const { return P.diag(); }

	matrix::Vector3f getRotVarBody() const;
	matrix::Vector3f getRotVarNed() const;
	float getYawVar() const;
	float getTiltVariance() const;

	Vector3f getVelocityVariance() const { return getStateVariance<State::vel>(); };

	Vector3f getPositionVariance() const { return getStateVariance<State::pos>(); }

	// get the ekf WGS-84 origin position and height and the system time it was last set
	void getEkfGlobalOrigin(uint64_t &origin_time, double &latitude, double &longitude, float &origin_alt) const;
	bool checkLatLonValidity(double latitude, double longitude);
	bool checkAltitudeValidity(float altitude);
	bool setEkfGlobalOrigin(double latitude, double longitude, float altitude, float hpos_var = NAN, float vpos_var = NAN);
	bool resetGlobalPositionTo(double latitude, double longitude, float altitude, float hpos_var = NAN,
				   float vpos_var = NAN);

	// get the 1-sigma horizontal and vertical position uncertainty of the ekf WGS-84 position
	void get_ekf_gpos_accuracy(float *ekf_eph, float *ekf_epv) const;

	// get the 1-sigma horizontal and vertical position uncertainty of the ekf local position
	void get_ekf_lpos_accuracy(float *ekf_eph, float *ekf_epv) const;

	// get the 1-sigma horizontal and vertical velocity uncertainty
	void get_ekf_vel_accuracy(float *ekf_evh, float *ekf_evv) const;

	// Returns the following vehicle control limits required by the estimator to keep within sensor limitations.
	//  vxy_max : Maximum ground relative horizontal speed (meters/sec). NaN when limiting is not needed.
	//  vz_max : Maximum ground relative vertical speed (meters/sec). NaN when limiting is not needed.
	//  hagl_min : Minimum height above ground (meters). NaN when limiting is not needed.
	// hagl_max : Maximum height above ground (meters). NaN when limiting is not needed.
	void get_ekf_ctrl_limits(float *vxy_max, float *vz_max, float *hagl_min, float *hagl_max) const;

	void resetGyroBias();
	void resetGyroBiasCov();

	void resetAccelBias();
	void resetAccelBiasCov();

	bool isGlobalHorizontalPositionValid() const
	{
		return _local_origin_lat_lon.isInitialized() && isLocalHorizontalPositionValid();
	}

	bool isGlobalVerticalPositionValid() const
	{
		return PX4_ISFINITE(_local_origin_alt) && isLocalVerticalPositionValid();
	}

	bool isLocalHorizontalPositionValid() const
	{
		return !_horizontal_deadreckon_time_exceeded;
	}

	bool isLocalVerticalPositionValid() const
	{
		return !_vertical_position_deadreckon_time_exceeded;
	}

	bool isLocalVerticalVelocityValid() const
	{
		return !_vertical_velocity_deadreckon_time_exceeded;
	}

	bool isYawFinalAlignComplete() const
	{
#if defined(CONFIG_EKF2_MAGNETOMETER)
		const bool is_using_mag = (_control_status.flags.mag_3D || _control_status.flags.mag_hdg);
		const bool is_mag_alignment_in_flight_complete = is_using_mag
				&& _control_status.flags.mag_aligned_in_flight
				&& ((_time_delayed_us - _flt_mag_align_start_time) > (uint64_t)1e6);
		return _control_status.flags.yaw_align
		       && (is_mag_alignment_in_flight_complete || !is_using_mag);
#else
		return _control_status.flags.yaw_align;
#endif
	}

	// fuse single direct state measurement (eg NED velocity, NED position, mag earth field, etc)
	void fuseDirectStateMeasurement(const float innov, const float innov_var, const float R, const int state_index);

	bool measurementUpdate(VectorState &K, const VectorState &H, const float R, const float innovation);

	// gyro bias
	const Vector3f &getGyroBias() const { return _state.gyro_bias; } // get the gyroscope bias in rad/s
	Vector3f getGyroBiasVariance() const { return getStateVariance<State::gyro_bias>(); } // get the gyroscope bias variance in rad/s
	float getGyroBiasLimit() const { return _params.gyro_bias_lim; }
	float getGyroNoise() const { return _params.gyro_noise; }

	// accel bias
	const Vector3f &getAccelBias() const { return _state.accel_bias; } // get the accelerometer bias in m/s**2
	Vector3f getAccelBiasVariance() const { return getStateVariance<State::accel_bias>(); } // get the accelerometer bias variance in m/s**2
	float getAccelBiasLimit() const { return _params.acc_bias_lim; }

#if defined(CONFIG_EKF2_MAGNETOMETER)
	const Vector3f &getMagEarthField() const { return _state.mag_I; }

	const Vector3f &getMagBias() const { return _state.mag_B; }
	Vector3f getMagBiasVariance() const { return getStateVariance<State::mag_B>(); } // get the mag bias variance in Gauss
	float getMagBiasLimit() const { return 0.5f; } // 0.5 Gauss
#endif // CONFIG_EKF2_MAGNETOMETER

	bool accel_bias_inhibited() const { return _accel_bias_inhibit[0] || _accel_bias_inhibit[1] || _accel_bias_inhibit[2]; }
	bool gyro_bias_inhibited() const { return _gyro_bias_inhibit[0] || _gyro_bias_inhibit[1] || _gyro_bias_inhibit[2]; }

	const auto &state_reset_status() const { return _state_reset_status; }

	// return the amount the local vertical position changed in the last reset and the number of reset events
	uint8_t get_posD_reset_count() const { return _state_reset_status.reset_count.posD; }
	void get_posD_reset(float *delta, uint8_t *counter) const
	{
		*delta = _state_reset_status.posD_change;
		*counter = _state_reset_status.reset_count.posD;
	}

	uint8_t get_hagl_reset_count() const { return _state_reset_status.reset_count.hagl; }
	void get_hagl_reset(float *delta, uint8_t *counter) const
	{
		*delta = _state_reset_status.hagl_change;
		*counter = _state_reset_status.reset_count.hagl;
	}

	// return the amount the local vertical velocity changed in the last reset and the number of reset events
	uint8_t get_velD_reset_count() const { return _state_reset_status.reset_count.velD; }
	void get_velD_reset(float *delta, uint8_t *counter) const
	{
		*delta = _state_reset_status.velD_change;
		*counter = _state_reset_status.reset_count.velD;
	}

	// return the amount the local horizontal position changed in the last reset and the number of reset events
	uint8_t get_posNE_reset_count() const { return _state_reset_status.reset_count.posNE; }
	void get_posNE_reset(float delta[2], uint8_t *counter) const
	{
		_state_reset_status.posNE_change.copyTo(delta);
		*counter = _state_reset_status.reset_count.posNE;
	}

	// return the amount the local horizontal velocity changed in the last reset and the number of reset events
	uint8_t get_velNE_reset_count() const { return _state_reset_status.reset_count.velNE; }
	void get_velNE_reset(float delta[2], uint8_t *counter) const
	{
		_state_reset_status.velNE_change.copyTo(delta);
		*counter = _state_reset_status.reset_count.velNE;
	}

	// return the amount the quaternion has changed in the last reset and the number of reset events
	uint8_t get_quat_reset_count() const { return _state_reset_status.reset_count.quat; }
	void get_quat_reset(float delta_quat[4], uint8_t *counter) const
	{
		_state_reset_status.quat_change.copyTo(delta_quat);
		*counter = _state_reset_status.reset_count.quat;
	}

	float getHeadingInnovationTestRatio() const;

	float getHorizontalVelocityInnovationTestRatio() const;
	float getVerticalVelocityInnovationTestRatio() const;

	float getHorizontalPositionInnovationTestRatio() const;
	float getVerticalPositionInnovationTestRatio() const;

	float getAirspeedInnovationTestRatio() const;
	float getSyntheticSideslipInnovationTestRatio() const;

	float getHeightAboveGroundInnovationTestRatio() const;

	// return a bitmask integer that describes which state estimates are valid
	uint16_t get_ekf_soln_status() const;

	HeightSensor getHeightSensorRef() const { return _height_sensor_ref; }

#if defined(CONFIG_EKF2_AIRSPEED)
	const auto &aid_src_airspeed() const { return _aid_src_airspeed; }
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	const auto &aid_src_sideslip() const { return _aid_src_sideslip; }
#endif // CONFIG_EKF2_SIDESLIP

	const auto &aid_src_fake_hgt() const { return _aid_src_fake_hgt; }
	const auto &aid_src_fake_pos() const { return _aid_src_fake_pos; }

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	const auto &aid_src_ev_hgt() const { return _aid_src_ev_hgt; }
	const auto &aid_src_ev_pos() const { return _aid_src_ev_pos; }
	const auto &aid_src_ev_vel() const { return _aid_src_ev_vel; }
	const auto &aid_src_ev_yaw() const { return _aid_src_ev_yaw; }

	const BiasEstimator::status &getEvHgtBiasEstimatorStatus() const { return _ev_hgt_b_est.getStatus(); }
	const BiasEstimator::status &getEvPosBiasEstimatorStatus(int i) const { return _ev_pos_b_est.getStatus(i); }
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_GNSS)
	// set minimum continuous period without GPS fail required to mark a healthy GPS status
	void set_min_required_gps_health_time(uint32_t time_us) { _min_gps_health_time_us = time_us; }

	const gps_check_fail_status_u &gps_check_fail_status() const { return _gps_check_fail_status; }
	const decltype(gps_check_fail_status_u::flags) &gps_check_fail_status_flags() const { return _gps_check_fail_status.flags; }

	bool gps_checks_passed() const { return _gps_checks_passed; };

	const BiasEstimator::status &getGpsHgtBiasEstimatorStatus() const { return _gps_hgt_b_est.getStatus(); }

	const auto &aid_src_gnss_hgt() const { return _aid_src_gnss_hgt; }
	const auto &aid_src_gnss_pos() const { return _aid_src_gnss_pos; }
	const auto &aid_src_gnss_vel() const { return _aid_src_gnss_vel; }

# if defined(CONFIG_EKF2_GNSS_YAW)
	const auto &aid_src_gnss_yaw() const { return _aid_src_gnss_yaw; }
# endif // CONFIG_EKF2_GNSS_YAW

	// Returns true if the output of the yaw emergency estimator can be used for a reset
	bool isYawEmergencyEstimateAvailable() const;

	// get solution data from the EKF-GSF emergency yaw estimator
	// returns false when data is not available
	bool getDataEKFGSF(float *yaw_composite, float *yaw_variance, float yaw[N_MODELS_EKFGSF],
			   float innov_VN[N_MODELS_EKFGSF], float innov_VE[N_MODELS_EKFGSF], float weight[N_MODELS_EKFGSF]);

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// set the magnetic field data returned by the geo library using position
	bool updateWorldMagneticModel(const double latitude_deg, const double longitude_deg);

	const auto &aid_src_mag() const { return _aid_src_mag; }
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_AUXVEL)
	const auto &aid_src_aux_vel() const { return _aid_src_aux_vel; }
#endif // CONFIG_EKF2_AUXVEL

	bool resetGlobalPosToExternalObservation(double latitude, double longitude, float altitude, float eph, float epv,
			uint64_t timestamp_observation);

	void updateParameters();

	friend class AuxGlobalPosition;

private:

	friend class ExternalVisionVel;
	friend class EvVelBodyFrameFrd;
	friend class EvVelLocalFrameNed;
	friend class EvVelLocalFrameFrd;

	// set the internal states and status to their default value
	void reset();

	bool initialiseTilt();

	// check if the EKF is dead reckoning horizontal velocity using inertial data only
	void updateDeadReckoningStatus();
	void updateHorizontalDeadReckoningstatus();
	void updateVerticalDeadReckoningStatus();

	static constexpr float kGyroBiasVarianceMin{1e-9f};
	static constexpr float kAccelBiasVarianceMin{1e-9f};

#if defined(CONFIG_EKF2_MAGNETOMETER)
	static constexpr float kMagVarianceMin = 1e-6f;
#endif // CONFIG_EKF2_MAGNETOMETER


	struct StateResetCounts {
		uint8_t velNE{0};	///< number of horizontal position reset events (allow to wrap if count exceeds 255)
		uint8_t velD{0};	///< number of vertical velocity reset events (allow to wrap if count exceeds 255)
		uint8_t posNE{0};	///< number of horizontal position reset events (allow to wrap if count exceeds 255)
		uint8_t posD{0};	///< number of vertical position reset events (allow to wrap if count exceeds 255)
		uint8_t quat{0};	///< number of quaternion reset events (allow to wrap if count exceeds 255)
		uint8_t hagl{0};	///< number of height above ground level reset events (allow to wrap if count exceeds 255)
	};

	struct StateResets {
		Vector2f velNE_change;  ///< North East velocity change due to last reset (m)
		float velD_change;	///< Down velocity change due to last reset (m/sec)
		Vector2f posNE_change;	///< North, East position change due to last reset (m)
		float posD_change;	///< Down position change due to last reset (m)
		Quatf quat_change;	///< quaternion delta due to last reset - multiply pre-reset quaternion by this to get post-reset quaternion
		float hagl_change;	///< Height above ground level change due to last reset (m)

		StateResetCounts reset_count{};
	};

	StateResets _state_reset_status{};	///< reset event monitoring structure containing velocity, position, height and yaw reset information
	StateResetCounts _state_reset_count_prev{};

	StateSample _state{};		///< state struct of the ekf running at the delayed time horizon

	LatLonAlt _gpos{0.0, 0.0, 0.f};

	bool _filter_initialised{false};	///< true when the EKF sttes and covariances been initialised

	uint64_t _time_last_horizontal_aiding{0}; ///< amount of time we have been doing inertial only deadreckoning (uSec)
	uint64_t _time_last_v_pos_aiding{0};
	uint64_t _time_last_v_vel_aiding{0};

	uint64_t _time_last_hor_pos_fuse{0};	///< time the last fusion of horizontal position measurements was performed (uSec)
	uint64_t _time_last_hgt_fuse{0};	///< time the last fusion of vertical position measurements was performed (uSec)
	uint64_t _time_last_hor_vel_fuse{0};	///< time the last fusion of horizontal velocity measurements was performed (uSec)
	uint64_t _time_last_ver_vel_fuse{0};	///< time the last fusion of verticalvelocity measurements was performed (uSec)
	uint64_t _time_last_heading_fuse{0};
	uint64_t _time_last_terrain_fuse{0};

	LatLonAlt _last_known_gpos{};

	Vector3f _earth_rate_NED{}; ///< earth rotation vector (NED) in rad/s
	double _earth_rate_lat_ref_rad{0.0}; ///< latitude at which the earth rate was evaluated (radians)

	Dcmf _R_to_earth{};	///< transformation matrix from body frame to earth frame from last EKF prediction

	Vector2f _accel_lpf_NE{};			///< Low pass filtered horizontal earth frame acceleration (m/sec**2)
	float _height_rate_lpf{0.0f};

	SquareMatrixState P{};	///< state covariance matrix

#if defined(CONFIG_EKF2_DRAG_FUSION)
	estimator_aid_source2d_s _aid_src_drag {};
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_TERRAIN)
	// Terrain height state estimation
	float _last_on_ground_posD{0.0f};	///< last vertical position when the in_air status was false (m)

	bool _terrain_valid{false};
#endif // CONFIG_EKF2_TERRAIN

#if defined(CONFIG_EKF2_RANGE_FINDER)
	estimator_aid_source1d_s _aid_src_rng_hgt {};
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	estimator_aid_source2d_s _aid_src_optical_flow {};

	// optical flow processing
	Vector3f _flow_gyro_bias{};	///< bias errors in optical flow sensor rate gyro outputs (rad/sec)
	Vector3f _ref_body_rate{};

	Vector2f _flow_vel_body{};                      ///< velocity from corrected flow measurement (body frame)(m/s)
	AlphaFilter<Vector2f> _flow_vel_body_lpf{0.1f}; ///< filtered velocity from corrected flow measurement (body frame)(m/s)
	uint32_t _flow_counter{0};                      ///< number of flow samples read for initialization

	Vector2f _flow_rate_compensated{}; ///< measured angular rate of the image about the X and Y body axes after removal of body rotation (rad/s), RH rotation is positive
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_AIRSPEED)
	estimator_aid_source1d_s _aid_src_airspeed {};
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_SIDESLIP)
	estimator_aid_source1d_s _aid_src_sideslip {};
#endif // CONFIG_EKF2_SIDESLIP

	estimator_aid_source2d_s _aid_src_fake_pos{};
	estimator_aid_source1d_s _aid_src_fake_hgt{};

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	estimator_aid_source1d_s _aid_src_ev_hgt {};
	estimator_aid_source2d_s _aid_src_ev_pos{};
	estimator_aid_source3d_s _aid_src_ev_vel{};
	estimator_aid_source1d_s _aid_src_ev_yaw{};

	uint8_t _nb_ev_pos_reset_available{0};
	uint8_t _nb_ev_vel_reset_available{0};
	uint8_t _nb_ev_yaw_reset_available{0};
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_GNSS)
	bool _gps_data_ready {false};	///< true when new GPS data has fallen behind the fusion time horizon and is available to be fused

	// variables used for the GPS quality checks
	Vector3f _gps_pos_deriv_filt{};	///< GPS NED position derivative (m/sec)
	Vector2f _gps_velNE_filt{};	///< filtered GPS North and East velocity (m/sec)

	float _gps_vel_d_filt{0.0f};		///< GNSS filtered Down velocity (m/sec)
	uint64_t _last_gps_fail_us{0};		///< last system time in usec that the GPS failed it's checks
	uint64_t _last_gps_pass_us{0};		///< last system time in usec that the GPS passed it's checks
	uint32_t _min_gps_health_time_us{10000000}; ///< GPS is marked as healthy only after this amount of time
	bool _gps_checks_passed{false};		///> true when all active GPS checks have passed

	gps_check_fail_status_u _gps_check_fail_status{};
	// height sensor status
	bool _gps_intermittent{true};           ///< true if data into the buffer is intermittent

	HeightBiasEstimator _gps_hgt_b_est{HeightSensor::GNSS, _height_sensor_ref};

	estimator_aid_source1d_s _aid_src_gnss_hgt{};
	estimator_aid_source2d_s _aid_src_gnss_pos{};
	estimator_aid_source3d_s _aid_src_gnss_vel{};

# if defined(CONFIG_EKF2_GNSS_YAW)
	estimator_aid_source1d_s _aid_src_gnss_yaw {};
# endif // CONFIG_EKF2_GNSS_YAW
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	estimator_aid_source3d_s _aid_src_gravity {};
#endif // CONFIG_EKF2_GRAVITY_FUSION

#if defined(CONFIG_EKF2_AUXVEL)
	estimator_aid_source2d_s _aid_src_aux_vel {};
#endif // CONFIG_EKF2_AUXVEL

	// Variables used by the initial filter alignment
	bool _is_first_imu_sample{true};
	AlphaFilter<Vector3f> _accel_lpf{0.1f};	///< filtered accelerometer measurement used to align tilt (m/s/s)
	AlphaFilter<Vector3f> _gyro_lpf{0.1f};	///< filtered gyro measurement used for alignment excessive movement check (rad/sec)

#if defined(CONFIG_EKF2_BAROMETER)
	estimator_aid_source1d_s _aid_src_baro_hgt {};

	// Variables used to perform in flight resets and switch between height sources
	AlphaFilter<float> _baro_lpf{0.1f};	///< filtered barometric height measurement (m)
	uint32_t _baro_counter{0};		///< number of baro samples read during initialisation

	HeightBiasEstimator _baro_b_est{HeightSensor::BARO, _height_sensor_ref};

	bool _baro_hgt_faulty{false};		///< true if baro data have been declared faulty TODO: move to fault flags
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// used by magnetometer fusion mode selection
	AlphaFilter<float> _mag_heading_innov_lpf{0.1f};
	uint32_t _min_mag_health_time_us{1'000'000}; ///< magnetometer is marked as healthy only after this amount of time

	estimator_aid_source3d_s _aid_src_mag{};

	AlphaFilter<Vector3f> _mag_lpf{0.1f};	///< filtered magnetometer measurement for instant reset (Gauss)
	uint32_t _mag_counter{0};		///< number of magnetometer samples read during initialisation

	// Variables used to control activation of post takeoff functionality
	uint64_t _flt_mag_align_start_time{0};	///< time that inflight magnetic field alignment started (uSec)
	uint64_t _time_last_mag_check_failing{0};
#endif // CONFIG_EKF2_MAGNETOMETER

	// variables used to inhibit accel bias learning
	bool _accel_bias_inhibit[3] {};		///< true when the accel bias learning is being inhibited for the specified axis
	bool _gyro_bias_inhibit[3] {};		///< true when the gyro bias learning is being inhibited for the specified axis
	float _accel_magnitude_filt{0.0f};	///< acceleration magnitude after application of a decaying envelope filter (rad/sec)
	float _ang_rate_magnitude_filt{0.0f};		///< angular rate magnitude after application of a decaying envelope filter (rad/sec)

	// imu fault status
	uint64_t _time_bad_vert_accel{0};	///< last time a bad vertical accel was detected (uSec)
	uint64_t _time_good_vert_accel{0};	///< last time a good vertical accel was detected (uSec)
	uint16_t _clip_counter[3];		///< counter per axis that increments when clipping ad decrements when not

	// initialise filter states of both the delayed ekf and the real time complementary filter
	bool initialiseFilter(void);

	// initialise ekf covariance matrix
	void initialiseCovariance();

	// predict ekf state
	void predictState(const imuSample &imu_delayed);

	// predict ekf covariance
	void predictCovariance(const imuSample &imu_delayed);

	template <const IdxDof &S>
	void resetStateCovariance(const matrix::SquareMatrix<float, S.dof> &cov)
	{
		P.uncorrelateCovarianceSetVariance<S.dof>(S.idx, 0.0f);
		P.slice<S.dof, S.dof>(S.idx, S.idx) = cov;
	}

	bool setLatLonOrigin(double latitude, double longitude, float hpos_var = NAN);
	bool setAltOrigin(float altitude, float vpos_var = NAN);

	bool resetLatLonTo(double latitude, double longitude, float hpos_var = NAN);
	bool initialiseAltitudeTo(float altitude, float vpos_var = NAN);

	// update quaternion states and covariances using an innovation, observation variance and Jacobian vector
	bool fuseYaw(estimator_aid_source1d_s &aid_src_status, const VectorState &H_YAW);
	void computeYawInnovVarAndH(float variance, float &innovation_variance, VectorState &H_YAW) const;

	void updateIMUBiasInhibit(const imuSample &imu_delayed);

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// ekf sequential fusion of magnetometer measurements
	bool fuseMag(const Vector3f &mag, const float R_MAG, VectorState &H, estimator_aid_source3d_s &aid_src,
		     bool update_all_states = false, bool update_tilt = false);

	// fuse magnetometer declination measurement
	//  R: declination observation variance (rad**2)
	bool fuseDeclination(const float decl_measurement_rad, const float R, bool update_all_states = false);

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_AIRSPEED)
	// control fusion of air data observations
	void controlAirDataFusion(const imuSample &imu_delayed);

	void updateAirspeed(const airspeedSample &airspeed_sample, estimator_aid_source1d_s &aid_src) const;
	void fuseAirspeed(const airspeedSample &airspeed_sample, estimator_aid_source1d_s &aid_src);

	void stopAirspeedFusion();

	// Reset the wind states using the current airspeed measurement, ground relative nav velocity, yaw angle and assumption of zero sideslip
	void resetWindUsingAirspeed(const airspeedSample &airspeed_sample);
	void resetVelUsingAirspeed(const airspeedSample &airspeed_sample);
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	// control fusion of synthetic sideslip observations
	void controlBetaFusion(const imuSample &imu_delayed);

	// fuse synthetic zero sideslip measurement
	void updateSideslip(estimator_aid_source1d_s &_aid_src_sideslip) const;
	bool fuseSideslip(estimator_aid_source1d_s &_aid_src_sideslip);
#endif // CONFIG_EKF2_SIDESLIP

#if defined(CONFIG_EKF2_DRAG_FUSION)
	// control fusion of multi-rotor drag specific force observations
	void controlDragFusion(const imuSample &imu_delayed);

	// fuse body frame drag specific forces for multi-rotor wind estimation
	void fuseDrag(const dragSample &drag_sample);
#endif // CONFIG_EKF2_DRAG_FUSION

	void resetVelocityTo(const Vector3f &vel, const Vector3f &new_vel_var);

	void resetHorizontalVelocityTo(const Vector2f &new_horz_vel, const Vector2f &new_horz_vel_var);
	void resetHorizontalVelocityTo(const Vector2f &new_horz_vel, float vel_var) { resetHorizontalVelocityTo(new_horz_vel, Vector2f(vel_var, vel_var)); }

	void resetHorizontalVelocityToZero();

	void resetVerticalVelocityTo(float new_vert_vel, float new_vert_vel_var);


	void resetHorizontalPositionToLastKnown();

	void resetHorizontalPositionTo(const double &new_latitude, const double &new_longitude,
				       const Vector2f &new_horz_pos_var);
	void resetHorizontalPositionTo(const double &new_latitude, const double &new_longitude, const float pos_var = NAN) { resetHorizontalPositionTo(new_latitude, new_longitude, Vector2f(pos_var, pos_var)); }
	void resetHorizontalPositionTo(const Vector2f &new_pos, const Vector2f &new_horz_pos_var);

	Vector2f getLocalHorizontalPosition() const;

	Vector2f computeDeltaHorizontalPosition(const double &new_latitude, const double &new_longitude) const;
	void updateHorizontalPositionResetStatus(const Vector2f &delta);

	void resetWindTo(const Vector2f &wind, const Vector2f &wind_var);

	bool isHeightResetRequired() const;

	void resetAltitudeTo(float new_altitude, float new_vert_pos_var = NAN);
	void updateVerticalPositionResetStatus(const float delta_z);

	void resetVerticalVelocityToZero();

	// horizontal and vertical position aid source
	void updateVerticalPositionAidStatus(estimator_aid_source1d_s &aid_src, const uint64_t &time_us,
					     const float observation, const float observation_variance, const float innovation_gate = 1.f) const;

	// horizontal and vertical position fusion
	bool fuseHorizontalPosition(estimator_aid_source2d_s &pos_aid_src);
	bool fuseVerticalPosition(estimator_aid_source1d_s &hgt_aid_src);

	// 2d & 3d velocity fusion
	bool fuseHorizontalVelocity(estimator_aid_source2d_s &vel_aid_src);
	bool fuseVelocity(estimator_aid_source3d_s &vel_aid_src);

#if defined(CONFIG_EKF2_TERRAIN)
	void initTerrain();
	float getTerrainVPos() const { return isTerrainEstimateValid() ? _state.terrain : _last_on_ground_posD; }
	void controlTerrainFakeFusion();

	void updateTerrainValidity();
	void updateTerrainResetStatus(const float delta_z);

# if defined(CONFIG_EKF2_RANGE_FINDER)
	// update the terrain vertical position estimate using a height above ground measurement from the range finder
	bool fuseHaglRng(estimator_aid_source1d_s &aid_src, bool update_height, bool update_terrain);
	void updateRangeHagl(estimator_aid_source1d_s &aid_src);
	void resetTerrainToRng(estimator_aid_source1d_s &aid_src);
	float getRngVar() const;
# endif // CONFIG_EKF2_RANGE_FINDER

# if defined(CONFIG_EKF2_OPTICAL_FLOW)
	void resetTerrainToFlow();
# endif // CONFIG_EKF2_OPTICAL_FLOW

#endif // CONFIG_EKF2_TERRAIN

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// range height
	void controlRangeHaglFusion(const imuSample &imu_delayed);
	bool isConditionalRangeAidSuitable();
	void stopRngHgtFusion();
	void stopRngTerrFusion();
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// control fusion of optical flow observations
	void controlOpticalFlowFusion(const imuSample &imu_delayed);
	void resetFlowFusion(const flowSample &flow_sample);
	void stopFlowFusion();

	void updateOnGroundMotionForOpticalFlowChecks();
	void resetOnGroundMotionForOpticalFlowChecks();

	// calculate the measurement variance for the optical flow sensor (rad/sec)^2
	float calcOptFlowMeasVar(const flowSample &flow_sample) const;

	// calculate optical flow body angular rate compensation
	void calcOptFlowBodyRateComp(const flowSample &flow_sample);

	float predictFlowRange() const;
	Vector2f predictFlow(const Vector3f &flow_gyro) const;

	// fuse optical flow line of sight rate measurements
	bool fuseOptFlow(VectorState &H, bool update_terrain);

#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// Return the magnetic declination in radians to be used by the alignment and fusion processing
	float getMagDeclination();
#endif // CONFIG_EKF2_MAGNETOMETER

	void clearInhibitedStateKalmanGains(VectorState &K) const;

	// limit the diagonal of the covariance matrix
	void constrainStateVariances();

	void constrainStateVar(const IdxDof &state, float min, float max);
	void constrainStateVarLimitRatio(const IdxDof &state, float min, float max, float max_ratio = 1.e6f);

	// generic function which will perform a fusion step given a kalman gain K
	// and a scalar innovation value
	void fuse(const VectorState &K, float innovation);

	// calculate the earth rotation vector from a given latitude
	Vector3f calcEarthRateNED(float lat_rad) const;

	// Control the filter fusion modes
	void controlFusionModes(const imuSample &imu_delayed);

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// control fusion of external vision observations
	void controlExternalVisionFusion(const imuSample &imu_sample);
	void updateEvAttitudeErrorFilter(extVisionSample &ev_sample, bool ev_reset);
	void controlEvHeightFusion(const imuSample &imu_sample, const extVisionSample &ev_sample,
				   const bool common_starting_conditions_passing, const bool ev_reset, const bool quality_sufficient,
				   estimator_aid_source1d_s &aid_src);
	void controlEvPosFusion(const imuSample &imu_sample, const extVisionSample &ev_sample,
				const bool common_starting_conditions_passing, const bool ev_reset, const bool quality_sufficient,
				estimator_aid_source2d_s &aid_src);
	void controlEvVelFusion(ExternalVisionVel &ev, const bool common_starting_conditions_passing, const bool ev_reset,
				const bool quality_sufficient, estimator_aid_source3d_s &aid_src);
	void controlEvYawFusion(const imuSample &imu_sample, const extVisionSample &ev_sample,
				const bool common_starting_conditions_passing, const bool ev_reset, const bool quality_sufficient,
				estimator_aid_source1d_s &aid_src);
	void fuseLocalFrameVelocity(estimator_aid_source3d_s &aid_src, const uint64_t &timestamp, const Vector3f &measurement,
				    const Vector3f &measurement_var, const float &innovation_gate);
	void fuseBodyFrameVelocity(estimator_aid_source3d_s &aid_src, const uint64_t &timestamp, const Vector3f &measurement,
				   const Vector3f &measurement_var, const float &innovation_gate);

	void startEvPosFusion(const Vector2f &measurement, const Vector2f &measurement_var, estimator_aid_source2d_s &aid_src);
	void updateEvPosFusion(const Vector2f &measurement, const Vector2f &measurement_var, bool quality_sufficient,
			       bool reset, estimator_aid_source2d_s &aid_src);

	void stopEvPosFusion();
	void stopEvHgtFusion();
	void stopEvVelFusion();
	void stopEvYawFusion();
	bool fuseEvVelocity(estimator_aid_source3d_s &aid_src, const extVisionSample &ev_sample);
	void fuseBodyVelocity(estimator_aid_source1d_s &aid_src, float &innov_var, VectorState &H)
	{
		VectorState Kfusion = P * H / innov_var;
		measurementUpdate(Kfusion, H, aid_src.observation_variance, aid_src.innovation);
		aid_src.fused = true;
	}
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_GNSS)
	// control fusion of GPS observations
	void controlGpsFusion(const imuSample &imu_delayed);
	void stopGpsFusion();
	void updateGnssVel(const imuSample &imu_sample, const gnssSample &gnss_sample, estimator_aid_source3d_s &aid_src);
	void updateGnssPos(const gnssSample &gnss_sample, estimator_aid_source2d_s &aid_src);
	void controlGnssYawEstimator(estimator_aid_source3d_s &aid_src_vel);
	bool tryYawEmergencyReset();
	void resetVelocityToGnss(estimator_aid_source3d_s &aid_src);
	void resetHorizontalPositionToGnss(estimator_aid_source2d_s &aid_src);
	bool shouldResetGpsFusion() const;

	/*
	 * Return true if the GPS solution quality is adequate.
	 * Checks are activated using the EKF2_GPS_CHECK bitmask parameter
	 * Checks are adjusted using the EKF2_REQ_* parameters
	*/
	bool runGnssChecks(const gnssSample &gps);

	void controlGnssHeightFusion(const gnssSample &gps_sample);
	void stopGpsHgtFusion();

	void resetGpsDriftCheckFilters();

# if defined(CONFIG_EKF2_GNSS_YAW)
	void controlGnssYawFusion(const gnssSample &gps_sample);
	void stopGnssYawFusion();

	// fuse the yaw angle obtained from a dual antenna GPS unit
	void fuseGnssYaw(float antenna_yaw_offset);

	// reset the quaternions states using the yaw angle obtained from a dual antenna GPS unit
	// return true if the reset was successful
	bool resetYawToGnss(float gnss_yaw, float gnss_yaw_offset);

	void updateGnssYaw(const gnssSample &gps_sample);

# endif // CONFIG_EKF2_GNSS_YAW

	// Declarations used to control use of the EKF-GSF yaw estimator
	bool isYawFailure() const;

	// Resets the main Nav EKf yaw to the estimator from the EKF-GSF yaw estimator
	// Returns true if the reset was successful
	bool resetYawToEKFGSF();

	// yaw estimator instance
	EKFGSF_yaw _yawEstimator{};

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// control fusion of magnetometer observations
	void controlMagFusion(const imuSample &imu_sample);

	bool checkHaglYawResetReq() const;

	void resetMagHeading(const Vector3f &mag);
	void resetMagStates(const Vector3f &mag, bool reset_heading = true);
	bool haglYawResetReq();

	void checkMagHeadingConsistency(const magSample &mag_sample);

	bool checkMagField(const Vector3f &mag);
	static bool isMeasuredMatchingExpected(float measured, float expected, float gate);

	void stopMagFusion();

	// calculate a synthetic value for the magnetometer Z component, given the 3D magnetomter
	// sensor measurement
	float calculate_synthetic_mag_z_measurement(const Vector3f &mag_meas, const Vector3f &mag_earth_predicted);

#endif // CONFIG_EKF2_MAGNETOMETER

	// control fusion of fake position observations to constrain drift
	void controlFakePosFusion();

	void controlFakeHgtFusion();
	void resetFakeHgtFusion();
	void resetHeightToLastKnown();
	void stopFakeHgtFusion();

	void controlZeroInnovationHeadingUpdate();

#if defined(CONFIG_EKF2_AUXVEL)
	// control fusion of auxiliary velocity observations
	void controlAuxVelFusion(const imuSample &imu_sample);
	void stopAuxVelFusion();
#endif // CONFIG_EKF2_AUXVEL

	void checkVerticalAccelerationHealth(const imuSample &imu_delayed);
	Likelihood estimateInertialNavFallingLikelihood() const;

	// control for combined height fusion mode (implemented for switching between baro and range height)
	void controlHeightFusion(const imuSample &imu_delayed);
	void checkHeightSensorRefFallback();

#if defined(CONFIG_EKF2_BAROMETER)
	void controlBaroHeightFusion(const imuSample &imu_sample);
	void stopBaroHgtFusion();

	void updateGroundEffect();

# if defined(CONFIG_EKF2_BARO_COMPENSATION)
	float compensateBaroForDynamicPressure(const imuSample &imu_sample, float baro_alt_uncompensated) const;
# endif // CONFIG_EKF2_BARO_COMPENSATION

#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	// gravity fusion: heuristically enable / disable gravity fusion
	void controlGravityFusion(const imuSample &imu_delayed);
#endif // CONFIG_EKF2_GRAVITY_FUSION

	void resetQuatCov(const float yaw_noise = NAN);
	void resetQuatCov(const Vector3f &rot_var_ned);

#if defined(CONFIG_EKF2_MAGNETOMETER)
	void resetMagEarthCov();
	void resetMagBiasCov();
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	void resetWindCov();
	void resetWindToZero();
#endif // CONFIG_EKF2_WIND

	void resetGyroBiasZCov();

	bool isTimedOut(uint64_t last_sensor_timestamp, uint64_t timeout_period) const
	{
		return (last_sensor_timestamp == 0) || (last_sensor_timestamp + timeout_period < _time_delayed_us);
	}

	bool isRecent(uint64_t sensor_timestamp, uint64_t acceptance_interval) const
	{
		return (sensor_timestamp != 0) && (sensor_timestamp + acceptance_interval > _time_delayed_us);
	}

	bool isNewestSampleRecent(uint64_t sensor_timestamp, uint64_t acceptance_interval) const
	{
		return (sensor_timestamp != 0) && (sensor_timestamp + acceptance_interval > _time_latest_us);
	}

	void resetFakePosFusion();
	bool runFakePosStateMachine(bool enable_condition_passing, bool status_flag, estimator_aid_source2d_s &aid_src);

	// reset the quaternion states and covariances to the new yaw value, preserving the roll and pitch
	// yaw : Euler yaw angle (rad)
	// yaw_variance : yaw error variance (rad^2)
	void resetQuatStateYaw(float yaw, float yaw_variance);

	HeightSensor _height_sensor_ref{HeightSensor::UNKNOWN};
	PositionSensor _position_sensor_ref{PositionSensor::GNSS};

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	HeightBiasEstimator _ev_hgt_b_est {HeightSensor::EV, _height_sensor_ref};
	PositionBiasEstimator _ev_pos_b_est{PositionSensor::EV, _position_sensor_ref};
	AlphaFilter<Quatf> _ev_q_error_filt{0.001f};
	bool _ev_q_error_initialized{false};
#endif // CONFIG_EKF2_EXTERNAL_VISION

	// state was reset to aid source, keep observation and update all other fields appropriately (zero innovation, etc)
	void resetAidSourceStatusZeroInnovation(estimator_aid_source1d_s &status) const
	{
		status.time_last_fuse = _time_delayed_us;

		status.innovation = 0.f;
		status.innovation_filtered = 0.f;
		status.innovation_variance = status.observation_variance;

		status.test_ratio = 0.f;
		status.test_ratio_filtered = 0.f;

		status.innovation_rejected = false;
		status.fused = true;
	}

	// helper used for populating and filtering estimator aid source struct for logging
	void updateAidSourceStatus(estimator_aid_source1d_s &status, const uint64_t &timestamp_sample,
				   const float &observation, const float &observation_variance,
				   const float &innovation, const float &innovation_variance,
				   float innovation_gate = 1.f) const;

	// state was reset to aid source, keep observation and update all other fields appropriately (zero innovation, etc)
	template <typename T>
	void resetAidSourceStatusZeroInnovation(T &status) const
	{
		status.time_last_fuse = _time_delayed_us;

		for (size_t i = 0; i < (sizeof(status.observation) / sizeof(status.observation[0])); i++) {
			status.innovation[i] = 0.f;
			status.innovation_filtered[i] = 0.f;
			status.innovation_variance[i] = status.observation_variance[i];

			status.test_ratio[i] = 0.f;
			status.test_ratio_filtered[i] = 0.f;
		}

		status.innovation_rejected = false;
		status.fused = true;
	}

	// helper used for populating and filtering estimator aid source struct for logging
	template <typename T, typename S, typename D>
	void updateAidSourceStatus(T &status, const uint64_t &timestamp_sample,
				   const D &observation, const S &observation_variance,
				   const S &innovation, const S &innovation_variance,
				   float innovation_gate = 1.f) const
	{
		bool innovation_rejected = false;

		const float dt_s = math::constrain((timestamp_sample - status.timestamp_sample) * 1e-6f, 0.001f, 1.f);

		static constexpr float tau = 0.5f;
		const float alpha = math::constrain(dt_s / (dt_s + tau), 0.f, 1.f);

		for (size_t i = 0; i < (sizeof(status.observation) / sizeof(status.observation[0])); i++) {

			const float test_ratio = sq(innovation(i)) / (sq(innovation_gate) * innovation_variance(i));

			if ((status.timestamp_sample > 0) && (timestamp_sample > status.timestamp_sample)) {

				// test_ratio_filtered
				if (PX4_ISFINITE(status.test_ratio_filtered[i])) {
					status.test_ratio_filtered[i] += alpha * (matrix::sign(innovation(i)) * test_ratio - status.test_ratio_filtered[i]);

				} else {
					// otherwise, init the filtered test ratio
					status.test_ratio_filtered[i] = test_ratio;
				}

				// innovation_filtered
				if (PX4_ISFINITE(status.innovation_filtered[i])) {
					status.innovation_filtered[i] += alpha * (innovation(i) - status.innovation_filtered[i]);

				} else {
					// otherwise, init the filtered innovation
					status.innovation_filtered[i] = innovation(i);
				}

				// limit extremes in filtered values
				static constexpr float kNormalizedInnovationLimit = 2.f;
				static constexpr float kTestRatioLimit = sq(kNormalizedInnovationLimit);

				if (test_ratio > kTestRatioLimit) {

					status.test_ratio_filtered[i] = math::constrain(status.test_ratio_filtered[i], -kTestRatioLimit, kTestRatioLimit);

					const float innov_limit = kNormalizedInnovationLimit * innovation_gate * sqrtf(innovation_variance(i));
					status.innovation_filtered[i] = math::constrain(status.innovation_filtered[i], -innov_limit, innov_limit);
				}

			} else {
				// invalid timestamp_sample, reset
				status.test_ratio_filtered[i] = test_ratio;
				status.innovation_filtered[i] = innovation(i);
			}

			status.test_ratio[i] = test_ratio;

			status.observation[i] = static_cast<double>(observation(i));
			status.observation_variance[i] = observation_variance(i);

			status.innovation[i] = innovation(i);
			status.innovation_variance[i] = innovation_variance(i);

			if ((test_ratio > 1.f)
			    || !PX4_ISFINITE(test_ratio)
			    || !PX4_ISFINITE(status.innovation[i])
			    || !PX4_ISFINITE(status.innovation_variance[i])
			   ) {
				innovation_rejected = true;
			}
		}

		status.timestamp_sample = timestamp_sample;

		// if any of the innovations are rejected, then the overall innovation is rejected
		status.innovation_rejected = innovation_rejected;

		// reset
		status.fused = false;
	}

	ZeroGyroUpdate _zero_gyro_update{};
	ZeroVelocityUpdate _zero_velocity_update{};

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION) && defined(MODULE_NAME)
	AuxGlobalPosition _aux_global_position {};
#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION
};

#endif // !EKF_EKF_H
