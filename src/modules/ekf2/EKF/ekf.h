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

#ifndef EKF_EKF_H
#define EKF_EKF_H

#include "estimator_interface.h"

#include "EKFGSF_yaw.h"
#include "bias_estimator.hpp"
#include "height_bias_estimator.hpp"
#include "position_bias_estimator.hpp"

#include <uORB/topics/estimator_aid_source1d.h>
#include <uORB/topics/estimator_aid_source2d.h>
#include <uORB/topics/estimator_aid_source3d.h>

enum class Likelihood { LOW, MEDIUM, HIGH };

class Ekf final : public EstimatorInterface
{
public:
	static constexpr uint8_t _k_num_states{24};		///< number of EKF states

	typedef matrix::Vector<float, _k_num_states> Vector24f;
	typedef matrix::SquareMatrix<float, _k_num_states> SquareMatrix24f;
	typedef matrix::SquareMatrix<float, 2> Matrix2f;
	template<int ... Idxs>

	using SparseVector24f = matrix::SparseVectorf<24, Idxs...>;

	Ekf()
	{
		reset();
	};

	virtual ~Ekf() = default;

	// initialise variables to sane values (also interface class)
	bool init(uint64_t timestamp) override;

	// should be called every time new data is pushed into the filter
	bool update();

	void getGpsVelPosInnov(float hvel[2], float &vvel, float hpos[2], float &vpos) const;
	void getGpsVelPosInnovVar(float hvel[2], float &vvel, float hpos[2], float &vpos) const;
	void getGpsVelPosInnovRatio(float &hvel, float &vvel, float &hpos, float &vpos) const;

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	void getEvVelPosInnov(float hvel[2], float &vvel, float hpos[2], float &vpos) const;
	void getEvVelPosInnovVar(float hvel[2], float &vvel, float hpos[2], float &vpos) const;
	void getEvVelPosInnovRatio(float &hvel, float &vvel, float &hpos, float &vpos) const;
#endif // CONFIG_EKF2_EXTERNAL_VISION

	void getBaroHgtInnov(float &baro_hgt_innov) const { baro_hgt_innov = _aid_src_baro_hgt.innovation; }
	void getBaroHgtInnovVar(float &baro_hgt_innov_var) const { baro_hgt_innov_var = _aid_src_baro_hgt.innovation_variance; }
	void getBaroHgtInnovRatio(float &baro_hgt_innov_ratio) const { baro_hgt_innov_ratio = _aid_src_baro_hgt.test_ratio; }

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// range height
	const BiasEstimator::status &getRngHgtBiasEstimatorStatus() const { return _rng_hgt_b_est.getStatus(); }
	const auto &aid_src_rng_hgt() const { return _aid_src_rng_hgt; }

	void getRngHgtInnov(float &rng_hgt_innov) const { rng_hgt_innov = _aid_src_rng_hgt.innovation; }
	void getRngHgtInnovVar(float &rng_hgt_innov_var) const { rng_hgt_innov_var = _aid_src_rng_hgt.innovation_variance; }
	void getRngHgtInnovRatio(float &rng_hgt_innov_ratio) const { rng_hgt_innov_ratio = _aid_src_rng_hgt.test_ratio; }

	void getHaglInnov(float &hagl_innov) const { hagl_innov = _hagl_innov; }
	void getHaglInnovVar(float &hagl_innov_var) const { hagl_innov_var = _hagl_innov_var; }
	void getHaglInnovRatio(float &hagl_innov_ratio) const { hagl_innov_ratio = _hagl_test_ratio; }

	void getHaglRateInnov(float &hagl_rate_innov) const { hagl_rate_innov = _rng_consistency_check.getInnov(); }
	void getHaglRateInnovVar(float &hagl_rate_innov_var) const { hagl_rate_innov_var = _rng_consistency_check.getInnovVar(); }
	void getHaglRateInnovRatio(float &hagl_rate_innov_ratio) const { hagl_rate_innov_ratio = _rng_consistency_check.getSignedTestRatioLpf(); }

	// terrain estimate
	bool isTerrainEstimateValid() const;

	uint8_t getTerrainEstimateSensorBitfield() const { return _hagl_sensor_status.value; }

	// get the estimated terrain vertical position relative to the NED origin
	float getTerrainVertPos() const { return _terrain_vpos; };

	// get the number of times the vertical terrain position has been reset
	uint8_t getTerrainVertPosResetCounter() const { return _terrain_vpos_reset_counter; };

	// get the terrain variance
	float get_terrain_var() const { return _terrain_var; }
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	const auto &aid_src_optical_flow() const { return _aid_src_optical_flow; }

	void getFlowInnov(float flow_innov[2]) const;
	void getFlowInnovVar(float flow_innov_var[2]) const;
	void getFlowInnovRatio(float &flow_innov_ratio) const { flow_innov_ratio = math::max(_aid_src_optical_flow.test_ratio[0], _aid_src_optical_flow.test_ratio[1]); }

	const Vector2f &getFlowVelBody() const { return _flow_vel_body; }
	const Vector2f &getFlowVelNE() const { return _flow_vel_ne; }

	const Vector2f &getFlowCompensated() const { return _flow_compensated_XY_rad; }
	const Vector2f &getFlowUncompensated() const { return _flow_sample_delayed.flow_xy_rad; }

	const Vector3f getFlowGyro() const { return _flow_sample_delayed.gyro_xyz * (1.f / _flow_sample_delayed.dt); }
	const Vector3f &getFlowGyroIntegral() const { return _flow_sample_delayed.gyro_xyz; }

	void getTerrainFlowInnov(float flow_innov[2]) const;
	void getTerrainFlowInnovVar(float flow_innov_var[2]) const;
	void getTerrainFlowInnovRatio(float &flow_innov_ratio) const { flow_innov_ratio = math::max(_aid_src_terrain_optical_flow.test_ratio[0], _aid_src_terrain_optical_flow.test_ratio[1]); }

	const auto &aid_src_terrain_optical_flow() const { return _aid_src_terrain_optical_flow; }
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_AUXVEL)
	void getAuxVelInnov(float aux_vel_innov[2]) const;
	void getAuxVelInnovVar(float aux_vel_innov[2]) const;
	void getAuxVelInnovRatio(float &aux_vel_innov_ratio) const { aux_vel_innov_ratio = math::max(_aid_src_aux_vel.test_ratio[0], _aid_src_aux_vel.test_ratio[1]); }
#endif // CONFIG_EKF2_AUXVEL

	void getHeadingInnov(float &heading_innov) const
	{
		if (_control_status.flags.mag_hdg) {
			heading_innov = _aid_src_mag_heading.innovation;
			return;
		}

		if (_control_status.flags.mag_3D) {
			heading_innov = Vector3f(_aid_src_mag.innovation).max();
			return;
		}

#if defined(CONFIG_EKF2_GNSS_YAW)
		if (_control_status.flags.gps_yaw) {
			heading_innov = _aid_src_gnss_yaw.innovation;
			return;
		}
#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
		if (_control_status.flags.ev_yaw) {
			heading_innov = _aid_src_ev_yaw.innovation;
			return;
		}
#endif // CONFIG_EKF2_EXTERNAL_VISION
	}

	void getHeadingInnovVar(float &heading_innov_var) const
	{
		if (_control_status.flags.mag_hdg) {
			heading_innov_var = _aid_src_mag_heading.innovation_variance;
			return;
		}

		if (_control_status.flags.mag_3D) {
			heading_innov_var = Vector3f(_aid_src_mag.innovation_variance).max();
			return;
		}

#if defined(CONFIG_EKF2_GNSS_YAW)
		if (_control_status.flags.gps_yaw) {
			heading_innov_var = _aid_src_gnss_yaw.innovation_variance;
			return;
		}
#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
		if (_control_status.flags.ev_yaw) {
			heading_innov_var = _aid_src_ev_yaw.innovation_variance;
			return;
		}
#endif // CONFIG_EKF2_EXTERNAL_VISION
	}

	void getHeadingInnovRatio(float &heading_innov_ratio) const
	{
		if (_control_status.flags.mag_hdg) {
			heading_innov_ratio = _aid_src_mag_heading.test_ratio;
			return;
		}

		if (_control_status.flags.mag_3D) {
			heading_innov_ratio = Vector3f(_aid_src_mag.test_ratio).max();
			return;
		}

#if defined(CONFIG_EKF2_GNSS_YAW)
		if (_control_status.flags.gps_yaw) {
			heading_innov_ratio = _aid_src_gnss_yaw.test_ratio;
			return;
		}
#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
		if (_control_status.flags.ev_yaw) {
			heading_innov_ratio = _aid_src_ev_yaw.test_ratio;
			return;
		}
#endif // CONFIG_EKF2_EXTERNAL_VISION
	}

	void getMagInnov(float mag_innov[3]) const { memcpy(mag_innov, _aid_src_mag.innovation, sizeof(_aid_src_mag.innovation)); }
	void getMagInnovVar(float mag_innov_var[3]) const { memcpy(mag_innov_var, _aid_src_mag.innovation_variance, sizeof(_aid_src_mag.innovation_variance)); }
	void getMagInnovRatio(float &mag_innov_ratio) const { mag_innov_ratio = Vector3f(_aid_src_mag.test_ratio).max(); }

#if defined(CONFIG_EKF2_DRAG_FUSION)
	void getDragInnov(float drag_innov[2]) const { _drag_innov.copyTo(drag_innov); }
	void getDragInnovVar(float drag_innov_var[2]) const { _drag_innov_var.copyTo(drag_innov_var); }
	void getDragInnovRatio(float drag_innov_ratio[2]) const { _drag_test_ratio.copyTo(drag_innov_ratio); }
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_AIRSPEED)
	void getAirspeedInnov(float &airspeed_innov) const { airspeed_innov = _aid_src_airspeed.innovation; }
	void getAirspeedInnovVar(float &airspeed_innov_var) const { airspeed_innov_var = _aid_src_airspeed.innovation_variance; }
	void getAirspeedInnovRatio(float &airspeed_innov_ratio) const { airspeed_innov_ratio = _aid_src_airspeed.test_ratio; }
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	void getBetaInnov(float &beta_innov) const { beta_innov = _aid_src_sideslip.innovation; }
	void getBetaInnovVar(float &beta_innov_var) const { beta_innov_var = _aid_src_sideslip.innovation_variance; }
	void getBetaInnovRatio(float &beta_innov_ratio) const { beta_innov_ratio = _aid_src_sideslip.test_ratio; }
#endif // CONFIG_EKF2_SIDESLIP

	void getGravityInnov(float grav_innov[3]) const { memcpy(grav_innov, _aid_src_gravity.innovation, sizeof(_aid_src_gravity.innovation)); }
	void getGravityInnovVar(float grav_innov_var[3]) const { memcpy(grav_innov_var, _aid_src_gravity.innovation_variance, sizeof(_aid_src_gravity.innovation_variance)); }
	void getGravityInnovRatio(float &grav_innov_ratio) const { grav_innov_ratio = Vector3f(_aid_src_gravity.test_ratio).max(); }

	// get the state vector at the delayed time horizon
	matrix::Vector<float, 24> getStateAtFusionHorizonAsVector() const;

	// get the wind velocity in m/s
	const Vector2f &getWindVelocity() const { return _state.wind_vel; };

	// get the wind velocity var
	Vector2f getWindVelocityVariance() const { return P.slice<2, 2>(22, 22).diag(); }

	// get the full covariance matrix
	const matrix::SquareMatrix<float, 24> &covariances() const { return P; }

	// get the diagonal elements of the covariance matrix
	matrix::Vector<float, 24> covariances_diagonal() const { return P.diag(); }

	// get the orientation (quaterion) covariances
	matrix::SquareMatrix<float, 4> orientation_covariances() const { return P.slice<4, 4>(0, 0); }

	// get the linear velocity covariances
	matrix::SquareMatrix<float, 3> velocity_covariances() const { return P.slice<3, 3>(4, 4); }

	// get the position covariances
	matrix::SquareMatrix<float, 3> position_covariances() const { return P.slice<3, 3>(7, 7); }

	// ask estimator for sensor data collection decision and do any preprocessing if required, returns true if not defined
	bool collect_gps(const gpsMessage &gps) override;

	// get the ekf WGS-84 origin position and height and the system time it was last set
	// return true if the origin is valid
	bool getEkfGlobalOrigin(uint64_t &origin_time, double &latitude, double &longitude, float &origin_alt) const;
	bool setEkfGlobalOrigin(const double latitude, const double longitude, const float altitude);

	float getEkfGlobalOriginAltitude() const { return PX4_ISFINITE(_gps_alt_ref) ? _gps_alt_ref : 0.f; }
	bool setEkfGlobalOriginAltitude(const float altitude);


	// get the 1-sigma horizontal and vertical position uncertainty of the ekf WGS-84 position
	void get_ekf_gpos_accuracy(float *ekf_eph, float *ekf_epv) const;

	// get the 1-sigma horizontal and vertical position uncertainty of the ekf local position
	void get_ekf_lpos_accuracy(float *ekf_eph, float *ekf_epv) const;

	// get the 1-sigma horizontal and vertical velocity uncertainty
	void get_ekf_vel_accuracy(float *ekf_evh, float *ekf_evv) const;

	// get the vehicle control limits required by the estimator to keep within sensor limitations
	void get_ekf_ctrl_limits(float *vxy_max, float *vz_max, float *hagl_min, float *hagl_max) const;

	// Reset all IMU bias states and covariances to initial alignment values.
	void resetImuBias();
	void resetGyroBias();
	void resetAccelBias();

	Vector3f getVelocityVariance() const { return P.slice<3, 3>(4, 4).diag(); };

	Vector3f getPositionVariance() const { return P.slice<3, 3>(7, 7).diag(); }

	// First argument returns GPS drift  metrics in the following array locations
	// 0 : Horizontal position drift rate (m/s)
	// 1 : Vertical position drift rate (m/s)
	// 2 : Filtered horizontal velocity (m/s)
	// Second argument returns true when IMU movement is blocking the drift calculation
	// Function returns true if the metrics have been updated and not returned previously by this function
	bool get_gps_drift_metrics(float drift[3], bool *blocked);

	// return true if the global position estimate is valid
	// return true if the origin is set we are not doing unconstrained free inertial navigation
	// and have not started using synthetic position observations to constrain drift
	bool global_position_is_valid() const
	{
		return (_NED_origin_initialised && local_position_is_valid());
	}

	// return true if the local position estimate is valid
	bool local_position_is_valid() const
	{
		return (!_horizontal_deadreckon_time_exceeded && !_control_status.flags.fake_pos);
	}

	bool isLocalVerticalPositionValid() const
	{
		return !_vertical_position_deadreckon_time_exceeded && !_control_status.flags.fake_hgt;
	}

	bool isLocalVerticalVelocityValid() const
	{
		return !_vertical_velocity_deadreckon_time_exceeded && !_control_status.flags.fake_hgt;
	}

	bool isYawFinalAlignComplete() const
	{
		const bool is_using_mag = (_control_status.flags.mag_3D || _control_status.flags.mag_hdg);
		const bool is_mag_alignment_in_flight_complete = is_using_mag
				&& _control_status.flags.mag_aligned_in_flight
				&& ((_time_delayed_us - _flt_mag_align_start_time) > (uint64_t)1e6);
		return _control_status.flags.yaw_align
		       && (is_mag_alignment_in_flight_complete || !is_using_mag);
	}

	// gyro bias (states 10, 11, 12)
	Vector3f getGyroBias() const { return _state.delta_ang_bias / _dt_ekf_avg; } // get the gyroscope bias in rad/s
	Vector3f getGyroBiasVariance() const { return Vector3f{P(10, 10), P(11, 11), P(12, 12)} / sq(_dt_ekf_avg); } // get the gyroscope bias variance in rad/s
	float getGyroBiasLimit() const { return _params.gyro_bias_lim; }

	// accel bias (states 13, 14, 15)
	Vector3f getAccelBias() const { return _state.delta_vel_bias / _dt_ekf_avg; } // get the accelerometer bias in m/s**2
	Vector3f getAccelBiasVariance() const { return Vector3f{P(13, 13), P(14, 14), P(15, 15)} / sq(_dt_ekf_avg); } // get the accelerometer bias variance in m/s**2
	float getAccelBiasLimit() const { return _params.acc_bias_lim; }

	// mag bias (states 19, 20, 21)
	const Vector3f &getMagBias() const { return _state.mag_B; }
	Vector3f getMagBiasVariance() const
	{
		if (_control_status.flags.mag_3D) {
			return Vector3f{P(19, 19), P(20, 20), P(21, 21)};
		}

		return _saved_mag_bf_variance;
	}
	float getMagBiasLimit() const { return 0.5f; } // 0.5 Gauss

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

	// get EKF innovation consistency check status information comprising of:
	// status - a bitmask integer containing the pass/fail status for each EKF measurement innovation consistency check
	// Innovation Test Ratios - these are the ratio of the innovation to the acceptance threshold.
	// A value > 1 indicates that the sensor measurement has exceeded the maximum acceptable level and has been rejected by the EKF
	// Where a measurement type is a vector quantity, eg magnetometer, GPS position, etc, the maximum value is returned.
	void get_innovation_test_status(uint16_t &status, float &mag, float &vel, float &pos, float &hgt, float &tas,
					float &hagl, float &beta) const;

	// return a bitmask integer that describes which state estimates can be used for flight control
	void get_ekf_soln_status(uint16_t *status) const;

	// rotate quaternion covariances into variances for an equivalent rotation vector
	Vector3f calcRotVecVariances() const;

	// set minimum continuous period without GPS fail required to mark a healthy GPS status
	void set_min_required_gps_health_time(uint32_t time_us) { _min_gps_health_time_us = time_us; }

	const gps_check_fail_status_u &gps_check_fail_status() const { return _gps_check_fail_status; }
	const decltype(gps_check_fail_status_u::flags) &gps_check_fail_status_flags() const { return _gps_check_fail_status.flags; }

	bool gps_checks_passed() const { return _gps_checks_passed; };

	// get solution data from the EKF-GSF emergency yaw estimator
	// returns false when data is not available
	bool getDataEKFGSF(float *yaw_composite, float *yaw_variance, float yaw[N_MODELS_EKFGSF],
			   float innov_VN[N_MODELS_EKFGSF], float innov_VE[N_MODELS_EKFGSF], float weight[N_MODELS_EKFGSF]);

	// Returns true if the output of the yaw emergency estimator can be used for a reset
	bool isYawEmergencyEstimateAvailable() const;

	uint8_t getHeightSensorRef() const { return _height_sensor_ref; }
	const BiasEstimator::status &getBaroBiasEstimatorStatus() const { return _baro_b_est.getStatus(); }
	const BiasEstimator::status &getGpsHgtBiasEstimatorStatus() const { return _gps_hgt_b_est.getStatus(); }

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	const BiasEstimator::status &getEvHgtBiasEstimatorStatus() const { return _ev_hgt_b_est.getStatus(); }

	const BiasEstimator::status &getEvPosBiasEstimatorStatus(int i) const { return _ev_pos_b_est.getStatus(i); }
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_AIRSPEED)
	const auto &aid_src_airspeed() const { return _aid_src_airspeed; }
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	const auto &aid_src_sideslip() const { return _aid_src_sideslip; }
#endif // CONFIG_EKF2_SIDESLIP

	const auto &aid_src_baro_hgt() const { return _aid_src_baro_hgt; }

	const auto &aid_src_fake_hgt() const { return _aid_src_fake_hgt; }
	const auto &aid_src_fake_pos() const { return _aid_src_fake_pos; }

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	const auto &aid_src_ev_hgt() const { return _aid_src_ev_hgt; }
	const auto &aid_src_ev_pos() const { return _aid_src_ev_pos; }
	const auto &aid_src_ev_vel() const { return _aid_src_ev_vel; }
	const auto &aid_src_ev_yaw() const { return _aid_src_ev_yaw; }
#endif // CONFIG_EKF2_EXTERNAL_VISION

	const auto &aid_src_gnss_hgt() const { return _aid_src_gnss_hgt; }
	const auto &aid_src_gnss_pos() const { return _aid_src_gnss_pos; }
	const auto &aid_src_gnss_vel() const { return _aid_src_gnss_vel; }

#if defined(CONFIG_EKF2_GNSS_YAW)
	const auto &aid_src_gnss_yaw() const { return _aid_src_gnss_yaw; }
#endif // CONFIG_EKF2_GNSS_YAW

	const auto &aid_src_mag_heading() const { return _aid_src_mag_heading; }
	const auto &aid_src_mag() const { return _aid_src_mag; }

	const auto &aid_src_gravity() const { return _aid_src_gravity; }

#if defined(CONFIG_EKF2_AUXVEL)
	const auto &aid_src_aux_vel() const { return _aid_src_aux_vel; }
#endif // CONFIG_EKF2_AUXVEL

private:

	// set the internal states and status to their default value
	void reset();

	bool initialiseTilt();

	// check if the EKF is dead reckoning horizontal velocity using inertial data only
	void updateDeadReckoningStatus();
	void updateHorizontalDeadReckoningstatus();
	void updateVerticalDeadReckoningStatus();

	struct StateResetCounts {
		uint8_t velNE{0};	///< number of horizontal position reset events (allow to wrap if count exceeds 255)
		uint8_t velD{0};	///< number of vertical velocity reset events (allow to wrap if count exceeds 255)
		uint8_t posNE{0};	///< number of horizontal position reset events (allow to wrap if count exceeds 255)
		uint8_t posD{0};	///< number of vertical position reset events (allow to wrap if count exceeds 255)
		uint8_t quat{0};	///< number of quaternion reset events (allow to wrap if count exceeds 255)
	};

	struct StateResets {
		Vector2f velNE_change;  ///< North East velocity change due to last reset (m)
		float velD_change;	///< Down velocity change due to last reset (m/sec)
		Vector2f posNE_change;	///< North, East position change due to last reset (m)
		float posD_change;	///< Down position change due to last reset (m)
		Quatf quat_change;	///< quaternion delta due to last reset - multiply pre-reset quaternion by this to get post-reset quaternion

		StateResetCounts reset_count{};
	};

	StateResets _state_reset_status{};	///< reset event monitoring structure containing velocity, position, height and yaw reset information
	StateResetCounts _state_reset_count_prev{};

	Vector3f _ang_rate_delayed_raw{};	///< uncorrected angular rate vector at fusion time horizon (rad/sec)

	stateSample _state{};		///< state struct of the ekf running at the delayed time horizon

	bool _filter_initialised{false};	///< true when the EKF sttes and covariances been initialised

	// booleans true when fresh sensor data is available at the fusion time horizon
	bool _gps_data_ready{false};	///< true when new GPS data has fallen behind the fusion time horizon and is available to be fused

	uint64_t _time_last_horizontal_aiding{0}; ///< amount of time we have been doing inertial only deadreckoning (uSec)
	uint64_t _time_last_v_pos_aiding{0};
	uint64_t _time_last_v_vel_aiding{0};

	uint64_t _time_last_hor_pos_fuse{0};	///< time the last fusion of horizontal position measurements was performed (uSec)
	uint64_t _time_last_hgt_fuse{0};	///< time the last fusion of vertical position measurements was performed (uSec)
	uint64_t _time_last_hor_vel_fuse{0};	///< time the last fusion of horizontal velocity measurements was performed (uSec)
	uint64_t _time_last_ver_vel_fuse{0};	///< time the last fusion of verticalvelocity measurements was performed (uSec)
	uint64_t _time_last_heading_fuse{0};
	uint64_t _time_last_zero_velocity_fuse{0}; ///< last time of zero velocity update (uSec)

	Vector3f _last_known_pos{};		///< last known local position vector (m)

	uint64_t _time_acc_bias_check{0};	///< last time the  accel bias check passed (uSec)

	Vector3f _earth_rate_NED{};	///< earth rotation vector (NED) in rad/s

	Dcmf _R_to_earth{};	///< transformation matrix from body frame to earth frame from last EKF prediction

	// used by magnetometer fusion mode selection
	Vector2f _accel_lpf_NE{};			///< Low pass filtered horizontal earth frame acceleration (m/sec**2)
	float _yaw_delta_ef{0.0f};		///< Recent change in yaw angle measured about the earth frame D axis (rad)
	float _yaw_rate_lpf_ef{0.0f};		///< Filtered angular rate about earth frame D axis (rad/sec)
	bool _mag_bias_observable{false};	///< true when there is enough rotation to make magnetometer bias errors observable
	bool _yaw_angle_observable{false};	///< true when there is enough horizontal acceleration to make yaw observable
	uint64_t _time_yaw_started{0};		///< last system time in usec that a yaw rotation manoeuvre was detected
	uint64_t _mag_use_not_inhibit_us{0};	///< last system time in usec before magnetometer use was inhibited
	float _last_static_yaw{NAN};		///< last yaw angle recorded when on ground motion checks were passing (rad)

	bool _mag_yaw_reset_req{false};		///< true when a reset of the yaw using the magnetometer data has been requested
	bool _mag_decl_cov_reset{false};	///< true after the fuseDeclination() function has been used to modify the earth field covariances after a magnetic field reset event.
	bool _synthetic_mag_z_active{false};	///< true if we are generating synthetic magnetometer Z measurements

	SquareMatrix24f P{};	///< state covariance matrix

	Vector3f _delta_angle_bias_var_accum{};	///< kahan summation algorithm accumulator for delta angle bias variance
	Vector3f _delta_vel_bias_var_accum{};   ///< kahan summation algorithm accumulator for delta velocity bias variance

#if defined(CONFIG_EKF2_DRAG_FUSION)
	Vector2f _drag_innov{};		///< multirotor drag measurement innovation (m/sec**2)
	Vector2f _drag_innov_var{};	///< multirotor drag measurement innovation variance ((m/sec**2)**2)
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_RANGE_FINDER)
	estimator_aid_source1d_s _aid_src_rng_hgt{};

	HeightBiasEstimator _rng_hgt_b_est{HeightSensor::RANGE, _height_sensor_ref};

	float _hagl_innov{0.0f};		///< innovation of the last height above terrain measurement (m)
	float _hagl_innov_var{0.0f};		///< innovation variance for the last height above terrain measurement (m**2)
	float _hagl_test_ratio{}; // height above terrain measurement innovation consistency check ratio

	uint64_t _time_last_healthy_rng_data{0};

	// Terrain height state estimation
	float _terrain_vpos{0.0f};		///< estimated vertical position of the terrain underneath the vehicle in local NED frame (m)
	float _terrain_var{1e4f};		///< variance of terrain position estimate (m**2)
	uint8_t _terrain_vpos_reset_counter{0};	///< number of times _terrain_vpos has been reset
	uint64_t _time_last_hagl_fuse{0};		///< last system time that a range sample was fused by the terrain estimator
	terrain_fusion_status_u _hagl_sensor_status{}; ///< Struct indicating type of sensor used to estimate height above ground

	float _last_on_ground_posD{0.0f};	///< last vertical position when the in_air status was false (m)
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	estimator_aid_source2d_s _aid_src_optical_flow{};
	estimator_aid_source2d_s _aid_src_terrain_optical_flow{};

	// optical flow processing
	Vector3f _flow_gyro_bias{};	///< bias errors in optical flow sensor rate gyro outputs (rad/sec)
	Vector2f _flow_vel_body{};	///< velocity from corrected flow measurement (body frame)(m/s)
	Vector2f _flow_vel_ne{};		///< velocity from corrected flow measurement (local frame) (m/s)
	Vector3f _imu_del_ang_of{};	///< bias corrected delta angle measurements accumulated across the same time frame as the optical flow rates (rad)

	float _delta_time_of{0.0f};	///< time in sec that _imu_del_ang_of was accumulated over (sec)
	uint64_t _time_bad_motion_us{0};	///< last system time that on-ground motion exceeded limits (uSec)
	uint64_t _time_good_motion_us{0};	///< last system time that on-ground motion was within limits (uSec)
	Vector2f _flow_compensated_XY_rad{};	///< measured delta angle of the image about the X and Y body axes after removal of body rotation (rad), RH rotation is positive

	bool _flow_data_ready{false};	///< true when the leading edge of the optical flow integration period has fallen behind the fusion time horizon
	uint64_t _time_last_flow_terrain_fuse{0}; ///< time the last fusion of optical flow measurements for terrain estimation were performed (uSec)
#endif // CONFIG_EKF2_OPTICAL_FLOW

	estimator_aid_source1d_s _aid_src_baro_hgt{};
#if defined(CONFIG_EKF2_AIRSPEED)
	estimator_aid_source1d_s _aid_src_airspeed{};
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_SIDESLIP)
	estimator_aid_source1d_s _aid_src_sideslip{};
#endif // CONFIG_EKF2_SIDESLIP

	estimator_aid_source2d_s _aid_src_fake_pos{};
	estimator_aid_source1d_s _aid_src_fake_hgt{};

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	estimator_aid_source1d_s _aid_src_ev_hgt{};
	estimator_aid_source2d_s _aid_src_ev_pos{};
	estimator_aid_source3d_s _aid_src_ev_vel{};
	estimator_aid_source1d_s _aid_src_ev_yaw{};

	float _ev_yaw_pred_prev{}; ///< previous value of yaw state used by odometry fusion (m)

	uint8_t _nb_ev_pos_reset_available{0};
	uint8_t _nb_ev_vel_reset_available{0};
	uint8_t _nb_ev_yaw_reset_available{0};
#endif // CONFIG_EKF2_EXTERNAL_VISION
	bool _inhibit_ev_yaw_use{false};	///< true when the vision yaw data should not be used (e.g.: NE fusion requires true North)

	estimator_aid_source1d_s _aid_src_gnss_hgt{};
	estimator_aid_source2d_s _aid_src_gnss_pos{};
	estimator_aid_source3d_s _aid_src_gnss_vel{};

#if defined(CONFIG_EKF2_GNSS_YAW)
	estimator_aid_source1d_s _aid_src_gnss_yaw{};
	uint8_t _nb_gps_yaw_reset_available{0}; ///< remaining number of resets allowed before switching to another aiding source
#endif // CONFIG_EKF2_GNSS_YAW

	estimator_aid_source1d_s _aid_src_mag_heading{};
	estimator_aid_source3d_s _aid_src_mag{};

	estimator_aid_source3d_s _aid_src_gravity{};

#if defined(CONFIG_EKF2_AUXVEL)
	estimator_aid_source2d_s _aid_src_aux_vel{};
#endif // CONFIG_EKF2_AUXVEL

	// variables used for the GPS quality checks
	Vector3f _gps_pos_deriv_filt{};	///< GPS NED position derivative (m/sec)
	Vector2f _gps_velNE_filt{};	///< filtered GPS North and East velocity (m/sec)

	float _gps_velD_diff_filt{0.0f};	///< GPS filtered Down velocity (m/sec)
	uint64_t _last_gps_fail_us{0};		///< last system time in usec that the GPS failed it's checks
	uint64_t _last_gps_pass_us{0};		///< last system time in usec that the GPS passed it's checks
	float _gps_error_norm{1.0f};		///< normalised gps error
	uint32_t _min_gps_health_time_us{10000000}; ///< GPS is marked as healthy only after this amount of time
	bool _gps_checks_passed{false};		///> true when all active GPS checks have passed

	// Variables used to publish the WGS-84 location of the EKF local NED origin
	float _gps_alt_ref{NAN};		///< WGS-84 height (m)

	// Variables used by the initial filter alignment
	bool _is_first_imu_sample{true};
	uint32_t _baro_counter{0};		///< number of baro samples read during initialisation
	uint32_t _mag_counter{0};		///< number of magnetometer samples read during initialisation
	AlphaFilter<Vector3f> _accel_lpf{0.1f};	///< filtered accelerometer measurement used to align tilt (m/s/s)
	AlphaFilter<Vector3f> _gyro_lpf{0.1f};	///< filtered gyro measurement used for alignment excessive movement check (rad/sec)

	// Variables used to perform in flight resets and switch between height sources
	AlphaFilter<Vector3f> _mag_lpf{0.1f};	///< filtered magnetometer measurement for instant reset (Gauss)
	AlphaFilter<float> _baro_lpf{0.1f};	///< filtered barometric height measurement (m)

	// Variables used to control activation of post takeoff functionality
	uint64_t _flt_mag_align_start_time{0};	///< time that inflight magnetic field alignment started (uSec)
	uint64_t _time_last_mov_3d_mag_suitable{0};	///< last system time that sufficient movement to use 3-axis magnetometer fusion was detected (uSec)
	Vector3f _saved_mag_bf_variance {}; ///< magnetic field state variances that have been saved for use at the next initialisation (Gauss**2)
	Matrix2f _saved_mag_ef_ne_covmat{}; ///< NE magnetic field state covariance sub-matrix saved for use at the next initialisation (Gauss**2)
	float _saved_mag_ef_d_variance{};   ///< D magnetic field state variance saved for use at the next initialisation (Gauss**2)

	gps_check_fail_status_u _gps_check_fail_status{};

	// variables used to inhibit accel bias learning
	bool _accel_bias_inhibit[3] {};		///< true when the accel bias learning is being inhibited for the specified axis
	bool _gyro_bias_inhibit[3] {};		///< true when the gyro bias learning is being inhibited for the specified axis
	Vector3f _accel_vec_filt{};		///< acceleration vector after application of a low pass filter (m/sec**2)
	float _accel_magnitude_filt{0.0f};	///< acceleration magnitude after application of a decaying envelope filter (rad/sec)
	float _ang_rate_magnitude_filt{0.0f};		///< angular rate magnitude after application of a decaying envelope filter (rad/sec)
	Vector3f _prev_delta_ang_bias_var{};	///< saved delta angle XYZ bias variances (rad/sec)
	Vector3f _prev_dvel_bias_var{};		///< saved delta velocity XYZ bias variances (m/sec)**2

	// height sensor status
	bool _baro_hgt_faulty{false};		///< true if baro data have been declared faulty TODO: move to fault flags
	bool _gps_intermittent{true};           ///< true if data into the buffer is intermittent

	// imu fault status
	uint64_t _time_bad_vert_accel{0};	///< last time a bad vertical accel was detected (uSec)
	uint64_t _time_good_vert_accel{0};	///< last time a good vertical accel was detected (uSec)
	uint16_t _clip_counter{0};		///< counter that increments when clipping ad decrements when not

	float _height_rate_lpf{0.0f};

	// initialise filter states of both the delayed ekf and the real time complementary filter
	bool initialiseFilter(void);

	// initialise ekf covariance matrix
	void initialiseCovariance();

	// predict ekf state
	void predictState(const imuSample &imu_delayed);

	// predict ekf covariance
	void predictCovariance(const imuSample &imu_delayed);

	// ekf sequential fusion of magnetometer measurements
	bool fuseMag(const Vector3f &mag, estimator_aid_source3d_s &aid_src_mag, bool update_all_states = true);

	// update quaternion states and covariances using an innovation, observation variance and Jacobian vector
	bool fuseYaw(float innovation, float variance, estimator_aid_source1d_s &aid_src_status);
	bool fuseYaw(float innovation, float variance, estimator_aid_source1d_s &aid_src_status, const Vector24f &H_YAW);
	void computeYawInnovVarAndH(float variance, float &innovation_variance, Vector24f &H_YAW) const;

#if defined(CONFIG_EKF2_GNSS_YAW)
	void controlGpsYawFusion(const gpsSample &gps_sample, bool gps_checks_passing, bool gps_checks_failing);

	// fuse the yaw angle obtained from a dual antenna GPS unit
	void fuseGpsYaw();

	// reset the quaternions states using the yaw angle obtained from a dual antenna GPS unit
	// return true if the reset was successful
	bool resetYawToGps(const float gnss_yaw);

	void updateGpsYaw(const gpsSample &gps_sample);

	void startGpsYawFusion(const gpsSample &gps_sample);

#endif // CONFIG_EKF2_GNSS_YAW
	void stopGpsYawFusion();

	// fuse magnetometer declination measurement
	// argument passed in is the declination uncertainty in radians
	bool fuseDeclination(float decl_sigma);

	// apply sensible limits to the declination and length of the NE mag field states estimates
	void limitDeclination();

#if defined(CONFIG_EKF2_AIRSPEED)
	// control fusion of air data observations
	void controlAirDataFusion(const imuSample &imu_delayed);

	void updateAirspeed(const airspeedSample &airspeed_sample, estimator_aid_source1d_s &aid_src) const;
	void fuseAirspeed(const airspeedSample &airspeed_sample, estimator_aid_source1d_s &aid_src);

	void stopAirspeedFusion();

	// Reset the wind states using the current airspeed measurement, ground relative nav velocity, yaw angle and assumption of zero sideslip
	void resetWindUsingAirspeed(const airspeedSample &airspeed_sample);

	// perform a limited reset of the wind state covariances
	void resetWindCovarianceUsingAirspeed(const airspeedSample &airspeed_sample);
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	// control fusion of synthetic sideslip observations
	void controlBetaFusion(const imuSample &imu_delayed);

	// fuse synthetic zero sideslip measurement
	void updateSideslip(estimator_aid_source1d_s &_aid_src_sideslip) const;
	void fuseSideslip(estimator_aid_source1d_s &_aid_src_sideslip);
#endif // CONFIG_EKF2_SIDESLIP

#if defined(CONFIG_EKF2_DRAG_FUSION)
	// control fusion of multi-rotor drag specific force observations
	void controlDragFusion();

	// fuse body frame drag specific forces for multi-rotor wind estimation
	void fuseDrag(const dragSample &drag_sample);
#endif // CONFIG_EKF2_DRAG_FUSION

	// fuse single velocity and position measurement
	bool fuseVelPosHeight(const float innov, const float innov_var, const int obs_index);

	void resetVelocityTo(const Vector3f &vel, const Vector3f &new_vel_var);

	void resetHorizontalVelocityTo(const Vector2f &new_horz_vel, const Vector2f &new_horz_vel_var);
	void resetHorizontalVelocityTo(const Vector2f &new_horz_vel, float vel_var) { resetHorizontalVelocityTo(new_horz_vel, Vector2f(vel_var, vel_var)); }

	void resetHorizontalVelocityToZero();

	void resetVerticalVelocityTo(float new_vert_vel, float new_vert_vel_var);
	void resetHorizontalPositionToLastKnown();

	void resetHorizontalPositionTo(const Vector2f &new_horz_pos, const Vector2f &new_horz_pos_var);
	void resetHorizontalPositionTo(const Vector2f &new_horz_pos, const float pos_var = NAN) { resetHorizontalPositionTo(new_horz_pos, Vector2f(pos_var, pos_var)); }

	bool isHeightResetRequired() const;

	void resetVerticalPositionTo(float new_vert_pos, float new_vert_pos_var = NAN);

	void resetVerticalVelocityToZero();

	// horizontal and vertical position aid source
	void updateHorizontalPositionAidSrcStatus(const uint64_t &time_us, const Vector2f &obs, const Vector2f &obs_var, const float innov_gate, estimator_aid_source2d_s &aid_src) const;
	void updateVerticalPositionAidSrcStatus(const uint64_t &time_us, const float obs, const float obs_var, const float innov_gate, estimator_aid_source1d_s &aid_src) const;

	// 2d & 3d velocity aid source
	void updateVelocityAidSrcStatus(const uint64_t &time_us, const Vector2f &obs, const Vector2f &obs_var, const float innov_gate, estimator_aid_source2d_s &aid_src) const;
	void updateVelocityAidSrcStatus(const uint64_t &time_us, const Vector3f &obs, const Vector3f &obs_var, const float innov_gate, estimator_aid_source3d_s &aid_src) const;

	// horizontal and vertical position fusion
	void fuseHorizontalPosition(estimator_aid_source2d_s &pos_aid_src);
	void fuseVerticalPosition(estimator_aid_source1d_s &hgt_aid_src);

	// 2d & 3d velocity fusion
	void fuseVelocity(estimator_aid_source2d_s &vel_aid_src);
	void fuseVelocity(estimator_aid_source3d_s &vel_aid_src);

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// range height
	void controlRangeHeightFusion();
	bool isConditionalRangeAidSuitable();
	void stopRngHgtFusion();

	// terrain vertical position estimator
	void initHagl();
	void runTerrainEstimator(const imuSample &imu_delayed);
	void predictHagl(const imuSample &imu_delayed);

	float getTerrainVPos() const { return isTerrainEstimateValid() ? _terrain_vpos : _last_on_ground_posD; }

	// update the terrain vertical position estimate using a height above ground measurement from the range finder
	void controlHaglRngFusion();
	void fuseHaglRng();
	void startHaglRngFusion();
	void resetHaglRngIfNeeded();
	void resetHaglRng();
	void stopHaglRngFusion();
	float getRngVar();

	void controlHaglFakeFusion();
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// control fusion of optical flow observations
	void controlOpticalFlowFusion(const imuSample &imu_delayed);
	void stopFlowFusion();

	void updateOnGroundMotionForOpticalFlowChecks();
	void resetOnGroundMotionForOpticalFlowChecks();

	// calculate the measurement variance for the optical flow sensor
	float calcOptFlowMeasVar(const flowSample &flow_sample);

	// calculate optical flow body angular rate compensation
	// returns false if bias corrected body rate data is unavailable
	bool calcOptFlowBodyRateComp();

	// fuse optical flow line of sight rate measurements
	void updateOptFlow(estimator_aid_source2d_s &aid_src);
	void fuseOptFlow();
	float predictFlowRange();
	Vector2f predictFlowVelBody();

	// update the terrain vertical position estimate using an optical flow measurement
	void controlHaglFlowFusion();
	void startHaglFlowFusion();
	void resetHaglFlow();
	void stopHaglFlowFusion();
	void fuseFlowForTerrain(estimator_aid_source2d_s &flow);
#endif // CONFIG_EKF2_OPTICAL_FLOW

	// reset the heading and magnetic field states using the declination and magnetometer measurements
	// return true if successful
	bool resetMagHeading();

	// Return the magnetic declination in radians to be used by the alignment and fusion processing
	float getMagDeclination();

	void clearInhibitedStateKalmanGains(Vector24f &K) const
	{
		// gyro bias: states 10, 11, 12
		for (unsigned i = 0; i < 3; i++) {
			if (_gyro_bias_inhibit[i]) {
				K(10 + i) = 0.f;
			}
		}

		// accel bias: states 13, 14, 15
		for (unsigned i = 0; i < 3; i++) {
			if (_accel_bias_inhibit[i]) {
				K(13 + i) = 0.f;
			}
		}

		// mag I: states 16, 17, 18
		if (!_control_status.flags.mag_3D) {
			K(16) = 0.f;
			K(17) = 0.f;
			K(18) = 0.f;
		}

		// mag B: states 19, 20, 21
		if (!_control_status.flags.mag_3D) {
			K(19) = 0.f;
			K(20) = 0.f;
			K(21) = 0.f;
		}

		// wind: states 22, 23
		if (!_control_status.flags.wind) {
			K(22) = 0.f;
			K(23) = 0.f;
		}
	}

	bool measurementUpdate(Vector24f &K, float innovation_variance, float innovation)
	{
		clearInhibitedStateKalmanGains(K);

		const Vector24f KS = K * innovation_variance;
		SquareMatrix24f KHP;

		for (unsigned row = 0; row < _k_num_states; row++) {
			for (unsigned col = 0; col < _k_num_states; col++) {
				// Instad of literally computing KHP, use an equvalent
				// equation involving less mathematical operations
				KHP(row, col) = KS(row) * K(col);
			}
		}

		const bool is_healthy = checkAndFixCovarianceUpdate(KHP);

		if (is_healthy) {
			// apply the covariance corrections
			P -= KHP;

			fixCovarianceErrors(true);

			// apply the state corrections
			fuse(K, innovation);
		}

		return is_healthy;
	}

	// if the covariance correction will result in a negative variance, then
	// the covariance matrix is unhealthy and must be corrected
	bool checkAndFixCovarianceUpdate(const SquareMatrix24f &KHP);

	// limit the diagonal of the covariance matrix
	// force symmetry when the argument is true
	void fixCovarianceErrors(bool force_symmetry);

	// constrain the ekf states
	void constrainStates();

	// generic function which will perform a fusion step given a kalman gain K
	// and a scalar innovation value
	void fuse(const Vector24f &K, float innovation);

	float compensateBaroForDynamicPressure(float baro_alt_uncompensated) const;

	// calculate the earth rotation vector from a given latitude
	Vector3f calcEarthRateNED(float lat_rad) const;

	// return true id the GPS quality is good enough to set an origin and start aiding
	bool gps_is_good(const gpsMessage &gps);

	// Control the filter fusion modes
	void controlFusionModes(const imuSample &imu_delayed);

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// control fusion of external vision observations
	void controlExternalVisionFusion();
	void controlEvHeightFusion(const extVisionSample &ev_sample, const bool common_starting_conditions_passing, const bool ev_reset, const bool quality_sufficient, estimator_aid_source1d_s &aid_src);
	void controlEvPosFusion(const extVisionSample &ev_sample, const bool common_starting_conditions_passing, const bool ev_reset, const bool quality_sufficient, estimator_aid_source2d_s &aid_src);
	void controlEvVelFusion(const extVisionSample &ev_sample, const bool common_starting_conditions_passing, const bool ev_reset, const bool quality_sufficient, estimator_aid_source3d_s &aid_src);
	void controlEvYawFusion(const extVisionSample &ev_sample, const bool common_starting_conditions_passing, const bool ev_reset, const bool quality_sufficient, estimator_aid_source1d_s &aid_src);

	void startEvPosFusion(const Vector2f &measurement, const Vector2f &measurement_var, estimator_aid_source2d_s &aid_src);
	void updateEvPosFusion(const Vector2f &measurement, const Vector2f &measurement_var, bool quality_sufficient, bool reset, estimator_aid_source2d_s &aid_src);
	void stopEvPosFusion();
	void stopEvHgtFusion();
	void stopEvVelFusion();
	void stopEvYawFusion();
#endif // CONFIG_EKF2_EXTERNAL_VISION

	// control fusion of GPS observations
	void controlGpsFusion(const imuSample &imu_delayed);
	bool shouldResetGpsFusion() const;
	bool isYawFailure() const;

	// control fusion of magnetometer observations
	void controlMagFusion();

	bool magReset();
	bool haglYawResetReq();

	void selectMagAuto();
	void check3DMagFusionSuitability();
	void checkYawAngleObservability();
	void checkMagBiasObservability();
	bool canUse3DMagFusion() const;

	void checkMagDeclRequired();
	bool shouldInhibitMag() const;
	bool magFieldStrengthDisturbed(const Vector3f &mag) const;
	static bool isMeasuredMatchingExpected(float measured, float expected, float gate);
	void runMagAndMagDeclFusions(const Vector3f &mag);
	void run3DMagAndDeclFusions(const Vector3f &mag);

	// control fusion of fake position observations to constrain drift
	void controlFakePosFusion();

	void controlFakeHgtFusion();
	void resetFakeHgtFusion();
	void resetHeightToLastKnown();
	void stopFakeHgtFusion();

	void controlZeroVelocityUpdate();

	void controlZeroInnovationHeadingUpdate();

#if defined(CONFIG_EKF2_AUXVEL)
	// control fusion of auxiliary velocity observations
	void controlAuxVelFusion();
	void stopAuxVelFusion();
#endif // CONFIG_EKF2_AUXVEL

	void checkVerticalAccelerationHealth(const imuSample &imu_delayed);
	Likelihood estimateInertialNavFallingLikelihood() const;

	// control for combined height fusion mode (implemented for switching between baro and range height)
	void controlHeightFusion(const imuSample &imu_delayed);
	void checkHeightSensorRefFallback();
	void controlBaroHeightFusion();
	void controlGnssHeightFusion(const gpsSample &gps_sample);

	void stopMagFusion();
	void stopMag3DFusion();
	void stopMagHdgFusion();
	void startMagHdgFusion();
	void startMag3DFusion();

	void stopBaroHgtFusion();
	void stopGpsHgtFusion();

	void updateGroundEffect();

	// gravity fusion: heuristically enable / disable gravity fusion
	void controlGravityFusion(const imuSample &imu_delayed);

	// initialise the quaternion covariances using rotation vector variances
	// do not call before quaternion states are initialised
	void initialiseQuatCovariances(Vector3f &rot_vec_var);

	// perform a limited reset of the magnetic field related state covariances
	void resetMagRelatedCovariances();

	void resetQuatCov();
	void zeroQuatCov();
	void resetMagCov();

	// perform a reset of the wind states and related covariances
	void resetWind();
	void resetWindToZero();

	// Increase the yaw error variance of the quaternions
	// Argument is additional yaw variance in rad**2
	void increaseQuatYawErrVariance(float yaw_variance);

	// load and save mag field state covariance data for re-use
	void loadMagCovData();
	void saveMagCovData();
	void clearMagCov();
	void zeroMagCov();

	void resetZDeltaAngBiasCov();

	// uncorrelate quaternion states from other states
	void uncorrelateQuatFromOtherStates();

	// calculate a synthetic value for the magnetometer Z component, given the 3D magnetomter
	// sensor measurement
	float calculate_synthetic_mag_z_measurement(const Vector3f &mag_meas, const Vector3f &mag_earth_predicted);

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

	void stopGpsFusion();
	void stopGpsPosFusion();
	void stopGpsVelFusion();

	void resetFakePosFusion();
	void stopFakePosFusion();

	void setVelPosStatus(const int index, const bool healthy);

	// reset the quaternion states and covariances to the new yaw value, preserving the roll and pitch
	// yaw : Euler yaw angle (rad)
	// yaw_variance : yaw error variance (rad^2)
	void resetQuatStateYaw(float yaw, float yaw_variance);

	// Declarations used to control use of the EKF-GSF yaw estimator

	// yaw estimator instance
	EKFGSF_yaw _yawEstimator{};

	uint8_t _height_sensor_ref{HeightSensor::UNKNOWN};
	uint8_t _position_sensor_ref{static_cast<uint8_t>(PositionSensor::GNSS)};

	HeightBiasEstimator _baro_b_est{HeightSensor::BARO, _height_sensor_ref};
	HeightBiasEstimator _gps_hgt_b_est{HeightSensor::GNSS, _height_sensor_ref};

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	HeightBiasEstimator _ev_hgt_b_est{HeightSensor::EV, _height_sensor_ref};
	PositionBiasEstimator _ev_pos_b_est{static_cast<uint8_t>(PositionSensor::EV), _position_sensor_ref};
#endif // CONFIG_EKF2_EXTERNAL_VISION

	// Resets the main Nav EKf yaw to the estimator from the EKF-GSF yaw estimator
	// Resets the horizontal velocity and position to the default navigation sensor
	// Returns true if the reset was successful
	bool resetYawToEKFGSF();

	void resetGpsDriftCheckFilters();

	void resetEstimatorAidStatus(estimator_aid_source1d_s &status) const
	{
		// only bother resetting if timestamp_sample is set
		if (status.timestamp_sample != 0) {
			status.timestamp_sample = 0;

			// preserve status.time_last_fuse

			status.observation = 0;
			status.observation_variance = 0;

			status.innovation = 0;
			status.innovation_variance = 0;
			status.test_ratio = INFINITY;

			status.fusion_enabled = false;
			status.innovation_rejected = true;
			status.fused = false;
		}
	}

	template <typename T>
	void resetEstimatorAidStatus(T &status) const
	{
		// only bother resetting if timestamp_sample is set
		if (status.timestamp_sample != 0) {
			status.timestamp_sample = 0;

			// preserve status.time_last_fuse

			for (size_t i = 0; i < (sizeof(status.observation) / sizeof(status.observation[0])); i++) {
				status.observation[i] = 0;
				status.observation_variance[i] = 0;

				status.innovation[i] = 0;
				status.innovation_variance[i] = 0;
				status.test_ratio[i] = INFINITY;
			}

			status.fusion_enabled = false;
			status.innovation_rejected = true;
			status.fused = false;
		}
	}

	void setEstimatorAidStatusTestRatio(estimator_aid_source1d_s &status, float innovation_gate) const
	{
		if (PX4_ISFINITE(status.innovation)
		    && PX4_ISFINITE(status.innovation_variance)
		    && (status.innovation_variance > 0.f)
		   ) {
			status.test_ratio = sq(status.innovation) / (sq(innovation_gate) * status.innovation_variance);
			status.innovation_rejected = (status.test_ratio > 1.f);

		} else {
			status.test_ratio = INFINITY;
			status.innovation_rejected = true;
		}
	}

	template <typename T>
	void setEstimatorAidStatusTestRatio(T &status, float innovation_gate) const
	{
		bool innovation_rejected = false;

		for (size_t i = 0; i < (sizeof(status.test_ratio) / sizeof(status.test_ratio[0])); i++) {
			if (PX4_ISFINITE(status.innovation[i])
			    && PX4_ISFINITE(status.innovation_variance[i])
			    && (status.innovation_variance[i] > 0.f)
			   ) {
				status.test_ratio[i] = sq(status.innovation[i]) / (sq(innovation_gate) * status.innovation_variance[i]);

				if (status.test_ratio[i] > 1.f) {
					innovation_rejected = true;
				}

			} else {
				status.test_ratio[i] = INFINITY;
				innovation_rejected = true;
			}
		}

		// if any of the innovations are rejected, then the overall innovation is rejected
		status.innovation_rejected = innovation_rejected;
	}
};

#endif // !EKF_EKF_H
