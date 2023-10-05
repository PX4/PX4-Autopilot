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

#if defined(MODULE_NAME)
#include <px4_platform_common/log.h>
# define ECL_INFO PX4_DEBUG
# define ECL_WARN PX4_DEBUG
# define ECL_ERR  PX4_DEBUG
# define ECL_DEBUG PX4_DEBUG
#else
# define ECL_INFO(X, ...) printf(X "\n", ##__VA_ARGS__)
# define ECL_WARN(X, ...) fprintf(stderr, X "\n", ##__VA_ARGS__)
# define ECL_ERR(X, ...) fprintf(stderr, X "\n", ##__VA_ARGS__)

# if defined(DEBUG_BUILD)
#  define ECL_DEBUG(X, ...) fprintf(stderr, X "\n", ##__VA_ARGS__)
# else
#  define ECL_DEBUG(X, ...)
#endif

#endif

#include "common.h"
#include "RingBuffer.h"
#include "imu_down_sampler.hpp"
#include "output_predictor.h"

#if defined(CONFIG_EKF2_RANGE_FINDER)
# include "range_finder_consistency_check.hpp"
# include "sensor_range_finder.hpp"
#endif // CONFIG_EKF2_RANGE_FINDER

#include <lib/geo/geo.h>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <mathlib/math/filter/AlphaFilter.hpp>

#include "EKFGSF_yaw.h"
#include "bias_estimator.hpp"
#include "height_bias_estimator.hpp"
#include "position_bias_estimator.hpp"
#include "python/ekf_derivation/generated/state.h"

#include <uORB/topics/estimator_aid_source1d.h>
#include <uORB/topics/estimator_aid_source2d.h>
#include <uORB/topics/estimator_aid_source3d.h>

using namespace estimator;

enum class Likelihood { LOW, MEDIUM, HIGH };

class Ekf
{
public:
	typedef matrix::Vector<float, State::size> VectorState;
	typedef matrix::SquareMatrix<float, State::size> SquareMatrixState;
	typedef matrix::SquareMatrix<float, 2> Matrix2f;

	Ekf()
	{
		reset();
	};

	~Ekf();

	// return a address to the parameters struct
	// in order to give access to the application
	parameters *getParamHandle() { return &_params; }

	// initialise variables to sane values (also interface class)
	bool init(uint64_t timestamp);

	void print_status();

	// should be called every time new data is pushed into the filter
	bool update();

	static uint8_t getNumberOfStates() { return State::size; }

	// Getter for the average EKF update period in s
	float get_dt_ekf_avg() const { return _dt_ekf_avg; }

	// Getters for samples on the delayed time horizon
	const imuSample &get_imu_sample_delayed() const { return _imu_buffer.get_oldest(); }
	const uint64_t &time_delayed_us() const { return _time_delayed_us; }

	// set vehicle landed status data
	void set_in_air_status(bool in_air)
	{
		if (!in_air) {
			if (_control_status.flags.in_air) {
				ECL_DEBUG("no longer in air");
			}

			_time_last_on_ground_us = _time_delayed_us;

		} else {
			if (!_control_status.flags.in_air) {
				ECL_DEBUG("in air");
			}

			_time_last_in_air = _time_delayed_us;
		}

		_control_status.flags.in_air = in_air;
	}

	void set_vehicle_at_rest(bool at_rest)
	{
		if (!_control_status.flags.vehicle_at_rest && at_rest) {
			ECL_DEBUG("at rest");

		} else if (_control_status.flags.vehicle_at_rest && !at_rest) {
			ECL_DEBUG("no longer at rest");
		}

		_control_status.flags.vehicle_at_rest = at_rest;
	}

	// set air density used by the multi-rotor specific drag force fusion
	void set_air_density(float air_density) { _air_density = air_density; }

	// set vehicle is fixed wing status
	void set_is_fixed_wing(bool is_fixed_wing) { _control_status.flags.fixed_wing = is_fixed_wing; }

	// set flag if static pressure rise due to ground effect is expected
	// use _params.gnd_effect_deadzone to adjust for expected rise in static pressure
	// flag will clear after GNDEFFECT_TIMEOUT uSec
	void set_gnd_effect()
	{
		_control_status.flags.gnd_effect = true;
		_time_last_gnd_effect_on = _time_delayed_us;
	}

	void setIMUData(const imuSample &imu_sample);

	void setSystemFlagData(const systemFlagUpdate &system_flags);

#if defined(CONFIG_EKF2_BAROMETER)
	void setBaroData(const baroSample &baro_sample);

	const auto &aid_src_baro_hgt() const { return _aid_src_baro_hgt; }
	const BiasEstimator::status &getBaroBiasEstimatorStatus() const { return _baro_b_est.getStatus(); }
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_TERRAIN)
	// terrain estimate
	bool isTerrainEstimateValid() const;

	uint8_t getTerrainEstimateSensorBitfield() const { return _hagl_sensor_status.value; }

	// get the estimated terrain vertical position relative to the NED origin
	float getTerrainVertPos() const { return _terrain_vpos; };

	// get the number of times the vertical terrain position has been reset
	uint8_t getTerrainVertPosResetCounter() const { return _terrain_vpos_reset_counter; };

	// get the terrain variance
	float get_terrain_var() const { return _terrain_var; }

# if defined(CONFIG_EKF2_RANGE_FINDER)
	const auto &aid_src_terrain_range_finder() const { return _aid_src_terrain_range_finder; }

	void getHaglInnov(float &hagl_innov) const { hagl_innov = _aid_src_terrain_range_finder.innovation; }
	void getHaglInnovVar(float &hagl_innov_var) const { hagl_innov_var = _aid_src_terrain_range_finder.innovation_variance; }
	void getHaglInnovRatio(float &hagl_innov_ratio) const { hagl_innov_ratio = _aid_src_terrain_range_finder.test_ratio; }
# endif // CONFIG_EKF2_RANGE_FINDER

# if defined(CONFIG_EKF2_OPTICAL_FLOW)
	const auto &aid_src_terrain_optical_flow() const { return _aid_src_terrain_optical_flow; }
# endif // CONFIG_EKF2_OPTICAL_FLOW

#endif // CONFIG_EKF2_TERRAIN

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// range height
	void setRangeData(const rangeSample &range_sample);

	// set sensor limitations reported by the rangefinder
	void set_rangefinder_limits(float min_distance, float max_distance) { _range_sensor.setLimits(min_distance, max_distance); }

	const rangeSample &get_rng_sample_delayed() { return *(_range_sensor.getSampleAddress()); }

	const BiasEstimator::status &getRngHgtBiasEstimatorStatus() const { return _rng_hgt_b_est.getStatus(); }
	const auto &aid_src_rng_hgt() const { return _aid_src_rng_hgt; }

	float getHaglRateInnov() const { return _rng_consistency_check.getInnov(); }
	float getHaglRateInnovVar() const { return _rng_consistency_check.getInnovVar(); }
	float getHaglRateInnovRatio() const { return _rng_consistency_check.getSignedTestRatioLpf(); }
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// if optical flow sensor gyro delta angles are not available, set gyro_xyz vector fields to NaN and the EKF will use its internal delta angle data instead
	void setOpticalFlowData(const flowSample &flow);

	// set sensor limitations reported by the optical flow sensor
	void set_optical_flow_limits(float max_flow_rate, float min_distance, float max_distance)
	{
		_flow_max_rate = max_flow_rate;
		_flow_min_distance = min_distance;
		_flow_max_distance = max_distance;
	}

	const auto &aid_src_optical_flow() const { return _aid_src_optical_flow; }

	const Vector2f &getFlowVelBody() const { return _flow_vel_body; }
	const Vector2f &getFlowVelNE() const { return _flow_vel_ne; }

	const Vector2f &getFlowCompensated() const { return _flow_compensated_XY_rad; }
	const Vector2f &getFlowUncompensated() const { return _flow_sample_delayed.flow_xy_rad; }

	const Vector3f getFlowGyro() const { return _flow_sample_delayed.gyro_xyz * (1.f / _flow_sample_delayed.dt); }
	const Vector3f &getFlowGyroIntegral() const { return _flow_sample_delayed.gyro_xyz; }
	const Vector3f &getFlowGyroBias() const { return _flow_gyro_bias; }
	const Vector3f &getRefBodyRate() const { return _ref_body_rate; }
	const Vector3f &getMeasuredBodyRate() const { return _measured_body_rate; }
#endif // CONFIG_EKF2_OPTICAL_FLOW

	float getHeadingInnov() const
	{
#if defined(CONFIG_EKF2_MAGNETOMETER)

		if (_control_status.flags.mag_hdg) {
			return _aid_src_mag_heading.innovation;
		}

		if (_control_status.flags.mag_3D) {
			return Vector3f(_aid_src_mag.innovation).max();
		}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GNSS_YAW)

		if (_control_status.flags.gps_yaw) {
			return _aid_src_gnss_yaw.innovation;
		}

#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

		if (_control_status.flags.ev_yaw) {
			return _aid_src_ev_yaw.innovation;
		}

#endif // CONFIG_EKF2_EXTERNAL_VISION

		return 0.f;
	}

	float getHeadingInnovVar() const
	{
#if defined(CONFIG_EKF2_MAGNETOMETER)

		if (_control_status.flags.mag_hdg) {
			return _aid_src_mag_heading.innovation_variance;
		}

		if (_control_status.flags.mag_3D) {
			return Vector3f(_aid_src_mag.innovation_variance).max();
		}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GNSS_YAW)

		if (_control_status.flags.gps_yaw) {
			return _aid_src_gnss_yaw.innovation_variance;
		}

#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

		if (_control_status.flags.ev_yaw) {
			return _aid_src_ev_yaw.innovation_variance;
		}

#endif // CONFIG_EKF2_EXTERNAL_VISION

		return 0.f;
	}

	float getHeadingInnovRatio() const
	{
#if defined(CONFIG_EKF2_MAGNETOMETER)

		if (_control_status.flags.mag_hdg) {
			return _aid_src_mag_heading.test_ratio;
		}

		if (_control_status.flags.mag_3D) {
			return Vector3f(_aid_src_mag.test_ratio).max();
		}

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_GNSS_YAW)

		if (_control_status.flags.gps_yaw) {
			return _aid_src_gnss_yaw.test_ratio;
		}

#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)

		if (_control_status.flags.ev_yaw) {
			return _aid_src_ev_yaw.test_ratio;
		}

#endif // CONFIG_EKF2_EXTERNAL_VISION

		return 0.f;
	}

#if defined(CONFIG_EKF2_DRAG_FUSION)
	void setDragData(const imuSample &imu);

	const auto &aid_src_drag() const { return _aid_src_drag; }
#endif // CONFIG_EKF2_DRAG_FUSION

	// get the state vector at the delayed time horizon
	matrix::Vector<float, State::size> getStateAtFusionHorizonAsVector() const;

	template <const IdxDof &S>
	matrix::Vector<float, S.dof>getStateVariance() const { return P.slice<S.dof, S.dof>(S.idx, S.idx).diag(); } // calling getStateCovariance().diag() uses more flash space

	template <const IdxDof &S>
	matrix::SquareMatrix<float, S.dof>getStateCovariance() const { return P.slice<S.dof, S.dof>(S.idx, S.idx); }

	// get the full covariance matrix
	const matrix::SquareMatrix<float, State::size> &covariances() const { return P; }

	// get the diagonal elements of the covariance matrix
	matrix::Vector<float, State::size> covariances_diagonal() const { return P.diag(); }

	// orientation state
	const matrix::Quatf &getQuaternion() const { return _output_predictor.getQuaternion(); }
	matrix::Vector<float, State::quat_nominal.dof> getQuaternionVariance() const { return getStateVariance<State::quat_nominal>(); }
	bool attitude_valid() const { return _control_status.flags.tilt_align; } // return true if the attitude is usable
	float getUnaidedYaw() const { return _output_predictor.getUnaidedYaw(); }

	// velocity state
	Vector3f getVelocity() const { return _output_predictor.getVelocity(); }
	Vector3f getVelocityVariance() const { return getStateVariance<State::vel>(); };

	// position state
	Vector3f getPosition() const { return _output_predictor.getPosition(); }
	Vector3f getPositionVariance() const { return getStateVariance<State::pos>(); }

	// gyro bias state
	const Vector3f &getGyroBias() const { return _state.gyro_bias; } // get the gyroscope bias in rad/s
	Vector3f getGyroBiasVariance() const { return getStateVariance<State::gyro_bias>(); } // get the gyroscope bias variance in rad/s
	float getGyroBiasLimit() const { return _params.gyro_bias_lim; }
	bool gyro_bias_inhibited() const { return _gyro_bias_inhibit[0] || _gyro_bias_inhibit[1] || _gyro_bias_inhibit[2]; }

	// accel bias state
	const Vector3f &getAccelBias() const { return _state.accel_bias; } // get the accelerometer bias in m/s**2
	Vector3f getAccelBiasVariance() const { return getStateVariance<State::accel_bias>(); } // get the accelerometer bias variance in m/s**2
	float getAccelBiasLimit() const { return _params.acc_bias_lim; }
	bool accel_bias_inhibited() const { return _accel_bias_inhibit[0] || _accel_bias_inhibit[1] || _accel_bias_inhibit[2]; }

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// mag I state
	const Vector3f &getMagEarthField() const { return _state.mag_I; }

	// mag B state
	const Vector3f &getMagBias() const { return _state.mag_B; }
	Vector3f getMagBiasVariance() const
	{
		if (_control_status.flags.mag) {
			return getStateVariance<State::mag_B>();
		}

		return _saved_mag_bf_covmat.diag();
	}
	float getMagBiasLimit() const { return 0.5f; } // 0.5 Gauss
#endif // CONFIG_EKF2_MAGNETOMETER

	// wind state
	const Vector2f &getWindVelocity() const { return _state.wind_vel; };
	Vector2f getWindVelocityVariance() const { return getStateVariance<State::wind_vel>(); }

	// get EKF mode status
	const filter_control_status_u &control_status() const { return _control_status; }
	const decltype(filter_control_status_u::flags) &control_status_flags() const { return _control_status.flags; }

	const filter_control_status_u &control_status_prev() const { return _control_status_prev; }
	const decltype(filter_control_status_u::flags) &control_status_prev_flags() const { return _control_status_prev.flags; }

	// get EKF internal fault status
	const fault_status_u &fault_status() const { return _fault_status; }
	const decltype(fault_status_u::flags) &fault_status_flags() const { return _fault_status.flags; }

	const innovation_fault_status_u &innov_check_fail_status() const { return _innov_check_fail_status; }
	const decltype(innovation_fault_status_u::flags) &innov_check_fail_status_flags() const { return _innov_check_fail_status.flags; }

	const warning_event_status_u &warning_event_status() const { return _warning_events; }
	const decltype(warning_event_status_u::flags) &warning_event_flags() const { return _warning_events.flags; }
	void clear_warning_events() { _warning_events.value = 0; }

	const information_event_status_u &information_event_status() const { return _information_events; }
	const decltype(information_event_status_u::flags) &information_event_flags() const { return _information_events.flags; }
	void clear_information_events() { _information_events.value = 0; }


	// output predictor
	OutputPredictor &output_predictor() { return _output_predictor; };
	const Vector3f &getVelocityDerivative() const { return _output_predictor.getVelocityDerivative(); }
	float getVerticalPositionDerivative() const { return _output_predictor.getVerticalPositionDerivative(); }

	const Vector3f &getOutputTrackingError() const { return _output_predictor.getOutputTrackingError(); }



	// get the ekf WGS-84 origin position and height and the system time it was last set
	// return true if the origin is valid
	bool getEkfGlobalOrigin(uint64_t &origin_time, double &latitude, double &longitude, float &origin_alt) const;
	bool setEkfGlobalOrigin(const double latitude, const double longitude, const float altitude);

	float getEkfGlobalOriginAltitude() const { return PX4_ISFINITE(_gps_alt_ref) ? _gps_alt_ref : 0.f; }
	bool setEkfGlobalOriginAltitude(const float altitude);

	const bool &global_origin_valid() const { return _NED_origin_initialised; }
	const MapProjection &global_origin() const { return _pos_ref; }

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

	float gps_horizontal_position_drift_rate_m_s() const { return _gps_horizontal_position_drift_rate_m_s; }
	float gps_vertical_position_drift_rate_m_s() const { return _gps_vertical_position_drift_rate_m_s; }
	float gps_filtered_horizontal_velocity_m_s() const { return _gps_filtered_horizontal_velocity_m_s; }

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
	float getYawVar() const;

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

#if defined(CONFIG_EKF2_AIRSPEED)
	void setAirspeedData(const airspeedSample &airspeed_sample);

	const auto &aid_src_airspeed() const { return _aid_src_airspeed; }
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	const auto &aid_src_sideslip() const { return _aid_src_sideslip; }
#endif // CONFIG_EKF2_SIDESLIP

	const auto &aid_src_fake_hgt() const { return _aid_src_fake_hgt; }
	const auto &aid_src_fake_pos() const { return _aid_src_fake_pos; }

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	void setExtVisionData(const extVisionSample &evdata);

	const auto &aid_src_ev_hgt() const { return _aid_src_ev_hgt; }
	const auto &aid_src_ev_pos() const { return _aid_src_ev_pos; }
	const auto &aid_src_ev_vel() const { return _aid_src_ev_vel; }
	const auto &aid_src_ev_yaw() const { return _aid_src_ev_yaw; }

	const BiasEstimator::status &getEvHgtBiasEstimatorStatus() const { return _ev_hgt_b_est.getStatus(); }
	const BiasEstimator::status &getEvPosBiasEstimatorStatus(int i) const { return _ev_pos_b_est.getStatus(i); }
#endif // CONFIG_EKF2_EXTERNAL_VISION

	void setGpsData(const gpsMessage &gps);

	const gpsSample &get_gps_sample_delayed() const { return _gps_sample_delayed; }

	// ask estimator for sensor data collection decision and do any preprocessing if required, returns true if not defined
	bool collect_gps(const gpsMessage &gps);

	const auto &aid_src_gnss_hgt() const { return _aid_src_gnss_hgt; }
	const auto &aid_src_gnss_pos() const { return _aid_src_gnss_pos; }
	const auto &aid_src_gnss_vel() const { return _aid_src_gnss_vel; }

	const BiasEstimator::status &getGpsHgtBiasEstimatorStatus() const { return _gps_hgt_b_est.getStatus(); }

#if defined(CONFIG_EKF2_GNSS_YAW)
	const auto &aid_src_gnss_yaw() const { return _aid_src_gnss_yaw; }
#endif // CONFIG_EKF2_GNSS_YAW

#if defined(CONFIG_EKF2_MAGNETOMETER)
	void setMagData(const magSample &mag_sample);

	const auto &aid_src_mag_heading() const { return _aid_src_mag_heading; }
	const auto &aid_src_mag() const { return _aid_src_mag; }

	// Get the value of magnetic declination in degrees to be saved for use at the next startup
	// Returns true when the declination can be saved
	// At the next startup, set param.mag_declination_deg to the value saved
	bool get_mag_decl_deg(float &val) const
	{
		if (_NED_origin_initialised && (_params.mag_declination_source & GeoDeclinationMask::SAVE_GEO_DECL)) {
			val = math::degrees(_mag_declination_gps);
			return true;

		} else {
			return false;
		}
	}

	bool get_mag_inc_deg(float &val) const
	{
		if (_NED_origin_initialised) {
			val = math::degrees(_mag_inclination_gps);
			return true;

		} else {
			return false;
		}
	}

	void get_mag_checks(float &inc_deg, float &inc_ref_deg, float &strength_gs, float &strength_ref_gs) const
	{
		inc_deg = math::degrees(_mag_inclination);
		inc_ref_deg = math::degrees(_mag_inclination_gps);
		strength_gs = _mag_strength;
		strength_ref_gs = _mag_strength_gps;
	}
#endif // CONFIG_EKF2_MAGNETOMETER

	const auto &aid_src_gravity() const { return _aid_src_gravity; }

#if defined(CONFIG_EKF2_AUXVEL)
	void setAuxVelData(const auxVelSample &auxvel_sample);

	const auto &aid_src_aux_vel() const { return _aid_src_aux_vel; }
#endif // CONFIG_EKF2_AUXVEL


	// the flags considered are opt_flow, gps, ev_vel and ev_pos
	bool isOnlyActiveSourceOfHorizontalAiding(bool aiding_flag) const { return aiding_flag && !isOtherSourceOfHorizontalAidingThan(aiding_flag); }

	/*
	 * Check if there are any other active source of horizontal aiding
	 * Warning: does not tell if the selected source is
	 * active, use isOnlyActiveSourceOfHorizontalAiding() for this
	 *
	 * The flags considered are opt_flow, gps, ev_vel and ev_pos
	 *
	 * @param aiding_flag a flag in _control_status.flags
	 * @return true if an other source than aiding_flag is active
	 */
	bool isOtherSourceOfHorizontalAidingThan(bool aiding_flag) const;

	// Return true if at least one source of horizontal aiding is active
	// the flags considered are opt_flow, gps, ev_vel and ev_pos
	bool isHorizontalAidingActive() const { return getNumberOfActiveHorizontalAidingSources() > 0; }
	bool isVerticalAidingActive() const { return isVerticalPositionAidingActive() || isVerticalVelocityAidingActive(); }

	int getNumberOfActiveHorizontalAidingSources() const;

	bool isOtherSourceOfVerticalPositionAidingThan(bool aiding_flag) const;
	bool isVerticalPositionAidingActive() const { return getNumberOfActiveVerticalPositionAidingSources() > 0; }
	bool isOnlyActiveSourceOfVerticalPositionAiding(bool aiding_flag) const { return aiding_flag && !isOtherSourceOfVerticalPositionAidingThan(aiding_flag); }
	int getNumberOfActiveVerticalPositionAidingSources() const;

	bool isVerticalVelocityAidingActive() const { return getNumberOfActiveVerticalVelocityAidingSources() > 0; }
	int getNumberOfActiveVerticalVelocityAidingSources() const { return int(_control_status.flags.gps) + int(_control_status.flags.ev_vel);}

private:
	// set the internal states and status to their default value
	void reset();

	// allocate data buffers and initialize interface variables
	bool initialise_interface(uint64_t timestamp);

	bool initialiseTilt();

	void printBufferAllocationFailed(const char *buffer_name);

	// check if the EKF is dead reckoning horizontal velocity using inertial data only
	void updateDeadReckoningStatus();
	void updateHorizontalDeadReckoningstatus();
	void updateVerticalDeadReckoningStatus();

	SquareMatrixState P{};	///< state covariance matrix
	stateSample _state{};		///< state struct of the ekf running at the delayed time horizon

	bool _initialised{false};      // true if the ekf interface instance (data buffering) is initialized
	bool _filter_initialised{false};	///< true when the EKF sttes and covariances been initialised
	bool _imu_updated{false};      // true if the ekf should update (completed downsampling process)

	static constexpr uint8_t kBufferLengthDefault = 12;
	RingBuffer<imuSample> _imu_buffer{kBufferLengthDefault};
	RingBuffer<systemFlagUpdate> *_system_flag_buffer{nullptr};

	parameters _params{};		// filter parameters

	/*
	 OBS_BUFFER_LENGTH defines how many observations (non-IMU measurements) we can buffer
	 which sets the maximum frequency at which we can process non-IMU measurements. Measurements that
	 arrive too soon after the previous measurement will not be processed.
	 max freq (Hz) = (OBS_BUFFER_LENGTH - 1) / (IMU_BUFFER_LENGTH * FILTER_UPDATE_PERIOD_S)
	 This can be adjusted to match the max sensor data rate plus some margin for jitter.
	*/
	uint8_t _obs_buffer_length{0};

	/*
	IMU_BUFFER_LENGTH defines how many IMU samples we buffer which sets the time delay from current time to the
	EKF fusion time horizon and therefore the maximum sensor time offset relative to the IMU that we can compensate for.
	max sensor time offet (msec) =  IMU_BUFFER_LENGTH * FILTER_UPDATE_PERIOD_MS
	This can be adjusted to a value that is FILTER_UPDATE_PERIOD_MS longer than the maximum observation time delay.
	*/
	uint8_t _imu_buffer_length{0};

	unsigned _min_obs_interval_us{0}; // minimum time interval between observations that will guarantee data is not lost (usec)

	float _dt_ekf_avg{0.010f}; ///< average update rate of the ekf in s

	uint64_t _time_delayed_us{0}; // captures the imu sample on the delayed time horizon
	uint64_t _time_latest_us{0}; // imu sample capturing the newest imu data

	ImuDownSampler _imu_down_sampler{_params.filter_update_interval_us};
	OutputPredictor _output_predictor{};


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

	float _air_density{CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C};		// air density (kg/m**3)

	RingBuffer<gpsSample> *_gps_buffer{nullptr};
	uint64_t _time_last_gps_buffer_push{0};

	bool _gps_data_ready{false};	///< true when new GPS data has fallen behind the fusion time horizon and is available to be fused

	bool _NED_origin_initialised{false};
	float _gpos_origin_eph{0.0f}; // horizontal position uncertainty of the global origin
	float _gpos_origin_epv{0.0f}; // vertical position uncertainty of the global origin
	MapProjection _pos_ref{}; // Contains WGS-84 position latitude and longitude of the EKF origin
	MapProjection _gps_pos_prev{}; // Contains WGS-84 position latitude and longitude of the previous GPS message
	float _gps_alt_prev{0.0f};	// height from the previous GPS message (m)

	float _gps_horizontal_position_drift_rate_m_s{NAN}; // Horizontal position drift rate (m/s)
	float _gps_vertical_position_drift_rate_m_s{NAN};   // Vertical position drift rate (m/s)
	float _gps_filtered_horizontal_velocity_m_s{NAN};   // Filtered horizontal velocity (m/s)

	uint64_t _time_last_on_ground_us{0};	///< last time we were on the ground (uSec)
	uint64_t _time_last_in_air{0};		///< last time we were in air (uSec)
	uint64_t _time_last_gnd_effect_on{0};

	uint64_t _time_last_horizontal_aiding{0}; ///< amount of time we have been doing inertial only deadreckoning (uSec)
	uint64_t _time_last_v_pos_aiding{0};
	uint64_t _time_last_v_vel_aiding{0};

	uint64_t _time_last_hor_pos_fuse{0};	///< time the last fusion of horizontal position measurements was performed (uSec)
	uint64_t _time_last_hgt_fuse{0};	///< time the last fusion of vertical position measurements was performed (uSec)
	uint64_t _time_last_hor_vel_fuse{0};	///< time the last fusion of horizontal velocity measurements was performed (uSec)
	uint64_t _time_last_ver_vel_fuse{0};	///< time the last fusion of verticalvelocity measurements was performed (uSec)
	uint64_t _time_last_heading_fuse{0};
	uint64_t _time_last_zero_velocity_fuse{0}; ///< last time of zero velocity update (uSec)

	bool _horizontal_deadreckon_time_exceeded{true};
	bool _vertical_position_deadreckon_time_exceeded{true};
	bool _vertical_velocity_deadreckon_time_exceeded{true};

	Vector3f _last_known_pos{};		///< last known local position vector (m)

	uint64_t _time_acc_bias_check{0};	///< last time the  accel bias check passed (uSec)

	Vector3f _earth_rate_NED{};	///< earth rotation vector (NED) in rad/s

	Dcmf _R_to_earth{};	///< transformation matrix from body frame to earth frame from last EKF prediction

	// zero gyro update
	Vector3f _zgup_delta_ang{};
	float _zgup_delta_ang_dt{0.f};

	Vector2f _accel_lpf_NE{};			///< Low pass filtered horizontal earth frame acceleration (m/sec**2)
	float _yaw_delta_ef{0.0f};		///< Recent change in yaw angle measured about the earth frame D axis (rad)
	float _yaw_rate_lpf_ef{0.0f};		///< Filtered angular rate about earth frame D axis (rad/sec)

#if defined(CONFIG_EKF2_DRAG_FUSION)
	RingBuffer<dragSample> *_drag_buffer {nullptr};
	dragSample _drag_down_sampled{};	// down sampled drag specific force data (filter prediction rate -> observation rate)

	// Used by the multi-rotor specific drag force fusion
	uint8_t _drag_sample_count{0};	// number of drag specific force samples assumulated at the filter prediction rate
	float _drag_sample_time_dt{0.0f};	// time integral across all samples used to form _drag_down_sampled (sec)

	estimator_aid_source2d_s _aid_src_drag{};
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_TERRAIN)
	// Terrain height state estimation
	float _terrain_vpos{0.0f};		///< estimated vertical position of the terrain underneath the vehicle in local NED frame (m)
	float _terrain_var{1e4f};		///< variance of terrain position estimate (m**2)
	uint8_t _terrain_vpos_reset_counter{0};	///< number of times _terrain_vpos has been reset

	terrain_fusion_status_u _hagl_sensor_status{}; ///< Struct indicating type of sensor used to estimate height above ground
	float _last_on_ground_posD{0.0f};	///< last vertical position when the in_air status was false (m)

# if defined(CONFIG_EKF2_RANGE_FINDER)
	RingBuffer<rangeSample> *_range_buffer {nullptr};
	uint64_t _time_last_range_buffer_push{0};

	sensor::SensorRangeFinder _range_sensor{};
	RangeFinderConsistencyCheck _rng_consistency_check;

	estimator_aid_source1d_s _aid_src_terrain_range_finder{};
	uint64_t _time_last_healthy_rng_data{0};
# endif // CONFIG_EKF2_RANGE_FINDER

# if defined(CONFIG_EKF2_OPTICAL_FLOW)
	RingBuffer<flowSample> 	*_flow_buffer {nullptr};

	flowSample _flow_sample_delayed{};

	// Sensor limitations
	float _flow_max_rate{1.0f}; ///< maximum angular flow rate that the optical flow sensor can measure (rad/s)
	float _flow_min_distance{0.0f};	///< minimum distance that the optical flow sensor can operate at (m)
	float _flow_max_distance{10.f};	///< maximum distance that the optical flow sensor can operate at (m)

	estimator_aid_source2d_s _aid_src_terrain_optical_flow{};
# endif // CONFIG_EKF2_OPTICAL_FLOW

#endif // CONFIG_EKF2_TERRAIN

#if defined(CONFIG_EKF2_RANGE_FINDER)
	estimator_aid_source1d_s _aid_src_rng_hgt {};
	HeightBiasEstimator _rng_hgt_b_est{HeightSensor::RANGE, _height_sensor_ref};
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	estimator_aid_source2d_s _aid_src_optical_flow {};

	// optical flow processing
	Vector3f _flow_gyro_bias{};	///< bias errors in optical flow sensor rate gyro outputs (rad/sec)
	Vector2f _flow_vel_body{};	///< velocity from corrected flow measurement (body frame)(m/s)
	Vector2f _flow_vel_ne{};		///< velocity from corrected flow measurement (local frame) (m/s)
	Vector3f _imu_del_ang_of{};	///< bias corrected delta angle measurements accumulated across the same time frame as the optical flow rates (rad)
	Vector3f _ref_body_rate{};
	Vector3f _measured_body_rate{};

	float _delta_time_of{0.0f};	///< time in sec that _imu_del_ang_of was accumulated over (sec)
	uint64_t _time_bad_motion_us{0};	///< last system time that on-ground motion exceeded limits (uSec)
	uint64_t _time_good_motion_us{0};	///< last system time that on-ground motion was within limits (uSec)
	Vector2f _flow_compensated_XY_rad{};	///< measured delta angle of the image about the X and Y body axes after removal of body rotation (rad), RH rotation is positive

	bool _flow_data_ready{false};	///< true when the leading edge of the optical flow integration period has fallen behind the fusion time horizon
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_AIRSPEED)
	RingBuffer<airspeedSample> *_airspeed_buffer {nullptr};
	airspeedSample _airspeed_sample_delayed{};

	estimator_aid_source1d_s _aid_src_airspeed{};
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_SIDESLIP)
	estimator_aid_source1d_s _aid_src_sideslip {};
#endif // CONFIG_EKF2_SIDESLIP

	estimator_aid_source2d_s _aid_src_fake_pos{};
	estimator_aid_source1d_s _aid_src_fake_hgt{};

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	RingBuffer<extVisionSample> *_ext_vision_buffer {nullptr};
	uint64_t _time_last_ext_vision_buffer_push{0};

	extVisionSample _ev_sample_prev{};

	estimator_aid_source1d_s _aid_src_ev_hgt{};
	estimator_aid_source2d_s _aid_src_ev_pos{};
	estimator_aid_source3d_s _aid_src_ev_vel{};
	estimator_aid_source1d_s _aid_src_ev_yaw{};

	float _ev_yaw_pred_prev{}; ///< previous value of yaw state used by odometry fusion (m)

	uint8_t _nb_ev_pos_reset_available{0};
	uint8_t _nb_ev_vel_reset_available{0};
	uint8_t _nb_ev_yaw_reset_available{0};
#endif // CONFIG_EKF2_EXTERNAL_VISION

	estimator_aid_source1d_s _aid_src_gnss_hgt{};
	estimator_aid_source2d_s _aid_src_gnss_pos{};
	estimator_aid_source3d_s _aid_src_gnss_vel{};

#if defined(CONFIG_EKF2_GNSS_YAW)
	float _gps_yaw_offset {0.0f};	// Yaw offset angle for dual GPS antennas used for yaw estimation (radians).
	// innovation consistency check monitoring ratios
	AlphaFilter<float> _gnss_yaw_signed_test_ratio_lpf{0.1f}; // average signed test ratio used to detect a bias in the state
	uint64_t _time_last_gps_yaw_buffer_push{0};

	estimator_aid_source1d_s _aid_src_gnss_yaw{};
	uint8_t _nb_gps_yaw_reset_available{0}; ///< remaining number of resets allowed before switching to another aiding source
#endif // CONFIG_EKF2_GNSS_YAW

	estimator_aid_source3d_s _aid_src_gravity{};

#if defined(CONFIG_EKF2_AUXVEL)
	RingBuffer<auxVelSample> *_auxvel_buffer {nullptr};

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

	uint64_t _wmm_gps_time_last_checked{0};  // time WMM last checked
	uint64_t _wmm_gps_time_last_set{0};      // time WMM last set
	float _mag_declination_gps{NAN};         // magnetic declination returned by the geo library using the last valid GPS position (rad)
	float _mag_inclination_gps{NAN};	  // magnetic inclination returned by the geo library using the last valid GPS position (rad)
	float _mag_strength_gps{NAN};	          // magnetic strength returned by the geo library using the last valid GPS position (T)

	float _mag_inclination{NAN};
	float _mag_strength{NAN};

	// Variables used by the initial filter alignment
	bool _is_first_imu_sample{true};
	AlphaFilter<Vector3f> _accel_lpf{0.1f};	///< filtered accelerometer measurement used to align tilt (m/s/s)
	AlphaFilter<Vector3f> _gyro_lpf{0.1f};	///< filtered gyro measurement used for alignment excessive movement check (rad/sec)

#if defined(CONFIG_EKF2_BAROMETER)
	RingBuffer<baroSample> *_baro_buffer {nullptr};
	uint64_t _time_last_baro_buffer_push{0};

	estimator_aid_source1d_s _aid_src_baro_hgt{};

	// Variables used to perform in flight resets and switch between height sources
	AlphaFilter<float> _baro_lpf{0.1f};	///< filtered barometric height measurement (m)
	uint32_t _baro_counter{0};		///< number of baro samples read during initialisation

	HeightBiasEstimator _baro_b_est{HeightSensor::BARO, _height_sensor_ref};

	bool _baro_hgt_faulty{false};		///< true if baro data have been declared faulty TODO: move to fault flags
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_MAGNETOMETER)
	RingBuffer<magSample> *_mag_buffer {nullptr};
	uint64_t _time_last_mag_buffer_push{0};

	float _mag_heading_prev{};                 ///< previous value of mag heading (rad)
	float _mag_heading_pred_prev{};            ///< previous value of yaw state used by mag heading fusion (rad)

	// used by magnetometer fusion mode selection
	bool _mag_bias_observable{false};	///< true when there is enough rotation to make magnetometer bias errors observable
	bool _yaw_angle_observable{false};	///< true when there is enough horizontal acceleration to make yaw observable
	uint64_t _time_yaw_started{0};		///< last system time in usec that a yaw rotation manoeuvre was detected
	AlphaFilter<float> _mag_heading_innov_lpf{0.1f};
	float _mag_heading_last_declination{}; ///< last magnetic field declination used for heading fusion (rad)
	bool _mag_decl_cov_reset{false};	///< true after the fuseDeclination() function has been used to modify the earth field covariances after a magnetic field reset event.
	uint8_t _nb_mag_heading_reset_available{0};
	uint8_t _nb_mag_3d_reset_available{0};
	uint32_t _min_mag_health_time_us{1'000'000}; ///< magnetometer is marked as healthy only after this amount of time

	estimator_aid_source1d_s _aid_src_mag_heading{};
	estimator_aid_source3d_s _aid_src_mag{};

	AlphaFilter<Vector3f> _mag_lpf{0.1f};	///< filtered magnetometer measurement for instant reset (Gauss)
	uint32_t _mag_counter{0};		///< number of magnetometer samples read during initialisation

	// Variables used to control activation of post takeoff functionality
	uint64_t _flt_mag_align_start_time{0};	///< time that inflight magnetic field alignment started (uSec)
	uint64_t _time_last_mov_3d_mag_suitable{0};	///< last system time that sufficient movement to use 3-axis magnetometer fusion was detected (uSec)
	uint64_t _time_last_mag_check_failing{0};
	Matrix3f _saved_mag_ef_covmat{}; ///< NED magnetic field state covariance sub-matrix saved for use at the next initialisation (Gauss**2)
	Matrix3f _saved_mag_bf_covmat{}; ///< magnetic field state covariance sub-matrix that has been saved for use at the next initialisation (Gauss**2)
#endif // CONFIG_EKF2_MAGNETOMETER

	// this is the current status of the filter control modes
	filter_control_status_u _control_status{};

	// this is the previous status of the filter control modes - used to detect mode transitions
	filter_control_status_u _control_status_prev{};

	fault_status_u _fault_status{};
	innovation_fault_status_u _innov_check_fail_status{};

	// these are used to record single frame events for external monitoring and should NOT be used for
	// state logic becasue they will be cleared externally after being read.
	warning_event_status_u _warning_events{};
	information_event_status_u _information_events{};

	gps_check_fail_status_u _gps_check_fail_status{};

	// variables used to inhibit accel bias learning
	bool _accel_bias_inhibit[3] {};		///< true when the accel bias learning is being inhibited for the specified axis
	bool _gyro_bias_inhibit[3] {};		///< true when the gyro bias learning is being inhibited for the specified axis
	Vector3f _accel_vec_filt{};		///< acceleration vector after application of a low pass filter (m/sec**2)
	float _accel_magnitude_filt{0.0f};	///< acceleration magnitude after application of a decaying envelope filter (rad/sec)
	float _ang_rate_magnitude_filt{0.0f};		///< angular rate magnitude after application of a decaying envelope filter (rad/sec)

	Vector3f _prev_gyro_bias_var{};         ///< saved gyro XYZ bias variances
	Vector3f _prev_accel_bias_var{};        ///< saved accel XYZ bias variances

	// height sensor status
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

	template <const IdxDof &S>
	void resetStateCovariance(const matrix::SquareMatrix<float, S.dof> &cov)
	{
		P.uncorrelateCovarianceSetVariance<S.dof>(S.idx, 0.0f);
		P.slice<S.dof, S.dof>(S.idx, S.idx) = cov;
	}

	// update quaternion states and covariances using an innovation, observation variance and Jacobian vector
	bool fuseYaw(estimator_aid_source1d_s &aid_src_status);
	bool fuseYaw(estimator_aid_source1d_s &aid_src_status, const VectorState &H_YAW);
	void computeYawInnovVarAndH(float variance, float &innovation_variance, VectorState &H_YAW) const;

#if defined(CONFIG_EKF2_GNSS_YAW)
	void controlGpsYawFusion(const gpsSample &gps_sample, bool gps_checks_passing, bool gps_checks_failing);

	// fuse the yaw angle obtained from a dual antenna GPS unit
	void fuseGpsYaw();

	// reset the quaternions states using the yaw angle obtained from a dual antenna GPS unit
	// return true if the reset was successful
	bool resetYawToGps(const float gnss_yaw);

	void updateGpsYaw(const gpsSample &gps_sample);

#endif // CONFIG_EKF2_GNSS_YAW
	void stopGpsYawFusion();

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// ekf sequential fusion of magnetometer measurements
	bool fuseMag(const Vector3f &mag, estimator_aid_source3d_s &aid_src_mag, bool update_all_states = true);

	// fuse magnetometer declination measurement
	// argument passed in is the declination uncertainty in radians
	bool fuseDeclination(float decl_sigma);

	// apply sensible limits to the declination and length of the NE mag field states estimates
	void limitDeclination();
#endif // CONFIG_EKF2_MAGNETOMETER

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
	void controlDragFusion(const imuSample &imu_delayed);

	// fuse body frame drag specific forces for multi-rotor wind estimation
	void fuseDrag(const dragSample &drag_sample);
#endif // CONFIG_EKF2_DRAG_FUSION

	// fuse single velocity and position measurement
	bool fuseVelPosHeight(const float innov, const float innov_var, const int state_index);

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
	void updateHorizontalPositionAidSrcStatus(const uint64_t &time_us, const Vector2f &obs, const Vector2f &obs_var,
			const float innov_gate, estimator_aid_source2d_s &aid_src) const;
	void updateVerticalPositionAidSrcStatus(const uint64_t &time_us, const float obs, const float obs_var,
						const float innov_gate, estimator_aid_source1d_s &aid_src) const;

	// 2d & 3d velocity aid source
	void updateVelocityAidSrcStatus(const uint64_t &time_us, const Vector2f &obs, const Vector2f &obs_var,
					const float innov_gate, estimator_aid_source2d_s &aid_src) const;
	void updateVelocityAidSrcStatus(const uint64_t &time_us, const Vector3f &obs, const Vector3f &obs_var,
					const float innov_gate, estimator_aid_source3d_s &aid_src) const;

	// horizontal and vertical position fusion
	void fuseHorizontalPosition(estimator_aid_source2d_s &pos_aid_src);
	void fuseVerticalPosition(estimator_aid_source1d_s &hgt_aid_src);

	// 2d & 3d velocity fusion
	void fuseVelocity(estimator_aid_source2d_s &vel_aid_src);
	void fuseVelocity(estimator_aid_source3d_s &vel_aid_src);

#if defined(CONFIG_EKF2_TERRAIN)
	// terrain vertical position estimator
	void initHagl();
	void runTerrainEstimator(const imuSample &imu_delayed);
	void predictHagl(const imuSample &imu_delayed);

	float getTerrainVPos() const { return isTerrainEstimateValid() ? _terrain_vpos : _last_on_ground_posD; }

	void controlHaglFakeFusion();
	void terrainHandleVerticalPositionReset(float delta_z);

# if defined(CONFIG_EKF2_RANGE_FINDER)
	// update the terrain vertical position estimate using a height above ground measurement from the range finder
	void controlHaglRngFusion();
	void updateHaglRng(estimator_aid_source1d_s &aid_src) const;
	void fuseHaglRng(estimator_aid_source1d_s &aid_src);
	void resetHaglRng();
	void stopHaglRngFusion();
	float getRngVar() const;
# endif // CONFIG_EKF2_RANGE_FINDER

# if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// update the terrain vertical position estimate using an optical flow measurement
	void controlHaglFlowFusion();
	void resetHaglFlow();
	void stopHaglFlowFusion();
	void fuseFlowForTerrain(estimator_aid_source2d_s &flow);
# endif // CONFIG_EKF2_OPTICAL_FLOW

#endif // CONFIG_EKF2_TERRAIN

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// range height
	void controlRangeHeightFusion();
	bool isConditionalRangeAidSuitable();
	void stopRngHgtFusion();
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
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// Return the magnetic declination in radians to be used by the alignment and fusion processing
	float getMagDeclination();
#endif // CONFIG_EKF2_MAGNETOMETER

	void clearInhibitedStateKalmanGains(VectorState &K) const
	{
		for (unsigned i = 0; i < State::gyro_bias.dof; i++) {
			if (_gyro_bias_inhibit[i]) {
				K(State::gyro_bias.idx + i) = 0.f;
			}
		}

		for (unsigned i = 0; i < State::accel_bias.dof; i++) {
			if (_accel_bias_inhibit[i]) {
				K(State::accel_bias.idx + i) = 0.f;
			}
		}

		if (!_control_status.flags.mag) {
			for (unsigned i = 0; i < State::mag_I.dof; i++) {
				K(State::mag_I.idx + i) = 0.f;
			}
		}

		if (!_control_status.flags.mag) {
			for (unsigned i = 0; i < State::mag_B.dof; i++) {
				K(State::mag_B.idx + i) = 0.f;
			}
		}

		if (!_control_status.flags.wind) {
			for (unsigned i = 0; i < State::wind_vel.dof; i++) {
				K(State::wind_vel.idx + i) = 0.f;
			}
		}
	}

	bool measurementUpdate(VectorState &K, float innovation_variance, float innovation)
	{
		clearInhibitedStateKalmanGains(K);

		const VectorState KS = K * innovation_variance;
		SquareMatrixState KHP;

		for (unsigned row = 0; row < State::size; row++) {
			for (unsigned col = 0; col < State::size; col++) {
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
	bool checkAndFixCovarianceUpdate(const SquareMatrixState &KHP);

	// limit the diagonal of the covariance matrix
	// force symmetry when the argument is true
	void fixCovarianceErrors(bool force_symmetry);

	void constrainStateVar(const IdxDof &state, float min, float max);

	// constrain the ekf states
	void constrainStates();

	// generic function which will perform a fusion step given a kalman gain K
	// and a scalar innovation value
	void fuse(const VectorState &K, float innovation);

#if defined(CONFIG_EKF2_BARO_COMPENSATION)
	float compensateBaroForDynamicPressure(float baro_alt_uncompensated) const;
#endif // CONFIG_EKF2_BARO_COMPENSATION

	// calculate the earth rotation vector from a given latitude
	Vector3f calcEarthRateNED(float lat_rad) const;

	// return true id the GPS quality is good enough to set an origin and start aiding
	bool gps_is_good(const gpsMessage &gps);

	// Control the filter fusion modes
	void controlFusionModes(const imuSample &imu_delayed);

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// control fusion of external vision observations
	void controlExternalVisionFusion();
	void updateEvAttitudeErrorFilter(extVisionSample &ev_sample, bool ev_reset);
	void controlEvHeightFusion(const extVisionSample &ev_sample, const bool common_starting_conditions_passing,
				   const bool ev_reset, const bool quality_sufficient, estimator_aid_source1d_s &aid_src);
	void controlEvPosFusion(const extVisionSample &ev_sample, const bool common_starting_conditions_passing,
				const bool ev_reset, const bool quality_sufficient, estimator_aid_source2d_s &aid_src);
	void controlEvVelFusion(const extVisionSample &ev_sample, const bool common_starting_conditions_passing,
				const bool ev_reset, const bool quality_sufficient, estimator_aid_source3d_s &aid_src);
	void controlEvYawFusion(const extVisionSample &ev_sample, const bool common_starting_conditions_passing,
				const bool ev_reset, const bool quality_sufficient, estimator_aid_source1d_s &aid_src);

	void startEvPosFusion(const Vector2f &measurement, const Vector2f &measurement_var, estimator_aid_source2d_s &aid_src);
	void updateEvPosFusion(const Vector2f &measurement, const Vector2f &measurement_var, bool quality_sufficient,
			       bool reset, estimator_aid_source2d_s &aid_src);
	void stopEvPosFusion();
	void stopEvHgtFusion();
	void stopEvVelFusion();
	void stopEvYawFusion();
#endif // CONFIG_EKF2_EXTERNAL_VISION

	// control fusion of GPS observations
	void controlGpsFusion(const imuSample &imu_delayed);
	bool shouldResetGpsFusion() const;
	bool isYawFailure() const;

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// control fusion of magnetometer observations
	void controlMagFusion();
	void controlMagHeadingFusion(const magSample &mag_sample, const bool common_starting_conditions_passing,
				     estimator_aid_source1d_s &aid_src);
	void controlMag3DFusion(const magSample &mag_sample, const bool common_starting_conditions_passing,
				estimator_aid_source3d_s &aid_src);

	bool checkHaglYawResetReq() const;

	void resetMagHeading(const Vector3f &mag);
	void resetMagStates(const Vector3f &mag, bool reset_heading = true);
	bool haglYawResetReq();

	void checkYawAngleObservability();
	void checkMagBiasObservability();
	void checkMagHeadingConsistency();

	bool checkMagField(const Vector3f &mag);
	static bool isMeasuredMatchingExpected(float measured, float expected, float gate);

	void stopMagHdgFusion();
	void stopMagFusion();

	// load and save mag field state covariance data for re-use
	void loadMagCovData();
	void saveMagCovData();

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

	void controlZeroVelocityUpdate();
	void controlZeroGyroUpdate(const imuSample &imu_delayed);
	void fuseDeltaAngBias(float innov, float innov_var, int obs_index);

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
	void controlGnssHeightFusion(const gpsSample &gps_sample);

#if defined(CONFIG_EKF2_BAROMETER)
	void controlBaroHeightFusion();
	void stopBaroHgtFusion();

	void updateGroundEffect();
#endif // CONFIG_EKF2_BAROMETER

	void stopGpsHgtFusion();

	// gravity fusion: heuristically enable / disable gravity fusion
	void controlGravityFusion(const imuSample &imu_delayed);

	void resetQuatCov(const float yaw_noise = NAN);
	void resetQuatCov(const Vector3f &euler_noise_ned);

	void resetMagCov();

	// perform a reset of the wind states and related covariances
	void resetWind();
	void resetWindToZero();

	void resetGyroBiasZCov();

	// uncorrelate quaternion states from other states
	void uncorrelateQuatFromOtherStates();

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

	void resetFakePosFusion();
	void stopFakePosFusion();

	void setVelPosStatus(const int state_index, const bool healthy);

	// reset the quaternion states and covariances to the new yaw value, preserving the roll and pitch
	// yaw : Euler yaw angle (rad)
	// yaw_variance : yaw error variance (rad^2)
	void resetQuatStateYaw(float yaw, float yaw_variance);

	// Declarations used to control use of the EKF-GSF yaw estimator

	// yaw estimator instance
	EKFGSF_yaw _yawEstimator{};

	uint8_t _height_sensor_ref{HeightSensor::UNKNOWN};
	uint8_t _position_sensor_ref{static_cast<uint8_t>(PositionSensor::GNSS)};

	// measurement samples capturing measurements on the delayed time horizon
	gpsSample _gps_sample_delayed{};
	HeightBiasEstimator _gps_hgt_b_est{HeightSensor::GNSS, _height_sensor_ref};

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	HeightBiasEstimator _ev_hgt_b_est {HeightSensor::EV, _height_sensor_ref};
	PositionBiasEstimator _ev_pos_b_est{static_cast<uint8_t>(PositionSensor::EV), _position_sensor_ref};
	AlphaFilter<Quatf> _ev_q_error_filt{0.001f};
	bool _ev_q_error_initialized{false};
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
