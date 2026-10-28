/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file VTEPosition.h
 * @brief Estimate the state of a target by processing and fusing sensor data in
 * a Kalman Filter.
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#pragma once

#include "../common.h"
#include "KF_position.h"
#include <drivers/drv_hrt.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/perf/perf_counter.h>
#include <matrix/math.hpp>
#include <parameters/param.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/fiducial_marker_pos_report.h>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/target_gnss.h>
#include <uORB/topics/vehicle_gnss.h>
#include <uORB/topics/vte_bias_init_status.h>
#include <uORB/topics/vte_position.h>
#include <uORB/topics/vte_aid_source3d.h>
#include <vtest_derivation/generated/state.h>

class VisionTargetEstTestable;

namespace vision_target_estimator
{

class VTEPosition : public ModuleParams
{
public:
	VTEPosition();
	virtual ~VTEPosition();

	/**
	 * Run the predict/update cycle once. Pulls in all new sensor samples, initializes the
	 * filter on the first viable set, fuses them in order of effective fusion time, and publishes the state.
	 * @param acc_ned  Latest UAV acceleration in NED, used as prediction input.
	 */
	void update(const matrix::Vector3f &acc_ned);

	/** Drop the estimator state and publish it as invalid. A nonzero sample boundary also invalidates cached NED inputs. */
	void resetFilter(hrt_abstime min_sample_time = 0);

	/** Cache the task-specific absolute reference used to build the mission GNSS observation. */
	void setMissionPosition(double lat_deg, double lon_deg, float alt_m);
	/** Clear the task-specific absolute reference and its cached relative GNSS observation. */
	void clearMissionPosition()
	{
		_mission_land_position = {};
		_pos_rel_gnss = {};
	}

	/** Feed the latest EKF2 local NED velocity. Used for init and to compensate GNSS latency. */
	void setLocalVelocity(const matrix::Vector3f &vel_xyz, bool valid, hrt_abstime timestamp);

	/** Feed the latest EKF2 local NED position. Used to publish the absolute target pose. */
	void setLocalPosition(const matrix::Vector3f &xyz, bool valid, hrt_abstime timestamp);

	/** Configure the antenna→CoM position offset in NED. @p gps_is_offset gates its use. */
	void setGpsPosOffset(const matrix::Vector3f &xyz, bool gps_is_offset);

	/** Configure the antenna→CoM velocity offset in NED (e.g. yaw-rate * lever-arm). */
	void setVelOffset(const matrix::Vector3f &xyz);

	/** Timeout after which the whole filter is declared stale (see @c timedOut). */
	void setVteTimeout(hrt_abstime tout) { _vte_timeout_us = tout; }

	/** Maximum age for the published relative pose to be marked valid. */
	void setTargetValidTimeout(hrt_abstime tout)
	{
		_target_valid_timeout_us = tout;
	}

	/** Maximum age for a raw observation to still be considered for fusion. */
	void setMeasRecentTimeout(hrt_abstime tout)
	{
		_meas_recent_timeout_us = tout;
	}

	/** Maximum age for cached inputs (offsets, local pos/vel) to remain valid. */
	void setMeasUpdatedTimeout(hrt_abstime tout)
	{
		_meas_updated_timeout_us = tout;
	}

	/** Set the active aid-source bitmap (@see SensorFusionMaskU). */
	void setVteAidMask(uint16_t mask_value)
	{
		_vte_aid_mask.value = mask_value;
	}

	/** True once the filter has been initialised and has not received updates within the timeout. */
	bool timedOut() const
	{
		return _estimator_initialized && hasTimedOut(_last_update, _vte_timeout_us);
	}

	/** True when the aid mask enables at least one relative-position source (required to init). */
	bool fusionEnabled() const
	{
		return _vte_aid_mask.flags.use_vision_pos
		       || _vte_aid_mask.flags.use_target_gps_pos
		       || _vte_aid_mask.flags.use_mission_pos;
	}

	bool missionPosAidEnabled() const { return _vte_aid_mask.flags.use_mission_pos; }

	void print_status() const;

protected:
	static constexpr double kLatAbsMaxDeg = 90.0;
	static constexpr double kLonAbsMaxDeg = 180.0;
	static constexpr float kAltMinM = -350.f;
	static constexpr float kAltMaxM = 10000.f;

	void updateParams() override;

	uORB::Publication<landing_target_pose_s> _target_pose_pub{ORB_ID(landing_target_pose)};
	uORB::Publication<vte_position_s> _target_estimator_state_pub{ORB_ID(vte_position)};

	uORB::Publication<vte_bias_init_status_s> _vte_bias_init_status_pub{ORB_ID(vte_bias_init_status)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

private:
	friend class ::VisionTargetEstTestable;

	// Observation types used by the estimator. Keep ordering stable for array indexing.
	enum class ObsType : uint8_t {
		kTargetGpsPos,
		kMissionGpsPos,
		kUavGpsVel,
		kTargetGpsVel,
		kFiducialMarker,
		kTypeCount
	};

	enum class PreBiasReference : uint8_t {
		kUnknown,
		kVision,
		kGnss
	};

	static constexpr size_t kObsTypeCount = static_cast<size_t>(ObsType::kTypeCount);
#if defined(CONFIG_VTEST_MOVING)
	static constexpr bool kTargetIsStatic = false;
#else
	static constexpr bool kTargetIsStatic = true;
#endif // CONFIG_VTEST_MOVING
	static constexpr size_t obsIndex(ObsType type)
	{
		return static_cast<size_t>(type);
	}

	// One 3D observation of a single state, the same state index on every axis.
	struct TargetObs {
		ObsType type{ObsType::kTypeCount};
		hrt_abstime timestamp{0}; // 0: nothing observed this cycle
		uint8_t state_index{0};   // vtest::State index observed by meas_xyz
		matrix::Vector3f meas_xyz{};
		matrix::Vector3f meas_unc_xyz{}; // Variances, off-diagonal terms neglected
	};

	union ObsValidMaskU {
		struct {
			uint8_t fuse_target_gps_pos : 1; ///< bit0: target GPS position ready to be fused
			uint8_t fuse_uav_gps_vel    : 1; ///< bit1: UAV GPS velocity ready to be fused
			uint8_t fuse_vision         : 1; ///< bit2: vision relative-position ready to be fused
			uint8_t fuse_mission_pos    : 1; ///< bit3: mission position ready to be fused
			uint8_t fuse_target_gps_vel : 1; ///< bit4: target GPS velocity ready to be fused
			uint8_t reserved            : 3; ///< bits5..7: reserved for future use
		} flags;

		uint8_t value{0};
	};

	static_assert(sizeof(ObsValidMaskU) == 1, "Unexpected masking size");

	using EstimatorState = KF_position::VectorState;
	using EstimatorStateCovariance = KF_position::SquareMatrixState;
	using AxisEstimatorStates = EstimatorState[vtest::Axis::size];

	bool initEstimator(const AxisEstimatorStates &state_init, const matrix::Vector3f &position_variance);
	void performUpdateStep();
	void predictionStep(const matrix::Vector3f &acc, float dt);
	float getMinGpsVelVar() const;
	float getMinGpsPosVar() const;
	float getBiasAveragingThreshold() const;
	hrt_abstime getBiasAveragingTimeoutUs() const;

	inline bool isMeasRecent(hrt_abstime ts) const
	{
		return ts >= _min_sample_time && !hasTimedOut(ts, _meas_recent_timeout_us);
	}

	inline bool isMeasUpdated(hrt_abstime ts) const
	{
		return ts >= _min_sample_time && !hasTimedOut(ts, _meas_updated_timeout_us);
	}

	inline bool hasNewPositionSensorData(const ObsValidMaskU &fusion_mask) const
	{
		return fusion_mask.flags.fuse_mission_pos ||
		       fusion_mask.flags.fuse_target_gps_pos ||
		       fusion_mask.flags.fuse_vision;
	}

	inline bool isTimeDifferenceWithin(const hrt_abstime a, const hrt_abstime b, const hrt_abstime timeout_us) const
	{
		if ((a == 0) || (b == 0)) {
			return false;
		}

		const hrt_abstime diff = (a > b) ? (a - b) : (b - a);
		return diff <= timeout_us;
	}

	// Only estimate the GNSS bias if we have a GNSS estimate and vision.
	inline bool shouldSetBias(const bool has_vision_observation) const
	{
		return _pos_rel_gnss.valid && isMeasRecent(_pos_rel_gnss.timestamp) && has_vision_observation;
	};

	inline bool isBiasAveragingEnabled() const
	{
		return getBiasAveragingThreshold() > 0.f && (getBiasAveragingTimeoutUs() > 0);
	}

	/*
	 * Bias averaging only makes sense when we have both a vision and a valid GNSS observation,
	 * the current pre-bias reference is GNSS, and the averaging window is configured.
	 */
	inline bool shouldAverageInitialBias(const bool has_vision_observation,
					     const PreBiasReference reference) const
	{
		return shouldSetBias(has_vision_observation) && (reference == PreBiasReference::kGnss)
		       && isBiasAveragingEnabled();
	}

	inline bool shouldAverageInitialBias(const bool has_vision_observation) const
	{
		return shouldAverageInitialBias(has_vision_observation, _pre_bias_reference);
	}

	inline bool shouldBlockVisionFusionUntilBiasReady() const
	{
		// Vision must wait until the GNSS bias is ready when the state is still
		// referenced to GNSS.
		return !_bias.set && (_pre_bias_reference == PreBiasReference::kGnss);
	}

	inline bool shouldBlockGnssFusionUntilBiasReady() const
	{
		// GNSS position must wait until the GNSS bias is ready when the state is
		// still referenced to vision.
		return !_bias.set && (_pre_bias_reference == PreBiasReference::kVision);
	}

	/** Build the first filter state once a valid mix of observations is available. */
	bool initializeEstimator(const ObsValidMaskU &fusion_mask,
				 const TargetObs observations[kObsTypeCount]);
	/** Seed the initial GNSS bias directly from the freshest GNSS-relative sample. */
	void selectInitialBiasFromGnss(const TargetObs observations[kObsTypeCount],
				       const matrix::Vector3f &initial_position, matrix::Vector3f &initial_bias);
	/** Build the per-axis state vectors used by the generated Kalman filter. */
	void buildInitialStateArray(const matrix::Vector3f &initial_position,
				    const matrix::Vector3f &initial_uav_velocity, const matrix::Vector3f &initial_bias,
				    AxisEstimatorStates &state_init) const;
	/** Track the GNSS/vision bias until it is ready to be activated in the state.
	 *  Returns true when this cycle's vision sample became the position. */
	bool updateBiasIfObservable(const ObsValidMaskU &fusion_mask,
				    const TargetObs observations[kObsTypeCount]);
	/** Pick the GNSS-relative sample that best matches the requested timestamp. */
	bool selectBiasGnssSample(hrt_abstime sample_time, matrix::Vector3f &gnss_sample);
	/** Fuse every ready observation of this cycle, one batch per axis. */
	void fuseActiveMeasurements(const ObsValidMaskU &fusion_mask, const TargetObs observations[kObsTypeCount]);
	/** Refresh subscriptions and populate the per-cycle observation buffer. */
	void processObservations(ObsValidMaskU &fusion_mask,
				 TargetObs observations[kObsTypeCount]);
	bool getOffsetUavVel(hrt_abstime sample_time, matrix::Vector3f &uav_velocity_ned) const;
	void startBiasAveraging(const matrix::Vector3f &bias_sample, hrt_abstime sample_time);
	bool updateBiasAveraging(const matrix::Vector3f &bias_sample, hrt_abstime sample_time);
	void resetBiasAveraging();
	/** Reset the position and bias from a sample: restart the OOSM history at its time and propagate to now. */
	void activateBiasEstimate(const matrix::Vector3f &position, const matrix::Vector3f &bias,
				  const TargetObs &vision_obs, bool position_from_vision);
	void publishBiasInitStatus(const matrix::Vector3f &raw_bias, const matrix::Vector3f &filtered_bias,
				   float raw_bias_delta_norm, hrt_abstime sample_time);

	/** Validate geographic coordinates before they are cached or fused. */
	bool isLatLonAltValid(double lat_deg, double lon_deg, float alt_m, const char *who = nullptr,
			      hrt_abstime *warn_last = nullptr);

	/* Vision data */
	bool isVisionDataValid(const fiducial_marker_pos_report_s &fiducial_marker_pose);
	bool processObsVision(TargetObs &obs);

	/* UAV GPS data */
	bool updateUavGnssData();
	bool processObsGNSSPosMission(TargetObs &obs);
	bool processObsGNSSVelUav(TargetObs &obs) const;

	/* Target GPS data */
	bool pollTargetGnss(target_gnss_s &target_gnss);
	bool isTargetGpsPositionValid(const target_gnss_s &target_gnss);
	bool isTargetGpsVelocityValid(const target_gnss_s &target_gnss);
	bool processObsGNSSPosTarget(const target_gnss_s &target_gnss, TargetObs &obs);
#if defined(CONFIG_VTEST_MOVING)
	bool processObsGNSSVelTarget(const target_gnss_s &target_gnss, TargetObs &obs) const;
	void updateTargetGpsVelocity(const target_gnss_s &target_gnss);
#endif // CONFIG_VTEST_MOVING

	using AxisResults = FusionResult[vtest::Axis::size][kObsTypeCount]; // [axis][observation]
	/** Publish the per-axis fusion results of one observation. */
	void publishAidSource(const TargetObs &target_obs, const AxisResults &results, unsigned obs_pos);
	void publishTarget();
	/** Report an observation that seeded the state instead of being fused. */
	void publishInitializedAid(const TargetObs &obs);
	void publishInvalidState();
	void resetObservations();

	uORB::Subscription _vehicle_gnss_sub{ORB_ID(vehicle_gnss)};
	uORB::Subscription _fiducial_marker_pos_report_sub{ORB_ID(fiducial_marker_pos_report)};
	uORB::Subscription _target_gnss_sub{ORB_ID(target_gnss)};

	perf_counter_t _vte_predict_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": VTE prediction")};
	perf_counter_t _vte_update_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": VTE update")};
	perf_counter_t _vte_fusion_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": VTE fusion")};

	TargetObs _obs_buffer[kObsTypeCount] {};

	// Fusion results per source, in ObsType order.
	uORB::Publication<vte_aid_source3d_s> _vte_aid_pubs[kObsTypeCount] {
		{ORB_ID(vte_aid_gps_pos_target)}, {ORB_ID(vte_aid_gps_pos_mission)}, {ORB_ID(vte_aid_gps_vel_uav)},
		{ORB_ID(vte_aid_gps_vel_target)}, {ORB_ID(vte_aid_fiducial_marker)}
	};
	landing_target_pose_s _target_pose{};
	vte_position_s _vte_state{};

	struct GlobalPose {
		hrt_abstime timestamp = 0;
		bool valid = false;
		double lat_deg = 0.0; // Latitude in degrees
		double lon_deg = 0.0; // Longitude in degrees
		float alt_m = 0.f;    // Altitude in meters AMSL
		float eph = 0.f;
		float epv = 0.f;
	};

	GlobalPose _mission_land_position{};
	GlobalPose _uav_gps_position{};

	struct VelStamped {
		hrt_abstime timestamp = 0;
		bool valid = false;
		matrix::Vector3f xyz{};
		float uncertainty = 0.f;
	};

	VelStamped _uav_gps_vel{};

	Vector3fStamped _local_position{};
	Vector3fStamped _local_velocity{};
	Vector3fStamped _pos_rel_gnss{};
	bool _pos_rel_gnss_is_target{false};
	Vector3fStamped _velocity_offset_ned{};
	Vector3fStamped _gps_pos_offset_ned{};

#if defined(CONFIG_VTEST_MOVING)
	Vector3fStamped _target_gps_vel {};
#endif // CONFIG_VTEST_MOVING

	static constexpr hrt_abstime kInitialBiasLpfTimeConstant{300_ms};

	struct BiasState {
		bool set{false};
		bool averaging_active{false};
		uint8_t stable_delta_count{0};
		hrt_abstime averaging_start_time{0};
		hrt_abstime last_sample_time{0};
		AlphaFilter<matrix::Vector3f> initial_lpf{kInitialBiasLpfTimeConstant};
	};

	bool _gps_pos_is_offset{false};
	BiasState _bias{};
	PreBiasReference _pre_bias_reference{PreBiasReference::kUnknown};

	uint64_t _last_relative_meas_fused_time{0};
	bool _estimator_initialized{false};

	KF_position _target_est_pos[vtest::Axis::size] {};

	hrt_abstime _min_sample_time{0}; // Discard measurements from before an attitude reset.
	hrt_abstime _last_predict{0}; // timestamp of last filter prediction
	hrt_abstime _last_update{0}; // capture time of the newest fused position sample (used to check timeout)
	hrt_abstime _uav_gps_vel_warn_last{0};
	hrt_abstime _target_gps_vel_warn_last{0};
	hrt_abstime _vision_pos_warn_last{0};
	hrt_abstime _uav_gps_pos_warn_last{0};
	hrt_abstime _target_gps_pos_warn_last{0};
	hrt_abstime _mission_pos_warn_last{0};
	hrt_abstime _mission_pos_status_warn_last{0};
	hrt_abstime _init_vel_warn_last{0};

	/* parameters from vision_target_estimator_params.yaml */
	void checkMeasurementInputs();

	hrt_abstime _vte_timeout_us{3_s};
	hrt_abstime _target_valid_timeout_us{2_s};
	hrt_abstime _meas_recent_timeout_us{1_s};
	hrt_abstime _meas_updated_timeout_us{100_ms};
	SensorFusionMaskU _vte_aid_mask{};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::VTE_ACC_D_UNC>) _param_vte_acc_d_unc,
		(ParamFloat<px4::params::VTE_ACC_T_UNC>) _param_vte_acc_t_unc,
		(ParamFloat<px4::params::VTE_BIAS_UNC>) _param_vte_bias_unc,
		(ParamFloat<px4::params::VTE_BIA_AVG_THR>) _param_vte_bias_avg_thr,
		(ParamFloat<px4::params::VTE_BIA_AVG_TOUT>) _param_vte_bias_avg_tout,
		(ParamFloat<px4::params::VTE_POS_UNC_IN>) _param_vte_pos_unc_in,
		(ParamFloat<px4::params::VTE_VEL_UNC_IN>) _param_vte_vel_unc_in,
		(ParamFloat<px4::params::VTE_BIA_UNC_IN>) _param_vte_bias_unc_in,
		(ParamFloat<px4::params::VTE_ACC_UNC_IN>) _param_vte_acc_unc_in,
		(ParamFloat<px4::params::VTE_GPS_V_NOISE>) _param_vte_gps_vel_noise,
		(ParamFloat<px4::params::VTE_GPS_P_NOISE>) _param_vte_gps_pos_noise,
		(ParamFloat<px4::params::VTE_EVP_NOISE>) _param_vte_ev_pos_noise,
		(ParamFloat<px4::params::VTE_POS_NIS_THRE>) _param_vte_pos_nis_thre
	)
};
} // namespace vision_target_estimator
