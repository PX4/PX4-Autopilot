/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @brief Estimate the state of a target by processing and fusing sensor data in a Kalman Filter.
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#pragma once

#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/workqueue.h>
#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/fiducial_marker_pos_report.h>
#include <uORB/topics/target_gnss.h>
#include <uORB/topics/vision_target_est_position.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/estimator_aid_source3d.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/sensor_gps.h>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <matrix/Matrix.hpp>
#include <lib/geo/geo.h>
#include <containers/Array.hpp>
#include "KF_position.h"
#include <vtest_derivation/generated/state.h>
#include "../common.h"


namespace vision_target_estimator
{

class VTEPosition: public ModuleParams
{
public:

	VTEPosition();
	virtual ~VTEPosition();

	/*
	 * Get new measurements and update the state estimate
	 */
	void update(const matrix::Vector3f &acc_ned);

	bool init();

	void resetFilter();

	void set_mission_position(const double lat_deg, const double lon_deg, const float alt_m);
	void set_range_sensor(const float dist, const bool valid, const hrt_abstime timestamp);
	void set_local_velocity(const matrix::Vector3f &vel_xyz, const bool valid, const hrt_abstime timestamp);
	void set_local_position(const matrix::Vector3f &xyz, const bool valid, const hrt_abstime timestamp);
	void set_gps_pos_offset(const matrix::Vector3f &xyz, const bool gps_is_offset);
	void set_vel_offset(const matrix::Vector3f &xyz);
	void set_vte_timeout(const uint32_t tout) {_vte_timeout_us = tout;};
	void set_target_valid_timeout(const uint32_t tout) {_target_valid_timeout_us = tout;};
	void set_meas_recent_timeout(const uint32_t tout) {_meas_recent_timeout_us = tout;};
	void set_meas_updated_timeout(const uint32_t tout) {_meas_updated_timeout_us = tout;};
	void set_vte_aid_mask(const uint16_t mask_value) {_vte_aid_mask.value = mask_value;};

	bool timedOut() const {return _estimator_initialized && hasTimedOut(_last_update, _vte_timeout_us);};
	// TODO: Could be more strict and require a relative position meas (vision, GPS)
	bool fusionEnabled() const {return _vte_aid_mask.value != 0;};

protected:

	/*
	 * update parameters.
	 */
	void updateParams() override;

	uORB::Publication<landing_target_pose_s> _targetPosePub{ORB_ID(landing_target_pose)};
	uORB::Publication<vision_target_est_position_s> _targetEstimatorStatePub{ORB_ID(vision_target_est_position)};

	// publish innovations target_estimator_gps_pos
	uORB::Publication<estimator_aid_source3d_s> _vte_aid_gps_pos_target_pub{ORB_ID(vte_aid_gps_pos_target)};
	uORB::Publication<estimator_aid_source3d_s> _vte_aid_gps_pos_mission_pub{ORB_ID(vte_aid_gps_pos_mission)};
	uORB::Publication<estimator_aid_source3d_s> _vte_aid_gps_vel_target_pub{ORB_ID(vte_aid_gps_vel_target)};
	uORB::Publication<estimator_aid_source3d_s> _vte_aid_gps_vel_uav_pub{ORB_ID(vte_aid_gps_vel_uav)};
	uORB::Publication<estimator_aid_source3d_s> _vte_aid_fiducial_marker_pub{ORB_ID(vte_aid_fiducial_marker)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

private:
	// Observation types used by the estimator. Keep ordering stable for array indexing.
	enum class ObsType : uint8_t {
		Target_gps_pos,
		Mission_gps_pos,
		Uav_gps_vel,
		Target_gps_vel,
		Fiducial_marker,
		Type_count
	};

	static constexpr size_t kObsTypeCount = static_cast<size_t>(ObsType::Type_count);
	static constexpr size_t obsIndex(ObsType type)
	{
		return static_cast<size_t>(type);
	}

	struct TargetObs {

		ObsType type;
		hrt_abstime timestamp = 0;

		matrix::Vector<bool, vtest::Axis::size> updated{}; // Indicates if observations were updated.
		matrix::Vector3f meas_xyz{};			// Measurements (meas_x, meas_y, meas_z)
		matrix::Vector3f meas_unc_xyz{};		// Measurements' uncertainties
		matrix::Matrix<float, vtest::Axis::size, vtest::State::size>
		meas_h_xyz{}; // Observation matrix where the rows correspond to the x,y,z observations and the columns to the state
	};

	union ObsValidMaskU {
		struct {
			uint8_t fuse_target_gps_pos : 1; ///< bit0: target GPS position ready to be fused
			uint8_t fuse_uav_gps_vel    : 1; ///< bit1: UAV GPS velocity ready to be fused
			uint8_t fuse_vision         : 1; ///< bit2: external vision-relative position ready to be fused
			uint8_t fuse_mission_pos    : 1; ///< bit3: mission position ready to be fused
			uint8_t fuse_target_gps_vel : 1; ///< bit4: target GPS velocity ready to be fused
			uint8_t reserved            : 3; ///< bits5..7: reserved for future use
		} flags;

		uint8_t value{0};
	};

	static_assert(sizeof(ObsValidMaskU) == 1, "Unexpected masking size");

	bool initEstimator(const matrix::Matrix <float, vtest::Axis::size, vtest::State::size>
			   &state_init);
	bool performUpdateStep(const matrix::Vector3f &vehicle_acc_ned);
	void predictionStep(const matrix::Vector3f &acc);

	inline bool isMeasRecent(hrt_abstime ts) const
	{
		return !hasTimedOut(ts, _meas_recent_timeout_us);
	}

	inline bool isMeasUpdated(hrt_abstime ts) const
	{
		return !hasTimedOut(ts, _meas_updated_timeout_us);
	}

	inline bool hasNewNonGpsPositionSensorData(const ObsValidMaskU &fusion_mask) const
	{
		return fusion_mask.flags.fuse_vision;
	}

	inline bool hasNewPositionSensorData(const ObsValidMaskU &fusion_mask) const
	{
		return fusion_mask.flags.fuse_mission_pos
		       || fusion_mask.flags.fuse_target_gps_pos
		       || fusion_mask.flags.fuse_vision;
	}

	inline bool isTimeDifferenceWithin(const hrt_abstime a, const hrt_abstime b, const uint32_t timeout_us) const
	{
		if ((a == 0) || (b == 0)) {
			return false;
		}

		const hrt_abstime diff = (a > b) ? (a - b) : (b - a);
		return diff <= timeout_us;
	}

	// Only estimate the GNSS bias if we have a GNSS estimation and a secondary source of position
	inline bool shouldSetBias(const ObsValidMaskU &fusion_mask) const
	{
		const bool gnss_bias_ready = _pos_rel_gnss.valid
					     && isMeasRecent(_pos_rel_gnss.timestamp);

		return gnss_bias_ready && hasNewNonGpsPositionSensorData(fusion_mask);
	};

	bool initializeEstimator(const ObsValidMaskU &fusion_mask,
				 const TargetObs observations[kObsTypeCount]);
	void updateBiasIfObservable(const ObsValidMaskU &fusion_mask,
				    const TargetObs observations[kObsTypeCount]);
	void selectInitialPosition(const ObsValidMaskU &fusion_mask,
				   const TargetObs observations[kObsTypeCount], matrix::Vector3f &initial_position);
	bool fuseActiveMeasurements(const matrix::Vector3f &vehicle_acc_ned, ObsValidMaskU &fusion_mask,
				    const TargetObs observations[kObsTypeCount]);
	void processObservations(ObsValidMaskU &fusion_mask,
				 TargetObs observations[kObsTypeCount]);

	bool isLatLonAltValid(double lat_deg, double lon_deg, float alt_m, const char *who = nullptr,
			      hrt_abstime *warn_last = nullptr);

	/* Vision data */
	void handleVisionData(ObsValidMaskU &fusion_mask, TargetObs &vision_obs);
	bool isVisionDataValid(const fiducial_marker_pos_report_s &fiducial_marker_pose) const;
	bool processObsVision(const fiducial_marker_pos_report_s &fiducial_marker_pose, TargetObs &obs);

	/* UAV GPS data */
	void handleUavGpsData(ObsValidMaskU &fusion_mask,
			      TargetObs &mission_pos_obs,
			      TargetObs &uav_vel_obs);
	bool updateUavGpsData();
	bool isUavGpsPositionValid();
	bool isUavGpsVelocityValid();
	bool processObsGNSSPosMission(TargetObs &obs);
	bool processObsGNSSVelUav(TargetObs &obs) const;

	/* Target GPS data */
	void handleTargetGpsData(ObsValidMaskU &fusion_mask,
				 TargetObs &target_pos_obs,
				 TargetObs &target_vel_obs);
	bool isTargetGpsPositionValid(const target_gnss_s &target_gnss);
	bool isTargetGpsVelocityValid(const target_gnss_s &target_gnss);
	bool processObsGNSSPosTarget(const target_gnss_s &target_gnss, TargetObs &obs);
#if defined(CONFIG_VTEST_MOVING)
	bool ProcessObsGNSSVelTarget(const target_gnss_s &target_gnss, TargetObs &obs) const;
	void updateTargetGpsVelocity(const target_gnss_s &target_gnss);
#endif // CONFIG_VTEST_MOVING

	bool fuseMeas(const matrix::Vector3f &vehicle_acc_ned, const TargetObs &target_pos_obs);
	void publishTarget();
	void publishInnov(const estimator_aid_source3d_s &target_innov, const ObsType type);
	void resetObservations();
	bool shouldEmitWarning(hrt_abstime &last_warn);

	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _fiducial_marker_pos_report_sub{ORB_ID(fiducial_marker_pos_report)};
	uORB::Subscription _target_gnss_sub{ORB_ID(target_gnss)};

	perf_counter_t _vte_predict_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": VTE prediction")};
	perf_counter_t _vte_update_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": VTE update")};

	TargetObs _obs_buffer[kObsTypeCount] {};
	estimator_aid_source3d_s _target_innov{};
	landing_target_pose_s _target_pose{};
	vision_target_est_position_s _vte_state{};

	FloatStamped _range_sensor{};

	struct GlobalPose {
		hrt_abstime timestamp = 0;
		bool valid = false;
		double lat_deg = 0.0; 	// Latitude in degrees
		double lon_deg	= 0.0; 	// Longitude in degrees
		float alt_m = 0.f;	// Altitude in meters AMSL
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
	Vector3fStamped _velocity_offset_ned{};
	Vector3fStamped _gps_pos_offset_ned{};

#if defined(CONFIG_VTEST_MOVING)
	Vector3fStamped _target_gps_vel {};
	float _mpc_z_v_auto_dn{0.f};
	param_t _param_mpc_z_v_auto_dn{PARAM_INVALID};
#endif // CONFIG_VTEST_MOVING

	bool _gps_pos_is_offset{false};
	bool _bias_set{false};

	uint64_t _last_relative_meas_fused_time{0};
	bool _estimator_initialized{false};

	px4::Array<KF_position, vtest::Axis::size> _target_est_pos{};

	hrt_abstime _last_predict{0}; // timestamp of last filter prediction
	hrt_abstime _last_update{0}; // timestamp of last filter update (used to check timeout)
	hrt_abstime _uav_gps_vel_warn_last{0};
	hrt_abstime _target_gps_vel_warn_last{0};
	hrt_abstime _vision_pos_warn_last{0};
	hrt_abstime _uav_gps_pos_warn_last{0};
	hrt_abstime _target_gps_pos_warn_last{0};
	hrt_abstime _mission_pos_warn_last{0};
	hrt_abstime _mission_pos_status_warn_last{0};
	hrt_abstime _init_vel_warn_last{0};

	/* parameters from vision_target_estimator_params.c*/
	void checkMeasurementInputs();

	uint32_t _vte_timeout_us{3_s};
	uint32_t _target_valid_timeout_us{2_s};
	uint32_t _meas_recent_timeout_us{1_s};
	uint32_t _meas_updated_timeout_us{100_ms};
	SensorFusionMaskU _vte_aid_mask{};
	float _target_acc_unc{1.f};
	float _bias_unc{0.05f};
	float _uav_acc_unc{1.f};
	float _min_gps_vel_var{0.09f};
	float _min_gps_pos_var{0.25f};
	bool  _ev_noise_md{false};
	float _min_ev_pos_var{0.01f};
	float _nis_threshold{3.84f};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::VTE_ACC_D_UNC>) _param_vte_acc_d_unc,
		(ParamFloat<px4::params::VTE_ACC_T_UNC>) _param_vte_acc_t_unc,
		(ParamFloat<px4::params::VTE_BIAS_LIM>) _param_vte_bias_lim,
		(ParamFloat<px4::params::VTE_BIAS_UNC>) _param_vte_bias_unc,
		(ParamFloat<px4::params::VTE_POS_UNC_IN>) _param_vte_pos_unc_in,
		(ParamFloat<px4::params::VTE_VEL_UNC_IN>) _param_vte_vel_unc_in,
		(ParamFloat<px4::params::VTE_BIA_UNC_IN>) _param_vte_bias_unc_in,
		(ParamFloat<px4::params::VTE_ACC_UNC_IN>) _param_vte_acc_unc_in,
		(ParamFloat<px4::params::VTE_GPS_V_NOISE>) _param_vte_gps_vel_noise,
		(ParamFloat<px4::params::VTE_GPS_P_NOISE>) _param_vte_gps_pos_noise,
		(ParamInt<px4::params::VTE_EV_NOISE_MD>) _param_vte_ev_noise_md,
		(ParamFloat<px4::params::VTE_EVP_NOISE>) _param_vte_ev_pos_noise,
		(ParamInt<px4::params::VTE_EKF_AID>) _param_vte_ekf_aid,
		(ParamFloat<px4::params::VTE_MOVING_T_MAX>) _param_vte_moving_t_max,
		(ParamFloat<px4::params::VTE_MOVING_T_MIN>) _param_vte_moving_t_min,
		(ParamFloat<px4::params::VTE_POS_NIS_THRE>) _param_vte_pos_nis_thre
	)
};
} // namespace vision_target_estimator
