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
#include <uORB/topics/vehicle_status.h>
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
#include <uORB/topics/sensor_uwb.h>
#include <uORB/topics/irlock_report.h>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <matrix/Matrix.hpp>
#include <lib/conversion/rotation.h>
#include <lib/geo/geo.h>
#include "KF_position.h"
#include <vtest_derivation/generated/state.h>
#include "../common.h"

using namespace time_literals;

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
	void update(const matrix::Vector3f &acc_ned, const matrix::Quaternionf &q_att);

	bool init();

	void resetFilter();

	void set_mission_position(const double lat_deg, const double lon_deg, const float alt_m);
	void set_range_sensor(const float dist, const bool valid, const hrt_abstime timestamp);
	void set_local_velocity(const matrix::Vector3f &vel_xyz, const bool valid, const hrt_abstime timestamp);
	void set_local_position(const matrix::Vector3f &xyz, const bool valid, const hrt_abstime timestamp);
	void set_gps_pos_offset(const matrix::Vector3f &xyz, const bool gps_is_offset);
	void set_vel_offset(const matrix::Vector3f &xyz);
	void set_vte_timeout(const float tout) {_vte_TIMEOUT_US = static_cast<uint32_t>(tout * 1_s);};
	void set_vte_aid_mask(const uint16_t mask_value) {_vte_aid_mask.value = mask_value;};

	bool timedOut() {return hasTimedOut(_last_update, _vte_TIMEOUT_US);};
	// TODO: Could be more strict and require a relative position meas (vision, GPS, irlock, uwb)
	bool fusionEnabled() {return _vte_aid_mask.value != 0;};

protected:

	/*
	 * update parameters.
	 */
	void updateParams() override;

	// Geographic limits
	static constexpr double kLatAbsMaxDeg =  90.0;
	static constexpr double kLonAbsMaxDeg = 180.0;
	static constexpr float kAltMinM = -350.f;
	static constexpr float kAltMaxM = 10000.f;

	uORB::Publication<landing_target_pose_s> _targetPosePub{ORB_ID(landing_target_pose)};
	uORB::Publication<vision_target_est_position_s> _targetEstimatorStatePub{ORB_ID(vision_target_est_position)};

	// publish innovations target_estimator_gps_pos
	uORB::Publication<estimator_aid_source3d_s> _vte_aid_gps_pos_target_pub{ORB_ID(vte_aid_gps_pos_target)};
	uORB::Publication<estimator_aid_source3d_s> _vte_aid_gps_pos_mission_pub{ORB_ID(vte_aid_gps_pos_mission)};
	uORB::Publication<estimator_aid_source3d_s> _vte_aid_gps_vel_target_pub{ORB_ID(vte_aid_gps_vel_target)};
	uORB::Publication<estimator_aid_source3d_s> _vte_aid_gps_vel_uav_pub{ORB_ID(vte_aid_gps_vel_uav)};
	uORB::Publication<estimator_aid_source3d_s> _vte_aid_fiducial_marker_pub{ORB_ID(vte_aid_fiducial_marker)};
	uORB::Publication<estimator_aid_source3d_s> _vte_aid_uwb_pub{ORB_ID(vte_aid_uwb)};
	uORB::Publication<estimator_aid_source3d_s> _vte_aid_irlock_pub{ORB_ID(vte_aid_irlock)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

private:
	// TODO: enum class
	enum ObsType {
		Target_gps_pos,
		Mission_gps_pos,
		Uav_gps_vel,
		Target_gps_vel,
		Fiducial_marker,
		Uwb,
		Irlock,
		Type_count
	};

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
			uint8_t fuse_uwb            : 1; ///< bit5: UWB data ready to be fused
			uint8_t fuse_irlock         : 1; ///< bit6: IRLOCK data ready to be fused
			uint8_t reserved            : 1; ///< bit7: reserved for future use
		} flags;

		uint8_t value{0};
	};

	static_assert(sizeof(ObsValidMaskU) == 1, "Unexpected masking size");

	bool initEstimator(const matrix::Matrix <float, vtest::Axis::size, vtest::State::size>
			   &state_init);
	bool updateStep(const matrix::Vector3f &vehicle_acc_ned, const matrix::Quaternionf &q_att);
	void predictionStep(const matrix::Vector3f &acc);

	void updateTargetGpsVelocity(const target_gnss_s &target_GNSS_report);

	inline bool hasNewNonGpsPositionSensorData(const ObsValidMaskU &vte_fusion_aid_mask) const
	{
		return vte_fusion_aid_mask.flags.fuse_vision
		       || vte_fusion_aid_mask.flags.fuse_uwb
		       || vte_fusion_aid_mask.flags.fuse_irlock;
	}

	inline bool hasNewPositionSensorData(const ObsValidMaskU &vte_fusion_aid_mask) const
	{
		return vte_fusion_aid_mask.flags.fuse_mission_pos
		       || vte_fusion_aid_mask.flags.fuse_target_gps_pos
		       || vte_fusion_aid_mask.flags.fuse_vision
		       || vte_fusion_aid_mask.flags.fuse_uwb
		       || vte_fusion_aid_mask.flags.fuse_irlock;
	}

	// Only estimate the GNSS bias if we have a GNSS estimation and a secondary source of position
	inline bool shouldSetBias(const ObsValidMaskU &vte_fusion_aid_mask)
	{
		return isMeasValid(_pos_rel_gnss.timestamp) && hasNewNonGpsPositionSensorData(vte_fusion_aid_mask);
	};

	bool initializeEstimator(const ObsValidMaskU &vte_fusion_aid_mask,
				 const TargetObs observations[ObsType::Type_count]);
	void updateBias(const ObsValidMaskU &vte_fusion_aid_mask,
			const TargetObs observations[ObsType::Type_count]);
	void get_pos_init(const ObsValidMaskU &vte_fusion_aid_mask,
			  const TargetObs observations[ObsType::Type_count], matrix::Vector3f &pos_init);
	bool fuseNewSensorData(const matrix::Vector3f &vehicle_acc_ned, ObsValidMaskU &vte_fusion_aid_mask,
			       const TargetObs observations[ObsType::Type_count]);
	void processObservations(const matrix::Quaternionf &q_att, ObsValidMaskU &vte_fusion_aid_mask,
				 TargetObs observations[ObsType::Type_count]);

	bool isLatLonAltValid(double lat_deg, double lon_deg, float alt_m, const char *who = nullptr) const;

	/* Vision data */
	void handleVisionData(ObsValidMaskU &vte_fusion_aid_mask, TargetObs &obs_fiducial_marker);
	bool isVisionDataValid(const fiducial_marker_pos_report_s &fiducial_marker_pose);
	bool processObsVision(const fiducial_marker_pos_report_s &fiducial_marker_pose, TargetObs &obs);

	/* IRLOCK data */
	void handleIrlockData(const matrix::Quaternionf &q_att, ObsValidMaskU &vte_fusion_aid_mask, TargetObs &obs_irlock);
	bool isIrlockDataValid(const irlock_report_s &irlock_report) const;
	bool processObsIrlock(const matrix::Quaternionf &q_att, const irlock_report_s &irlock_report, TargetObs &obs);

	/* UWB data */
	void handleUwbData(const matrix::Quaternionf &q_att, ObsValidMaskU &vte_fusion_aid_mask, TargetObs &obs_uwb);
	bool isUwbDataValid(const sensor_uwb_s &uwb_report);
	bool processObsUwb(const matrix::Quaternionf &q_att, const sensor_uwb_s &uwb_report, TargetObs &obs);

	/* UAV GPS data */
	void handleUavGpsData(ObsValidMaskU &vte_fusion_aid_mask,
			      TargetObs &obs_gps_pos_mission,
			      TargetObs &obs_gps_vel_uav);
	bool updateUavGpsData();
	bool isUavGpsPositionValid();
	bool isUavGpsVelocityValid();
	bool processObsGNSSPosMission(TargetObs &obs);
	bool processObsGNSSVelUav(TargetObs &obs);

	/* Target GPS data */
	void handleTargetGpsData(ObsValidMaskU &vte_fusion_aid_mask,
				 TargetObs &obs_gps_pos_target,
				 TargetObs &obs_gps_vel_target);
	bool isTargetGpsPositionValid(const target_gnss_s &target_GNSS_report);
	bool isTargetGpsVelocityValid(const target_gnss_s &target_GNSS_report);
	bool processObsGNSSPosTarget(const target_gnss_s &target_GNSS_report, TargetObs &obs);
#if defined(CONFIG_VTEST_MOVING)
	bool ProcessObsGNSSVelTarget(const target_gnss_s &target_GNSS_report, TargetObs &obs);
#endif // CONFIG_VTEST_MOVING

	bool fuseMeas(const matrix::Vector3f &vehicle_acc_ned, const TargetObs &target_pos_obs);
	void publishTarget();
	void publishInnov(const estimator_aid_source3d_s &target_innov, const ObsType type);

	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _fiducial_marker_report_sub{ORB_ID(fiducial_marker_pos_report)};
	uORB::Subscription _irlock_report_sub{ORB_ID(irlock_report)};
	uORB::Subscription _target_gnss_sub{ORB_ID(target_gnss)};
	uORB::Subscription _sensor_uwb_sub{ORB_ID(sensor_uwb)};

	perf_counter_t _vte_predict_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": VTE prediction")};
	perf_counter_t _vte_update_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": VTE update")};


	RangeSensor _range_sensor{};

	struct IrlockConfig {
		float scale_x{1.0f};
		float scale_y{1.0f};
		float offset_x{0.0f};
		float offset_y{0.0f};
		float offset_z{0.0f};
		enum Rotation sensor_yaw {ROTATION_YAW_90};
		float meas_unc{0.005f};
		float noise{0.1f};
	};

	IrlockConfig _irlock_config{};

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

	struct VecStamped {
		hrt_abstime timestamp = 0;
		bool valid = false;
		matrix::Vector3f xyz{};
	};

	VecStamped _local_position{};
	VecStamped _local_velocity{};
	VecStamped _target_gps_vel{};
	VecStamped _pos_rel_gnss{};
	VecStamped _velocity_offset_ned{};
	VecStamped _gps_pos_offset_ned{};
	bool _gps_pos_is_offset{false};
	bool _bias_set{false};

	uint64_t _last_relative_meas_fused_time{0};
	bool _estimator_initialized{false};

	KF_position _target_est_pos[vtest::Axis::size];

	hrt_abstime _last_predict{0}; // timestamp of last filter prediction
	hrt_abstime _last_update{0}; // timestamp of last filter update (used to check timeout)

	/* parameters from vision_target_estimator_params.c*/
	void checkMeasurementInputs();

	uint32_t _vte_TIMEOUT_US = 3_s;
	SensorFusionMaskU _vte_aid_mask{};
	float _target_acc_unc{1.f};
	float _bias_unc{0.05f};
	float _uav_acc_unc{1.f};
	float _gps_vel_noise{0.3f};
	float _gps_pos_noise{0.5f};
	bool  _ev_noise_md{false};
	float _ev_pos_noise{0.1f};
	float _nis_threshold{3.84f};
	float _uwb_noise{0.1f};

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
		(ParamFloat<px4::params::VTE_POS_NIS_THRE>) _param_vte_pos_nis_thre,
		(ParamFloat<px4::params::VTE_UWB_NOISE>) _param_vte_uwb_noise,
		(ParamFloat<px4::params::VTE_IRL_SCALE_X>) _param_vte_irl_scale_x,
		(ParamFloat<px4::params::VTE_IRL_SCALE_Y>) _param_vte_irl_scale_y,
		(ParamInt<px4::params::VTE_IRL_SENS_ROT>) _param_vte_irl_sens_rot,
		(ParamFloat<px4::params::VTE_IRL_POS_X>) _param_vte_irl_pos_x,
		(ParamFloat<px4::params::VTE_IRL_POS_Y>) _param_vte_irl_pos_y,
		(ParamFloat<px4::params::VTE_IRL_POS_Z>) _param_vte_irl_pos_z,
		(ParamFloat<px4::params::VTE_IRL_MEAS_UNC>) _param_vte_irl_meas_unc,
		(ParamFloat<px4::params::VTE_IRL_NOISE>) _param_vte_irl_noise
	)
};
} // namespace vision_target_estimator
