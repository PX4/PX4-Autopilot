/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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
 * @brief Estimate the state of a target by processessing and fusing sensor data in a Kalman Filter.
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
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/estimator_aid_source3d.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/sensor_gps.h>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <matrix/Matrix.hpp>
#include <lib/conversion/rotation.h>
#include <lib/geo/geo.h>
#include "KF_xyzb_decoupled_static.h"
#include "KF_xyzb_v_decoupled_moving.h"

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
	void update(const matrix::Vector3f &acc_ned);

	bool init();

	void resetFilter();

	void set_landpoint(const double lat_deg, const double lon_deg, const float alt_m);

	void set_range_sensor(const float dist, const bool valid, const hrt_abstime timestamp);

	void set_local_velocity(const matrix::Vector3f &vel_xyz, const bool valid, const hrt_abstime timestamp);

	void set_local_position(const matrix::Vector3f &xyz, const bool valid, const hrt_abstime timestamp);

	void set_gps_pos_offset(const matrix::Vector3f &xyz, const bool gps_is_offset);

	void set_velocity_offset(const matrix::Vector3f &xyz);

	bool has_timed_out() {return _has_timed_out;};

private:
	struct accInput {

		bool acc_ned_valid = false;
		matrix::Vector3f vehicle_acc_ned;
	};

protected:

	/*
	 * Update parameters.
	 */
	void updateParams() override;


	/* timeout after which the target is not valid if no measurements are seen*/
	static constexpr uint32_t target_valid_TIMEOUT_US = 2_s;

	/* timeout after which the measurement is not valid*/
	static constexpr uint32_t measurement_valid_TIMEOUT_US = 1_s;

	/* timeout after which the measurement is not considered updated*/
	static constexpr uint32_t measurement_updated_TIMEOUT_US = 100_ms;

	uORB::Publication<landing_target_pose_s> _targetPosePub{ORB_ID(landing_target_pose)};
	uORB::Publication<vision_target_est_position_s> _targetEstimatorStatePub{ORB_ID(vision_target_est_position)};

	uORB::Publication<vehicle_odometry_s>	_visual_odometry_pub{ORB_ID(vehicle_visual_odometry)};

	// publish innovations target_estimator_gps_pos
	uORB::Publication<estimator_aid_source3d_s> _vte_aid_gps_pos_target_pub{ORB_ID(vte_aid_gps_pos_target)};
	uORB::Publication<estimator_aid_source3d_s> _vte_aid_gps_pos_mission_pub{ORB_ID(vte_aid_gps_pos_mission)};
	uORB::Publication<estimator_aid_source3d_s> _vte_aid_gps_vel_target_pub{ORB_ID(vte_aid_gps_vel_target)};
	uORB::Publication<estimator_aid_source3d_s> _vte_aid_gps_vel_rel_pub{ORB_ID(vte_aid_gps_vel_rel)};
	uORB::Publication<estimator_aid_source3d_s> _vte_aid_fiducial_marker_pub{ORB_ID(vte_aid_fiducial_marker)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

private:

	static inline bool _is_meas_valid(hrt_abstime time_stamp) {return (hrt_absolute_time() - time_stamp) < measurement_valid_TIMEOUT_US;};
	static inline bool _is_meas_updated(hrt_abstime time_stamp) {return (hrt_absolute_time() - time_stamp) < measurement_updated_TIMEOUT_US;};

	bool _has_timed_out{false};

	enum class TargetMode {
		Stationary = 0,
		Moving = 1,
		NotInit
	};

	TargetMode _target_mode{TargetMode::NotInit};

	enum ObservationType {
		target_gps_pos = 0,
		mission_gps_pos = 1,
		vel_rel_gps = 2,
		vel_target_gps = 3,
		fiducial_marker = 4,
	};

	struct targetObsPos {

		ObservationType type;
		hrt_abstime timestamp;

		matrix::Vector<bool, 3> updated_xyz; // Indicates if we have an observation in the x, y or z direction
		matrix::Vector3f meas_xyz;			// Measurements (meas_x, meas_y, meas_z)
		matrix::Vector3f meas_unc_xyz;		// Measurements' uncertainties
		matrix::Matrix<float, 3, 15>
		meas_h_xyz; // Observation matrix where the rows correspond to the x,y,z observations and the columns to the state = [r_xyz, v_drone_xyz, b_xyz, v_target_xyz, a_target_xyz]
	};

	enum Directions {
		x = 0,
		y = 1,
		z = 2,
		nb_directions = 3,
	};

	enum SensorFusionMask : uint16_t {
		// Bit locations for fusion_mode
		USE_TARGET_GPS_POS  = (1 << 0),    ///< set to true to use target GPS position data
		USE_GPS_REL_VEL     = (1 << 1),    ///< set to true to use drone GPS velocity data (and target GPS velocity data if the target is moving)
		USE_EXT_VIS_POS 	= (1 << 2),    ///< set to true to use target external vision-based relative position data
		USE_MISSION_POS     = (1 << 3),    ///< set to true to use the PX4 mission landing position
	};

	enum ObservationValidMask : uint16_t {
		// Bit locations for valid observations
		FUSE_TARGET_GPS_POS  = (1 << 0),    ///< set to true if target GPS position data is ready to be fused
		FUSE_GPS_REL_VEL     = (1 << 1),    ///< set to true if drone GPS velocity data (and target GPS velocity data if the target is moving)
		FUSE_EXT_VIS_POS 	  = (1 << 2),    ///< set to true if target external vision-based relative position data is ready to be fused
		FUSE_MISSION_POS     = (1 << 3),    ///< set to true if the PX4 mission landing position is ready to be fused
		FUSE_TARGET_GPS_VEL     = (1 << 4),   ///< set to true if target GPS velocity data is ready to be fused
	};

	bool selectTargetEstimator();
	bool initEstimator(const matrix::Vector3f &pos_init, const matrix::Vector3f &vel_rel_init,
			   const matrix::Vector3f &acc_init,
			   const matrix::Vector3f &bias_init, const matrix::Vector3f &target_vel_init);
	bool update_step(const matrix::Vector3f &vehicle_acc_ned);
	void predictionStep(const matrix::Vector3f &acc);

	bool processObsVision(const fiducial_marker_pos_report_s &fiducial_marker_pose, targetObsPos &obs);
	bool processObsGNSSPosTarget(const target_gnss_s &target_GNSS_report,
				     const sensor_gps_s &vehicle_gps_position, targetObsPos &obs);
	bool processObsGNSSPosMission(const sensor_gps_s &vehicle_gps_position, targetObsPos &obs);
	bool processObsGNSSVelRel(const sensor_gps_s &vehicle_gps_position, targetObsPos &obs);
	bool processObsGNSSVelTarget(const target_gnss_s &target_GNSS_report, targetObsPos &obs);

	bool fuse_meas(const matrix::Vector3f &vehicle_acc_ned, const targetObsPos &target_pos_obs);
	void publishTarget();

	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _fiducial_marker_report_sub{ORB_ID(fiducial_marker_pos_report)};
	uORB::Subscription _target_gnss_sub{ORB_ID(target_gnss)};

	perf_counter_t _vte_predict_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": VTE prediction")};
	perf_counter_t _vte_update_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": VTE update")};

	struct rangeSensor {
		bool valid = false;
		float dist_bottom;
		hrt_abstime timestamp = 0;
	};

	rangeSensor _range_sensor{};

	struct globalPos {
		bool valid = false;
		double lat_deg = 0.0; 	// Latitude in degrees
		double lon_deg	= 0.0; 	// Longitude in degrees
		float alt_m = 0.f;	// Altitude in meters AMSL
	};

	globalPos _landing_pos{};

	struct vecStamped {
		hrt_abstime timestamp;
		bool valid = false;
		matrix::Vector3f xyz;
	};

	vecStamped _local_position{};
	vecStamped _local_velocity{};
	vecStamped _uav_gps_vel{};
	vecStamped _target_gps_vel{};
	vecStamped _pos_rel_gnss{};
	vecStamped _velocity_offset_ned{};
	vecStamped _gps_pos_offset_ned{};
	bool _gps_pos_is_offset{false};
	bool _bias_set{false};

	bool _estimator_initialized{false};

	matrix::Quaternionf _q_att; //Quaternion orientation of the body frame
	Base_KF_decoupled *_target_estimator[nb_directions] {nullptr, nullptr, nullptr};
	hrt_abstime _last_predict{0}; // timestamp of last filter prediction
	hrt_abstime _last_update{0}; // timestamp of last filter update (used to check timeout)

	void _check_params(const bool force);

	/* parameters */
	/* timeout after which filter is reset if target not seen */
	uint32_t _vte_TIMEOUT_US = 3_s;
	int _vte_aid_mask{0};
	float _target_acc_unc;
	float _bias_unc;
	float _drone_acc_unc;
	float _gps_vel_noise;
	float _gps_pos_noise;
	bool  _ev_noise_md{false};
	float _ev_pos_noise;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::VTE_AID_MASK>) _param_vte_aid_mask,
		(ParamFloat<px4::params::VTE_BTOUT>) _param_vte_btout,
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
		(ParamInt<px4::params::VTE_MODE>) _param_vte_mode,
		(ParamInt<px4::params::VTE_EKF_AID>) _param_vte_ekf_aid,
		(ParamFloat<px4::params::VTE_MOVING_T_MAX>) _param_vte_moving_t_max,
		(ParamFloat<px4::params::VTE_MOVING_T_MIN>) _param_vte_moving_t_min
	)
};
} // namespace vision_target_estimator
