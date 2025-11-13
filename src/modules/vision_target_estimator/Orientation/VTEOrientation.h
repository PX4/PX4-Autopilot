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
 * @file VTEOrientation.h
 * @brief Estimate the orientation of a target by processing and fusing sensor data in a Kalman Filter.
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
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/fiducial_marker_yaw_report.h>
#include <uORB/topics/vision_target_est_orientation.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/estimator_aid_source1d.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <matrix/Matrix.hpp>
#include <matrix/Quaternion.hpp>
#include <lib/conversion/rotation.h>
#include <lib/geo/geo.h>
#include "KF_orientation.h"
#include "../common.h"


namespace vision_target_estimator
{

using namespace time_literals;

class VTEOrientation: public ModuleParams
{
public:

	VTEOrientation();
	virtual ~VTEOrientation();

	/*
	 * Get new measurements and update the state estimate
	 */
	void update();

	bool init();

	void resetFilter();

	void set_range_sensor(const float dist, const bool valid, const hrt_abstime timestamp);
	void set_vte_timeout(const uint32_t tout) {_vte_timeout_us = tout;};
	void set_target_valid_timeout(const uint32_t tout) {_target_valid_timeout_us = tout;};
	void set_meas_recent_timeout(const uint32_t tout) {_meas_recent_timeout_us = tout;};
	void set_meas_updated_timeout(const uint32_t tout) {_meas_updated_timeout_us = tout;};
	void set_vte_aid_mask(const uint16_t mask_value) {_vte_aid_mask.value = mask_value;};

	bool timedOut() const {return _estimator_initialized && hasTimedOut(_last_update, _vte_timeout_us);};
	bool fusionEnabled() const {return _vte_aid_mask.value != 0;};

protected:

	/*
	 * update parameters.
	 */
	void updateParams() override;

	uORB::Publication<vision_target_est_orientation_s> _targetOrientationPub{ORB_ID(vision_target_est_orientation)};

	// publish innovations target_estimator_gps_pos
	uORB::Publication<estimator_aid_source1d_s> _vte_aid_ev_yaw_pub{ORB_ID(vte_aid_ev_yaw)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

private:
	enum class ObsType : uint8_t {
		Fiducial_marker,
		Type_count
	};

	static constexpr size_t kObsTypeCount = static_cast<size_t>(ObsType::Type_count);
	static constexpr size_t obsIndex(ObsType type)
	{
		return static_cast<size_t>(type);
	}

	union ObsValidMaskU {
		struct {
			uint8_t fuse_vision : 1; ///< bit0: external vision data ready to be fused
			uint8_t reserved    : 7; ///< bit1..7: reserved for future use
		} flags;

		uint8_t value{0};
	};

	static_assert(sizeof(ObsValidMaskU) == 1, "Unexpected masking size");

	struct TargetObs {
		ObsType type;
		hrt_abstime timestamp = 0;
		bool updated = false;
		float meas{};
		float meas_unc{};
		matrix::Vector<float, State::size> meas_h_theta{};
	};

	bool initEstimator(const ObsValidMaskU &fusion_mask, const TargetObs observations[kObsTypeCount]);
	bool performUpdateStep();
	void predictionStep();

	inline bool isMeasRecent(hrt_abstime ts) const
	{
		return !hasTimedOut(ts, _meas_recent_timeout_us);
	}

	inline bool isMeasUpdated(hrt_abstime ts) const
	{
		return !hasTimedOut(ts, _meas_updated_timeout_us);
	}

	void processObservations(ObsValidMaskU &fusion_mask,
				 TargetObs observations[kObsTypeCount]);
	bool fuseActiveMeasurements(ObsValidMaskU &fusion_mask, const TargetObs observations[kObsTypeCount]);

	/* Vision data */
	void handleVisionData(ObsValidMaskU &fusion_mask, TargetObs &vision_obs);
	bool isVisionDataValid(const fiducial_marker_yaw_report_s &fiducial_marker_yaw) const;
	bool processObsVision(const fiducial_marker_yaw_report_s &fiducial_marker_yaw, TargetObs &obs) const;

	bool fuseMeas(const TargetObs &target_pos_obs);
	void publishTarget();
	void resetObservations();

	uORB::Subscription _fiducial_marker_yaw_report_sub{ORB_ID(fiducial_marker_yaw_report)};

	FloatStamped _range_sensor{};

	bool _estimator_initialized{false};

	KF_orientation _target_est_yaw{};
	TargetObs _obs_buffer[kObsTypeCount] {};
	estimator_aid_source1d_s _aid_src1d_buffer{};
	vision_target_est_orientation_s _orientation_msg{};

	hrt_abstime _last_predict{0}; // timestamp of last filter prediction
	hrt_abstime _last_update{0}; // timestamp of last filter update (used to check timeout)

	void checkMeasurementInputs();

	/* parameters from vision_target_estimator_params.c*/
	uint32_t _vte_timeout_us{3_s};
	uint32_t _target_valid_timeout_us{2_s};
	uint32_t _meas_recent_timeout_us{1_s};
	uint32_t _meas_updated_timeout_us{100_ms};
	SensorFusionMaskU _vte_aid_mask{};
	float _yaw_unc{1.f};
	float _min_ev_angle_var{0.025f};
	bool  _ev_noise_md{false};
	float _nis_threshold{3.84f};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::VTE_YAW_UNC_IN>) _param_vte_yaw_unc_in,
		(ParamInt<px4::params::VTE_YAW_EN>) _param_vte_yaw_en,
		(ParamFloat<px4::params::VTE_EVA_NOISE>) _param_vte_ev_angle_noise,
		(ParamInt<px4::params::VTE_EV_NOISE_MD>) _param_vte_ev_noise_md,
		(ParamFloat<px4::params::VTE_YAW_NIS_THRE>) _param_vte_yaw_nis_thre
	)
};
} // namespace vision_target_estimator
