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
 * @file VTEOrientation.h
 * @brief Estimate the orientation of a target by processessing and fusing sensor data in a Kalman Filter.
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
#include <uORB/topics/vehicle_status.h>
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
#include <lib/conversion/rotation.h>
#include <lib/geo/geo.h>
#include "KF_orientation_moving.h"
#include "KF_orientation_static.h"

using namespace time_literals;

namespace vision_target_estimator
{

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

	void set_range_sensor(const float dist, const bool valid);

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

	uORB::Publication<vision_target_est_orientation_s> _targetOrientationPub{ORB_ID(vision_target_est_orientation)};

	// publish innovations target_estimator_gps_pos
	uORB::Publication<estimator_aid_source1d_s> _vte_aid_ev_yaw_pub{ORB_ID(vte_aid_ev_yaw)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

private:

	enum class TargetMode {
		Stationary = 0,
		Moving = 1,
		NotInit
	};

	TargetMode _target_mode{TargetMode::NotInit};

	struct targetObsOrientation {
		hrt_abstime timestamp;
		// Theta
		bool updated_theta = false;
		float meas_theta = 0.f;
		float meas_unc_theta = 0.f;
		matrix::Vector2f meas_h_theta;
	};

	bool selectTargetEstimator();
	bool initEstimator(const float theta_init);
	bool update_step();
	void predictionStep();

	bool processObsVisionOrientation(const fiducial_marker_yaw_report_s &fiducial_marker_pose, targetObsOrientation &obs);

	bool fuse_orientation(const targetObsOrientation &target_pos_obs);
	void publishTarget();

	uORB::Subscription _fiducial_marker_orientation_sub{ORB_ID(fiducial_marker_yaw_report)};

	struct localOrientation {
		bool valid = false;
		float yaw = 0.f;
		hrt_abstime last_update = 0;
	};

	localOrientation _local_orientation{};

	struct rangeSensor {
		bool valid = false;
		float dist_bottom;
		hrt_abstime last_update = 0;
	};

	rangeSensor _range_sensor{};

	bool _estimator_initialized{false};

	Base_KF_orientation *_target_estimator_orientation {nullptr};

	hrt_abstime _last_predict{0}; // timestamp of last filter prediction
	hrt_abstime _last_update{0}; // timestamp of last filter update (used to check timeout)

	void _check_params(const bool force);

	/* parameters */
	uint32_t _vte_TIMEOUT_US = 3_s; // timeout after which filter is reset if target not seen
	float _yaw_unc;
	float _ev_angle_noise;
	bool  _ev_noise_md{false};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::VTE_YAW_UNC_IN>) _param_vte_yaw_unc_in,
		(ParamFloat<px4::params::VTE_BTOUT>) _param_vte_btout,
		(ParamInt<px4::params::VTE_MODE>) _param_vte_mode,
		(ParamInt<px4::params::VTE_YAW_EN>) _param_vte_yaw_en,
		(ParamFloat<px4::params::VTE_EVA_NOISE>) _param_vte_ev_angle_noise,
		(ParamInt<px4::params::VTE_EV_NOISE_MD>) _param_vte_ev_noise_md
	)
};
} // namespace vision_target_estimator
