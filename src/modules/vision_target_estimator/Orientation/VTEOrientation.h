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
 * @file VTEOrientation.h
 * @brief Estimate the orientation of a target by processing and fusing sensor
 * data in a Kalman Filter.
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#pragma once

#include <px4_platform_common/module_params.h>

#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/fiducial_marker_yaw_report.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vte_orientation.h>
#include <uORB/topics/vte_aid_source1d.h>

#include <matrix/math.hpp>

#include "../common.h"
#include "KF_orientation.h"

namespace vision_target_estimator
{

using namespace time_literals;

class VTEOrientation : public ModuleParams
{
public:
	VTEOrientation();
	virtual ~VTEOrientation();

	/**
	 * Run the predict/update cycle once. Pulls in the latest yaw observation, initializes the
	 * filter on the first viable sample, fuses it, and publishes the state.
	 */
	void update();

	/** Prepare per-axis history buffers. Call once at startup. */
	bool init();

	/** Drop the estimator state so the next valid observation re-initializes it. */
	void resetFilter();

	/** Timeout after which the whole filter is declared stale (see @c timedOut). */
	void setVteTimeout(hrt_abstime tout) { _vte_timeout_us = tout; }

	/** Maximum age for the published yaw to be marked valid. */
	void setTargetValidTimeout(hrt_abstime tout)
	{
		_target_valid_timeout_us = tout;
	}

	/** Maximum age for a raw observation to still be considered for fusion. */
	void setMeasRecentTimeout(hrt_abstime tout)
	{
		_meas_recent_timeout_us = tout;
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

	/** True when at least one yaw aid source is enabled (required to keep the estimator running). */
	bool fusionEnabled() const
	{
		return _vte_aid_mask.flags.use_vision_pos;
	}

	void print_status() const;

protected:
	/*
	 * update parameters.
	 */
	void updateParams() override;

	uORB::Publication<vte_orientation_s> _target_orientation_pub{ORB_ID(vte_orientation)};

	// publish innovations target_estimator_gps_pos
	uORB::Publication<vte_aid_source1d_s> _vte_aid_ev_yaw_pub{ORB_ID(vte_aid_ev_yaw)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

private:
	struct VisionObs {
		hrt_abstime timestamp{0};
		float yaw{0.f};
		float yaw_var{0.f};
	};

	/** Seed state and covariance from the first valid yaw sample. */
	void initEstimator(const VisionObs &vision_obs);
	/** Pull in new observations, initialise on first valid sample, and run fusion. */
	bool performUpdateStep();
	/** Advance the yaw state forward by @p dt using the constant yaw-rate model. */
	void predictionStep(float dt);

	inline bool isMeasRecent(hrt_abstime ts) const
	{
		return !hasTimedOut(ts, _meas_recent_timeout_us);
	}

	bool isVisionDataValid(const fiducial_marker_yaw_report_s &fiducial_marker_yaw);
	bool processObsVision(VisionObs &vision_obs);
	bool fuseMeas(const VisionObs &vision_obs);
	void publishTarget();
	bool shouldEmitWarning(hrt_abstime &last_warn);

	uORB::Subscription _fiducial_marker_yaw_report_sub{ORB_ID(fiducial_marker_yaw_report)};

	bool _estimator_initialized{false};

	KF_orientation _target_est_yaw{};
	vte_orientation_s _orientation_msg{};

	hrt_abstime _last_predict{0}; // timestamp of last filter prediction
	hrt_abstime _last_update{0}; // timestamp of last filter update (used to check timeout)
	hrt_abstime _vision_warn_last{0};

	/* parameters from vision_target_estimator_params.yaml */
	hrt_abstime _vte_timeout_us{3_s};
	hrt_abstime _target_valid_timeout_us{2_s};
	hrt_abstime _meas_recent_timeout_us{1_s};
	SensorFusionMaskU _vte_aid_mask{};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::VTE_YAW_UNC_IN>) _param_vte_yaw_unc_in,
		(ParamFloat<px4::params::VTE_YAW_ACC_UNC>) _param_vte_yaw_acc_unc,
		(ParamFloat<px4::params::VTE_EVA_NOISE>) _param_vte_ev_angle_noise,
		(ParamFloat<px4::params::VTE_YAW_NIS_THRE>) _param_vte_yaw_nis_thre
	)
};
} // namespace vision_target_estimator
