/***************************************************************************
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
 * @file prec_takeoff.h
 *
 * Keeps a vertical takeoff over the precision pad: steers the current takeoff setpoint onto
 * the landing_target_pose estimate and publishes prec_takeoff_status for the estimator and
 * the flight task.
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/position_setpoint.h>
#include <uORB/topics/prec_takeoff_status.h>
#include <uORB/topics/vehicle_local_position.h>

class PrecTakeoff : public ModuleParams
{
public:
	explicit PrecTakeoff(ModuleParams *parent);
	~PrecTakeoff() override = default;

	bool enabled() const { return _param_mis_tko_prec.get(); }

	/**
	 * Call every cycle while a vertical takeoff setpoint is active.
	 * Moves the setpoint onto the target when a fresh estimate is available.
	 *
	 * @param takeoff_reached true once the takeoff altitude is reached
	 * @param now current time, used to reject stale target estimates
	 * @return true if the setpoint was changed
	 */
	bool run(const vehicle_local_position_s &local_pos, position_setpoint_s &current_sp, bool takeoff_reached,
		 hrt_abstime now);

	/**
	 * Call once per navigator cycle after the modes ran. Publishes the current state and
	 * falls back to STOPPED when run() was not called this cycle.
	 */
	void publish_status();

private:
	bool update_setpoint(const vehicle_local_position_s &local_pos, position_setpoint_s &current_sp, hrt_abstime now);

	uORB::Subscription _target_pose_sub{ORB_ID(landing_target_pose)};
	uORB::Publication<prec_takeoff_status_s> _status_pub{ORB_ID(prec_takeoff_status)};

	MapProjection _map_ref{};

	bool _active{false}; /**< run() was called this cycle */
	bool _was_active{false}; /**< run() was called in the previous cycle */
	bool _reached{false};
	bool _setpoint_adjusted{false}; /**< a valid target adjusted the current takeoff setpoint */

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::MIS_TKO_PREC>) _param_mis_tko_prec
	)
};
