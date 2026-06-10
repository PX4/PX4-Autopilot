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
 * @file PrecLandTask.h
 * @brief Precision-landing VteTask: owns precland-specific uORB state and the
 * cached mission land setpoint used to seed the position estimator.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include <uORB/Subscription.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/navigator_mission_item.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/prec_land_status.h>
#include <uORB/topics/vehicle_land_detected.h>

#include "VteTask.h"

class VisionTargetEstTest;
class VisionTargetEstTestable;

namespace vision_target_estimator
{

class VTEPosition;

class PrecLandTask final : public VteTask
{
public:
	/* VteTask interface */
	uint8_t maskBit() const override { return task_bits::kPrecLand; }
	const char *name() const override { return "prec_land"; }
	void pollStatus() override;
	bool isReady() const override { return _is_in_prec_land; }
	bool isComplete() override;
	void onActivate() override;
	void onPosEstStart(VTEPosition &pos) override;

	/* Helpers used by tests and by VisionTargetEst logging */
	// Select the first valid LAND setpoint in the live triplet (next preferred over current).
	const position_setpoint_s *findLandSetpoint();
	// Resolve and cache the precision-landing target from navigator_mission_item
	// (preferred) or the triplet; re-used across estimator restarts in the same task.
	bool updateMissionSetpoint();
	bool cacheMissionSetpoint(const navigator_mission_item_s &mission_item);
	void resetMissionSetpoint() { _cached_mission_setpoint = {}; }

private:
	friend class ::VisionTargetEstTest;
	friend class ::VisionTargetEstTestable;

	uORB::Subscription _home_position_sub{ORB_ID(home_position)};
	uORB::Subscription _navigator_mission_item_sub{ORB_ID(navigator_mission_item)};
	uORB::Subscription _pos_sp_triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _prec_land_status_sub{ORB_ID(prec_land_status)};

	position_setpoint_triplet_s _pos_sp_triplet_buffer{};
	position_setpoint_s _cached_mission_setpoint{};
	bool _is_in_prec_land{false};
};

} // namespace vision_target_estimator
