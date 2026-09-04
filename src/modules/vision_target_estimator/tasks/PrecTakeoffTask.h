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
 * @file PrecTakeoffTask.h
 * @brief Precision-takeoff VteTask: runs the estimator while a vertical takeoff is active and
 * uses the home position as pad reference.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include <uORB/Subscription.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/prec_takeoff_status.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_land_detected.h>

#include "VteTask.h"

class VisionTargetEstTest;
class VisionTargetEstTestable;

namespace vision_target_estimator
{

class VTEPosition;

class PrecTakeoffTask final : public VteTask
{
public:
	// Home is only used as pad reference if it is this close to the vehicle GNSS fix while landed.
	static constexpr float kMaxHomeDistM = 10.f;

	/* VteTask interface */
	uint8_t maskBit() const override { return task_bits::kPrecTakeoff; }
	const char *name() const override { return "prec_takeoff"; }
	void pollStatus() override;
	bool isReady() const override { return _is_taking_off; }
	bool isComplete() override { return !_is_taking_off; }
	void onActivate() override;
	void onPosEstStart(VTEPosition &pos) override;

	// Cache home as pad reference. Accepted only while landed and within kMaxHomeDistM of the GNSS fix.
	bool updateHomeReference();

private:
	friend class ::VisionTargetEstTest;
	friend class ::VisionTargetEstTestable;

	struct HomeReference {
		bool valid{false};
		double lat_deg{0.0};
		double lon_deg{0.0};
		float alt_m{0.f};
	};

	uORB::Subscription _prec_takeoff_status_sub{ORB_ID(prec_takeoff_status)};
	uORB::Subscription _home_position_sub{ORB_ID(home_position)};
	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

	HomeReference _home_ref{};
	bool _is_taking_off{false};
	bool _home_dist_warned{false};
};

} // namespace vision_target_estimator
