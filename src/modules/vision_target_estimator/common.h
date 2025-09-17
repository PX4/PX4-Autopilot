/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file common.h
 * @brief Definitions common to the position and orientation filters.
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#pragma once

#include <cstdint>
#include <drivers/drv_hrt.h>

using namespace time_literals;

namespace vision_target_estimator
{

/* timeout after which the target is not valid if no measurements are seen*/
static constexpr uint32_t target_valid_TIMEOUT_US = 2_s;

/* timeout after which the measurement is not valid*/
static constexpr uint32_t meas_valid_TIMEOUT_US = 1_s;

/* timeout after which the measurement is not considered updated*/
static constexpr uint32_t meas_updated_TIMEOUT_US = 100_ms;

/* Valid AoA measurement range between -60.00° and +60.00° for UWB*/
static constexpr float max_uwb_aoa_angle_degree = 60.0f;

static inline bool isMeasValid(hrt_abstime time_stamp)
{
	const hrt_abstime now = hrt_absolute_time();
	return (time_stamp <= now) &&
	       ((now - time_stamp) < static_cast<hrt_abstime>(meas_valid_TIMEOUT_US));
}

static inline bool isMeasUpdated(hrt_abstime time_stamp)
{
	const hrt_abstime now = hrt_absolute_time();
	return (time_stamp <= now) &&
	       ((now - time_stamp) < static_cast<hrt_abstime>(meas_updated_TIMEOUT_US));
}

enum SensorFusionMask : uint8_t {
	// Bit locations for fusion_mode
	NO_SENSOR_FUSION    = 0,
	USE_TARGET_GPS_POS  = (1 << 0),    ///< set to true to use target GPS position data
	USE_UAV_GPS_VEL     = (1 << 1),    ///< set to true to use drone GPS velocity data
	USE_EXT_VIS_POS     = (1 << 2),    ///< set to true to use target external vision-based relative position data
	USE_MISSION_POS     = (1 << 3),    ///< set to true to use the PX4 mission position
	USE_TARGET_GPS_VEL  = (1 << 4),	   ///< set to true to use target GPS velocity data. Only for moving targets.
	USE_UWB = (1 << 5) ///< set to true to use UWB.
};

} // namespace vision_target_estimator
