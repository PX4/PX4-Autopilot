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
 * @file detect_and_avoid.h
 *
 * Helper class to do detect and avoid based on F3442 standard
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include "../DaaHelper.h"

#include <stdbool.h>
#include <stdint.h>
#include <matrix/math.hpp>

#include <lib/geo/geo.h>

#include <drivers/drv_hrt.h>
#include <uORB/topics/detect_and_avoid.h>
#include <px4_platform_common/module_params.h>

/**
 * @brief ASTM F3442 conflict evaluation.
 *
 * Uses four nested cylindrical zones around each aircraft: NMAC (CRITICAL),
 * Well-Clear (HIGH), and two augmented zones grown by relative speed and a
 * latency budget (MEDIUM and LOW). The strictest zone breached defines the
 * conflict level returned to the navigator.
 */
class DaaF3442 : public ModuleParams
{
public:
	DaaF3442();

	/** @brief Compute distance, time-to-min-distance and the F3442 conflict level for one target. */
	uint8_t calculate_daa_stats(const aircraft_state_s &uav_state, const aircraft_state_s &traffic_state,
				    daa_stats_s &daa_stats);

	/** @brief True if every component of @p distance is within the symmetric @p bounds box. */
	static bool is_in_bounds(const matrix::Vector2f &distance, const matrix::Vector2f &bounds);

	/**
	 * @brief Expand a base bounds box by |vel| * |latency|.
	 *
	 * Result is symmetric so it covers a sudden course change regardless of @p vel sign.
	 */
	static void calculate_aircraft_conflict_volume(const matrix::Vector2f &base_bounds, const matrix::Vector2f &vel,
			const float latency, matrix::Vector2f &output_bounds);

	/** @brief Sum of the ownship and traffic augmented volumes for a given base zone and latency. */
	static void calculate_augmented_boundaries(const matrix::Vector2f &base_bounds, const matrix::Vector2f &uav_vel_hor_vert,
			const matrix::Vector2f &traffic_vel, const float latency, matrix::Vector2f &augmented_bounds);

	/** @brief Walk the four nested zones from CRITICAL outwards and return the strictest one breached. */
	uint8_t calculate_conflict_level(const matrix::Vector2f &distance, const matrix::Vector2f &uav_vel_hor_vert,
					 const matrix::Vector2f &traffic_vel);

	/**
	 * @brief Refresh F3442 zone radii and latency from parameters.
	 *
	 * Rejects negative or nan values and zones that are not properly nested
	 * (e.g. NMAC must fit inside WC).
	 */
	bool try_setting_params();

private:
	// Per-aircraft half-zones (radius, height). They match the parameter defaults and the ASTM
	// F3442 zones once doubled in calculate_conflict_level(): NMAC = 2 x (153 m, 31 m),
	// Well-Clear = 2 x (610 m, 77 m). Overwritten by try_setting_params() on activation.
	matrix::Vector2f _nmac_bounds_m{77, 16};
	matrix::Vector2f _wc_bounds_m{305, 39};
	float _aug_nmac_latency_s{33};
	float _aug_wc_latency_s{33};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::DAA_LVL_CRIT_RAD>) _param_daa_lvl_critical_rad,
		(ParamFloat<px4::params::DAA_LVL_CRIT_HGT>) _param_daa_lvl_critical_hgt,
		(ParamFloat<px4::params::DAA_LVL_HIGH_RAD>) _param_daa_lvl_high_rad,
		(ParamFloat<px4::params::DAA_LVL_HIGH_HGT>) _param_daa_lvl_high_hgt,
		(ParamInt<px4::params::DAA_LVL_MED_TIME>) _param_daa_lvl_medium_time,
		(ParamInt<px4::params::DAA_LVL_LOW_TIME>) _param_daa_lvl_low_time
	)
};
