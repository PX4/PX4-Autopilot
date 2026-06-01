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
 * @file mavlink_command_params.h
 *
 * Per-MAV_CMD bitmask table declaring which params (1–7) PX4 supports.
 * Params not in the mask must be NaN (or 0.0, the conventional GCS default)
 * on receipt; any other value is rejected at the MAVLink boundary.
 *
 * Three masks per entry:
 *   mission  – params 1-4 for mission items (MISSION_ITEM / MISSION_ITEM_INT)
 *   command  – params 1-4 for commands (COMMAND_LONG / COMMAND_INT)
 *   params567 – params 5-7 for both; callers pass 0.0 to skip checking
 *
 * mission/command: bit 0 = param1, bit 1 = param2, bit 2 = param3, bit 3 = param4.
 * params567:       bit 0 = param5, bit 1 = param6, bit 2 = param7.
 *
 * For global-frame mission items the caller passes 0.0 for p5–p7 so that
 * lat/lon/alt are never checked through this table.
 *
 * The table must remain sorted ascending by cmd for binary search.
 * The static_assert below enforces this at compile time.
 * When adding a new MAV_CMD to PX4's mission or command handler, add an entry.
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <cstddef>

namespace mavlink_cmd_params
{

struct Entry {
	uint16_t cmd;
	uint8_t  mission;   // supported param bits (1-4) for mission items
	uint8_t  command;   // supported param bits (1-4) for COMMAND_LONG/INT
	uint8_t  params567; // supported param bits for params 5-7 (same for mission and command)
};

// Keep sorted by cmd value. Update when adding new supported commands or params.
// Symbolic names are listed in comments; raw integers are used so this header
// remains self-contained and requires no MAVLink includes.
static constexpr Entry SupportedCommandParams[] = {
	{   16, 0x0B, 0x0B, 0x07 }, // NAV_WAYPOINT:               p1:hold,p2:accept_r,p4:yaw; p5-7:lat/lon/alt
	{   17, 0x0C, 0x0C, 0x07 }, // NAV_LOITER_UNLIM:            p3:radius,p4:yaw; p5-7:lat/lon/alt
	{   19, 0x0F, 0x0F, 0x07 }, // NAV_LOITER_TIME:             p1-p4 all used; p5-7:lat/lon/alt
	{   20, 0x00, 0x00, 0x00 }, // NAV_RETURN_TO_LAUNCH:         no params
	{   21, 0x0A, 0x0A, 0x07 }, // NAV_LAND:                    p2:precision,p4:yaw; p5-7:lat/lon/alt
	{   22, 0x08, 0x08, 0x07 }, // NAV_TAKEOFF:                 p4:yaw; p5-7:lat/lon/alt
	{   31, 0x0B, 0x0B, 0x07 }, // NAV_LOITER_TO_ALT:           p1:hdg,p2:radius,p4:xtrack; p5-7:lat/lon/alt
	{   80, 0x07, 0x07, 0x07 }, // NAV_ROI:                     p1:mode,p2:wp_idx,p3:roi_idx; p5-7:lat/lon/alt
	{   84, 0x08, 0x08, 0x07 }, // NAV_VTOL_TAKEOFF:            p4:yaw; p5-7:lat/lon/alt
	{   85, 0x08, 0x08, 0x07 }, // NAV_VTOL_LAND:               p4:yaw; p5-7:lat/lon/alt
	{   93, 0x0F, 0x0F, 0x00 }, // NAV_DELAY:                   p1:delay,p2:hour,p3:min,p4:sec
	{  112, 0x01, 0x01, 0x00 }, // CONDITION_DELAY:             p1:seconds
	{  114, 0x01, 0x01, 0x00 }, // CONDITION_DISTANCE:          p1:distance
	{  176, 0x07, 0x07, 0x00 }, // DO_SET_MODE:                 p1:mode,p2:custom,p3:submode
	{  177, 0x03, 0x03, 0x00 }, // DO_JUMP:                     p1:index,p2:count
	{  178, 0x07, 0x07, 0x00 }, // DO_CHANGE_SPEED:             p1:type,p2:speed,p3:throttle
	{  179, 0x0F, 0x0F, 0x07 }, // DO_SET_HOME:                 p1:use_current,p2:roll,p3:pitch,p4:yaw; p5-7:lat/lon/alt
	{  189, 0x00, 0x00, 0x00 }, // DO_LAND_START:               no params
	{  195, 0x00, 0x01, 0x07 }, // DO_SET_ROI_LOCATION:         cmd:p1:gimbal; p5-7:lat/lon/alt
	{  196, 0x01, 0x01, 0x00 }, // DO_SET_ROI_WPNEXT_OFFSET:   p1:gimbal_id
	{  197, 0x01, 0x01, 0x00 }, // DO_SET_ROI_NONE:             p1:gimbal_id
	{  201, 0x07, 0x07, 0x00 }, // DO_SET_ROI:                  p1:mode,p2:wp_idx,p3:roi_idx
	{  206, 0x0F, 0x0F, 0x00 }, // DO_SET_CAM_TRIGG_DIST:       p1:dist,p2:shutter,p3:trigger,p4:camera_id
	{  211, 0x03, 0x03, 0x00 }, // DO_GRIPPER:                  p1:id,p2:action
	{  212, 0x03, 0x03, 0x00 }, // DO_AUTOTUNE_ENABLE:          p1:enable,p2:axis
	{  214, 0x07, 0x07, 0x00 }, // DO_SET_CAM_TRIGG_INTERVAL:  p1:cycle,p2:shutter,p3:camera_id
	{  400, 0x03, 0x03, 0x00 }, // COMPONENT_ARM_DISARM:        p1:arm,p2:force
	{  420, 0x07, 0x07, 0x00 }, // INJECT_FAILURE:              p1:unit,p2:type,p3:instance
	{  530, 0x03, 0x03, 0x00 }, // SET_CAMERA_MODE:             p1:camera_id,p2:mode
	{  532, 0x07, 0x07, 0x00 }, // SET_CAMERA_FOCUS:            p1:focus_type,p2:value,p3:camera_id
	{  534, 0x07, 0x07, 0x00 }, // SET_CAMERA_SOURCE:           p1:camera_id,p2:primary,p3:secondary
	{ 2000, 0x0F, 0x0F, 0x00 }, // IMAGE_START_CAPTURE:         p1:camera_id,p2:interval,p3:total,p4:seq
	{ 2001, 0x01, 0x01, 0x00 }, // IMAGE_STOP_CAPTURE:          p1:camera_id
	{ 2003, 0x0F, 0x0F, 0x00 }, // DO_TRIGGER_CONTROL:          p1:enable,p2:reset,p3:pause,p4:camera_id
	{ 2500, 0x07, 0x07, 0x00 }, // VIDEO_START_CAPTURE:         p1:stream_id,p2:status_freq,p3:camera_id
	{ 2501, 0x03, 0x03, 0x00 }, // VIDEO_STOP_CAPTURE:          p1:stream_id,p2:camera_id
	{ 3000, 0x03, 0x03, 0x00 }, // DO_VTOL_TRANSITION:          p1:state,p2:force_immediate
	{ 4501, 0x00, 0x00, 0x00 }, // CONDITION_GATE:              no params used by PX4
	{ 5000, 0x00, 0x00, 0x07 }, // NAV_FENCE_RETURN_POINT:      p5-7:lat/lon/alt
	{ 5001, 0x01, 0x01, 0x03 }, // NAV_FENCE_POLYGON_VERTEX_INCLUSION: p1:vertex_count; p5-6:lat/lon
	{ 5002, 0x01, 0x01, 0x03 }, // NAV_FENCE_POLYGON_VERTEX_EXCLUSION: p1:vertex_count; p5-6:lat/lon
	{ 5003, 0x01, 0x01, 0x03 }, // NAV_FENCE_CIRCLE_INCLUSION:  p1:radius; p5-6:lat/lon
	{ 5004, 0x01, 0x01, 0x03 }, // NAV_FENCE_CIRCLE_EXCLUSION:  p1:radius; p5-6:lat/lon
	{ 5100, 0x00, 0x00, 0x07 }, // NAV_RALLY_POINT:             p5-7:lat/lon/alt
	{42600, 0x0F, 0x0F, 0x00 }, // DO_WINCH:                    p1-p4 all used
};

static constexpr size_t SupportedCommandParamsCount = sizeof(SupportedCommandParams) / sizeof(SupportedCommandParams[0]);

// Verify the table is sorted at compile time.
static constexpr bool _is_sorted()
{
	for (size_t i = 1; i < SupportedCommandParamsCount; ++i) {
		if (SupportedCommandParams[i].cmd <= SupportedCommandParams[i - 1].cmd) { return false; }
	}

	return true;
}

static_assert(_is_sorted(), "mavlink_command_params::SupportedCommandParams must be sorted ascending by cmd");

// Returns true when the float value means "param not provided" per MAVLink convention.
// Accepts NaN (the standard) and ±0.0 (the common GCS default for unused fields).
// Bit-mask avoids float comparison (-Wfloat-equal) and fpclassify (absent in NuttX <cmath>).
static inline bool param_is_unset(float v)
{
	uint32_t bits;
	__builtin_memcpy(&bits, &v, sizeof(bits));
	return std::isnan(v) || (bits & 0x7FFFFFFFu) == 0u;
}

/**
 * Check params 1–7 of an incoming mission item or command against the table.
 *
 * Pass 0.0f for p5–p7 to skip checking those params (e.g. for global-frame
 * mission items where p5–p7 carry lat/lon/alt).
 *
 * @param cmd         MAV_CMD value
 * @param for_mission true for mission items, false for COMMAND_LONG/INT
 * @param p1–p7       raw float param values from the MAVLink message
 * @return            0  all unsupported params are unset
 *                    1–7 1-based index of the first offending param
 *                    -1  command not in table (no validation applied)
 */
[[maybe_unused]] static int check_params(uint16_t cmd, bool for_mission,
		float p1, float p2, float p3, float p4,
		float p5 = 0.0f, float p6 = 0.0f, float p7 = 0.0f)
{
	// Binary search
	size_t lo = 0;
	size_t hi = SupportedCommandParamsCount - 1;

	while (lo <= hi) {
		const size_t mid = lo + (hi - lo) / 2;

		if (SupportedCommandParams[mid].cmd == cmd) {
			const uint8_t mask    = for_mission ? SupportedCommandParams[mid].mission : SupportedCommandParams[mid].command;
			const uint8_t mask567 = SupportedCommandParams[mid].params567;

			if (!(mask & (1u << 0)) && !param_is_unset(p1)) { return 1; }

			if (!(mask & (1u << 1)) && !param_is_unset(p2)) { return 2; }

			if (!(mask & (1u << 2)) && !param_is_unset(p3)) { return 3; }

			if (!(mask & (1u << 3)) && !param_is_unset(p4)) { return 4; }

			if (!(mask567 & (1u << 0)) && !param_is_unset(p5)) { return 5; }

			if (!(mask567 & (1u << 1)) && !param_is_unset(p6)) { return 6; }

			if (!(mask567 & (1u << 2)) && !param_is_unset(p7)) { return 7; }

			return 0;

		} else if (SupportedCommandParams[mid].cmd < cmd) {
			lo = mid + 1;

		} else {
			if (mid == 0) { break; }

			hi = mid - 1;
		}
	}

	return -1; // command not in table — no validation applied
}

} // namespace mavlink_cmd_params
