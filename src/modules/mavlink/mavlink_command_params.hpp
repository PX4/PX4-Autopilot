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
 * @file mavlink_command_params.hpp
 *
 * Per-MAV_CMD bitmask table declaring which params (1–7) PX4 supports.
 * Params not in the mask must be NaN (or 0.0, the conventional GCS default)
 * on receipt; any other value is rejected at the MAVLink boundary.
 *
 * Two masks per entry — one for mission items, one for commands:
 *   bits 0–3: params 1–4  (bit 0 = param1, …, bit 3 = param4)
 *   bits 4–6: params 5–7  (bit 4 = param5, bit 5 = param6, bit 6 = param7)
 *
 * For global-frame MISSION_ITEM_INT, check_params_int_for_vehicle() is used so
 * that p5/p6 are validated as int32 (INT32_MAX = unset) and p7 as float.
 *
 * A secondary VehicleParamOverrides table holds per-airframe additions (e.g.
 * NAV_TAKEOFF pitch angle on FW); use check_params_for_vehicle() for callers
 * that know the vehicle type.
 *
 * The table must remain sorted ascending by cmd for binary search.
 * The static_assert below enforces this at compile time.
 * When adding a new MAV_CMD to PX4's mission or command handler, add an entry.
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <cstddef>
#include <uORB/topics/vehicle_status.h>

namespace mavlink_cmd_params
{

struct Entry {
	uint16_t cmd;
	uint8_t  mission; // bits 0-3: params 1-4; bits 4-6: params 5-7 (mission items)
	uint8_t  command; // bits 0-3: params 1-4; bits 4-6: params 5-7 (COMMAND_LONG/INT)
};

// Keep sorted by cmd value. Update when adding new supported commands or params.
// Symbolic names are listed in comments; raw integers are used so this header
// remains self-contained (aside from vehicle_status.h for VEHICLE_TYPE_* constants).
//
// Encoding: mission/command byte = (params1_4_mask) | (params5_7_mask << 4)
//   where params1_4_mask has bit N = param N+1 for N in 0..3,
//   and   params5_7_mask has bit N = param N+5 for N in 0..2.
static constexpr Entry SupportedCommandParams[] = {
	//  cmd  mission command
	{   16, 0x7B, 0x7B }, // NAV_WAYPOINT:               p1:hold,p2:accept_r,p4:yaw; p5-7:lat/lon/alt
	{   17, 0x7C, 0x7C }, // NAV_LOITER_UNLIM:            p3:radius,p4:yaw; p5-7:lat/lon/alt
	{   19, 0x7F, 0x7F }, // NAV_LOITER_TIME:             p1-p4 all used; p5-7:lat/lon/alt
	{   20, 0x00, 0x00 }, // NAV_RETURN_TO_LAUNCH:         no params
	{   21, 0x7B, 0x7B }, // NAV_LAND:                    p1:abort_alt,p2:precision,p4:yaw; p5-7:lat/lon/alt
	{   22, 0x78, 0x78 }, // NAV_TAKEOFF:                 p4:yaw; p5-7:lat/lon/alt (FW/VTOL also get p1 via override)
	{   31, 0x7B, 0x7B }, // NAV_LOITER_TO_ALT:           p1:hdg,p2:radius,p4:xtrack; p5-7:lat/lon/alt
	{   80, 0x77, 0x77 }, // NAV_ROI:                     p1:mode,p2:wp_idx,p3:roi_idx; p5-7:lat/lon/alt
	{   84, 0x78, 0x7C }, // NAV_VTOL_TAKEOFF:            mission:p4:yaw only (p3 unused by mission_block); cmd:p3:approach_hdg,p4:yaw; p5-7:lat/lon/alt
	{   85, 0x78, 0x7F }, // NAV_VTOL_LAND:               mission:p4:yaw only (p1-3 unused by mission_block); cmd:p1:options,p2:approach_hdg,p3:loiter_r,p4:yaw; p5-7:lat/lon/alt
	{   93, 0x0F, 0x0F }, // NAV_DELAY:                   p1:delay,p2:hour,p3:min,p4:sec
	{  112, 0x01, 0x01 }, // CONDITION_DELAY:             p1:seconds
	{  114, 0x01, 0x01 }, // CONDITION_DISTANCE:          p1:distance
	{  176, 0x07, 0x07 }, // DO_SET_MODE:                 p1:mode,p2:custom,p3:submode
	{  177, 0x03, 0x03 }, // DO_JUMP:                     p1:index,p2:count
	{  178, 0x07, 0x07 }, // DO_CHANGE_SPEED:             p1:type,p2:speed,p3:throttle
	{  179, 0x7F, 0x7F }, // DO_SET_HOME:                 p1:use_current,p2:roll,p3:pitch,p4:yaw; p5-7:lat/lon/alt
	{  189, 0x00, 0x00 }, // DO_LAND_START:               no params
	{  195, 0x70, 0x71 }, // DO_SET_ROI_LOCATION:         mission:p5-7:lat/lon/alt; cmd:p1:gimbal,p5-7:lat/lon/alt
	{  196, 0x01, 0x01 }, // DO_SET_ROI_WPNEXT_OFFSET:   p1:gimbal_id
	{  197, 0x01, 0x01 }, // DO_SET_ROI_NONE:             p1:gimbal_id
	{  201, 0x07, 0x07 }, // DO_SET_ROI:                  p1:mode,p2:wp_idx,p3:roi_idx
	{  206, 0x0F, 0x0F }, // DO_SET_CAM_TRIGG_DIST:       p1:dist,p2:shutter,p3:trigger,p4:camera_id
	{  211, 0x03, 0x03 }, // DO_GRIPPER:                  p1:id,p2:action
	{  212, 0x03, 0x03 }, // DO_AUTOTUNE_ENABLE:          p1:enable,p2:axis
	{  214, 0x07, 0x07 }, // DO_SET_CAM_TRIGG_INTERVAL:  p1:cycle,p2:shutter,p3:camera_id
	{  400, 0x03, 0x03 }, // COMPONENT_ARM_DISARM:        p1:arm,p2:force
	{  420, 0x07, 0x07 }, // INJECT_FAILURE:              p1:unit,p2:type,p3:instance
	{  530, 0x03, 0x03 }, // SET_CAMERA_MODE:             p1:camera_id,p2:mode
	{  532, 0x07, 0x07 }, // SET_CAMERA_FOCUS:            p1:focus_type,p2:value,p3:camera_id
	{  534, 0x07, 0x07 }, // SET_CAMERA_SOURCE:           p1:camera_id,p2:primary,p3:secondary
	{  611, 0x00, 0x70 }, // DO_SET_GLOBAL_ORIGIN:        cmd:p5-7:lat/lon/alt (not a mission item)
	{ 2000, 0x0F, 0x0F }, // IMAGE_START_CAPTURE:         p1:camera_id,p2:interval,p3:total,p4:seq
	{ 2001, 0x01, 0x01 }, // IMAGE_STOP_CAPTURE:          p1:camera_id
	{ 2003, 0x0F, 0x0F }, // DO_TRIGGER_CONTROL:          p1:enable,p2:reset,p3:pause,p4:camera_id
	{ 2500, 0x07, 0x07 }, // VIDEO_START_CAPTURE:         p1:stream_id,p2:status_freq,p3:camera_id
	{ 2501, 0x03, 0x03 }, // VIDEO_STOP_CAPTURE:          p1:stream_id,p2:camera_id
	{ 3000, 0x03, 0x03 }, // DO_VTOL_TRANSITION:          p1:state,p2:force_immediate
	{ 4501, 0x00, 0x00 }, // CONDITION_GATE:              no params used by PX4
	{ 5000, 0x70, 0x00 }, // NAV_FENCE_RETURN_POINT:      mission:p5-7:lat/lon/alt; cmd:none
	{ 5001, 0x31, 0x01 }, // NAV_FENCE_POLYGON_VERTEX_INCLUSION: p1:vertex_count; mission:p5-6:lat/lon
	{ 5002, 0x31, 0x01 }, // NAV_FENCE_POLYGON_VERTEX_EXCLUSION: p1:vertex_count; mission:p5-6:lat/lon
	{ 5003, 0x31, 0x01 }, // NAV_FENCE_CIRCLE_INCLUSION:  p1:radius; mission:p5-6:lat/lon
	{ 5004, 0x31, 0x01 }, // NAV_FENCE_CIRCLE_EXCLUSION:  p1:radius; mission:p5-6:lat/lon
	{ 5100, 0x70, 0x00 }, // NAV_RALLY_POINT:             mission:p5-7:lat/lon/alt; cmd:none
	{42600, 0x0F, 0x0F }, // DO_WINCH:                    p1-p4 all used
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

// Bit-mask helpers avoid float comparison (-Wfloat-equal) and fpclassify (absent in NuttX).
static inline bool param_is_unset(float v)
{
	uint32_t bits;
	__builtin_memcpy(&bits, &v, sizeof(bits));
	return std::isnan(v) || (bits & 0x7FFFFFFFu) == 0u;
}

// True when v is exactly ±0.0 (not NaN). Used to detect GCS sending 0 instead of NaN.
static inline bool param_is_zero(float v)
{
	uint32_t bits;
	__builtin_memcpy(&bits, &v, sizeof(bits));
	return !std::isnan(v) && (bits & 0x7FFFFFFFu) == 0u;
}

// INT32_MAX is the MAVLink sentinel for "int32 param not provided".
static inline bool int_param_is_unset(int32_t v)
{
	return v == INT32_MAX;
}

// Vehicle type bitmask for per-vehicle parameter support.
// bit 0 = fixed-wing (FW), bit 1 = multicopter (MC), bit 2 = VTOL, bit 3 = rover.
enum VehicleType : uint8_t {
	VEHICLE_FW    = (1u << 0),
	VEHICLE_MC    = (1u << 1),
	VEHICLE_VTOL  = (1u << 2),
	VEHICLE_ROVER = (1u << 3),
};

// Maps vehicle_status_s fields to a VehicleType bitmask.
static inline uint8_t vehicle_type_bitmask(bool is_vtol, uint8_t vehicle_type)
{
	if (is_vtol) { return VEHICLE_VTOL; }

	if (vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) { return VEHICLE_FW; }

	if (vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) { return VEHICLE_MC; }

	if (vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROVER) { return VEHICLE_ROVER; }

	return 0;
}

// Extends the base table for vehicle-specific params that differ across airframe types.
// Each entry's masks are OR'd with the base Entry masks when the vehicle_mask matches.
struct VehicleOverride {
	uint16_t cmd;
	uint8_t  vehicle_mask; // bitmask of VehicleType values this entry applies to
	uint8_t  mission;      // additional allowed param bits (bits 0-3 = p1-p4, bits 4-6 = p5-p7)
	uint8_t  command;      // additional allowed param bits (bits 0-3 = p1-p4, bits 4-6 = p5-p7)
};

// When adding vehicle-specific param differences, add entries here.
static constexpr VehicleOverride VehicleParamOverrides[] = {
	// NAV_TAKEOFF: p1 (minimum pitch angle) is used by FW and VTOL-FW; MC ignores it, but
	// GCS (e.g. QGC) send p1=-1 on MC takeoff, so accept p1 for MC too instead of denying.
	{ 22, VEHICLE_FW | VEHICLE_VTOL | VEHICLE_MC, 0x01, 0x01 },
};

// Binary search + vehicle override lookup. Returns the adjusted mask, or -1 if cmd not in table.
static int _find_mask(uint16_t cmd, bool for_mission, uint8_t vehicle_type)
{
	size_t lo = 0;
	size_t hi = SupportedCommandParamsCount - 1;

	while (lo <= hi) {
		const size_t mid = lo + (hi - lo) / 2;

		if (SupportedCommandParams[mid].cmd == cmd) {
			uint8_t mask = for_mission
				       ? SupportedCommandParams[mid].mission
				       : SupportedCommandParams[mid].command;

			for (const auto &ov : VehicleParamOverrides) {
				if (ov.cmd == cmd && (ov.vehicle_mask & vehicle_type)) {
					mask |= for_mission ? ov.mission : ov.command;
				}
			}

			return mask;

		} else if (SupportedCommandParams[mid].cmd < cmd) {
			lo = mid + 1;

		} else {
			if (mid == 0) { break; }

			hi = mid - 1;
		}
	}

	return -1;
}

// Shared by check_params_for_vehicle and check_params_int_for_vehicle so the scan
// logic exists once instead of being duplicated per param-count variant.
static int _scan_params(uint8_t mask, const float *ps, int count, uint8_t *zero_sentinel_mask)
{
	for (int i = 0; i < count; ++i) {
		if (!((mask >> i) & 1u)) {
			if (!param_is_unset(ps[i])) { return i + 1; }

			if (zero_sentinel_mask && param_is_zero(ps[i])) {
				*zero_sentinel_mask |= (uint8_t)(1u << i);
			}
		}
	}

	return 0;
}

/**
 * Check params 1–7 of an incoming mission item or command against the base table,
 * then OR-extend the allowed mask with any matching VehicleParamOverrides entry.
 *
 * @param vehicle_type  bitmask from vehicle_type_bitmask() — VEHICLE_FW / VEHICLE_MC / VEHICLE_VTOL
 * @return  0   all unsupported params are unset
 *          1–7 1-based index of the first offending param
 *          -1  command not in table (no validation applied)
 */
[[maybe_unused]] static int check_params_for_vehicle(uint16_t cmd, bool for_mission, uint8_t vehicle_type,
		float p1, float p2, float p3, float p4,
		float p5 = 0.0f, float p6 = 0.0f, float p7 = 0.0f,
		uint8_t *zero_sentinel_mask = nullptr)
{
	const int mask_result = _find_mask(cmd, for_mission, vehicle_type);

	if (mask_result < 0) { return -1; }

	const float ps[7] = {p1, p2, p3, p4, p5, p6, p7};
	return _scan_params((uint8_t)mask_result, ps, 7, zero_sentinel_mask);
}

/**
 * Variant of check_params_for_vehicle for global-frame MISSION_ITEM_INT where p5/p6
 * are raw int32 lat/lon (INT32_MAX = unset) and p7 (altitude) remains a float.
 */
[[maybe_unused]] static int check_params_int_for_vehicle(uint16_t cmd, bool for_mission, uint8_t vehicle_type,
		float p1, float p2, float p3, float p4,
		int32_t p5_int, int32_t p6_int, float p7 = 0.0f,
		uint8_t *zero_sentinel_mask = nullptr)
{
	const int mask_result = _find_mask(cmd, for_mission, vehicle_type);

	if (mask_result < 0) { return -1; }

	const uint8_t mask = (uint8_t)mask_result;
	const float ps14[4] = {p1, p2, p3, p4};

	const int bad = _scan_params(mask, ps14, 4, zero_sentinel_mask);

	if (bad != 0) { return bad; }

	if (!((mask >> 4) & 1u) && !int_param_is_unset(p5_int)) { return 5; }

	if (!((mask >> 5) & 1u) && !int_param_is_unset(p6_int)) { return 6; }

	if (!((mask >> 6) & 1u) && !param_is_unset(p7)) { return 7; }

	return 0;
}

} // namespace mavlink_cmd_params
