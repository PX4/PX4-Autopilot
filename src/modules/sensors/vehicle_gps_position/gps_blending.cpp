/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file gps_blending.cpp
 */

#include "gps_blending.hpp"


void GpsBlending::update(uint64_t hrt_now_us)
{
	_is_new_output_data_available = false;

	updateReceiverTimeouts(hrt_now_us);

	// Only use selected receiver data if it has been updated
	uint8_t gps_select_index = 0;

	// Find the single "best" GPS from the data we have
	// First, find the GPS(s) with the best fix
	uint8_t best_fix = 0;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
		if (_gnss_state[i].fix_type > best_fix) {
			best_fix = _gnss_state[i].fix_type;
		}
	}

	// Second, compare GPS's with best fix and take the one with most satellites
	uint8_t max_sats = 0;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
		if (_gnss_state[i].fix_type == best_fix && _gnss_state[i].satellites_used > max_sats) {
			max_sats = _gnss_state[i].satellites_used;
			gps_select_index = i;
		}
	}

	// Only use a secondary instance if the fallback is allowed
	if ((_primary_instance > -1)
	    && (gps_select_index != _primary_instance)
	    && _primary_instance_available
	    && (_gnss_state[_primary_instance].fix_type >= 3)) {
		gps_select_index = _primary_instance;
	}

	_selected_gps = gps_select_index;
	_output_antenna_offset = _antenna_offset[gps_select_index];
	_is_new_output_data_available =  _gps_updated[gps_select_index];

	if (_is_new_output_data_available) {
		if ((_output_instance >= 0) && (_output_instance != _selected_gps)) {
			_selection_count++;
		}

		_output_instance = _selected_gps;
	}

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
		// clear updated flags
		_gps_updated[i] = false;
		_time_prev_us[i] = _gnss_state[i].timestamp;
	}
}

void GpsBlending::updateReceiverTimeouts(uint64_t hrt_now_us)
{
	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
		const uint64_t timestamp = _gnss_state[i].timestamp;

		if ((timestamp > _time_prev_us[i]) && (timestamp - _time_prev_us[i] < GPS_TIMEOUT_US)) {
			if (i == _primary_instance) {
				_primary_instance_available = true;
			}

		} else if ((timestamp > 0) && (hrt_now_us >= timestamp + GPS_TIMEOUT_US)) {
			// Timed out - kill the stored fix for this receiver
			_gnss_state[i].timestamp = 0;
			_gnss_state[i].fix_type = 0;
			_gnss_state[i].satellites_used = 0;
			_gnss_state[i].vel_ned_valid = 0;

			if (i == _primary_instance) {
				// Allow using a secondary instance when the primary receiver has timed out
				_primary_instance_available = false;
			}
		}
	}
}
