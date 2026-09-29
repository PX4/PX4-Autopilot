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

	// blend multiple receivers if available
	if (!blend_gnss_data(hrt_now_us)) {
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

		// Check for new data on selected GPS, and clear blend offsets
		for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
			_NE_pos_offset_m[i].zero();
			_hgt_offset_m[i] = 0.0;
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

		for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
			// clear updated flags
			_gps_updated[i] = false;
		}
	}

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
		_time_prev_us[i] = _gnss_state[i].timestamp;
	}
}

bool GpsBlending::blend_gnss_data(uint64_t hrt_now_us)
{
	/*
	 * If both receivers have the same update rate, use the oldest non-zero time.
	 * If two receivers with different update rates are used, use the slowest.
	 * If time difference is excessive, use newest to prevent a disconnected receiver
	 * from blocking updates.
	 */

	// Calculate the time step for each receiver with some filtering to reduce the effects of jitter
	// Find the largest and smallest time step.
	float dt_max = 0.0f;
	float dt_min = GPS_TIMEOUT_S;
	_np_gps_suitable_for_blending = 0;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {

		float raw_dt = 0.f;

		if (_gnss_state[i].timestamp > _time_prev_us[i]) {
			raw_dt = 1e-6f * (_gnss_state[i].timestamp - _time_prev_us[i]);
		}

		float present_dt = 0.f;

		if (hrt_now_us > _gnss_state[i].timestamp) {
			present_dt = 1e-6f * (hrt_now_us - _gnss_state[i].timestamp);
		}

		if (raw_dt > 0.0f && raw_dt < GPS_TIMEOUT_S) {
			_gps_dt[i] = 0.1f * raw_dt + 0.9f * _gps_dt[i];

			if (i == _primary_instance) {
				_primary_instance_available = true;
			}

		} else if ((present_dt >= GPS_TIMEOUT_S) && (_gnss_state[i].timestamp > 0)) {
			// Timed out - kill the stored fix for this receiver and don't track its (stale) gps_dt
			_gnss_state[i].timestamp = 0;
			_gnss_state[i].fix_type = 0;
			_gnss_state[i].satellites_used = 0;
			_gnss_state[i].vel_ned_valid = 0;

			if (i == _primary_instance) {
				// Allow using a secondary instance when the primary receiver has timed out
				_primary_instance_available = false;
			}

			continue;
		}

		// Only count GPSs with at least a 2D fix for blending purposes
		if (_gnss_state[i].fix_type < 2) {
			continue;
		}

		if (_gps_dt[i] > dt_max) {
			dt_max = _gps_dt[i];
			_gps_slowest_index = i;
		}

		if (_gps_dt[i] < dt_min) {
			dt_min = _gps_dt[i];
		}

		_np_gps_suitable_for_blending++;
	}

	// Find the receiver that is last be updated
	uint64_t max_us = 0; // newest non-zero system time of arrival of a GPS message
	uint64_t min_us = -1; // oldest non-zero system time of arrival of a GPS message

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
		// Find largest and smallest times
		if (_gnss_state[i].timestamp > max_us) {
			max_us = _gnss_state[i].timestamp;
			_gps_newest_index = i;
		}

		if ((_gnss_state[i].timestamp < min_us) && (_gnss_state[i].timestamp > 0)) {
			min_us = _gnss_state[i].timestamp;
		}
	}

	if (_np_gps_suitable_for_blending < 2) {
		// Less than 2 receivers left, so fall out of blending
		return false;
	}

	/*
	 * If the largest dt is less than 20% greater than the smallest, then we have  receivers
	 * running at the same rate then we wait until we have two messages with an arrival time
	 * difference that is less than 50% of the smallest time step and use the time stamp from
	 * the newest data.
	 * Else we have two receivers at different update rates and use the slowest receiver
	 * as the timing reference.
	 */
	bool gps_new_output_data = false;

	if ((dt_max - dt_min) < 0.2f * dt_min) {
		// both receivers assumed to be running at the same rate
		if ((max_us - min_us) < (uint64_t)(5e5f * dt_min)) {
			// data arrival within a short time window enables the two measurements to be blended
			_gps_time_ref_index = _gps_newest_index;
			gps_new_output_data = true;
		}

	} else {
		// both receivers running at different rates
		_gps_time_ref_index = _gps_slowest_index;

		if (_gnss_state[_gps_time_ref_index].timestamp > _time_prev_us[_gps_time_ref_index]) {
			// blend data at the rate of the slower receiver
			gps_new_output_data = true;
		}
	}

	if (gps_new_output_data) {
		// calculate the sum squared speed accuracy across all GPS sensors
		float speed_accuracy_sum_sq = 0.0f;

		if (_blend_use_spd_acc) {
			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
				if (_gnss_state[i].fix_type >= 3 && _gnss_state[i].speed_accuracy > 0.0f) {
					speed_accuracy_sum_sq += _gnss_state[i].speed_accuracy * _gnss_state[i].speed_accuracy;
				}
			}
		}

		// calculate the sum squared horizontal position accuracy across all GPS sensors
		float horizontal_accuracy_sum_sq = 0.0f;

		if (_blend_use_hpos_acc) {
			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
				if (_gnss_state[i].fix_type >= 2 && _gnss_state[i].eph > 0.0f) {
					horizontal_accuracy_sum_sq += _gnss_state[i].eph * _gnss_state[i].eph;
				}
			}
		}

		// calculate the sum squared vertical position accuracy across all GPS sensors
		float vertical_accuracy_sum_sq = 0.0f;

		if (_blend_use_vpos_acc) {
			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
				if (_gnss_state[i].fix_type >= 3 && _gnss_state[i].epv > 0.0f) {
					vertical_accuracy_sum_sq += _gnss_state[i].epv * _gnss_state[i].epv;
				}
			}
		}

		// Check if we can do blending using reported accuracy
		bool can_do_blending = (horizontal_accuracy_sum_sq > 0.0f || vertical_accuracy_sum_sq > 0.0f
					|| speed_accuracy_sum_sq > 0.0f);

		// if we can't do blending using reported accuracy, return false and hard switch logic will be used instead
		if (!can_do_blending) {
			return false;
		}

		float sum_of_all_weights = 0.0f;

		// calculate a weighting using the reported speed accuracy
		float spd_blend_weights[GPS_MAX_RECEIVERS_BLEND] {};

		if (speed_accuracy_sum_sq > 0.0f && _blend_use_spd_acc) {
			// calculate the weights using the inverse of the variances
			float sum_of_spd_weights = 0.0f;

			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
				if (_gnss_state[i].fix_type >= 3 && _gnss_state[i].speed_accuracy >= 0.001f) {
					spd_blend_weights[i] = 1.0f / (_gnss_state[i].speed_accuracy * _gnss_state[i].speed_accuracy);
					sum_of_spd_weights += spd_blend_weights[i];
				}
			}

			// normalise the weights
			if (sum_of_spd_weights > 0.0f) {
				for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
					spd_blend_weights[i] = spd_blend_weights[i] / sum_of_spd_weights;
				}

				sum_of_all_weights += 1.0f;
			}
		}

		// calculate a weighting using the reported horizontal position
		float hpos_blend_weights[GPS_MAX_RECEIVERS_BLEND] {};

		if (horizontal_accuracy_sum_sq > 0.0f && _blend_use_hpos_acc) {
			// calculate the weights using the inverse of the variances
			float sum_of_hpos_weights = 0.0f;

			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
				if (_gnss_state[i].fix_type >= 2 && _gnss_state[i].eph >= 0.001f) {
					hpos_blend_weights[i] = horizontal_accuracy_sum_sq / (_gnss_state[i].eph * _gnss_state[i].eph);
					sum_of_hpos_weights += hpos_blend_weights[i];
				}
			}

			// normalise the weights
			if (sum_of_hpos_weights > 0.0f) {
				for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
					hpos_blend_weights[i] = hpos_blend_weights[i] / sum_of_hpos_weights;
				}

				sum_of_all_weights += 1.0f;
			}
		}

		// calculate a weighting using the reported vertical position accuracy
		float vpos_blend_weights[GPS_MAX_RECEIVERS_BLEND] = {};

		if (vertical_accuracy_sum_sq > 0.0f && _blend_use_vpos_acc) {
			// calculate the weights using the inverse of the variances
			float sum_of_vpos_weights = 0.0f;

			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
				if (_gnss_state[i].fix_type >= 3 && _gnss_state[i].epv >= 0.001f) {
					vpos_blend_weights[i] = vertical_accuracy_sum_sq / (_gnss_state[i].epv * _gnss_state[i].epv);
					sum_of_vpos_weights += vpos_blend_weights[i];
				}
			}

			// normalise the weights
			if (sum_of_vpos_weights > 0.0f) {
				for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
					vpos_blend_weights[i] = vpos_blend_weights[i] / sum_of_vpos_weights;
				}

				sum_of_all_weights += 1.0f;
			};
		}

		// blend weight for each GPS. The blend weights must sum to 1.0 across all instances.
		float blend_weights[GPS_MAX_RECEIVERS_BLEND];

		// calculate an overall weight
		for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
			blend_weights[i] = (hpos_blend_weights[i] + vpos_blend_weights[i] + spd_blend_weights[i]) / sum_of_all_weights;
		}

		// With updated weights we can calculate a blended GPS solution and
		// offsets for each physical receiver
		sensor_gnss_s gnss_blended_state = gnss_blend_states(blend_weights);

		// blend antenna offsets using the same weights
		_output_antenna_offset.zero();

		for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
			if (blend_weights[i] > 0.0f) {
				_output_antenna_offset += _antenna_offset[i] * blend_weights[i];
			}
		}

		update_gnss_offsets(gnss_blended_state);

		// calculate a blended output from the offset corrected receiver data
		// publish if blending was successful
		calc_gnss_blend_output(gnss_blended_state, blend_weights);

		_gnss_blended_state = gnss_blended_state;
		_selected_gps = GPS_MAX_RECEIVERS_BLEND;
		_is_new_output_data_available = true;
	}

	return true;
}

sensor_gnss_s GpsBlending::gnss_blend_states(float blend_weights[GPS_MAX_RECEIVERS_BLEND]) const
{
	// Use the GPS with the highest weighting as the reference position
	float best_weight = 0.0f;

	// index of the physical receiver with the lowest reported error
	uint8_t gps_best_index = 0;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
		if (blend_weights[i] > best_weight) {
			best_weight = blend_weights[i];
			gps_best_index = i;
		}
	}

	// initialise the blended states so we can accumulate the results using the weightings for each GPS receiver.
	sensor_gnss_s gnss_blended_state{_gnss_state[gps_best_index]}; // start with best GPS for all other misc fields

	// zerp all fields that are an accumulated blend below
	gnss_blended_state.timestamp = 0;
	gnss_blended_state.timestamp_sample = 0;
	gnss_blended_state.ground_speed = 0;
	gnss_blended_state.vel_north = 0;
	gnss_blended_state.vel_east = 0;
	gnss_blended_state.vel_down = 0;

	// Accumulate in double and round once, truncating each weighted term biases the result low.
	double blended_timestamp_us = 0.0;
	double blended_timestamp_sample_us = 0.0;
	double sum_of_timing_weights = 0.0;

	// combine the the GPS states into a blended solution using the weights calculated in calc_blend_weights()
	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
		// Assume blended error magnitude, DOP and sat count is equal to the best value from contributing receivers
		// If any receiver contributing has an invalid velocity, then report blended velocity as invalid
		if (blend_weights[i] > 0.0f) {

			// blend the timing data
			const double timing_weight = (double)blend_weights[i];
			blended_timestamp_us += (double)_gnss_state[i].timestamp * timing_weight;
			blended_timestamp_sample_us += (double)_gnss_state[i].timestamp_sample * timing_weight;
			sum_of_timing_weights += timing_weight;

			// calculate a blended average speed and velocity vector
			gnss_blended_state.ground_speed += _gnss_state[i].ground_speed * blend_weights[i];
			gnss_blended_state.vel_north += _gnss_state[i].vel_north * blend_weights[i];
			gnss_blended_state.vel_east += _gnss_state[i].vel_east * blend_weights[i];
			gnss_blended_state.vel_down += _gnss_state[i].vel_down * blend_weights[i];


			// use the lowest value
			if (_gnss_state[i].eph > 0.0f
			    && _gnss_state[i].eph < gnss_blended_state.eph) {
				gnss_blended_state.eph = _gnss_state[i].eph;
			}

			if (_gnss_state[i].epv > 0.0f
			    && _gnss_state[i].epv < gnss_blended_state.epv) {
				gnss_blended_state.epv = _gnss_state[i].epv;
			}

			if (_gnss_state[i].speed_accuracy > 0.0f
			    && _gnss_state[i].speed_accuracy < gnss_blended_state.speed_accuracy) {
				gnss_blended_state.speed_accuracy = _gnss_state[i].speed_accuracy;
			}

			if (_gnss_state[i].hdop > 0
			    && _gnss_state[i].hdop < gnss_blended_state.hdop) {
				gnss_blended_state.hdop = _gnss_state[i].hdop;
			}

			if (_gnss_state[i].vdop > 0
			    && _gnss_state[i].vdop < gnss_blended_state.vdop) {
				gnss_blended_state.vdop = _gnss_state[i].vdop;
			}


			// use the highest status
			if (_gnss_state[i].fix_type > gnss_blended_state.fix_type) {
				gnss_blended_state.fix_type = _gnss_state[i].fix_type;
			}

			if (_gnss_state[i].satellites_used > gnss_blended_state.satellites_used) {
				gnss_blended_state.satellites_used = _gnss_state[i].satellites_used;
			}

			if (_gnss_state[i].vel_ned_valid) {
				gnss_blended_state.vel_ned_valid = true;
			}
		}

	}

	if (sum_of_timing_weights > 0.0) {
		gnss_blended_state.timestamp = (uint64_t)llround(blended_timestamp_us / sum_of_timing_weights);
		gnss_blended_state.timestamp_sample = (uint64_t)llround(blended_timestamp_sample_us / sum_of_timing_weights);
	}

	/*
	 * Calculate the instantaneous weighted average location using  available GPS instances and store in  _gnss_state.
	 * This is statistically the most likely location, but may not be stable enough for direct use by the EKF.
	*/

	// Convert each GPS position to a local NEU offset relative to the reference position
	Vector2f blended_NE_offset_m{0, 0};
	double blended_alt_offset_m = 0.0;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
		if ((blend_weights[i] > 0.0f) && (i != gps_best_index)) {
			// calculate the horizontal offset
			Vector2f horiz_offset{};
			get_vector_to_next_waypoint(gnss_blended_state.latitude, gnss_blended_state.longitude,
						    _gnss_state[i].latitude, _gnss_state[i].longitude,
						    &horiz_offset(0), &horiz_offset(1));

			// sum weighted offsets
			blended_NE_offset_m += horiz_offset * blend_weights[i];

			// calculate vertical offset, meters
			double vert_offset_m = _gnss_state[i].altitude_msl - gnss_blended_state.altitude_msl;

			// sum weighted offsets
			blended_alt_offset_m += vert_offset_m * (double)blend_weights[i];
		}
	}

	// Add the sum of weighted offsets to the reference position to obtain the blended position
	const double lat_deg_now = gnss_blended_state.latitude;
	const double lon_deg_now = gnss_blended_state.longitude;
	double lat_deg_res = 0;
	double lon_deg_res = 0;
	add_vector_to_global_position(lat_deg_now, lon_deg_now,
				      blended_NE_offset_m(0), blended_NE_offset_m(1),
				      &lat_deg_res, &lon_deg_res);
	gnss_blended_state.latitude = lat_deg_res;
	gnss_blended_state.longitude = lon_deg_res;
	gnss_blended_state.altitude_msl += blended_alt_offset_m;

	// Blend UTC timestamp from all receivers that are publishing a valid time_utc_usec value
	double utc_weight_sum = 0.0;
	double utc_time_sum = 0.0;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
		if (_gnss_state[i].time_utc_usec > 0) {
			utc_time_sum += (double)_gnss_state[i].time_utc_usec * (double)blend_weights[i];
			utc_weight_sum += (double)blend_weights[i];
		}
	}

	if (utc_weight_sum > 0.0) {
		gnss_blended_state.time_utc_usec = (uint64_t)(utc_time_sum / utc_weight_sum);
	}

	return gnss_blended_state;
}

void GpsBlending::update_gnss_offsets(const sensor_gnss_s &gnss_blended_state)
{
	// Calculate filter coefficients to be applied to the offsets for each GPS position and height offset
	// A weighting of 1 will make the offset adjust the slowest, a weighting of 0 will make it adjust with zero filtering
	float alpha[GPS_MAX_RECEIVERS_BLEND] {};
	float omega_lpf = 1.0f / fmaxf(_blending_time_constant, 1.0f);

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
		if (_gnss_state[i].timestamp > _time_prev_us[i]) {
			// calculate the filter coefficient that achieves the time constant specified by the user adjustable parameter
			alpha[i] = constrain(omega_lpf * 1e-6f * (float)(_gnss_state[i].timestamp - _time_prev_us[i]),
					     0.0f, 1.0f);
		}
	}

	// Calculate a filtered position delta for each GPS relative to the blended solution state
	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
		Vector2f offset;
		get_vector_to_next_waypoint(_gnss_state[i].latitude, _gnss_state[i].longitude,
					    gnss_blended_state.latitude, gnss_blended_state.longitude,
					    &offset(0), &offset(1));

		_NE_pos_offset_m[i] = offset * alpha[i] + _NE_pos_offset_m[i] * (1.0f - alpha[i]);

		_hgt_offset_m[i] = (gnss_blended_state.altitude_msl - _gnss_state[i].altitude_msl) * (double)alpha[i] +
				   _hgt_offset_m[i] * (1.0 - (double)alpha[i]);
	}

	// calculate offset limits from the largest difference between receivers
	Vector2f max_ne_offset{};
	double max_alt_offset = 0.0;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
		for (uint8_t j = i; j < GPS_MAX_RECEIVERS_BLEND; j++) {
			if (i != j) {
				Vector2f offset;
				get_vector_to_next_waypoint(_gnss_state[i].latitude, _gnss_state[i].longitude,
							    _gnss_state[j].latitude, _gnss_state[j].longitude,
							    &offset(0), &offset(1));
				max_ne_offset(0) = fmax(max_ne_offset(0), fabsf(offset(0)));
				max_ne_offset(1) = fmax(max_ne_offset(1), fabsf(offset(1)));
				max_alt_offset = fmax(max_alt_offset, fabs(_gnss_state[i].altitude_msl - _gnss_state[j].altitude_msl));
			}
		}
	}

	// apply offset limits
	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
		_NE_pos_offset_m[i](0) = constrain(_NE_pos_offset_m[i](0), -max_ne_offset(0), max_ne_offset(0));
		_NE_pos_offset_m[i](1) = constrain(_NE_pos_offset_m[i](1), -max_ne_offset(1), max_ne_offset(1));
		_hgt_offset_m[i] = constrain(_hgt_offset_m[i], -max_alt_offset, max_alt_offset);
	}
}

void GpsBlending::calc_gnss_blend_output(sensor_gnss_s &gnss_blended_state,
		float blend_weights[GPS_MAX_RECEIVERS_BLEND]) const
{
	// Convert each GPS position to a local NEU offset relative to the reference position
	// which is defined as the positon of the blended solution calculated from non offset corrected data
	Vector2f blended_NE_offset_m{0, 0};
	double blended_alt_offset_m = 0.0;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
		if (blend_weights[i] > 0.0f) {

			// Add the sum of weighted offsets to the reference position to obtain the blended position
			const double lat_deg_orig = _gnss_state[i].latitude;
			const double lon_deg_orig = _gnss_state[i].longitude;
			double lat_deg_offset_res = 0;
			double lon_deg_offset_res = 0;
			add_vector_to_global_position(lat_deg_orig, lon_deg_orig,
						      _NE_pos_offset_m[i](0), _NE_pos_offset_m[i](1),
						      &lat_deg_offset_res, &lon_deg_offset_res);

			double alt_offset_m = _gnss_state[i].altitude_msl + _hgt_offset_m[i];


			// calculate the horizontal offset
			Vector2f horiz_offset{};
			get_vector_to_next_waypoint(gnss_blended_state.latitude, gnss_blended_state.longitude,
						    lat_deg_offset_res, lon_deg_offset_res,
						    &horiz_offset(0), &horiz_offset(1));

			// sum weighted offsets
			blended_NE_offset_m += horiz_offset * blend_weights[i];

			// calculate vertical offset
			double vert_offset_m = alt_offset_m - gnss_blended_state.altitude_msl;

			// sum weighted offsets
			blended_alt_offset_m += vert_offset_m * (double)blend_weights[i];
		}
	}

	// Add the sum of weighted offsets to the reference position to obtain the blended position
	const double lat_deg_now = gnss_blended_state.latitude;
	const double lon_deg_now = gnss_blended_state.longitude;
	double lat_deg_res = 0;
	double lon_deg_res = 0;
	add_vector_to_global_position(lat_deg_now, lon_deg_now,
				      blended_NE_offset_m(0), blended_NE_offset_m(1),
				      &lat_deg_res, &lon_deg_res);

	gnss_blended_state.latitude = lat_deg_res;
	gnss_blended_state.longitude = lon_deg_res;
	gnss_blended_state.altitude_msl = gnss_blended_state.altitude_msl + blended_alt_offset_m;
}
