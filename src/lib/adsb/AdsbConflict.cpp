/****************************************************************************
 *
 *   Copyright (C) 2023 PX4 Development Team. All rights reserved.
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




#include "AdsbConflict.h"
#include "geo/geo.h"

#include <uORB/topics/transponder_report.h>

#include <float.h>


void AdsbConflict::detect_traffic_conflict(double lat_now, double lon_now, float alt_now, float vx_now, float vy_now,
		float vz_now)

{

	float d_hor, d_vert;
	get_distance_to_point_global_wgs84(lat_now, lon_now, alt_now,
					   _transponder_report.lat, _transponder_report.lon, _transponder_report.altitude, &d_hor, &d_vert);

	const float xyz_traffic_speed = sqrtf(_transponder_report.hor_velocity * _transponder_report.hor_velocity +
					      _transponder_report.ver_velocity * _transponder_report.ver_velocity);

	const float hor_uav_speed = sqrtf(vx_now * vx_now + vy_now * vy_now);
	const float xyz_uav_speed = sqrtf(hor_uav_speed * hor_uav_speed + vz_now * vz_now);

	//assume always pointing at each other
	const float relative_uav_traffic_speed = xyz_traffic_speed + xyz_uav_speed;


	// Predict until the vehicle would have passed this system at its current speed
	const float prediction_distance = d_hor + TRAFFIC_TO_UAV_DISTANCE_EXTENSION;

	double end_lat, end_lon;
	waypoint_from_heading_and_distance(_transponder_report.lat, _transponder_report.lon,
					   _transponder_report.heading, prediction_distance, &end_lat, &end_lon);


	const bool cs_distance_conflict_threshold = (!get_distance_to_line(_crosstrack_error, lat_now,
			lon_now, _transponder_report.lat,
			_transponder_report.lon, end_lat,
			end_lon))
			&& (!_crosstrack_error.past_end)
			&& (fabsf(_crosstrack_error.distance) < _conflict_detection_params.crosstrack_separation);

	const bool _crosstrack_separation_check = (fabsf(alt_now - _transponder_report.altitude) <
			_conflict_detection_params.vertical_separation);

	bool collision_time_check = false;

	const float d_xyz = sqrtf(d_hor * d_hor + d_vert * d_vert);

	if (relative_uav_traffic_speed > FLT_EPSILON) {
		const float time_to_collsion =  d_xyz / relative_uav_traffic_speed;
		collision_time_check = (time_to_collsion < _conflict_detection_params.collision_time_threshold);
	}

	_conflict_detected = (cs_distance_conflict_threshold && _crosstrack_separation_check
			      && collision_time_check);
}

int AdsbConflict::find_icao_address_in_conflict_list(uint32_t icao_address)
{

	for (size_t i = 0; i < _traffic_buffer.icao_address.size(); i++) {
		if (_traffic_buffer.icao_address[i] == icao_address) {
			return static_cast<int>(i);
		}
	}

	return -1;
}

void AdsbConflict::remove_icao_address_from_conflict_list(int traffic_index)
{
	_traffic_buffer.icao_address.remove(traffic_index);
	_traffic_buffer.timestamp.remove(traffic_index);
	PX4_INFO("icao_address removed. Buffer Size: %d", (int)_traffic_buffer.timestamp.size());
}

void AdsbConflict::add_icao_address_from_conflict_list(uint32_t icao_address, hrt_abstime now)
{
	_traffic_buffer.timestamp.push_back(now);
	_traffic_buffer.icao_address.push_back(icao_address);
	PX4_INFO("icao_address added. Buffer Size: %d", (int)_traffic_buffer.timestamp.size());
}

void AdsbConflict::get_traffic_state(hrt_abstime now)
{
	const int traffic_index = find_icao_address_in_conflict_list(_transponder_report.icao_address);

	const bool old_conflict = (traffic_index >= 0);
	const bool new_traffic = (traffic_index < 0);
	const bool _traffic_buffer_full = (_traffic_buffer.icao_address.size() >= NAVIGATOR_MAX_TRAFFIC);

	bool old_conflict_warning_expired = false;

	if (old_conflict && _conflict_detected) {
		old_conflict_warning_expired = now > _traffic_buffer.timestamp[traffic_index] + CONFLICT_WARNING_TIMEOUT;
	}

	if (new_traffic && _conflict_detected && !_traffic_buffer_full) {
		add_icao_address_from_conflict_list(_transponder_report.icao_address, now);
		_traffic_state = TRAFFIC_STATE::ADD_CONFLICT;

	} else if (new_traffic && _conflict_detected && _traffic_buffer_full) {
		_traffic_state = TRAFFIC_STATE::BUFFER_FULL;

	} else if (old_conflict && _conflict_detected
		   && old_conflict_warning_expired) {
		_traffic_buffer.timestamp[traffic_index] = now;
		_traffic_state = TRAFFIC_STATE::REMIND_CONFLICT;

	} else if (old_conflict && !_conflict_detected) {
		remove_icao_address_from_conflict_list(traffic_index);
		_traffic_state = TRAFFIC_STATE::REMOVE_OLD_CONFLICT;

	} else {
		_traffic_state = TRAFFIC_STATE::NO_CONFLICT;
	}
}

void AdsbConflict::remove_expired_conflicts()
{
	for (uint8_t traffic_index = 0; traffic_index < _traffic_buffer.timestamp.size();) {
		if (hrt_elapsed_time(&_traffic_buffer.timestamp[traffic_index]) > TRAFFIC_CONFLICT_LIFETIME) {
			events::send<uint32_t>(events::ID("navigator_traffic_expired"), events::Log::Notice,
					       "Traffic Conflict {1} Expired and removed from buffer",
					       _traffic_buffer.icao_address[traffic_index]);
			remove_icao_address_from_conflict_list(traffic_index);

		} else {
			traffic_index++;
		}
	}
}

bool AdsbConflict::handle_traffic_conflict()
{
	const hrt_abstime now = hrt_absolute_time();

	get_traffic_state(now);

	bool take_action = false;

	switch (_traffic_state) {

	case TRAFFIC_STATE::ADD_CONFLICT:
	case TRAFFIC_STATE::REMIND_CONFLICT: {
			take_action = send_traffic_warning((int)math::degrees(_transponder_report.heading),
							   (int)fabsf(_crosstrack_error.distance), _transponder_report.flags,
							   _transponder_report.callsign,
							   _transponder_report.icao_address,
							   now);
		}
		break;

	case TRAFFIC_STATE::REMOVE_OLD_CONFLICT: {
			events::send<uint32_t>(events::ID("navigator_traffic_resolved"), events::Log::Notice,
					       "Traffic Conflict Resolved {1}!",
					       _transponder_report.icao_address);
			_last_traffic_warning_time = now;
		}
		break;

	case TRAFFIC_STATE::BUFFER_FULL: {

			if ((_traffic_state_previous != TRAFFIC_STATE::BUFFER_FULL)
			    && (hrt_elapsed_time(&_last_buffer_full_warning_time) > TRAFFIC_WARNING_TIMESTEP)) {
				events::send(events::ID("buffer_full"), events::Log::Notice, "Too much traffic! Showing all messages from now on");
				_last_buffer_full_warning_time = now;
			}

			//disable conflict warnings when buffer is full
		}
		break;

	case TRAFFIC_STATE::NO_CONFLICT: {

		}
		break;
	}


	_traffic_state_previous = _traffic_state;

	return take_action;

}

void AdsbConflict::set_conflict_detection_params(float crosstrack_separation, float vertical_separation,
		int collision_time_threshold, uint8_t traffic_avoidance_mode)
{

	_conflict_detection_params.crosstrack_separation = crosstrack_separation;
	_conflict_detection_params.vertical_separation = vertical_separation;
	_conflict_detection_params.collision_time_threshold = collision_time_threshold;
	_conflict_detection_params.traffic_avoidance_mode = traffic_avoidance_mode;

}


bool AdsbConflict::send_traffic_warning(int traffic_direction, int traffic_seperation, uint16_t tr_flags,
					char tr_callsign[UTM_CALLSIGN_LENGTH], uint32_t icao_address, hrt_abstime now)
{

	switch (_conflict_detection_params.traffic_avoidance_mode) {

	case 0: {

			if (tr_flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN) {

				PX4_WARN("Traffic alert - UTM callsign %s! Separation Distance %d, Heading %d, ICAO Address %d",
					 tr_callsign,
					 traffic_seperation,
					 traffic_direction, (int)icao_address);

			}


			_last_traffic_warning_time = now;

			break;
		}

	case 1: {


			if (tr_flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN) {

				PX4_WARN("Traffic alert - UTM callsign %s! Separation Distance %d, Heading %d, ICAO Address %d",
					 tr_callsign,
					 traffic_seperation,
					 traffic_direction, (int)icao_address);

			}

			/* EVENT
			 * @description
			 * - ICAO Address: {1}
			 * - Traffic Separation Distance: {2m}
			 * - Heading: {3} degrees
			 */
			events::send<uint32_t, int32_t, int16_t>(events::ID("navigator_traffic"), events::Log::Notice,
					"Traffic alert - ICAO Address {1}! Separation Distance {2}, Heading {3}",
					icao_address, traffic_seperation, traffic_direction);

			_last_traffic_warning_time = now;

			break;
		}

	case 2: {
			/* EVENT
			 * @description
			 * - ICAO Address: {1}
			 * - Traffic Separation Distance: {2m}
			 * - Heading: {3} degrees
			 */
			events::send<uint32_t, int32_t, int16_t>(events::ID("navigator_traffic_rtl"), events::Log::Notice,
					"Traffic alert - ICAO Address {1}! Separation Distance {2}, Heading {3}, returning home",
					icao_address, traffic_seperation, traffic_direction);

			_last_traffic_warning_time = now;

			return true;

			break;
		}

	case 3: {
			/* EVENT
			 * @description
			 * - ICAO Address: {1}
			 * - Traffic Separation Distance: {2m}
			 * - Heading: {3} degrees
			 */
			events::send<uint32_t, int32_t, int16_t>(events::ID("navigator_traffic_land"), events::Log::Notice,
					"Traffic alert - ICAO Address {1}! Separation Distance {2}, Heading {3}, landing",
					icao_address, traffic_seperation, traffic_direction);

			_last_traffic_warning_time = now;

			return true;

			break;

		}

	case 4: {
			/* EVENT
			 * @description
			 * - ICAO Address: {1}
			 * - Traffic Separation Distance: {2m}
			 * - Heading: {3} degrees
			 */
			events::send<uint32_t, int32_t, int16_t>(events::ID("navigator_traffic_hold"), events::Log::Notice,
					"Traffic alert - ICAO Address {1}! Separation Distance {2}, Heading {3}, holding position",
					icao_address, traffic_seperation, traffic_direction);

			_last_traffic_warning_time = now;


			return true;

			break;

		}
	}


	return false;

}
