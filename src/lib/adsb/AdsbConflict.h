/****************************************************************************
 *
 *   Copyright (C) 2012-2023 PX4 Development Team. All rights reserved.
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



#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <lib/geo/geo.h>

#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/transponder_report.h>
#include <uORB/topics/vehicle_command.h>

#include <systemlib/mavlink_log.h>
#include <px4_platform_common/events.h>

#include <px4_platform_common/board_common.h>

#include <containers/Array.hpp>

using namespace time_literals;

static constexpr uint8_t NAVIGATOR_MAX_TRAFFIC{10};

static constexpr uint8_t UTM_GUID_MSG_LENGTH{11};

static constexpr uint8_t UTM_CALLSIGN_LENGTH{9};

static constexpr uint64_t CONFLICT_WARNING_TIMEOUT{60_s};

static constexpr float TRAFFIC_TO_UAV_DISTANCE_EXTENSION{1000.0f};


struct traffic_data_s {
	double lat_traffic;
	double lon_traffic;
	float alt_traffic;
	float heading_traffic;
	float vxy_traffic;
	float vz_traffic;
	bool in_conflict;
};

struct traffic_buffer_s {
	px4::Array<uint32_t, NAVIGATOR_MAX_TRAFFIC> icao_address {};
	px4::Array<hrt_abstime, NAVIGATOR_MAX_TRAFFIC> timestamp {};
};

struct conflict_detection_params_s {
	float crosstrack_separation;
	float vertical_separation;
	int collision_time_threshold;
	uint8_t traffic_avoidance_mode;
};

enum class TRAFFIC_STATE {
	NO_CONFLICT = 0,
	ADD_CONFLICT = 1,
	REMIND_CONFLICT = 2,
	REMOVE_OLD_CONFLICT = 3,
	BUFFER_FULL = 4
};


class AdsbConflict
{
public:
	AdsbConflict() = default;
	~AdsbConflict() = default;

	void detect_traffic_conflict(double lat_now, double lon_now, float alt_now, float vx_now, float vy_now, float vz_now);

	int find_icao_address_in_conflict_list(uint32_t icao_address);

	void remove_icao_address_from_conflict_list(int traffic_index);

	void add_icao_address_from_conflict_list(uint32_t icao_address);

	void get_traffic_state();

	void set_conflict_detection_params(float crosstrack_separation, float vertical_separation,
					   int collision_time_threshold, uint8_t traffic_avoidance_mode);


	bool send_traffic_warning(int traffic_direction, int traffic_seperation, uint16_t tr_flags,
				  char uas_id[UTM_GUID_MSG_LENGTH], char tr_callsign[UTM_CALLSIGN_LENGTH], uint64_t uas_id_int);

	transponder_report_s _transponder_report{};

	bool handle_traffic_conflict();

	void fake_traffic(const char *const callsign, float distance, float direction, float traffic_heading,
			  float altitude_diff,
			  float hor_velocity, float ver_velocity, int emitter_type, uint32_t icao_address, double lat_uav, double lon_uav,
			  float &alt_uav);

	void run_fake_traffic(double &lat_uav, double &lon_uav,
			      float &alt_uav);

	bool _conflict_detected{false};

	TRAFFIC_STATE _traffic_state{TRAFFIC_STATE::NO_CONFLICT};

	conflict_detection_params_s _conflict_detection_params{};


protected:
	traffic_buffer_s _traffic_buffer;

private:

	orb_advert_t _mavlink_log_pub{nullptr};

	crosstrack_error_s _crosstrack_error{};


	transponder_report_s tr{};

	orb_advert_t fake_traffic_report_publisher = orb_advertise_queue(ORB_ID(transponder_report), &tr, (unsigned int)20);

	TRAFFIC_STATE _traffic_state_previous{TRAFFIC_STATE::NO_CONFLICT};

};
