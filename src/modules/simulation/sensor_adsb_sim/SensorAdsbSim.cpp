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

#include "SensorAdsbSim.hpp"

#include <mathlib/mathlib.h>

#include <cstdio>
#include <cstdlib>
#include <cmath>

ModuleBase::Descriptor SensorAdsbSim::desc{task_spawn, custom_command, print_usage};

SensorAdsbSim::SensorAdsbSim() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	srand((unsigned)hrt_absolute_time()); // seed with time so that each run produces different random traffic patterns
}

SensorAdsbSim::~SensorAdsbSim()
{
	perf_free(_loop_perf);
}

bool SensorAdsbSim::init()
{
	ScheduleOnInterval(1_s); // 1 Hz matches real ADS-B transponder squitter rate
	return true;
}

float SensorAdsbSim::random_float(float min, float max)
{
	return min + (max - min) * (float)rand() / (float)RAND_MAX;
}

void SensorAdsbSim::spawn_vehicle(uint8_t index, double lat_ref, double lon_ref)
{
	SimulatedVehicle &v = _vehicles[index];

	const float radius  = _param_radius.get();
	const float bearing = random_float(0.f, 2.f * M_PI_F);
	const float distance = random_float(radius * 0.5f, radius);

	waypoint_from_heading_and_distance(lat_ref, lon_ref, bearing, distance, &v.lat, &v.lon);
	v.alt = _param_alt.get();

	const float speed   = random_float(MIN_SPEED_M_S, MAX_SPEED_M_S);
	const float heading = random_float(0.f, 2.f * M_PI_F);
	v.vel_n = speed * cosf(heading);
	v.vel_e = speed * sinf(heading);
	v.vel_u = random_float(-MAX_VERTICAL_SPEED_M_S, MAX_VERTICAL_SPEED_M_S);

	// Stable ICAO per slot so the conflict buffer doesn't accumulate stale entries on respawn
	v.icao_address = ICAO_BASE + index;

	v.emitter_type = EMITTER_TYPES[rand() % sizeof(EMITTER_TYPES)];

	v.initialized = true;
}

void SensorAdsbSim::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_loop_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	vehicle_global_position_s gpos{};

	if (!_vehicle_global_position_sub.copy(&gpos) || gpos.timestamp == 0) {
		perf_end(_loop_perf);
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain((now - _last_update) * 1e-6f, 0.001f, 2.0f);
	_last_update = now;

	const int count  = math::min((int)_param_count.get(), (int)MAX_VEHICLES);
	const float radius = _param_radius.get();

	for (int i = 0; i < count; i++) {
		SimulatedVehicle &v = _vehicles[i];

		if (!v.initialized) {
			spawn_vehicle(i, gpos.lat, gpos.lon);
		}

		// Respawn if the aircraft has flown out of the radius
		float dist_h, dist_v;
		get_distance_to_point_global_wgs84(gpos.lat, gpos.lon, gpos.alt,
						   v.lat, v.lon, v.alt, &dist_h, &dist_v);

		if (dist_h > radius || v.alt < 0.f || v.alt > _param_alt.get() + 1000.f) {
			spawn_vehicle(i, gpos.lat, gpos.lon);
		}

		// Integrate position forward using flat-earth approximation
		v.lat += (double)(v.vel_n * dt) / CONSTANTS_RADIUS_OF_EARTH * (180.0 / M_PI);
		v.lon += (double)(v.vel_e * dt) / (CONSTANTS_RADIUS_OF_EARTH * cos(v.lat * M_PI / 180.0)) * (180.0 / M_PI);
		v.alt += v.vel_u * dt;

		// Course over ground [0, 2pi], north = 0
		float cog = atan2f(v.vel_e, v.vel_n);

		if (cog < 0.f) {
			cog += 2.f * M_PI_F;
		}

		transponder_report_s report{};
		report.timestamp     = now;
		report.icao_address  = v.icao_address;
		report.lat           = v.lat;
		report.lon           = v.lon;
		report.altitude_type = 1;
		report.altitude      = v.alt;
		report.heading       = cog;
		report.hor_velocity  = sqrtf(v.vel_n * v.vel_n + v.vel_e * v.vel_e);
		report.ver_velocity  = v.vel_u;
		report.emitter_type  = v.emitter_type;
		report.tslc          = 1;
		report.squawk        = 1200;
		report.flags         = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS
				       | transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE
				       | transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING
				       | transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY
				       | transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN
				       | transponder_report_s::PX4_ADSB_FLAGS_VALID_SQUAWK
				       | transponder_report_s::PX4_ADSB_FLAGS_RETRANSLATE;

		snprintf(report.callsign, sizeof(report.callsign), "SIM%02u", i);

		_transponder_report_pub.publish(report);
	}

	perf_end(_loop_perf);
}

int SensorAdsbSim::task_spawn(int argc, char *argv[])
{
	SensorAdsbSim *instance = new SensorAdsbSim();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int SensorAdsbSim::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SensorAdsbSim::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Publishes simulated ADS-B transponder reports for nearby traffic, enabling
testing of Navigator conflict detection and avoidance without hardware.

Each simulated aircraft is assigned a stable ICAO address per slot, spawned
at a random bearing and distance within SIM_ADSB_RADIUS, and flies a constant
random heading at a random speed. Aircraft that leave the radius, hit the
ground, or exceed the max alt by 1000m are re-spawned at a new random position.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sensor_adsb_sim", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int sensor_adsb_sim_main(int argc, char *argv[])
{
	return ModuleBase::main(SensorAdsbSim::desc, argc, argv);
}
