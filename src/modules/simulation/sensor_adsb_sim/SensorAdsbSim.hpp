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

#pragma once

#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/transponder_report.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_global_position.h>

using namespace time_literals;

class SensorAdsbSim : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	SensorAdsbSim();
	~SensorAdsbSim() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	static constexpr uint8_t  MAX_VEHICLES{10};
	static constexpr float    MIN_SPEED_M_S{10.f};   // ~36 km/h, slow rotorcraft / fast UAV
	static constexpr float    MAX_SPEED_M_S{90.f};   // ~324 km/h, light aircraft cruise
	static constexpr float    MAX_VERTICAL_SPEED_M_S{8.f}; // typical cruise climb/descent rate
	// Base ICAO address: "SIM\0" in ASCII, one per slot
	static constexpr uint32_t ICAO_BASE{0x53494D00u};

	// Airborne emitter types only — excludes NO_INFO, UNASSIGNED*, SPACE, and surface/obstacle types
	static constexpr uint8_t EMITTER_TYPES[] {
		transponder_report_s::ADSB_EMITTER_TYPE_LIGHT,
		transponder_report_s::ADSB_EMITTER_TYPE_SMALL,
		transponder_report_s::ADSB_EMITTER_TYPE_LARGE,
		transponder_report_s::ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE,
		transponder_report_s::ADSB_EMITTER_TYPE_HEAVY,
		transponder_report_s::ADSB_EMITTER_TYPE_HIGHLY_MANUV,
		transponder_report_s::ADSB_EMITTER_TYPE_ROTOCRAFT,
		transponder_report_s::ADSB_EMITTER_TYPE_GLIDER,
		transponder_report_s::ADSB_EMITTER_TYPE_LIGHTER_AIR,
		transponder_report_s::ADSB_EMITTER_TYPE_PARACHUTE,
		transponder_report_s::ADSB_EMITTER_TYPE_ULTRA_LIGHT,
		transponder_report_s::ADSB_EMITTER_TYPE_UAV,
	};

	struct SimulatedVehicle {
		uint32_t icao_address{0};
		double   lat{0.0};
		double   lon{0.0};
		float    alt{0.f};
		float    vel_n{0.f};  // m/s north
		float    vel_e{0.f};  // m/s east
		float    vel_u{0.f};  // m/s up
		uint8_t  emitter_type{0};
		bool     initialized{false};
	};

	void Run() override;
	void spawn_vehicle(uint8_t index, double lat_ref, double lon_ref);
	void check_failure_injection();
	float random_float(float min, float max);

	SimulatedVehicle _vehicles[MAX_VEHICLES]{};
	hrt_abstime      _last_update{0};
	bool             _adsb_failed{false};

	uORB::SubscriptionInterval          	_parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription                  	_vehicle_global_position_sub{ORB_ID(vehicle_global_position_groundtruth)};
	uORB::Subscription                  	_vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Publication<transponder_report_s> _transponder_report_pub{ORB_ID(transponder_report)};
	uORB::Publication<vehicle_command_ack_s> _command_ack_pub{ORB_ID(vehicle_command_ack)};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SIM_ADSB_COUNT>)    _param_count,
		(ParamFloat<px4::params::SIM_ADSB_RADIUS>) _param_radius,
		(ParamFloat<px4::params::SIM_ADSB_ALT>)    _param_alt
	)
};
