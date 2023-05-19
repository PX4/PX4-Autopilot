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
 * @file fake_ev.cpp
 */

#include "fake_ev.hpp"

#include <mathlib/mathlib.h>

FakeEV::FakeEV() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	updateParams();
}

FakeEV::~FakeEV()
{
	perf_free(_cycle_perf);
}

bool FakeEV::init()
{
	if (!_vehicle_gps_position_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void FakeEV::updateParams()
{
	ModuleParams::updateParams();
}

void FakeEV::Run()
{
	if (should_exit()) {
		_vehicle_gps_position_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (!_vehicle_gps_position_sub.updated()) {
		return;
	}

	perf_begin(_cycle_perf);

	sensor_gps_s gps{};

	if (_vehicle_gps_position_sub.copy(&gps)) {
		vehicle_odometry_s odom = gpsToOdom(gps);
		_vehicle_visual_odometry_pub.publish(odom);
	}

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}

	perf_end(_cycle_perf);
}

vehicle_odometry_s FakeEV::gpsToOdom(const sensor_gps_s &gps)
{
	vehicle_odometry_s odom{vehicle_odometry_empty};

	/* odom.timestamp_sample = gps.timestamp_sample; */ // gps timestamp_sample isn't set in SITL
	odom.timestamp_sample = gps.timestamp;

	const hrt_abstime now = odom.timestamp_sample;
	const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.5f);
	_last_run = now;

	if (gps.fix_type >= 3) {
		if (PX4_ISFINITE(_alt_ref)) {
			matrix::Vector2f hpos = _hpos_prev;

			if (!_param_fev_stale.get()) {
				hpos = _pos_ref.project(gps.lat / 1.0e7, gps.lon / 1.0e7);
				_hpos_prev = hpos;
			}

			_h_drift += _param_fev_h_drift_rate.get() * dt;
			hpos += matrix::Vector2f(_h_drift, _h_drift);
			const float vpos = -((float)gps.alt * 1e-3f - _alt_ref);
			matrix::Vector3f(hpos(0), hpos(1), vpos).copyTo(odom.position);

			const float hvar = std::pow(gps.eph, 2.f);
			const float vvar = std::pow(gps.epv, 2.f);
			matrix::Vector3f(hvar, hvar, vvar).copyTo(odom.position_variance);

			odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;

		} else {
			_pos_ref.initReference(gps.lat / 1.0e7, gps.lon / 1.0e7, gps.timestamp);
			_alt_ref = (float)gps.alt * 1e-3f;
		}
	}

	if (gps.vel_ned_valid) {
		matrix::Vector3f(gps.vel_n_m_s, gps.vel_e_m_s, gps.vel_d_m_s).copyTo(odom.velocity);

		matrix::Vector3f vel_var;
		vel_var.setAll(std::pow(gps.s_variance_m_s, 2.f));
		vel_var.copyTo(odom.velocity_variance);
		odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_NED;
	}

	odom.timestamp = hrt_absolute_time();

	return odom;
}

int FakeEV::task_spawn(int argc, char *argv[])
{
	FakeEV *instance = new FakeEV();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int FakeEV::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FakeEV::print_status()
{
	perf_print_counter(_cycle_perf);

	return 0;
}

int FakeEV::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fake_ev", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fake_ev_main(int argc, char *argv[])
{
	return FakeEV::main(argc, argv);
}
