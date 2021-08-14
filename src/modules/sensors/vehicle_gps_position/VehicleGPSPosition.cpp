/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "VehicleGPSPosition.hpp"

#include <px4_platform_common/log.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>

namespace sensors
{
VehicleGPSPosition::VehicleGPSPosition() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

VehicleGPSPosition::~VehicleGPSPosition()
{
	Stop();
	perf_free(_cycle_perf);
}

bool VehicleGPSPosition::Start()
{
	// force initial updates
	ParametersUpdate(true);

	ScheduleNow();

	return true;
}

void VehicleGPSPosition::Stop()
{
	Deinit();

	// clear all registered callbacks
	for (auto &sub : _sensor_gps_sub) {
		sub.unregisterCallback();
	}
}

void VehicleGPSPosition::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();

		if (_param_sens_gps_mask.get() == 0) {
			_sensor_gps_sub[0].registerCallback();

		} else {
			for (auto &sub : _sensor_gps_sub) {
				sub.registerCallback();
			}
		}

		_gps_blending.setBlendingUseSpeedAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_SPD_ACC);
		_gps_blending.setBlendingUseHPosAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_HPOS_ACC);
		_gps_blending.setBlendingUseVPosAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_VPOS_ACC);
		_gps_blending.setBlendingTimeConstant(_param_sens_gps_tau.get());
		_gps_blending.setPrimaryInstance(_param_sens_gps_prime.get());
	}
}

void VehicleGPSPosition::resetGpsDriftCheckFilters()
{
	for (int i = 0; i < GPS_MAX_RECEIVERS; i++) {
		_gps_velNE_filt[i].setZero();
		_gps_pos_deriv_filt[i].setZero();
	}
}

void VehicleGPSPosition::Run()
{
	perf_begin(_cycle_perf);
	ParametersUpdate();

	// GPS blending
	ScheduleDelayed(500_ms); // backup schedule

	// Check all GPS instance
	bool any_gps_updated = false;
	bool gps_updated = false;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		gps_updated = _sensor_gps_sub[i].updated();

		sensor_gps_s gps;

		if (gps_updated) {
			any_gps_updated = true;

			_sensor_gps_sub[i].copy(&gps);
			_gps_blending.setGpsData(gps, i);

			if (!_sensor_gps_sub[i].registered()) {
				_sensor_gps_sub[i].registerCallback();
			}

			// update drift metrics
			if (gps.timestamp > _timestamp_prev[i]) {

				// check if GPS quality is degraded
				_sensors_status_gps.error_norm[i] = fmaxf((gps.eph / _param_sens_gps_rq_eph.get()),
								    (gps.epv / _param_sens_gps_rq_epv.get()));
				_sensors_status_gps.error_norm[i] = fmaxf(_sensors_status_gps.error_norm[i],
								    (gps.s_variance_m_s / _param_sens_gps_rq_sacc.get()));

				_sensors_status_gps.device_ids[i] = gps.device_id;

				// Check the fix type
				_sensors_status_gps.fail_fix[i] = (gps.fix_type < 3);

				// Check the number of satellites
				_sensors_status_gps.fail_nsats[i] = (gps.satellites_used < _param_sens_gps_rq_nsat.get());

				// Check the position dilution of precision
				float pdop = sqrtf(gps.hdop * gps.hdop + gps.vdop * gps.vdop);
				_sensors_status_gps.fail_pdop[i] = (pdop > _param_sens_gps_rq_pdop.get());

				// Check the reported horizontal and vertical position accuracy
				_sensors_status_gps.fail_hacc[i] = (gps.eph > _param_sens_gps_rq_eph.get());
				_sensors_status_gps.fail_vacc[i] = (gps.epv > _param_sens_gps_rq_epv.get());

				// Check the reported speed accuracy
				_sensors_status_gps.fail_sacc[i] = (gps.s_variance_m_s > _param_sens_gps_rq_sacc.get());


				if (_landed && _at_rest) {

					// The following checks are only valid when the vehicle is at rest
					const double lat = gps.lat * 1.0e-7;
					const double lon = gps.lon * 1.0e-7;

					// Calculate position movement since last measurement
					float delta_pos_n = 0.0f;
					float delta_pos_e = 0.0f;

					// calculate position movement since last GPS fix
					if (_gps_pos_prev[i].timestamp > 0) {
						map_projection_project(&_gps_pos_prev[i], lat, lon, &delta_pos_n, &delta_pos_e);

					} else {
						// no previous position has been set
						map_projection_init_timestamped(&_gps_pos_prev[i], lat, lon, gps.timestamp);
						_gps_alt_prev[i] = 1e-3f * (float)gps.alt;
					}

					if (_timestamp_prev[i] != 0) {
						static constexpr float filt_time_const = 10.0f;
						const float dt = math::constrain((gps.timestamp - _timestamp_prev[i]) * 1e-6f, 0.001f, filt_time_const);
						const float filter_coef = dt / filt_time_const;

						// Calculate the horizontal and vertical drift velocity components and limit to 10x the threshold
						const matrix::Vector3f vel_limit{_param_sens_gps_rq_hdrf.get(), _param_sens_gps_rq_hdrf.get(), _param_sens_gps_rq_vdrf.get()};
						matrix::Vector3f pos_derived{delta_pos_n, delta_pos_e, (_gps_alt_prev[i] - 1e-3f * (float)gps.alt)};
						pos_derived = matrix::constrain(pos_derived / dt, -10.f * vel_limit, 10.f * vel_limit);

						// Apply a low pass filter
						_gps_pos_deriv_filt[i] = pos_derived * filter_coef + _gps_pos_deriv_filt[i] * (1.f - filter_coef);

						// Calculate the horizontal drift speed and fail if too high
						_sensors_status_gps.drift_rate_hpos[i] = Vector2f(_gps_pos_deriv_filt[i].xy()).norm();
						_sensors_status_gps.fail_hdrift[i] = (_gps_drift_metrics[i][0] > _param_sens_gps_rq_hdrf.get());

						// Fail if the vertical drift speed is too high
						_sensors_status_gps.drift_rate_vpos[i] = fabsf(_gps_pos_deriv_filt[i](2));
						_sensors_status_gps.fail_vdrift[i] = (_gps_drift_metrics[i][1] > _param_sens_gps_rq_vdrf.get());

						// Check the magnitude of the filtered horizontal GPS velocity

						// Calculate time lapsed since last update, limit to prevent numerical errors and calculate a lowpass filter coefficient
						const Vector2f gps_velNE = matrix::constrain(Vector2f{gps.vel_n_m_s, gps.vel_e_m_s},
									   -10.f * _param_sens_gps_rq_hdrf.get(), 10.f * _param_sens_gps_rq_hdrf.get());
						_gps_velNE_filt[i] = gps_velNE * filter_coef + _gps_velNE_filt[i] * (1.f - filter_coef);
						_sensors_status_gps.drift_hspd[i] = _gps_velNE_filt[i].norm();
						_sensors_status_gps.fail_hspeed[i] = (_gps_drift_metrics[i][2] > _param_sens_gps_rq_hdrf.get());
					}

				} else if (!_landed) {
					// These checks are always declared as passed when flying
					// If on ground and moving, the last result before movement commenced is kept
					// _gps_check_fail_status.flags.hdrift = false;
					// _gps_check_fail_status.flags.vdrift = false;
					// _gps_check_fail_status.flags.hspeed = false;
					// _gps_drift_updated = false;

					resetGpsDriftCheckFilters();

				} else {
					// This is the case where the vehicle is on ground and IMU movement is blocking the drift calculation
					//_gps_drift_updated = true;

					resetGpsDriftCheckFilters();
				}

				_sensors_status_gps.healthy[i] = (_sensors_status_gps.fail_fix[i] && (_param_sens_gps_checks.get() & MASK_GPS_NSATS))
								 && (_sensors_status_gps.fail_fix[i] && (_param_sens_gps_checks.get() & MASK_GPS_PDOP))
								 && (_sensors_status_gps.fail_pdop[i] && (_param_sens_gps_checks.get() & MASK_GPS_HACC))
								 && (_sensors_status_gps.fail_vacc[i] && (_param_sens_gps_checks.get() & MASK_GPS_VACC))
								 && (_sensors_status_gps.fail_sacc[i] && (_param_sens_gps_checks.get() & MASK_GPS_SACC))
								 && (_sensors_status_gps.fail_hdrift[i] && (_param_sens_gps_checks.get() & MASK_GPS_HDRIFT))
								 && (_sensors_status_gps.fail_vdrift[i] && (_param_sens_gps_checks.get() & MASK_GPS_VDRIFT))
								 && (_sensors_status_gps.fail_hspeed[i] && (_param_sens_gps_checks.get() & MASK_GPS_HSPD))
								 && (_sensors_status_gps.fail_vspeed[i] && (_param_sens_gps_checks.get() & MASK_GPS_VSPD));
			}

			_timestamp_prev[i] = gps.timestamp;
		}
	}

	if (any_gps_updated) {
		_sensors_status_gps.timestamp = hrt_absolute_time();
		_sensors_status_gps_pub.publish(_sensors_status_gps);

		_gps_blending.update(hrt_absolute_time());

		if (_gps_blending.isNewOutputDataAvailable()) {
			Publish(_gps_blending.getOutputGpsData(), _gps_blending.getSelectedGps());
		}
	}

	perf_end(_cycle_perf);
}

void VehicleGPSPosition::Publish(const sensor_gps_s &gps, uint8_t selected)
{
	vehicle_gps_position_s gps_output{};

	gps_output.timestamp = gps.timestamp;
	gps_output.time_utc_usec = gps.time_utc_usec;
	gps_output.lat = gps.lat;
	gps_output.lon = gps.lon;
	gps_output.alt = gps.alt;
	gps_output.alt_ellipsoid = gps.alt_ellipsoid;
	gps_output.s_variance_m_s = gps.s_variance_m_s;
	gps_output.c_variance_rad = gps.c_variance_rad;
	gps_output.eph = gps.eph;
	gps_output.epv = gps.epv;
	gps_output.hdop = gps.hdop;
	gps_output.vdop = gps.vdop;
	gps_output.noise_per_ms = gps.noise_per_ms;
	gps_output.jamming_indicator = gps.jamming_indicator;
	gps_output.jamming_state = gps.jamming_state;
	gps_output.vel_m_s = gps.vel_m_s;
	gps_output.vel_n_m_s = gps.vel_n_m_s;
	gps_output.vel_e_m_s = gps.vel_e_m_s;
	gps_output.vel_d_m_s = gps.vel_d_m_s;
	gps_output.cog_rad = gps.cog_rad;
	gps_output.timestamp_time_relative = gps.timestamp_time_relative;
	gps_output.heading = gps.heading;
	gps_output.heading_offset = gps.heading_offset;
	gps_output.fix_type = gps.fix_type;
	gps_output.vel_ned_valid = gps.vel_ned_valid;
	gps_output.satellites_used = gps.satellites_used;

	gps_output.selected = selected;

	_vehicle_gps_position_pub.publish(gps_output);
}

void VehicleGPSPosition::PrintStatus()
{
	//PX4_INFO("selected GPS: %d", _gps_select_index);
	for (int i = 0; i < GPS_MAX_RECEIVERS; i++) {
		if (_timestamp_prev[i] != 0) {
			PX4_INFO_RAW("GPS %d healthy: %d", i, _sensors_status_gps.healthy[i]);
		}
	}
}

}; // namespace sensors
