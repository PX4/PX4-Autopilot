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

#include "MagCalibrator.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

#include <lib/ecl/geo_lookup/geo_mag_declination.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

MagCalibrator::MagCalibrator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	for (int mag_instance = 0; mag_instance < MAX_SENSOR_COUNT; mag_instance++) {
		TRICAL_init(&_trical_instance[mag_instance]);

		TRICAL_norm_set(&_trical_instance[mag_instance], 0.4f);
		TRICAL_noise_set(&_trical_instance[mag_instance], 1e-4f);
	}
}

MagCalibrator::~MagCalibrator()
{
	perf_free(_loop_perf);
}

bool MagCalibrator::init()
{
	ScheduleOnInterval(10_ms); // 100 Hz

	return true;
}

void MagCalibrator::Run()
{
	if (should_exit()) {
		//_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		//parameters_updated();
	}

	if (_vehicle_gps_position_sub.updated()) {
		vehicle_gps_position_s gps;

		if (_vehicle_gps_position_sub.copy(&gps)) {
			if ((gps.fix_type >= 2) && (gps.eph < 1000)) {

				const double lat = gps.lat / 1.e7;
				const double lon = gps.lon / 1.e7;

				// set the magnetic field data returned by the geo library using the current GPS position
				_mag_declination_gps = get_mag_declination_radians(lat, lon);
				_mag_inclination_gps = get_mag_inclination_radians(lat, lon);

				const float mag_strength_gps = get_mag_strength_gauss(lat, lon);

				_mag_earth_pred = Dcmf(Eulerf(0, -_mag_inclination_gps, _mag_declination_gps)) * Vector3f(_mag_strength_gps, 0, 0);

				//  || (fabsf(mag_strength_gps - _mag_strength_gps) > 0.01f)

				if (!_mag_earth_available) {
					for (int mag_instance = 0; mag_instance < MAX_SENSOR_COUNT; mag_instance++) {
						TRICAL_norm_set(&_trical_instance[mag_instance], mag_strength_gps);
					}
				}

				_mag_strength_gps = mag_strength_gps;

				_mag_earth_available = true;
			}
		}
	}

	if (_mag_earth_available) {
		vehicle_attitude_s attitude;

		for (int mag_instance = 0; mag_instance < MAX_SENSOR_COUNT; mag_instance++) {
			sensor_mag_s mag;

			while (_sensor_mag_subs[mag_instance].update(&mag)) {

				_vehicle_attitude_sub.update(&attitude);

				if (labs(mag.timestamp - attitude.timestamp) < 20_ms) {

					_initialized[mag_instance] = true;

					// expected earth field
					const Vector3f expected_field_v = Dcmf(Quatf{attitude.q}).transpose() * _mag_earth_pred;

					float sensor_reading[3] {mag.x, mag.y, mag.z};
					float expected_field[3];
					expected_field_v.copyTo(expected_field);

					TRICAL_estimate_update(&_trical_instance[mag_instance], sensor_reading, expected_field);

					// Calibrates `measurement` based on the current calibration estimates, and copies the result to `calibrated_measurement`.
					float calibrated_reading[3];
					TRICAL_measurement_calibrate(&_trical_instance[mag_instance], sensor_reading, calibrated_reading);


					estimator_mag_cal_s mag_cal{};

					mag_cal.instance = mag_instance;
					mag_cal.device_id = mag.device_id;

					mag_cal.sample[0] = mag.x;
					mag_cal.sample[1] = mag.y;
					mag_cal.sample[2] = mag.z;

					expected_field_v.copyTo(mag_cal.expected_field);

					mag_cal.calibrated_sample[0] = calibrated_reading[0];
					mag_cal.calibrated_sample[1] = calibrated_reading[1];
					mag_cal.calibrated_sample[2] = calibrated_reading[2];

					mag_cal.calibrated_sample_magnitude = Vector3f(calibrated_reading).norm();

					mag_cal.declination = _mag_declination_gps;
					mag_cal.inclination = _mag_inclination_gps;
					mag_cal.mag_strength = _mag_strength_gps;

					// simple error metric
					mag_cal.error = Vector3f(expected_field_v - Vector3f{calibrated_reading}).norm();

					TRICAL_estimate_get_ext(&_trical_instance[mag_instance], mag_cal.bias_estimate, mag_cal.scale_estimate,
								mag_cal.bias_estimate_variance, mag_cal.scale_estimate_variance);

					//  0  1  2
					//  3  4  5
					//  6  7  8
					SquareMatrix3f I3;
					I3.setIdentity();
					SquareMatrix3f D = SquareMatrix3f{I3 + SquareMatrix3f{mag_cal.scale_estimate}};
					D.makeBlockSymmetric<3>(0);
					mag_cal.diagonal_scale[0] = D(0, 0);
					mag_cal.diagonal_scale[1] = D(1, 1);
					mag_cal.diagonal_scale[2] = D(2, 2);

					mag_cal.offdiagonal_scale[0] = (D(0, 1) + D(1, 0)) / 2;
					mag_cal.offdiagonal_scale[1] = (D(0, 2) + D(2, 0)) / 2;
					mag_cal.offdiagonal_scale[2] = (D(1, 2) + D(2, 1)) / 2;

					// TODO:
					//  B' = (I_{3x3} + D)B - b
					//    D*(B+b) = D*B + D^-1*b
					const Vector3f offsets = D.I() * Vector3f{mag_cal.bias_estimate};
					offsets.copyTo(mag_cal.offsets);

					mag_cal.timestamp = hrt_absolute_time();
					_estimator_mag_cal_pub[mag_instance].publish(mag_cal);

					// update calibration
					//_calibration.set_offsets(offsets);
				}
			}
		}
	}

	perf_end(_loop_perf);
}

int MagCalibrator::print_status()
{
	for (int mag_instance = 0; mag_instance < MAX_SENSOR_COUNT; mag_instance++) {

		if (_initialized[mag_instance]) {

			float bias_estimate[3];
			float scale_estimate[9];
			float bias_estimate_variance[3];
			float scale_estimate_variance[9];

			TRICAL_estimate_get_ext(&_trical_instance[mag_instance], bias_estimate, scale_estimate, bias_estimate_variance,
						scale_estimate_variance);

			printf("bias_estimate: %d\n", mag_instance);

			for (int i = 0; i < 3; i++) {
				printf("%05.4f ", (double)bias_estimate[i]);
			}

			printf("\n\n");

			printf("scale_estimate: %d\n", mag_instance);

			for (int c = 0; c < 3; c++) {
				for (int r = 0; r < 3; r++) {
					printf("%05.4f ", (double)scale_estimate[r + 3 * c]);
				}

				printf("\n");
			}

			printf("\n");
		}
	}

	return 0;
}

int MagCalibrator::task_spawn(int argc, char *argv[])
{
	MagCalibrator *instance = new MagCalibrator();

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

int MagCalibrator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MagCalibrator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description


)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mag_calibrator", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mag_calibrator_main(int argc, char *argv[])
{
	return MagCalibrator::main(argc, argv);
}
