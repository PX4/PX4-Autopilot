/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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

#include "VehicleIMU.hpp"

#include <lib/sensor_calibration/Accelerometer.hpp>
#include <lib/sensor_calibration/Gyroscope.hpp>
#include <lib/sensor_calibration/Utilities.hpp>

#include <px4_platform_common/log.h>

#include <lib/geo/geo.h>

using namespace matrix;

namespace sensors
{

static constexpr bool clipping(const int16_t sample) { return (sample == INT16_MIN) || (sample == INT16_MAX); }

static constexpr int clipping(const int16_t samples[], int length)
{
	int clip_count = 0;

	for (int n = 0; n < length; n++) {
		if (clipping(samples[n])) {
			clip_count++;
		}
	}

	return clip_count;
}

static constexpr int32_t sum(const int16_t samples[], int length)
{
	int32_t sum = 0;

	for (int n = 0; n < length; n++) {
		sum += samples[n];
	}

	return sum;
}

static matrix::Vector3f AverageFifoAccel(const sensor_imu_fifo_s &sensor_imu_fifo)
{
	const float scale = sensor_imu_fifo.accel_scale / sensor_imu_fifo.samples;

	return matrix::Vector3f{
		scale * sum(sensor_imu_fifo.accel_x, sensor_imu_fifo.samples),
		scale * sum(sensor_imu_fifo.accel_y, sensor_imu_fifo.samples),
		scale * sum(sensor_imu_fifo.accel_z, sensor_imu_fifo.samples)
	};
}

static matrix::Vector3f AverageFifoGyro(const sensor_imu_fifo_s &sensor_imu_fifo)
{
	const float scale = sensor_imu_fifo.gyro_scale / sensor_imu_fifo.samples;

	return matrix::Vector3f{
		scale * sum(sensor_imu_fifo.gyro_x, sensor_imu_fifo.samples),
		scale * sum(sensor_imu_fifo.gyro_y, sensor_imu_fifo.samples),
		scale * sum(sensor_imu_fifo.gyro_z, sensor_imu_fifo.samples)
	};
}

VehicleIMU::VehicleIMU() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

VehicleIMU::~VehicleIMU()
{
	Stop();

	perf_free(_cycle_perf);
	perf_free(_selection_changed_perf);
}

bool VehicleIMU::Start()
{
	// force initial updates
	ParametersUpdate(true);

	// sensor_accel
	for (auto &sub : _sensor_accel_subs) {
		sub.set_required_updates(sensor_accel_s::ORB_QUEUE_LENGTH);
		sub.registerCallback();
	}

	// sensor_gyro
	for (auto &sub : _sensor_gyro_subs) {
		sub.set_required_updates(sensor_gyro_s::ORB_QUEUE_LENGTH);
		sub.registerCallback();
	}

	// sensor_imu_fifo
	for (auto &sub : _sensor_imu_fifo_subs) {
		sub.set_required_updates(sensor_imu_fifo_s::ORB_QUEUE_LENGTH);
		sub.registerCallback();
	}

	ScheduleNow();

	return true;
}

void VehicleIMU::Stop()
{
	// clear all registered callbacks

	// sensor_accel
	for (auto &sub : _sensor_accel_subs) {
		sub.unregisterCallback();
	}

	// sensor_gyro
	for (auto &sub : _sensor_gyro_subs) {
		sub.unregisterCallback();
	}

	// sensor_imu_fifo
	for (auto &sub : _sensor_imu_fifo_subs) {
		sub.unregisterCallback();
	}

	ScheduleClear();

	Deinit();
}

void VehicleIMU::SensorBiasUpdate()
{
	// find corresponding estimated sensor bias
	if (_estimator_selector_status_sub.updated()) {
		estimator_selector_status_s estimator_selector_status;

		if (_estimator_selector_status_sub.copy(&estimator_selector_status)) {
			//_estimator_sensor_bias_sub.ChangeInstance(estimator_selector_status.primary_instance);

			// TODO:
			//  find corresponding primary accel and gyro device id
		}
	}

	for (int est_bias_instance = 0; est_bias_instance < _estimator_sensor_bias_subs.size(); est_bias_instance++) {
		estimator_sensor_bias_s estimator_sensor_bias;

		if (_estimator_sensor_bias_subs[est_bias_instance].update(&estimator_sensor_bias)) {

			for (auto &imu : _imus) {
				if (estimator_sensor_bias.accel_bias_valid
				    && (estimator_sensor_bias.accel_device_id != 0)
				    && (estimator_sensor_bias.accel_device_id == imu.accel.calibration.device_id())
				   ) {
					const Vector3f bias{estimator_sensor_bias.accel_bias};
					const Vector3f bias_var{estimator_sensor_bias.accel_bias_variance};

					if ((bias_var.norm() < imu.accel.estimated_bias_variance.norm())
					    || !imu.accel.estimated_bias_variance.longerThan(FLT_EPSILON)) {

						imu.accel.estimated_bias = bias;
						imu.accel.estimated_bias_variance = bias_var;
					}

					// TODO: store all
					imu.accel.learned_calibration[0].instance = est_bias_instance;
					imu.accel.learned_calibration[0].offset = imu.accel.calibration.BiasCorrectedSensorOffset(bias);
					imu.accel.learned_calibration[0].bias_variance = bias_var;
					imu.accel.learned_calibration[0].valid = true;
					_accel_cal_available = true;
				}

				if (estimator_sensor_bias.gyro_bias_valid
				    && (estimator_sensor_bias.gyro_device_id != 0)
				    && (estimator_sensor_bias.gyro_device_id == imu.gyro.calibration.device_id())
				   ) {
					const Vector3f bias{estimator_sensor_bias.gyro_bias};
					const Vector3f bias_var{estimator_sensor_bias.gyro_bias_variance};

					if ((bias_var.norm() < imu.gyro.estimated_bias_variance.norm())
					    || !imu.gyro.estimated_bias_variance.longerThan(FLT_EPSILON)) {

						imu.gyro.estimated_bias = bias;
						imu.gyro.estimated_bias_variance = bias_var;
					}

					// TODO: store all
					imu.gyro.learned_calibration[0].instance = est_bias_instance;
					imu.gyro.learned_calibration[0].offset = imu.gyro.calibration.BiasCorrectedSensorOffset(bias);
					imu.gyro.learned_calibration[0].bias_variance = bias_var;
					imu.gyro.learned_calibration[0].valid = true;
					_gyro_cal_available = true;
				}

			}
		}
	}
}

bool VehicleIMU::SensorSelectionUpdate(bool force)
{
	// find highest priority
	int highest_priority = 0;
	int highest_priority_index = -1;

	for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
		const IMU &imu = _imus[i];

		if (imu.accel.calibration.enabled() && (hrt_elapsed_time(&imu.time_last_valid) < 1_s)) {
			if (imu.accel.calibration.priority() > highest_priority) {
				highest_priority = imu.accel.calibration.priority();
				highest_priority_index = i;
			}
		}
	}

	// timeout

	// clipping and other health checks


	// multi-EKF respect selected IMU
#if 0

	// find corresponding estimated sensor bias
	if (_estimator_selector_status_sub.updated()) {
		estimator_selector_status_s estimator_selector_status;

		if (_estimator_selector_status_sub.copy(&estimator_selector_status)) {
			//_estimator_sensor_bias_sub.ChangeInstance(estimator_selector_status.primary_instance);

			// TODO:
			//  find corresponding primary accel and gyro device id
		}
	}

#endif



	// TODO:
	//  - timeout based on expected sensor publication rate



	if (highest_priority_index >= 0) {

		if (_selected_imu_index != highest_priority_index) {

			for (auto &imu : _imus) {
				imu.primary = false;
			}

			_selected_imu_index = highest_priority_index;
			_selected_accel_device_id = _imus[_selected_imu_index].accel.calibration.device_id();
			_selected_gyro_device_id = _imus[_selected_imu_index].gyro.calibration.device_id();

			_imus[_selected_imu_index].primary = true;

			PX4_INFO("selected primary IMU %d", _selected_imu_index);

			// TODO: on primary change update callback scheduling
			// sub.set_required_updates(1);

			return true;
		}
	}


	return false;
}

bool VehicleIMU::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);


		// if calibration changed clear anything learned in flight
		for (auto &imu : _imus) {
			const auto accel_calibration_count = imu.accel.calibration.calibration_count();
			const auto gyro_calibration_count = imu.gyro.calibration.calibration_count();

			imu.accel.calibration.ParametersUpdate();
			imu.gyro.calibration.ParametersUpdate();

			if (accel_calibration_count != imu.accel.calibration.calibration_count()) {
				// if calibration changed reset any existing learned calibration
				_accel_cal_available = false;
				_in_flight_calibration_check_timestamp_last = hrt_absolute_time() + INFLIGHT_CALIBRATION_QUIET_PERIOD_US;

				for (auto &learned_cal : imu.accel.learned_calibration) {
					for (auto &learned_cal : imu.accel.learned_calibration) {
						learned_cal = {};
					}
				}
			}

			if (gyro_calibration_count != imu.gyro.calibration.calibration_count()) {
				// if calibration changed reset any existing learned calibration
				_gyro_cal_available = false;
				_in_flight_calibration_check_timestamp_last = hrt_absolute_time() + INFLIGHT_CALIBRATION_QUIET_PERIOD_US;

				for (auto &learned_cal : imu.gyro.learned_calibration) {
					learned_cal = {};
				}
			}

		}




		// 1. mark all existing sensor calibrations active even if sensor is missing
		//    this preserves the calibration in the event of a parameter export while the sensor is missing
		// 2. ensure calibration slots are active for the number of sensors currently available
		//    this to done to eliminate differences in the active set of parameters before and after sensor calibration
		for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
			// sensor_accel
			{
				const uint32_t device_id_accel = calibration::GetCalibrationParamInt32("ACC",  "ID", i);

				if (device_id_accel != 0) {
					calibration::Accelerometer accel_cal(device_id_accel);
				}

				uORB::SubscriptionData<sensor_accel_s> sensor_accel_sub{ORB_ID(sensor_accel), i};

				if (sensor_accel_sub.advertised() && (sensor_accel_sub.get().device_id != 0)) {
					calibration::Accelerometer cal;
					cal.set_calibration_index(i);
					cal.ParametersLoad();
				}
			}


			// sensor_gyro
			{
				const uint32_t device_id_gyro = calibration::GetCalibrationParamInt32("GYRO", "ID", i);

				if (device_id_gyro != 0) {
					calibration::Gyroscope gyro_cal(device_id_gyro);
				}

				uORB::SubscriptionData<sensor_gyro_s> sensor_gyro_sub{ORB_ID(sensor_gyro), i};

				if (sensor_gyro_sub.advertised() && (sensor_gyro_sub.get().device_id != 0)) {
					calibration::Gyroscope cal;
					cal.set_calibration_index(i);
					cal.ParametersLoad();
				}
			}
		}





		// TODO: IMU priority changes
#if 0

		if (accel_cal_index >= 0) {
			// found matching CAL_ACCx_PRIO
			int32_t accel_priority_old = _accel.priority_configured[uorb_index];

			_accel.priority_configured[uorb_index] = calibration::GetCalibrationParamInt32("ACC", "PRIO", accel_cal_index);

			if (accel_priority_old != _accel.priority_configured[uorb_index]) {
				if (_accel.priority_configured[uorb_index] == 0) {
					// disabled
					_accel.priority[uorb_index] = 0;

				} else {
					// change relative priority to incorporate any sensor faults
					int priority_change = _accel.priority_configured[uorb_index] - accel_priority_old;
					_accel.priority[uorb_index] = math::constrain(_accel.priority[uorb_index] + priority_change, static_cast<int32_t>(1),
								      static_cast<int32_t>(100));
				}
			}
		}

		if (gyro_cal_index >= 0) {
			// found matching CAL_GYROx_PRIO
			int32_t gyro_priority_old = _gyro.priority_configured[uorb_index];

			_gyro.priority_configured[uorb_index] = calibration::GetCalibrationParamInt32("GYRO", "PRIO", gyro_cal_index);

			if (gyro_priority_old != _gyro.priority_configured[uorb_index]) {
				if (_gyro.priority_configured[uorb_index] == 0) {
					// disabled
					_gyro.priority[uorb_index] = 0;

				} else {
					// change relative priority to incorporate any sensor faults
					int priority_change = _gyro.priority_configured[uorb_index] - gyro_priority_old;
					_gyro.priority[uorb_index] = math::constrain(_gyro.priority[uorb_index] + priority_change, static_cast<int32_t>(1),
								     static_cast<int32_t>(100));
				}
			}
		}

#endif


		updateParams();

		_vehicle_acceleration.ParametersUpdate();
		_vehicle_angular_velocity.ParametersUpdate();

		for (auto &imu : _imus) {
			imu.accel.calibration.ParametersUpdate();
			imu.gyro.calibration.ParametersUpdate();
		}

		// IMU_GYRO_RATEMAX
		if (_param_imu_gyro_ratemax.get() <= 0) {
			const int32_t imu_gyro_ratemax = _param_imu_gyro_ratemax.get();
			_param_imu_gyro_ratemax.reset();
			PX4_WARN("IMU_GYRO_RATEMAX invalid (%" PRId32 "), resetting to default %" PRId32 ")", imu_gyro_ratemax,
				 _param_imu_gyro_ratemax.get());
		}

		// constrain IMU_GYRO_RATEMAX 50-10,000 Hz
		const int32_t imu_gyro_ratemax = constrain(_param_imu_gyro_ratemax.get(), (int32_t)50, (int32_t)10'000);

		if (imu_gyro_ratemax != _param_imu_gyro_ratemax.get()) {
			PX4_WARN("IMU_GYRO_RATEMAX updated %" PRId32 " -> %" PRIu32, _param_imu_gyro_ratemax.get(), imu_gyro_ratemax);
			_param_imu_gyro_ratemax.set(imu_gyro_ratemax);
			_param_imu_gyro_ratemax.commit_no_notification();
		}



		_last_calibration_update = hrt_absolute_time();


		return true;
	}

	return false;
}

void VehicleIMU::Run()
{
	perf_begin(_cycle_perf);

	// backup schedule
	ScheduleDelayed(1.5e6 / _param_imu_gyro_ratemax.get()); // backup 150% of expected/desired interval

	// check vehicle status for changes to armed state
	if (_vehicle_control_mode_sub.updated()) {
		vehicle_control_mode_s vehicle_control_mode;

		if (_vehicle_control_mode_sub.copy(&vehicle_control_mode)) {
			_armed = vehicle_control_mode.flag_armed;
		}
	}

	const hrt_abstime time_now_us = hrt_absolute_time();

	const bool parameters_updated = ParametersUpdate();

	SensorSelectionUpdate();
	SensorBiasUpdate();

	for (int sensor_instance = 0; sensor_instance < MAX_SENSOR_COUNT; sensor_instance++) {
		UpdateSensorImuFifo(sensor_instance);
		UpdateSensorAccel(sensor_instance);
		UpdateSensorGyro(sensor_instance);
	}

	// TODO: IMU at rest (across all enabled)
	//  - if still don't reset gyro/accel welford mean, instead use it for calibration
	//   - delete gyro_calibraiton module


	//  accel integration on raw int16 data

	// sensor_imu_fifo -> imu instance
	//  sensor_accel + sensor_gyro -> imu instance
	//  sensors_combined, sensor selection


	PublishSensorsStatusIMU();



	// TODO: IMU selection
	//  - timeouts (checked all at once)
	//  - errors (sensor_accel + sensor_gyro)
	//  - stuck data
	//  - sensor variance
	//  -  variance between sensors
	//  - publish sensor_selection_s




	// TODO: calibration
	//  - gyro if still
	//  - post flight estimated

	// TODO: non-FIFO case


	if (_param_sens_imu_autocal.get() && !parameters_updated) {

		updateGyroCalibration();

		// TODO: accel uncalibrated
		// TODO: gyro uncalibrated
		bool accel_uncalibrated = false; // !_accel_calibration.calibrated()
		bool gyro_uncalibrated = false; // !_gyro_calibration.calibrated()

		if ((_armed || accel_uncalibrated || gyro_uncalibrated)
		    && (time_now_us > _in_flight_calibration_check_timestamp_last + 1_s)) {

			//SensorCalibrationUpdate();
			_in_flight_calibration_check_timestamp_last = time_now_us;

		} else if (!_armed) {
			//SensorCalibrationSaveAccel();
			//SensorCalibrationSaveGyro();
		}
	}

	perf_end(_cycle_perf);
}

int8_t VehicleIMU::findAccelInstance(uint32_t device_id)
{
	if (device_id == 0) {
		return -1;
	}

	// look for existing matching
	for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
		if (_imus[i].accel.device_id == device_id) {
			return i;
		}
	}

	// look for empty slot
	for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
		if (_imus[i].accel.device_id == 0) {
			_imus[i].accel.device_id = device_id;
			return i;
		}
	}

	return -1;
}

void VehicleIMU::UpdateSensorImuFifo(uint8_t sensor_instance)
{
	// sensor_imu_fifo
	auto &sub = _sensor_imu_fifo_subs[sensor_instance];

	while (sub.updated()) {
		unsigned last_generation = sub.get_last_generation();
		sensor_imu_fifo_s sensor_imu_fifo;
		sub.copy(&sensor_imu_fifo);

		int8_t imu_instance = findAccelInstance(sensor_imu_fifo.device_id);

		if (imu_instance == -1) {
			return;
		}

		IMU &imu = _imus[imu_instance];

		// publication interval
		//  sensor output data rate or interval
		if (sub.get_last_generation() == last_generation + 1) {

			if ((sensor_imu_fifo.timestamp_sample > imu.accel.timestamp_sample_last)
			    && (sensor_imu_fifo.timestamp_sample > imu.gyro.timestamp_sample_last)
			   ) {

				const float interval_us = sensor_imu_fifo.timestamp_sample - imu.accel.timestamp_sample_last;
				const float sample_interval_us = interval_us / sensor_imu_fifo.samples;

				imu.accel.mean_publish_interval_us.update(interval_us);
				imu.accel.mean_sample_interval_us.update(sample_interval_us);

				imu.gyro.mean_publish_interval_us.update(interval_us);
				imu.gyro.mean_sample_interval_us.update(sample_interval_us);

				// check and update sensor rate
				if (imu.gyro.mean_publish_interval_us.valid() &&
				    (imu.gyro.mean_publish_interval_us.count() > 1000 || imu.gyro.mean_publish_interval_us.standard_deviation() < 100)
				   ) {

					// determine number of sensor samples that will get closest to the desired integration interval
					const float imu_integration_interval_us = 1e6f / _param_imu_integ_rate.get();
					const float pub_interval_avg_us = imu.gyro.mean_publish_interval_us.mean();

					const uint8_t imu_publications = roundf(imu_integration_interval_us / pub_interval_avg_us);

					const float integration_interval_us = roundf(imu_publications * pub_interval_avg_us);

					// TODO: variance
					if ((static_cast<float>(imu.gyro.integrator.reset_interval_us()) - integration_interval_us) >
					    imu.gyro.mean_publish_interval_us.standard_deviation()
					    // || primary_changed
					   ) {

						const auto accel_reset_samples_prev = imu.accel.integrator.get_reset_samples();
						const auto gyro_reset_samples_prev = imu.gyro.integrator.get_reset_samples();

						imu.gyro.integrator.set_reset_interval(integration_interval_us);
						imu.accel.integrator.set_reset_interval(integration_interval_us / 2.f);

						// number of samples per publication
						int interval_samples = roundf(integration_interval_us / imu.gyro.mean_sample_interval_us.mean());
						imu.gyro.integrator.set_reset_samples(math::max(interval_samples, 1));
						imu.accel.integrator.set_reset_samples(1);

						if (accel_reset_samples_prev != imu.accel.integrator.get_reset_samples()
						    || gyro_reset_samples_prev != imu.gyro.integrator.get_reset_samples()
						    || (static_cast<float>(imu.gyro.integrator.reset_interval_us()) - integration_interval_us > 100)
						   ) {

							if (imu.primary) {
								sub.set_required_updates(1);

							} else {
								// avg samples per publication
								float samples_per_pub = imu.gyro.mean_publish_interval_us.mean() / imu.gyro.mean_sample_interval_us.mean();
								uint8_t pubs_per_integrator_reset = floorf(imu.gyro.integrator.get_reset_samples() / samples_per_pub);

								sub.set_required_updates(math::constrain(pubs_per_integrator_reset, (uint8_t)1, sensor_imu_fifo_s::ORB_QUEUE_LENGTH));
							}

							sub.registerCallback();

							imu.accel.interval_configured = true;
							imu.gyro.interval_configured = true;

							PX4_INFO("IMU FIFO (%" PRIu32 "), publish interval: %.1f us (STD:%.1f us), integrator: %.1f us, %d samples",
								 imu.gyro.calibration.device_id(),
								 (double)imu.gyro.mean_publish_interval_us.mean(), (double)imu.gyro.mean_publish_interval_us.standard_deviation(),
								 (double)imu.gyro.integrator.reset_interval_us(), imu.gyro.integrator.get_reset_samples()
								);
						}

					}
				}
			}
		}

		imu.accel.timestamp_sample_last = sensor_imu_fifo.timestamp_sample;
		imu.accel.calibration.set_device_id(sensor_imu_fifo.device_id);
		imu.accel.calibration.SensorCorrectionsUpdate();
		imu.accel.error_count = sensor_imu_fifo.error_count;

		imu.gyro.timestamp_sample_last = sensor_imu_fifo.timestamp_sample;
		imu.gyro.calibration.set_device_id(sensor_imu_fifo.device_id);
		imu.gyro.calibration.SensorCorrectionsUpdate();
		imu.gyro.error_count = sensor_imu_fifo.error_count;

		// temperature average
		if (PX4_ISFINITE(sensor_imu_fifo.temperature)) {
			imu.accel.temperature.update(sensor_imu_fifo.temperature);
			imu.gyro.temperature.update(sensor_imu_fifo.temperature);
		}

		if (!imu.accel.calibration.enabled() || !imu.gyro.calibration.enabled()) {
			return;
		}

		const int N = sensor_imu_fifo.samples;

		const float dt_s = sensor_imu_fifo.dt * 1e-6f;
		//const float inverse_dt_s = 1e6f / sensor_imu_fifo.dt;

		// integrate accel
		for (int n = 0; n < N; n++) {
			const Vector3f accel_raw{
				static_cast<float>(sensor_imu_fifo.accel_x[n]) *sensor_imu_fifo.accel_scale,
				static_cast<float>(sensor_imu_fifo.accel_y[n]) *sensor_imu_fifo.accel_scale,
				static_cast<float>(sensor_imu_fifo.accel_z[n]) *sensor_imu_fifo.accel_scale};

			imu.accel.integrator.put(accel_raw, dt_s);
		}

		const Vector3f accel_raw_avg{AverageFifoAccel(sensor_imu_fifo)};
		imu.accel.raw_mean.update(accel_raw_avg - imu.accel.calibration.thermal_offset());

		// integrate accel

		// check for scale change
		if (fabsf(sensor_imu_fifo.accel_scale - imu.accel.fifo_scale) > FLT_EPSILON) {
			// rescale last sample on scale change
			const float rescale = imu.accel.fifo_scale / sensor_imu_fifo.accel_scale;

			imu.accel.last_raw_sample[0] = roundf(imu.accel.last_raw_sample[0] * rescale);
			imu.accel.last_raw_sample[1] = roundf(imu.accel.last_raw_sample[1] * rescale);
			imu.accel.last_raw_sample[2] = roundf(imu.accel.last_raw_sample[2] * rescale);

			imu.accel.fifo_scale = sensor_imu_fifo.accel_scale;
		}

		// trapezoidal integration (equally spaced)
		const float scale = dt_s * sensor_imu_fifo.accel_scale;
		imu.accel.integral(0) += (0.5f * (imu.accel.last_raw_sample[0] + sensor_imu_fifo.accel_x[N - 2])
					  + sum(sensor_imu_fifo.accel_x, N - 1)) * scale;
		imu.accel.last_raw_sample[0] = sensor_imu_fifo.accel_x[N - 1];

		imu.accel.integral(1) += (0.5f * (imu.accel.last_raw_sample[1] + sensor_imu_fifo.accel_y[N - 2])
					  + sum(sensor_imu_fifo.accel_y, N - 1)) * scale;
		imu.accel.last_raw_sample[1] = sensor_imu_fifo.accel_y[N - 1];

		imu.accel.integral(2) += (0.5f * (imu.accel.last_raw_sample[2] + sensor_imu_fifo.accel_z[N - 2])
					  + sum(sensor_imu_fifo.accel_z, N - 1)) * scale;
		imu.accel.last_raw_sample[2] = sensor_imu_fifo.accel_z[N - 1];

		// accel clipping
		{
			const Vector3f clip_counter{
				(float)clipping(sensor_imu_fifo.accel_x, sensor_imu_fifo.samples),
				(float)clipping(sensor_imu_fifo.accel_y, sensor_imu_fifo.samples),
				(float)clipping(sensor_imu_fifo.accel_z, sensor_imu_fifo.samples)};

			if (clip_counter(0) > 0 || clip_counter(1) > 0 || clip_counter(2) > 0) {
				// rotate sensor clip counts into vehicle body frame
				const Vector3f clipping{imu.accel.calibration.rotation() *clip_counter};

				// round to get reasonble clip counts per axis (after board rotation)
				const uint8_t clip_x = roundf(fabsf(clipping(0)));

				if (clip_x > 0) {
					imu.accel.clipping_total[0] += clip_x;
					imu.accel.clipping_flags |= vehicle_imu_s::CLIPPING_X;
				}

				const uint8_t clip_y = roundf(fabsf(clipping(1)));

				if (clip_y > 0) {
					imu.accel.clipping_total[1] += clip_y;
					imu.accel.clipping_flags |= vehicle_imu_s::CLIPPING_Y;
				}

				const uint8_t clip_z = roundf(fabsf(clipping(2)));

				if (clip_z > 0) {
					imu.accel.clipping_total[2] += clip_z;
					imu.accel.clipping_flags |= vehicle_imu_s::CLIPPING_Z;
				}
			}
		}

		// integrate gyro
		for (int n = 0; n < N; n++) {
			const Vector3f gyro_raw{
				static_cast<float>(sensor_imu_fifo.gyro_x[n]) *sensor_imu_fifo.gyro_scale,
				static_cast<float>(sensor_imu_fifo.gyro_y[n]) *sensor_imu_fifo.gyro_scale,
				static_cast<float>(sensor_imu_fifo.gyro_z[n]) *sensor_imu_fifo.gyro_scale};

			imu.gyro.integrator.put(gyro_raw, dt_s);
		}

		imu.gyro.raw_mean.update(AverageFifoGyro(sensor_imu_fifo) - imu.gyro.calibration.thermal_offset());

		// gyro clipping
		{
			const Vector3f clip_counter{
				(float)clipping(sensor_imu_fifo.gyro_x, sensor_imu_fifo.samples),
				(float)clipping(sensor_imu_fifo.gyro_y, sensor_imu_fifo.samples),
				(float)clipping(sensor_imu_fifo.gyro_z, sensor_imu_fifo.samples)};


			if (clip_counter(0) > 0 || clip_counter(1) > 0 || clip_counter(2) > 0) {
				// rotate sensor clip counts into vehicle body frame
				const Vector3f clipping{imu.gyro.calibration.rotation() *clip_counter};

				// round to get reasonble clip counts per axis (after board rotation)
				const uint8_t clip_x = roundf(fabsf(clipping(0)));

				if (clip_x > 0) {
					imu.gyro.clipping_total[0] += clip_x;
					imu.gyro.clipping_flags |= vehicle_imu_s::CLIPPING_X;
				}

				const uint8_t clip_y = roundf(fabsf(clipping(1)));

				if (clip_y > 0) {
					imu.gyro.clipping_total[1] += clip_y;
					imu.gyro.clipping_flags |= vehicle_imu_s::CLIPPING_Y;
				}

				const uint8_t clip_z = roundf(fabsf(clipping(2)));

				if (clip_z > 0) {
					imu.gyro.clipping_total[2] += clip_z;
					imu.gyro.clipping_flags |= vehicle_imu_s::CLIPPING_Z;
				}
			}
		}

		if (imu.accel.integrator.integral_ready() && imu.gyro.integrator.integral_ready()) {


			// Vector3f delta_velocity{imu.accel.integral * imu.accel.fifo_scale * dt_s};
			// imu.accel.integral.zero();
			// uint16_t delta_velocity_dt = delta_angle_dt;

			// TODO: rescale if changed?



			Vector3f delta_angle;
			uint16_t delta_angle_dt;
			Vector3f delta_velocity;
			uint16_t delta_velocity_dt;

			const Vector3f accumulated_coning_corrections = imu.gyro.integrator.accumulated_coning_corrections();

			if (imu.accel.integrator.reset(delta_velocity, delta_velocity_dt)
			    && imu.gyro.integrator.reset(delta_angle, delta_angle_dt)
			   ) {

				// TODO:
				// delta_velocity = imu.accel.integral; // TODO

				PublishImu(imu, delta_angle, delta_angle_dt, delta_velocity, delta_velocity_dt);
			}

			imu.accel.integral.zero();
		}

		// TODO: call to filter data
		if (imu.primary) {

			if (imu.accel.interval_configured && imu.accel.mean_publish_interval_us.valid()) {
				float sample_rate_hz = 1e6f / imu.accel.mean_publish_interval_us.mean();
				_vehicle_acceleration.updateAccel(imu, accel_raw_avg, sample_rate_hz);
			}

			if (imu.gyro.interval_configured) {

				// TODO: generic update?
				//_vehicle_angular_velocity.update(imu)?
				// for (int axis = 0; axis < 3; axis++) {
				//	_vehicle_angular_velocity.updateAxis(axis, raw_data[], length)?
				// }

				_vehicle_angular_velocity.updateSensorImuFifo(imu, sensor_imu_fifo);

				// TODO: or do it axis by axis?
				//  float raw_data_array[]

				// update per axis?
				// publish when
				if (!sub.updated()) {
					// _vehicle_angular_velocity.publish()?
				}
			}
		}
	}
}

void VehicleIMU::UpdateSensorAccel(uint8_t sensor_instance)
{
	// sensor_accel
	auto &sub = _sensor_accel_subs[sensor_instance];

	while (sub.updated()) {
		unsigned last_generation = sub.get_last_generation();
		sensor_accel_s sensor_accel;
		sub.copy(&sensor_accel);

		int8_t imu_instance = findAccelInstance(sensor_accel.device_id);

		if (imu_instance == -1) {
			return;
		}

		IMU &imu = _imus[imu_instance];

		const float interval_us = sensor_accel.timestamp_sample - imu.accel.timestamp_sample_last;

		// publication interval
		//  sensor output data rate or interval
		if (sub.get_last_generation() == last_generation + 1) {

			if (sensor_accel.timestamp_sample > imu.accel.timestamp_sample_last) {

				const float sample_interval_us = interval_us;

				imu.accel.mean_publish_interval_us.update(interval_us);
				imu.accel.mean_sample_interval_us.update(sample_interval_us);
			}
		}

		const float dt_s = math::constrain(interval_us * 1e-6f, 0.0001f, 0.1f);

		imu.accel.timestamp_sample_last = sensor_accel.timestamp_sample;
		imu.accel.calibration.set_device_id(sensor_accel.device_id);
		imu.accel.calibration.SensorCorrectionsUpdate();
		imu.accel.error_count = sensor_accel.error_count;

		// temperature average
		if (PX4_ISFINITE(sensor_accel.temperature)) {
			imu.accel.temperature.update(sensor_accel.temperature);
		}

		if (!imu.accel.calibration.enabled()) {
			return;
		}

		// integrate accel
		const Vector3f accel_raw{sensor_accel.x, sensor_accel.y, sensor_accel.z};
		imu.accel.integrator.put(accel_raw, dt_s);

		imu.accel.raw_mean.update(accel_raw - imu.accel.calibration.thermal_offset());

		// TODO: call to filter data
		if (imu.primary) {

			if (imu.accel.interval_configured && imu.accel.mean_publish_interval_us.valid()) {
				float sample_rate_hz = 1e6f / imu.accel.mean_publish_interval_us.mean();
				_vehicle_acceleration.updateAccel(imu, accel_raw, sample_rate_hz);
			}
		}

		// don't get ahead of gyro
		if (imu.accel.timestamp_sample_last > imu.gyro.timestamp_sample_last
		    + 0.5f * imu.accel.mean_sample_interval_us.mean()
		   ) {
			return;
		}
	}
}

void VehicleIMU::UpdateSensorGyro(uint8_t sensor_instance)
{
	// sensor_gyro
	auto &sub = _sensor_gyro_subs[sensor_instance];

	while (sub.updated()) {
		unsigned last_generation = sub.get_last_generation();
		sensor_gyro_s sensor_gyro;
		sub.copy(&sensor_gyro);

		int8_t imu_instance = findAccelInstance(sensor_gyro.device_id);

		if (imu_instance == -1) {
			return;
		}

		IMU &imu = _imus[imu_instance];

		// publication interval
		//  sensor output data rate or interval
		if (sub.get_last_generation() == last_generation + 1) {

			if (sensor_gyro.timestamp_sample > imu.accel.timestamp_sample_last) {

				const float interval_us = sensor_gyro.timestamp_sample - imu.accel.timestamp_sample_last;
				const float sample_interval_us = interval_us;

				imu.accel.mean_publish_interval_us.update(interval_us);
				imu.accel.mean_sample_interval_us.update(sample_interval_us);

				imu.gyro.mean_publish_interval_us.update(interval_us);
				imu.gyro.mean_sample_interval_us.update(sample_interval_us);

				// check and update sensor rate
				if (imu.gyro.mean_publish_interval_us.valid() &&
				    (imu.gyro.mean_publish_interval_us.count() > 1000 || imu.gyro.mean_publish_interval_us.standard_deviation() < 100)
				   ) {

					// determine number of sensor samples that will get closest to the desired integration interval
					const float imu_integration_interval_us = 1e6f / _param_imu_integ_rate.get();
					const float pub_interval_avg_us = imu.gyro.mean_publish_interval_us.mean();

					const uint8_t imu_publications = roundf(imu_integration_interval_us / pub_interval_avg_us);

					const float integration_interval_us = roundf(imu_publications * pub_interval_avg_us);

					// TODO: variance
					if ((static_cast<float>(imu.gyro.integrator.reset_interval_us()) - integration_interval_us) >
					    imu.gyro.mean_publish_interval_us.standard_deviation()
					    // || primary_changed
					   ) {

						const auto accel_reset_samples_prev = imu.accel.integrator.get_reset_samples();
						const auto gyro_reset_samples_prev = imu.gyro.integrator.get_reset_samples();

						imu.gyro.integrator.set_reset_interval(integration_interval_us);
						imu.accel.integrator.set_reset_interval(integration_interval_us / 2.f);

						// number of samples per publication
						int interval_samples = roundf(integration_interval_us / imu.gyro.mean_sample_interval_us.mean());
						imu.gyro.integrator.set_reset_samples(math::max(interval_samples, 1));
						imu.accel.integrator.set_reset_samples(1);

						if (accel_reset_samples_prev != imu.accel.integrator.get_reset_samples()
						    || gyro_reset_samples_prev != imu.gyro.integrator.get_reset_samples()
						    || (static_cast<float>(imu.gyro.integrator.reset_interval_us()) - integration_interval_us > 100)
						   ) {

							if (imu.primary) {
								sub.set_required_updates(1);

							} else {
								// avg samples per publication
								float samples_per_pub = imu.gyro.mean_publish_interval_us.mean() / imu.gyro.mean_sample_interval_us.mean();
								uint8_t pubs_per_integrator_reset = floorf(imu.gyro.integrator.get_reset_samples() / samples_per_pub);

								sub.set_required_updates(math::constrain(pubs_per_integrator_reset, (uint8_t)1, sensor_gyro_s::ORB_QUEUE_LENGTH));
							}

							sub.registerCallback();

							imu.accel.interval_configured = true;
							imu.gyro.interval_configured = true;

							PX4_INFO("IMU FIFO (%" PRIu32 "), publish interval: %.1f us (STD:%.1f us), integrator: %.1f us, %d samples",
								 imu.gyro.calibration.device_id(),
								 (double)imu.gyro.mean_publish_interval_us.mean(), (double)imu.gyro.mean_publish_interval_us.standard_deviation(),
								 (double)imu.gyro.integrator.reset_interval_us(), imu.gyro.integrator.get_reset_samples()
								);
						}

					}
				}
			}
		}

		const float dt_s = math::constrain((sensor_gyro.timestamp_sample - imu.gyro.timestamp_sample_last) * 1e-6f, 0.0001f,
						   0.1f);

		imu.gyro.timestamp_sample_last = sensor_gyro.timestamp_sample;
		imu.gyro.calibration.set_device_id(sensor_gyro.device_id);
		imu.gyro.calibration.SensorCorrectionsUpdate();
		imu.gyro.error_count = sensor_gyro.error_count;

		// temperature average
		if (PX4_ISFINITE(sensor_gyro.temperature)) {
			imu.gyro.temperature.update(sensor_gyro.temperature);
		}

		if (!imu.gyro.calibration.enabled()) {
			return;
		}

		// integrate gyro
		const Vector3f gyro_raw{sensor_gyro.x, sensor_gyro.y, sensor_gyro.z};
		imu.gyro.integrator.put(gyro_raw, dt_s);
		imu.gyro.raw_mean.update(gyro_raw - imu.gyro.calibration.thermal_offset());

		if (imu.accel.integrator.integral_ready() && imu.gyro.integrator.integral_ready()) {

			// Vector3f delta_velocity{imu.accel.integral * imu.accel.fifo_scale * dt_s};
			// imu.accel.integral.zero();
			// uint16_t delta_velocity_dt = delta_angle_dt;

			// TODO: rescale if changed?
			Vector3f delta_angle;
			uint16_t delta_angle_dt;
			Vector3f delta_velocity;
			uint16_t delta_velocity_dt;

			const Vector3f accumulated_coning_corrections = imu.gyro.integrator.accumulated_coning_corrections();

			if (imu.accel.integrator.reset(delta_velocity, delta_velocity_dt)
			    && imu.gyro.integrator.reset(delta_angle, delta_angle_dt)
			   ) {

				PublishImu(imu, delta_angle, delta_angle_dt, delta_velocity, delta_velocity_dt);
			}

			imu.accel.integral.zero();
		}

		// TODO: call to filter data
		if (imu.primary) {

			if (imu.gyro.interval_configured) {

				// TODO: generic update?
				//_vehicle_angular_velocity.update(imu)?
				// for (int axis = 0; axis < 3; axis++) {
				//	_vehicle_angular_velocity.updateAxis(axis, raw_data[], length)?
				// }

				_vehicle_angular_velocity.updateSensorGyro(imu, sensor_gyro);

				// TODO: or do it axis by axis?
				//  float raw_data_array[]

				// update per axis?
				// publish when
				if (!sub.updated()) {
					// _vehicle_angular_velocity.publish()?
				}
			}
		}
	}
}

bool VehicleIMU::PublishImu(sensors::IMU &imu,
			    const Vector3f &delta_angle, const uint16_t delta_angle_dt,
			    const Vector3f &delta_velocity, const uint16_t delta_velocity_dt
			   )
{
	bool updated = false;

	// TODO: fix
	const Vector3f accumulated_coning_corrections = imu.gyro.integrator.accumulated_coning_corrections();

	if (imu.accel.calibration.enabled() && imu.gyro.calibration.enabled()) {

		// delta angle: apply offsets, scale, and board rotation
		const float gyro_dt_s = 1.e-6f * delta_angle_dt;
		const Vector3f angular_velocity{imu.gyro.calibration.Correct(delta_angle / gyro_dt_s)};

		// Gyro high frequency vibe = filtered length of (angular_velocity - angular_velocity_prev)
		imu.gyro.vibration_metric = 0.99f * imu.gyro.vibration_metric + 0.01f * Vector3f(angular_velocity -
					    imu.gyro.angular_velocity_prev).norm();
		imu.gyro.angular_velocity_prev = angular_velocity;

		const Vector3f delta_angle_corrected{angular_velocity * gyro_dt_s};

		// accumulate delta angle coning corrections
		imu.gyro.coning_norm_accum += accumulated_coning_corrections.norm() * gyro_dt_s;
		imu.gyro.coning_norm_accum_total_time_s += gyro_dt_s;

		// delta velocity: apply offsets, scale, and board rotation
		const float accel_dt_s = 1.e-6f * delta_velocity_dt;
		const Vector3f acceleration{imu.accel.calibration.Correct(delta_velocity / accel_dt_s)};

		// Accel high frequency vibe = filtered length of (acceleration - acceleration_prev)
		imu.accel.vibration_metric = 0.99f * imu.accel.vibration_metric + 0.01f * Vector3f(acceleration -
					     imu.accel.acceleration_prev).norm();
		imu.accel.acceleration_prev = acceleration;

		const Vector3f delta_velocity_corrected{acceleration * accel_dt_s};


		// at rest determination
		static constexpr float GYRO_VIBE_METRIC_MAX = 0.02f; // gyro_vibration_metric * dt * 4.0e4f > is_moving_scaler)
		static constexpr float ACCEL_VIBE_METRIC_MAX = 1.2f; // accel_vibration_metric * dt * 2.1e2f > is_moving_scaler
		static constexpr float GYRO_NORM_MAX = math::radians(10.f);

		const float acceleration_norm = acceleration.norm();
		const float angular_velocity_norm = angular_velocity.norm();

		if ((imu.gyro.vibration_metric > GYRO_VIBE_METRIC_MAX)
		    || (imu.accel.vibration_metric > ACCEL_VIBE_METRIC_MAX)
		    || (acceleration_norm < 0.5f * CONSTANTS_ONE_G)
		    || (acceleration_norm > 1.5f * CONSTANTS_ONE_G)
		    || (angular_velocity_norm > GYRO_NORM_MAX)
		    || (imu.accel.raw_mean.variance().min() > 0.1f)
		    || (imu.gyro.raw_mean.variance().min() > 0.1f)
		   ) {
			imu.time_last_move_detect_us = hrt_absolute_time();
			imu.time_still_detect_us = 0;

		} else {
			if (imu.time_still_detect_us == 0) {
				imu.time_still_detect_us = hrt_absolute_time();
			}
		}

		const bool at_rest = (hrt_elapsed_time(&imu.time_last_move_detect_us) > 400_ms)
				     && (imu.time_still_detect_us != 0) && (hrt_elapsed_time(&imu.time_still_detect_us) > 10_ms);

		if (imu.at_rest != at_rest) {
			PX4_DEBUG("IMU at rest %d->%d (accel:%d gyro:%d)", imu.at_rest, at_rest, imu.accel.calibration.device_id(),
				  imu.gyro.calibration.device_id());

			imu.at_rest = at_rest;
		}


		// vehicle_imu_status
		//  publish before vehicle_imu so that error counts are available synchronously if needed
		const bool status_publish_interval_exceeded = (hrt_elapsed_time(&imu.time_last_status_publication) >= 100_ms);

		if (imu.accel.raw_mean.valid() && imu.gyro.raw_mean.valid()
		    && imu.accel.mean_sample_interval_us.valid() && imu.gyro.mean_sample_interval_us.valid()
		    && status_publish_interval_exceeded
		   ) {

			vehicle_imu_status_s imu_status{};

			// Accel
			{
				imu_status.accel_device_id = imu.accel.calibration.device_id();

				imu_status.accel_error_count = imu.accel.error_count;

				imu_status.accel_clipping[0] = imu.accel.clipping_total[0];
				imu_status.accel_clipping[1] = imu.accel.clipping_total[1];
				imu_status.accel_clipping[2] = imu.accel.clipping_total[2];

				imu_status.accel_rate_hz = 1e6f / imu.accel.mean_publish_interval_us.mean();
				imu_status.accel_raw_rate_hz = 1e6f / imu.accel.mean_sample_interval_us.mean();

				// accel mean and variance
				const Dcmf &R = imu.accel.calibration.rotation();
				Vector3f(R * imu.accel.raw_mean.mean()).copyTo(imu_status.mean_accel);

				// variance from R * COV * R^T
				const Matrix3f cov = R * imu.accel.raw_mean.covariance() * R.transpose();
				cov.diag().copyTo(imu_status.var_accel);

				imu_status.accel_vibration_metric = imu.accel.vibration_metric;

				// temperature
				if (imu.accel.temperature.valid()) {
					imu_status.temperature_accel = imu.accel.temperature.mean();

				} else {
					imu_status.temperature_accel = NAN;
				}
			}

			// Gyro
			{
				imu_status.gyro_device_id = imu.gyro.calibration.device_id();

				imu_status.gyro_error_count = imu.gyro.error_count;

				imu_status.gyro_clipping[0] = imu.gyro.clipping_total[0];
				imu_status.gyro_clipping[1] = imu.gyro.clipping_total[1];
				imu_status.gyro_clipping[2] = imu.gyro.clipping_total[2];

				imu_status.gyro_rate_hz = 1e6f / imu.gyro.mean_publish_interval_us.mean();
				imu_status.gyro_raw_rate_hz = 1e6f / imu.gyro.mean_sample_interval_us.mean();

				// gyro mean and variance
				const Dcmf &R = imu.gyro.calibration.rotation();
				Vector3f(R * imu.gyro.raw_mean.mean()).copyTo(imu_status.mean_gyro);

				// variance from R * COV * R^T
				const Matrix3f cov = R * imu.gyro.raw_mean.covariance() * R.transpose();
				cov.diag().copyTo(imu_status.var_gyro);

				imu_status.gyro_vibration_metric = imu.gyro.vibration_metric;

				// Gyro delta angle coning metric = length of coning corrections averaged since last status publication
				imu_status.delta_angle_coning_metric = imu.gyro.coning_norm_accum / imu.gyro.coning_norm_accum_total_time_s;
				imu.gyro.coning_norm_accum = 0;
				imu.gyro.coning_norm_accum_total_time_s = 0;

				// temperature
				if (imu.gyro.temperature.valid()) {
					imu_status.temperature_gyro = imu.gyro.temperature.mean();

				} else {
					imu_status.temperature_gyro = NAN;
				}
			}

			// publish
			imu_status.timestamp = hrt_absolute_time();
			imu.vehicle_imu_status_pub.publish(imu_status);

			imu.time_last_status_publication = imu_status.timestamp;

			if (status_publish_interval_exceeded && !at_rest) {
				imu.accel.raw_mean.reset();
				imu.gyro.raw_mean.reset();
			}
		}

		// publish vehicle_imu
		vehicle_imu_s vehicle_imu{};
		vehicle_imu.timestamp_sample = imu.gyro.timestamp_sample_last;
		vehicle_imu.accel_device_id = imu.accel.calibration.device_id();
		vehicle_imu.gyro_device_id = imu.gyro.calibration.device_id();
		delta_angle_corrected.copyTo(vehicle_imu.delta_angle);
		delta_velocity_corrected.copyTo(vehicle_imu.delta_velocity);
		vehicle_imu.delta_angle_dt = delta_angle_dt;
		vehicle_imu.delta_velocity_dt = delta_velocity_dt;
		vehicle_imu.delta_angle_clipping = imu.gyro.clipping_flags;
		vehicle_imu.delta_velocity_clipping = imu.accel.clipping_flags;
		vehicle_imu.accel_calibration_count = imu.accel.calibration.calibration_count();
		vehicle_imu.gyro_calibration_count = imu.gyro.calibration.calibration_count();
		vehicle_imu.timestamp = hrt_absolute_time();
		imu.vehicle_imu_pub.publish(vehicle_imu);



		if (delta_angle_corrected.isAllFinite() && delta_velocity_corrected.isAllFinite()) {
			imu.time_last_valid = vehicle_imu.timestamp;

		} else {
			imu.time_last_invalid = vehicle_imu.timestamp;
		}


#if 0
		_accel.voter.put(uorb_index, imu_report.timestamp, _last_sensor_data[uorb_index].accelerometer_m_s2,
				 imu_status.accel_error_count, _accel.priority[uorb_index]);

		_gyro.voter.put(uorb_index, imu_report.timestamp, _last_sensor_data[uorb_index].gyro_rad,
				imu_status.gyro_error_count, _gyro.priority[uorb_index]);


		// find the best sensor
		int accel_best_index = _accel.last_best_vote;
		int gyro_best_index = _gyro.last_best_vote;

		if (!_parameter_update) {
			// update current accel/gyro selection, skipped on cycles where parameters update
			_accel.voter.get_best(time_now_us, &accel_best_index);
			_gyro.voter.get_best(time_now_us, &gyro_best_index);

			if (!_param_sens_imu_mode.get() && ((_selection.timestamp != 0) || (_sensor_selection_sub.updated()))) {
				// use sensor_selection to find best
				if (_sensor_selection_sub.update(&_selection)) {
					// reset inconsistency checks against primary
					for (int sensor_index = 0; sensor_index < MAX_SENSOR_COUNT; sensor_index++) {
						_accel_diff[sensor_index].zero();
						_gyro_diff[sensor_index].zero();
					}
				}

				for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
					if ((_accel_device_id[i] != 0) && (_accel_device_id[i] == _selection.accel_device_id)) {
						accel_best_index = i;
					}

					if ((_gyro_device_id[i] != 0) && (_gyro_device_id[i] == _selection.gyro_device_id)) {
						gyro_best_index = i;
					}
				}

			} else {
				// use sensor voter to find best if SENS_IMU_MODE is enabled or ORB_ID(sensor_selection) has never published
				checkFailover(_accel, "Accel", events::px4::enums::sensor_type_t::accel);
				checkFailover(_gyro, "Gyro", events::px4::enums::sensor_type_t::gyro);
			}
		}


		if (!sensor_data.advertised[i] && imu_sub.advertised()) {
			sensor_data.advertised[i] = true;
			sensor_data.priority[i] = DEFAULT_PRIORITY;
			sensor_data.priority_configured[i] = DEFAULT_PRIORITY;

			if (i > 0) {
				/* the first always exists, but for each further sensor, add a new validator */
				if (sensor_data.voter.add_new_validator()) {
					added = true;

				} else {
					PX4_ERR("failed to add validator for sensor %i", i);
				}
			}
		}



#endif

		if (imu.primary) {



			// publish sensor_combined
			sensor_combined_s sensor_combined{};

			sensor_combined.timestamp = vehicle_imu.timestamp_sample;
			sensor_combined.accelerometer_m_s2[0] = acceleration(0);
			sensor_combined.accelerometer_m_s2[1] = acceleration(1);
			sensor_combined.accelerometer_m_s2[2] = acceleration(2);
			sensor_combined.accelerometer_integral_dt = vehicle_imu.delta_velocity_dt;
			sensor_combined.accelerometer_clipping = vehicle_imu.delta_velocity_clipping;
			sensor_combined.gyro_rad[0] = angular_velocity(0);
			sensor_combined.gyro_rad[1] = angular_velocity(1);
			sensor_combined.gyro_rad[2] = angular_velocity(2);
			sensor_combined.gyro_integral_dt = vehicle_imu.delta_angle_dt;
			sensor_combined.gyro_clipping = vehicle_imu.delta_angle_clipping;
			sensor_combined.accel_calibration_count = vehicle_imu.accel_calibration_count;
			sensor_combined.gyro_calibration_count = vehicle_imu.gyro_calibration_count;

			sensor_combined.accelerometer_timestamp_relative = (int32_t)((int64_t)imu.accel.timestamp_sample_last -
					(int64_t)imu.gyro.timestamp_sample_last);

			_sensor_combined_pub.publish(sensor_combined);
		}






		// reset clipping flags
		imu.gyro.clipping_flags = 0;
		imu.accel.clipping_flags = 0;

		// record gyro publication latency and integrated samples
		//_gyro_publish_latency_mean_us.update(imu.timestamp - _gyro_timestamp_last);
		//_gyro_update_latency_mean_us.update(imu.timestamp - _gyro_timestamp_sample_last);

		updated = true;
	}

	return updated;
}

void VehicleIMU::PublishSensorsStatusIMU()
{
	// publish sensors_status_imu
	sensors_status_imu_s status{};

	for (const auto &imu : _imus) {
		if (imu.primary) {
			status.accel_device_id_primary = imu.accel.calibration.device_id();
			status.gyro_device_id_primary = imu.gyro.calibration.device_id();
			break;
		}
	}

	static_assert(MAX_SENSOR_COUNT == (sizeof(sensors_status_imu_s::accel_inconsistency_m_s_s) / sizeof(
			sensors_status_imu_s::accel_inconsistency_m_s_s[0])), "check sensors_status_imu accel_inconsistency_m_s_s size");
	static_assert(MAX_SENSOR_COUNT == (sizeof(sensors_status_imu_s::gyro_inconsistency_rad_s) / sizeof(
			sensors_status_imu_s::gyro_inconsistency_rad_s[0])), "check sensors_status_imu accel_inconsistency_m_s_s size");


	{
		// accel diff
		Vector3f accel_mean{};
		Vector3f accel_all[MAX_SENSOR_COUNT] {};
		uint8_t accel_count = 0;

		for (int sensor_index = 0; sensor_index < MAX_SENSOR_COUNT; sensor_index++) {
			const auto &accel = _imus[sensor_index].accel;

			if ((accel.calibration.device_id() != 0) && accel.calibration.enabled()) {
				accel_count++;
				accel_all[sensor_index] = accel.acceleration_prev;
				accel_mean += accel_all[sensor_index];
			}
		}

		if (accel_count > 0) {
			accel_mean /= accel_count;

			for (int sensor_index = 0; sensor_index < MAX_SENSOR_COUNT; sensor_index++) {
				const auto &accel = _imus[sensor_index].accel;

				if ((accel.calibration.device_id() != 0) && accel.calibration.enabled()) {
					_accel_diff[sensor_index] = 0.95f * _accel_diff[sensor_index] + 0.05f * (accel_all[sensor_index] - accel_mean);
				}
			}
		}
	}

	{
		// gyro diff
		Vector3f gyro_mean {};
		Vector3f gyro_all[MAX_SENSOR_COUNT] {};
		uint8_t gyro_count = 0;

		for (int sensor_index = 0; sensor_index < MAX_SENSOR_COUNT; sensor_index++) {
			const auto &gyro = _imus[sensor_index].gyro;

			if ((gyro.calibration.device_id() != 0) && gyro.calibration.enabled()) {
				gyro_count++;
				gyro_all[sensor_index] = gyro.angular_velocity_prev;
				gyro_mean += gyro_all[sensor_index];
			}
		}

		if (gyro_count > 0) {
			gyro_mean /= gyro_count;

			for (int sensor_index = 0; sensor_index < MAX_SENSOR_COUNT; sensor_index++) {
				for (int sensor_index = 0; sensor_index < MAX_SENSOR_COUNT; sensor_index++) {
					const auto &gyro = _imus[sensor_index].gyro;

					if ((gyro.calibration.device_id() != 0) && gyro.calibration.enabled()) {
						_gyro_diff[sensor_index] = 0.95f * _gyro_diff[sensor_index] + 0.05f * (gyro_all[sensor_index] - gyro_mean);
					}
				}
			}
		}
	}



	for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
		const auto &current_imu = _imus[i];

		if ((current_imu.accel.calibration.device_id() != 0) && current_imu.accel.calibration.enabled()) {
			status.accel_device_ids[i] = current_imu.accel.calibration.device_id();
			status.accel_inconsistency_m_s_s[i] = _accel_diff[i].norm();
			status.accel_healthy[i] = true; // TODO (_accel.voter.get_sensor_state(i) == DataValidator::ERROR_FLAG_NO_ERROR);
			status.accel_priority[i] = current_imu.accel.calibration.priority();
		}

		if ((current_imu.gyro.calibration.device_id() != 0) && current_imu.gyro.calibration.enabled()) {
			status.gyro_device_ids[i] = current_imu.gyro.calibration.device_id();
			status.gyro_inconsistency_rad_s[i] = _gyro_diff[i].norm();
			status.gyro_healthy[i] = true; // TODO (_accel.voter.get_sensor_state(i) == DataValidator::ERROR_FLAG_NO_ERROR);
			status.gyro_priority[i] = current_imu.gyro.calibration.priority();
		}
	}

	status.timestamp = hrt_absolute_time();
	_sensors_status_imu_pub.publish(status);

}

#if 0
void VehicleIMU::SensorCalibrationSaveAccel()
{
	if (_accel_cal_available) {
		const Vector3f cal_orig{_accel_calibration.offset()};
		bool initialised = false;
		Vector3f offset_estimate{};
		Vector3f bias_variance{};

		// apply all valid saved offsets
		for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
			if (_accel_learned_calibration[i].valid) {
				if (!initialised) {
					bias_variance = _accel_learned_calibration[i].bias_variance;
					offset_estimate = _accel_learned_calibration[i].offset;
					initialised = true;

				} else {
					for (int axis_index = 0; axis_index < 3; axis_index++) {
						const float sum_of_variances = _accel_learned_calibration[i].bias_variance(axis_index) + bias_variance(axis_index);
						const float k1 = bias_variance(axis_index) / sum_of_variances;
						const float k2 = _accel_learned_calibration[i].bias_variance(axis_index) / sum_of_variances;
						offset_estimate(axis_index) = k2 * offset_estimate(axis_index) + k1 * _accel_learned_calibration[i].offset(axis_index);
						bias_variance(axis_index) *= k2;
					}
				}

				// reset
				_accel_learned_calibration[i] = {};
			}
		}

		if (initialised && ((cal_orig - offset_estimate).longerThan(0.05f) || !_accel_calibration.calibrated())) {
			if (_accel_calibration.set_offset(offset_estimate)) {
				PX4_INFO("%s %d (%" PRIu32 ") offset committed: [%.3f %.3f %.3f]->[%.3f %.3f %.3f])",
					 _accel_calibration.SensorString(), _instance, _accel_calibration.device_id(),
					 (double)cal_orig(0), (double)cal_orig(1), (double)cal_orig(2),
					 (double)offset_estimate(0), (double)offset_estimate(1), (double)offset_estimate(2));

				// save parameters with preferred calibration slot to current sensor index
				if (_accel_calibration.ParametersSave(_sensor_accel_sub.get_instance())) {
					param_notify_changes();
				}

				_in_flight_calibration_check_timestamp_last = hrt_absolute_time() + INFLIGHT_CALIBRATION_QUIET_PERIOD_US;
			}
		}

		// reset
		_accel_cal_available = false;
	}
}

void VehicleIMU::SensorCalibrationSaveGyro()
{
	if (_gyro_cal_available) {
		const Vector3f cal_orig{_gyro_calibration.offset()};
		bool initialised = false;
		Vector3f offset_estimate{};
		Vector3f bias_variance{};

		// apply all valid saved offsets
		for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
			if (_gyro_learned_calibration[i].valid) {
				if (!initialised) {
					bias_variance = _gyro_learned_calibration[i].bias_variance;
					offset_estimate = _gyro_learned_calibration[i].offset;
					initialised = true;

				} else {
					for (int axis_index = 0; axis_index < 3; axis_index++) {
						const float sum_of_variances = _gyro_learned_calibration[i].bias_variance(axis_index) + bias_variance(axis_index);
						const float k1 = bias_variance(axis_index) / sum_of_variances;
						const float k2 = _gyro_learned_calibration[i].bias_variance(axis_index) / sum_of_variances;
						offset_estimate(axis_index) = k2 * offset_estimate(axis_index) + k1 * _gyro_learned_calibration[i].offset(axis_index);
						bias_variance(axis_index) *= k2;
					}
				}

				// reset
				_gyro_learned_calibration[i] = {};
			}
		}

		if (initialised && ((cal_orig - offset_estimate).longerThan(0.01f) || !_gyro_calibration.calibrated())) {
			if (_gyro_calibration.set_offset(offset_estimate)) {
				PX4_INFO("%s %d (%" PRIu32 ") offset committed: [%.3f %.3f %.3f]->[%.3f %.3f %.3f])",
					 _gyro_calibration.SensorString(), _instance, _gyro_calibration.device_id(),
					 (double)cal_orig(0), (double)cal_orig(1), (double)cal_orig(2),
					 (double)offset_estimate(0), (double)offset_estimate(1), (double)offset_estimate(2));

				// save parameters with preferred calibration slot to current sensor index
				if (_gyro_calibration.ParametersSave(_sensor_gyro_sub.get_instance())) {
					param_notify_changes();
				}

				_in_flight_calibration_check_timestamp_last = hrt_absolute_time() + INFLIGHT_CALIBRATION_QUIET_PERIOD_US;
			}
		}

		// reset
		_gyro_cal_available = false;
	}
}
#endif

void VehicleIMU::updateGyroCalibration()
{
	// update calibrations for all available gyros
	if (hrt_elapsed_time(&_last_calibration_update) > 5_s) {
		bool calibration_updated = false;
		int imu_index = 0;

		for (auto &imu : _imus) {

			if (imu.gyro.calibration.enabled()
			    && (imu.gyro.calibration.device_id() != 0)
			    && imu.at_rest
			    && (hrt_elapsed_time(&imu.time_last_move_detect_us) > 5_s)
			    && (hrt_elapsed_time(&imu.time_last_valid) < 1_s)
			    && imu.gyro.raw_mean.valid() && (imu.gyro.raw_mean.count() > 100)
			    && !imu.gyro.raw_mean.variance().longerThan(0.001f)
			   ) {
				// update calibration
				const Vector3f old_offset{imu.gyro.calibration.offset()};
				const Vector3f new_offset{imu.gyro.raw_mean.mean()};

				bool change_exceeds_stddev = false;
				bool variance_significantly_better = false;

				const Vector3f variance = imu.gyro.raw_mean.variance();

				for (int i = 0; i < 3; i++) {
					// check if offset changed by more than 1 standard deviation
					if (sq(new_offset(i) - old_offset(i)) > variance(i)) {
						change_exceeds_stddev = true;
					}

					// check if current variance is significantly better than previous cal
					// if (variance(i) < 0.1f * _gyro_cal_variance[gyro](i)) {
					// 	variance_significantly_better = true;
					// }
				}

				// update if offset changed by more than 1 standard deviation or currently uncalibrated
				if ((change_exceeds_stddev || !imu.gyro.calibration.calibrated())
				    && imu.gyro.calibration.set_offset(new_offset)
				   ) {

					//_gyro_cal_variance[gyro] = variance;

					PX4_INFO("gyro %d (%" PRIu32 ") updating offsets [%.3f, %.3f, %.3f]->[%.3f, %.3f, %.3f]",
						 imu_index, imu.gyro.calibration.device_id(),
						 (double)old_offset(0), (double)old_offset(1), (double)old_offset(2),
						 (double)new_offset(0), (double)new_offset(1), (double)new_offset(2));

					if (imu.gyro.calibration.ParametersSave(imu_index)) {

					}

					calibration_updated = true;
				}
			}

			imu_index++;
		}

		// save all calibrations
		if (calibration_updated) {
			param_notify_changes();
			_last_calibration_update = hrt_absolute_time();
		}
	}
}

void VehicleIMU::PrintStatus()
{

#if 0
	PX4_INFO_RAW("selected gyro: %" PRIu32 " (%" PRIu8 ")\n", _selection.gyro_device_id, _gyro.last_best_vote);
	_gyro.voter.print();

	PX4_INFO_RAW("\n");
	PX4_INFO_RAW("selected accel: %" PRIu32 " (%" PRIu8 ")\n", _selection.accel_device_id, _accel.last_best_vote);
	_accel.voter.print();
#endif




	// for (unsigned i = 0; i < dimensions; i++) {
	//
	// }

	// print accel
	int accel_index = 0;

	for (auto &imu : _imus) {
		if (imu.accel.calibration.device_id() != 0) {
			PX4_INFO_RAW("accel #%u (%d), prio: %" PRIi32 "\n", accel_index++, imu.accel.calibration.device_id(),
				     imu.accel.calibration.priority());

			for (unsigned axis = 0; axis < 3; axis++) {
				PX4_INFO_RAW("\tval: %8.4f, mean: %8.4f VAR: %8.6f\n", (double)imu.accel.acceleration_prev(axis),
					     (double)imu.accel.raw_mean.mean()(axis), (double)imu.accel.raw_mean.variance()(axis));
			}
		}
	}

	// print gyro
	int gyro_index = 0;

	for (auto &imu : _imus) {
		if (imu.gyro.calibration.device_id() != 0) {
			PX4_INFO_RAW("gyro #%u (%d), prio: %" PRIi32 "\n", gyro_index++, imu.gyro.calibration.device_id(),
				     imu.gyro.calibration.priority());

			for (unsigned axis = 0; axis < 3; axis++) {
				PX4_INFO_RAW("\tval: %8.4f, mean: %8.4f VAR: %8.6f\n", (double)imu.gyro.angular_velocity_prev(axis),
					     (double)imu.gyro.raw_mean.mean()(axis), (double)imu.gyro.raw_mean.variance()(axis));
			}
		}
	}



	// PX4_INFO_RAW("[vehicle_angular_velocity] selected sensor: %" PRIu32
	// 	     ", rate: %.1f Hz %s, estimated bias: [%.5f %.5f %.5f]\n",
	// 	     _gyro_calibration.device_id(), (double)_filter_sample_rate_hz, _fifo_available ? "FIFO" : "",
	// 	     (double)_gyro_bias(0), (double)_gyro_bias(1), (double)_gyro_bias(2));

	// _gyro_calibration.PrintStatus();

	// uint32_t imu_fifo_index = 0;

	// for (auto &s : _sensor_imu_fifo_subs) {
	// 	if (s.mean_sample_interval_us.mean() > 0.f) {
	// 		PX4_INFO_RAW("[vehicle_imu] IMU FIFO (%" PRIu32 "), publish interval: %.1f us (STD:%.1f us)\n",
	// 			     imu_fifo_index++, (double)s.mean_sample_interval_us.mean(), (double)s.mean_sample_interval_us.standard_deviation());
	// 	}
	// }

	for (auto &imu : _imus) {
		if (imu.accel.calibration.device_id() != 0) {
			PX4_INFO_RAW("[vehicle_imu] IMU (%" PRIu32 "), pub:%.1f us (STD:%.1f us), sample:%.1f us (STD:%.1f us) %s\n",
				     imu.accel.calibration.device_id(),
				     (double)imu.accel.mean_publish_interval_us.mean(), (double)imu.accel.mean_publish_interval_us.standard_deviation(),
				     (double)imu.accel.mean_sample_interval_us.mean(), (double)imu.accel.mean_sample_interval_us.standard_deviation(),
				     imu.primary ? "*" : "");
		}
	}





	// PX4_INFO_RAW("[vehicle_imu_fifo] %" PRIu8 " - IMU: %" PRIu32 ", interval: %.1f us (SD %.1f us)\n",
	// 	     _instance, _accel_calibration.device_id(), (double)_interval_us, (double)sqrtf(_interval_best_variance));

	// PX4_INFO_RAW("gyro update mean sample latency: %.6f s, publish latency %.6f s, gyro interval %.6f s",
	// 	     (double)_update_latency_mean.mean()(0),
	// 	     (double)_update_latency_mean.mean()(1),
	// 	     (double)(_interval_us * 1e-6f));

	// perf_print_counter(_imu_generation_gap_perf);

	// _accel_calibration.PrintStatus();
	// _gyro_calibration.PrintStatus();

	_vehicle_acceleration.PrintStatus();
	_vehicle_angular_velocity.PrintStatus();

	perf_print_counter(_cycle_perf);
	perf_print_counter(_selection_changed_perf);
}

} // namespace sensors
