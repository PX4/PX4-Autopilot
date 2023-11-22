/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "VehicleOpticalFlow.hpp"

#include <px4_platform_common/log.h>

namespace sensors
{

using namespace matrix;
using namespace time_literals;

static constexpr uint32_t SENSOR_TIMEOUT{300_ms};

VehicleOpticalFlow::VehicleOpticalFlow() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_vehicle_optical_flow_pub.advertise();

	_gyro_integrator.set_reset_samples(1);
}

VehicleOpticalFlow::~VehicleOpticalFlow()
{
	Stop();
	perf_free(_cycle_perf);
}

bool VehicleOpticalFlow::Start()
{
	_sensor_flow_sub.registerCallback();

	_sensor_gyro_sub.registerCallback();
	_sensor_gyro_sub.set_required_updates(sensor_gyro_s::ORB_QUEUE_LENGTH / 2);

	_sensor_selection_sub.registerCallback();

	ScheduleNow();
	return true;
}

void VehicleOpticalFlow::Stop()
{
	Deinit();

	// clear all registered callbacks
	_sensor_flow_sub.unregisterCallback();
	_sensor_gyro_sub.unregisterCallback();
	_sensor_selection_sub.unregisterCallback();
}

void VehicleOpticalFlow::ParametersUpdate()
{
	// Check if parameters have changed
	if (_params_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_params_sub.copy(&param_update);

		updateParams();

		_flow_rotation = get_rot_matrix((enum Rotation)_param_sens_flow_rot.get());
	}
}

void VehicleOpticalFlow::Run()
{
	perf_begin(_cycle_perf);

	ParametersUpdate();

	UpdateDistanceSensor();

	if (!_delta_angle_available) {
		UpdateSensorGyro();
	}

	sensor_optical_flow_s sensor_optical_flow;

	if (_sensor_flow_sub.update(&sensor_optical_flow)) {

		// clear data accumulation if there's a gap in data
		const uint64_t integration_gap_threshold_us = sensor_optical_flow.integration_timespan_us * 2;

		if ((sensor_optical_flow.timestamp_sample >= _flow_timestamp_sample_last + integration_gap_threshold_us)
		    || (_accumulated_count > 0 && (sensor_optical_flow.quality > 0) && _quality_sum == 0)) {

			ClearAccumulatedData();
		}


		const hrt_abstime timestamp_oldest = sensor_optical_flow.timestamp_sample - sensor_optical_flow.integration_timespan_us;
		const hrt_abstime timestamp_newest = sensor_optical_flow.timestamp;

		// delta angle
		//  - from sensor_optical_flow if available, otherwise use synchronized sensor_gyro if available
		if (sensor_optical_flow.delta_angle_available && Vector2f(sensor_optical_flow.delta_angle).isAllFinite()) {
			// passthrough integrated gyro if available
			Vector3f delta_angle(sensor_optical_flow.delta_angle);

			if (!PX4_ISFINITE(delta_angle(2))) {
				// Some sensors only provide X and Y angular rates, rotate them but place back the NAN on the Z axis
				delta_angle(2) = 0.f;
				_delta_angle += _flow_rotation * delta_angle;
				_delta_angle(2) = NAN;

			} else {
				_delta_angle += _flow_rotation * delta_angle;
			}

			_delta_angle_available = true;

		} else {
			_delta_angle_available = false;

			// integrate synchronized gyro
			gyroSample gyro_sample;

			while (_gyro_buffer.pop_oldest(timestamp_oldest, timestamp_newest, &gyro_sample)) {

				_gyro_integrator.put(gyro_sample.data, gyro_sample.dt);

				float min_interval_s = (sensor_optical_flow.integration_timespan_us * 1e-6f) * 0.99f;

				if (_gyro_integrator.integral_dt() > min_interval_s) {
					//PX4_INFO("integral dt: %.6f, min interval: %.6f", (double)_gyro_integrator.integral_dt(),(double) min_interval_s);
					break;
				}
			}

			Vector3f delta_angle{NAN, NAN, NAN};
			uint16_t delta_angle_dt;

			if (_gyro_integrator.reset(delta_angle, delta_angle_dt)) {
				_delta_angle += delta_angle;

			} else {
				// force integrator reset
				_gyro_integrator.reset();
			}
		}

		// distance
		//  - from sensor_optical_flow if available, otherwise use downward distance_sensor if available
		if (sensor_optical_flow.distance_available && PX4_ISFINITE(sensor_optical_flow.distance_m)) {
			if (!PX4_ISFINITE(_distance_sum)) {
				_distance_sum = sensor_optical_flow.distance_m;
				_distance_sum_count = 1;

			} else {
				_distance_sum += sensor_optical_flow.distance_m;
				_distance_sum_count += 1;
			}

		} else {
			// otherwise use buffered downward facing distance_sensor if available
			rangeSample range_sample;

			if (_range_buffer.peak_first_older_than(sensor_optical_flow.timestamp_sample, &range_sample)) {
				if (!PX4_ISFINITE(_distance_sum)) {
					_distance_sum = range_sample.data;
					_distance_sum_count = 1;

				} else {
					_distance_sum += range_sample.data;
					_distance_sum_count += 1;
				}
			}
		}

		_flow_timestamp_sample_last = sensor_optical_flow.timestamp_sample;
		_flow_integral(0) += sensor_optical_flow.pixel_flow[0];
		_flow_integral(1) += sensor_optical_flow.pixel_flow[1];

		_integration_timespan_us += sensor_optical_flow.integration_timespan_us;

		_quality_sum += sensor_optical_flow.quality;
		_accumulated_count++;

		bool publish = true;

		if (_param_sens_flow_rate.get() > 0) {
			const float interval_us = 1e6f / _param_sens_flow_rate.get();

			// don't allow publishing faster than SENS_FLOW_RATE
			if (_integration_timespan_us < interval_us) {
				publish = false;
			}
		}

		if (publish) {
			vehicle_optical_flow_s vehicle_optical_flow{};

			vehicle_optical_flow.timestamp_sample = sensor_optical_flow.timestamp_sample;
			vehicle_optical_flow.device_id = sensor_optical_flow.device_id;

			_flow_integral.copyTo(vehicle_optical_flow.pixel_flow);
			_delta_angle.copyTo(vehicle_optical_flow.delta_angle);

			vehicle_optical_flow.integration_timespan_us = _integration_timespan_us;

			vehicle_optical_flow.quality = _quality_sum / _accumulated_count;

			if (_distance_sum_count > 0 && PX4_ISFINITE(_distance_sum)) {
				vehicle_optical_flow.distance_m = _distance_sum / _distance_sum_count;

			} else {
				vehicle_optical_flow.distance_m = NAN;
			}

			// SENS_FLOW_MAXR
			if (PX4_ISFINITE(sensor_optical_flow.max_flow_rate)
			    && (sensor_optical_flow.max_flow_rate <= _param_sens_flow_maxr.get())) {

				vehicle_optical_flow.max_flow_rate = sensor_optical_flow.max_flow_rate;

			} else {
				vehicle_optical_flow.max_flow_rate = _param_sens_flow_maxr.get();
			}

			// SENS_FLOW_MINHGT
			if (PX4_ISFINITE(sensor_optical_flow.min_ground_distance)
			    && (sensor_optical_flow.min_ground_distance >= _param_sens_flow_minhgt.get())) {

				vehicle_optical_flow.min_ground_distance = sensor_optical_flow.min_ground_distance;

			} else {
				vehicle_optical_flow.min_ground_distance = _param_sens_flow_minhgt.get();
			}

			// SENS_FLOW_MAXHGT
			if (PX4_ISFINITE(sensor_optical_flow.max_ground_distance)
			    && (sensor_optical_flow.max_ground_distance <= _param_sens_flow_maxhgt.get())) {

				vehicle_optical_flow.max_ground_distance = sensor_optical_flow.max_ground_distance;

			} else {
				vehicle_optical_flow.max_ground_distance = _param_sens_flow_maxhgt.get();
			}


			// rotate (SENS_FLOW_ROT)
			float zeroval = 0.f;
			rotate_3f((enum Rotation)_param_sens_flow_rot.get(), vehicle_optical_flow.pixel_flow[0],
				  vehicle_optical_flow.pixel_flow[1], zeroval);

			vehicle_optical_flow.timestamp = hrt_absolute_time();
			_vehicle_optical_flow_pub.publish(vehicle_optical_flow);

			// vehicle_optical_flow_vel if distance is available (for logging)
			if (_distance_sum_count > 0 && PX4_ISFINITE(_distance_sum)) {
				const float range = _distance_sum / _distance_sum_count;

				vehicle_optical_flow_vel_s flow_vel{};

				flow_vel.timestamp_sample = vehicle_optical_flow.timestamp_sample;

				// NOTE: the EKF uses the reverse sign convention to the flow sensor. EKF assumes positive LOS rate
				// is produced by a RH rotation of the image about the sensor axis.
				const Vector2f flow_xy_rad{-vehicle_optical_flow.pixel_flow[0], -vehicle_optical_flow.pixel_flow[1]};
				const Vector3f gyro_rate_integral{-vehicle_optical_flow.delta_angle[0], -vehicle_optical_flow.delta_angle[1], -vehicle_optical_flow.delta_angle[2]};

				const float flow_dt = 1e-6f * vehicle_optical_flow.integration_timespan_us;

				// compensate for body motion to give a LOS rate
				const Vector2f flow_compensated_XY_rad = flow_xy_rad - gyro_rate_integral.xy();

				Vector3f vel_optflow_body;
				vel_optflow_body(0) = - range * flow_compensated_XY_rad(1) / flow_dt;
				vel_optflow_body(1) =   range * flow_compensated_XY_rad(0) / flow_dt;
				vel_optflow_body(2) = 0.f;

				// vel_body
				flow_vel.vel_body[0] = vel_optflow_body(0);
				flow_vel.vel_body[1] = vel_optflow_body(1);

				// vel_ne
				flow_vel.vel_ne[0] = NAN;
				flow_vel.vel_ne[1] = NAN;

				vehicle_attitude_s vehicle_attitude{};

				if (_vehicle_attitude_sub.copy(&vehicle_attitude)) {
					const matrix::Dcmf R_to_earth = matrix::Quatf(vehicle_attitude.q);
					const Vector3f flow_vel_ne = R_to_earth * vel_optflow_body;

					flow_vel.vel_ne[0] = flow_vel_ne(0);
					flow_vel.vel_ne[1] = flow_vel_ne(1);
				}

				const Vector2f flow_rate(flow_xy_rad * (1.f / flow_dt));
				flow_rate.copyTo(flow_vel.flow_rate_uncompensated);

				const Vector2f flow_rate_compensated(flow_compensated_XY_rad * (1.f / flow_dt));
				flow_rate_compensated.copyTo(flow_vel.flow_rate_compensated);

				const Vector3f measured_body_rate(gyro_rate_integral * (1.f / flow_dt));

				// gyro_rate
				flow_vel.gyro_rate[0] = measured_body_rate(0);
				flow_vel.gyro_rate[1] = measured_body_rate(1);
				flow_vel.gyro_rate[2] = measured_body_rate(2);

				flow_vel.timestamp = hrt_absolute_time();

				_vehicle_optical_flow_vel_pub.publish(flow_vel);
			}

			ClearAccumulatedData();
		}
	}

	// reschedule backup
	ScheduleDelayed(10_ms);

	perf_end(_cycle_perf);
}

void VehicleOpticalFlow::UpdateDistanceSensor()
{
	// update range finder buffer
	distance_sensor_s distance_sensor;

	if ((_distance_sensor_selected < 0) && _distance_sensor_subs.advertised()) {
		for (unsigned i = 0; i < _distance_sensor_subs.size(); i++) {

			if (_distance_sensor_subs[i].update(&distance_sensor)) {
				// only use the first instace which has the correct orientation
				if ((hrt_elapsed_time(&distance_sensor.timestamp) < 100_ms)
				    && (distance_sensor.orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING)) {

					int ndist = orb_group_count(ORB_ID(distance_sensor));

					if (ndist > 1) {
						PX4_INFO("selected distance_sensor:%d (%d advertised)", i, ndist);
					}

					_distance_sensor_selected = i;
					_last_range_sensor_update = distance_sensor.timestamp;
					break;
				}
			}
		}
	}

	if (_distance_sensor_selected >= 0 && _distance_sensor_subs[_distance_sensor_selected].update(&distance_sensor)) {
		// range sample
		if (distance_sensor.orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING) {

			if ((distance_sensor.current_distance >= distance_sensor.min_distance)
			    && (distance_sensor.current_distance <= distance_sensor.max_distance)) {

				rangeSample sample;
				sample.time_us = distance_sensor.timestamp;
				sample.data = distance_sensor.current_distance;

				_range_buffer.push(sample);

				_last_range_sensor_update = distance_sensor.timestamp;

				return;
			}

		} else {
			_distance_sensor_selected = -1;
		}
	}

	if (hrt_elapsed_time(&_last_range_sensor_update) > 1_s) {
		_distance_sensor_selected = -1;
	}
}

void VehicleOpticalFlow::UpdateSensorGyro()
{
	if (_sensor_selection_sub.updated()) {
		sensor_selection_s sensor_selection{};
		_sensor_selection_sub.copy(&sensor_selection);

		for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
			uORB::SubscriptionData<sensor_gyro_s> sensor_gyro_sub{ORB_ID(sensor_gyro), i};

			if (sensor_gyro_sub.advertised()
			    && (sensor_gyro_sub.get().timestamp != 0)
			    && (sensor_gyro_sub.get().device_id != 0)
			    && (hrt_elapsed_time(&sensor_gyro_sub.get().timestamp) < 1_s)) {

				if (sensor_gyro_sub.get().device_id == sensor_selection.gyro_device_id) {
					if (_sensor_gyro_sub.ChangeInstance(i) && _sensor_gyro_sub.registerCallback()) {

						_gyro_calibration.set_device_id(sensor_gyro_sub.get().device_id);
						PX4_DEBUG("selecting sensor_gyro:%" PRIu8 " %" PRIu32, i, sensor_gyro_sub.get().device_id);
						break;

					} else {
						PX4_ERR("unable to register callback for sensor_gyro:%" PRIu8 " %" PRIu32, i, sensor_gyro_sub.get().device_id);
					}
				}
			}
		}
	}

	// buffer
	bool sensor_gyro_lost_printed = false;
	int gyro_updates = 0;

	while (_sensor_gyro_sub.updated() && (gyro_updates < sensor_gyro_s::ORB_QUEUE_LENGTH)) {
		gyro_updates++;
		const unsigned last_generation = _sensor_gyro_sub.get_last_generation();
		sensor_gyro_s sensor_gyro;

		if (_sensor_gyro_sub.copy(&sensor_gyro)) {

			if (_sensor_gyro_sub.get_last_generation() != last_generation + 1) {
				if (!sensor_gyro_lost_printed) {
					PX4_ERR("sensor_gyro lost, generation %u -> %u", last_generation, _sensor_gyro_sub.get_last_generation());
					sensor_gyro_lost_printed = true;
				}
			}

			_gyro_calibration.set_device_id(sensor_gyro.device_id);
			_gyro_calibration.SensorCorrectionsUpdate();

			const float dt_s = (sensor_gyro.timestamp_sample - _gyro_timestamp_sample_last) * 1e-6f;
			_gyro_timestamp_sample_last = sensor_gyro.timestamp_sample;

			gyroSample gyro_sample;
			gyro_sample.time_us = sensor_gyro.timestamp_sample;
			gyro_sample.data = _gyro_calibration.Correct(Vector3f{sensor_gyro.x, sensor_gyro.y, sensor_gyro.z});
			gyro_sample.dt = dt_s;

			_gyro_buffer.push(gyro_sample);
		}
	}
}

void VehicleOpticalFlow::ClearAccumulatedData()
{
	// clear accumulated data
	_flow_integral.zero();
	_integration_timespan_us = 0;

	_delta_angle.zero();

	_distance_sum = NAN;
	_distance_sum_count = 0;

	_quality_sum = 0;
	_accumulated_count = 0;

	_gyro_integrator.reset();
}

void VehicleOpticalFlow::PrintStatus()
{

}

}; // namespace sensors
