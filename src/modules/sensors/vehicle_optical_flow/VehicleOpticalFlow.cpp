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
}

VehicleOpticalFlow::~VehicleOpticalFlow()
{
	Stop();
	perf_free(_cycle_perf);
}

bool VehicleOpticalFlow::Start()
{
	_sensor_flow_sub.registerCallback();

	_gyro_feed.start();

	ScheduleNow();
	return true;
}

void VehicleOpticalFlow::Stop()
{
	Deinit();

	// clear all registered callbacks
	_sensor_flow_sub.unregisterCallback();
	_gyro_feed.stop();
}

void VehicleOpticalFlow::ParametersUpdate()
{
	// Check if parameters have changed
	if (_params_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_params_sub.copy(&param_update);

		updateParams();

		_accumulator.setSensorRotation(get_rot_matrix((enum Rotation)_param_sens_flow_rot.get()));
	}
}

void VehicleOpticalFlow::Run()
{
	perf_begin(_cycle_perf);

	ParametersUpdate();

	UpdateDistanceSensor();

	_gyro_feed.update(_accumulator);

	sensor_optical_flow_s sensor_optical_flow;

	if (_sensor_flow_sub.update(&sensor_optical_flow)) {

		if (_accumulator.addFrame(sensor_optical_flow)) {
			ClearDistance();
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

		bool publish = true;

		if (_param_sens_flow_rate.get() > 0) {
			const float interval_us = 1e6f / _param_sens_flow_rate.get();

			// don't allow publishing faster than SENS_FLOW_RATE
			if (_accumulator.accumulatedTimespanUs() < interval_us) {
				publish = false;
			}
		}

		if (publish) {
			const OpticalFlowAccumulator::Window window = _accumulator.window();

			vehicle_optical_flow_s vehicle_optical_flow{};

			vehicle_optical_flow.timestamp_sample = sensor_optical_flow.timestamp_sample;
			vehicle_optical_flow.device_id = sensor_optical_flow.device_id;

			(window.pixel_flow * _param_sens_flow_scale.get()).copyTo(vehicle_optical_flow.pixel_flow);
			window.delta_angle.copyTo(vehicle_optical_flow.delta_angle);
			vehicle_optical_flow.integration_timespan_us = window.integration_timespan_us;
			vehicle_optical_flow.quality = window.quality;

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
			if (window.frame_count > 0 && _distance_sum_count > 0 && PX4_ISFINITE(_distance_sum)) {
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

			_accumulator.reset();
			ClearDistance();
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

void VehicleOpticalFlow::ClearDistance()
{
	_distance_sum = NAN;
	_distance_sum_count = 0;
}

void VehicleOpticalFlow::PrintStatus()
{

}

}; // namespace sensors
