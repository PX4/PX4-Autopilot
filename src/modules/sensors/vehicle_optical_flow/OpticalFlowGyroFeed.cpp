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


#include "OpticalFlowGyroFeed.hpp"

#include <px4_platform_common/log.h>
#include <uORB/Subscription.hpp>

namespace sensors
{

using namespace matrix;
using namespace time_literals;

OpticalFlowGyroFeed::OpticalFlowGyroFeed(px4::WorkItem *work_item) :
	_sensor_gyro_sub(work_item, ORB_ID(sensor_gyro)),
	_sensor_selection_sub(work_item, ORB_ID(sensor_selection))
{
}

void OpticalFlowGyroFeed::start()
{
	_sensor_gyro_sub.registerCallback();
	_sensor_gyro_sub.set_required_updates(sensor_gyro_s::ORB_QUEUE_LENGTH / 2);

	_sensor_selection_sub.registerCallback();
}

void OpticalFlowGyroFeed::stop()
{
	_sensor_gyro_sub.unregisterCallback();
	_sensor_selection_sub.unregisterCallback();
}

void OpticalFlowGyroFeed::update(OpticalFlowAccumulator &accumulator)
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

						_gyro_calibration.set_device_id(sensor_gyro_sub.get().device_id, sensor_gyro_sub.get().is_external);
						PX4_DEBUG("selecting sensor_gyro:%" PRIu8 " %" PRIu32, i, sensor_gyro_sub.get().device_id);
						break;

					} else {
						PX4_ERR("unable to register callback for sensor_gyro:%" PRIu8 " %" PRIu32, i, sensor_gyro_sub.get().device_id);
					}
				}
			}
		}
	}

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

			_gyro_calibration.set_device_id(sensor_gyro.device_id, sensor_gyro.is_external);
			_gyro_calibration.SensorCorrectionsUpdate();

			const float dt_s = (sensor_gyro.timestamp_sample - _gyro_timestamp_sample_last) * 1e-6f;
			_gyro_timestamp_sample_last = sensor_gyro.timestamp_sample;

			accumulator.addGyroSample(sensor_gyro.timestamp_sample,
						  _gyro_calibration.Correct(Vector3f{sensor_gyro.x, sensor_gyro.y, sensor_gyro.z}), dt_s);
		}
	}
}

} // namespace sensors
