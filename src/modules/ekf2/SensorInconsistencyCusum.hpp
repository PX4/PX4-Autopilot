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

#include <math.h>
#include <stdint.h>

/**
 * One-sided CUSUM test on the inconsistency of redundant sensors.
 *
 * The part of each sensor's inconsistency above a deadband is integrated over
 * time, and a fault is declared while one of these integrals exceeds a limit.
 */
template<uint8_t SIZE>
class SensorInconsistencyCusum
{
public:
	/**
	 * @param device_ids 0 for an absent sensor
	 * @param inconsistency magnitude of the difference between each sensor and the mean of all sensors
	 * @param deadband inconsistency above which the error accumulates, in the unit of the inconsistency
	 * @param limit accumulated error above which a sensor is faulty, in the unit of the inconsistency times seconds
	 */
	void update(const uint32_t device_ids[SIZE], const float inconsistency[SIZE], float deadband, float limit,
		    float dt)
	{
		uint8_t sensor_count = 0;
		bool limit_exceeded = false;
		float largest_error = 0.f;
		uint8_t largest_error_index = 0;

		for (uint8_t i = 0; i < SIZE; i++) {
			if (device_ids[i] == 0) {
				_accumulated_error[i] = NAN;
				continue;
			}

			sensor_count++;

			// fmaxf returns its other argument for a NAN, which restarts a sensor that was absent
			_accumulated_error[i] = fmaxf(_accumulated_error[i] + (inconsistency[i] - deadband) * dt, 0.f);

			if (_accumulated_error[i] > limit) {
				limit_exceeded = true;
			}

			if (_accumulated_error[i] > largest_error) {
				largest_error = _accumulated_error[i];
				largest_error_index = i;
			}
		}

		_fault_detected = limit_exceeded && (sensor_count >= kMinSensorsToDetect);

		const bool faulty_sensor_known = _fault_detected && (sensor_count >= kMinSensorsToIsolate);
		_faulty_device_id = faulty_sensor_known ? device_ids[largest_error_index] : 0;
	}

	bool faultDetected() const { return _fault_detected; }

	/**
	 * @return 0 when there is no fault or when the faulty sensor cannot be identified
	 */
	uint32_t faultyDeviceId() const { return _faulty_device_id; }

	float accumulatedError(uint8_t index) const { return _accumulated_error[index]; }

private:
	static constexpr uint8_t kMinSensorsToDetect = 2;

	// two sensors deviate equally from their mean, a third one is needed to tell which one is off
	static constexpr uint8_t kMinSensorsToIsolate = 3;

	float _accumulated_error[SIZE] {};
	bool _fault_detected{false};
	uint32_t _faulty_device_id{0};
};
