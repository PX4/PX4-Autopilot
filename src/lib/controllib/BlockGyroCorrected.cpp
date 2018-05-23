/****************************************************************************
 *
 *   Copyright (C) 2017 PX4 Development Team. All rights reserved.
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

#include "BlockGyroCorrected.hpp"

#include <mathlib/mathlib.h>
#include <px4_posix.h>

namespace control
{

BlockGyroCorrected::BlockGyroCorrected(SuperBlock *parent) :
	SuperBlock(parent, ""),
	_board_rotation(this)
{
	_sensor_bias_sub = orb_subscribe(ORB_ID(sensor_bias));
	_sensor_correction_sub = orb_subscribe(ORB_ID(sensor_correction));

	for (uint8_t i = 0; i < MAX_GYRO_COUNT; i++) {
		_sensor_gyro_sub[i] = -1;
	}

	_gyro_count = math::min(orb_group_count(ORB_ID(sensor_gyro)), MAX_GYRO_COUNT);

	if (_gyro_count == 0) {
		_gyro_count = 1;
	}

	for (unsigned s = 0; s < _gyro_count; s++) {
		_sensor_gyro_sub[s] = orb_subscribe_multi(ORB_ID(sensor_gyro), s);
	}

	update();
}

BlockGyroCorrected::~BlockGyroCorrected()
{
	orb_unsubscribe(_sensor_bias_sub);
	orb_unsubscribe(_sensor_correction_sub);

	for (unsigned s = 0; s < _gyro_count; s++) {
		orb_unsubscribe(_sensor_gyro_sub[s]);
	}
}

bool BlockGyroCorrected::updateBlocking(int timeout)
{
	// wakeup source: gyro data from sensor selected by the sensor app
	px4_pollfd_struct_t poll_fds = {};
	poll_fds.events = POLLIN;
	poll_fds.fd = _sensor_gyro_sub[_selected_gyro];

	int pret = px4_poll(&poll_fds, 1, timeout);

	if ((pret == 1) && (poll_fds.revents & POLLIN)) {
		// update gyro data
		return update();

	} else if (pret < 0) {
		// this is undesirable but not much we can do - might want to flag unhappy status
		PX4_ERR("poll error %d, %d", pret, errno);

		// sleep a bit before next try
		usleep(100000);
	}

	return false;
}

void BlockGyroCorrected::update_bias()
{
	bool updated = false;
	orb_check(_sensor_bias_sub, &updated);

	if (updated) {
		sensor_bias_s bias;

		if (orb_copy(ORB_ID(sensor_bias), _sensor_bias_sub, &bias) == PX4_OK) {
			_bias = {bias.gyro_x_bias, bias.gyro_y_bias, bias.gyro_z_bias};
		}
	}
}

void BlockGyroCorrected::update_correction()
{
	bool updated = false;
	orb_check(_sensor_correction_sub, &updated);

	if (updated) {
		sensor_correction_s corr;

		if (orb_copy(ORB_ID(sensor_correction), _sensor_correction_sub, &corr) == PX4_OK) {

			// update the latest gyro selection
			if (corr.selected_gyro_instance < _gyro_count) {
				_selected_gyro = corr.selected_gyro_instance;
			}

			if (_selected_gyro == 0) {
				_offset = corr.gyro_offset_0;
				_scale = corr.gyro_scale_0;

			} else if (_selected_gyro == 1) {
				_offset = corr.gyro_offset_1;
				_scale = corr.gyro_scale_1;

			} else if (_selected_gyro == 2) {
				_offset = corr.gyro_offset_2;
				_scale = corr.gyro_scale_2;

			} else {
				_offset = {0.0f, 0.0f, 0.0f};
				_scale = {1.0f, 1.0f, 1.0f};
			}
		}
	}
}

bool BlockGyroCorrected::update()
{
	update_bias();
	update_correction();

	sensor_gyro_s sensor_gyro;

	if (orb_copy(ORB_ID(sensor_gyro), _sensor_gyro_sub[_selected_gyro], &sensor_gyro) == PX4_OK) {

		const matrix::Vector3f gyro{sensor_gyro.x, sensor_gyro.y, sensor_gyro.z};

		// rotate corrected measurements from sensor to body frame
		// correct for in-run bias errors
		_rates = (_board_rotation.get() * (gyro - _offset).emult(_scale)) - _bias;

		_timestamp = sensor_gyro.timestamp;

		return true;
	}

	return false;
}

} // namespacea control
