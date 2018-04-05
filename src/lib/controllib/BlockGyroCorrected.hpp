/****************************************************************************
 *
 *   Copyright (C) 2018 PX4 Development Team. All rights reserved.
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

#include "BlockBoardRotation.hpp"

#include "block/Block.hpp"

#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_bias.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_gyro.h>

namespace control
{

static constexpr int MAX_GYRO_COUNT = 3;

class __EXPORT BlockGyroCorrected : public SuperBlock
{
public:

	BlockGyroCorrected(SuperBlock *parent = nullptr);
	~BlockGyroCorrected();

	// no copy, assignment, move, move assignment
	BlockGyroCorrected(const BlockGyroCorrected &) = delete;
	BlockGyroCorrected &operator=(const BlockGyroCorrected &) = delete;
	BlockGyroCorrected(BlockGyroCorrected &&) = delete;
	BlockGyroCorrected &operator=(BlockGyroCorrected &&) = delete;

	const matrix::Vector3f &get() { return _rates; }
	const uint64_t &timestamp() { return _timestamp; }

	bool updateBlocking(int timeout = 100);
	bool update();

private:

	void update_bias();
	void update_correction();

	int _sensor_correction_sub{-1};		/**< sensor thermal correction subscription */
	int _sensor_bias_sub{-1};		/**< sensor in-run bias correction subscription */
	int	_sensor_gyro_sub[MAX_GYRO_COUNT] {};

	BlockBoardRotation _board_rotation;

	uint8_t	_selected_gyro{0};
	uint8_t	_gyro_count{0};

	matrix::Vector3f _offset{0.0f, 0.0f, 0.0f};
	matrix::Vector3f _scale{1.0f, 1.0f, 1.0f};
	matrix::Vector3f _bias{0.0f, 0.0f, 0.0f};

	uint64_t _timestamp{0};
	matrix::Vector3f _rates{0.0f, 0.0f, 0.0f};

};

} // namespace control
