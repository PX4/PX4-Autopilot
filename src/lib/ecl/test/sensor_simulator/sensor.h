/****************************************************************************
 *
 *   Copyright (c) 2019 ECL Development Team. All rights reserved.
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
 * Base class for defining the interface for simulaton of a sensor
 * @author Kamil Ritz <ka.ritz@hotmail.com>
 */

#pragma once

#include "EKF/ekf.h"
#include <math.h>
#include <memory>

namespace sensor_simulator
{

class Sensor
{
public:

	Sensor(std::shared_ptr<Ekf> ekf);
	virtual ~Sensor();

	void update(uint64_t time);

	void setRateHz(uint32_t rate){ _update_period = uint32_t(1000000)/rate; }

	bool isRunning() const { return _is_running; }

	void start(){ _is_running = true; }

	void stop(){ _is_running = false; }

	bool should_send(uint64_t time) const;

protected:

	std::shared_ptr<Ekf> _ekf;
	// time in microseconds
	uint32_t _update_period;
	uint64_t _time_last_data_sent{0};

	bool _is_running{false};

	// Checks that the right amount time passed since last send data to fulfill rate
	bool is_time_to_send(uint64_t time) const;

	// call set*Data function of Ekf
	virtual void send(uint64_t time) = 0;

};

} // namespace sensor_simulator
