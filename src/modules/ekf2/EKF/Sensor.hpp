/****************************************************************************
 *
 *   Copyright (c) 2020 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file Sensor.hpp
 * Abstract class for sensors
 *
 * @author Mathieu Bresciani <brescianimathieu@gmail.com>
 *
 */

#ifndef EKF_SENSOR_HPP
#define EKF_SENSOR_HPP

#include "common.h"

namespace estimator
{
namespace sensor
{

class Sensor
{
public:
	virtual ~Sensor() {};

	/*
	 * run sanity checks on the current data
	 * this has to be called immediately after
	 * setting new data
	 */
	virtual void runChecks() {};

	/*
	 * return true if the sensor is healthy
	 */
	virtual bool isHealthy() const = 0;

	/*
	 * return true if the delayed sample is healthy
	 * and can be fused in the estimator
	 */
	virtual bool isDataHealthy() const = 0;

	/*
	 * return true if the sensor data rate is
	 * stable and high enough
	 */
	virtual bool isRegularlySendingData() const = 0;
};

} // namespace sensor
} // namespace estimator
#endif // !EKF_SENSOR_HPP
