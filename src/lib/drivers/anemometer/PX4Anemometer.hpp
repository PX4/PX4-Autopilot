/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include <drivers/drv_hrt.h>
#include <drivers/drv_anemometer.h>
#include <lib/conversion/rotation.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/windspeed.h>

class PX4Anemometer
{

public:
	PX4Anemometer(const uint32_t device_id,
		       const ORB_PRIO priority = ORB_PRIO_DEFAULT,
		       const uint8_t device_orientation = windspeed_s::ROTATION_DOWNWARD_FACING);
	~PX4Anemometer();

	void set_device_type(uint8_t device_type);
	//void set_air_temperature_celsius(uint64_t air_temperature_celsius) { _anemometer_pub.get().air_temperature_celsius = air_temperature_celsius; }

	void set_device_id(const uint8_t device_id) { _anemometer_pub.get().id = device_id; };

	void set_orientation(const uint8_t device_orientation = windspeed_s::ROTATION_DOWNWARD_FACING);

	void update(const hrt_abstime &timestamp_sample, const float measurement[3], const float confidence[3], const int orientation = windspeed_s::ROTATION_DOWNWARD_FACING, const float air_temperature_celsius = 0);

private:

	uORB::PublicationMultiData<windspeed_s> _anemometer_pub;

};
