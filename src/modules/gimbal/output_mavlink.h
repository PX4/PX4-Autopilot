/****************************************************************************
*
*   Copyright (c) 2016-2022 PX4 Development Team. All rights reserved.
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

#include "output.h"

#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/gimbal_device_set_attitude.h>
#include <uORB/topics/gimbal_device_information.h>
#include <uORB/topics/gimbal_device_attitude_status.h>


namespace gimbal
{
class OutputMavlinkV1 : public OutputBase
{
public:
	OutputMavlinkV1(const Parameters &parameters);
	virtual ~OutputMavlinkV1() = default;

	virtual void update(const ControlData &control_data, bool new_setpoints, uint8_t &gimbal_device_id);

	virtual void print_status() const;

private:
	void _stream_device_attitude_status();
	uORB::Publication<vehicle_command_s> _gimbal_v1_command_pub{ORB_ID(gimbal_v1_command)};
	uORB::Publication <gimbal_device_attitude_status_s>	_attitude_status_pub{ORB_ID(gimbal_device_attitude_status)};

	ControlData::Type _previous_control_data_type {ControlData::Type::Neutral};
};

class OutputMavlinkV2 : public OutputBase
{
public:
	OutputMavlinkV2(const Parameters &parameters);
	virtual ~OutputMavlinkV2() = default;

	virtual void update(const ControlData &control_data, bool new_setpoints, uint8_t &gimbal_device_id);

	virtual void print_status() const;

private:
	void _publish_gimbal_device_set_attitude();
	void _request_gimbal_device_information();
	void _check_for_gimbal_device_information();

	uORB::Publication<gimbal_device_set_attitude_s> _gimbal_device_set_attitude_pub{ORB_ID(gimbal_device_set_attitude)};
	uORB::Subscription _gimbal_device_information_sub{ORB_ID(gimbal_device_information)};

	uint8_t _gimbal_device_id{0};
	hrt_abstime _last_gimbal_device_checked{0};
	bool _gimbal_device_found {false};
};

} /* namespace gimbal */
