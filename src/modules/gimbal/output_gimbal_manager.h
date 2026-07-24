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

#include "output.h"

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/gimbal_manager_set_pitchyaw.h>
#include <uORB/topics/external_gimbal_manager_information.h>
#include <uORB/topics/external_gimbal_manager_status.h>

namespace gimbal
{

// Output that makes PX4 act as a gimbal manager *client*: instead of driving a
// gimbal device directly, it forwards the setpoints to an external gimbal
// manager (e.g. a smart camera-gimbal that runs its own manager). It discovers
// the manager, requests control while there is an active setpoint, and streams
// GIMBAL_MANAGER_SET_PITCHYAW to it.
class OutputToGimbalManager : public OutputBase
{
public:
	OutputToGimbalManager() = delete;
	explicit OutputToGimbalManager(const Parameters &parameters);
	virtual ~OutputToGimbalManager() = default;

	void update(const ControlData &control_data, bool new_setpoints, uint8_t &gimbal_device_id) override;
	void print_status() const override;

private:
	enum class ControlState {
		Released,	// we don't hold control of the manager
		Acquiring,	// we requested control and wait for confirmation
		InControl	// the manager reports us as primary control
	};

	void _update_manager_info();
	void _update_manager_status();
	bool _have_primary_control() const;
	void _send_configure(bool acquire);
	void _publish_set_pitchyaw();

	uORB::Subscription _information_sub{ORB_ID(external_gimbal_manager_information)};
	uORB::Subscription _status_sub{ORB_ID(external_gimbal_manager_status)};
	uORB::Publication<gimbal_manager_set_pitchyaw_s> _set_pitchyaw_pub{ORB_ID(gimbal_manager_set_pitchyaw)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};

	bool _manager_found{false};
	uint8_t _manager_sysid{0};
	uint8_t _manager_compid{0};
	uint8_t _gimbal_device_id{0};

	external_gimbal_manager_status_s _status{};
	bool _status_valid{false};

	ControlState _control_state{ControlState::Released};
	hrt_abstime _last_acquire_request{0};

	static constexpr hrt_abstime kAcquireRetryInterval{3000000};	// 3 s
};

} /* namespace gimbal */
