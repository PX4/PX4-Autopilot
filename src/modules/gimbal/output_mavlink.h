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
#include <uORB/Subscription.hpp>

#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/gimbal_device_set_attitude.h>
#include <uORB/topics/gimbal_device_information.h>
#include <uORB/topics/gimbal_device_attitude_status.h>
#include <uORB/topics/gimbal_manager_status.h>
#include <uORB/topics/gimbal_manager_set_attitude.h>
#include <uORB/topics/gimbal_manager_information.h>
#include <uORB/topics/external_gimbal_manager_information.h>
#include <uORB/topics/external_gimbal_manager_status.h>
#include <uORB/topics/external_gimbal_manager_set_attitude.h>

#include <px4_platform_common/time.h>
#include <cmath>


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

/**
 * @brief	Bridges PX4 gimbal setpoints to an external MAVLink gimbal manager (Gremsy Lynx)
 *
 * @class	OutputMavlinkToGimbalManager
 */
class OutputMavlinkToGimbalManager : public OutputBase
{
public:
	explicit OutputMavlinkToGimbalManager(const Parameters &parameters);
	~OutputMavlinkToGimbalManager() override = default;

	void update(const ControlData &control_data, bool new_setpoints, uint8_t &gimbal_device_id) override;
	void print_status() const override;

private:
	// uORB publications
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<external_gimbal_manager_set_attitude_s> _external_gimbal_manager_set_attitude_pub{ORB_ID(external_gimbal_manager_set_attitude)};

	// uORB subscriptions
	uORB::Subscription _external_gimbal_manager_information_sub{ORB_ID(external_gimbal_manager_information)};
	uORB::Subscription _external_gimbal_manager_status_sub{ORB_ID(external_gimbal_manager_status)};
	uORB::Subscription _vehicle_command_ack_sub{ORB_ID(vehicle_command_ack)};

	/**
	 * Control ownership reported by the external gimbal manager
	 */
	enum class ControlRights : uint8_t {
		NONE = 0,
		PRIMARY = 1,
		SECONDARY = 2
	};

	// Discovery
	void _check_for_gimbal_manager_information();
	void _request_gimbal_manager_information();
	void _check_for_gimbal_manager_status();

	// Control ownership
	bool _check_for_take_control_ack();
	bool _handle_take_control_command_ack(const vehicle_command_ack_s &ack);

	bool _acquire_control_for_autopilot(const hrt_abstime &now);
	void _send_take_control_request();
	void _reset_take_control_state();

	// Setpoint output
	void _publish_gimbal_manager_set_attitude();

	// Helpers
	bool _have_valid_manager() const
	{
		return	((_manager_sysid != 0) &&
			 (_manager_compid != 0) &&
			 (_gimbal_device_id != 0));
	}

	static const char *_control_right_str(ControlRights control_rights);

	// Cache identity of the external gimbal manager
	uint8_t		_manager_sysid{0};
	uint8_t		_manager_compid{0};
	uint8_t		_gimbal_device_id{0};

	// Control state
	ControlRights 	_control_rights{ControlRights::NONE};
	uint8_t 	_take_control_retry_count{0};

	bool 		_can_publish_set_attitude{false};
	bool		_take_control_ack_received{false};
	bool		_take_control_ack_accepted{false};

	// Timing state
	hrt_abstime _last_info_request{0};
	hrt_abstime _wait_ack_start_time{0};
	hrt_abstime _take_control_backoff_start{0};

	static constexpr hrt_abstime INFO_REQUEST_PERIOD_US	= 1'000'000;	///< Discovery request rate
	static constexpr hrt_abstime GIMBAL_BUSY_TIMEOUT_US 	= 15'000'000;	///< Backoff after repeated failures
	static constexpr hrt_abstime WAIT_ACK_TIMEOUT_US	= 1'000'000;	///< Timeout while waiting for ACK
};
} /* namespace gimbal */
