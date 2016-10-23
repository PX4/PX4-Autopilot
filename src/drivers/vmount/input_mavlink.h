/****************************************************************************
*
*   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file input_mavlink.h
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 */

#pragma once

#include "input.h"
#include "input_rc.h"
#include <uORB/topics/vehicle_roi.h>

namespace vmount
{


/**
 ** class InputMavlinkROI
 ** Input based on the vehicle_roi topic
 */
class InputMavlinkROI : public InputBase
{
public:

	/**
	 * @param manual_control if non-null allow manual input as long as we have not received any mavlink
	 * command yet or ROI mode is set to NONE.
	 */
	InputMavlinkROI(InputRC *manual_control = nullptr);
	virtual ~InputMavlinkROI();

	virtual void print_status();

protected:
	virtual int update_impl(unsigned int timeout_ms, ControlData **control_data);
	virtual int initialize();

private:
	void _read_control_data_from_position_setpoint_sub();

	int _vehicle_roi_sub = -1;
	int _position_setpoint_triplet_sub = -1;
	bool _allow_manual_control = true;
	InputRC *_manual_control;
	uint8_t _cur_roi_mode = vehicle_roi_s::VEHICLE_ROI_NONE;
};


/**
 ** class InputMavlinkCmdMount
 ** Input based on the VEHICLE_CMD_DO_MOUNT_CONTROL mavlink command
 */
class InputMavlinkCmdMount : public InputBase
{
public:

	/**
	 * @param manual_control if non-null allow manual input as long as we have not received any mavlink
	 * command yet.
	 */
	InputMavlinkCmdMount(InputRC *manual_control = nullptr);
	virtual ~InputMavlinkCmdMount();

	virtual void print_status();

protected:
	virtual int update_impl(unsigned int timeout_ms, ControlData **control_data);
	virtual int initialize();

private:
	void _ack_vehicle_command(uint16_t command);

	int _vehicle_command_sub = -1;
	bool _allow_manual_control = true;
	InputRC *_manual_control;
	orb_advert_t _vehicle_command_ack_pub = nullptr;
	bool _stabilize[3] = { false, false, false };
};


} /* namespace vmount */
