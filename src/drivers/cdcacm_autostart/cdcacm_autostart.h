/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_armed.h>

#include <termios.h>

using namespace time_literals;

class CdcAcmAutostart : public ModuleBase<CdcAcmAutostart>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	CdcAcmAutostart();
	~CdcAcmAutostart() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase */
	int print_status() override;

	int Start();

private:

	enum class UsbAutoStartState {
		disconnected,
		connecting,
		connected,
		disconnecting,
	};

	enum class UsbProtocol {
		none,
		mavlink,
		nsh,
		ublox,
	};

	void Run() override;

	void UpdateParams(const bool force = false);

	void run_state_machine();

	void state_disconnected();
	void state_connecting();
	void state_connected();
	void state_disconnecting();

	bool scan_buffer_for_mavlink_reboot();
	bool scan_buffer_for_mavlink_heartbeat();
	bool scan_buffer_for_carriage_returns();
	bool scan_buffer_for_ublox_bytes();

	bool start_mavlink();
	bool start_nsh();
#if defined(CONFIG_SERIAL_PASSTHRU_UBLOX)
	bool start_ublox_serial_passthru(speed_t baudrate);
#endif
	int execute_process(char **argv);

	uORB::Subscription	_actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 500_ms};

	UsbAutoStartState _state{UsbAutoStartState::disconnected};
	UsbProtocol _active_protocol{UsbProtocol::none};
	bool _vbus_present = false;
	bool _vbus_present_prev = false;
	int _ttyacm_fd = -1;

	char _buffer[80] = {};
	int _bytes_read = 0;

	uint32_t _reschedule_time = 0;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_USB_AUTO>) _sys_usb_auto,
		(ParamInt<px4::params::USB_MAV_MODE>) _usb_mav_mode
	)
};
