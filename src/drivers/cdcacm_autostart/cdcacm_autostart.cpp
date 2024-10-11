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

#if defined(CONFIG_SYSTEM_CDCACM)

#include "cdcacm_autostart.h"

__BEGIN_DECLS
#include <arch/board/board.h>
#include <builtin/builtin.h>

extern int sercon_main(int c, char **argv);
extern int serdis_main(int c, char **argv);
__END_DECLS

#include <px4_platform_common/shutdown.h>

#define USB_DEVICE_PATH "/dev/ttyACM0"

#if defined(CONFIG_SERIAL_PASSTHRU_UBLOX)
#  undef SERIAL_PASSTHRU_UBLOX_DEV
#  if defined(CONFIG_SERIAL_PASSTHRU_GPS1) && defined(CONFIG_BOARD_SERIAL_GPS1)
#    define SERIAL_PASSTHRU_UBLOX_DEV CONFIG_BOARD_SERIAL_GPS1
#  elif defined(CONFIG_SERIAL_PASSTHRU_GPS2)&& defined(CONFIG_BOARD_SERIAL_GPS2)
#    define SERIAL_PASSTHRU_UBLOX_DEV CONFIG_BOARD_SERIAL_GPS2
#  elif defined(CONFIG_SERIAL_PASSTHRU_GPS3)&& defined(CONFIG_BOARD_SERIAL_GPS3)
#    define SERIAL_PASSTHRU_UBLOX_DEV CONFIG_BOARD_SERIAL_GPS3
#  elif defined(CONFIG_SERIAL_PASSTHRU_GPS4)&& defined(CONFIG_BOARD_SERIAL_GPS4)
#    define SERIAL_PASSTHRU_UBLOX_DEV CONFIG_BOARD_SERIAL_GPS4
#  elif defined(CONFIG_SERIAL_PASSTHRU_GPS5) && defined(CONFIG_BOARD_SERIAL_GPS5)
#    define SERIAL_PASSTHRU_UBLOX_DEV CONFIG_BOARD_SERIAL_GPS5
#  endif
#  if !defined(SERIAL_PASSTHRU_UBLOX_DEV)
#    error "CONFIG_SERIAL_PASSTHRU_GPSn and CONFIG_BOARD_SERIAL_GPSn must be defined"
#  endif
#endif

CdcAcmAutostart::CdcAcmAutostart() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{}

CdcAcmAutostart::~CdcAcmAutostart()
{
	PX4_INFO("Stopping CDC/ACM autostart");

	if (_ttyacm_fd >= 0) {
		px4_close(_ttyacm_fd);
		_ttyacm_fd = -1;
	}

	ScheduleClear();
}

int CdcAcmAutostart::Start()
{
	PX4_INFO("Starting CDC/ACM autostart");
	UpdateParams(true);

	ScheduleNow();

	return PX4_OK;
}

void CdcAcmAutostart::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	UpdateParams();

	run_state_machine();
}

void CdcAcmAutostart::run_state_machine()
{
	_reschedule_time = 500_ms;
	_vbus_present = (board_read_VBUS_state() == PX4_OK);

	// If the hardware supports RESET lockout that has nArmed ANDed with VBUS
	// vbus_sense may drop during a param save which uses
	// BOARD_INDICATE_EXTERNAL_LOCKOUT_STATE to prevent external resets
	// while writing the params.  If we are not armed and nARMRED is low
	// we are in such a lock out so ignore changes on VBUS_SENSE during this
	// time.
#if defined(BOARD_GET_EXTERNAL_LOCKOUT_STATE)

	if (BOARD_GET_EXTERNAL_LOCKOUT_STATE() == 0) {
		_vbus_present = _vbus_present_prev;
		ScheduleDelayed(500_ms);
		return;
	}

#endif

	// Do not reconfigure USB while flying
	actuator_armed_s report;
	_actuator_armed_sub.copy(&report);

	if (report.armed) {
		_vbus_present_prev = _vbus_present;

	} else {

		switch (_state) {
		case UsbAutoStartState::disconnected:
			state_disconnected();
			break;

		case UsbAutoStartState::connecting:
			state_connecting();
			break;

		case UsbAutoStartState::connected:
			state_connected();
			break;

		case UsbAutoStartState::disconnecting:
			state_disconnecting();
			break;
		}
	}

	_vbus_present_prev = _vbus_present;
	ScheduleDelayed(_reschedule_time);
}

void CdcAcmAutostart::state_connected()
{
	if (!_vbus_present && !_vbus_present_prev && (_active_protocol == UsbProtocol::mavlink)) {
		PX4_DEBUG("lost vbus!");
		sched_lock();
		static const char app[] {"mavlink"};
		static const char *stop_argv[] {"mavlink", "stop", "-d", USB_DEVICE_PATH, NULL};
		exec_builtin(app, (char **)stop_argv, NULL, 0);
		sched_unlock();
		_state = UsbAutoStartState::disconnecting;
	}
}

void CdcAcmAutostart::state_disconnected()
{
	if (_vbus_present && _vbus_present_prev) {
		PX4_DEBUG("starting sercon");

		if (sercon_main(0, nullptr) == EXIT_SUCCESS) {
			_state = UsbAutoStartState::connecting;
			PX4_DEBUG("state connecting");
			_reschedule_time = 1_s;
		}

	} else if (_vbus_present && !_vbus_present_prev) {
		// USB just connected, try again soon
		_reschedule_time = 100_ms;
	}
}

void CdcAcmAutostart::state_connecting()
{
	int bytes_available = 0;
#if defined(CONFIG_SERIAL_PASSTHRU_UBLOX)
	struct termios uart_config;
	speed_t baudrate;
#endif

	if (!_vbus_present) {
		PX4_DEBUG("No VBUS");
		goto fail;
	}

	if (_ttyacm_fd < 0) {
		PX4_DEBUG("opening port");
		_ttyacm_fd = px4_open(USB_DEVICE_PATH, O_RDONLY | O_NONBLOCK);
	}

	if (_ttyacm_fd < 0) {
		PX4_DEBUG("can't open port");
		// fail silently and keep trying to open the port
		return;
	}

	if (_sys_usb_auto.get() == 2) {
		PX4_INFO("Starting mavlink on %s (SYS_USB_AUTO=2)", USB_DEVICE_PATH);

		if (start_mavlink()) {
			_state = UsbAutoStartState::connected;
			_active_protocol = UsbProtocol::mavlink;

		} else {
			_state = UsbAutoStartState::disconnecting;
			_reschedule_time = 100_ms;
		}

		return;

	} else if (_sys_usb_auto.get() == 0) {
		// Do nothing
		_state = UsbAutoStartState::connected;
		_active_protocol = UsbProtocol::none;
		return;
	}

	// Otherwise autodetect

	if ((px4_ioctl(_ttyacm_fd, FIONREAD, &bytes_available) != PX4_OK) ||
	    (bytes_available < 3)) {
		PX4_DEBUG("bytes_available: %d", bytes_available);
		// Return back to connecting state to check again soon
		return;
	}

	// Non-blocking read
	_bytes_read = px4_read(_ttyacm_fd, _buffer, sizeof(_buffer));

#if defined(DEBUG_BUILD)

	if (_bytes_read > 0) {
		fprintf(stderr, "%d bytes\n", _bytes_read);

		for (int i = 0; i < _bytes_read; i++) {
			fprintf(stderr, "|%X", _buffer[i]);
		}

		fprintf(stderr, "\n");
	}

#endif // DEBUG_BUILD

	if (_bytes_read <= 0) {
		PX4_DEBUG("no _bytes_read");
		// Return back to connecting state to check again soon
		return;
	}

#if defined(CONFIG_SERIAL_PASSTHRU_UBLOX)
	// Get the baudrate for serial passthru before closing the port.
	tcgetattr(_ttyacm_fd, &uart_config);
	baudrate = cfgetspeed(&uart_config);
#endif
	PX4_DEBUG("_bytes_read %d", _bytes_read);
	px4_close(_ttyacm_fd);
	_ttyacm_fd = -1;

	// Parse for mavlink reboot command
	if (scan_buffer_for_mavlink_reboot()) {
		// Reboot incoming. Return without rescheduling.
		return;
	}

	// Parse for mavlink heartbeats (v1 and v2).
	if (scan_buffer_for_mavlink_heartbeat()) {
		if (start_mavlink()) {
			_state = UsbAutoStartState::connected;
			_active_protocol = UsbProtocol::mavlink;

		} else {
			_state = UsbAutoStartState::disconnecting;
			_reschedule_time = 100_ms;
		}

		return;
	}

	// Parse for carriage returns indicating someone is trying to use the nsh.
	if (scan_buffer_for_carriage_returns()) {
		if (start_nsh()) {
			_state = UsbAutoStartState::connected;
			_active_protocol = UsbProtocol::nsh;

		} else {
			_state = UsbAutoStartState::disconnecting;
			_reschedule_time = 100_ms;
		}

		return;
	}

#if defined(CONFIG_SERIAL_PASSTHRU_UBLOX)

	// Parse for ublox start of packet byte sequence.
	if (scan_buffer_for_ublox_bytes()) {
		if (start_ublox_serial_passthru(baudrate)) {
			_state = UsbAutoStartState::connected;
			_active_protocol = UsbProtocol::ublox;

		} else {
			_state = UsbAutoStartState::disconnecting;
			_reschedule_time = 100_ms;
		}

		return;
	}

#endif

	return;

fail:
	PX4_DEBUG("fail...");

	// VBUS not present, open failed
	if (_ttyacm_fd >= 0) {
		px4_close(_ttyacm_fd);
		_ttyacm_fd = -1;
	}

	_state = UsbAutoStartState::disconnecting;
}

void CdcAcmAutostart::state_disconnecting()
{
	PX4_DEBUG("state_disconnecting");

	if (_ttyacm_fd > 0) {
		px4_close(_ttyacm_fd);
		_ttyacm_fd = -1;
	}

	// Disconnect serial
	serdis_main(0, NULL);
	_state = UsbAutoStartState::disconnected;
	_active_protocol = UsbProtocol::none;
}

bool CdcAcmAutostart::scan_buffer_for_mavlink_reboot()
{
	bool rebooting = false;

	// Mavlink reboot/shutdown command
	// COMMAND_LONG (#76) with command MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (246)
	static constexpr int MAVLINK_COMMAND_LONG_MIN_LENGTH = 41;

	if (_bytes_read < MAVLINK_COMMAND_LONG_MIN_LENGTH) {
		return rebooting;
	}

	// scan buffer for mavlink COMMAND_LONG
	for (int i = 0; i < _bytes_read - MAVLINK_COMMAND_LONG_MIN_LENGTH; i++) {
		if ((_buffer[i] == 0xFE)        // Mavlink v1 start byte
		    && (_buffer[i + 5] == 76)   //  76=0x4C COMMAND_LONG
		    && (_buffer[i + 34] == 246) // 246=0xF6 MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
		   ) {
			// mavlink v1 COMMAND_LONG
			//  buffer[0]: start byte (0xFE for mavlink v1)
			//  buffer[3]: SYSID
			//  buffer[4]: COMPID
			//  buffer[5]: message id (COMMAND_LONG 76=0x4C)
			//  buffer[6-10]: COMMAND_LONG param 1 (little endian float)
			//  buffer[34]: COMMAND_LONG command MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (246/0xF6)
			float param1_raw = 0;
			memcpy(&param1_raw, &_buffer[i + 6], 4);
			int param1 = roundf(param1_raw);

			PX4_INFO("%s: Mavlink MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN param 1: %d (SYSID:%d COMPID:%d)",
				 USB_DEVICE_PATH, param1, _buffer[i + 3], _buffer[i + 4]);

			if (param1 == 1) {
				// 1: Reboot autopilot
				rebooting = true;
				px4_reboot_request(REBOOT_REQUEST, 0);

			} else if (param1 == 2) {
				// 2: Shutdown autopilot
#if defined(BOARD_HAS_POWER_CONTROL)
				rebooting = true;
				px4_shutdown_request(0);
#endif // BOARD_HAS_POWER_CONTROL

			} else if (param1 == 3) {
				// 3: Reboot autopilot and keep it in the bootloader until upgraded.
				rebooting = true;
				px4_reboot_request(REBOOT_TO_BOOTLOADER, 0);
			}
		}
	}

	return rebooting;
}

bool CdcAcmAutostart::scan_buffer_for_mavlink_heartbeat()
{
	static constexpr int MAVLINK_HEARTBEAT_MIN_LENGTH = 9;
	bool start_mavlink = false;

	if (_bytes_read < MAVLINK_HEARTBEAT_MIN_LENGTH) {
		return start_mavlink;
	}

	// scan buffer for mavlink HEARTBEAT (v1 & v2)
	for (int i = 0; i < _bytes_read - MAVLINK_HEARTBEAT_MIN_LENGTH; i++) {
		if ((_buffer[i] == 0xFE) && (_buffer[i + 1] == 9) && (_buffer[i + 5] == 0)) {
			// mavlink v1 HEARTBEAT
			//  buffer[0]: start byte (0xFE for mavlink v1)
			//  buffer[1]: length (9 for HEARTBEAT)
			//  buffer[3]: SYSID
			//  buffer[4]: COMPID
			//  buffer[5]: mavlink message id (0 for HEARTBEAT)
			PX4_INFO("%s: launching mavlink (HEARTBEAT v1 from SYSID:%d COMPID:%d)",
				 USB_DEVICE_PATH, _buffer[i + 3], _buffer[i + 4]);
			start_mavlink = true;

		} else if ((_buffer[i] == 0xFD) && (_buffer[i + 1] == 9)
			   && (_buffer[i + 7] == 0) && (_buffer[i + 8] == 0) && (_buffer[i + 9] == 0)) {
			// mavlink v2 HEARTBEAT
			//  buffer[0]: start byte (0xFD for mavlink v2)
			//  buffer[1]: length (9 for HEARTBEAT)
			//  buffer[5]: SYSID
			//  buffer[6]: COMPID
			//  buffer[7:9]: mavlink message id (0 for HEARTBEAT)
			PX4_INFO("%s: launching mavlink (HEARTBEAT v2 from SYSID:%d COMPID:%d)",
				 USB_DEVICE_PATH, _buffer[i + 5], _buffer[i + 6]);
			start_mavlink = true;
		}
	}

	return start_mavlink;
}

bool CdcAcmAutostart::scan_buffer_for_carriage_returns()
{
	bool start_nsh = false;

	if (_bytes_read < 3) {
		return start_nsh;
	}

	// nshterm (3 carriage returns)
	// scan buffer looking for 3 consecutive carriage returns (0xD)
	for (int i = 1; i < _bytes_read - 1; i++) {
		if (_buffer[i - 1] == 0xD && _buffer[i] == 0xD && _buffer[i + 1] == 0xD) {
			PX4_INFO("%s: launching nshterm", USB_DEVICE_PATH);
			start_nsh = true;
			break;
		}
	}

	return start_nsh;
}

bool CdcAcmAutostart::scan_buffer_for_ublox_bytes()
{
	bool success = false;

	if (_bytes_read < 4) {
		return success;
	}

	// scan buffer looking for 0xb5 0x62 which indicates the start of a packet
	for (int i = 0; i < _bytes_read; i++) {
		bool ub = _buffer[i] == 0xb5 && _buffer[i + 1] == 0x62;

		if (ub && ((_buffer[i + 2 ] == 0x6 && (_buffer[i + 3 ] == 0xb8 || _buffer[i + 3 ] == 0x13)) ||
			   (_buffer[i + 2 ] == 0xa && _buffer[i + 3 ] == 0x4))) {
			PX4_INFO("%s: launching ublox serial passthru", USB_DEVICE_PATH);
			success = true;
			break;
		}
	}

	return success;
}

bool CdcAcmAutostart::start_mavlink()
{
	bool success = false;
	char mavlink_mode_string[3];
	snprintf(mavlink_mode_string, sizeof(mavlink_mode_string), "%ld", _usb_mav_mode.get());
	static const char *argv[] {"mavlink", "start", "-d", USB_DEVICE_PATH, "-m", mavlink_mode_string, nullptr};

	if (execute_process((char **)argv) > 0) {
		success = true;
	}

	return success;
}

bool CdcAcmAutostart::start_nsh()
{
	bool success = false;
	static const char *argv[] {"nshterm", USB_DEVICE_PATH, nullptr};

	if (execute_process((char **)argv) > 0) {
		success = true;
	}

	return success;
}

#if defined(CONFIG_SERIAL_PASSTHRU_UBLOX)
bool CdcAcmAutostart::start_ublox_serial_passthru(speed_t baudrate)
{
	bool success = false;
	char baudstring[16];
	snprintf(baudstring, sizeof(baudstring), "%ld", baudrate);

	// Stop the GPS driver first
	static const char *gps_argv[] {"gps", "stop", nullptr};
	static const char *passthru_argv[] {"serial_passthru", "start", "-t", "-b", baudstring, "-e", USB_DEVICE_PATH, "-d", SERIAL_PASSTHRU_UBLOX_DEV, nullptr};

	if (execute_process((char **)gps_argv) > 0) {
		if (execute_process((char **)passthru_argv) > 0) {
			success = true;
		}
	}

	return success;
}
#endif

int CdcAcmAutostart::execute_process(char **argv)
{
	int pid = -1;
	sched_lock();

	pid = exec_builtin(argv[0], argv, nullptr, 0);

	sched_unlock();
	return pid;
}

int CdcAcmAutostart::task_spawn(int argc, char *argv[])
{
	CdcAcmAutostart *instance = new CdcAcmAutostart();

	if (!instance) {
		PX4_ERR("alloc failed");
		return -1;
	}

	int ret = instance->Start();

	if (ret != PX4_OK) {
		delete instance;
		return ret;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	return ret;
}

void CdcAcmAutostart::UpdateParams(const bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		ModuleParams::updateParams();
	}
}

int CdcAcmAutostart::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CdcAcmAutostart::print_status()
{
	const char *state = "";
	const char *protocol = "";

	switch (_state) {
	case UsbAutoStartState::disconnected:
		state = "disconnected";
		break;

	case UsbAutoStartState::connecting:
		state = "connecting";
		break;

	case UsbAutoStartState::connected:
		state = "connected";
		break;

	case UsbAutoStartState::disconnecting:
		state = "disconnecting";
		break;
	}

	switch (_active_protocol) {
	case UsbProtocol::none:
		protocol = "none";
		break;

	case UsbProtocol::mavlink:
		protocol = "mavlink";
		break;

	case UsbProtocol::nsh:
		protocol = "nsh";
		break;

	case UsbProtocol::ublox:
		protocol = "ublox";
		break;
	}

	PX4_INFO("Running");
	PX4_INFO("State: %s", state);
	PX4_INFO("Protocol: %s", protocol);
	return PX4_OK;
}

int CdcAcmAutostart::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module listens on USB and auto-configures the protocol depending on the bytes received.
The supported protocols are: MAVLink, nsh, and ublox serial passthrough. If the parameter SYS_USB_AUTO=2
the module will only try to start mavlink as long as the USB VBUS is detected. Otherwise it will spin
and continue to check for VBUS and start mavlink once it is detected.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("cdcacm_autostart", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

#endif

extern "C" __EXPORT int cdcacm_autostart_main(int argc, char *argv[])
{
#if defined(CONFIG_SYSTEM_CDCACM)
	return CdcAcmAutostart::main(argc, argv);
#endif
	return 1;
}
