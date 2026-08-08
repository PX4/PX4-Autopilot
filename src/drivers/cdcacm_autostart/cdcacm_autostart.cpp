/****************************************************************************
 *
 *   Copyright (c) 2023-2026 PX4 Development Team. All rights reserved.
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
#include <px4_platform_common/posix.h>

#include <errno.h>
#include <signal.h>

ModuleBase::Descriptor CdcAcmAutostart::desc{task_spawn, custom_command, print_usage};

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

	if (_active_protocol == UsbProtocol::mavlink) {
		stop_mavlink();
	}

	close_ttyacm();
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
		exit_and_cleanup(desc);
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
	actuator_armed_s report{};
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

void CdcAcmAutostart::close_ttyacm()
{
	if (_ttyacm_fd >= 0) {
		px4_close(_ttyacm_fd);
		_ttyacm_fd = -1;
	}
}

bool CdcAcmAutostart::process_running(int pid) const
{
	if (pid <= 0) {
		return false;
	}

	// kill(pid, 0) succeeds when the task exists (NuttX/POSIX).
	return kill(pid, 0) == 0;
}

void CdcAcmAutostart::state_connected()
{
	// Lost VBUS for two consecutive samples: tear down.
	if (!_vbus_present && !_vbus_present_prev) {
		PX4_DEBUG("lost vbus");

		if (_active_protocol == UsbProtocol::mavlink) {
			stop_mavlink();
		}

		_state = UsbAutoStartState::disconnecting;
		return;
	}

	// SYS_USB_AUTO=2 (and autodetect mavlink): if the mavlink task exited after a
	// successful spawn (e.g. failed to open the UART after retries), restart it.
	if (_active_protocol == UsbProtocol::mavlink && !process_running(_mavlink_pid)) {
		PX4_WARN("mavlink on %s exited, restarting", USB_DEVICE_PATH);
		_mavlink_pid = -1;

		if (start_mavlink()) {
			// Stay connected; next tick re-checks the new PID.
		} else {
			_state = UsbAutoStartState::disconnecting;
			_reschedule_time = 100_ms;
		}
	}
}

void CdcAcmAutostart::state_disconnected()
{
	if (_vbus_present && _vbus_present_prev) {
		PX4_DEBUG("starting sercon");

		if (sercon_main(0, nullptr) == EXIT_SUCCESS) {
			_state = UsbAutoStartState::connecting;
			PX4_DEBUG("state connecting");
			// Give CDC/ACM time to create /dev/ttyACM0 before mavlink open retries.
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

	// SYS_USB_AUTO=2: always start MAVLink. Do not open/hold the port here —
	// mavlink opens it itself (with open retries). Holding O_RDONLY previously
	// was unnecessary and left a second open on the device for the life of the link.
	if (_sys_usb_auto.get() == 2) {
		close_ttyacm();
		PX4_INFO("Starting mavlink on %s (SYS_USB_AUTO=2)", USB_DEVICE_PATH);

		if (start_mavlink()) {
			_state = UsbAutoStartState::connected;
			_active_protocol = UsbProtocol::mavlink;

		} else {
			_state = UsbAutoStartState::disconnecting;
			_reschedule_time = 100_ms;
		}

		return;
	}

	// SYS_USB_AUTO=0: sercon only, no protocol.
	if (_sys_usb_auto.get() == 0) {
		close_ttyacm();
		_state = UsbAutoStartState::connected;
		_active_protocol = UsbProtocol::none;
		return;
	}

	// SYS_USB_AUTO=1: autodetect from host bytes.
	if (_ttyacm_fd < 0) {
		PX4_DEBUG("opening port");
		_ttyacm_fd = px4_open(USB_DEVICE_PATH, O_RDONLY | O_NONBLOCK);
	}

	if (_ttyacm_fd < 0) {
		PX4_DEBUG("can't open port");
		// Port not ready yet (e.g. USB power-only / not fully enumerated). Keep trying.
		return;
	}

	if ((px4_ioctl(_ttyacm_fd, FIONREAD, &bytes_available) != PX4_OK) ||
	    (bytes_available < 3)) {
		PX4_DEBUG("bytes_available: %d", bytes_available);
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
		return;
	}

#if defined(CONFIG_SERIAL_PASSTHRU_UBLOX)
	// Get the baudrate for serial passthru before closing the port.
	tcgetattr(_ttyacm_fd, &uart_config);
	baudrate = cfgetspeed(&uart_config);
#endif
	PX4_DEBUG("_bytes_read %d", _bytes_read);

	// Release the probe open before starting any protocol that needs the device.
	close_ttyacm();

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
	close_ttyacm();
	_state = UsbAutoStartState::disconnecting;
}

void CdcAcmAutostart::state_disconnecting()
{
	PX4_DEBUG("state_disconnecting");

	close_ttyacm();
	_mavlink_pid = -1;

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
	bool launch_mavlink = false;

	if (_bytes_read < MAVLINK_HEARTBEAT_MIN_LENGTH) {
		return launch_mavlink;
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
			launch_mavlink = true;
			break;

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
			launch_mavlink = true;
			break;
		}
	}

	return launch_mavlink;
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
	for (int i = 0; i < _bytes_read - 1; i++) {
		bool ub = _buffer[i] == 0xb5 && _buffer[i + 1] == 0x62;

		if (ub && (i + 3) < _bytes_read &&
		    ((_buffer[i + 2] == 0x6 && (_buffer[i + 3] == 0xb8 || _buffer[i + 3] == 0x13)) ||
		     (_buffer[i + 2] == 0xa && _buffer[i + 3] == 0x4))) {
			PX4_INFO("%s: launching ublox serial passthru", USB_DEVICE_PATH);
			success = true;
			break;
		}
	}

	return success;
}

bool CdcAcmAutostart::start_mavlink()
{
	// Do not double-start if a prior instance is still alive.
	if (process_running(_mavlink_pid)) {
		return true;
	}

	char mavlink_mode_string[16];
	snprintf(mavlink_mode_string, sizeof(mavlink_mode_string), "%ld", _usb_mav_mode.get());

	// Non-static argv: NuttX task_spawn copies argv into the new task stack before return.
	char *argv[] { (char *)"mavlink", (char *)"start", (char *)"-d", (char *)USB_DEVICE_PATH,
		       (char *)"-m", mavlink_mode_string, nullptr
		     };

	const int pid = execute_process(argv);

	if (pid > 0) {
		_mavlink_pid = pid;
		return true;
	}

	_mavlink_pid = -1;
	return false;
}

void CdcAcmAutostart::stop_mavlink()
{
	char *stop_argv[] { (char *)"mavlink", (char *)"stop", (char *)"-d", (char *)USB_DEVICE_PATH, nullptr };
	execute_process(stop_argv);
	_mavlink_pid = -1;
}

bool CdcAcmAutostart::start_nsh()
{
	char *argv[] { (char *)"nshterm", (char *)USB_DEVICE_PATH, nullptr };
	return execute_process(argv) > 0;
}

#if defined(CONFIG_SERIAL_PASSTHRU_UBLOX)
bool CdcAcmAutostart::start_ublox_serial_passthru(speed_t baudrate)
{
	char baudstring[16];
	snprintf(baudstring, sizeof(baudstring), "%ld", baudrate);

	// Stop the GPS driver first
	char *gps_argv[] { (char *)"gps", (char *)"stop", nullptr };
	char *passthru_argv[] {
		(char *)"serial_passthru", (char *)"start", (char *)"-t", (char *)"-b", baudstring,
		(char *)"-e", (char *)USB_DEVICE_PATH, (char *)"-d", (char *)SERIAL_PASSTHRU_UBLOX_DEV, nullptr
	};

	if (execute_process(gps_argv) > 0) {
		if (execute_process(passthru_argv) > 0) {
			return true;
		}
	}

	return false;
}
#endif

int CdcAcmAutostart::execute_process(char *const *argv)
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

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;

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

	PX4_INFO("State: %s", state);
	PX4_INFO("Protocol: %s", protocol);
	PX4_INFO("VBUS: %s", _vbus_present ? "present" : "absent");
	PX4_INFO("SYS_USB_AUTO: %ld", _sys_usb_auto.get());

	if (_active_protocol == UsbProtocol::mavlink) {
		PX4_INFO("mavlink pid: %d (%s)", _mavlink_pid, process_running(_mavlink_pid) ? "running" : "not running");
	}

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
Manages the USB CDC/ACM serial device (`/dev/ttyACM0`).

`SYS_USB_AUTO` selects the protocol policy once USB VBUS is detected:
- `0` Disabled: bring up the USB serial device only.
- `1` Auto-detect: wait for host bytes and start MAVLink, nsh, or u-blox passthrough.
- `2` MAVLink (default): start MAVLink immediately so the autopilot transmits first

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
	return ModuleBase::main(CdcAcmAutostart::desc, argc, argv);
#endif
	return 1;
}
