/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file EscFlasher.cpp
 *
 * AM32 ESC firmware update module.
 */

#include "EscFlasher.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <px4_platform_common/px4_config.h>

#ifdef __PX4_NUTTX
#include <px4_arch/io_timer.h>
#endif

using namespace time_literals;

EscFlasher::EscFlasher() :
	ModuleParams(nullptr)
{
}

EscFlasher::~EscFlasher()
{
	cleanup();
}

// ───────────────────────────── Module lifecycle ─────────────────────────────

int EscFlasher::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("esc_flasher",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      PX4_STACK_ADJUSTED(4096),
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

EscFlasher *EscFlasher::instantiate(int argc, char *argv[])
{
	EscFlasher *instance = new EscFlasher();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

void EscFlasher::run()
{
	while (!should_exit()) {

		switch (_state) {
		case State::Idle:
			handle_vehicle_command();
			break;

		case State::StoppingDShot:
			state_stopping_dshot();
			break;

		case State::InitGPIOs:
			state_init_gpios();
			break;

		case State::ConnectESC:
			state_connect_esc();
			break;

		case State::ReadInfo:
			state_read_info();
			break;

		case State::LoadFirmware:
			state_load_firmware();
			break;

		case State::Flashing:
			state_flash_chunk();
			break;

		case State::RunApp:
			state_run_app();
			break;

		case State::Complete:
			state_complete();
			break;

		case State::Failed:
			state_failed();
			break;
		}

		// Idle → slow poll; active → no extra sleep (protocol
		// bit-bang already takes wall-clock time per chunk)
		if (_state == State::Idle) {
			px4_usleep(IDLE_INTERVAL_US);
		}
	}
}

// ───────────────────────────── Vehicle command ──────────────────────────────

void EscFlasher::handle_vehicle_command()
{
	vehicle_command_s cmd;

	while (_vehicle_cmd_sub.update(&cmd)) {
		if (cmd.command != vehicle_command_s::VEHICLE_CMD_ESC_FIRMWARE_UPDATE) {
			continue;
		}

		// Must be disarmed
		vehicle_status_s vs{};
		_vehicle_status_sub.copy(&vs);

		if (vs.arming_state != vehicle_status_s::ARMING_STATE_DISARMED) {
			PX4_ERR("cannot flash ESCs while armed");
			ack_vehicle_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED);
			_status.error = esc_firmware_update_status_s::ERROR_NOT_DISARMED;
			publish_status();
			continue;
		}

		ack_vehicle_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);

		uint8_t esc_index = (uint8_t)cmd.param1;  // 255 = all
		begin_update(esc_index);
		return;  // exit the poll loop — we are now active
	}
}

void EscFlasher::ack_vehicle_command(const vehicle_command_s &cmd, uint8_t result)
{
	vehicle_command_ack_s ack{};
	ack.command          = cmd.command;
	ack.result           = result;
	ack.target_system    = cmd.source_system;
	ack.target_component = cmd.source_component;
	ack.timestamp        = hrt_absolute_time();
	_cmd_ack_pub.publish(ack);
}

void EscFlasher::begin_update(uint8_t esc_index)
{
	_target_esc     = esc_index;
	_current_esc    = 0;
	_esc_count      = 0;
	_flash_started  = hrt_absolute_time();

	memset(&_status, 0, sizeof(_status));
	_status.status = esc_firmware_update_status_s::STATUS_CONNECTING;
	publish_status();

	mavlink_log_info(&_mavlink_log_pub, "ESC firmware update starting");

	transition(State::StoppingDShot);
}

// ───────────────────────────── State: StoppingDShot ─────────────────────────

void EscFlasher::state_stopping_dshot()
{
	PX4_INFO("stopping dshot driver");

	int ret = system("dshot stop");

	if (ret != 0) {
		PX4_WARN("dshot stop returned %d (may already be stopped)", ret);
	}

	// Give DShot destructor time to release DMA/timers
	px4_usleep(200000);  // 200 ms

	transition(State::InitGPIOs);
}

// ───────────────────────────── State: InitGPIOs ─────────────────────────────

void EscFlasher::state_init_gpios()
{
#ifdef __PX4_NUTTX

	// Discover motor GPIO pins from the io_timer subsystem
	_esc_count = 0;

	for (unsigned ch = 0; ch < MAX_ESCS; ch++) {
		uint32_t gpio = io_timer_channel_get_gpio_output(ch);

		if (gpio != 0) {
			_gpio_pins[_esc_count] = gpio & (GPIO_PORT_MASK | GPIO_PIN_MASK);
			_esc_count++;
		}
	}

	if (_esc_count == 0) {
		PX4_ERR("no motor GPIOs found");
		fail(esc_firmware_update_status_s::ERROR_DSHOT_STOP_FAILED);
		return;
	}

	PX4_INFO("found %u motor GPIO(s)", _esc_count);

	// If targeting a specific ESC, validate index
	if (_target_esc != 255 && _target_esc >= _esc_count) {
		PX4_ERR("ESC index %u out of range (have %u)", _target_esc, _esc_count);
		fail(esc_firmware_update_status_s::ERROR_UNSUPPORTED_HARDWARE);
		return;
	}

	// Drive all target pins HIGH to force ESCs into bootloader on next reset
	for (uint8_t i = 0; i < _esc_count; i++) {
		bool target = (_target_esc == 255) || (i == _target_esc);

		if (target) {
			uint32_t gpio_out = _gpio_pins[i]
					    | GPIO_OUTPUT | GPIO_OUTPUT_SET | GPIO_PULLUP;
			px4_arch_configgpio(gpio_out);
		}
	}

	PX4_INFO("GPIOs HIGH — waiting %u ms for ESC bootloader entry",
		 (unsigned)(GPIO_SETTLE_TIME_US / 1000));

	_status.esc_count = (_target_esc == 255) ? _esc_count : 1;
	publish_status();

	// Wait for ESCs to detect HIGH and enter bootloader
	px4_usleep(GPIO_SETTLE_TIME_US);

	// Set starting ESC
	if (_target_esc != 255) {
		_current_esc = _target_esc;

	} else {
		_current_esc = 0;
	}

	transition(State::ConnectESC);

#else
	PX4_ERR("GPIO operations not supported on this platform");
	fail(esc_firmware_update_status_s::ERROR_DSHOT_STOP_FAILED);
#endif
}

// ───────────────────────────── State: ConnectESC ────────────────────────────

void EscFlasher::state_connect_esc()
{
	PX4_INFO("connecting to ESC %u", _current_esc);

	_status.esc_index = _current_esc;
	_status.status    = esc_firmware_update_status_s::STATUS_CONNECTING;
	_status.error     = esc_firmware_update_status_s::ERROR_NONE;
	publish_status();

	int ret = am32::handshake(_gpio_pins[_current_esc], _device_info);

	if (ret != am32::OK) {
		PX4_ERR("bootloader handshake failed on ESC %u (err %d)", _current_esc, ret);
		fail(esc_firmware_update_status_s::ERROR_COMMS_TIMEOUT);
		return;
	}

	PX4_INFO("ESC %u bootloader: %02x %02x %02x %02x %02x %02x %02x %02x %02x",
		 _current_esc,
		 _device_info[0], _device_info[1], _device_info[2],
		 _device_info[3], _device_info[4], _device_info[5],
		 _device_info[6], _device_info[7], _device_info[8]);

	transition(State::ReadInfo);
}

// ───────────────────────────── State: ReadInfo ──────────────────────────────

void EscFlasher::state_read_info()
{
	_status.status = esc_firmware_update_status_s::STATUS_READING_INFO;
	publish_status();

	// Device info already obtained during handshake.
	// Future: read firmware tag to auto-select firmware file.

	transition(State::LoadFirmware);
}

// ───────────────────────────── State: LoadFirmware ──────────────────────────

void EscFlasher::state_load_firmware()
{
	// Close any previously open file (when looping over ESCs)
	if (_fw_fd >= 0) {
		close(_fw_fd);
		_fw_fd = -1;
	}

	_fw_fd = open(FIRMWARE_PATH, O_RDONLY);

	if (_fw_fd < 0) {
		PX4_ERR("cannot open %s", FIRMWARE_PATH);
		mavlink_log_critical(&_mavlink_log_pub, "No firmware file on SD card");
		fail(esc_firmware_update_status_s::ERROR_NO_FIRMWARE_FILE);
		return;
	}

	struct stat st;

	if (fstat(_fw_fd, &st) != 0 || st.st_size == 0) {
		PX4_ERR("cannot stat firmware file");
		close(_fw_fd);
		_fw_fd = -1;
		fail(esc_firmware_update_status_s::ERROR_NO_FIRMWARE_FILE);
		return;
	}

	_fw_size    = (uint32_t)st.st_size;
	_fw_written = 0;

	PX4_INFO("firmware %s: %lu bytes, %lu chunks",
		 FIRMWARE_PATH, (unsigned long)_fw_size,
		 (unsigned long)((_fw_size + am32::MAX_CHUNK_SIZE - 1) / am32::MAX_CHUNK_SIZE));

	transition(State::Flashing);
}

// ───────────────────────────── State: Flashing ──────────────────────────────

void EscFlasher::state_flash_chunk()
{
	_status.status = esc_firmware_update_status_s::STATUS_WRITING;

	uint32_t remaining = _fw_size - _fw_written;
	uint16_t chunk_len = (remaining > am32::MAX_CHUNK_SIZE)
			     ? am32::MAX_CHUNK_SIZE : (uint16_t)remaining;

	uint8_t buf[am32::MAX_CHUNK_SIZE];
	ssize_t nread = read(_fw_fd, buf, chunk_len);

	if (nread != (ssize_t)chunk_len) {
		PX4_ERR("firmware read error at offset %lu", (unsigned long)_fw_written);
		fail(esc_firmware_update_status_s::ERROR_NO_FIRMWARE_FILE);
		return;
	}

	// Set flash address (16-bit, relative to STM32 flash base)
	uint16_t address = (uint16_t)((am32::FIRMWARE_ADDR & 0xFFFF) + _fw_written);

	int ret = am32::set_address(_gpio_pins[_current_esc], address);

	if (ret != am32::OK) {
		PX4_ERR("set_address failed on ESC %u (err %d)", _current_esc, ret);
		fail(esc_firmware_update_status_s::ERROR_COMMS_TIMEOUT);
		return;
	}

	ret = am32::write_chunk(_gpio_pins[_current_esc], buf, chunk_len);

	if (ret != am32::OK) {
		PX4_ERR("write_chunk failed on ESC %u at 0x%04x (err %d)",
			_current_esc, address, ret);
		fail(esc_firmware_update_status_s::ERROR_CRC_MISMATCH);
		return;
	}

	_fw_written += chunk_len;

	// Progress for current ESC
	_status.progress_pct = (uint8_t)((_fw_written * 100) / _fw_size);

	// Overall progress across all ESCs
	uint8_t esc_total = (_target_esc == 255) ? _esc_count : 1;
	uint8_t esc_done;

	if (_target_esc == 255) {
		esc_done = _current_esc;

	} else {
		esc_done = 0;
	}

	_status.overall_progress_pct = (uint8_t)(
					       ((esc_done * 100) + _status.progress_pct) / esc_total);

	publish_status();

	uint32_t total_chunks = (_fw_size + am32::MAX_CHUNK_SIZE - 1) / am32::MAX_CHUNK_SIZE;
	uint32_t current_chunk = (_fw_written + am32::MAX_CHUNK_SIZE - 1) / am32::MAX_CHUNK_SIZE;
	PX4_INFO("ESC %u: chunk %lu/%lu (%u%%)",
		 _current_esc, (unsigned long)current_chunk,
		 (unsigned long)total_chunks, _status.progress_pct);

	if (_fw_written >= _fw_size) {
		transition(State::RunApp);
	}
}

// ───────────────────────────── State: RunApp ────────────────────────────────

void EscFlasher::state_run_app()
{
	PX4_INFO("sending RUN_APP to ESC %u", _current_esc);

	am32::run_app(_gpio_pins[_current_esc]);

	// Close firmware file so it can be re-opened for the next ESC
	if (_fw_fd >= 0) {
		close(_fw_fd);
		_fw_fd = -1;
	}

	advance_to_next_esc();
}

void EscFlasher::advance_to_next_esc()
{
	if (_target_esc != 255) {
		// Single-ESC mode — done
		transition(State::Complete);
		return;
	}

	_current_esc++;

	if (_current_esc >= _esc_count) {
		transition(State::Complete);

	} else {
		// Next ESC
		transition(State::ConnectESC);
	}
}

// ───────────────────────────── State: Complete ──────────────────────────────

void EscFlasher::state_complete()
{
	hrt_abstime elapsed = hrt_absolute_time() - _flash_started;
	unsigned seconds = (unsigned)(elapsed / 1000000);

	PX4_INFO("ESC firmware update complete in %u seconds", seconds);
	mavlink_log_info(&_mavlink_log_pub, "ESC flash done in %us — reboot required", seconds);

	_status.status               = esc_firmware_update_status_s::STATUS_REBOOT_REQUIRED;
	_status.progress_pct         = 100;
	_status.overall_progress_pct = 100;
	_status.error                = esc_firmware_update_status_s::ERROR_NONE;
	publish_status();

	cleanup();
	transition(State::Idle);
}

// ───────────────────────────── State: Failed ────────────────────────────────

void EscFlasher::state_failed()
{
	PX4_ERR("ESC firmware update failed (error %u)", _status.error);
	mavlink_log_critical(&_mavlink_log_pub, "ESC flash failed (error %u)", _status.error);

	_status.status = esc_firmware_update_status_s::STATUS_FAILED;
	publish_status();

	cleanup();
	transition(State::Idle);
}

// ───────────────────────────── Helpers ──────────────────────────────────────

void EscFlasher::transition(State s)
{
	_state         = s;
	_state_entered = hrt_absolute_time();
}

void EscFlasher::fail(uint8_t error_code)
{
	_status.error = error_code;
	transition(State::Failed);
}

void EscFlasher::cleanup()
{
	if (_fw_fd >= 0) {
		close(_fw_fd);
		_fw_fd = -1;
	}

	_fw_size    = 0;
	_fw_written = 0;
}

void EscFlasher::publish_status()
{
	_status.timestamp = hrt_absolute_time();
	_status_pub.publish(_status);
}

// ───────────────────────────── ModuleBase boilerplate ────────────────────────

int EscFlasher::print_status()
{
	static const char *state_names[] = {
		"Idle", "StoppingDShot", "InitGPIOs", "ConnectESC",
		"ReadInfo", "LoadFirmware", "Flashing", "RunApp",
		"Complete", "Failed"
	};

	PX4_INFO("state: %s", state_names[(int)_state]);
	PX4_INFO("ESC %u / %u, firmware %lu / %lu bytes",
		 _current_esc, _esc_count,
		 (unsigned long)_fw_written, (unsigned long)_fw_size);

	return 0;
}

int EscFlasher::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
AM32 ESC firmware update module.

Reads firmware from SD card, stops DShot, and flashes ESCs
via bit-banged bootloader protocol.  Triggered by the
VEHICLE_CMD_ESC_FIRMWARE_UPDATE vehicle command.
A reboot is required after flashing to restore normal operation.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("esc_flasher", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the module (auto-started on supported boards)");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int EscFlasher::custom_command(int argc, char *argv[])
{
	return print_usage("unrecognised command");
}

extern "C" __EXPORT int esc_flasher_main(int argc, char *argv[])
{
	return EscFlasher::main(argc, argv);
}
