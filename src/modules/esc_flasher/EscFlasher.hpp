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
 * @file EscFlasher.hpp
 *
 * AM32 ESC firmware update module.
 *
 * Triggered by MAV_CMD_ESC_FIRMWARE_UPDATE via vehicle_command.
 * Reads firmware from SD card, stops DShot, takes over motor GPIOs,
 * and flashes each ESC via bit-banged AM32 bootloader protocol.
 * Requires reboot to restore normal DShot operation.
 */

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <drivers/drv_hrt.h>
#include <systemlib/mavlink_log.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/esc_firmware_update_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_status.h>

#include "am32_protocol.hpp"

class EscFlasher : public ModuleBase<EscFlasher>, public ModuleParams
{
public:
	EscFlasher();
	~EscFlasher() override;

	EscFlasher(const EscFlasher &) = delete;
	EscFlasher &operator=(const EscFlasher &) = delete;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static EscFlasher *instantiate(int argc, char *argv[]);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:
	static constexpr uint8_t  MAX_ESCS = 8;
	static constexpr const char *FIRMWARE_PATH = "/fs/microsd/am32/firmware.bin";
	static constexpr uint32_t DSHOT_STOP_TIMEOUT_US  = 2000000;  // 2 s
	static constexpr uint32_t GPIO_SETTLE_TIME_US    = 600000;   // 600 ms
	static constexpr uint32_t IDLE_INTERVAL_US       = 1000000;  // 1 s

	enum class State : uint8_t {
		Idle,
		StoppingDShot,
		InitGPIOs,
		ConnectESC,
		ReadInfo,
		LoadFirmware,
		Flashing,
		RunApp,
		Complete,
		Failed
	};

	// --- state machine helpers ---
	void handle_vehicle_command();
	void ack_vehicle_command(const vehicle_command_s &cmd, uint8_t result);
	void begin_update(uint8_t esc_index);
	void advance_to_next_esc();
	void transition(State s);
	void fail(uint8_t error_code);
	void cleanup();
	void publish_status();

	// --- state handlers (called from run loop) ---
	void state_stopping_dshot();
	void state_init_gpios();
	void state_connect_esc();
	void state_read_info();
	void state_load_firmware();
	void state_flash_chunk();
	void state_run_app();
	void state_complete();
	void state_failed();

	// --- current state ---
	State _state{State::Idle};
	hrt_abstime _state_entered{0};
	hrt_abstime _flash_started{0};

	// --- ESC tracking ---
	uint8_t  _target_esc{255};          // from vehicle_command param1
	uint8_t  _current_esc{0};           // index into _gpio_pins[]
	uint8_t  _esc_count{0};
	uint32_t _gpio_pins[MAX_ESCS] {};
	uint8_t  _device_info[am32::DEVICE_INFO_SIZE] {};

	// --- firmware file ---
	int      _fw_fd{-1};
	uint32_t _fw_size{0};
	uint32_t _fw_written{0};

	// --- published status ---
	esc_firmware_update_status_s _status{};

	// --- publications ---
	uORB::Publication<esc_firmware_update_status_s> _status_pub{ORB_ID(esc_firmware_update_status)};
	uORB::Publication<vehicle_command_ack_s>        _cmd_ack_pub{ORB_ID(vehicle_command_ack)};

	// --- subscriptions ---
	uORB::Subscription _vehicle_cmd_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	// --- mavlink log ---
	orb_advert_t _mavlink_log_pub{nullptr};
};
