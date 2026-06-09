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

/**
 * @file serialpassthrough.hpp
 *
 * Serial passthrough driven by MAVLink SERIAL_CONTROL messages.
 *
 * Exposes a thread-safe push/pop interface so MavlinkReceiver can push
 * incoming payload bytes to the UART, and the Mavlink main loop can drain
 * UART-received bytes back as SERIAL_CONTROL replies.
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/Serial.hpp>
#ifdef CONFIG_SERIALPASSTHROUGH_BITBANG
#include <drivers/drv_bitbang_uart.h>
#endif
#include <stdint.h>
#include <stddef.h>
#include <pthread.h>

static constexpr size_t SP_BUF_SIZE      = 1024;
static constexpr int    SP_MAX_INSTANCES = 8;

class SerialPassthrough
{
public:
	SerialPassthrough(const char *device_path, unsigned baudrate, bool swap_rxtx, bool single_wire, int esc_channel = -1);
	~SerialPassthrough();

	static int task_spawn(int argc, char *argv[]);
	static int run_trampoline(int argc, char *argv[]);
	static SerialPassthrough *instantiate(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	void run();

	/**
	 * Print status of all running instances.
	 */
	static int print_status_all();

	/**
	 * Get instance by slot index (0..SP_MAX_INSTANCES-1). Returns nullptr if slot empty.
	 * Used by Mavlink main loop to iterate all instances.
	 */
	static SerialPassthrough *get_instance_by_index(int index);

	/**
	 * Get the running instance for a specific device_id. Returns nullptr if not running.
	 */
	static SerialPassthrough *get_instance_for_device(uint8_t device_id);

	/**
	 * Start an instance for a given device ID (from SERIAL_CONTROL device field).
	 * Maps device IDs to board-specific UART paths using CONFIG_BOARD_SERIAL_*.
	 * If already running on the same device, does nothing.
	 * Device IDs: 1=TEL2, 2=GPS1, 3=GPS2, 91-94=ESC bitbang ch0-3
	 * Returns 0 on success, -1 if device ID is not supported on this board.
	 */
	static int startForDevice(uint8_t device_id, uint32_t baudrate);

	/**
	 * Stop the instance for a specific device_id.
	 */
	static void stop_for_device(uint8_t device_id);

	/**
	 * Stop all running instances.
	 */
	static void stop_all();

	/**
	 * Push data received via MAVLink SERIAL_CONTROL to the UART.
	 * Thread-safe; called from MavlinkReceiver thread.
	 * channel: the MAVLink channel index (0-based) this arrived on.
	 */
	void pushFromMavlink(const uint8_t *data, size_t len,
			     uint8_t sysid, uint8_t compid, uint8_t device,
			     uint8_t channel);

	/**
	 * Pop up to max_len bytes of UART-received data for sending back as
	 * SERIAL_CONTROL reply. Thread-safe; called from Mavlink main loop.
	 * Only returns data if channel matches the channel that sent the request.
	 */
	size_t popToMavlink(uint8_t *buf, size_t max_len,
			    uint8_t *out_sysid, uint8_t *out_compid, uint8_t *out_device,
			    uint8_t channel);

	bool should_exit() const { return _should_exit.load(); }
	void request_stop()      { _should_exit.store(true); }

private:
	// --- Static instance registry ---
	static SerialPassthrough *_instances[SP_MAX_INSTANCES];
	static pthread_mutex_t    _instances_mutex;

	bool register_instance(uint8_t device_id);
	void unregister_instance();

	device::Serial _serial_port;
	char _dev_path[20] {};
	unsigned _baudrate{115200};
	bool _swap_rxtx{false};
	bool _single_wire{false};
	int _esc_channel{-1}; // if >= 0, use bitbang_uart on this motor channel instead of _serial_port
	uint8_t _registered_device_id{255};

	px4::atomic_bool _should_exit{false};

	// MAVLink → UART ring buffer (written by pushFromMavlink, drained by run())
	uint8_t _rx_buf[SP_BUF_SIZE] {};
	size_t _rx_len{0};
	pthread_mutex_t _rx_mutex;

	// UART → MAVLink ring buffer (written by run(), drained by popToMavlink)
	uint8_t _tx_buf[SP_BUF_SIZE] {};
	size_t _tx_len{0};
	pthread_mutex_t _tx_mutex;

	// Reply metadata (sysid/compid/device/channel of last received message)
	uint8_t _target_sysid{0};
	uint8_t _target_compid{0};
	uint8_t _device_id{0};
	uint8_t _tx_channel{255}; // MAVLink channel index to reply on (255 = not set)
	pthread_mutex_t _meta_mutex;

	uint8_t _ser_read_buf[256] {};
};
