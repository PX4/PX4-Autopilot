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
 * @file serialpassthrough.cpp
 *
 * Serial passthrough driven by MAVLink SERIAL_CONTROL messages.
 *
 * Data flow:
 *   SERIAL_CONTROL -> MavlinkReceiver::handle_message_serial_control()
 *       -> SerialPassthrough::pushFromMavlink()
 *       -> [mutex-protected rx buffer]
 *       -> SerialPassthrough::run() drains to UART
 *
 *   UART -> SerialPassthrough::run() reads to [mutex-protected tx buffer]
 *       -> Mavlink::handleSerialPassthroughOutput() drains
 *       -> SERIAL_CONTROL (FLAG_REPLY)
 */

#include <px4_platform_common/px4_config.h>

#include "serialpassthrough.hpp"

#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>

#include <string.h>
#include <errno.h>
#include <unistd.h>

using device::SerialConfig::ByteSize;
using device::SerialConfig::Parity;
using device::SerialConfig::StopBits;
using device::SerialConfig::FlowControl;

static constexpr int TASK_STACK_SIZE = PX4_STACK_ADJUSTED(1224 * 2);

// ---------------------------------------------------------------------------
// Static member definitions
// ---------------------------------------------------------------------------

SerialPassthrough *SerialPassthrough::_instances[SP_MAX_INSTANCES] {};
pthread_mutex_t    SerialPassthrough::_instances_mutex = PTHREAD_MUTEX_INITIALIZER;

SerialPassthrough::SerialPassthrough(const char *device_path, unsigned baudrate,
				     bool swap_rxtx, bool single_wire, int esc_channel) :
	_serial_port(device_path, baudrate,
		     ByteSize::EightBits,
		     Parity::None,
		     StopBits::One,
		     FlowControl::Disabled),
	_baudrate(baudrate),
	_swap_rxtx(swap_rxtx),
	_single_wire(single_wire),
	_esc_channel(esc_channel)
{
	strncpy(_dev_path, device_path, sizeof(_dev_path) - 1);
	_dev_path[sizeof(_dev_path) - 1] = '\0';

	pthread_mutex_init(&_rx_mutex, nullptr);
	pthread_mutex_init(&_tx_mutex, nullptr);
	pthread_mutex_init(&_meta_mutex, nullptr);
}

SerialPassthrough::~SerialPassthrough()
{
	if (_serial_port.isOpen()) {
		_serial_port.close();
	}

	pthread_mutex_destroy(&_rx_mutex);
	pthread_mutex_destroy(&_tx_mutex);
	pthread_mutex_destroy(&_meta_mutex);
}

void SerialPassthrough::run()
{
	px4_prctl(PR_SET_NAME, "serialpassthrough", px4_getpid());

	if (_esc_channel >= 0) {
#ifdef CONFIG_SERIALPASSTHROUGH_BITBANG

		// --- Bitbang UART mode (motor signal pin) ---
		if (bitbang_uart_init(_baudrate, true /* single_wire */) < 0) {
			PX4_ERR("Failed to init bitbang_uart at %u baud", _baudrate);
			return;
		}

		PX4_INFO("serialpassthrough running on ESC channel %d via bitbang at %u baud",
			 _esc_channel, _baudrate);

		while (!should_exit()) {
			// MAVLink -> ESC
			pthread_mutex_lock(&_rx_mutex);

			if (_rx_len > 0) {
				uint8_t tmp[SP_BUF_SIZE];
				size_t to_write = _rx_len;
				memcpy(tmp, _rx_buf, to_write);
				_rx_len = 0;
				pthread_mutex_unlock(&_rx_mutex);

				bitbang_uart_write((uint8_t)_esc_channel, tmp, to_write);

			} else {
				pthread_mutex_unlock(&_rx_mutex);
			}

			// ESC -> MAVLink: try to read up to 5ms
			ssize_t nread = bitbang_uart_read((uint8_t)_esc_channel,
							  _ser_read_buf, sizeof(_ser_read_buf),
							  5000 /* us */);

			if (nread > 0) {
				pthread_mutex_lock(&_tx_mutex);
				size_t space   = SP_BUF_SIZE - _tx_len;
				size_t to_copy = ((size_t)nread < space) ? (size_t)nread : space;
				memcpy(_tx_buf + _tx_len, _ser_read_buf, to_copy);
				_tx_len += to_copy;
				pthread_mutex_unlock(&_tx_mutex);
			}
		}

		bitbang_uart_deinit();
#else
		PX4_ERR("Bitbang UART not supported on this platform");
#endif // CONFIG_SERIALPASSTHROUGH_BITBANG
		return;
	}

	if (!_serial_port.isOpen()) {
		if (!_serial_port.open()) {
			PX4_ERR("Failed to open serial port %s", _dev_path);
			return;
		}
	}

	if (_swap_rxtx) {
		_serial_port.setSwapRxTxMode();
	}

	if (_single_wire) {
		_serial_port.setSingleWireMode();
	}

	PX4_INFO("serialpassthrough running on %s at %u baud", _dev_path, _baudrate);

	while (!should_exit()) {

		// --- MAVLink -> UART ---
		pthread_mutex_lock(&_rx_mutex);

		if (_rx_len > 0) {
			uint8_t tmp[SP_BUF_SIZE];
			size_t to_write = _rx_len;
			memcpy(tmp, _rx_buf, to_write);
			_rx_len = 0;
			pthread_mutex_unlock(&_rx_mutex);

			_serial_port.write(tmp, to_write);

		} else {
			pthread_mutex_unlock(&_rx_mutex);
		}

		// --- UART -> MAVLink ---
		// Wait up to 5ms for the first byte, then drain the rest with 1ms inter-byte timeout.
		// Keep timeout short so data reaches _tx_buf quickly for the MAVLink loop to send.
		ssize_t nread = _serial_port.readAtLeast(_ser_read_buf, sizeof(_ser_read_buf), 1, 5);

		if (nread > 0) {
			// Keep draining as long as more bytes arrive within 1ms
			while (nread < (ssize_t)sizeof(_ser_read_buf)) {
				ssize_t n = _serial_port.readAtLeast(_ser_read_buf + nread,
								     sizeof(_ser_read_buf) - nread, 1, 1);

				if (n <= 0) { break; }

				nread += n;
			}

			pthread_mutex_lock(&_tx_mutex);
			size_t space   = SP_BUF_SIZE - _tx_len;
			size_t to_copy = ((size_t)nread < space) ? (size_t)nread : space;

			if (to_copy < (size_t)nread) {
				PX4_WARN("serialpassthrough: tx buffer overflow, dropping %zd bytes",
					 nread - (ssize_t)to_copy);
			}

			memcpy(_tx_buf + _tx_len, _ser_read_buf, to_copy);
			_tx_len += to_copy;
			pthread_mutex_unlock(&_tx_mutex);
		}
	}

	if (_serial_port.isOpen()) {
		_serial_port.close();
	}
}

int SerialPassthrough::run_trampoline(int argc, char *argv[])
{
	// Parse -i <device_id> before instantiate() to get the registry key.
	// This flag is injected internally by startForDevice() and is not user-facing.
	uint8_t device_id = 255;
	{
		int myoptind = 1;
		int ch;
		const char *myoptarg = nullptr;

		while ((ch = px4_getopt(argc, argv, "b:d:e:i:sx", &myoptind, &myoptarg)) != EOF) {
			if (ch == 'i') { device_id = (uint8_t)strtoul(myoptarg, nullptr, 0); }
		}
	}

	SerialPassthrough *instance = instantiate(argc, argv);

	if (!instance) {
		return -1;
	}

	if (!instance->register_instance(device_id)) {
		PX4_ERR("serialpassthrough: max instances (%d) already running", SP_MAX_INSTANCES);
		delete instance;
		return -1;
	}

	instance->run();

	instance->unregister_instance();
	delete instance;
	return 0;
}

void SerialPassthrough::pushFromMavlink(const uint8_t *data, size_t len,
					uint8_t sysid, uint8_t compid, uint8_t device,
					uint8_t channel)
{
	// Store reply metadata
	pthread_mutex_lock(&_meta_mutex);
	_target_sysid  = sysid;
	_target_compid = compid;
	_device_id     = device;
	_tx_channel    = channel;
	pthread_mutex_unlock(&_meta_mutex);

	if (len == 0) {
		return;
	}

	pthread_mutex_lock(&_rx_mutex);
	size_t space   = SP_BUF_SIZE - _rx_len;
	size_t to_copy = (len < space) ? len : space;

	if (to_copy < len) {
		PX4_WARN("serialpassthrough: rx buffer overflow, dropping %zu bytes", len - to_copy);
	}

	memcpy(_rx_buf + _rx_len, data, to_copy);
	_rx_len += to_copy;
	pthread_mutex_unlock(&_rx_mutex);
}

size_t SerialPassthrough::popToMavlink(uint8_t *buf, size_t max_len,
				       uint8_t *out_sysid, uint8_t *out_compid, uint8_t *out_device,
				       uint8_t channel)
{
	pthread_mutex_lock(&_meta_mutex);
	uint8_t tx_channel = _tx_channel;
	*out_sysid  = _target_sysid;
	*out_compid = _target_compid;
	*out_device = _device_id;
	pthread_mutex_unlock(&_meta_mutex);

	// Only the channel that received the request should send the reply
	if (channel != tx_channel) {
		PX4_DEBUG("popToMavlink: channel %u != tx_channel %u, skipping", channel, tx_channel);
		return 0;
	}

	pthread_mutex_lock(&_tx_mutex);
	size_t to_copy = (_tx_len < max_len) ? _tx_len : max_len;

	if (to_copy > 0) {

		memcpy(buf, _tx_buf, to_copy);
		_tx_len -= to_copy;

		if (_tx_len > 0) {
			memmove(_tx_buf, _tx_buf + to_copy, _tx_len);
		}
	}

	pthread_mutex_unlock(&_tx_mutex);
	return to_copy;
}

int SerialPassthrough::startForDevice(uint8_t device_id, uint32_t baudrate)
{
	// Map device_id to device path and options
	const char *dev = nullptr;
	bool single_wire = false;
	int  esc_channel = -1;

	switch (device_id) {
#ifdef CONFIG_BOARD_SERIAL_TEL1

	case 0: dev = CONFIG_BOARD_SERIAL_TEL1; break;
#endif
#ifdef CONFIG_BOARD_SERIAL_TEL2

	case 1: dev = CONFIG_BOARD_SERIAL_TEL2; break;
#endif
#ifdef CONFIG_BOARD_SERIAL_GPS1

	case 2: dev = CONFIG_BOARD_SERIAL_GPS1; break;
#endif
#ifdef CONFIG_BOARD_SERIAL_GPS2

	case 3: dev = CONFIG_BOARD_SERIAL_GPS2; break;
#endif
#ifdef CONFIG_BOARD_SERIAL_TEL3

	case 4: dev = CONFIG_BOARD_SERIAL_TEL3; break;
#endif
#ifdef CONFIG_BOARD_SERIAL_TEL4

	case 5: dev = CONFIG_BOARD_SERIAL_TEL4; break;
#endif
#ifdef CONFIG_SERIALPASSTHROUGH_BITBANG

	case 20: esc_channel = 0; break;

	case 21: esc_channel = 1; break;

	case 22: esc_channel = 2; break;

	case 23: esc_channel = 3; break;

	case 24: esc_channel = 4; break;

	case 25: esc_channel = 5; break;

	case 26: esc_channel = 6; break;

	case 27: esc_channel = 7; break;
#endif

	default:
		PX4_WARN("serialpassthrough: device_id %u not supported on this board", device_id);
		return -1;
	}

	// Check if already running on the same device_id
	SerialPassthrough *sp = get_instance_for_device(device_id);

	if (sp) {
		PX4_DEBUG("serialpassthrough: already running for device_id %u", device_id);
		return 0;
	}

#ifdef CONFIG_SERIALPASSTHROUGH_BITBANG

	if (esc_channel >= 0) {
		// The bitbang UART uses a single shared hardware timer and a single
		// global state struct — only one ESC channel can be active at a time.
		// Stop any existing bitbang instance and wait for it to exit before
		// spawning the new task, otherwise both tasks race on the shared HW
		// and corrupt each other's RX buffers via bitbang_uart_switch_channel().
		pthread_mutex_lock(&_instances_mutex);

		for (int i = 0; i < SP_MAX_INSTANCES; i++) {
			if (_instances[i] && _instances[i]->_esc_channel >= 0) {
				_instances[i]->request_stop();
			}
		}

		pthread_mutex_unlock(&_instances_mutex);

		// Wait up to 100 ms (20 × 5 ms) for the old task to exit and
		// unregister itself.  The bitbang run-loop polls every 5 ms, so
		// it should exit almost immediately.
		for (int i = 0; i < 20; i++) {
			bool any_bitbang = false;
			pthread_mutex_lock(&_instances_mutex);

			for (int j = 0; j < SP_MAX_INSTANCES; j++) {
				if (_instances[j] && _instances[j]->_esc_channel >= 0) {
					any_bitbang = true;
					break;
				}
			}

			pthread_mutex_unlock(&_instances_mutex);

			if (!any_bitbang) { break; }

			px4_usleep(5000); // 5 ms
		}
	}

#endif // CONFIG_SERIALPASSTHROUGH_BITBANG

	if (esc_channel >= 0) {
		PX4_INFO("serialpassthrough: starting on ESC bitbang channel %d at %lu baud",
			 esc_channel, (unsigned long)baudrate);

	} else {
		PX4_INFO("serialpassthrough: starting on %s at %lu baud",
			 dev, (unsigned long)baudrate);
	}

	// Build argv for task_spawn: ["start", "-d", dev, "-b", baud_str] or ["start", "-e", "0", "-b", baud_str]
	char baud_str[12];
	snprintf(baud_str, sizeof(baud_str), "%lu", (unsigned long)baudrate);

	char esc_str[4];
	snprintf(esc_str, sizeof(esc_str), "%d", esc_channel);

	char device_id_str[4];
	snprintf(device_id_str, sizeof(device_id_str), "%u", (unsigned)device_id);

	const char *argv_buf[12];
	int argc = 0;

	argv_buf[argc++] = "start";
	argv_buf[argc++] = "-i";
	argv_buf[argc++] = device_id_str;

	if (esc_channel >= 0) {
		argv_buf[argc++] = "-e";
		argv_buf[argc++] = esc_str;

	} else {
		argv_buf[argc++] = "-d";
		argv_buf[argc++] = dev;

		if (single_wire) { argv_buf[argc++] = "-s"; }
	}

	argv_buf[argc++] = "-b";
	argv_buf[argc++] = baud_str;
	argv_buf[argc] = nullptr;

	return task_spawn(argc, (char **)argv_buf);
}

// ---------------------------------------------------------------------------
// Instance registry
// ---------------------------------------------------------------------------

bool SerialPassthrough::register_instance(uint8_t device_id)
{
	pthread_mutex_lock(&_instances_mutex);

	for (int i = 0; i < SP_MAX_INSTANCES; i++) {
		if (_instances[i] == nullptr) {
			_instances[i] = this;
			_registered_device_id = device_id;
			pthread_mutex_unlock(&_instances_mutex);
			return true;
		}
	}

	pthread_mutex_unlock(&_instances_mutex);
	return false;
}

void SerialPassthrough::unregister_instance()
{
	pthread_mutex_lock(&_instances_mutex);

	for (int i = 0; i < SP_MAX_INSTANCES; i++) {
		if (_instances[i] == this) {
			_instances[i] = nullptr;
			break;
		}
	}

	pthread_mutex_unlock(&_instances_mutex);
}

SerialPassthrough *SerialPassthrough::get_instance_by_index(int index)
{
	if (index < 0 || index >= SP_MAX_INSTANCES) { return nullptr; }

	pthread_mutex_lock(&_instances_mutex);
	SerialPassthrough *inst = _instances[index];
	pthread_mutex_unlock(&_instances_mutex);
	return inst;
}

SerialPassthrough *SerialPassthrough::get_instance_for_device(uint8_t device_id)
{
	pthread_mutex_lock(&_instances_mutex);

	for (int i = 0; i < SP_MAX_INSTANCES; i++) {
		if (_instances[i] && _instances[i]->_registered_device_id == device_id) {
			SerialPassthrough *inst = _instances[i];
			pthread_mutex_unlock(&_instances_mutex);
			return inst;
		}
	}

	pthread_mutex_unlock(&_instances_mutex);
	return nullptr;
}

void SerialPassthrough::stop_for_device(uint8_t device_id)
{
	pthread_mutex_lock(&_instances_mutex);

	for (int i = 0; i < SP_MAX_INSTANCES; i++) {
		if (_instances[i] && _instances[i]->_registered_device_id == device_id) {
			_instances[i]->request_stop();
			pthread_mutex_unlock(&_instances_mutex);
			return;
		}
	}

	pthread_mutex_unlock(&_instances_mutex);
}

void SerialPassthrough::stop_all()
{
	pthread_mutex_lock(&_instances_mutex);

	for (int i = 0; i < SP_MAX_INSTANCES; i++) {
		if (_instances[i]) {
			_instances[i]->request_stop();
		}
	}

	pthread_mutex_unlock(&_instances_mutex);
}

int SerialPassthrough::task_spawn(int argc, char *argv[])
{
	int task_id = px4_task_spawn_cmd("serialpassthrough",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_DEFAULT,
					 TASK_STACK_SIZE,
					 run_trampoline,
					 (char *const *)argv);

	if (task_id < 0) {
		return -errno;
	}

	return 0;
}

SerialPassthrough *SerialPassthrough::instantiate(int argc, char *argv[])
{
	const char *device    = nullptr;
	unsigned    baudrate  = 115200;
	bool        swap_rxtx = false;
	bool        single_wire = false;
	int         esc_channel = -1;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "b:d:e:i:sx", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			baudrate = strtoul(myoptarg, nullptr, 0);
			break;

		case 'd':
			device = myoptarg;
			break;

		case 'e':
			esc_channel = (int)strtol(myoptarg, nullptr, 0);
			break;

		case 's':
			single_wire = true;
			break;

		case 'x':
			swap_rxtx = true;
			break;

		case 'i':
			// internal registry key injected by startForDevice(), ignore here
			break;

		default:
			print_usage("unrecognized flag");
			return nullptr;
		}
	}

	if (esc_channel >= 0) {
		// Bitbang mode: no device path needed
		return new SerialPassthrough("", baudrate, false, false, esc_channel);
	}

	if (!device || access(device, R_OK | W_OK) != 0) {
		PX4_ERR("Invalid device (-d): %s", device ? device : "(not set)");
		return nullptr;
	}

	return new SerialPassthrough(device, baudrate, swap_rxtx, single_wire);
}

int SerialPassthrough::print_status_all()
{
	pthread_mutex_lock(&_instances_mutex);
	int count = 0;

	for (int i = 0; i < SP_MAX_INSTANCES; i++) {
		if (_instances[i]) {
			SerialPassthrough *inst = _instances[i];

			pthread_mutex_lock(&inst->_rx_mutex);
			size_t rx = inst->_rx_len;
			pthread_mutex_unlock(&inst->_rx_mutex);

			pthread_mutex_lock(&inst->_tx_mutex);
			size_t tx = inst->_tx_len;
			pthread_mutex_unlock(&inst->_tx_mutex);

			PX4_INFO("[%d] device_id=%u  dev=%s  baud=%u  esc_ch=%d  swap=%d  single=%d",
				 i, inst->_registered_device_id, inst->_dev_path, inst->_baudrate,
				 inst->_esc_channel, (int)inst->_swap_rxtx, (int)inst->_single_wire);
			PX4_INFO("    pending rx(mavlink->uart)=%zu  tx(uart->mavlink)=%zu", rx, tx);
			count++;
		}
	}

	pthread_mutex_unlock(&_instances_mutex);

	if (count == 0) {
		PX4_INFO("serialpassthrough: no instances running");
	}

	return 0;
}

int SerialPassthrough::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PX4_INFO(
		"Usage:\n"
		"  serialpassthrough start -d <dev> [-b <baud>] [-x] [-s]\n"
		"  serialpassthrough stop\n"
		"  serialpassthrough status\n"
		"\n"
		"Options:\n"
		"  -d <dev>   Serial device path\n"
		"  -b <baud>  Baudrate (default 115200)\n"
		"  -x         Swap RX/TX pins\n"
		"  -s         Single-wire (half-duplex) mode\n"
		"  -e <ch>    ESC bitbang channel (0-3), instead of -d\n"
		"\n"
		"Up to %d instances can run simultaneously.",
		SP_MAX_INSTANCES
	);
	return 0;
}

extern "C" __EXPORT int serialpassthrough_main(int argc, char *argv[])
{
	if (argc < 2) {
		return SerialPassthrough::print_usage(nullptr);
	}

	if (strcmp(argv[1], "start") == 0) {
		return SerialPassthrough::task_spawn(argc - 1, argv + 1);
	}

	if (strcmp(argv[1], "stop") == 0) {
		SerialPassthrough::stop_all();
		return 0;
	}

	if (strcmp(argv[1], "status") == 0) {
		return SerialPassthrough::print_status_all();
	}

	return SerialPassthrough::print_usage("unknown command");
}
