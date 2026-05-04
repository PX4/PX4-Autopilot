/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file cubered_bridge_primary_serial.cpp
 *
 * Serial transport for the CubeRed primary bridge. Talks to the
 * secondary MCU over UART7 using device::Serial (NuttX lower-half
 * driver, DMA-backed via CONFIG_UART7_RXDMA/TXDMA=y).
 *
 * Originally derived from src/drivers/px4io/px4io_serial.cpp.
 */

#include "cubered_bridge_primary_driver.h"

#include <drivers/device/device.h>
#include <lib/perf/perf_counter.h>
#include <modules/px4iofirmware/protocol.h>
#include <px4_platform_common/Serial.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/sem.h>

#include <string.h>

namespace
{

class CuberedSerial : public device::Device
{
public:
	CuberedSerial();
	~CuberedSerial() override;

	int init() override;

	int read(unsigned address, void *data, unsigned count) override;
	int write(unsigned address, void *data, unsigned count) override;

private:
	static constexpr uint32_t TRANSACTION_TIMEOUT_MS = 10;
	static constexpr unsigned MAX_RETRIES = 3;

	int ensure_open();
	int exchange(size_t request_size, size_t expected_response_size);

public:
	void release();

private:
	device::Serial _uart{};
	IOPacket _io_buffer{};
	px4_sem_t _bus_semaphore;

	perf_counter_t _pc_txns{perf_alloc(PC_ELAPSED, MODULE_NAME": txns")};
	perf_counter_t _pc_retries{perf_alloc(PC_COUNT, MODULE_NAME": retries")};
	perf_counter_t _pc_timeouts{perf_alloc(PC_COUNT, MODULE_NAME": timeouts")};
	perf_counter_t _pc_crcerrs{perf_alloc(PC_COUNT, MODULE_NAME": crcerrs")};
	perf_counter_t _pc_protoerrs{perf_alloc(PC_COUNT, MODULE_NAME": protoerrs")};
};

CuberedSerial::CuberedSerial() :
	Device("cubered_bridge_primary")
{
	px4_sem_init(&_bus_semaphore, 0, 1);
}

CuberedSerial::~CuberedSerial()
{
	if (_uart.isOpen()) {
		_uart.close();
	}

	px4_sem_destroy(&_bus_semaphore);

	perf_free(_pc_txns);
	perf_free(_pc_retries);
	perf_free(_pc_timeouts);
	perf_free(_pc_crcerrs);
	perf_free(_pc_protoerrs);
}

int CuberedSerial::init()
{
	if (!_uart.setPort(CUBERED_BRIDGE_PRIMARY_DEVICE)) {
		PX4_ERR("setPort %s failed", CUBERED_BRIDGE_PRIMARY_DEVICE);
		return -1;
	}

	if (!_uart.setBaudrate(CUBERED_BRIDGE_PRIMARY_BITRATE)) {
		PX4_ERR("setBaudrate %u failed", CUBERED_BRIDGE_PRIMARY_BITRATE);
		return -1;
	}

	return ensure_open();
}

int CuberedSerial::ensure_open()
{
	if (_uart.isOpen()) {
		return 0;
	}

	if (!_uart.open()) {
		PX4_ERR("open %s failed", CUBERED_BRIDGE_PRIMARY_DEVICE);
		return -1;
	}

	// UART7 TX/RX are wired swapped between the primary and secondary MCUs.
	if (!_uart.setSwapRxTxMode()) {
		PX4_ERR("setSwapRxTxMode failed");
		_uart.close();
		return -1;
	}

	return 0;
}

void CuberedSerial::release()
{
	if (_uart.isOpen()) {
		_uart.close();
	}
}

int CuberedSerial::exchange(size_t request_size, size_t expected_response_size)
{
	perf_begin(_pc_txns);

	if (_uart.write(&_io_buffer, request_size) != (ssize_t)request_size) {
		perf_cancel(_pc_txns);
		return -EIO;
	}

	uint8_t *buf = reinterpret_cast<uint8_t *>(&_io_buffer);
	ssize_t got = _uart.readAtLeast(buf, sizeof(IOPacket),
					expected_response_size, TRANSACTION_TIMEOUT_MS);

	if (got < (ssize_t)expected_response_size) {
		perf_count(_pc_timeouts);
		perf_cancel(_pc_txns);
		return -ETIMEDOUT;
	}

	const uint8_t received_crc = _io_buffer.crc;
	_io_buffer.crc = 0;

	if (crc_packet(&_io_buffer) != received_crc) {
		perf_count(_pc_crcerrs);
		perf_cancel(_pc_txns);
		return -EIO;
	}

	perf_end(_pc_txns);
	return 0;
}

int CuberedSerial::write(unsigned address, void *data, unsigned count)
{
	const uint8_t page = address >> 8;
	const uint8_t offset = address & 0xff;
	const uint16_t *values = reinterpret_cast<const uint16_t *>(data);

	if (count > PKT_MAX_REGS) {
		return -EINVAL;
	}

	px4_sem_wait(&_bus_semaphore);

	int result = -EIO;

	if (ensure_open() != 0) {
		px4_sem_post(&_bus_semaphore);
		return result;
	}

	for (unsigned retries = 0; retries < MAX_RETRIES; retries++) {
		_io_buffer.count_code = count | PKT_CODE_WRITE;
		_io_buffer.page = page;
		_io_buffer.offset = offset;
		memcpy(&_io_buffer.regs[0], values, 2 * count);

		for (unsigned i = count; i < PKT_MAX_REGS; i++) {
			_io_buffer.regs[i] = 0x55aa;
		}

		_io_buffer.crc = 0;
		_io_buffer.crc = crc_packet(&_io_buffer);

		const size_t request_size = PKT_SIZE(_io_buffer);
		// Write ack response is just the 4-byte header.
		const size_t response_size = sizeof(IOPacket) - sizeof(_io_buffer.regs);

		result = exchange(request_size, response_size);

		if (result == 0) {
			if (PKT_CODE(_io_buffer) == PKT_CODE_ERROR) {
				// Secondary rejected the request — retrying won't help.
				result = -EINVAL;
				perf_count(_pc_protoerrs);
			}

			break;
		}

		perf_count(_pc_retries);
	}

	px4_sem_post(&_bus_semaphore);

	return (result == 0) ? (int)count : result;
}

int CuberedSerial::read(unsigned address, void *data, unsigned count)
{
	const uint8_t page = address >> 8;
	const uint8_t offset = address & 0xff;
	uint16_t *values = reinterpret_cast<uint16_t *>(data);

	if (count > PKT_MAX_REGS) {
		return -EINVAL;
	}

	px4_sem_wait(&_bus_semaphore);

	int result = -EIO;

	if (ensure_open() != 0) {
		px4_sem_post(&_bus_semaphore);
		return result;
	}

	for (unsigned retries = 0; retries < MAX_RETRIES; retries++) {
		memset(&_io_buffer, 0, sizeof(IOPacket));

		_io_buffer.count_code = count | PKT_CODE_READ;
		_io_buffer.page = page;
		_io_buffer.offset = offset;
		_io_buffer.crc = 0;
		_io_buffer.crc = crc_packet(&_io_buffer);

		// Read requests carry `count` reg slots of padding so that the
		// secondary's PKT_SIZE / CRC accounting matches what the response
		// will look like.
		const size_t request_size = PKT_SIZE(_io_buffer);
		const size_t response_size = request_size;

		result = exchange(request_size, response_size);

		if (result == 0) {
			if (PKT_CODE(_io_buffer) == PKT_CODE_ERROR) {
				result = -EINVAL;
				perf_count(_pc_protoerrs);

			} else if (PKT_COUNT(_io_buffer) != count) {
				result = -EIO;
				perf_count(_pc_protoerrs);

			} else {
				memcpy(values, &_io_buffer.regs[0], 2 * count);
			}

			break;
		}

		perf_count(_pc_retries);
	}

	px4_sem_post(&_bus_semaphore);

	return (result == 0) ? (int)count : result;
}

} // namespace

device::Device *cubered_bridge_primary_interface()
{
	return new CuberedSerial();
}

void cubered_bridge_primary_release(device::Device *interface)
{
	if (interface != nullptr) {
		static_cast<CuberedSerial *>(interface)->release();
	}
}
