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
 * @file esc_4way.hpp
 *
 * ESC configuration server. Answers the MSP v1 subset the ESC configurators (am32.ca,
 * esc-configurator.com) use, switches to the 4way interface on MSP_SET_PASSTHROUGH and
 * runs each 4way device command as a BLHeli/AM32 bootloader exchange on the ESC signal pin.
 */

#pragma once

#include <px4_platform_common/module.h>

#include <string.h>

/** BLHeli/AM32 bootloader client on one ESC signal pin; the caller owns the ESC pin UART */
class EscBootloader
{
public:
	uint8_t channel{0};
	uint8_t info[4] {};	///< sigLo, sigHi, pinCode, interface mode (0: no bootloader connected)
	uint8_t buf[256 + 3];	///< init reply, or read data followed by its CRC and the ack

	bool connect(uint8_t ch, uint32_t timeout_us);
	bool read(uint8_t cmd, uint16_t addr, size_t n);
	bool write(uint8_t cmd, uint16_t addr, uint8_t *data, size_t n);	///< data needs two spare bytes after n
	bool erase(uint16_t addr);
	bool run();
	bool keep_alive();

private:
	bool send(uint8_t *frame, size_t len);
	int ack(uint32_t timeout_us);
	bool init();
	bool set_address(uint16_t addr);
};

class Esc4way : public ModuleBase
{
public:
	static Descriptor desc;

	Esc4way(int fd, const char *device) : _fd(fd) { strncpy(_device, device, sizeof(_device) - 1); }
	~Esc4way() override;

	static int task_spawn(int argc, char *argv[]);
	static int run_trampoline(int argc, char *argv[]);
	static Esc4way *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr);

	void run() override;

private:
	bool receive(uint8_t *buf, size_t len);
	void reopen();
	bool start_session();

	void handle_msp();
	void send_msp(uint8_t cmd, const uint8_t *payload, uint8_t size, uint8_t direction = '>');

	void handle_4way();
	void send_4way(uint8_t cmd, uint16_t addr, const uint8_t *params, size_t n, uint8_t ack);

	int _fd{-1};
	char _device[32] {};		///< serial device path, for a reopen

	bool _session{false};		///< first valid request handled, output drivers possibly stopped
	bool _uart{false};		///< ESC pin UART owned by this session
	bool _fourway{false};		///< 4way mode (after MSP_SET_PASSTHROUGH or a 4way request)
	bool _served{false};		///< at least one device command was run
	bool _reboot{false};		///< session over: reboot to restore the outputs

	EscBootloader _esc;		///< ESC of the last device command

	uint8_t _frame[264];		///< request frame; 4way: '/' cmd addrH addrL n params[256] crc16
	uint8_t _reply[264];		///< reply frame
};
