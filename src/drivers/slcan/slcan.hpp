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

#pragma once

#include <px4_platform_common/module.h>

#include <cstdint>

class Slcan : public ModuleBase
{
public:
	static Descriptor desc;

	Slcan(const char *serial_dev, const char *can_iface);
	~Slcan() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	static int run_trampoline(int argc, char *argv[]);
	static Slcan *instantiate(int argc, char *argv[]);

	void run();
	int print_status() override;

private:
	int open_can();
	int open_serial();
	void close_can();
	void close_fds();
	void poll_loop();
	void apply_armed_state();

	void handle_serial();
	void handle_can();
	void handle_command(const char *cmd, int len);

	void ack();
	void nack();
	void send_ascii(const char *s);

	void send_can_frame(uint32_t can_id, const uint8_t *data, uint8_t len, bool ext, bool fd, bool brs);

	char _serial_dev[32] {};
	char _can_iface[16] {};

	int _serial_fd{-1};
	int _can_sock{-1};

	bool _open{true}; // bus is already up via UAVCAN; LAWICEL O/C still ACKed
	bool _armed{false};
	uint32_t _rx_count{0};
	uint32_t _tx_count{0};

	char _line[128] {};
	int _line_len{0};
};
