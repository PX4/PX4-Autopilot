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

#define MODULE_NAME "mavlink_fuzz_tests"

#include <gtest/gtest.h>
#include <fuzztest/fuzztest.h>

#include <px4_platform_common/init.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>
#include "px4_daemon/pxh.h"
#include <common/mavlink.h>

#include <netinet/in.h>
#include <arpa/inet.h>

namespace px4
{
void init_once();
}

extern "C" const char *__asan_default_options() { return "detect_leaks=0"; }

class MavlinkFuzzTest
{
public:
	MavlinkFuzzTest()
	{
		// This object will be reused for each fuzzing iteration call
		px4::init_once();
		px4::init(0, nullptr, "px4");

		px4_daemon::Pxh pxh;
		pxh.process_line("param load", true);
		pxh.process_line("dataman start", true);
		pxh.process_line("load_mon start", true);
		pxh.process_line("battery_simulator start", true);
		pxh.process_line("tone_alarm start", true);
		pxh.process_line("rc_update start", true);
		pxh.process_line("sensors start", true);
		pxh.process_line("commander start", true);
		pxh.process_line("navigator start", true);
		pxh.process_line("ekf2 start", true);
		pxh.process_line("mc_att_control start", true);
		pxh.process_line("mc_pos_control start", true);
		pxh.process_line("land_detector start multicopter", true);
		pxh.process_line("logger start", true);
		pxh.process_line("mavlink start -x -o 14513 -u 14514 -r 4000000", true);
		pxh.process_line("mavlink boot_complete", true);

		_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);

		if (_socket_fd < 0) {
			PX4_ERR("socket error: %s", strerror(errno));
			return;
		}

		sockaddr_in addr {};

		addr.sin_family = AF_INET;

		inet_pton(AF_INET, "0.0.0.0", &(addr.sin_addr));

		addr.sin_port = htons(14513);

		if (bind(_socket_fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
			PX4_ERR("bind error: %s", strerror(errno));
			close(_socket_fd);
			return;
		}
	}
	~MavlinkFuzzTest()
	{
		close(_socket_fd);
	}

	void run(std::vector<uint8_t> data)
	{
		sendMavlink(data.data(), data.size());
	}
private:
	void sendMavlink(const uint8_t *data, const size_t size)
	{
		mavlink_message_t message {};
		uint8_t buffer[MAVLINK_MAX_PACKET_LEN] {};

		for (size_t i = 0; i < size; i += sizeof(message)) {

			const size_t copy_len = std::min(sizeof(message), size - i);
			memcpy(reinterpret_cast<void *>(&message), data + i, copy_len);

			// Set a correct checksum as it is hard for the fuzzer to pass the correct checksum
			mavlink_finalize_message_buffer(&message, message.sysid, message.compid, &_status, 0, message.len,
							mavlink_get_crc_extra(&message));

			const ssize_t buffer_len = mavlink_msg_to_send_buffer(buffer, &message);

			sockaddr_in dest_addr {};
			dest_addr.sin_family = AF_INET;

			inet_pton(AF_INET, "127.0.0.1", &dest_addr.sin_addr.s_addr);
			dest_addr.sin_port = htons(14514);

			if (sendto(_socket_fd, buffer, buffer_len, 0, reinterpret_cast<sockaddr *>(&dest_addr),
				   sizeof(dest_addr)) != buffer_len) {
				PX4_ERR("sendto error: %s", strerror(errno));
			}
		}
	}

	mavlink_status_t _status{};
	int _socket_fd{-1};
};

FUZZ_TEST_F(MavlinkFuzzTest, run);
