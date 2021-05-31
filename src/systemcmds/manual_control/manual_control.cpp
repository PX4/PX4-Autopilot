/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file manual_control.cpp
 * Command line manual control
 *
 */

#include <px4_platform_common/px4_config.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <poll.h>

#include <px4_platform/cpuload.h>
#include <px4_platform_common/printload.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/module.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/manual_control_setpoint.h>

using namespace time_literals;

static void print_usage()
{
	PRINT_MODULE_DESCRIPTION("Keyboard manual control");

	PRINT_MODULE_USAGE_NAME_SIMPLE("manual_control", "command");
}

static void draw_output(const manual_control_setpoint_s &manual_control_setpoint)
{
	// clear screen
	fprintf(stdout, "\033[2J");

	// left stick, right stick calculate coordinates
	//  R1 C1    R2 C2
	static constexpr int HEIGHT = 6;
	static constexpr int WIDTH = HEIGHT * 3; // approximate aspect ratio

	int r1 = HEIGHT - floorf((manual_control_setpoint.z + 1.f) / 2.f * HEIGHT);
	int c1 = floorf((manual_control_setpoint.r + 1.f) / 2.f * WIDTH);

	int r2 = HEIGHT - floorf((manual_control_setpoint.x + 1.f) / 2.f * HEIGHT);
	int c2 = floorf((manual_control_setpoint.y + 1.f) / 2.f * WIDTH);

	// start
	fprintf(stdout, "Left: W-A-S-D             Right: I-J-K-L or Arrow keys\n");
	fprintf(stdout, "|-------------------|     |-------------------|\n");

	for (int row = 0; row < (HEIGHT + 1); row++) {
		static constexpr int left_offset = 1;
		static constexpr int right_offset = 27;
		char buffer[] {"|                   |     |                   |\n"};

		if (row == r1) {
			buffer[left_offset + c1] = 'X';
		}

		if (row == r2) {
			buffer[right_offset + c2] = 'X';
		}

		fprintf(stdout, buffer);
	}

	fprintf(stdout, "|-------------------|     |-------------------|\n");
	fprintf(stdout, "  Yaw:      %4.0f%%             Roll:  %4.0f%%\n",
		(double)manual_control_setpoint.r * 100,
		(double)manual_control_setpoint.y * 100);
	fprintf(stdout, "  Throttle: %4.0f%%             Pitch: %4.0f%%\n",
		(double)manual_control_setpoint.z * 100,
		(double)manual_control_setpoint.x * 100);
	// end
}

extern "C" __EXPORT int manual_control_main(int argc, char *argv[])
{
	if (argc > 1) {
		// do something
		print_usage();
		return 0;
	}

	PX4_INFO("Left Stick (Throttle/Yaw) W-A-S-D, Right Stick (Pitch/Roll) Arrow Keys or I-J-K-L");
	PX4_INFO("Press a valid key to begin...");

	hrt_abstime last_stdin = hrt_absolute_time();

	uORB::PublicationMulti<manual_control_setpoint_s> manual_control_setpoint_pub{ORB_ID(manual_control_setpoint)};
	manual_control_setpoint_s manual_control_setpoint{};

	while (true) {
		if (hrt_elapsed_time(&last_stdin) > 30_s) {
			PX4_INFO("no input, exiting");
			return 0;
		}

		pollfd fds{};
		fds.fd = STDIN_FILENO;
		fds.events = POLLIN;
		int ret = poll(&fds, 1, 10);

		static constexpr float key_increment = 1.f / 8.f;
		static constexpr float key_auto_decrement = 0.05f;

		if (ret == 0) {
			// timeout

			if (hrt_elapsed_time(&last_stdin) > 500_ms) {
				// pitch self center
				if (manual_control_setpoint.x > key_auto_decrement) {
					manual_control_setpoint.x -= key_auto_decrement;

				} else if (manual_control_setpoint.x < -key_auto_decrement) {
					manual_control_setpoint.x += key_auto_decrement;

				} else {
					manual_control_setpoint.x = 0.f;
				}

				// roll self center
				if (manual_control_setpoint.y > key_auto_decrement) {
					manual_control_setpoint.y -= key_auto_decrement;

				} else if (manual_control_setpoint.y < -key_auto_decrement) {
					manual_control_setpoint.y += key_auto_decrement;

				} else {
					manual_control_setpoint.y = 0.f;
				}

				// yaw self center
				if (manual_control_setpoint.r > key_auto_decrement) {
					manual_control_setpoint.r -= key_auto_decrement;

				} else if (manual_control_setpoint.r < -key_auto_decrement) {
					manual_control_setpoint.r += key_auto_decrement;

				} else {
					manual_control_setpoint.r = 0.f;
				}
			}

			// update timestamp and publish
			manual_control_setpoint.timestamp = hrt_absolute_time();
			manual_control_setpoint_pub.publish(manual_control_setpoint);

			draw_output(manual_control_setpoint);

		} else if (ret > 0 && fds.revents == POLLIN) {
			int bytes_available = 0;

			while ((ioctl(STDIN_FILENO, FIONREAD, (unsigned long)&bytes_available) >= 0) && bytes_available > 0) {

				char c;

				if (read(STDIN_FILENO, &c, 1) <= 0) {
					return 1;
				}

				bytes_available--;

				// W-A-S-D for left stick
				// Arrow keys or I-J-K-L for right right
				switch (c) {
				case 'w': // +z
					manual_control_setpoint.z += key_increment;
					break;

				case 's': // -z
					manual_control_setpoint.z -= key_increment;
					break;

				case 'd': // +r
					manual_control_setpoint.r += key_increment;
					break;

				case 'a': // -r
					manual_control_setpoint.r -= key_increment;
					break;

				case 'i': // +x
					manual_control_setpoint.x = math::max(manual_control_setpoint.x + key_increment, 0.f);
					break;

				case 'k': // -x
					manual_control_setpoint.x = math::min(manual_control_setpoint.x - key_increment, 0.f);
					break;

				case 'j': // -y
					manual_control_setpoint.y = math::min(manual_control_setpoint.y - key_increment, 0.f);
					break;

				case 'l': // +y
					manual_control_setpoint.y = math::max(manual_control_setpoint.y + key_increment, 0.f);
					break;

				case '\x1b': { // escape sequence
						char seq[3] {};

						if (read(STDIN_FILENO, &seq[0], 1) != 1) {
							break;
						}

						if (read(STDIN_FILENO, &seq[1], 1) != 1) {
							break;
						}

						if (seq[0] == '[') {
							switch (seq[1]) {
							case 'A': // Up: +x
								manual_control_setpoint.x = math::max(manual_control_setpoint.x + key_increment, 0.f);
								break;

							case 'B': // Down: -x
								manual_control_setpoint.x = math::min(manual_control_setpoint.x - key_increment, 0.f);
								break;

							case 'C': // Right: +y
								manual_control_setpoint.y = math::max(manual_control_setpoint.y + key_increment, 0.f);
								break;

							case 'D': // Left: -y
								manual_control_setpoint.y = math::min(manual_control_setpoint.y - key_increment, 0.f);
								break;
							}
						}
					}
					break;

				case 0x03: // ctrl-c
				case 'c':
				case 'q':
					return 0;
				}

				last_stdin = hrt_absolute_time();

				manual_control_setpoint.x = math::constrain(manual_control_setpoint.x, -1.f, 1.f);
				manual_control_setpoint.y = math::constrain(manual_control_setpoint.y, -1.f, 1.f);
				manual_control_setpoint.z = math::constrain(manual_control_setpoint.z, -1.f, 1.f);
				manual_control_setpoint.r = math::constrain(manual_control_setpoint.r, -1.f, 1.f);

				manual_control_setpoint.timestamp = hrt_absolute_time();
				manual_control_setpoint_pub.publish(manual_control_setpoint);

				draw_output(manual_control_setpoint);
			}
		}
	}

	return 0;
}
