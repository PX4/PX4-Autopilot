/****************************************************************************
 *
 *   Copyright (C) 2018 PX4 Development Team. All rights reserved.
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
 * @file sock_protocol.cpp
 *
 * @author Mara Bos <m-ou.se@m-ou.se>
 */

#include "sock_protocol.h"

#include <cstdlib>

namespace px4_daemon
{

#ifdef __PX4_WINDOWS
uint16_t get_socket_port(int instance_id)
{
	// Keep the local daemon control socket away from the default SITL MAVLink
	// UDP ranges (for example 14580 + instance is used by px4-rc.mavlink).
	// Override by setting PX4_DAEMON_PORT if the default collides with another
	// app embedding PX4.
	const char *override_port = std::getenv("PX4_DAEMON_PORT");

	if (override_port) {
		return static_cast<uint16_t>(std::atoi(override_port) + instance_id);
	}

	return static_cast<uint16_t>(14680 + instance_id);
}
#else
std::string get_socket_path(int instance_id)
{
	// TODO: Use /var/run/px4/$instance/sock (or /var/run/user/$UID/... for non-root).
	return "/tmp/px4-sock-" + std::to_string(instance_id);
}
#endif

} // namespace px4_daemon
