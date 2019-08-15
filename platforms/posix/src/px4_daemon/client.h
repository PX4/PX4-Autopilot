/****************************************************************************
 *
 *   Copyright (C) 2016, 2018 PX4 Development Team. All rights reserved.
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
 * @file client.h
 *
 * The client can connect and write a command to the socket that is supplied by
 * the server. It will then close its half of the connection, and read back the
 * stdout stream of the process that it started, followed by its return value.
 *
 * It the client dies, the connection gets closed automatically and the corresponding
 * thread in the server gets cancelled.
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Beat Küng <beat-kueng@gmx.net>
 * @author Mara Bos <m-ou.se@m-ou.se>
 */
#pragma once

#include <stdint.h>

#include "sock_protocol.h"

namespace px4_daemon
{


class Client
{
public:
	Client(int instance_id = 0);

	~Client();

	Client(Client &&other) : _fd(other._fd), _instance_id(other._instance_id)
	{
		// Steal the fd from the moved-from client.
		other._fd = -1;
	}

	/**
	 * Process the supplied command line arguments and send them to server.
	 *
	 * @param argc: number of arguments
	 * @param argv: argument values
	 * @return 0 on success
	 */
	int process_args(const int argc, const char **argv);

private:
	int _send_cmds(const int argc, const char **argv);
	int _listen();

	int _fd;
	int _instance_id; ///< instance ID for running multiple instances of the px4 server
};

} // namespace px4_daemon

