/****************************************************************************
 *
 *   Copyright (C) 2016 PX4 Development Team. All rights reserved.
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
 * The client can write a command to the pipe that is supplied by the server.
 * It will then open another pipe with its own UUID encoded and listen for
 * stdout of the process that it started and the return value.
 *
 * It the client receives a signal (e.g. Ctrl+C) it will catch it and send it
 * as a message to the server in order to terminate the thread.
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Beat Küng <beat-kueng@gmx.net>
 */
#pragma once

#include <stdint.h>

#include "pipe_protocol.h"

namespace px4_daemon
{


class Client
{
public:
	Client();
	~Client();

	/*
	 * Initialize the unique ID of the client.
	 *
	 * @return 0 on success.
	 */
	int generate_uuid();

	/*
	 * Make sure to catch signals in order to forward them to the server.
	 */
	void register_sig_handler();

	/*
	 * Process the supplied command line arguments and send them to server.
	 *
	 * @param argc: number of arguments
	 * @param argv: argument values
	 * @return 0 on success
	 */
	int process_args(const int argc, const char **argv);

private:
	int _prepare_recv_pipe();
	int _send_cmds(const int argc, const char **argv);
	int _listen();

	int _parse_client_recv_packet(const client_recv_packet_s &packet, int &retval, bool &should_exit);

	int _retval_cmd_packet(const client_recv_packet_s &packet, int &retval);
	int _stdout_msg_packet(const client_recv_packet_s &packet);


	static void _static_sig_handler(int sig_num);
	void _sig_handler(int sig_num);

	uint64_t _uuid;
	int _client_send_pipe_fd;
	char _recv_pipe_path[RECV_PIPE_PATH_LEN];
};

} // namespace px4_daemon

