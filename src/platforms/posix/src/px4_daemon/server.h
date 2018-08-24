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
 * @file server.h
 *
 * The server (also called daemon) opens a pipe for clients to write to.
 *
 * Once a client connects it will send a command as well as a unique ID.
 * The server will return the stdout of the executing command, as well as the return
 * value to the client on a client specific pipe.
 * The client specific pipe are idenified by the unique ID of the client.
 *
 * There should only every be one server running, therefore the static instance.
 * The Singleton implementation is not complete, but it should be obvious not
 * to instantiate multiple servers.
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Beat Küng <beat-kueng@gmx.net>
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <map>

#include "pipe_protocol.h"


namespace px4_daemon
{

class Server
{
public:
	Server(int instance_id = 0);
	~Server();

	/**
	 * Start the server. This will spawn a thread with a
	 * while loop waiting for clients sending commands.
	 *
	 * @return 0 if started successfully
	 */
	int start();


	struct CmdThreadSpecificData {
		int pipe_fd; // pipe fd to send data to descriptor
		bool is_atty; // whether file descriptor refers to a terminal
		client_recv_packet_s packet;
	};

	static bool is_running()
	{
		return _instance != nullptr;
	}

	static pthread_key_t get_pthread_key()
	{
		return _instance->_key;
	}
private:
	static void *_server_main_trampoline(void *arg);
	void _server_main(void *arg);

	void _parse_client_send_packet(const client_send_packet_s &packet);
	void _execute_cmd_packet(const client_send_packet_s &packet);
	void _kill_cmd_packet(const client_send_packet_s &packet);
	void _cleanup_thread(const uint64_t client_uuid);

	/**
	 * Like _cleanup_thread(), but does not take the mutex
	 */
	void __cleanup_thread(const uint64_t client_uuid);

	void _lock()
	{
		pthread_mutex_lock(&_mutex);
	}

	void _unlock()
	{
		pthread_mutex_unlock(&_mutex);
	}


	static void _send_retval(const int pipe_fd, const int retval, const uint64_t client_uuid);

	struct RunCmdArgs {
		char cmd[sizeof(client_send_packet_s::payload.execute_msg.cmd)];
		uint64_t client_uuid;
		bool is_atty;
		int pipe_fd;
	};

	static void *_run_cmd(void *arg);

	pthread_t _server_main_pthread;

	std::map<pthread_t, int> _pthread_to_pipe_fd;
	std::map<uint64_t, pthread_t> _client_uuid_to_pthread;
	pthread_mutex_t _mutex; ///< protects access to _pthread_to_pipe_fd and _client_uuid_to_pthread

	pthread_key_t _key;

	int _instance_id; ///< instance ID for running multiple instances of the px4 server


	static void _pthread_key_destructor(void *arg);

	static Server *_instance;
};


} // namespace px4_daemon

