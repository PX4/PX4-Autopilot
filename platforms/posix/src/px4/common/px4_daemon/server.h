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
 * @file server.h
 *
 * The server (also called daemon) opens a socket for clients to connect to.
 *
 * Once a client connects it will send a command and close its side of the connection.
 * The server will return the stdout of the executing command, as well as the return
 * value to the client.
 *
 * There should only every be one server running, therefore the static instance.
 * The Singleton implementation is not complete, but it should be obvious not
 * to instantiate multiple servers.
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Beat Küng <beat-kueng@gmx.net>
 * @author Mara Bos <m-ou.se@m-ou.se>
 */
#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <map>

#include "sock_protocol.h"


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
		FILE *thread_stdout; // stdout of this thread
		bool is_atty; // whether file descriptor refers to a terminal
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
	void _server_main();

	void _lock()
	{
		pthread_mutex_lock(&_mutex);
	}

	void _unlock()
	{
		pthread_mutex_unlock(&_mutex);
	}

	static void *_handle_client(void *arg);
	static void _cleanup(int fd);

	pthread_t _server_main_pthread;

	std::map<int, pthread_t> _fd_to_thread;
	pthread_mutex_t _mutex; ///< Protects _fd_to_thread.

	pthread_key_t _key;

	int _instance_id; ///< instance ID for running multiple instances of the px4 server

	int _fd;

	static void _pthread_key_destructor(void *arg);

	static Server *_instance;
};


} // namespace px4_daemon
