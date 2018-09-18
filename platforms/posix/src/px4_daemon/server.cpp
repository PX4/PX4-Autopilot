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
 * @file server.cpp
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <pthread.h>
#include <poll.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <px4_log.h>

#include "pxh.h"
#include "server.h"

// In Cygwin opening and closing the same named pipe multiple times within one process doesn't work POSIX compliant.
// As a workaround we open the client send pipe file in read write mode such that we can keep it open all the time.
#if !defined(__PX4_CYGWIN)
#define CLIENT_SEND_PIPE_OFLAGS O_RDONLY
#else
#define CLIENT_SEND_PIPE_OFLAGS O_RDWR
#endif

namespace px4_daemon
{

Server *Server::_instance = nullptr;

Server::Server(int instance_id)
	: _mutex(PTHREAD_MUTEX_INITIALIZER),
	  _instance_id(instance_id)
{
	_instance = this;
}

Server::~Server()
{
	_instance = nullptr;
}

int
Server::start()
{
	std::string client_send_pipe_path = get_client_send_pipe_path(_instance_id);

	// Delete pipe in case it exists already.
	unlink(client_send_pipe_path.c_str());

	// Create new pipe to listen to clients.
	// This needs to happen before we return from this method, so that the caller can launch clients.
	mkfifo(client_send_pipe_path.c_str(), 0666);

	if (0 != pthread_create(&_server_main_pthread,
				nullptr,
				_server_main_trampoline,
				nullptr)) {

		PX4_ERR("error creating client handler thread");

		return -1;
	}

	return 0;
}

void *
Server::_server_main_trampoline(void *arg)
{
	if (_instance) {
		_instance->_server_main(arg);
	}

	return nullptr;
}

void Server::_pthread_key_destructor(void *arg)
{
	delete ((CmdThreadSpecificData *)arg);
}

void
Server::_server_main(void *arg)
{
	// Set thread specific pipe to supplied file descriptor.
	int ret = pthread_key_create(&_key, _pthread_key_destructor);

	if (ret != 0) {
		PX4_ERR("failed to create pthread key");
		return;
	}

	std::string client_send_pipe_path = get_client_send_pipe_path(_instance_id);
	int client_send_pipe_fd = open(client_send_pipe_path.c_str(), CLIENT_SEND_PIPE_OFLAGS);

	while (true) {

		client_send_packet_s packet;

		// We only read as much as we need, otherwise we might get out of
		// sync with packets.
		int bytes_read = read(client_send_pipe_fd, &packet, sizeof(client_send_packet_s::header));

		if (bytes_read > 0) {

			// Using the header we can determine how big the payload is.
			int payload_to_read = sizeof(packet)
					      - sizeof(packet.header)
					      - sizeof(packet.payload)
					      + packet.header.payload_length;

			// Again, we only read as much as we need because otherwise we need
			// hold a buffer and parse it.
			bytes_read = read(client_send_pipe_fd, ((uint8_t *)&packet) + bytes_read, payload_to_read);

			if (bytes_read > 0) {

				_parse_client_send_packet(packet);

			}
		}

		if (bytes_read == 0) {
			// 0 means the pipe has been closed by all clients
			// and we need to re-open it.
			close(client_send_pipe_fd);
			client_send_pipe_fd = open(client_send_pipe_path.c_str(), O_RDONLY);
		}
	}

	close(client_send_pipe_fd);
}

void
Server::_parse_client_send_packet(const client_send_packet_s &packet)
{
	switch (packet.header.msg_id) {
	case client_send_packet_s::message_header_s::e_msg_id::EXECUTE:
		_execute_cmd_packet(packet);
		break;

	case client_send_packet_s::message_header_s::e_msg_id::KILL:
		_kill_cmd_packet(packet);
		break;

	default:
		PX4_ERR("send msg_id not handled");
		break;
	}
}

void
Server::_execute_cmd_packet(const client_send_packet_s &packet)
{
	if (packet.header.payload_length == 0) {
		PX4_ERR("command length 0");
		return;
	}

	// We open the client's specific pipe to write the return value and stdout back to.
	// The pipe's path is created knowing the UUID of the client.
	char path[RECV_PIPE_PATH_LEN];
	int ret = get_client_recv_pipe_path(packet.header.client_uuid, path, RECV_PIPE_PATH_LEN);

	if (ret < 0) {
		PX4_ERR("failed to assemble path");
		return;
	}

	int pipe_fd = open(path, O_WRONLY);

	if (pipe_fd < 0) {
		PX4_ERR("pipe open fail");
		return;
	}

	// To execute a command we start a new thread.
	pthread_t new_pthread;

	// We need to copy everything that the new thread needs because we will go
	// out of scope.
	RunCmdArgs *args = new RunCmdArgs;
	strncpy(args->cmd, (char *)packet.payload.execute_msg.cmd, sizeof(args->cmd));
	args->client_uuid = packet.header.client_uuid;
	args->pipe_fd = pipe_fd;
	args->is_atty = packet.payload.execute_msg.is_atty;

	_lock(); // need to lock, otherwise the thread could already exit before we insert into the map
	ret = pthread_create(&new_pthread, nullptr, Server::_run_cmd, (void *)args);

	if (ret != 0) {
		PX4_ERR("could not start pthread (%i)", ret);
		delete args;

	} else {
		// We won't join the thread, so detach to automatically release resources at its end
		pthread_detach(new_pthread);
		// We keep two maps for cleanup if a thread is finished or killed.
		_client_uuid_to_pthread.insert(std::pair<uint64_t, pthread_t>
					       (packet.header.client_uuid, new_pthread));
		_pthread_to_pipe_fd.insert(std::pair<pthread_t, int>
					   (new_pthread, pipe_fd));
	}

	_unlock();
}


void
Server::_kill_cmd_packet(const client_send_packet_s &packet)
{
	_lock();

	// TODO: we currently ignore the signal type.
	auto client_uuid_iter = _client_uuid_to_pthread.find(packet.header.client_uuid);

	if (client_uuid_iter == _client_uuid_to_pthread.end()) {
		_unlock();
		return;
	}

	pthread_t pthread_to_kill = client_uuid_iter->second;

	// TODO: use a more graceful exit method to avoid resource leaks
	int ret = pthread_cancel(pthread_to_kill);

	__cleanup_thread(packet.header.client_uuid);

	_unlock();

	if (ret != 0) {
		PX4_ERR("failed to cancel thread");
	}

	// We don't send retval when we get killed.
	// The client knows this and just exits without confirmation.
}



void
*Server::_run_cmd(void *arg)
{
	RunCmdArgs *args = (RunCmdArgs *)arg;

	// Copy arguments so that we can cleanup the arg structure.
	uint64_t client_uuid = args->client_uuid;
	int pipe_fd = args->pipe_fd;
	bool is_atty = args->is_atty;
	std::string message_str(args->cmd);

	// Clean up the args from the heap in case the thread gets canceled
	// from outside.
	delete args;

	// We register thread specific data. This is used for PX4_INFO (etc.) log calls.
	CmdThreadSpecificData *thread_data_ptr;

	if ((thread_data_ptr = (CmdThreadSpecificData *)pthread_getspecific(_instance->_key)) == nullptr) {
		thread_data_ptr = new CmdThreadSpecificData;
		thread_data_ptr->pipe_fd = pipe_fd;
		thread_data_ptr->is_atty = is_atty;
		thread_data_ptr->packet.header.msg_id = client_recv_packet_s::message_header_s::e_msg_id::STDOUT;

		(void)pthread_setspecific(_instance->_key, (void *)thread_data_ptr);
	}

	// Run the actual command.
	int retval = Pxh::process_line(message_str, true);

	// Report return value.
	_send_retval(pipe_fd, retval, client_uuid);

	// Clean up before returning.
	_instance->_cleanup_thread(client_uuid);

	return nullptr;
}


void
Server::_send_retval(const int pipe_fd, const int retval, const uint64_t client_uuid)
{
	client_recv_packet_s packet;

	packet.header.msg_id = client_recv_packet_s::message_header_s::e_msg_id::RETVAL;
	packet.header.payload_length = sizeof(packet.payload.retval_msg);
	packet.payload.retval_msg.retval = retval;

	int bytes_to_send = get_client_recv_packet_length(&packet);
	int bytes_sent = write(pipe_fd, &packet, bytes_to_send);

	if (bytes_sent != bytes_to_send) {
		printf("write fail\n");
		return;
	}
}

void
Server::_cleanup_thread(const uint64_t client_uuid)
{
	_lock();
	__cleanup_thread(client_uuid);
	_unlock();
}

void
Server::__cleanup_thread(const uint64_t client_uuid)
{
	pthread_t pthread_killed = _client_uuid_to_pthread[client_uuid];
	auto pipe_iter = _pthread_to_pipe_fd.find(pthread_killed);

	if (pipe_iter == _pthread_to_pipe_fd.end()) {
		// can happen if the thread already exited and then got a kill packet
		PX4_DEBUG("pipe fd already closed");
		return;
	}

	int pipe_fd = pipe_iter->second;
	close(pipe_fd);

	char path[RECV_PIPE_PATH_LEN] = {};
	get_client_recv_pipe_path(client_uuid, path, RECV_PIPE_PATH_LEN);
	unlink(path);

	_client_uuid_to_pthread.erase(client_uuid);
	_pthread_to_pipe_fd.erase(pthread_killed);
}

} //namespace px4_daemon
