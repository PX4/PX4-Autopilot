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



namespace px4_daemon
{

Server *Server::_instance = nullptr;

Server::Server()
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
	if (0 != pthread_create(&_server_main_pthread,
				NULL,
				_server_main_trampoline,
				NULL)) {

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

	return NULL;
}

void Server::_pthread_key_destructor(void *arg)
{
	delete((CmdThreadSpecificData *)arg);
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

	// Delete pipe in case it exists already.
	unlink(CLIENT_SEND_PIPE_PATH);

	// Create new pipe to listen to clients.
	mkfifo(CLIENT_SEND_PIPE_PATH, 0666);
	int client_send_pipe_fd = open(CLIENT_SEND_PIPE_PATH, O_RDONLY);

	while (true) {

		client_send_packet_s packet;

		int bytes_read = read(client_send_pipe_fd, &packet, sizeof(packet));

		if (bytes_read > 0) {
			_parse_client_send_packet(packet);

		} else if (bytes_read == 0) {
			// 0 means the pipe has been closed by all clients
			// and we need to re-open it.
			close(client_send_pipe_fd);
			client_send_pipe_fd = open(CLIENT_SEND_PIPE_PATH, O_RDONLY);
		}
	}

	close(client_send_pipe_fd);
	return;
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
	char path[RECV_PIPE_PATH_LEN] = {};
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
	memcpy(args->cmd, packet.payload.execute_msg.cmd, sizeof(args->cmd));
	args->client_uuid = packet.header.client_uuid;
	args->pipe_fd = pipe_fd;
	args->is_atty = packet.payload.execute_msg.is_atty;

	if (0 != pthread_create(&new_pthread, NULL, Server::_run_cmd, (void *)args)) {
		PX4_ERR("could not start pthread");
		delete args;
		return;
	}

	// We keep two maps for cleanup if a thread is finished or killed.
	_client_uuid_to_pthread.insert(std::pair<uint64_t, pthread_t>
				       (packet.header.client_uuid, new_pthread));
	_pthread_to_pipe_fd.insert(std::pair<pthread_t, int>
				   (new_pthread, pipe_fd));
}


void
Server::_kill_cmd_packet(const client_send_packet_s &packet)
{
	// TODO: we currently ignore the signal type.

	pthread_t pthread_to_kill = _client_uuid_to_pthread[packet.header.client_uuid];

	int ret = pthread_cancel(pthread_to_kill);

	if (ret != 0) {
		PX4_ERR("failed to cancel thread");
	}

	_cleanup_thread(packet.header.client_uuid);

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

	if ((thread_data_ptr = (CmdThreadSpecificData *)pthread_getspecific(_instance->_key)) == NULL) {
		thread_data_ptr = new CmdThreadSpecificData;
		thread_data_ptr->pipe_fd = pipe_fd;
		thread_data_ptr->is_atty = is_atty;
		thread_data_ptr->packet.header.msg_id = client_recv_packet_s::message_header_s::e_msg_id::STDOUT;

		(void)pthread_setspecific(_instance->_key, (void *)thread_data_ptr);
	}

	// Run the actual command.
	int retval = Pxh::process_line(message_str, false);

	// Report return value.
	_send_retval(pipe_fd, retval, client_uuid);

	// Clean up before returning.
	_instance->_cleanup_thread(client_uuid);

	return NULL;
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
		PX4_ERR("write fail");
		return;
	}
}

void
Server::_cleanup_thread(const uint64_t client_uuid)
{
	pthread_t pthread_killed = _client_uuid_to_pthread[client_uuid];
	int pipe_fd = _pthread_to_pipe_fd[pthread_killed];

	close(pipe_fd);

	_client_uuid_to_pthread.erase(client_uuid);
	_pthread_to_pipe_fd.erase(pthread_killed);
}

} //namespace px4_daemon
