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
 * @file server_io.cpp
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <pthread.h>
#include <poll.h>
#include <assert.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <px4_log.h>

#include "server.h"
#include <px4_daemon/server_io.h>
#include "sock_protocol.h"


using namespace px4_daemon;


int get_stdout_pipe_buffer(char **buffer, unsigned *max_length, bool *is_atty)
{
	// The thread specific data won't be initialized if the server is not running.
	Server::CmdThreadSpecificData *thread_data_ptr;

	if (!Server::is_running()) {
		return -1;
	}

	// If we are not in a thread that has been started by a client, we don't
	// have any thread specific data set and we won't have a pipe to write
	// stdout to.
	if ((thread_data_ptr = (Server::CmdThreadSpecificData *)pthread_getspecific(
				       Server::get_pthread_key())) == nullptr) {
		return -1;
	}

#ifdef __PX4_POSIX_EAGLE

	// XXX FIXME: thread_data_ptr is set to 0x1 in the main thread on Snapdragon
	// even though the pthread_key has been created.
	// We can catch this using the check below but we have no clue why this happens.
	if (thread_data_ptr == (void *)0x1) {
		return -1;
	}

#endif

	*buffer = thread_data_ptr->buffer;
	*max_length = sizeof(thread_data_ptr->buffer);
	*is_atty = thread_data_ptr->is_atty;

	return 0;
}

int send_stdout_pipe_buffer(unsigned buffer_length)
{
	Server::CmdThreadSpecificData *thread_data_ptr;

	if (!Server::is_running()) {
		return -1;
	}

	if ((thread_data_ptr = (Server::CmdThreadSpecificData *)pthread_getspecific(
				       Server::get_pthread_key())) == nullptr) {
		return -1;
	}

	int bytes_sent = write(thread_data_ptr->fd, thread_data_ptr->buffer, buffer_length);

	if (bytes_sent != (int)buffer_length) {
		printf("write fail\n");
		return -1;
	}

	return 0;
}
