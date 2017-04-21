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
 * @file pipe_protocol.h
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Beat Küng <beat-kueng@gmx.net>
 */
#pragma once

#include <stdint.h>

namespace px4_daemon
{


static const char CLIENT_SEND_PIPE_PATH[] = "/tmp/px4_client_send_pipe";
static const char CLIENT_RECV_PIPE_PATH[] = "/tmp/px4_client_recv_pipe";

static const unsigned RECV_PIPE_PATH_LEN = 64;

struct client_send_packet_s {
	struct message_header_s {
		enum class e_msg_id : uint8_t {
			EXECUTE,
			KILL
		} msg_id;
		uint64_t client_uuid;
		unsigned payload_length;
	} header;

	union {
		struct execute_msg_s {
			uint8_t is_atty;
			uint8_t cmd[512 - sizeof(message_header_s) - sizeof(uint8_t)];
		} execute_msg;
		struct kill_msg_s {
			int cmd_id;
		} kill_msg;
	} payload;
};

// We have per client receiver a pipe with the uuid in its file path.
struct client_recv_packet_s {
	struct message_header_s {
		enum class e_msg_id : uint8_t {
			RETVAL,
			STDOUT
		} msg_id;
		unsigned payload_length;
	} header;

	union {
		struct retval_msg_s {
			int retval;
		} retval_msg;
		struct stdout_msg_s {
			uint8_t text[512 - sizeof(message_header_s)];
		} stdout_msg;
	} payload;
};

unsigned get_client_send_packet_length(const client_send_packet_s *packet);
unsigned get_client_recv_packet_length(const client_recv_packet_s *packet);
int get_client_recv_pipe_path(const uint64_t uuid, char *path, const size_t path_len);

} // namespace px4_daemon

