/****************************************************************************
 *
 *   Copyright (c) 2025 ModalAI, inc. All rights reserved.
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
#include <modal_pipe.h>

#pragma once

class MPA {
public:
	static int Initialize();

	typedef void (*mpa_data_cb_t)(char* data, int bytes);
	typedef void (*mpa_control_cb_t)(int ch, char* data, int bytes, void* context);

	static int PipeClient(const char *pipe_name, int size, mpa_data_cb_t cb);

	static int PipeCreate(char *pipe_name, int flags = 0);
	static int PipeWrite(int ch, const void* data, int bytes);
	static int PipeServerSetControlCb(int ch, mpa_control_cb_t cb, void* context);
	static void PipeServerClose(int ch);

private:
	static void HelperCB( __attribute__((unused)) int ch, char* data, int bytes, __attribute__((unused)) void* context);
	static void DisconnectCB(__attribute__((unused)) int ch, __attribute__((unused)) void* context);
	static void ConnectCB(__attribute__((unused)) int ch, __attribute__((unused)) void* context);

	typedef int (*pipe_client_set_simple_helper_cb_t)(int ch, client_simple_cb* cb, void* context);
	typedef int (*pipe_client_set_connect_cb_t)(int ch, client_connect_cb* cb, void* context);
	typedef int (*pipe_client_set_disconnect_cb_t)(int ch, client_disc_cb* cb, void* context);
	typedef int (*pipe_client_open_t)(int ch, const char* name_or_location, const char* client_name, int flags, int buf_len);
	typedef int (*pipe_server_create_t)(int ch, pipe_info_t info, int flags);
	typedef int (*pipe_server_write_t)(int ch, const void* data, int bytes);
	typedef int (*pipe_server_set_control_cb_t)(int ch, server_control_cb* cb, void* context);
	typedef void (*pipe_server_close_t)(int ch);

	static pipe_client_set_simple_helper_cb_t helper_cb;
	static pipe_client_set_connect_cb_t connect_cb;
	static pipe_client_set_disconnect_cb_t disconnect_cb;
	static pipe_client_open_t open_pipe;
	static pipe_server_create_t create_pipe;
	static pipe_server_write_t write_pipe;
	static pipe_server_set_control_cb_t set_control_cb;
	static pipe_server_close_t close_pipe;

	static bool initialized;
	static void *handle;

	static int current_client;
	static int current_server;

	static const int MAX_MPA_CLIENTS{8};
	static mpa_data_cb_t data_cb[MAX_MPA_CLIENTS];
};
