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

#include "mpa.hpp"
#include <dlfcn.h>
#include <px4_log.h>
#include <string.h>

bool MPA::initialized = false;
void *MPA::handle = nullptr;
int MPA::current_client = 0;
int MPA::current_server = 0;

MPA::pipe_client_set_simple_helper_cb_t MPA::helper_cb = nullptr;
MPA::pipe_client_set_connect_cb_t MPA::connect_cb = nullptr;
MPA::pipe_client_set_disconnect_cb_t MPA::disconnect_cb = nullptr;
MPA::pipe_client_open_t MPA::open_pipe = nullptr;
MPA::pipe_server_create_t MPA::create_pipe = nullptr;
MPA::pipe_server_write_t MPA::write_pipe = nullptr;
MPA::mpa_data_cb_t MPA::data_cb[MAX_MPA_CLIENTS];

// called whenever we connect or reconnect to the server
void MPA::ConnectCB(__attribute__((unused)) int ch, __attribute__((unused)) void* context)
{
	PX4_INFO("vfc status server connected");
	return;
}

// called whenever we disconnect from the server
void MPA::DisconnectCB(__attribute__((unused)) int ch, __attribute__((unused)) void* context)
{
	PX4_INFO("vfc status server disconnected");
	return;
}

void MPA::HelperCB( __attribute__((unused)) int ch, char* data, int bytes, __attribute__((unused)) void* context)
{
	// PX4_INFO("Got %d bytes in pipe callback", bytes);

	if (data_cb[ch]) data_cb[ch](data, bytes);

	return;
}

int MPA::PipeClient(const char *pipe_name, int size, mpa_data_cb_t cb)
{
	if (!initialized) {
		PX4_ERR("Cannot open pipe %s before initialization", pipe_name);
		return -1;
	}

	printf("waiting for server for pipe %s\n", pipe_name);

	if (open_pipe(current_client, pipe_name, "px4", EN_PIPE_CLIENT_SIMPLE_HELPER, size * 10) < 0) {
		PX4_ERR("Error opening pipe %s", pipe_name);
		return -1;
	}

	data_cb[current_client] = cb;
	current_client++;

	return current_client - 1;
}

int MPA::PipeCreate(char *pipe_name)
{
	if (!initialized) {
		PX4_ERR("Cannot open pipe %s before initialization", pipe_name);
		return -1;
	}

	pipe_info_t server_pipe;
	strncpy(server_pipe.name, pipe_name, MODAL_PIPE_MAX_NAME_LEN);
	server_pipe.name[MODAL_PIPE_MAX_NAME_LEN - 1] = 0;
	server_pipe.location[0] = 0;
	server_pipe.type[0]     = 0;
	strncpy(server_pipe.server_name, "px4_mpa", MODAL_PIPE_MAX_NAME_LEN);
	server_pipe.size_bytes  = MODAL_PIPE_DEFAULT_PIPE_SIZE;
	server_pipe.server_pid  = 0;

	if (create_pipe(current_server, server_pipe, 0) < 0) {
		// remove_pid_file(server_pipe.server_name);
		PX4_ERR("Error opening pipe %s", pipe_name);
		return -1;
	}

	current_server++;

	return current_server - 1;
}

int MPA::PipeWrite(int ch, const void* data, int bytes) {
	return write_pipe(ch, data, bytes);
}

int MPA::Initialize()
{
	if (initialized) {
		// Already successfully initialized
		return 0;
	}

	char libname[] = "libmodal_pipe.so";
	handle = dlopen(libname, RTLD_LAZY | RTLD_GLOBAL);
	if (!handle) {
		PX4_ERR("Error opening library %s: %s\n", libname, dlerror());
		return -1;
	} else {
		PX4_INFO("Successfully loaded library %s", libname);
	}

	// set up all our MPA callbacks
	char helper_cb_name[] = "pipe_client_set_simple_helper_cb";
	helper_cb = (pipe_client_set_simple_helper_cb_t) dlsym(handle, helper_cb_name);
	if (!helper_cb) {
		PX4_ERR("Error finding symbol %s: %s\n", helper_cb_name, dlerror());
		return -1;
	} else {
		PX4_DEBUG("Successfully loaded function %s", helper_cb_name);
	}
	helper_cb(0, HelperCB, NULL);

	char connect_cb_name[] = "pipe_client_set_connect_cb";
	connect_cb = (pipe_client_set_connect_cb_t) dlsym(handle, connect_cb_name);
	if (!connect_cb) {
		PX4_ERR("Error finding symbol %s: %s", connect_cb_name, dlerror());
		return -1;
	} else {
		PX4_DEBUG("Successfully loaded function %s", connect_cb_name);
	}
	connect_cb(0, ConnectCB, NULL);

	char disconnect_cb_name[] = "pipe_client_set_disconnect_cb";
	disconnect_cb = (pipe_client_set_disconnect_cb_t) dlsym(handle, disconnect_cb_name);
	if (!disconnect_cb) {
		PX4_ERR("Error finding symbol %s: %s", disconnect_cb_name, dlerror());
		return -1;
	} else {
		PX4_DEBUG("Successfully loaded function %s", disconnect_cb_name);
	}
	disconnect_cb(0, DisconnectCB, NULL);

	// request a new pipe from the server
	char open_pipe_name[] = "pipe_client_open";
	open_pipe = (pipe_client_open_t) dlsym(handle, open_pipe_name);
	if (!open_pipe) {
		PX4_ERR("Error finding symbol %s: %s", open_pipe_name, dlerror());
		return -1;
	} else {
		PX4_DEBUG("Successfully loaded function %s", open_pipe_name);
	}

	// Create a new server pipe
	char create_pipe_name[] = "pipe_server_create";
	create_pipe = (pipe_server_create_t) dlsym(handle, create_pipe_name);
	if (!create_pipe) {
		PX4_ERR("Error finding symbol %s: %s", create_pipe_name, dlerror());
		return -1;
	} else {
		PX4_DEBUG("Successfully loaded function %s", create_pipe_name);
	}

	// Write to a server pipe
	char write_pipe_name[] = "pipe_server_write";
	write_pipe = (pipe_server_write_t) dlsym(handle, write_pipe_name);
	if (!write_pipe) {
		PX4_ERR("Error finding symbol %s: %s", write_pipe_name, dlerror());
		return -1;
	} else {
		PX4_DEBUG("Successfully loaded function %s", write_pipe_name);
	}

	initialized = true;

	return 0;
}

