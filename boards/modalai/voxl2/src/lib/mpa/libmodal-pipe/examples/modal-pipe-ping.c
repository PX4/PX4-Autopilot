/*******************************************************************************
 * Copyright 2025 ModalAI Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 ******************************************************************************/


#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>
#include <getopt.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#include <modal_start_stop.h>
#include <modal_pipe_server.h>
#include <modal_pipe_client.h>

#define SERVER_NAME	"ping-server"
#define CLIENT_NAME	"ping-client"
#define PIPE_NAME	"hello"
#define TEST_LOCATION (MODAL_PIPE_DEFAULT_BASE_DIR PIPE_NAME "/request")
#define CH			0 // arbitrarily use channel 0 for our one pipe
#define PIPE_READ_BUF_SIZE 256

// vars
static int en_debug;
static double frequency_hz = 2.0;
static int priority = 0;

// program automatically decides to be in client or server mode
#define MODE_SERVER 0
#define MODE_CLIENT 1
static int mode;

// printed if some invalid argument was given
static void print_usage(void)
{
	printf("\n\
Start two instances of this program in two separate termainals.\n\
The first to start will make itself a server, the other will be a client.\n\
They will ping eachother back and forth timing the latency.\n\
\n\
-d, --debug                 print debug info\n\
-f, --frequency             publish frequency in hz\n\
-h, --help                  print this help message\n\
-p, --priority              0-99\n\
\n");
	return;
}


static int _exists(char* path)
{
	// file exists
	if(access(path, F_OK ) != -1 ) return 1;
	// file doesn't exist
	return 0;
}

static int64_t _time_monotonic_ns(void)
{
	struct timespec ts;
	if(clock_gettime(CLOCK_MONOTONIC, &ts)){
		fprintf(stderr,"ERROR calling clock_gettime\n");
		return -1;
	}
	return (int64_t)ts.tv_sec*1000000000 + (int64_t)ts.tv_nsec;
}


static void _server_control_handler(__attribute__((unused))int ch, char* data, int bytes, __attribute__((unused)) void* context)
{
	if(bytes != sizeof(int64_t)) return;
	int64_t* t_ptr = (int64_t*)data;
	double dt_ms = (_time_monotonic_ns() - *t_ptr)/1000000.0;

	static int has_printed_properties = 0;
	if(!has_printed_properties){
		pipe_pthread_print_properties(0);
		has_printed_properties = 1;
	}

	printf("latency client to server: %6.2fms\n", dt_ms);
	return;
}


static void _server_connect_handler(int ch, int client_id, char* client_name, __attribute__((unused)) void* context)
{
	printf("client \"%s\" connected to channel %d  with client id %d\n", client_name, ch, client_id);
	return;
}


static void _server_disconnect_handler(int ch, int client_id, char* name, __attribute__((unused)) void* context)
{
	printf("client \"%s\" with id %d has disconnected from channel %d\n", name, client_id, ch);
	return;
}


static void _client_simple_cb(__attribute__((unused))int ch, char* data, int bytes, __attribute__((unused)) void* context)
{
	if(bytes != sizeof(int64_t)) return;
	int64_t* t_ptr = (int64_t*)data;
	double dt_ms = (_time_monotonic_ns() - *t_ptr)/1000000.0;

	static int has_printed_properties = 0;
	if(!has_printed_properties){
		pipe_pthread_print_properties(0);
		has_printed_properties = 1;
	}

	printf("latency server to client: %6.2fms\n", dt_ms);

	// also send a time back to server
	int64_t t = _time_monotonic_ns();
	pipe_client_send_control_cmd_bytes(CH, &t, sizeof(t));

	return;
}


static void _client_connect_cb(int ch, __attribute__((unused)) void* context)
{
	fprintf(stderr, "channel %d connected to server\n", ch);
	return;
}


static void _client_disconnect_cb(int ch, __attribute__((unused)) void* context)
{
	fprintf(stderr, "channel %d disconnected from server\n", ch);
	return;
}


static int _parse_opts(int argc, char* argv[])
{
	static struct option long_options[] =
	{
		{"debug",			no_argument,		0,	'd'},
		{"frequency",		required_argument,	0,	'f'},
		{"help",			no_argument,		0,	'h'},
		{"priority",		required_argument,	0,	'p'},
		{0, 0, 0, 0}
	};

	while(1){
		int option_index = 0;
		int c = getopt_long(argc, argv, "df:hp:", long_options, &option_index);

		if(c == -1) break; // Detect the end of the options.

		switch(c){
		case 0:
			// for long args without short equivalent that just set a flag
			// nothing left to do so just break.
			if (long_options[option_index].flag != 0) break;
			break;

		case 'd':
			en_debug = 1;
			break;

		case 'f':
			frequency_hz = atof(optarg);
			if(frequency_hz<0.5){
				fprintf(stderr, "ERROR: frequency must be > 0.5hz\n");
				return -1;
			}
			break;

		case 'h':
			print_usage();
			return -1;

		case 'p':
			priority = atoi(optarg);
			break;

		default:
			print_usage();
			return -1;
		}
	}

	return 0;
}




int main(int argc, char* argv[])
{
	// check for options
	if(_parse_opts(argc, argv)) return -1;

	// if pipe exists, start as client
	if(_exists(TEST_LOCATION)){
		printf("ping pipe exists, starting as client\n");
		mode = MODE_CLIENT;
	}
	else{
		printf("ping pipe does not exist yet, starting as server\n");
		mode = MODE_SERVER;
	}

////////////////////////////////////////////////////////////////////////////////
// gracefully handle an existing instance of the process and associated PID file
////////////////////////////////////////////////////////////////////////////////

	// make sure another instance isn't running
	// if return value is -3 then a background process is running with
	// higher privaledges and we couldn't kill it, in which case we should
	// not continue or there may be hardware conflicts. If it returned -4
	// then there was an invalid argument that needs to be fixed.
	if(mode==MODE_SERVER){
		if(kill_existing_process(SERVER_NAME, 2.0)<-2) return -1;
	}

	// start signal handler so we can exit cleanly
	if(enable_signal_handler()==-1){
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return -1;
	}

////////////////////////////////////////////////////////////////////////////////
// Server mode, set up the pipe
////////////////////////////////////////////////////////////////////////////////

	if(mode==MODE_SERVER){
		// enable the control pipe feature and optionally debug prints
		int flags = SERVER_FLAG_EN_CONTROL_PIPE;
		if(en_debug) flags|=SERVER_FLAG_EN_DEBUG_PRINTS;

		// configure optional callbacks
		pipe_server_set_control_cb(CH, &_server_control_handler, NULL);
		pipe_server_set_connect_cb(CH, &_server_connect_handler, NULL);
		pipe_server_set_disconnect_cb(CH, &_server_disconnect_handler, NULL);
		if(priority!=0){
			pipe_server_set_control_thread_priority(CH, priority);
		}

		// create the pipe
		pipe_info_t info = { \
			.name        = PIPE_NAME,\
			.location    = "",\
			.type        = "text",\
			.server_name = SERVER_NAME,\
			.size_bytes  = MODAL_PIPE_DEFAULT_PIPE_SIZE};

		if(pipe_server_create(CH, info, flags)) return -1;

		// add in an optional field to the info JSON file
		cJSON* info_json = pipe_server_get_info_json_ptr(CH);
		cJSON_AddStringToObject(info_json, "description", "Test pipe sends hello text messages");
		pipe_server_update_info(CH);

		// make PID file to indicate your project is running
		// due to the check made on the call to rc_kill_existing_process() above
		// we can be fairly confident there is no PID file already and we can
		// make our own safely.
		make_pid_file(SERVER_NAME);

		printf("now start another instance of modal-pipe-ping in another window to be the client\n");
	}

////////////////////////////////////////////////////////////////////////////////
// Client mode, connect to the pipe
////////////////////////////////////////////////////////////////////////////////
	else{

		// for this test we will use the simple helper with optional debug mode
		int flags 						= CLIENT_FLAG_EN_SIMPLE_HELPER;
		if(en_debug)			flags  |= CLIENT_FLAG_EN_DEBUG_PRINTS;

		// assign callabcks for data, connection, and disconnect. the "NULL" arg
		// here can be an optional void* context pointer passed back to the callbacks
		pipe_client_set_simple_helper_cb(0, _client_simple_cb, NULL);
		pipe_client_set_connect_cb(0, _client_connect_cb, NULL);
		pipe_client_set_disconnect_cb(0, _client_disconnect_cb, NULL);
		if(priority!=0){
			pipe_client_set_helper_thread_priority(CH, priority);
		}

		// init connection to server. In auto-reconnect mode this will "succeed"
		// even if the server is offline, but it will connect later on automatically
		int ret = pipe_client_open(0, PIPE_NAME, CLIENT_NAME, flags, PIPE_READ_BUF_SIZE);
		// check for success
		if(ret){
			fprintf(stderr, "ERROR opening channel:\n");
			pipe_print_error(ret);
			return -1;
		}
		printf("waiting for server\n");
	}

////////////////////////////////////////////////////////////////////////////////
// all threads started, wait for signal handler to stop it
////////////////////////////////////////////////////////////////////////////////

	main_running=1; // this is an extern variable in start_stop.c
	while(main_running){

		if(mode==MODE_SERVER){
			int64_t t = _time_monotonic_ns();
			pipe_server_write(CH, &t, sizeof(t));
		}

		// rough publish rate
		usleep(1000000/frequency_hz);
	}

////////////////////////////////////////////////////////////////////////////////
// Stop all the threads and do cleanup HERE
////////////////////////////////////////////////////////////////////////////////

	if(mode==MODE_SERVER){
		pipe_server_close_all();
		remove_pid_file(SERVER_NAME);
	}
	else{
		pipe_client_close_all();
	}

	printf("exiting cleanly\n");
	return 0;
}
