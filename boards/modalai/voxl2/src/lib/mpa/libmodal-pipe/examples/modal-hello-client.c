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
#include <getopt.h>
#include <unistd.h>	// for usleep()
#include <string.h>

#include <modal_start_stop.h>
#include <modal_pipe_client.h>


// this is the directory used by the voxl-hello-server for named pipes
#define PIPE_NAME	"hello"
#define CLIENT_CH	0

// you may need a larger buffer for your application!
#define PIPE_READ_BUF_SIZE	1024
#define CLIENT_NAME	"modal-hello-client"

static int en_debug;


static void _print_usage(void)
{
	printf("\n\
This is a test of libmodal_pipe. It connects to the pipe dir /run/mpa/hello/\n\
created by modal-hello-server.\n\
\n\
Run this in debug mode to enable debug prints in the libmodal_pipe client code.\n\
This is a good example of using the simple helper feature.\n\
\n\
See the voxl-inspect-* examples in the voxl-mpa-tools repository for examples on\n\
reading other sorts of MPA data such as camera and IMU data\n\
\n\
-d, --debug                 print debug info\n\
-h, --help                  print this help message\n\
\n");
	return;
}

// called whenever the simple helper has data for us to process
static void _simple_cb(int ch, char* data, int bytes, __attribute__((unused)) void* context)
{
	printf("received %d bytes on channel %d: %s\n", bytes, ch, data);
	return;
}

// called whenever we connect or reconnect to the server
static void _connect_cb(int ch, __attribute__((unused)) void* context)
{
	int ret;
	fprintf(stderr, "channel %d connected to server\n", ch);

	// send a hello message back to server via the control pipe (for fun)
	// not all servers will have a control pie, it's optional
	printf("sending hello to server control pipe\n");
	ret = pipe_client_send_control_cmd(0, "hello from client!");
	if(ret<0){
		fprintf(stderr, "failed to send control command to server\n");
		pipe_print_error(ret);
	}

	// now we are connected and before we read data,
	// check that the type is correct!!!
	if(!pipe_is_type(PIPE_NAME, "text")){
		fprintf(stderr, "ERROR, pipe is not of type \"text\"\n");
		main_running = 0;
		return;
	}

	return;
}


// called whenever we disconnect from the server
static void _disconnect_cb(int ch, __attribute__((unused)) void* context)
{
	fprintf(stderr, "channel %d disconnected from server\n", ch);
	return;
}


// not many command line arguments
static int _parse_opts(int argc, char* argv[])
{
	static struct option long_options[] =
	{
		{"debug",			no_argument,		0,	'd'},
		{"help",			no_argument,		0,	'h'},
		{0, 0, 0, 0}
	};
	while(1){
		int option_index = 0;
		int c = getopt_long(argc, argv, "dh", long_options, &option_index);
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
		case 'h':
			_print_usage();
			return -1;
		default:
			_print_usage();
			return -1;
		}
	}
	return 0;
}


int main(int argc, char* argv[])
{
	int ret;

	// check for options
	if(_parse_opts(argc, argv)) return -1;

	// set some basic signal handling for safe shutdown.
	// quitting without cleanup up the pipe can result in the pipe staying
	// open and overflowing, so always cleanup properly!!!
	enable_signal_handler();
	main_running = 1;

	// for this test we will use the simple helper with optional debug mode
	int flags 						= CLIENT_FLAG_EN_SIMPLE_HELPER;
	if(en_debug)			flags  |= CLIENT_FLAG_EN_DEBUG_PRINTS;

	printf("waiting for modal-hello-server\n");

	// assign callabcks for data, connection, and disconnect. the "NULL" arg
	// here can be an optional void* context pointer passed back to the callbacks
	pipe_client_set_simple_helper_cb(	CLIENT_CH,	_simple_cb,		NULL);
	pipe_client_set_connect_cb(			CLIENT_CH,	_connect_cb,	NULL);
	pipe_client_set_disconnect_cb(		CLIENT_CH,	_disconnect_cb,	NULL);

	// init connection to server. In auto-reconnect mode this will "succeed"
	// even if the server is offline, but it will connect later on automatically
	ret = pipe_client_open(CLIENT_CH, PIPE_NAME, CLIENT_NAME, flags, PIPE_READ_BUF_SIZE);

	// check for success
	if(ret){
		fprintf(stderr, "ERROR opening channel:\n");
		pipe_print_error(ret);
		return -1;
	}

	// keep going until signal handler sets the main_running flag to 0
	while(main_running) usleep(500000);

	// all done, signal pipe read threads to stop
	printf("closing\n");
	fflush(stdout);
	pipe_client_close_all();

	return 0;
}
