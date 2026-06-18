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
#include <modal_pipe_sink.h>


// this is the directory used by the voxl-hello-server for named pipes
#define SINK_PATH	(MODAL_PIPE_DEFAULT_BASE_DIR "hello-sink")

// you may need a larger buffer for your application!
#define READ_BUF_SIZE	1024
#define PIPE_SIZE		(64*1024)


static void simple_cb(int ch, char* data, int bytes, __attribute__((unused)) void* context){
	// for this tester we expect users to echo to the sink
	// echo does not write a null-terminated string, it's newline terminated,
	// so specify the number of bytes for printf to print or it will print
	// any other nonsense in the buffer up to the first NULL
	// # bytes includes the newline character
	printf("received %d bytes on channel %d: %.*s", bytes, ch, bytes, data);
	return;
}

int main()
{
	// set some basic signal handling for safe shutdown
	enable_signal_handler();

	// start both sinks
	if(pipe_sink_create(0, SINK_PATH, SINK_FLAG_EN_SIMPLE_HELPER, PIPE_SIZE, READ_BUF_SIZE)) return -1;
	pipe_sink_set_simple_cb(0, &simple_cb, NULL);


	printf("done initializing, try the following command in another terminal:\n");
	printf("echo hello > %s\n", SINK_PATH);

	// keep going until signal handler sets the running flag to 0
	main_running = 1;
	while(main_running) usleep(500000);

	// all done, signal pipe read threads to stop
	pipe_sink_close_all();

	printf("exiting cleanly\n");
	return 0;
}
