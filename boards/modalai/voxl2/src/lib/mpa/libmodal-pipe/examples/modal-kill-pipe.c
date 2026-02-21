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
#include <string.h>
#include <stdlib.h>


#include <modal_pipe_client.h>
#include <modal_start_stop.h>

static char pipe_path[MODAL_PIPE_MAX_PATH_LEN];
static float timeout_s = 2.0;


static void _print_usage(void)
{
	printf("\n\
Tool to stop the owning process for a given pipe name\n\
and clean up any dangling pipes that may remain.\n\
\n\
This will first send SIGINT to simulate ctrl-C and wait\n\
for the specified timeout (default 2 seconds).\n\
If the process does not exit gracefully in the timeout, it\n\
will be sent SIGKILL to force the process to stop with blind\n\
and barbaric fury.\n\
\n\
If the process does not exit gracefully, it may leave the\n\
specified pipe dangling in the file system. If that were to\n\
occur, this program will cleanup the pipe from the file system.\n\
\n\
You can perform this same operation with this function that is\n\
part of libmodal_pipe: pipe_kill_server_process(pipe_path, 2.0)\n\
\n\
-h, --help                print this help message\n\
-t, --timeout_s{seconds}  timeout to wait before sending SIGKILL\n\
                            default is 2.0 seconds.\n\
\n\
example useage:\n\
\n\
long method to kill voxl-imu-server:\n\
/# modal_kill_pipe /run/mpa/imu0/\n\
\n\
shortcut method to kill voxl-camera-server\n\
/# modal_kill_pipe tracking\n\
\n");
	return;
}


static int parse_opts(int argc, char* argv[])
{
	static struct option long_options[] =
	{
		{"help",			no_argument,		0,	'h'},
		{"timeout_s",		required_argument,	0,	't'},
		{0, 0, 0, 0}
	};

	while(1){
		int option_index = 0;
		int c = getopt_long(argc, argv, "ht:", long_options, &option_index);

		if(c == -1) break; // Detect the end of the options.

		switch(c){
		case 0:
			// for long args without short equivalent that just set a flag
			// nothing left to do so just break.
			if (long_options[option_index].flag != 0) break;
			break;
		case 'h':
			_print_usage();
			exit(0);
		case 't':
			timeout_s = atof(optarg);
			if(timeout_s<0.1f){
				fprintf(stderr, "ERROR: timeout must be >=0.1s\n");
			}
			exit(-1);
		default:
			_print_usage();
			exit(-1);
		}
	}

	// scan through the non-flagged arguments for the desired pipe
	for(int i=optind; i<argc; i++){
		if(pipe_path[0]!=0){
			fprintf(stderr, "ERROR: Please specify only one pipe\n");
			_print_usage();
			exit(-1);
		}
		if(pipe_expand_location_string(argv[i], pipe_path)<0){
			fprintf(stderr, "ERROR: Invalid pipe name or location: %s\n", argv[i]);
			exit(-1);
		}
	}

	// make sure a pipe was given
	if(pipe_path[0] == 0){
		fprintf(stderr, "ERROR: You must specify a pipe name\n");
		_print_usage();
		exit(-1);
	}

	return 0;
}


int main(int argc, char* argv[])
{
	// check for options
	if(parse_opts(argc, argv)) return -1;

	pipe_kill_server_process(pipe_path, timeout_s);

	return 0;
}
