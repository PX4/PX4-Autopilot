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

#include <modal_json.h>
#include <modal_pipe_client.h>
#include <modal_start_stop.h>

static char pipe_path[MODAL_PIPE_MAX_PATH_LEN];


static void _print_usage(void)
{
	printf("\n\
Tool to read and print the json info for a pipe\n\
\n\
-h, --help                print this help message\n\
\n\
example useage:\n\
\n\
/# voxl-inspect-pipe-info imu_apps\n\
\n");
	return;
}


static int parse_opts(int argc, char* argv[])
{
	static struct option long_options[] =
	{
		{"help",			no_argument,		0,	'h'},
		{0, 0, 0, 0}
	};

	while(1){
		int option_index = 0;
		int c = getopt_long(argc, argv, "h", long_options, &option_index);

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

	if(!pipe_exists(pipe_path)){
		fprintf(stderr, "pipe %s does not exist\n", pipe_path);
		exit(-1);
	}

	cJSON* json = pipe_get_info_json(pipe_path);

	if(json==NULL){
		fprintf(stderr, "error reading info json from %s\n", pipe_path);
		exit(-1);
	}

	int ret = json_print(json);
	cJSON_free(json);
	printf("\n");

	return ret;;
}
