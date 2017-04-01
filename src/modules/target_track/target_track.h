/*
 * target_track.h

 *
 *  Created on: Mar 5, 2017
 *      Author: kzf
 */
#include <px4_app.h>
#include <px4_tasks.h>
#include <stdio.h>
#include <string.h>
#include <sched.h>

class target_track;


#define SCHED_DEFAULT	SCHED_FIFO

int target_track_thread_main(int argc, char *argv[]);

int target_track_thread_main(int argc, char *argv[]);

static void usage(const char *reason);



