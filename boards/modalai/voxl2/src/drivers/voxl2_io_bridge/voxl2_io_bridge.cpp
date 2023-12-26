/****************************************************************************
 *
 *   Copyright (c) 2012-2020 PX4 Development Team. All rights reserved.
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


#include <string>
#include <semaphore.h>
#include <px4_log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/getopt.h>
#include <uORB/Publication.hpp>
#include <drivers/drv_hrt.h>
#include <uORB/topics/voxl2_io_data.h>
#include <lib/parameters/param.h>


#include "modal_pipe_sink.h"

#define SINK_PATH (MODAL_PIPE_DEFAULT_BASE_DIR "voxl2_io_bridge")
#define READ_BUF_SIZE 1024
#define PIPE_SIZE (64*1024)

#include <limits.h>     // for PATH_MAX
#include <sys/stat.h>   // for mkdir

extern "C" { 
int _mkdir_recursive(const char* dir)
{
    char tmp[PATH_MAX];
    char* p = NULL;

    snprintf(tmp, sizeof(tmp),"%s",dir);
    for(p = tmp + 1; *p!=0; p++){
        if(*p == '/'){
            *p = 0;
            if(mkdir(tmp, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) && errno!=EEXIST){
                perror("ERROR calling mkdir");
                printf("tried to make %s\n", tmp);
                return -1;
            }
            *p = '/';
        }
    }
    return 0;
}
}

extern "C" { __EXPORT int voxl2_io_bridge_main(int argc, char *argv[]); }

namespace voxl2_io_bridge
{

bool _initialized = false;
bool _is_running = false;
bool _debug = false;

static px4_task_t _task_handle = -1;

uint8_t _data_buffer[READ_BUF_SIZE];
uint32_t _data_len = 0;

px4_sem_t _new_data_sem;

uORB::Publication<voxl2_io_data_s> _data_pub{ORB_ID(voxl2_io_data)};

static void simple_cb(int ch, char* data, int bytes, __attribute__((unused)) void* context) {

	if (bytes) {
		memcpy(_data_buffer, (uint8_t*) data, bytes);
		_data_len = bytes;

		if (_debug) {
			PX4_INFO("Received %d bytes on channel %d", bytes, ch);
			PX4_INFO("   0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x",
					 _data_buffer[0], _data_buffer[1], _data_buffer[2], _data_buffer[3],
					 _data_buffer[4], _data_buffer[5], _data_buffer[6], _data_buffer[7]);
		}

		px4_sem_post(&_new_data_sem);
	} else if (_debug) {
		PX4_ERR("Got callback with zero data available");
	}

	return;
}

int initialize()
{
	if (_initialized) {
		// Already successfully initialized
		return 0;
	}

   (void) px4_sem_init(&_new_data_sem, 0, 0);

	if (pipe_sink_create(0, SINK_PATH, SINK_FLAG_EN_SIMPLE_HELPER, PIPE_SIZE, READ_BUF_SIZE)) {
		return -1;
	}

	pipe_sink_set_simple_cb(0, &simple_cb, NULL);

	_initialized = true;

	return 0;
}

void voxl2_io_bridge_task() {
	
	voxl2_io_data_s	io_data;

	_is_running = true;

	PX4_INFO("Modal IO Bridge driver starting");

	while (true) {

		do {} while (px4_sem_wait(&_new_data_sem) != 0);

		if (_data_len) {
			memset(&io_data, 0, sizeof(voxl2_io_data_s));

			io_data.timestamp = hrt_absolute_time();
			io_data.len = _data_len;
			memcpy(io_data.data, _data_buffer, _data_len);

			if (_debug) {
				PX4_INFO("Publishing modal io data");
			}

			_data_pub.publish(io_data);
		} else if (_debug) {
			PX4_ERR("Got semaphore with zero data to be sent");
		}
	}
}

int start(int argc, char *argv[]) {

	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			_debug = true;
			PX4_INFO("Setting debug flag to true");
			break;
		default:
			break;
		}
	}

	if (! _initialized) {
		if (initialize()) {
			return -1;
		}
	}

	if (_is_running) {
		PX4_WARN("Already started");
		return 0;
	}

	_task_handle = px4_task_spawn_cmd("voxl2_io_bridge_main",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT,
					  2000,
					  (px4_main_t) &voxl2_io_bridge_task,
					  (char *const *)argv);

	if (_task_handle < 0) {
		PX4_ERR("task start failed");
		return -1;
	}

	return 0;
}

void
usage()
{
	PX4_INFO("Usage: voxl2_io_bridge start [options]");
	PX4_INFO("Options: -d    enable debug output");
}

} // End namespance voxl2_io_bridge

int voxl2_io_bridge_main(int argc, char *argv[])
{
	int myoptind = 1;

	if (argc <= 1) {
		voxl2_io_bridge::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		return voxl2_io_bridge::start(argc - 1, argv + 1);
	} else {
		voxl2_io_bridge::usage();
		return -1;
	}

	return 0;
}
