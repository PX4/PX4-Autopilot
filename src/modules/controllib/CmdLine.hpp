/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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

/*
 * @file cmdline_main.cpp
 *
 * Attitude estimator (quaternion based)
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author James Goppert <james.goppert@gmail.com>
 */

#include <px4_posix.h>
#include <inttypes.h>
#include <errno.h>

namespace control
{

template<class T>
class CmdLine
{
public:
	static CmdLine * instance;
	CmdLine() :
		_task_should_exit(false),
		_task(-1)
	{
	}

	~CmdLine() {
		if (_task != -1) {
			/* task wakes up every 100ms or so at the longest */
			_task_should_exit = true;

			/* wait for a second for the task to quit at our request */
			unsigned i = 0;

			do {
				/* wait 20ms */
				usleep(20000);

				/* if we have given up, kill it */
				if (++i > 50) {
					px4_task_delete(_task);
					break;
				}
			} while (_task != -1);
		}
	}

	int	start()
	{
		ASSERT(_task == -1);

		/* start the task */
		_task = px4_task_spawn_cmd("cmdline",
			SCHED_DEFAULT,
		   	SCHED_PRIORITY_MAX - 5,
		   	2100,
			(px4_main_t)&CmdLine::task_main_trampoline,
			nullptr);

		if (_task < 0) {
			warn("task start failed");
			return -errno;
		}

		return OK;
	}

	static void	task_main_trampoline(int argc, char *argv[])
	{
		CmdLine::instance->task_main();
	}

	void task_main() {
		T block;
		while (!_task_should_exit) {
			block.update();
		}
	}

	void status() {
		warnx("status:");
	}

	static int main(int argc, char *argv[])
	{
		if (argc < 1) {
			warnx("usage: cmdline {start|stop|status}");
			return 1;
		}

		if (!strcmp(argv[1], "start")) {

			if (CmdLine::instance != nullptr) {
				warnx("already running");
				return 1;
			}

			CmdLine::instance = new CmdLine;

			if (CmdLine::instance == nullptr) {
				warnx("alloc failed");
				return 1;
			}

			if (OK != CmdLine::instance->start()) {
				delete CmdLine::instance;
				CmdLine::instance = nullptr;
				warnx("start failed");
				return 1;
			}

			return 0;
		}

		if (!strcmp(argv[1], "stop")) {
			if (CmdLine::instance == nullptr) {
				warnx("not running");
				return 1;
			}

			delete CmdLine::instance;
			CmdLine::instance = nullptr;
			return 0;
		}

		if (!strcmp(argv[1], "status")) {
			if (CmdLine::instance) {
				CmdLine::instance->status();
				warnx("running");
				return 0;

			} else {
				warnx("not running");
				return 1;
			}
		}

		warnx("unrecognized command");
		return 1;
	}

private:
	bool _task_should_exit;
	int	_task;
};


template<class T>
CmdLine<T> * CmdLine<T>::instance = nullptr;

} // namespace control
