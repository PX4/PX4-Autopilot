/****************************************************************************
 *
 *   Copyright (C) 2015-2022 PX4 Development Team. All rights reserved.
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
/**
 * @file pxh.cpp
 *
 * This is a simple PX4 shell implementation used to start modules.
 *
 * @author Mark Charlebois <charlebm@gmail.com>
 * @author Roman Bapst <bapstroman@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 */

#include <cstdint>
#include <string.h>
#include <strings.h>
#include <errno.h>
#include <sstream>
#include <vector>
#include <algorithm>
#include <stdio.h>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>

#include <px4_log.h>
#include "pxh.h"
#include "server.h"

namespace px4_daemon
{

apps_map_type Pxh::_apps = {};
Pxh *Pxh::_instance = nullptr;

Pxh::Pxh()
{
}

Pxh::~Pxh()
{
}

int Pxh::process_line(const std::string &line, bool silently_fail)
{
	if (line.empty()) {
		return 0;
	}

	if (_apps.empty()) {
		init_app_map(_apps);
	}

	std::stringstream line_stream(line);
	std::string word;
	std::vector<std::string> words;

	// First arg should be the command.
	while (line_stream >> word) {
		words.push_back(word);
	}

	if (words.empty()) {
		return 0;
	}

	const std::string &command(words.front());

	if (_apps.find(command) != _apps.end()) {

		// Note that argv[argc] always needs to be a nullptr.
		// Therefore add one more entry.
		const char *arg[words.size() + 1];

		for (unsigned i = 0; i < words.size(); ++i) {
			arg[i] = (char *)words[i].c_str();
		}

		// Explicitly set this nullptr.
		arg[words.size()] = nullptr;

		int retval = _apps[command](words.size(), (char **)arg);

		return retval;

	} else if (command == "help") {
		list_builtins(_apps);
		return 0;

	} else if (command.length() == 0 || command[0] == '#') {
		// Do nothing
		return 0;

	} else if (!silently_fail) {
		//std::cout << "Invalid command: " << command << "\ntype 'help' for a list of commands" << endl;
		PX4_INFO_RAW("Invalid command: %s\ntype 'help' for a list of commands\n", command.c_str());
		return -1;

	} else {
		return -1;
	}
}

void Pxh::run_remote_pxh(int remote_in_fd, int remote_out_fd)
{
	Server::CmdThreadSpecificData *thread_data_ptr;

	pthread_key_t _key = Server::get_pthread_key();

	if ((thread_data_ptr = (Server::CmdThreadSpecificData *)pthread_getspecific(_key)) == nullptr) {
		thread_data_ptr = new Server::CmdThreadSpecificData;
		thread_data_ptr->thread_stdout = fdopen(remote_out_fd, "w");
		thread_data_ptr->thread_stdin = fdopen(remote_in_fd, "r");
		thread_data_ptr->is_atty = 0;

		(void)pthread_setspecific(_key, (void *)thread_data_ptr);

	} else {
		thread_data_ptr->thread_stdout = fdopen(remote_out_fd, "w");
		thread_data_ptr->thread_stdin = fdopen(remote_in_fd, "r");
		thread_data_ptr->is_atty = 0;
	}

	setvbuf(thread_data_ptr->thread_stdout, nullptr, _IOLBF, 4096);
	fflush(thread_data_ptr->thread_stdout);

	linenoiseState linenoise_state = nullptr;
	char buf[1024];
	linenoiseConfig cfg = {
		.fd_in = remote_in_fd,
		.fd_out = remote_out_fd,
		.fd_tty = -1,
		.buf = buf,
		.buf_len = sizeof(buf),
	};
	linenoiseCreateState(&linenoise_state, &cfg);
	linenoiseSetCompletionCallback(linenoise_state, completion);
	linenoiseSetHintsCallback(linenoise_state, hints);
	linenoiseHistorySetMaxLen(linenoise_state, 128);

	int cmd_retcode = 0;

	while (!_should_exit) {
		char *line = nullptr;

		char prompt[32];

		if (cmd_retcode == 0) {
			snprintf(prompt, sizeof(prompt), "pxh> ");

		} else {
			snprintf(prompt, sizeof(prompt), "(%d) pxh> ", cmd_retcode);
		}

		linenoiseEditStart(linenoise_state, prompt);

		for (;;) {
			struct pollfd fds[] = {
				{
					.fd = remote_in_fd,
					.events = POLLIN,
				}
			};
			int retval = poll(fds, sizeof(fds) / sizeof(fds[0]), -1);

			if (retval == -1) {
				if (errno == EINTR && _should_exit) {
					break;
				}

				perror("poll()");
				exit(1);

			} else if (retval == 0) {
				// Timeout occurred
				linenoiseHide(linenoise_state);
				printf("poll timeout\n");
				linenoiseShow(linenoise_state);

			} else {
				if (fds[0].revents & POLLIN) {
					line = linenoiseEditFeed(linenoise_state);

					if (line != linenoiseEditMore) {
						break;
					}
				}
			}
		}

		linenoiseEditStop(linenoise_state);
		dprintf(remote_out_fd, "%s\n", line);

		if (_should_exit && line == nullptr) {
			// case when px4 is terminated by signals sent by other process/kernel
			break;
		}

		if (line == nullptr) {
			// case when ctrl-c is captured by console (which is not propagated as signal)
			break;
		}

		if (line[0] != '\0') {
			linenoiseHistoryAdd(linenoise_state, line);
		}

		cmd_retcode = process_line(std::string(line), false);

		free(line);
	}

	linenoiseDeleteState(linenoise_state);
}

void Pxh::run_pxh()
{
	// Only the local_terminal needed for static calls
	_instance = this;

	// populate app list ahead, to build TAB completion
	if (_apps.empty()) {
		init_app_map(_apps);
	}

	// backup and replace original stdout to avoid output collision
	int orig_stdout = dup(STDOUT_FILENO);
	// create new pipe and let it be the new stdout, only for background outputs from other threads,
	// which requires special async output logic
	int piped_stdout[2];
	int ret = pipe(piped_stdout);

	if (ret == 0) {}

	dup2(piped_stdout[1], STDOUT_FILENO);
	fcntl(piped_stdout[0], F_SETFL, fcntl(piped_stdout[0], F_GETFL) | O_NONBLOCK);

	setvbuf(stdout, nullptr, _IOLBF, 4096);
	fflush(stdout);

	linenoiseState linenoise_state = nullptr;
	char buf[1024];
	linenoiseConfig cfg = {
		.fd_in = 0,
		.fd_out = orig_stdout,
		.fd_tty = -1,
		.buf = buf,
		.buf_len = sizeof(buf),
	};
	linenoiseCreateState(&linenoise_state, &cfg);
	linenoiseSetCompletionCallback(linenoise_state, completion);
	linenoiseSetHintsCallback(linenoise_state, hints);
	// TODO: enable history loading, once there is proper way to configure file location
	//linenoiseHistoryLoad(linenoise_state, "history.txt");
	linenoiseHistorySetMaxLen(linenoise_state, 128);

	int cmd_retcode = 0;

	for (;;) {
		char *line = nullptr;

		char prompt[32];

		if (cmd_retcode == 0) {
			snprintf(prompt, sizeof(prompt), "pxh> ");

		} else {
			snprintf(prompt, sizeof(prompt), "(%d) pxh> ", cmd_retcode);
		}

		// change to fake stdout right before raw mode is deployed onto real stdout
		fsync(STDOUT_FILENO);
		dup2(piped_stdout[1], STDOUT_FILENO);
		linenoiseEditStart(linenoise_state, prompt);

		for (;;) {
			struct pollfd fds[] = {
				{
					.fd = 0,
					.events = POLLIN,
				},
				{
					.fd = piped_stdout[0],
					.events = POLLIN,
				}
			};
			int retval = poll(fds, sizeof(fds) / sizeof(fds[0]), -1);

			if (retval == -1) {
				if (errno == EINTR && _should_exit) {
					break;
				}

				perror("poll()");
				exit(1);

			} else if (retval == 0) {
				// Timeout occurred
				linenoiseHide(linenoise_state);
				printf("poll timeout\n");
				linenoiseShow(linenoise_state);

			} else {
				if (fds[1].revents & POLLIN) {
					linenoiseHide(linenoise_state);

					// always copy amount of bytes blindly
					for (;;) {
						char tmpbuf[256];
						ssize_t readlen = read(piped_stdout[0], tmpbuf, sizeof(tmpbuf));

						if (readlen == 0 || (readlen == -1 && errno == EAGAIN)) {
							break;

						} else if (readlen == -1) {
							dprintf(2, "ERROR: read() failed: %s\n", strerror(errno));
						}

						ssize_t writelen = write(orig_stdout, tmpbuf, readlen);

						if (readlen != writelen) {
							dprintf(2, "ERROR: write() returned %zd, errno: %s\n", writelen, strerror(errno));
						}

						fsync(orig_stdout);
					}

					linenoiseShow(linenoise_state);
					continue;
				}

				if (fds[0].revents & POLLIN) {
					line = linenoiseEditFeed(linenoise_state);

					if (line != linenoiseEditMore) {
						break;
					}
				}
			}
		}

		// restore original stdout, with raw mode disabled
		fsync(STDOUT_FILENO);
		linenoiseEditStop(linenoise_state);
		dup2(orig_stdout, STDOUT_FILENO);

		if (_should_exit && line == nullptr) {
			// case when px4 is terminated by signals sent by other process/kernel
			break;
		}

		if (line == nullptr) {
			// case when ctrl-c is captured by console (which is not propagated as signal)
			kill(0, SIGINT);    // TODO: this should avoid. sitl requires this one to kill gazebo
			break;
		}

		if (line[0] != '\0') {
			linenoiseHistoryAdd(linenoise_state, line);
			// TODO: save history file
			//linenoiseHistorySave("history.txt");
		}

		cmd_retcode = process_line(std::string(line), false);

		free(line);
	}

	linenoiseDeleteState(linenoise_state);
	dup2(orig_stdout, STDOUT_FILENO);
	close(piped_stdout[1]); // [0] is closed by dup2()
	close(orig_stdout);
}

void Pxh::completion(const char *buf, linenoiseCompletions lc)
{
	// pick the first word
	std::stringstream line(buf);
	std::string cmd;
	line >> cmd;

	if (cmd.empty()) {
		for (auto it = _apps.begin(); it != _apps.end();  ++it) {
			linenoiseAddCompletion(lc, it->first.c_str());
		}

	} else {
		for (auto it = _apps.begin(); it != _apps.end();  ++it) {
			if (it->first.compare(0, cmd.length(), cmd) == 0) {
				linenoiseAddCompletion(lc, it->first.c_str());
			}
		}
	}

	// TODO: support sub-command completion (start/stop/etc. from ModuleBase)
}

char *Pxh::hints(const char *buf, int *color, int *bold)
{
	// TODO: see if any hint applies
	return nullptr;
}

void Pxh::stop()
{
	if (_instance) {
		_instance->_should_exit = true;
	}
}

} // namespace px4_daemon
