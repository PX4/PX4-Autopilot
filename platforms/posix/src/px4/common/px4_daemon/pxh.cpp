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
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include <stdio.h>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>

#include "pxh.h"

namespace px4_daemon
{

apps_map_type Pxh::_apps = {};
Pxh *Pxh::_instance = nullptr;

Pxh::Pxh()
{
	_history.try_to_add("commander takeoff"); // for convenience
	_history.reset_to_end();
}

Pxh::~Pxh()
{
	if (_local_terminal) {
		tcsetattr(0, TCSANOW, &_orig_term);
		_instance = nullptr;
	}
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

		if (retval) {
			if (!silently_fail) {
				printf("Command '%s' failed, returned %d.\n", command.c_str(), retval);
			}
		}

		return retval;

	} else if (command == "help") {
		list_builtins(_apps);
		return 0;

	} else if (command.length() == 0 || command[0] == '#') {
		// Do nothing
		return 0;

	} else if (!silently_fail) {
		//std::cout << "Invalid command: " << command << "\ntype 'help' for a list of commands" << endl;
		printf("Invalid command: %s\ntype 'help' for a list of commands\n", command.c_str());
		return -1;

	} else {
		return -1;
	}
}

void Pxh::_check_remote_uorb_command(std::string &line)
{

	if (line.empty()) {
		return;
	}

	std::stringstream line_stream(line);
	std::string word;

	line_stream >> word;

	if (word == "uorb") {
		line += " -1";  // Run uorb command only once
	}
}

void Pxh::run_remote_pxh(int remote_in_fd, int remote_out_fd)
{
	std::string mystr;
	int p1[2], pipe_stdout;
	int p2[2], pipe_stderr;
	int backup_stdout_fd = dup(STDOUT_FILENO);
	int backup_stderr_fd = dup(STDERR_FILENO);

	if (pipe(p1) != 0) {
		perror("Remote shell pipe creation failed");
		return;
	}

	if (pipe(p2) != 0) {
		perror("Remote shell pipe 2 creation failed");
		close(p1[0]);
		close(p1[1]);
		return;
	}

	// Create pipe to receive stdout and stderr
	dup2(p1[1], STDOUT_FILENO);
	close(p1[1]);

	dup2(p2[1], STDERR_FILENO);
	close(p2[1]);

	pipe_stdout = p1[0];
	pipe_stderr = p2[0];

	// Set fds for non-blocking operation
	fcntl(pipe_stdout, F_SETFL, fcntl(pipe_stdout, F_GETFL) | O_NONBLOCK);
	fcntl(pipe_stderr, F_SETFL, fcntl(pipe_stderr, F_GETFL) | O_NONBLOCK);
	fcntl(remote_in_fd, F_SETFL, fcntl(remote_in_fd, F_GETFL) | O_NONBLOCK);

	// Check for input on any pipe (i.e. stdout, stderr, or remote_in_fd
	// stdout and stderr will be sent to the local terminal and a copy of the data
	// will be sent over to the mavlink shell through the remote_out_fd.
	//
	// Any data from remote_in_fd will be process as shell commands when an '\n' is received
	while (!_should_exit) {

		struct pollfd fds[3] { {pipe_stderr, POLLIN}, {pipe_stdout, POLLIN}, {remote_in_fd, POLLIN}};

		if (poll(fds, 3, -1) == -1) {
			perror("Mavlink Shell Poll Error");
			break;
		}

		if (fds[0].revents & POLLIN) {

			uint8_t buffer[512];
			size_t len;

			if ((len = read(pipe_stderr, buffer, sizeof(buffer))) <= 0) {
				break; //EOF or ERROR
			}

			// Send all the stderr data to the local terminal as well as the remote shell
			if (write(backup_stderr_fd, buffer, len) <= 0) {
				perror("Remote shell write stdout");
				break;
			}

			if (write(remote_out_fd, buffer, len) <= 0) {
				perror("Remote shell write");
				break;
			}

			// Process all the stderr data first
			continue;
		}

		if (fds[1].revents & POLLIN) {

			uint8_t buffer[512];
			size_t len;

			if ((len = read(pipe_stdout, buffer, sizeof(buffer))) <= 0) {
				break; //EOF or ERROR
			}

			// Send all the stdout data to the local terminal as well as the remote shell
			if (write(backup_stdout_fd, buffer, len) <= 0) {
				perror("Remote shell write stdout");
				break;
			}

			if (write(remote_out_fd, buffer, len) <= 0) {
				perror("Remote shell write");
				break;
			}
		}

		if (fds[2].revents & POLLIN) {

			char c;

			if (read(remote_in_fd, &c, 1) <= 0) {
				break; // EOF or ERROR
			}

			switch (c) {

			case '\n':	// user hit enter
				printf("\n");
				_check_remote_uorb_command(mystr);
				process_line(mystr, false);
				// reset string
				mystr = "";

				_print_prompt();

				break;

			default:	// any other input
				if (c > 3) {
					fprintf(stdout, "%c", c);
					fflush(stdout);
					mystr += (char)c;
				}

				break;
			}
		}
	}

	// Restore stdout and stderr
	dup2(backup_stdout_fd, STDOUT_FILENO);
	dup2(backup_stderr_fd, STDERR_FILENO);
	close(backup_stdout_fd);
	close(backup_stderr_fd);

	close(pipe_stdout);
	close(pipe_stderr);
	close(remote_in_fd);
	close(remote_out_fd);
}

void Pxh::run_pxh()
{
	// Only the local_terminal needed for static calls
	_instance = this;
	_local_terminal = true;
	_setup_term();

	std::string mystr;
	int cursor_position = 0; // position of the cursor from right to left
	// (0: all the way to the right, mystr.length: all the way to the left)

	_print_prompt();

	while (!_should_exit) {

		int c = getchar();
		std::string add_string; // string to add at current cursor position
		bool update_prompt = true;

		switch (c) {
		case EOF:
			break;

		case '\t':
			_tab_completion(mystr);
			break;

		case 127:	// backslash
			if ((int)mystr.length() - cursor_position > 0) {
				mystr.erase(mystr.length() - cursor_position - 1, 1);
			}

			break;

		case '\n':	// user hit enter
			_history.try_to_add(mystr);
			_history.reset_to_end();

			printf("\n");
			process_line(mystr, false);
			// reset string and cursor position
			mystr = "";
			cursor_position = 0;

			update_prompt = false;
			_print_prompt();
			break;

		case '\033': {	// arrow keys
				c = getchar();	// skip first one, does not have the info
				c = getchar();

				if (c == 'A') { // arrow up
					_history.try_to_save_current_line(mystr);
					_history.get_previous(mystr);
					cursor_position = 0; // move cursor to end of line

				} else if (c == 'B') { // arrow down
					_history.get_next(mystr);
					cursor_position = 0; // move cursor to end of line

				} else if (c == 'C') { // arrow right
					if (cursor_position > 0) {
						cursor_position--;
					}

				} else if (c == 'D') { // arrow left
					if (cursor_position < (int)mystr.length()) {
						cursor_position++;
					}

				} else if (c == 'H') { // Home (go to the beginning of the command)
					cursor_position = mystr.length();

				} else if (c == '1') { // Home (go to the beginning of the command, Editing key)
					(void)getchar(); // swallow '~'
					cursor_position = mystr.length();

				} else if (c == 'F') { // End (go to the end of the command)
					cursor_position = 0;

				} else if (c == '4') { // End (go to the end of the command, Editing key)
					(void)getchar(); // swallow '~'
					cursor_position = 0;
				}

				break;
			}

		default:	// any other input
			if (c > 3) {
				add_string += (char)c;

			} else {
				update_prompt = false;
			}

			break;
		}

		if (update_prompt) {
			// reprint prompt with mystr
			mystr.insert(mystr.length() - cursor_position, add_string);
			_clear_line();
			_print_prompt();
			printf("%s", mystr.c_str());

			// Move the cursor to its position
			if (cursor_position > 0) {
				_move_cursor(cursor_position);
			}
		}
	}
}

void Pxh::stop()
{
	if (_instance) {
		_instance->_should_exit = true;
	}
}

void Pxh::_setup_term()
{
	// Make sure we restore terminal at exit.
	tcgetattr(0, &_orig_term);
	atexit(Pxh::_restore_term);

	// change input mode so that we can manage shell
	struct termios term;
	tcgetattr(0, &term);
	term.c_lflag &= ~ICANON;
	term.c_lflag &= ~ECHO;
	tcsetattr(0, TCSANOW, &term);
	setbuf(stdin, nullptr);
}

void Pxh::_restore_term()
{
	if (_instance) {
		tcsetattr(0, TCSANOW, &_instance->_orig_term);
	}
}

void Pxh::_print_prompt()
{
	fflush(stdout);
	printf("pxh> ");
	fflush(stdout);
}

void Pxh::_clear_line()
{
	printf("%c[2K%c", (char)27, (char)13);
}
void Pxh::_move_cursor(int position)
{
	printf("\033[%dD", position);
}

void Pxh::_tab_completion(std::string &mystr)
{
	// parse line and get command
	std::stringstream line(mystr);
	std::string cmd;
	line >> cmd;

	// cmd is empty or white space send a list of available commands
	if (cmd.size() == 0) {

		printf("\n");

		for (auto it = _apps.begin(); it != _apps.end();  ++it) {
			printf("%s ", it->first.c_str());
		}

		printf("\n");
		mystr = "";

	} else {

		// find tab completion matches
		std::vector<std::string> matches;

		for (auto it = _apps.begin(); it != _apps.end();  ++it) {
			if (it->first.compare(0, cmd.size(), cmd) == 0) {
				matches.push_back(it->first);
			}
		}

		if (matches.size() >= 1) {
			// if more than one match print all matches
			if (matches.size() != 1) {
				printf("\n");

				for (const auto &item : matches) {
					printf("%s    ", item.c_str());
				}

				printf("\n");
			}

			// find minimum size match
			size_t min_size = 0;

			for (const auto &item : matches) {
				if (min_size == 0) {
					min_size = item.size();

				} else if (item.size() < min_size) {
					min_size = item.size();
				}
			}

			// parse through elements to find longest match
			std::string longest_match;
			bool done = false;

			for (int i = 0; i < (int)min_size ; ++i) {
				bool first_time = true;

				for (const auto &item : matches) {
					if (first_time) {
						longest_match += item[i];
						first_time = false;

					} else if (longest_match[i] != item[i]) {
						done = true;
						longest_match.pop_back();
						break;
					}
				}

				if (done) { break; }

				mystr = longest_match;
			}
		}

		std::string flags;

		while (line >> cmd) {
			flags += " " + cmd;
		}

		// add flags back in when there is a command match
		if (matches.size() == 1) {
			if (flags.empty()) {
				mystr += " ";

			} else {
				mystr += flags;
			}
		}
	}
}

} // namespace px4_daemon
