/****************************************************************************
 *
 *   Copyright (C) 2015-2016 PX4 Development Team. All rights reserved.
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

#include <string>
#include <sstream>
#include <vector>
#include <stdio.h>

#include "apps.h"
#include "pxh.h"

namespace px4_daemon
{

Pxh *Pxh::_instance = nullptr;

apps_map_type Pxh::_apps = {};


Pxh::Pxh()
{
	_instance = this;
}

Pxh::~Pxh()
{
	_instance = nullptr;
}

int Pxh::process_line(const std::string &line, bool silently_fail)
{
	if (line.empty()) {
		return 0;
	}

	if (_apps.size() == 0) {
		init_app_map(_apps);
	}

	std::stringstream line_stream(line);
	std::string word;
	std::vector<std::string> words;

	// First arg should be the command.
	while (line_stream >> word) {
		words.push_back(word);
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

	} else if (command.compare("help") == 0) {
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


void Pxh::run_pxh()
{
	_should_exit = false;

	_setup_term();

	std::string mystr = "";

	_print_prompt();

	while (!_should_exit) {

		char c = getchar();

		switch (c) {
		case 127:	// backslash
			if (mystr.length() > 0) {
				mystr.pop_back();
				_clear_line();
				printf("%c", (char)13);
				_print_prompt();
				printf("%s", mystr.c_str());
			}

			break;

		case'\n':	// user hit enter
			_history.try_to_add(mystr);
			_history.reset_to_end();

			printf("\n");
			process_line(mystr, false);
			mystr = "";

			_print_prompt();
			break;

		case '\033': {	// arrow keys
				c = getchar();	// skip first one, does not have the info
				c = getchar();

				// arrow up
				if (c == 'A') {
					_history.try_to_save_current_line(mystr);
					_history.get_previous(mystr);

					// arrow down

				} else if (c == 'B') {
					_history.get_next(mystr);
				}

				_clear_line();
				_print_prompt();
				printf("%s", mystr.c_str());
				break;
			}

		default:	// any other input
			if (c > 3) {
				printf("%c", c);
				mystr += c;
			}

			break;
		}
	}

	return;
}

void Pxh::stop()
{
	_restore_term();

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
	setbuf(stdin, NULL);
}

void Pxh::_restore_term(void)
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

} // namespace px4_daemon
