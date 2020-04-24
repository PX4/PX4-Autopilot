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
 * @file history.cpp
 *
 * @author Julian Oes <julian@oes.ch>
 */

#include <string>
#include <vector>

#include "history.h"

namespace px4_daemon
{

void History::try_to_add(const std::string &line)
{
	// Don't save an empty line.
	if (line.empty()) {
		return;
	}

	// Don't add duplicate entries.
	if (!_history.empty() && line == _history.back()) {
		return;
	}

	if (_history.size() == MAX_HISTORY_SIZE) {
		_history.erase(_history.begin());
	}

	_history.push_back(line);
}

void History::reset_to_end()
{
	_current_history_entry = _history.end();
}

void History::try_to_save_current_line(const std::string &line)
{
	// Don't save what's currently entered line if there is no history
	// entry to switch to.
	if (_history.empty()) {
		return;
	}

	// Don't save the current line if we are already jumping around in
	// the history, and we must have already saved it.
	if (_current_history_entry != _history.end()) {
		return;
	}

	_current_line = line;
}

void History::get_previous(std::string &line)
{
	if (_history.empty()) {
		return;
	}

	if (_current_history_entry == _history.begin()) {
		return;
	}

	_current_history_entry = std::prev(_current_history_entry);
	line = *_current_history_entry;
}

void History::get_next(std::string &line)
{
	if (_history.empty()) {
		return;
	}

	// Already at the end, don't even try to get the next.
	if (_current_history_entry == _history.end()) {
		line = _current_line;
		return;
	}

	_current_history_entry = std::next(_current_history_entry);

	// We might have reached next now, ignore it and use what we saved
	// in the beginning.
	if (_current_history_entry == _history.end()) {
		line = _current_line;
		return;
	}

	line = *_current_history_entry;
}

} // namespace px4_daemon
