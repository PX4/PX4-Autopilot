/****************************************************************************
 *
 *   Copyright (C) 2016 PX4 Development Team. All rights reserved.
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
 * @file history.h
 *
 * Simple command history for the PX4 shell (pxh).
 *
 * This allows to go back in the history of entered commands.
 * Additionally, before going back in the history, the current prompt can get saved.
 *
 * @author Julian Oes <julian@oes.ch>
 */
#pragma once

#include <vector>
#include <string>

namespace px4_daemon
{

class History
{
public:
	/**
	 * Try to append the current line to the history.
	 * Ignore the line if it is empty or duplicate of the
	 * last added one.
	 *
	 * Drop the first entry of the history if we reach the
	 * MAX_HISTORY_SIZE.
	 *
	 * @param line: command line to be added.
	 */
	void try_to_add(const std::string &line);

	/**
	 * After executing a command in the shell, we want to be at
	 * the end of the history again.
	 */
	void reset_to_end();

	/**
	 * If we start scrolling up in the history, we can try to save
	 * the current command line. When we scroll back down, we can
	 * get it out again.
	 *
	 * @param line: line to be saved
	 */
	void try_to_save_current_line(const std::string &line);


	/**
	 * Set the previous (earlier) command from the history.
	 *
	 * @param line: swap to previous line if available.
	 */
	void get_previous(std::string &line);

	/**
	 * Set the next (more recent) command from the history.
	 *
	 * @param line: swap to next line if available, otherwise saved current.
	 */
	void get_next(std::string &line);

	static const unsigned MAX_HISTORY_SIZE = 100;
private:
	std::vector<std::string> _history;
	std::vector<std::string>::iterator _current_history_entry;
	std::string _current_line;
};

} // namespace px4_daemon

