/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file UavcanNodeShell.hpp
 * An NSH shell instance driven through a pair of pipes, exposed remotely via the
 * DroneCAN uavcan.protocol.AccessCommandShell service.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <px4_platform_common/tasks.h>

namespace uavcannode
{

class UavcanNodeShell
{
public:
	UavcanNodeShell() = default;
	~UavcanNodeShell();

	/**
	 * Start the shell task. Must only be called once per instance.
	 * @return 0 on success, <0 errno otherwise.
	 */
	int start();

	/**
	 * Write to the shell's stdin.
	 */
	size_t write(const uint8_t *buffer, size_t len);

	/**
	 * Read from the shell's stdout/stderr.
	 */
	size_t read(uint8_t *buffer, size_t len);

	/**
	 * Number of bytes available to read().
	 */
	size_t available();

private:
	int _to_shell_fd = -1;		///< write end of the pipe feeding the shell's stdin
	int _from_shell_fd = -1;	///< read end of the pipe draining the shell's stdout+stderr
	int _shell_fds[2] = { -1, -1 };	///< the shell task's own ends of the two pipes
	px4_task_t _task = -1;

	static int shell_start_thread(int argc, char *argv[]);

	UavcanNodeShell(const UavcanNodeShell &) = delete;
	UavcanNodeShell operator=(const UavcanNodeShell &) = delete;
};

} // namespace uavcannode
