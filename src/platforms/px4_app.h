/****************************************************************************
 *
 * Copyright (c) 2015 Mark Charlebois. All rights reserved.
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
 * @file px4_app.h
 *
 * PX4 app template classes, functions and defines. Apps need to call their
 * main function PX4_MAIN.
 */

#pragma once

namespace px4
{

class AppState
{
public:
	~AppState() {}

#if defined(__PX4_ROS)
	AppState() {}

	bool exitRequested() { return !ros::ok(); }
	void requestExit() { ros::shutdown(); }
#else
	AppState() : _exitRequested(false), _isRunning(false) {}

	bool exitRequested() { return _exitRequested; }
	void requestExit() { _exitRequested = true; }

	bool isRunning() { return _isRunning; }
	void setRunning(bool running) { _isRunning = running; }

protected:
	bool _exitRequested;
	bool _isRunning;
#endif
private:
	AppState(const AppState &);
	const AppState &operator=(const AppState &);
};
}

// PX4_MAIN is defined if module.mk sets MODULE_COMMAND
// For ROS and NuttX it is "main" and for Linux it is
// $(MODULE_COMMAND)_app_main since some apps already
// define $(MODULE_COMMAND)_main

// Task/process based build
#if defined(__PX4_ROS) || defined(__PX4_NUTTX)

// Thread based build
#else

#ifdef PX4_MAIN
extern int PX4_MAIN(int argc, char *argv[]);
#endif

#endif

