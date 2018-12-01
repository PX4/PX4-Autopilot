/****************************************************************************
*
*   Copyright (c) 2016-2017 PX4 Development Team. All rights reserved.
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
 * @file input_test.h
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 */

#pragma once

#include "input.h"

namespace vmount
{

/**
 ** class InputTest
 * Send a single control command, configured via constructor arguments
 */
class InputTest : public InputBase
{
public:

	/**
	 * set to a fixed angle
	 */
	InputTest(float roll_deg, float pitch_deg, float yaw_deg);
	virtual ~InputTest() {}

	/** check whether the test finished, and thus the main thread can quit */
	bool finished();

	virtual int update(unsigned int timeout_ms, ControlData **control_data, bool already_active);

protected:
	virtual int update_impl(unsigned int timeout_ms, ControlData **control_data, bool already_active) { return 0; } //not needed

	virtual int initialize();

	virtual void print_status();

private:
	float _angles[3]; /**< desired angles in [deg] */
};


} /* namespace vmount */
