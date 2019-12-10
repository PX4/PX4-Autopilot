/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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

//
// Created by salimterryli on 2019/12/7.
//

#ifndef PX4_PWMDEVICEBASE_H
#define PX4_PWMDEVICEBASE_H

#include <px4_platform_common/module.h>
#include <cerrno>
#include <unistd.h>

class PWMDeviceBase
{
public:
	virtual ~PWMDeviceBase() = default;
	/*
	 * Called to generate command line usage.
	 * Add new usage description here.
	 */
	static int deviceUsage();
	/*
	 * Called to parse command line arguments.
	 */
	virtual int deviceConfigure(int argc, char **argv) = 0;
	/*
	 * Called when pwm driver is required to init.
	 * @params: Passed from command line.
	 */
	virtual int deviceInit() = 0;

	/*
	 * Called when pwm driver is going to be destroyed.
	 */
	virtual int deviceDeinit() = 0;

	/*
	 * Called when pwm pulses should be updated.
	 */
	virtual int updatePWM(const uint16_t *outputs, unsigned num_outputs) = 0;

	/*
	 * Called to set pwm frequency.
	 */
	virtual int setFreq(int freq) = 0;

protected:

private:

};


#endif //PX4_PWMDEVICEBASE_H
