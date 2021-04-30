/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file VescSerialDevice.hpp
 * @brief PX4 serial motor output device using VescDriver for core protocol
 * @author Matthias Grob <maetugr@gmail.com>
 */

#include "VescDriver/VescDriver.hpp"
#include <drivers/device/device.h>
#include <drivers/drv_mixer.h>
#include <lib/cdev/CDev.hpp>
#include <lib/mixer_module/mixer_module.hpp>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

class VescDevice : public VescWritableInterface, public cdev::CDev, public OutputModuleInterface
{
public:
	VescDevice(const char *port);
	~VescDevice();
	int init();
	void printStatus();
	int ioctl(device::file_t *filp, int cmd, unsigned long arg);

private:
	void Run();
	size_t writeCallback(const uint8_t *buffer, const uint16_t length) override;
	int setBaudrate(const unsigned baudrate);
	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
			   unsigned num_control_groups_updated) override;

	static constexpr size_t READ_BUFFER_SIZE{256};
	static constexpr int DISARM_VALUE = 0;
	static constexpr int MIN_THROTTLE = 100;
	static constexpr int MAX_THROTTLE = 1000;

	char _port[20] {};
	int _serial_fd{-1};
	VescDriver _vesc_driver;
	MixingOutput _mixing_output{4, *this, MixingOutput::SchedulingPolicy::Auto, false, false};
	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
};
