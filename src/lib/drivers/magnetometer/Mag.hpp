/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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


#include <mathlib/math/filter/LowPassFilter2p.hpp>

#include <drivers/device/integrator.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/cdev/CDev.hpp>
#include <lib/conversion/rotation.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/uORB.h>


class Mag : public cdev::CDev
{

public:
	Mag(const char *name, device::Device  *interface, uint8_t dev_type);
	~Mag() override;

	int	init() override;
	int	ioctl(cdev::file_t *filp, int cmd, unsigned long arg) override;

	int publish(float x, float y, float z, float scale, Rotation rotation);

	void configure_filter(float sample_freq, float cutoff_freq);

private:
	// Pointer to the communication interface
	const device::Device *_interface;

	mag_calibration_s _cal{};

	orb_advert_t _topic{nullptr};

	device::Device::DeviceId _device_id = {};

	int	_orb_class_instance{-1};

	math::LowPassFilter2p _filter_x{100, 20};
	math::LowPassFilter2p _filter_y{100, 20};
	math::LowPassFilter2p _filter_z{100, 20};
};