/****************************************************************************
 *
 * Copyright (C) 2022 ModalAI, Inc. All rights reserved.
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
 * @file px4_i2c.cpp
 *
 * NuttX-compatible I2C API shim for QuRT/SLPI.
 * Implements px4_i2cbus_initialize/uninitialize and I2C_TRANSFER
 * on top of the QuRT device::I2C callback infrastructure by deriving
 * from device::I2C to access its protected transfer methods.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_arch/micro_hal.h>

#if defined(CONFIG_I2C)

#include <lib/drivers/device/qurt/I2C.hpp>

/**
 * Minimal derived class that exposes the protected I2C::transfer()
 * and I2C::set_device_address() for use by the compatibility shim.
 */
class I2CShim : public device::I2C
{
public:
	I2CShim(int bus) :
		I2C(0, "i2c_shim", bus, 0x01, 100000)
	{}

	int do_transfer(uint8_t addr, const uint8_t *send, unsigned send_len,
			uint8_t *recv, unsigned recv_len)
	{
		set_device_address(addr, false);
		return transfer(send, send_len, recv, recv_len);
	}
};

struct i2c_master_s {
	I2CShim *shim;
};

struct i2c_master_s *px4_i2cbus_initialize(int bus)
{
	auto *shim = new I2CShim(bus);

	if (shim == nullptr) {
		return nullptr;
	}

	if (shim->init() != PX4_OK) {
		delete shim;
		return nullptr;
	}

	auto *dev = new i2c_master_s;

	if (dev == nullptr) {
		delete shim;
		return nullptr;
	}

	dev->shim = shim;
	return dev;
}

int px4_i2cbus_uninitialize(struct i2c_master_s *dev)
{
	if (dev != nullptr) {
		delete dev->shim;
		delete dev;
	}

	return PX4_OK;
}

int px4_qurt_i2c_transfer(struct i2c_master_s *dev, struct i2c_msg_s *msgs, unsigned count)
{
	if (dev == nullptr || dev->shim == nullptr || msgs == nullptr || count == 0) {
		return PX4_ERROR;
	}

	unsigned i = 0;

	while (i < count) {
		struct i2c_msg_s *msg = &msgs[i];

		// Check for write+read pair to the same address (common i2cdetect pattern)
		if ((i + 1 < count) &&
		    !(msg->flags & I2C_M_READ) &&
		    (msgs[i + 1].flags & I2C_M_READ) &&
		    (msg->addr == msgs[i + 1].addr)) {

			int ret = dev->shim->do_transfer(msg->addr,
							 msg->buffer, msg->length,
							 msgs[i + 1].buffer, msgs[i + 1].length);

			if (ret != PX4_OK) {
				return ret;
			}

			i += 2;

		} else if (msg->flags & I2C_M_READ) {
			// Single read
			int ret = dev->shim->do_transfer(msg->addr,
							 nullptr, 0,
							 msg->buffer, msg->length);

			if (ret != PX4_OK) {
				return ret;
			}

			i++;

		} else {
			// Single write
			int ret = dev->shim->do_transfer(msg->addr,
							 msg->buffer, msg->length,
							 nullptr, 0);

			if (ret != PX4_OK) {
				return ret;
			}

			i++;
		}
	}

	return PX4_OK;
}

#endif /* CONFIG_I2C */
