/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file baro_sim.cpp
 *
 * Simulation interface for barometer
 */

/* XXX trim includes */
#include <px4_config.h>
#include <px4_defines.h>

#include <sys/types.h>
#include <assert.h>
#include <unistd.h>

#include <drivers/device/sim.h>
#include <simulator/simulator.h>
#include "barosim.h"
#include "board_config.h"

BAROSIM_DEV::BAROSIM_DEV() :
	VirtDevObj("BAROSIM_DEV", "/dev/BAROSIM_DEV", BARO_BASE_DEVICE_PATH, 0)
{
}

BAROSIM_DEV::~BAROSIM_DEV()
{
}

int
BAROSIM_DEV::init()
{
	return VirtDevObj::init();
}

ssize_t 
BAROSIM_DEV::devRead(void *data, size_t count)
{
	/* read the most recent measurement */
	uint8_t cmd = 0;
	int ret = transfer(&cmd, 1, static_cast<uint8_t *>(data), count);

	return ret;
}

int
BAROSIM_DEV::devIOCTL(unsigned long operation, unsigned long arg)
{
	int ret;

	switch (operation) {
	case IOCTL_RESET:
		ret = _reset();
		break;

	case IOCTL_MEASURE:
		ret = _doMeasurement(arg);
		break;

	default:
		ret = EINVAL;
	}

	return ret;
}

int
BAROSIM_DEV::_reset()
{
	uint8_t		cmd = ADDR_RESET_CMD;
	int		result;

	/* bump the retry count */
	result = transfer(&cmd, 1, nullptr, 0);

	return result;
}

int
BAROSIM_DEV::_doMeasurement(unsigned addr)
{
	uint8_t cmd = addr;
	return transfer(&cmd, 1, nullptr, 0);
}

int
BAROSIM_DEV::transfer(const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len)
{
	if (send_len == 1 && send[0] == ADDR_RESET_CMD) {
		/* reset command */
		return 0;

	} else if (send_len == 1 && (send[0] == ADDR_CMD_CONVERT_D2 || send[0] == ADDR_CMD_CONVERT_D1)) {
		/* measure command */
		if (send[0] == ADDR_CMD_CONVERT_D2) {
		} else {
		}
		return 0;

	} else if (send[0] == 0 && send_len == 1) {
		/* read requested */
		Simulator *sim = Simulator::getInstance();

		if (sim == NULL) {
			PX4_ERR("Error BAROSIM_DEV::transfer no simulator");
			return -ENODEV;
		}

		PX4_DEBUG("BAROSIM_DEV::transfer getting sample");
		sim->getBaroSample(recv, recv_len);
		return recv_len;

	} else {
		PX4_WARN("BAROSIM_DEV::transfer invalid param %u %u %u", send_len, send[0], recv_len);
		return 1;
	}


	return 0;
}

void BAROSIM_DEV::_measure()
{
}
