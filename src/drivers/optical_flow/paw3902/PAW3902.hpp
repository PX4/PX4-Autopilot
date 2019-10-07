/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file paw3902.cpp
 *
 * Driver for the Pixart PAW3902 optical flow sensor connected via SPI.
 */

#pragma once

#include "PixArt_PAW3902JF_Registers.hpp"

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/device/spi.h>
#include <conversion/rotation.h>
#include <lib/perf/perf_counter.h>
#include <lib/parameters/param.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/optical_flow.h>

/* Configuration Constants */

#if defined PX4_SPI_BUS_EXPANSION   // crazyflie
# define PAW3902_BUS PX4_SPI_BUS_EXPANSION
#elif defined PX4_SPI_BUS_EXTERNAL1   // fmu-v5
# define PAW3902_BUS PX4_SPI_BUS_EXTERNAL1
#elif defined PX4_SPI_BUS_EXTERNAL    // fmu-v4 extspi
# define PAW3902_BUS PX4_SPI_BUS_EXTERNAL
#else
# error "add the required spi bus from board_config.h here"
#endif

#if defined PX4_SPIDEV_EXPANSION_2    // crazyflie flow deck
# define PAW3902_SPIDEV PX4_SPIDEV_EXPANSION_2
#elif defined PX4_SPIDEV_EXTERNAL1_1    // fmu-v5 ext CS1
# define PAW3902_SPIDEV PX4_SPIDEV_EXTERNAL1_1
#elif defined PX4_SPIDEV_EXTERNAL   // fmu-v4 extspi
# define PAW3902_SPIDEV PX4_SPIDEV_EXTERNAL
#else
# error "add the required spi dev from board_config.h here"
#endif

#define PAW3902_SPI_BUS_SPEED (2000000L) // 2MHz

#define DIR_WRITE(a) ((a) | (1 << 7))
#define DIR_READ(a) ((a) & 0x7f)

using namespace time_literals;
using namespace PixArt_PAW3902JF;

// PAW3902JF-TXQT is PixArt Imaging

class PAW3902 : public device::SPI, public px4::ScheduledWorkItem
{
public:
	PAW3902(int bus = PAW3902_BUS, enum Rotation yaw_rotation = ROTATION_NONE);
	virtual ~PAW3902();

	virtual int init();

	void print_info();

	void start();
	void stop();

protected:
	virtual int probe();

private:

	void Run() override;

	uint8_t	registerRead(uint8_t reg);
	void	registerWrite(uint8_t reg, uint8_t data);

	bool reset();

	bool modeBright();
	bool modeLowLight();
	bool modeSuperLowLight();

	bool changeMode(Mode newMode);

	uORB::PublicationMulti<optical_flow_s> _optical_flow_pub{ORB_ID(optical_flow)};

	perf_counter_t	_sample_perf;
	perf_counter_t	_comms_errors;
	perf_counter_t	_dupe_count_perf;

	static constexpr uint64_t _collect_time{15000}; // 15 milliseconds, optical flow data publish rate

	uint64_t	_previous_collect_timestamp{0};
	uint64_t	_flow_dt_sum_usec{0};
	unsigned	_frame_count_since_last{0};

	enum Rotation	_yaw_rotation {ROTATION_NONE};

	int		_flow_sum_x{0};
	int		_flow_sum_y{0};

	Mode		_mode{Mode::LowLight};

};
