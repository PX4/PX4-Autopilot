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

/*
 * MXS.hpp
 *
 * Sagetech MXS transponder driver
 * @author Megan McCormick megan.mccormick@sagetech.com
 */

#ifndef DRIVERS_TRANSPONDER_SAGETECH_MXS_MXS_HPP_
#define DRIVERS_TRANSPONDER_SAGETECH_MXS_MXS_HPP_

#include "Include/sg.h"

#include <termios.h>
#include <stdint.h>

#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/transponder_report.h>


using namespace time_literals;


/*********************************************************************************
 * Defines
 *********************************************************************************/

#define SAGETECH_MXS_POLL_RATE     						10_ms
#define SAGETECH_SCALE_FEET_TO_M 						0.3048f
#define SAGETECH_SCALE_KNOTS_TO_M_PER_SEC 				0.514444f
#define SAGETECH_SCALE_FT_PER_MIN_TO_M_PER_SEC 			0.00508f
#define ADSB_ALTITUDE_TYPE_PRESSURE_QNH 				0
#define ADSB_ALTITUDE_TYPE_GEOMETRIC					1

/*********************************************************************************
 * Class definition
 *********************************************************************************/

class MXS : public px4::ScheduledWorkItem ,public device::Device
{
public:
	MXS(const char *serial_port);
	~MXS() override;

	int init();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_info();

	/**
	 * Initialise the automatic measurement state machine and start it.
	 */
	void start();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void stop();

private:


	void Run() override;

	int collect();

	int open_serial_port();

	void handle_msg();

	void handle_svr(sg_svr_t svr);

	void handle_msr(sg_msr_t msr);

	uint8_t determine_emitter(sg_adsb_emitter_t emit);

	char 			*_serial_port{nullptr};
	unsigned			_baudrate{0};

	int 				_file_descriptor{-1};

	uint8_t 			_buffer[128];
	uint8_t 			_buffer_len{0};

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;

	uORB::PublicationMulti<transponder_report_s> 	_transponder_pub{ORB_ID(transponder_report)};

};



#endif /* DRIVERS_TRANSPONDER_SAGETECH_MXS_MXS_HPP_ */
