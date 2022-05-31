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

extern "C"
{
	#include "sg.h"
}


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
#include <uORB/uORB.h>
#include <uORB/topics/transponder_report.h>
#include <uORB/topics/sensor_gps.h>


using namespace time_literals;


/*********************************************************************************
 * Defines
 *********************************************************************************/

#define SAGETECH_MXS_POLL_RATE     						20_ms
#define SAGETECH_SCALE_FEET_TO_M 						0.3048f
#define SAGETECH_SCALE_KNOTS_TO_M_PER_SEC 				0.514444f
#define SAGETECH_SCALE_M_PER_SEC_TO_KNOTS				1.94384F
#define SAGETECH_SCALE_FT_PER_MIN_TO_M_PER_SEC 			0.00508f
#define ADSB_ALTITUDE_TYPE_PRESSURE_QNH 				0
#define ADSB_ALTITUDE_TYPE_GEOMETRIC					1
#define PAYLOAD_MXS_MAX_SIZE  							255
#define START_BYTE										0xAA
#define SAGETECH_PI										3.14159
#define SAGETECH_USEC_PER_HOUR							3600000000
#define SAGETECH_USEC_PER_MIN							60000000
#define SAGETECH_USEC_PER_SEC							1000000

#define SAGETECH_HFOM_UNKNOWN 							19000.0f
#define SAGETECH_VFOM_UNKNOWN 							151.0f
#define SAGETECH_HPL_UNKNOWN 							38000.0f

/*********************************************************************************
 * Enum definitions
 *********************************************************************************/
typedef enum
{
	startByte,
	msgByte,
	idByte,
	lengthByte,
	payload,
	checksumByte
}parse_state_t;

/*********************************************************************************
 * Struct definitions
 *********************************************************************************/
typedef struct
{
	uint8_t state;
	uint8_t index;
	uint8_t start;
	uint8_t type;
	uint8_t id;
	uint8_t length;
	uint8_t payload[PAYLOAD_MXS_MAX_SIZE];
	uint8_t checksum;
}sagetech_packet_t;
/*********************************************************************************
 * Class definition
 *********************************************************************************/

class MXS : public px4::ScheduledWorkItem ,public device::Device
{
public:
	MXS(const char *serial_port,unsigned baudrate);
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

	void handle_msg(const sagetech_packet_t packet);

	void parse_byte(uint8_t data);

	void handle_svr(sg_svr_t svr);

	void handle_msr(sg_msr_t msr);

	uint8_t determine_emitter(sg_adsb_emitter_t emit);

	void send_gps_msg();

	void send_install_msg();

	void send_op_msg();

	void send_target_req_msg();

	void buff_to_hex(char*out,const uint8_t *buff, int len);

	sg_nacv_t determine_nacv(float velAcc);

	speed_t convert_baudrate(unsigned baud);

	char 				*_serial_port{nullptr};
	unsigned			_baudrate{57600};

	int 				_file_descriptor{-1};

	sagetech_packet_t	_msgIn;

	uint8_t 			_msgId{0};

	uint8_t 			_buffer[128];
	uint8_t 			_buffer_len{0};

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	sensor_gps_s 		_gps;

	hrt_abstime			_last_gps_send{0};

	uORB::PublicationMulti<transponder_report_s> 	_transponder_pub{ORB_ID(transponder_report)};
	//uORB::PublicationMulti<sensor_gps_s>	_report_gps_pos_pub{ORB_ID(sensor_gps)};

};


#endif /* DRIVERS_TRANSPONDER_SAGETECH_MXS_MXS_HPP_ */
