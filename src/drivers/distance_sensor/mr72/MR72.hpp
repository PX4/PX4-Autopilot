/****************************************************************************
 *
 *   Copyright (C) 2017-2019 Intel Corporation. All rights reserved.
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

#pragma once

#include <termios.h>
#include <drivers/drv_hrt.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <systemlib/mavlink_log.h>

using namespace time_literals;


#define MR72_MEASURE_INTERVAL     10_ms // 10Hz
#define ULANDING_MIN_DISTANCE		0.1f
#define ULANDING_MAX_DISTANCE		100.0f
#define RANDAR_DATA_ABNORMAL_TIMEOUT (0.5 * 1000 * 1000)	/**< RANDAR finder data abnormal timeout */
#define _VAR_WINDOW_SIZE 10		/* window size for calc variance on RANDAR range */
#define _MF_WINDOW_SIZE 4		/* window size for median filter on RANDAR range */


#define NAR15_HEAD1  0XAA  //43690
#define NAR15_HEAD2  0XAA
#define NAR15_END1   0X55  //21845
#define NAR15_END2   0X55
#define UART_BUFFER_SIZE 		14
#define DATA_PAYLOAD_NUM 8




class MR72 : public px4::ScheduledWorkItem
{
public:
	MR72(const char *serial_port, const uint8_t device_orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	~MR72() override;

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


	enum class PARSE_STATE {
		WAITING_FRAME = 0,
		DIGIT_1,
		DIGIT_2,
		MSB_DATA,
		LSB_DATA,
		CHECKSUM
	};

	typedef struct {
		uint16_t data;// bit0-6 actl_mode MR72 is 1;bit 8-9 rollcount;bit10-11 rsvdl;bit 12-15 cfgstatus MR72 is 1.
		uint16_t rsvd2_1;
		uint32_t rsvd2_2;//rsvd2_2 and rsvd2_1,bit48 show rsvd2
	}SensorStatus;

	typedef struct {
		uint8_t nooftarget;//bit 0-7  Number of detected targets
		uint8_t rollcount_rsvd1_1;//bit 8-9 rollcount,bit 10-15 rsvd1_1
		uint16_t rsvd1_2;
		uint32_t rsvd1_3;//rsvd1_1 rsvd1_2 rsvd1_3,bit54 show rsvd1
	}TargerStatus;

	typedef struct {
		uint8_t Index;//The target ID
		uint8_t Rcs;  //Target reflection cross-sectional area
		uint8_t RangeH; // Target range is 8 bits higher .unit 0.01m
		uint8_t RangeL;//Target range is 8 bits lower
		uint8_t Rsvd1;//reserved
		uint8_t VrelH_rsvd2_rollcount;//reserved. bit0-2 verlH,bit 3-5 rsvd2,bit 6-7 rollcount
		uint8_t VrelL;//reserved
		uint8_t SNR;//Signal to noise ratio
	}TargetInfo;

	typedef enum {
		SENSOR_STATUS_L_8_BIT = 0x0A,
		TARGET_STATUS_L_8_BIT = 0x0B,
		TARGET_INFO_L_8_BIT = 0x0C,
		// SENSOR_CONFIGURATION = 0X200,//520//MR72 CONFIG
		// SENSOR_BACK = 0X400,//1024//MR72 BACK
		SENSOR_STATUS = 0X60A,//1546//MR72 SYSTEM STATUS
		TARGET_STATUS = 0X70B,//1803//TARGET OUTPUT STATUS
		TARGET_INFO = 0X70C, //1804,// 0X70C,//TARGET OUTPUT INFO
		MESSAGE_ID_MAX_NUM,
	}MR72_MESSAGE_ID;

	typedef enum {
		HEAD1,
		HEAD2,
		ID1,
		ID2,
		DATA,
		END1,
		END2,

	}PARSR_mr72_STATE;

	typedef  struct {
		uint8_t head;
		uint8_t tail;
		uint8_t dat_cnt;
		uint8_t tx_rx_uart_buf[UART_BUFFER_SIZE];
	}UART_BUF;

	typedef  struct {
		uint8_t start_sequence1;
		uint8_t start_sequence2;
		uint8_t message_id1;
		uint8_t message_id2;
		union {
			SensorStatus sensor_status;
			TargerStatus target_status;
			TargetInfo runinfo;
			uint8_t bytes[DATA_PAYLOAD_NUM];
		} payload;
		uint8_t stop_sequence1;
		uint8_t stop_sequence2;

	} Packet;

	UART_BUF 	_uartbuf = {};
	Packet 	    _packet = {};


	int read_uart_data(int uart_fd, UART_BUF *const uart_buf);
	int parse_uart_data(UART_BUF *const serial_buf, Packet *const packetdata);
	bool public_distance_sensor(uint16_t message_id);
	float _calc_variance(float value);
	float _median_filter(float value);
	int _mf_cycle_counter;
	int _var_cycle_couter;
	float _mf_window[_MF_WINDOW_SIZE] = {};
	float _mf_window_sorted[_MF_WINDOW_SIZE] = {};
	float _variance_window[_VAR_WINDOW_SIZE] = {};
	float _last_value;
	float _valid_orign_distance;
	hrt_abstime _keep_valid_time;

	/**
	 * Reads the data measrurement from serial UART.
	 */
	int collect();

	/**
	 * Opens and configures the UART serial communications port.
	 * @param speed The baudrate (speed) to configure the serial UART port.
	 */
	int open_serial_port(const speed_t speed = B115200);

	void Run() override;

	const char *_serial_port{nullptr};

	PX4Rangefinder _px4_rangefinder;

	int _file_descriptor{-1};



	hrt_abstime _measurement_time{0};

	perf_counter_t _comms_error{perf_alloc(PC_COUNT, MODULE_NAME": comms_error")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": sample")};
};
