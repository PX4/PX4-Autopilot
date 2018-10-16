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

#pragma once

/**
 * @file osd.h
 * @author Daniele Pettenuzzo
 *
 * Driver for the ATXXXX chip on the omnibus fcu connected via SPI.
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <perf/perf_counter.h>
#include <parameters/param.h>
#include <px4_getopt.h>
#include <px4_workqueue.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/spi.h>

#include <uORB/uORB.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

/* Configuration Constants */
#ifdef PX4_SPI_BUS_OSD
#define OSD_BUS PX4_SPI_BUS_OSD
#else
#error "add the required spi bus from board_config.h here"
#endif

#ifdef PX4_SPIDEV_OSD
#define OSD_SPIDEV PX4_SPIDEV_OSD
#else
#error "add the required spi bus from board_config.h here"
#endif

#define OSD_SPI_BUS_SPEED (2000000L) /*  2 MHz  */

#define DIR_READ(a) ((a) | (1 << 7))
#define DIR_WRITE(a) ((a) & 0x7f)

#define OSD_DEVICE_PATH "/dev/osd"

#define OSD_US 1000 /*  1 ms  */
#define OSD_UPDATE_RATE 500000 /*  2 Hz  */
#define OSD_CHARS_PER_ROW	30
#define OSD_ZERO_BYTE 0x00
#define OSD_PAL_TX_MODE 0x40

/* OSD Registers addresses */


#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class OSD : public device::SPI
{
public:
	OSD(int bus = OSD_BUS);

	virtual ~OSD();

	virtual int init();

	virtual ssize_t read(device::file_t *filp, char *buffer, size_t buflen);

	virtual int ioctl(device::file_t *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void print_info();

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void start();

protected:
	virtual int probe();

private:
	work_s _work;

	int _measure_ticks;

	perf_counter_t _sample_perf;
	perf_counter_t _comms_errors;

	param_t _p_tx_mode;
	int32_t _tx_mode;

	int _battery_sub;
	int _local_position_sub;
	int _vehicle_status_sub;

	bool _on;

	// battery
	float _battery_voltage_filtered_v;
	float _battery_discharge_mah;
	bool _battery_valid;

	// altitude
	float _local_position_z;
	bool _local_position_valid;

	// flight time
	uint8_t _arming_state;
	uint64_t _arming_timestamp;

	/**
	* Stop the automatic measurement state machine.
	*/
	void stop();

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void cycle();

	int reset();

	int init_osd();

	int readRegister(unsigned reg, uint8_t *data, unsigned count);
	int writeRegister(unsigned reg, uint8_t data);

	int add_character_to_screen(char c, uint8_t pos_x, uint8_t pos_y);
	int add_battery_symbol(uint8_t pos_x, uint8_t pos_y);
	int add_battery_info(uint8_t pos_x, uint8_t pos_y);
	int add_altitude_symbol(uint8_t pos_x, uint8_t pos_y);
	int add_altitude(uint8_t pos_x, uint8_t pos_y);
	int add_flighttime_symbol(uint8_t pos_x, uint8_t pos_y);
	int add_flighttime(float flight_time, uint8_t pos_x, uint8_t pos_y);

	int enable_screen();
	int disable_screen();

	int update_topics();
	int	update_screen();

	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg		Instance pointer for the driver that is polling.
	*/
	static void cycle_trampoline(void *arg);


};
/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int osd_main(int argc, char *argv[]);
