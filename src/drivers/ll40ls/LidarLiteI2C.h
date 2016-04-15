/****************************************************************************
 *
 *   Copyright (c) 2014, 2015 PX4 Development Team. All rights reserved.
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
 * @file LidarLiteI2C.h
 * @author Allyson Kreft
 *
 * Driver for the PulsedLight Lidar-Lite range finders connected via I2C.
 */

#pragma once

#include "LidarLite.h"

#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <systemlib/perf_counter.h>

#include <drivers/device/i2c.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>


/* Configuration Constants */
#define LL40LS_BUS          PX4_I2C_BUS_EXPANSION
#define LL40LS_BASEADDR     0x62 /* 7-bit address */
#define LL40LS_BASEADDR_OLD     0x42 /* previous 7-bit address */

/* LL40LS Registers addresses */

#define LL40LS_MEASURE_REG	0x00        /* Measure range register */
#define LL40LS_DISTHIGH_REG	0x0F        /* High byte of distance register, auto increment */
#define LL40LS_COUNT		0x11
#define LL40LS_HW_VERSION	0x41
#define LL40LS_INTERVAL		0x45
#define LL40LS_SW_VERSION	0x4f

// bit values
#define LL40LS_MSRREG_RESET	0x00        /* reset to power on defaults */
#define LL40LS_AUTO_INCREMENT	0x80
#define LL40LS_COUNT_CONTINUOUS	0xff
#define LL40LS_MSRREG_ACQUIRE	0x04        /* Value to initiate a measurement, varies based on sensor revision */

class LidarLiteI2C : public LidarLite, public device::I2C
{
public:
	LidarLiteI2C(int bus, const char *path, int address = LL40LS_BASEADDR);
	virtual ~LidarLiteI2C();

	int         init() override;

	ssize_t     read(struct file *filp, char *buffer, size_t buflen);
	int         ioctl(struct file *filp, int cmd, unsigned long arg) override;

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void print_info() override;

	/**
	 * print registers to console
	 */
	void print_registers() override;

protected:
	int         probe();
	int         read_reg(uint8_t reg, uint8_t &val);
	int         write_reg(uint8_t reg, uint8_t val);

	int                 measure() override;
	int                 reset_sensor();

private:
	work_s              _work;
	ringbuffer::RingBuffer          *_reports;
	bool                _sensor_ok;
	bool                _collect_phase;
	int                 _class_instance;
	int		    _orb_class_instance;

	orb_advert_t        _distance_sensor_topic;

	perf_counter_t      _sample_perf;
	perf_counter_t      _comms_errors;
	perf_counter_t      _buffer_overflows;
	perf_counter_t      _sensor_resets;
	perf_counter_t      _sensor_zero_resets;
	uint16_t        _last_distance;
	uint16_t        _zero_counter;
	uint64_t        _acquire_time_usec;
	volatile bool       _pause_measurements;
	uint8_t		_hw_version;
	uint8_t		_sw_version;
	bool		_v2_hardware;
	bool		_reset_complete;
	uint8_t		_same_value_counter;

	/**< the bus the device is connected to */
	int         _bus;

	/**
	 * LidarLite specific transfer function. This is needed
	 * to avoid a stop transition with SCL high
	 *
	 * @param send		Pointer to bytes to send.
	 * @param send_len	Number of bytes to send.
	 * @param recv		Pointer to buffer for bytes received.
	 * @param recv_len	Number of bytes to receive.
	 * @return		OK if the transfer was successful, -errno
	 *			otherwise.
	 */
	int lidar_transfer(const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len);

	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address    The I2C bus address to probe.
	* @return       True if the device is present.
	*/
	int                 probe_address(uint8_t address);

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void                start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void                stop();

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void                cycle();
	int                 collect();

	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg        Instance pointer for the driver that is polling.
	*/
	static void     cycle_trampoline(void *arg);

private:
	LidarLiteI2C(const LidarLiteI2C &copy) = delete;
	LidarLiteI2C operator=(const LidarLiteI2C &assignment) = delete;
};
