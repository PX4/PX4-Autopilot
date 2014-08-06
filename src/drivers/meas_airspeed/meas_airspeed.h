/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file meas_airspeed.h
 * @author Lorenz Meier
 * @author Sarthak Kaingade
 * @author Simon Wilks
 * @author Thomas Gubler
 *
 * Driver for the MEAS Spec series connected via I2C.
 *
 * Supported sensors:
 *
 *    - MS4525DO (http://www.meas-spec.com/downloads/MS4525DO.pdf)
 *    - untested: MS5525DSO (http://www.meas-spec.com/downloads/MS5525DSO.pdf)
 *
 * Interface application notes:
 *
 *    - Interfacing to MEAS Digital Pressure Modules (http://www.meas-spec.com/downloads/Interfacing_to_MEAS_Digital_Pressure_Modules.pdf)
 */

#pragma once

#include <nuttx/config.h>

#include <drivers/device/i2c.h>

#include <board_config.h>

#include <mathlib/math/filter/LowPassFilter2p.hpp>

#include <drivers/drv_airspeed.h>

#include <uORB/uORB.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/system_power.h>

#include <drivers/airspeed/airspeed.h>

/* I2C bus address is 1010001x */
#define I2C_ADDRESS_MS4525DO	0x28	/**< 7-bit address. Depends on the order code (this is for code "I") */
#define PATH_MS4525		"/dev/ms4525"
/* The MS5525DSO address is 111011Cx, where C is the complementary value of the pin CSB */
#define I2C_ADDRESS_MS5525DSO	0x77	//0x77/* 7-bit address, addr. pin pulled low */
#define PATH_MS5525		"/dev/ms5525"

/* Register address */
#define ADDR_READ_MR			0x00	/* write to this address to start conversion */

/* Measurement rate is 100Hz */
#define MEAS_RATE 100
#define MEAS_DRIVER_FILTER_FREQ 1.2f
#define CONVERSION_INTERVAL	(1000000 / MEAS_RATE)	/* microseconds */

class MEASAirspeed : public Airspeed
{
public:
	MEASAirspeed(int bus, int address = I2C_ADDRESS_MS4525DO, const char *path = PATH_MS4525);

protected:

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	virtual void	cycle();
	virtual int	measure();
	virtual int	collect();

	math::LowPassFilter2p	_filter;

	/**
	 * Correct for 5V rail voltage variations
	 */
	void voltage_correction(float &diff_pres_pa, float &temperature);

	int _t_system_power;
	struct system_power_s system_power;
};
