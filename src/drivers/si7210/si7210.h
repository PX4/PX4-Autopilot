/****************************************************************************
 *
 *   Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
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
 * @file si7210.h
 * @author: Amir Melzer  <amir.melzer@mavt.ethz.ch>
 *
 * Driver for the SI7210 connected via I2C.
 */

#ifndef SI7210_HPP_
#define SI7210_HPP_

#include <drivers/drv_hall.h>
#include <drivers/vane/vane.h>
#include <math.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/topics/sensor_hall.h>

#define I2C_ADDRESS_SI7210   0x30    /* Default SI7210 I2C address */

#define SI7210_MAX_DATA_RATE     50

#define SI7210_BUS_SPEED                     1000*100

/* This value is set based on Max output data rate value */
#define SI7210_CONVERSION_INTERVAL      (1000000 / 100) /* microseconds */

#define IDCHIPID 					   0x01
#define REVID						   0x04

#define ARAUTOINC_ARAUTOINC_MASK       0x01
#define OTP_CTRL_OPT_BUSY_MASK         0x01
#define OTP_CTRL_OPT_READ_EN_MASK      0x02
#define POWER_CTRL_SLEEP_MASK          0x01
#define POWER_CTRL_STOP_MASK           0x02
#define POWER_CTRL_ONEBURST_MASK       0x04
#define POWER_CTRL_USESTORE_MASK       0x08
#define POWER_CTRL_MEAS_MASK           0x80
#define DSPSIGSEL_MAG_VAL_SEL          0
#define DSPSIGSEL_TEMP_VAL_SEL         1

/** I2C registers for Si72xx */
#define OTP_TEMP_OFFSET  0x1D
#define OTP_TEMP_GAIN    0x1E
#define HREVID           0xC0
#define DSPSIGM          0xC1
#define DSPSIGL          0xC2
#define DSPSIGSEL        0xC3
#define POWER_CTRL       0xC4
#define ARAUTOINC        0xC5
#define CTRL1            0xC6
#define CTRL2            0xC7
#define SLTIME           0xC8
#define CTRL3            0xC9
#define A0               0xCA
#define A1               0xCB
#define A2               0xCC
#define CTRL4            0xCD
#define A3               0xCE
#define A4               0xCF
#define A5               0xD0
#define OTP_ADDR         0xE1
#define OTP_DATA         0xE2
#define OTP_CTRL         0xE3
#define TM_FG            0xE4

/** temperature conv for Si72xx */
#define TEMP_CONV_A2     -3.83e-6F
#define TEMP_CONV_A1     0.16094F
#define TEMP_CONV_A0     -279.80F

#define VDD              3.30F
#define VDD_CONV         -0.222F

/** Magnetic conv for Si72xx */
#define MAG_BIAS         0xC000
#define MAG_CONV         0.00125F

class SI7210 : public Vane, public I2CSPIDriver<SI7210>
{
public:
	SI7210(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address = I2C_ADDRESS_SI7210,
	       bool keep_retrying = false) :
		Vane(bus, bus_frequency, address, SI7210_CONVERSION_INTERVAL),
		I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus, address),
		_keep_retrying{keep_retrying},
		_i2c_address{address}
	{
	}

	virtual ~SI7210() = default;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	void	RunImpl();

	void start();

private:
	enum class State {
		RequireConfig,
		Configuring,
		Running
	};

	int	measure() override { return 0; }
	int	collect() override;
	int	probe() override;
	int	configure();

	bool init_si7210();

	/**
	 * Get registers values
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int				get_regs(uint8_t ptr, uint8_t *regs);

	/**
	 * Set registers values
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int				set_regs(uint8_t ptr, uint8_t value);

	/**
	 * Get measurement values
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int				get_measurement(uint8_t ptr, uint16_t *value);

	/**
	 * Get si7210 data
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int				get_sensor_data(uint8_t otpAddr, int8_t *data);

	/**
	 * Calculate the CRC8 for the sensor payload data
	 */
	bool crc(const uint8_t data[], unsigned size, uint8_t checksum);

	/**
	 * Write a command in Sensirion specific logic
	 */
	int write_command(uint16_t command);

	uint16_t _scale{0};
	const bool _keep_retrying;
	State _state{State::RequireConfig};
	int _i2c_address;
};

#endif /* SI7210_HPP_ */
