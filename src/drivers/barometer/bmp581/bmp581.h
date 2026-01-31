/****************************************************************************
 *
 *   Copyright (C) 2024 PX4 Development Team. All rights reserved.
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
 * @file bmp581.h
 *
 * Shared defines for the bmp581 driver.
 */
#pragma once

#include <math.h>
#include <drivers/drv_hrt.h>
#include <lib/cdev/CDev.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_baro.h>

#include "board_config.h"

// From https://github.com/boschsensortec/BMP5-Sensor-API/blob/master/bmp5_defs.h

/* BIT SLICE GET AND SET FUNCTIONS */
#define BMP5_GET_BITSLICE(regvar, bitname) \
	((regvar & bitname##_MSK) >> bitname##_POS)

#define BMP5_SET_BITSLICE(regvar, bitname, val) \
	((regvar & ~bitname##_MSK) | \
	 ((val << bitname##_POS) & bitname##_MSK))

#define BMP5_SET_BIT_VAL_0(reg_data, bitname)	(reg_data & ~(bitname##_MSK))

#define BMP5_SET_BITS_POS_0(reg_data, bitname, data) \
	((reg_data & ~(bitname##_MSK)) | \
	 (data & bitname##_MSK))

#define BMP5_GET_BITS_POS_0(reg_data, bitname)	(reg_data & (bitname##_MSK))

/* Chip id of BMP5 */
#define BMP581_CHIP_ID_PRIM		(0x50)
#define BMP581_CHIP_ID_SEC		(0x51)

#define BMP5_ENABLE                     (0x01)
#define BMP5_DISABLE                    (0x00)

/* Register addresses */
#define BMP5_REG_CHIP_ID_ADDR           (0x01)
#define BMP5_REG_REV_ID_ADDR            (0x02)
#define BMP5_REG_INT_CONFIG_ADDR        (0x14)
#define BMP5_REG_INT_SOURCE_ADDR        (0x15)
#define BMP5_REG_FIFO_SEL_ADDR          (0x18)
#define BMP5_REG_TEMP_DATA_XLSB_ADDR    (0x1D)
#define BMP5_REG_INT_STATUS_ADDR        (0x27)
#define BMP5_REG_STATUS_ADDR            (0x28)
#define BMP5_REG_NVM_ADDR               (0x2B)
#define BMP5_REG_DSP_CONFIG_ADDR        (0x30)
#define BMP5_REG_DSP_IIR_ADDR           (0x31)
#define BMP5_REG_OSR_CONFIG_ADDR        (0x36)
#define BMP5_REG_ODR_CONFIG_ADDR        (0x37)
#define BMP5_REG_CMD_ADDR               (0x7E)

/* Delay definition */
#define BMP5_DELAY_US_SOFT_RESET	(2000)
#define BMP5_DELAY_US_STANDBY           (2500)

/* Soft reset command */
#define BMP5_SOFT_RESET_CMD		(0xB6)

/* Deepstandby enable/disable */
#define BMP5_DEEP_ENABLED		(0)
#define BMP5_DEEP_DISABLED              (1)

/* ODR settings */
#define BMP5_ODR_50_HZ                  (0x0F)
#define BMP5_ODR_05_HZ                  (0x18)
#define BMP5_ODR_01_HZ                  (0x1C)


/* Oversampling for temperature and pressure */
#define BMP5_OVERSAMPLING_1X		(0x00)
#define BMP5_OVERSAMPLING_2X		(0x01)
#define BMP5_OVERSAMPLING_4X		(0x02)
#define BMP5_OVERSAMPLING_8X		(0x03)
#define BMP5_OVERSAMPLING_16X		(0x04)
#define BMP5_OVERSAMPLING_32X		(0x05)
#define BMP5_OVERSAMPLING_64X		(0x06)
#define BMP5_OVERSAMPLING_128X		(0x07)

/* IIR filter for temperature and pressure */
#define BMP5_IIR_FILTER_BYPASS		(0x00)
#define BMP5_IIR_FILTER_COEFF_1		(0x01)

/* Macro is used to bypass both iir_t and iir_p together */
#define BMP5_IIR_BYPASS                 (0xC0)

/* Interrupt configurations */
#define BMP5_INT_MODE_PULSED            (0)
#define BMP5_INT_POL_ACTIVE_HIGH        (1)
#define BMP5_INT_OD_PUSHPULL            (0)

/* NVM and Interrupt status asserted macros */
#define BMP5_INT_ASSERTED_POR_SOFTRESET_COMPLETE  (0x10)
#define BMP5_INT_NVM_RDY                          (0x02)
#define BMP5_INT_NVM_ERR                          (0x04)

/* Interrupt configurations */
#define BMP5_INT_MODE_MSK               (0x01)

#define BMP5_INT_POL_MSK                (0x02)
#define BMP5_INT_POL_POS                (1)

#define BMP5_INT_OD_MSK                 (0x04)
#define BMP5_INT_OD_POS                 (2)

#define BMP5_INT_EN_MSK                 (0x08)
#define BMP5_INT_EN_POS                 (3)

#define BMP5_INT_DRDY_EN_MSK            (0x01)

#define BMP5_INT_FIFO_FULL_EN_MSK       (0x02)
#define BMP5_INT_FIFO_FULL_EN_POS       (1)

#define BMP5_INT_FIFO_THRES_EN_MSK      (0x04)
#define BMP5_INT_FIFO_THRES_EN_POS      (2)

#define BMP5_INT_OOR_PRESS_EN_MSK       (0x08)
#define BMP5_INT_OOR_PRESS_EN_POS       (3)

/* ODR configuration */
#define BMP5_ODR_MSK                    (0x7C)
#define BMP5_ODR_POS                    (2)

/* OSR configurations */
#define BMP5_TEMP_OS_MSK                (0x07)

#define BMP5_PRESS_OS_MSK               (0x38)
#define BMP5_PRESS_OS_POS               (3)

/* Pressure enable */
#define BMP5_PRESS_EN_MSK               (0x40)
#define BMP5_PRESS_EN_POS               (6)

/* IIR configurations */
#define BMP5_SET_IIR_TEMP_MSK           (0x07)

#define BMP5_SET_IIR_PRESS_MSK          (0x38)
#define BMP5_SET_IIR_PRESS_POS          (3)

#define BMP5_SHDW_SET_IIR_TEMP_MSK      (0x08)
#define BMP5_SHDW_SET_IIR_TEMP_POS      (3)

#define BMP5_SHDW_SET_IIR_PRESS_MSK     (0x20)
#define BMP5_SHDW_SET_IIR_PRESS_POS     (5)

#define BMP5_IIR_FLUSH_FORCED_EN_MSK    (0x04)
#define BMP5_IIR_FLUSH_FORCED_EN_POS    (2)

/* Powermode */
#define BMP5_POWERMODE_MSK		(0x03)

#define BMP5_DEEP_DISABLE_MSK		(0x80)
#define BMP5_DEEP_DISABLE_POS		(7)

/* Fifo configurations */
#define BMP5_FIFO_FRAME_SEL_MSK        (0x03)

/* NVM and Interrupt status asserted macros */
#define BMP5_INT_ASSERTED_DRDY         (0x01)

/*!
 * @brief Enumerator for powermode selection
 */
enum bmp5_powermode {
	BMP5_POWERMODE_STANDBY,
	BMP5_POWERMODE_NORMAL,
	BMP5_POWERMODE_FORCED,
	BMP5_POWERMODE_CONTINOUS,
	BMP5_POWERMODE_DEEP_STANDBY
};

/*!
 * @brief Enumerator for interface selection
 */
enum bmp5_intf {
	BMP5_SPI_INTF,
	BMP5_I2C_INTF,
};

/*!
 * @brief BMP5 sensor data structure which comprises of temperature and pressure in floating point with data type as
 *  float for pressure and temperature.
 */
struct bmp5_sensor_data {
	float pressure;
	float temperature;
};

/*
 * BMP581 internal constants and data structures.
 */
class IBMP581
{
public:
	virtual ~IBMP581() = default;

	virtual int init() = 0;

	// read reg value
	virtual uint8_t get_reg(uint8_t addr) = 0;

	// bulk read reg value
	virtual int get_reg_buf(uint8_t addr, uint8_t *buf, uint8_t len) = 0;

	// write reg value
	virtual int set_reg(uint8_t value, uint8_t addr) = 0;

	virtual uint32_t get_device_id() const = 0;

	virtual uint8_t get_device_address() const = 0;
};

class BMP581 : public I2CSPIDriver<BMP581>
{
public:
	BMP581(const I2CSPIDriverConfig &config, IBMP581 *interface);
	virtual ~BMP581();

	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
	static void print_usage();

	virtual int init();

	void print_status();

	void RunImpl();


private:
	static constexpr uint8_t	OVERSAMPLING_TEMPERATURE{BMP5_OVERSAMPLING_2X};
	static constexpr uint8_t	OVERSAMPLING_PRESSURE{BMP5_OVERSAMPLING_32X};
	static constexpr uint8_t	OUTPUT_DATA_RATE{BMP5_ODR_50_HZ};
	static constexpr uint8_t	PRESSURE_ENABLE{BMP5_ENABLE};
	static constexpr uint8_t 	IIR_FILTER_COEFF_TEMPERATURE{BMP5_IIR_FILTER_COEFF_1};
	static constexpr uint8_t 	IIR_FILTER_COEFF_PRESSURE{BMP5_IIR_FILTER_COEFF_1};

	static constexpr uint8_t 	INTERRUPT_MODE{BMP5_INT_MODE_PULSED};
	static constexpr uint8_t	INTERRUPT_POLARITY{BMP5_INT_POL_ACTIVE_HIGH};
	static constexpr uint8_t 	INTERRUPT_DRIVE{BMP5_INT_OD_PUSHPULL};

	uORB::PublicationMulti<sensor_baro_s> _sensor_baro_pub{ORB_ID(sensor_baro)};
	IBMP581 		*_interface{nullptr};

	unsigned		_measure_interval{0};			// interval in microseconds needed to measure

	perf_counter_t 		_sample_perf;
	perf_counter_t 		_measure_perf;
	perf_counter_t 		_comms_errors;

	uint8_t			_chip_id{0};
	uint8_t			_chip_rev_id{0};

	void			start();
	int			measure();
	int		        collect(); // get results and publish

	uint32_t		get_measurement_time();

	int			soft_reset();
	int			set_config();
	uint8_t			get_interrupt_status();
	int			configure_interrupt();
	int			int_source_select();
	uint8_t 		get_power_mode();
	int			set_power_mode(uint8_t power_mode);
	uint8_t			check_deepstandby_mode();
	int			set_standby_mode();
	int			set_deep_standby_mode();
	int			set_osr_odr_press_config();
	int			set_iir_config();
	int			get_sensor_data(bmp5_sensor_data *sensor_data);
};


/* interface factories */
extern IBMP581 *bmp581_spi_interface(uint8_t busnum, uint32_t device, int bus_frequency, spi_mode_e spi_mode);
extern IBMP581 *bmp581_i2c_interface(uint8_t busnum, uint32_t device, int bus_frequency);
extern enum bmp5_intf intf;
