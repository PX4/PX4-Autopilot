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

/**
 * @file RM3100.hpp
 *
 * Driver for the RM3100 magnetometer connected via I2C.
 *
 * Author Ryan Johnston
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_workqueue.h>

#include <perf/perf_counter.h>

#include <drivers/device/i2c.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_device.h>

#include <uORB/uORB.h>

#include <float.h>
#include <getopt.h>
#include <lib/conversion/rotation.h>

/*
 * RM3100 internal constants and data structures.
 */


#define RM3100_CONVERSION_INTERVAL	(1000000 / 100)

#define RM3100_BUS_I2C_ADDR		0x20
#define RM3100_DEFAULT_BUS_SPEED	400000

/*
 * FSR:
 *   x, y: +- 800 µT
 *   z:    +- 800 µT
 *
 *
 */
#define RM3100_RESOLUTION	 0.0065

static const int32_t RM3100_MAX_VAL_XY	= (800 / RM3100_RESOLUTION) + 1;
static const int32_t RM3100_MIN_VAL_XY	= -RM3100_MAX_VAL_XY;
static const int32_t RM3100_MAX_VAL_Z  = (800 / RM3100_RESOLUTION) + 1;
static const int32_t RM3100_MIN_VAL_Z  = -RM3100_MAX_VAL_Z;

/* Hardware definitions */

//#define ADDR_WAI                0		/* WAI means 'Who Am I'*/
//# define WAI_EXPECTED_VALUE     0xE

/*#define ADDR_STAT1              0x02
# define STAT1_DRDY_SHFITS      0x0
# define STAT1_DRDY             (1 << STAT1_DRDY_SHFITS)
# define STAT1_DRO_SHFITS       0x1
# define STAT1_DRO              (1 << STAT1_DRO_SHFITS)
*/

#define ADDR_DATA_OUT_X_LSB     0x05
#define ADDR_DATA_OUT_X_MSB     0x04
#define ADDR_DATA_OUT_Y_LSB     0x07
#define ADDR_DATA_OUT_Y_MSB     0x06
#define ADDR_DATA_OUT_Z_LSB     0x09
#define ADDR_DATA_OUT_Z_MSB     0x08

/*#define ADDR_STAT2              0x09
# define STAT2_INT_SHFITS       3
# define STAT2_INT              (1 << STAT2_INT_SHFITS)
*/

# define ADDR_CTRL1              0x00
//# define CTRL1_MODE_SHFITS      0
//# define CTRL1_MODE_STDBY       (0 << CTRL1_MODE_SHFITS)
# define CTRL1_MODE_SINGLE      0x70

# define ADDR_CTRL2              0x34
//# define CTRL2_SRST_SHFITS      0   /* Begin POR (auto cleared) */
//# define CTRL2_SRST             (1 << CTRL2_SRST_SHFITS)
//# define CTRL2_DRP_SHIFTS       2
//# define CTRL2_DRP              (1 << CTRL2_DRP_SHIFTS)
//# define CTRL2_DREN_SHIFTS      3
//# define CTRL2_DREN             (1 << CTRL2_DREN_SHIFTS)

#define ADDR_CTRL3				0x0B
# define CTRL3_SAMPLEAVG_16		0x96	/* Sample Averaging 16 */
//# define CTRL3_SAMPLEAVG_8		0x1b	/* Sample Averaging 8 */
//# define CTRL3_SAMPLEAVG_4		0x12	/* Sample Averaging 4 */
//# define CTRL3_SAMPLEAVG_2		0x09	/* Sample Averaging 2 */

//#define ADDR_CTRL4				0x42
//# define CTRL4_SRPD				0xC0	/* Set Reset Pulse Duration */

#define ADDR_STR                0x33
# define STR_SELF_TEST_SHFITS   8
# define STR_SELF_TEST_ON       (1 << STR_SELF_TEST_SHFITS)
//# define STR_SELF_TEST_OFF      (0 << STR_SELF_TEST_SHFITS)

/*#define ADDR_Y11_Low			0x9c
#define ADDR_Y11_High			0x9d
#define ADDR_Y12_Low			0x9e
#define ADDR_Y12_High			0x9f
#define ADDR_Y13_Low			0xa0
#define ADDR_Y13_High			0xa1
#define ADDR_Y21_Low			0xa2
#define ADDR_Y21_High			0xa3
#define ADDR_Y22_Low			0xa4
#define ADDR_Y22_High			0xa5
#define ADDR_Y23_Low			0xa6
#define ADDR_Y23_High			0xa7
#define ADDR_Y31_Low			0xa8
#define ADDR_Y31_High			0xa9
#define ADDR_Y32_Low			0xaa
#define ADDR_Y32_High			0xab
#define ADDR_Y33_Low			0xac
#define ADDR_Y33_High			0xad
*/

//#define ADDR_TEMPL              0x1c
//#define ADDR_TEMPH              0x1d

enum RM3100_BUS {
	RM3100_BUS_ALL           = 0,
	RM3100_BUS_I2C_EXTERNAL  = 1,
	RM3100_BUS_I2C_EXTERNAL1 = 2,
	RM3100_BUS_I2C_EXTERNAL2 = 3,
	RM3100_BUS_I2C_EXTERNAL3 = 4,
	RM3100_BUS_I2C_INTERNAL  = 5,
};

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class RM3100 : public device::I2C
{
public:
	RM3100(int bus_number, int address, const char *path, enum Rotation rotation);
	virtual ~RM3100();

	virtual int     init();

	virtual ssize_t     read(struct file *filp, char *buffer, size_t buflen);
	virtual int         ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void            print_info();

//protected:
//	virtual int probe();

private:
	work_s          _work{};
	unsigned        _measure_ticks{0};

	ringbuffer::RingBuffer  *_reports{nullptr};

	mag_calibration_s    _scale {};

	float           _range_scale{0.003f};
	bool        _collect_phase{false};
	int         _class_instance{-1};
	int         _orb_class_instance{-1};

	orb_advert_t        _mag_topic{nullptr};

	perf_counter_t      _sample_perf;
	perf_counter_t      _comms_errors;
	perf_counter_t      _range_errors;
	perf_counter_t      _conf_errors;

	/* status reporting */
	bool            _sensor_ok{false};         /**< sensor was found and reports ok */
	bool            _calibrated{false};        /**< the calibration is valid */
	bool			_ctl_reg_mismatch{false};	/**< control register value mismatch after checking */

	enum Rotation       _rotation;

	sensor_mag_s	_last_report{};           /**< used for info() */

	uint8_t 		_ctl3_reg{0};
	uint8_t			_ctl4_reg{0};

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void		start();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void        stop();

	/**
	 * Reset the device
	 */
	int         reset();

	/**
	 * Perform the on-sensor scale calibration routine.
	 *
	 * @note The sensor will continue to provide measurements, these
	 *   will however reflect the uncalibrated sensor state until
	 *   the calibration routine has been completed.
	 *
	 * @param enable set to 1 to enable self-test strap, 0 to disable
	 */
	int         calibrate(struct file *filp, unsigned enable);

	/**
	 * check the sensor configuration.
	 *
	 * checks that the config of the sensor is correctly set, to
	 * cope with communication errors causing the configuration to
	 * change
	 */
	void            check_conf(void);

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 */
	void            cycle();

	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg       Instance pointer for the driver that is polling.
	 */
	static void     cycle_trampoline(void *arg);

	/**
	 * Write a register.
	 *
	 * @param reg       The register to write.
	 * @param val       The value to write.
	 * @return      OK on write success.
	 */
	int         write_reg(uint8_t reg, uint8_t val);

	/**
	 * Write to a register block.
	 *
	 * @param address   The register address to write to.
	 * @param data      The buffer to write from.
	 * @param count     The number of bytes to write.
	 * @return      OK on write success.
	 */
	int     write(unsigned address, void *data, unsigned count);

	/**
	 * Read a register.
	 *
	 * @param reg       The register to read.
	 * @param val       The value read.
	 * @return      OK on read success.
	 */
	int         read_reg(uint8_t reg, uint8_t &val);

	/**
	 * read register block.
	 *
	 * @param address   The register address to read from.
	 * @param data      The buffer to read into.
	 * @param count     The number of bytes to read.
	 * @return      OK on write success.
	 */
	int read(unsigned address, void *data, unsigned count);

	/**
	 * Issue a measurement command.
	 *
	 * @return      OK if the measurement command was successful.
	 */
	int         measure();

	/**
	 * Collect the result of the most recent measurement.
	 */
	int         collect();

	/**
	 * Convert a big-endian signed 16-bit value to a float.
	 *
	 * @param in        A signed 16-bit big-endian value.
	 * @return      The floating-point representation of the value.
	 */
	float       meas_to_float(uint8_t in[2]);

	/**
	 * Check the current calibration and update device status
	 *
	 * @return 0 if calibration is ok, 1 else
	 */
	int         check_calibration();

	/**
	* Check the current scale calibration
	*
	* @return 0 if scale calibration is ok, 1 else
	*/
	int         check_scale();

	/**
	* Check the current offset calibration
	*
	* @return 0 if offset calibration is ok, 1 else
	*/
	int         check_offset();

	/**
	* Place the device in self test mode
	*
	* @return 0 if mode is set, 1 else
	*/
	int         set_selftest(unsigned enable);

	/* this class has pointer data members, do not allow copying it */
	RM3100(const RM3100 &);
	RM3100 operator=(const RM3100 &);
};
