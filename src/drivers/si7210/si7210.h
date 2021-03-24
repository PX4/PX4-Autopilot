#ifndef SI7210_HPP_
#define SI7210_HPP_

#include <px4_config.h>
#include <parameters/param.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <getopt.h>
#include <px4_log.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <nuttx/wqueue.h>
#include <systemlib/conversions.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_hall.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>

#include "parameters.h"

using namespace si7210;

#define SI7210_BUS				PX4_I2C_BUS_EXPANSION

#define SI7210_SLAVE_ADDRESS_0   0x30    /* SI7210 I2C address */
#define SI7210_SLAVE_ADDRESS_1   0x31    /* SI7210 I2C address */
#define SI7210_SLAVE_ADDRESS_2   0x32    /* SI7210 I2C address */
#define SI7210_SLAVE_ADDRESS_3   0x33    /* SI7210 I2C address */

#define SI7210_MAX_DATA_RATE     50

#define SI7210_BUS_SPEED                     1000*100

/* This value is set based on Max output data rate value */
#define SI7210_CONVERSION_INTERVAL      (1000000 / 100) /* microseconds */

#define IDCHIPID 						0x01
#define REVID 							0x04

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

class SI7210 : public device::I2C
{
public:
	enum Instance : int8_t {
		INVALID = -1,
		ID_0 = 0,
		ID_1 = 1,
		ID_2 = 2,
		ID_3 = 3,
	};

	SI7210(int bus, SI7210::Instance instance, const char *path);
	virtual ~SI7210();

	virtual int       init();
	virtual ssize_t   read(struct file *filp, char *buffer, size_t buflen);
	virtual int       ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Stop automatic measurement.
	 */
	void              stop();

	/**
	  * Diagnostics - print some basic information about the driver.
	  */
	void              print_info();

protected:
	virtual int       probe();

private:
	work_s            _work{};

	bool 			  _running;

	/* altitude conversion calibration */
	unsigned         _call_interval;

	si7210_report 	 _report {};
	ringbuffer::RingBuffer  *_reports;

	bool            	_collect_phase;

	orb_advert_t        _hall_topic;
	int					_orb_class_instance;

	perf_counter_t      _sample_perf;
	perf_counter_t      _bad_transfers;
	perf_counter_t      _good_transfers;
	perf_counter_t      _measure_perf;
	perf_counter_t      _comms_errors;
	perf_counter_t      _duplicates;

	bool            	_got_duplicate;

	Instance			_instance;				/**< index of the i2c address and publisher instance */

	int   		    	_params_sub{-1};		/**< notification of parameter updates */

	si7210_report   	_last_report {};        /**< used for info() */

	Parameters			_parameters{};			/**< local copies of interesting parameters */
	ParameterHandles	_parameter_handles{};	/**< handles for interesting parameters */

	/**
	 * Start automatic measurement.
	 */
	void            start();

	int     		measure(); 			 //start measure
	int     		collect(); 			 //get results and publish
	int				parameters_update(); // update parameters

	static void     cycle_trampoline(void *arg);
	void            cycle(); //main execution

	/**
	 * Read the specified number of bytes from SI7210.
	 *
	 * @param reg       The register to read.
	 * @param data      Pointer to buffer for bytes read.
	 * @param len       Number of bytes to read
	 * @return          OK if the transfer was successful, -errno otherwise.
	 */
	int             get_data(uint8_t reg, uint8_t *data, unsigned len);

	/**
	 * Resets the chip.
	 */
	int             reset();

	/**
	 * Measurement self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int             self_test();

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

	/* do not allow to copy this class due to pointer data members */
	SI7210(const SI7210 &);
	SI7210 operator=(const SI7210 &);
};

#endif /* SI7210_HPP_ */
