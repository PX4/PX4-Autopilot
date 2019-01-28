/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file fxas21002c.cpp
 * Driver for the NXP FXAS21002C 3-Axis Digital Angular Rate Gyroscope
 * connected via SPI
 */

#include <px4_config.h>
#include <px4_defines.h>

#include <sys/types.h>
#include <sys/stat.h>
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


#include <px4_log.h>

#include <perf/perf_counter.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <drivers/drv_hrt.h>
#include <drivers/device/spi.h>
#include <drivers/drv_gyro.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>

#include <board_config.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>
#include <px4_getopt.h>
#include <systemlib/err.h>
#include <px4_work_queue/ScheduledWorkItem.hpp>

/* SPI protocol address bits */
#define DIR_READ(a)                     ((a) | (1 << 7))
#define DIR_WRITE(a)                    ((a) & 0x7f)
#define swap16(w)                       __builtin_bswap16((w))
#define FXAS21002C_DEVICE_PATH_GYRO     "/dev/fxas21002c_gyro"
#define FXAS21002C_DEVICE_PATH_GYRO_EXT "/dev/fxas21002c_gyro_ext"

#define FXAS21002C_STATUS                0x00
#define FXAS21002C_OUT_X_MSB             0x01
#define FXAS21002C_OUT_X_LSB             0x02
#define FXAS21002C_OUT_Y_MSB             0x03
#define FXAS21002C_OUT_Y_LSB             0x04
#define FXAS21002C_OUT_Z_MSB             0x05
#define FXAS21002C_OUT_Z_LSB             0x06

#define FXAS21002C_DR_STATUS             0x07
#  define DR_STATUS_ZYXOW                (1 << 7)
#  define DR_STATUS_ZOW                  (1 << 6)
#  define DR_STATUS_YOW                  (1 << 5)
#  define DR_STATUS_XOW                  (1 << 4)
#  define DR_STATUS_ZYXDR                (1 << 3)
#  define DR_STATUS_ZDR                  (1 << 2)
#  define DR_STATUS_YDR                  (1 << 1)
#  define DR_STATUS_XDR                  (1 << 0)

#define FXAS21002C_F_STATUS              0x08
#  define F_STATUS_F_OVF                 (1 << 7)
#  define F_STATUS_F_WMKF                (1 << 6)
#  define F_STATUS_F_CNT_SHIFTS          0
#  define F_STATUS_F_CNT_MASK            (0x3f << F_STATUS_F_CNT_SHIFTS)

#define FXAS21002C_F_SETUP               0x09
#  define F_SETUP_F_MODE_SHIFTS          6
#  define F_SETUP_F_MODE_MASK            (0x3 << F_SETUP_F_MODE_SHIFTS)
#  define F_SETUP_F_WMRK_SHIFTS          0
#  define F_SETUP_F_WMRK_MASK            (0x3f << F_SETUP_F_WMRK_SHIFTS)

#define FXAS21002C_F_EVENT               0x0a
#  define F_EVENT_F_EVENT                (1 << 5)
#  define F_EVENT_FE_TIME_SHIFTS         0
#  define F_EVENT_FE_TIME_MASK           (0x1f << F_EVENT_FE_TIME_SHIFTS)

#define FXAS21002C_INT_SRC_FLAG          0x0b
#  define INT_SRC_FLAG_BOOTEND           (1 << 3)
#  define INT_SRC_FLAG_SRC_FIFO          (1 << 2)
#  define INT_SRC_FLAG_SRC_RT            (1 << 1)
#  define INT_SRC_FLAG_SRC_DRDY          (1 << 0)

#define FXAS21002C_WHO_AM_I              0x0c
#define   WHO_AM_I                       0xd7

#define FXAS21002C_CTRL_REG0             0x0d
#  define CTRL_REG0_BW_SHIFTS            6
#  define CTRL_REG0_BW_MASK              (0x3 << CTRL_REG0_BW_SHIFTS)
#  define CTRL_REG0_BW(n)                (((n) & 0x3) << CTRL_REG0_BW_SHIFTS)
#    define CTRL_REG0_BW_HIGH             CTRL_REG0_BW(0)
#    define CTRL_REG0_BW_MED              CTRL_REG0_BW(1)
#    define CTRL_REG0_BW_LOW              CTRL_REG0_BW(2)
#  define CTRL_REG0_SPIW                 (1 << 6)
#  define CTRL_REG0_SEL_SHIFTS           3
#  define CTRL_REG0_SEL_MASK             (0x2 << CTRL_REG0_SEL_SHIFTS)
#  define CTRL_REG0_HPF_EN               (1 << 2)
#  define CTRL_REG0_FS_SHIFTS            0
#  define CTRL_REG0_FS_MASK              (0x3 << CTRL_REG0_FS_SHIFTS)
#  define CTRL_REG0_FS_2000_DPS          (0 << CTRL_REG0_FS_SHIFTS)
#  define CTRL_REG0_FS_1000_DPS          (1 << CTRL_REG0_FS_SHIFTS)
#  define CTRL_REG0_FS_500_DPS           (2 << CTRL_REG0_FS_SHIFTS)
#  define CTRL_REG0_FS_250_DPS           (3 << CTRL_REG0_FS_SHIFTS)

#define FXAS21002C_RT_CFG                0x0e
#  define RT_CFG_ELE                     (1 << 3)
#  define RT_CFG_ZTEFE                   (1 << 2)
#  define RT_CFG_YTEFE                   (1 << 1)
#  define RT_CFG_XTEFE                   (1 << 0)

#define FXAS21002C_RT_SRC                0x0f
#  define RT_SRC_EA                      (1 << 6)
#  define RT_SRC_ZRT                     (1 << 5)
#  define RT_SRC_Z_RT_POL                (1 << 4)
#  define RT_SRC_YRT                     (1 << 3)
#  define RT_SRC_Y_RT_POL                (1 << 2)
#  define RT_SRC_XRT                     (1 << 1)
#  define RT_SRC_X_RT_POL                (1 << 0)

#define FXAS21002C_RT_THS                0x10
#  define RT_THS_DBCNTM                  (1 << 7)
#  define RT_THS_THS_SHIFTS              0
#  define RT_THS_THS_MASK                (0x7f << RT_THS_THS_SHIFTS)

#define FXAS21002C_RT_COUNT              0x11
#define FXAS21002C_TEMP                  0x12

#define FXAS21002C_CTRL_REG1             0x13
#  define CTRL_REG1_RST                  (1 << 6)
#  define CTRL_REG1_ST                   (1 << 5)
#  define CTRL_REG1_DR_SHIFTS             2
#  define CTRL_REG1_DR_MASK               (0x07 << CTRL_REG1_DR_SHIFTS)
#  define CTRL_REG1_DR_12_5               (7 << CTRL_REG1_DR_SHIFTS)
#  define CTRL_REG1_DR_12_5_1             (6 << CTRL_REG1_DR_SHIFTS)
#  define CTRL_REG1_DR_25HZ               (5 << CTRL_REG1_DR_SHIFTS)
#  define CTRL_REG1_DR_50HZ               (4 << CTRL_REG1_DR_SHIFTS)
#  define CTRL_REG1_DR_100HZ              (3 << CTRL_REG1_DR_SHIFTS)
#  define CTRL_REG1_DR_200HZ              (2 << CTRL_REG1_DR_SHIFTS)
#  define CTRL_REG1_DR_400HZ              (1 << CTRL_REG1_DR_SHIFTS)
#  define CTRL_REG1_DR_800HZ              (0 << CTRL_REG1_DR_SHIFTS)
#  define CTRL_REG1_ACTIVE               (1 << 1)
#  define CTRL_REG1_READY                (1 << 0)

#define FXAS21002C_CTRL_REG2             0x14
#  define CTRL_REG2_INT_CFG_FIFO         (1 << 7)
#  define CTRL_REG2_INT_EN_FIFO          (1 << 6)
#  define CTRL_REG2_INT_CFG_RT           (1 << 5)
#  define CTRL_REG2_INT_EN_RT            (1 << 4)
#  define CTRL_REG2_INT_CFG_DRDY         (1 << 3)
#  define CTRL_REG2_INT_EN_DRDY          (1 << 2)
#  define CTRL_REG2_IPOL                 (1 << 1)
#  define CTRL_REG2_PP_OD                (1 << 0)

#define FXAS21002C_CTRL_REG3             0x15
#  define CTRL_REG3_WRAPTOONE            (1 << 3)
#  define CTRL_REG3_EXTCTRLEN            (1 << 2)
#  define CTRL_REG3_FS_DOUBLE            (1 << 0)

#define DEF_REG(r)   {r, #r}

/* default values for this device */
#define FXAS21002C_MAX_RATE              800
#define FXAS21002C_DEFAULT_RATE          FXAS21002C_MAX_RATE
#define FXAS21002C_MAX_OUTPUT_RATE       280
#define FXAS21002C_DEFAULT_RANGE_DPS     2000
#define FXAS21002C_DEFAULT_FILTER_FREQ   30
#define FXAS21002C_TEMP_OFFSET_CELSIUS   40
#define FXAS21002C_DEFAULT_ONCHIP_FILTER_FREQ 	64 // ODR dependant

#define FXAS21002C_MAX_OFFSET			0.45f /**< max offset: 25 degrees/s */

/*
  we set the timer interrupt to run a bit faster than the desired
  sample rate and then throw away duplicates using the data ready bit.
  This time reduction is enough to cope with worst case timing jitter
  due to other timers
  Typical reductions for the MPU6000 is 20% so 20% of 1/800 is 250 us
 */
#define FXAS21002C_TIMER_REDUCTION				250

extern "C" { __EXPORT int fxas21002c_main(int argc, char *argv[]); }

class FXAS21002C : public device::SPI, public px4::ScheduledWorkItem
{
public:
	FXAS21002C(int bus, const char *path, uint32_t device, enum Rotation rotation);
	virtual ~FXAS21002C();

	virtual int init();

	virtual ssize_t read(struct file *filp, char *buffer, size_t buflen);
	virtual int ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_info();

	/**
	 * dump register values
	 */
	void print_registers();

	/**
	 * deliberately trigger an error
	 */
	void test_error();

protected:
	virtual int probe();

private:

	unsigned _call_interval;

	ringbuffer::RingBuffer *_reports;

	struct gyro_calibration_s _gyro_scale;
	float _gyro_range_scale;
	float _gyro_range_rad_s;
	orb_advert_t _gyro_topic;
	int _orb_class_instance;
	int _class_instance;

	unsigned _current_rate;
	unsigned _orientation;
	float _last_temperature;

	unsigned _read;


	perf_counter_t _sample_perf;
	perf_counter_t _errors;
	perf_counter_t _bad_registers;
	perf_counter_t _duplicates;

	uint8_t _register_wait;

	math::LowPassFilter2p _gyro_filter_x;
	math::LowPassFilter2p _gyro_filter_y;
	math::LowPassFilter2p _gyro_filter_z;

	Integrator _gyro_int;

	enum Rotation _rotation;


	/* this is used to support runtime checking of key
	 *configuration registers to detect SPI bus errors and sensor
	 * reset
	 */
#define FXAS21002C_NUM_CHECKED_REGISTERS 6
	static const uint8_t _checked_registers[FXAS21002C_NUM_CHECKED_REGISTERS];
	uint8_t _checked_values[FXAS21002C_NUM_CHECKED_REGISTERS];
	uint8_t _checked_next;

	/**
	 * Start automatic measurement.
	 */
	void start();

	/**
	 * Stop automatic measurement.
	 */
	void stop();

	/**
	 * Reset chip.
	 *
	 * Resets the chip and measurements ranges, but not scale and offset.
	 */
	void reset();

	/**
	 * disable I2C on the chip
	 */
	void disable_i2c();

	/**
	 * Put the chip In stand by
	 */
	void set_standby(int rate, bool standby_true);

	void Run() override;

	/**
	 * check key registers for correct values
	 */
	void check_registers(void);

	/**
	 * Fetch accel measurements from the sensor and update the report ring.
	 */
	void measure();

	/**
	 * Read a register from the FXAS21002C
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t read_reg(unsigned reg);

	/**
	 * Write a register in the FXAS21002C
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void write_reg(unsigned reg, uint8_t value);

	/**
	 * Modify a register in the FXAS21002C
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Write a register in the FXAS21002C, updating _checked_values
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void write_checked_reg(unsigned reg, uint8_t value);

	/**
	 * Set the FXAS21002C measurement range.
	 *
	 * @param max_dps	The measurement range is set to permit reading at least
	 *			this rate in degrees per second.
	 *			Zero selects the maximum supported range.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int set_range(unsigned max_dps);

	/**
	 * Set the FXAS21002C internal sampling frequency.
	 *
	 * @param frequency	The internal sampling frequency is set to not less than
	 *			this value.
	 *			Zero selects the maximum rate supported.
	 * @return		OK if the value can be supported.
	 */
	int set_samplerate(unsigned frequency);

	/**
	 * Set the lowpass filter of the driver
	 *
	 * @param samplerate	The current samplerate
	 * @param frequency	The cutoff frequency for the lowpass filter
	 */
	void set_sw_lowpass_filter(float samplerate, float bandwidth);

	/*
	  set onchip low pass filter frequency
	 */
	void set_onchip_lowpass_filter(int frequency_hz);

	/**
	 * Self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int self_test();


	/* this class cannot be copied */
	FXAS21002C(const FXAS21002C &);
	FXAS21002C operator=(const FXAS21002C &);
};

/*
  list of registers that will be checked in check_registers(). Note
  that ADDR_WHO_AM_I must be first in the list.
 */
const uint8_t FXAS21002C::_checked_registers[FXAS21002C_NUM_CHECKED_REGISTERS] = {
	FXAS21002C_WHO_AM_I,
	FXAS21002C_F_SETUP,
	FXAS21002C_CTRL_REG0,
	FXAS21002C_CTRL_REG1,
	FXAS21002C_CTRL_REG2,
	FXAS21002C_CTRL_REG3,
};


FXAS21002C::FXAS21002C(int bus, const char *path, uint32_t device, enum Rotation rotation) :
	SPI("FXAS21002C", path, bus, device, SPIDEV_MODE0, 2 * 1000 * 1000),
	ScheduledWorkItem(px4::device_bus_to_wq(this->get_device_id())),
	_call_interval(0),
	_reports(nullptr),
	_gyro_scale{},
	_gyro_range_scale(0.0f),
	_gyro_range_rad_s(0.0f),
	_gyro_topic(nullptr),
	_orb_class_instance(-1),
	_class_instance(-1),
	_current_rate(800),
	_orientation(0),
	_last_temperature(0.0f),
	_read(0),
	_sample_perf(perf_alloc(PC_ELAPSED, "fxas21002c_acc_read")),
	_errors(perf_alloc(PC_COUNT, "fxas21002c_err")),
	_bad_registers(perf_alloc(PC_COUNT, "fxas21002c_bad_reg")),
	_duplicates(perf_alloc(PC_COUNT, "fxas21002c_acc_dupe")),
	_register_wait(0),
	_gyro_filter_x(FXAS21002C_DEFAULT_RATE, FXAS21002C_DEFAULT_FILTER_FREQ),
	_gyro_filter_y(FXAS21002C_DEFAULT_RATE, FXAS21002C_DEFAULT_FILTER_FREQ),
	_gyro_filter_z(FXAS21002C_DEFAULT_RATE, FXAS21002C_DEFAULT_FILTER_FREQ),
	_gyro_int(1000000 / FXAS21002C_MAX_OUTPUT_RATE, true),
	_rotation(rotation),
	_checked_values{},
	_checked_next(0)
{
	// enable debug() calls
	//_debug_enabled = false;

	_device_id.devid_s.devtype = DRV_GYR_DEVTYPE_FXAS2100C;


	// default scale factors
	_gyro_scale.x_offset = 0.0f;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0.0f;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0.0f;
	_gyro_scale.z_scale  = 1.0f;

}

FXAS21002C::~FXAS21002C()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(GYRO_BASE_DEVICE_PATH, _class_instance);
	}


	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_errors);
	perf_free(_bad_registers);
	perf_free(_duplicates);
}

int
FXAS21002C::init()
{
	/* do SPI init (and probe) first */
	if (SPI::init() != OK) {
		PX4_ERR("SPI init failed");
		return PX4_ERROR;
	}

	param_t gyro_cut_ph = param_find("IMU_GYRO_CUTOFF");
	float gyro_cut = FXAS21002C_DEFAULT_FILTER_FREQ;

	if (gyro_cut_ph != PARAM_INVALID && param_get(gyro_cut_ph, &gyro_cut) == PX4_OK) {
		set_sw_lowpass_filter(FXAS21002C_DEFAULT_RATE, gyro_cut);

	} else {
		PX4_ERR("IMU_GYRO_CUTOFF param invalid");
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_gyro_s));

	if (_reports == nullptr) {
		return PX4_ERROR;
	}

	reset();

	/* fill report structures */
	measure();

	_class_instance = register_class_devname(GYRO_BASE_DEVICE_PATH);

	/* advertise sensor topic, measure manually to initialize valid report */
	sensor_gyro_s grp;
	_reports->get(&grp);

	/* measurement will have generated a report, publish */
	_gyro_topic = orb_advertise_multi(ORB_ID(sensor_gyro), &grp,
					  &_orb_class_instance, (external()) ? ORB_PRIO_VERY_HIGH : ORB_PRIO_DEFAULT);

	if (_gyro_topic == nullptr) {
		PX4_ERR("ADVERT ERR");
		return PX4_ERROR;
	}

	return PX4_OK;
}


void
FXAS21002C::reset()
{
	/* write 0 0 0 000 00 = 0x00 to CTRL_REG1 to place FXOS21002 in Standby
	 * [6]: RST=0
	 * [5]: ST=0 self test disabled
	 * [4-2]: DR[2-0]=000 for 200Hz ODR
	 * [1-0]: Active=0, Ready=0 for Standby mode
	 */

	write_reg(FXAS21002C_CTRL_REG1, 0);

	/* write 0000 0000 = 0x00 to CTRL_REG0 to configure range and filters
	 * [7-6]: BW[1-0]=00, LPF 64 @ 800Hz ODR
	 *  [5]: SPIW=0 4 wire SPI
	 * [4-3]: SEL[1-0]=00 for 10Hz HPF at 200Hz ODR
	 *  [2]: HPF_EN=0 disable HPF
	 * [1-0]: FS[1-0]=00 for 1600dps (TBD CHANGE TO 2000dps when final trimmed parts available)
	 */

	write_checked_reg(FXAS21002C_CTRL_REG0, CTRL_REG0_BW_LOW | CTRL_REG0_FS_2000_DPS);

	/* write CTRL_REG1 to configure 800Hz ODR and enter Active mode */

	write_checked_reg(FXAS21002C_CTRL_REG1, CTRL_REG1_DR_800HZ | CTRL_REG1_ACTIVE);

	/* Set the default */

	set_samplerate(0);
	set_range(FXAS21002C_DEFAULT_RANGE_DPS);
	set_onchip_lowpass_filter(FXAS21002C_DEFAULT_ONCHIP_FILTER_FREQ);
	_read = 0;
}

int
FXAS21002C::probe()
{
	/* verify that the device is attached and functioning */
	bool success = (read_reg(FXAS21002C_WHO_AM_I) == WHO_AM_I);

	if (success) {
		_checked_values[0] = WHO_AM_I;
		return OK;
	}

	return -EIO;
}

ssize_t
FXAS21002C::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(sensor_gyro_s);
	sensor_gyro_s *gbuf = reinterpret_cast<sensor_gyro_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_call_interval > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the measurement code while we are doing this;
		 * we are careful to avoid racing with it.
		 */
		while (count--) {
			if (_reports->get(gbuf)) {
				ret += sizeof(*gbuf);
				gbuf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement */
	_reports->flush();
	measure();

	/* measurement will have generated a report, copy it out */
	if (_reports->get(gbuf)) {
		ret = sizeof(*gbuf);
	}

	return ret;
}

int
FXAS21002C::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default polling rate */
			case SENSOR_POLLRATE_DEFAULT:
				return ioctl(filp, SENSORIOCSPOLLRATE, FXAS21002C_DEFAULT_RATE);

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_call_interval == 0);

					/* convert hz to hrt interval via microseconds */
					unsigned interval = 1000000 / arg;

					/* check against maximum sane rate */
					if (interval < 1000) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					/* XXX this is a bit shady, but no other way to adjust... */
					_call_interval = interval;

					/* adjust filters */
					float cutoff_freq_hz = _gyro_filter_x.get_cutoff_freq();
					float sample_rate = 1.0e6f / interval;
					set_sw_lowpass_filter(sample_rate, cutoff_freq_hz);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCRESET:
		reset();
		return OK;

	case GYROIOCSSCALE:
		/* copy scale in */
		memcpy(&_gyro_scale, (struct gyro_calibration_s *) arg, sizeof(_gyro_scale));
		return OK;

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

int
FXAS21002C::self_test()
{
	if (_read == 0) {
		return 1;
	}

	return 0;
}

uint8_t
FXAS21002C::read_reg(unsigned reg)
{
	uint8_t cmd[2];

	cmd[0] = DIR_READ(reg);
	cmd[1] = 0;

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

void
FXAS21002C::write_reg(unsigned reg, uint8_t value)
{
	uint8_t cmd[2];

	cmd[0] = DIR_WRITE(reg);
	cmd[1] = value;

	transfer(cmd, nullptr, sizeof(cmd));
}

void
FXAS21002C::write_checked_reg(unsigned reg, uint8_t value)
{
	write_reg(reg, value);

	for (uint8_t i = 0; i < FXAS21002C_NUM_CHECKED_REGISTERS; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
		}
	}
}

void
FXAS21002C::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_checked_reg(reg, val);
}

int
FXAS21002C::set_range(unsigned max_dps)
{
	uint8_t bits = CTRL_REG0_FS_250_DPS;
	float new_range_scale_dps_digit;
	float new_range;

	if (max_dps == 0) {
		max_dps = 2000;
	}

	if (max_dps <= 250) {
		new_range = 250;
		new_range_scale_dps_digit = 7.8125e-3f;
		bits = CTRL_REG0_FS_250_DPS;

	} else if (max_dps <= 500) {
		new_range = 500;
		new_range_scale_dps_digit = 15.625e-3f;
		bits = CTRL_REG0_FS_500_DPS;

	} else if (max_dps <= 1000) {
		new_range = 1000;
		new_range_scale_dps_digit = 31.25e-3f;
		bits = CTRL_REG0_FS_1000_DPS;

	} else if (max_dps <= 2000) {
		new_range = 2000;
		new_range_scale_dps_digit = 62.5e-3f;
		bits = CTRL_REG0_FS_2000_DPS;

	} else {
		return -EINVAL;
	}

	set_standby(_current_rate, true);
	_gyro_range_rad_s = new_range / 180.0f * M_PI_F;
	_gyro_range_scale = new_range_scale_dps_digit / 180.0f * M_PI_F;
	modify_reg(FXAS21002C_CTRL_REG0, CTRL_REG0_FS_MASK, bits);
	set_standby(_current_rate, false);

	return OK;
}

void FXAS21002C::set_standby(int rate, bool standby_true)
{
	uint8_t c = 0;
	uint8_t s = 0;

	if (standby_true) {
		c = CTRL_REG1_ACTIVE | CTRL_REG1_READY;

	} else {
		s = CTRL_REG1_ACTIVE | CTRL_REG1_READY;
	}

	modify_reg(FXAS21002C_CTRL_REG1, c, s);

	// From the data sheet

	int wait_ms = (1000 / rate) + 60 + 1;

	usleep(wait_ms * 1000);
}

int
FXAS21002C::set_samplerate(unsigned frequency)
{
	uint8_t bits = 0;

	unsigned last_rate = _current_rate;

	if (frequency == 0) {
		frequency = FXAS21002C_DEFAULT_RATE;
	}

	if (frequency <= 13) {
		_current_rate = 13;
		bits = CTRL_REG1_DR_12_5;

	} else if (frequency <= 25) {
		_current_rate = 25;
		bits = CTRL_REG1_DR_25HZ;

	} else if (frequency <= 50) {
		_current_rate = 50;
		bits = CTRL_REG1_DR_50HZ;

	} else if (frequency <= 100) {
		_current_rate = 100;
		bits = CTRL_REG1_DR_100HZ;

	} else if (frequency <= 200) {
		_current_rate = 200;
		bits = CTRL_REG1_DR_200HZ;

	} else if (frequency <= 400) {
		_current_rate = 400;
		bits = CTRL_REG1_DR_400HZ;

	} else if (frequency <= 800) {
		_current_rate = 800;
		bits = CTRL_REG1_DR_800HZ;

	} else {
		return -EINVAL;
	}

	set_standby(last_rate, true);
	modify_reg(FXAS21002C_CTRL_REG1, CTRL_REG1_DR_MASK, bits);
	set_standby(_current_rate, false);
	return OK;
}

void	FXAS21002C::set_onchip_lowpass_filter(int frequency_hz)
{
	int high = 256 / (800 / _current_rate);
	int med = high / 2 ;
	int low = med / 2;

	if (_current_rate <= 25) {
		low = -1;
	}

	if (_current_rate == 13) {
		med = -1;
		low = -1;
	}

	uint8_t filter = CTRL_REG0_BW_HIGH;

	if (frequency_hz == 0) {
		filter = CTRL_REG0_BW_HIGH;

	} else if (frequency_hz <= low) {
		filter = CTRL_REG0_BW_LOW;

	} else if (frequency_hz <= med) {
		filter = CTRL_REG0_BW_MED;

	} else if (frequency_hz <= high) {
		filter = CTRL_REG0_BW_HIGH;
	}

	set_standby(_current_rate, true);
	modify_reg(FXAS21002C_CTRL_REG1, CTRL_REG0_BW_MASK, filter);
	set_standby(_current_rate, false);
}


void
FXAS21002C::set_sw_lowpass_filter(float samplerate, float bandwidth)
{
	_gyro_filter_x.set_cutoff_frequency(samplerate, bandwidth);
	_gyro_filter_y.set_cutoff_frequency(samplerate, bandwidth);
	_gyro_filter_z.set_cutoff_frequency(samplerate, bandwidth);
}


void
FXAS21002C::start()
{
	/* make sure we are stopped first */
	stop();

	/* reset the report ring */
	_reports->flush();

	/* start polling at the specified rate */
	ScheduleOnInterval(_call_interval - FXAS21002C_TIMER_REDUCTION, 10000);
}

void
FXAS21002C::stop()
{
	ScheduleClear();

	/* reset internal states */
	/* discard unread data in the buffers */
	_reports->flush();
}

void
FXAS21002C::Run()
{
	/* make another measurement */
	measure();
}

void
FXAS21002C::check_registers(void)
{
	uint8_t v;

	if ((v = read_reg(_checked_registers[_checked_next])) != _checked_values[_checked_next]) {
		/*
		  if we get the wrong value then we know the SPI bus
		  or sensor is very sick. We set _register_wait to 20
		  and wait until we have seen 20 good values in a row
		  before we consider the sensor to be OK again.
		 */
		perf_count(_bad_registers);

		/*
		  try to fix the bad register value. We only try to
		  fix one per loop to prevent a bad sensor hogging the
		  bus. We skip zero as that is the WHO_AM_I, which
		  is not writeable
		 */
		if (_checked_next != 0) {
			write_reg(_checked_registers[_checked_next], _checked_values[_checked_next]);
		}

		_register_wait = 20;
	}

	_checked_next = (_checked_next + 1) % FXAS21002C_NUM_CHECKED_REGISTERS;
}

void
FXAS21002C::measure()
{
	/* status register and data as read back from the device */

#pragma pack(push, 1)
	struct {
		uint8_t		cmd;
		uint8_t		status;
		int16_t		x;
		int16_t		y;
		int16_t		z;
	} raw_gyro_report;
#pragma pack(pop)

	sensor_gyro_s gyro_report;

	/* start the performance counter */
	perf_begin(_sample_perf);

	check_registers();

	if (_register_wait != 0) {
		// we are waiting for some good transfers before using
		// the sensor again.
		_register_wait--;
		perf_end(_sample_perf);
		return;
	}

	/* fetch data from the sensor */
	memset(&raw_gyro_report, 0, sizeof(raw_gyro_report));
	raw_gyro_report.cmd = DIR_READ(FXAS21002C_STATUS);
	transfer((uint8_t *)&raw_gyro_report, (uint8_t *)&raw_gyro_report, sizeof(raw_gyro_report));

	if (!(raw_gyro_report.status & DR_STATUS_ZYXDR)) {
		perf_end(_sample_perf);
		perf_count(_duplicates);
		return;
	}

	/*
	 * The TEMP register contains an 8-bit 2's complement temperature value with a range
	 * of –128 °C to +127 °C and a scaling of 1 °C/LSB. The temperature data is only
	 * compensated (factory trim values applied) when the device is operating in the Active
	 * mode and actively measuring the angular rate.
	 */

	if ((_read % _current_rate) == 0) {
		_last_temperature = read_reg(FXAS21002C_TEMP) * 1.0f;
		gyro_report.temperature = _last_temperature;
	}

	/*
	 * 1) Scale raw value to SI units using scaling from datasheet.
	 * 2) Subtract static offset (in SI units)
	 * 3) Scale the statically calibrated values with a linear
	 *    dynamically obtained factor
	 *
	 * Note: the static sensor offset is the number the sensor outputs
	 * 	 at a nominally 'zero' input. Therefore the offset has to
	 * 	 be subtracted.
	 *
	 *	 Example: A gyro outputs a value of 74 at zero angular rate
	 *	 	  the offset is 74 from the origin and subtracting
	 *		  74 from all measurements centers them around zero.
	 */

	gyro_report.timestamp = hrt_absolute_time();

	// report the error count as the number of bad
	// register reads. This allows the higher level
	// code to decide if it should use this sensor based on
	// whether it has had failures
	gyro_report.error_count = perf_event_count(_bad_registers);

	gyro_report.x_raw = swap16(raw_gyro_report.x);
	gyro_report.y_raw = swap16(raw_gyro_report.y);
	gyro_report.z_raw = swap16(raw_gyro_report.z);

	float xraw_f = gyro_report.x_raw;
	float yraw_f = gyro_report.y_raw;
	float zraw_f = gyro_report.z_raw;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_in_new = ((xraw_f * _gyro_range_scale) - _gyro_scale.x_offset) * _gyro_scale.x_scale;
	float y_in_new = ((yraw_f * _gyro_range_scale) - _gyro_scale.y_offset) * _gyro_scale.y_scale;
	float z_in_new = ((zraw_f * _gyro_range_scale) - _gyro_scale.z_offset) * _gyro_scale.z_scale;

	gyro_report.x = _gyro_filter_x.apply(x_in_new);
	gyro_report.y = _gyro_filter_y.apply(y_in_new);
	gyro_report.z = _gyro_filter_z.apply(z_in_new);

	matrix::Vector3f gval(x_in_new, y_in_new, z_in_new);
	matrix::Vector3f gval_integrated;

	bool gyro_notify = _gyro_int.put(gyro_report.timestamp, gval, gval_integrated, gyro_report.integral_dt);
	gyro_report.x_integral = gval_integrated(0);
	gyro_report.y_integral = gval_integrated(1);
	gyro_report.z_integral = gval_integrated(2);

	gyro_report.scaling = _gyro_range_scale;

	/* return device ID */
	gyro_report.device_id = _device_id.devid;


	_reports->force(&gyro_report);

	/* notify anyone waiting for data */
	if (gyro_notify) {
		poll_notify(POLLIN);

		if (!(_pub_blocked)) {
			/* publish it */
			orb_publish(ORB_ID(sensor_gyro), _gyro_topic, &gyro_report);
		}
	}

	_read++;

	/* stop the perf counter */
	perf_end(_sample_perf);
}


void
FXAS21002C::print_info()
{
	printf("gyro reads:          %u\n", _read);
	perf_print_counter(_sample_perf);
	perf_print_counter(_errors);
	perf_print_counter(_bad_registers);
	perf_print_counter(_duplicates);
	_reports->print_info("report queue");
	::printf("checked_next: %u\n", _checked_next);

	for (uint8_t i = 0; i < FXAS21002C_NUM_CHECKED_REGISTERS; i++) {
		uint8_t v = read_reg(_checked_registers[i]);

		if (v != _checked_values[i]) {
			::printf("reg %02x:%02x should be %02x\n",
				 (unsigned)_checked_registers[i],
				 (unsigned)v,
				 (unsigned)_checked_values[i]);
		}
	}

	::printf("temperature: %.2f\n", (double)_last_temperature);
}

void
FXAS21002C::print_registers()
{
	const struct {
		uint8_t reg;
		const char *name;
	} regmap[] = {
		DEF_REG(FXAS21002C_STATUS),
		DEF_REG(FXAS21002C_OUT_X_MSB),
		DEF_REG(FXAS21002C_OUT_X_LSB),
		DEF_REG(FXAS21002C_OUT_Y_MSB),
		DEF_REG(FXAS21002C_OUT_Y_LSB),
		DEF_REG(FXAS21002C_OUT_Z_MSB),
		DEF_REG(FXAS21002C_OUT_Z_LSB),
		DEF_REG(FXAS21002C_DR_STATUS),
		DEF_REG(FXAS21002C_F_STATUS),
		DEF_REG(FXAS21002C_F_SETUP),
		DEF_REG(FXAS21002C_F_EVENT),
		DEF_REG(FXAS21002C_INT_SRC_FLAG),
		DEF_REG(FXAS21002C_WHO_AM_I),
		DEF_REG(FXAS21002C_CTRL_REG0),
		DEF_REG(FXAS21002C_RT_CFG),
		DEF_REG(FXAS21002C_RT_SRC),
		DEF_REG(FXAS21002C_RT_THS),
		DEF_REG(FXAS21002C_RT_COUNT),
		DEF_REG(FXAS21002C_TEMP),
		DEF_REG(FXAS21002C_CTRL_REG1),
		DEF_REG(FXAS21002C_CTRL_REG2),
		DEF_REG(FXAS21002C_CTRL_REG3),
	};

	for (uint8_t i = 0; i < sizeof(regmap) / sizeof(regmap[0]); i++) {
		printf("0x%02x %d:%s\n", read_reg(regmap[i].reg), regmap[i].reg, regmap[i].name);
	}
}

void
FXAS21002C::test_error()
{
	// trigger an error
	write_reg(FXAS21002C_CTRL_REG1, 0);
}

/**
 * Local functions in support of the shell command.
 */
namespace fxas21002c
{

FXAS21002C	*g_dev;

void	start(bool external_bus, enum Rotation rotation);
void	test();
void	reset();
void	info();
void	regdump();
void	usage();
void	test_error();

/**
 * Start the driver.
 *
 * This function call only returns once the driver is
 * up and running or failed to detect the sensor.
 */
void
start(bool external_bus, enum Rotation rotation)
{
	int fd;

	if (g_dev != nullptr) {
		PX4_INFO("already started");
		exit(0);
	}

	/* create the driver */
	if (external_bus) {
#if defined(PX4_SPI_BUS_EXT) && defined(PX4_SPIDEV_EXT_GYRO)
		g_dev = new FXAS21002C(PX4_SPI_BUS_EXT, FXAS21002C_DEVICE_PATH_GYRO, PX4_SPIDEV_EXT_GYRO, rotation);
#else
		PX4_ERR("External SPI not available");
		exit(0);
#endif

	} else {
		g_dev = new FXAS21002C(PX4_SPI_BUS_SENSORS, FXAS21002C_DEVICE_PATH_GYRO, PX4_SPIDEV_GYRO, rotation);
	}

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating FXAS21002C obj");
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(FXAS21002C_DEVICE_PATH_GYRO, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	close(fd);

	exit(0);
fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	int fd_gyro = -1;
	sensor_gyro_s g_report{};
	ssize_t sz;

	/* get the driver */
	fd_gyro = open(FXAS21002C_DEVICE_PATH_GYRO, O_RDONLY);

	if (fd_gyro < 0) {
		err(1, "%s open failed", FXAS21002C_DEVICE_PATH_GYRO);
	}

	/* do a simple demand read */
	sz = read(fd_gyro, &g_report, sizeof(g_report));

	if (sz != sizeof(g_report)) {
		err(1, "immediate gyro read failed");
	}

	print_message(g_report);

	if (ioctl(fd_gyro, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "reset to default polling");
	}

	close(fd_gyro);

	/* XXX add poll-rate tests here too */
	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(FXAS21002C_DEVICE_PATH_GYRO, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("Open failed\n");
		exit(1);
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		PX4_ERR("driver reset failed");
		exit(1);
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("accel pollrate reset failed");
		exit(1);
	}

	close(fd);

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running\n");
		exit(1);
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

/**
 * dump registers from device
 */
void
regdump()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running\n");
		exit(1);
	}

	printf("regdump @ %p\n", g_dev);
	g_dev->print_registers();

	exit(0);
}

/**
 * trigger an error
 */
void
test_error()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running\n");
		exit(1);
	}

	g_dev->test_error();

	exit(0);
}

void
usage()
{
	PX4_INFO("missing command: try 'start', 'info', 'test', 'reset', 'testerror' or 'regdump'");
	PX4_INFO("options:");
	PX4_INFO("    -X    (external bus)");
	PX4_INFO("    -R rotation");
	PX4_INFO("    -a range in ga 2,4,8");
}

} // namespace

int
fxas21002c_main(int argc, char *argv[])
{
	bool external_bus = false;
	int ch;
	enum Rotation rotation = ROTATION_NONE;

	int myoptind = 1;
	const char *myoptarg = NULL;

	while ((ch = px4_getopt(argc, argv, "XR:a:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			external_bus = true;
			break;

		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		default:
			fxas21002c::usage();
			exit(0);
		}
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.

	 */
	if (!strcmp(verb, "start")) {
		fxas21002c::start(external_bus, rotation);
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		fxas21002c::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		fxas21002c::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		fxas21002c::info();
	}

	/*
	 * dump device registers
	 */
	if (!strcmp(verb, "regdump")) {
		fxas21002c::regdump();
	}

	/*
	 * trigger an error
	 */
	if (!strcmp(verb, "testerror")) {
		fxas21002c::test_error();
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset', 'info', 'testerror' or 'regdump'");
}
