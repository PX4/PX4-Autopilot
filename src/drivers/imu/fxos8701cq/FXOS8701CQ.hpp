/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
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
 * @file fxos8701cq.cpp
 * Driver for the NXP FXOS8701CQ 6-axis sensor with integrated linear accelerometer and
 * magnetometer connected via SPI.
 */

#include <drivers/device/integrator.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/spi.h>
#include <drivers/drv_accel.h>
#include <ecl/geo/geo.h>
#include <lib/conversion/rotation.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <perf/perf_counter.h>
#include <px4_getopt.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <systemlib/err.h>

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
#  include <drivers/drv_mag.h>
#endif

/* SPI protocol address bits */
#define DIR_READ(a)                     ((a) & 0x7f)
#define DIR_WRITE(a)                    ((a) | (1 << 7))
#define ADDR_7(a)                       ((a) & (1 << 7))
#define swap16(w)                       __builtin_bswap16((w))
#define swap16RightJustify14(w)         (((int16_t)swap16(w)) >> 2)
#define FXOS8701C_DEVICE_PATH_ACCEL     "/dev/fxos8701cq_accel"
#define FXOS8701C_DEVICE_PATH_ACCEL_EXT "/dev/fxos8701cq_accel_ext"
#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
#  define FXOS8701C_DEVICE_PATH_MAG       "/dev/fxos8701cq_mag"
#endif

#define FXOS8701CQ_DR_STATUS       0x00
#  define DR_STATUS_ZYXDR          (1 << 3)

#define FXOS8701CQ_OUT_X_MSB       0x01

#define FXOS8701CQ_XYZ_DATA_CFG    0x0e
#  define XYZ_DATA_CFG_FS_SHIFTS   0
#  define XYZ_DATA_CFG_FS_MASK     (3 << XYZ_DATA_CFG_FS_SHIFTS)
#  define XYZ_DATA_CFG_FS_2G       (0 << XYZ_DATA_CFG_FS_SHIFTS)
#  define XYZ_DATA_CFG_FS_4G       (1 << XYZ_DATA_CFG_FS_SHIFTS)
#  define XYZ_DATA_CFG_FS_8G       (2 << XYZ_DATA_CFG_FS_SHIFTS)

#define FXOS8701CQ_WHOAMI          0x0d
#  define FXOS8700CQ_WHOAMI_VAL    0xC7
#  define FXOS8701CQ_WHOAMI_VAL    0xCA

#define FXOS8701CQ_CTRL_REG1       0x2a
#  define CTRL_REG1_ACTIVE         (1 << 0)
#  define CTRL_REG1_DR_SHIFTS      3
#  define CTRL_REG1_DR_MASK        (7 << CTRL_REG1_DR_SHIFTS)
#  define CTRL_REG1_DR(n)          (((n) & 7) << CTRL_REG1_DR_SHIFTS)
#define FXOS8701CQ_CTRL_REG2       0x2b
#  define CTRL_REG2_AUTO_INC       (1 << 5)

#define FXOS8701CQ_M_DR_STATUS     0x32
#  define M_DR_STATUS_ZYXDR         (1 << 3)
#define FXOS8701CQ_M_OUT_X_MSB     0x33
#define FXOS8701CQ_TEMP            0x51
#define FXOS8701CQ_M_CTRL_REG1     0x5b
#  define M_CTRL_REG1_HMS_SHIFTS   0
#  define M_CTRL_REG1_HMS_MASK     (3 << M_CTRL_REG1_HMS_SHIFTS)
#  define M_CTRL_REG1_HMS_A        (0 << M_CTRL_REG1_HMS_SHIFTS)
#  define M_CTRL_REG1_HMS_M        (1 << M_CTRL_REG1_HMS_SHIFTS)
#  define M_CTRL_REG1_HMS_AM       (3 << M_CTRL_REG1_HMS_SHIFTS)
#  define M_CTRL_REG1_OS_SHIFTS    2
#  define M_CTRL_REG1_OS_MASK      (7 << M_CTRL_REG1_HMS_SHIFTS)
#  define M_CTRL_REG1_OS(n)        (((n) & 7) << M_CTRL_REG1_OS_SHIFTS)

#define FXOS8701CQ_M_CTRL_REG2     0x5c
#define FXOS8701CQ_M_CTRL_REG3     0x5d

#define DEF_REG(r)   {r, #r}

/* default values for this device */
#define FXOS8701C_ACCEL_DEFAULT_RANGE_G              8
#define FXOS8701C_ACCEL_DEFAULT_RATE                 400 /* ODR is 400 in Hybird mode (accel + mag) */
#define FXOS8701C_ACCEL_DEFAULT_ONCHIP_FILTER_FREQ   50
#define FXOS8701C_ACCEL_DEFAULT_DRIVER_FILTER_FREQ   30
#define FXOS8701C_ACCEL_MAX_OUTPUT_RATE              280

#define FXOS8701C_MAG_DEFAULT_RANGE_GA               12 /* It is fixed at 12 G */
#define FXOS8701C_MAG_DEFAULT_RATE                   100

/*
  we set the timer interrupt to run a bit faster than the desired
  sample rate and then throw away duplicates using the data ready bit.
  This time reduction is enough to cope with worst case timing jitter
  due to other timers
 */
#define FXOS8701C_TIMER_REDUCTION				240

extern "C" { __EXPORT int fxos8701cq_main(int argc, char *argv[]); }


#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
class FXOS8701CQ_mag;
#endif

class FXOS8701CQ : public device::SPI, public px4::ScheduledWorkItem
{
public:
	FXOS8701CQ(int bus, const char *path, uint32_t device, enum Rotation rotation);
	virtual ~FXOS8701CQ();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

	/**
	 * dump register values
	 */
	void			print_registers();

	/**
	 * deliberately trigger an error
	 */
	void			test_error();

protected:
	virtual int		probe();

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	friend class 		FXOS8701CQ_mag;

	virtual ssize_t		mag_read(struct file *filp, char *buffer, size_t buflen);
	virtual int		mag_ioctl(struct file *filp, int cmd, unsigned long arg);
#endif

private:

	void Run() override;

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	FXOS8701CQ_mag		*_mag;
	unsigned    _call_mag_interval;
	ringbuffer::RingBuffer    *_mag_reports;

	struct mag_calibration_s  _mag_scale;
	unsigned    _mag_range_ga;
	float     _mag_range_scale;
	unsigned    _mag_samplerate;
	unsigned    _mag_read;
	perf_counter_t    _mag_sample_perf;
	int16_t     _last_raw_mag_x;
	int16_t     _last_raw_mag_y;
	int16_t     _last_raw_mag_z;

	hrt_abstime		_mag_last_measure{0};
#endif

	unsigned		_call_accel_interval;

	ringbuffer::RingBuffer	*_accel_reports;

	struct accel_calibration_s	_accel_scale;
	unsigned		_accel_range_m_s2;
	float			_accel_range_scale;
	unsigned		_accel_samplerate;
	unsigned		_accel_onchip_filter_bandwith;


	orb_advert_t		_accel_topic;
	int			_accel_orb_class_instance;
	int			_accel_class_instance;

	unsigned		_accel_read;

	perf_counter_t		_accel_sample_perf;
	perf_counter_t		_bad_registers;
	perf_counter_t		_bad_values;
	perf_counter_t		_accel_duplicates;

	uint8_t			_register_wait;

	math::LowPassFilter2p	_accel_filter_x;
	math::LowPassFilter2p	_accel_filter_y;
	math::LowPassFilter2p	_accel_filter_z;

	Integrator		_accel_int;

	enum Rotation		_rotation;

	// values used to
	float			_last_accel[3];
	uint8_t			_constant_accel_count;

	// last temperature value
	float			_last_temperature;

	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset
#define FXOS8701C_NUM_CHECKED_REGISTERS 5
	static const uint8_t	_checked_registers[FXOS8701C_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_values[FXOS8701C_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_next;

	/**
	 * Start automatic measurement.
	 */
	void			start();

	/**
	 * Stop automatic measurement.
	 */
	void			stop();

	/**
	 * Reset chip.
	 *
	 * Resets the chip and measurements ranges, but not scale and offset.
	 */
	void			reset();

	/**
	 * disable I2C on the chip
	 */
	void			disable_i2c();

	/**
	 * check key registers for correct values
	 */
	void			check_registers(void);

	/**
	 * Fetch accel measurements from the sensor and update the report ring.
	 */
	void			measure();

	/**
	 * Fetch mag measurements from the sensor and update the report ring.
	 */
	void			mag_measure();

	/**
	 * Read a register from the FXOS8701C
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t			read_reg(unsigned reg);

	/**
	 * Write a register in the FXOS8701C
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_reg(unsigned reg, uint8_t value);

	/**
	 * Modify a register in the FXOS8701C
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Write a register in the FXOS8701C, updating _checked_values
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_checked_reg(unsigned reg, uint8_t value);

	/**
	 * Set the FXOS8701C accel measurement range.
	 *
	 * @param max_g	The measurement range of the accel is in g (9.81m/s^2)
	 *			Zero selects the maximum supported range.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			accel_set_range(unsigned max_g);

	/**
	 * Set the FXOS8701C mag measurement range.
	 *
	 * @param max_ga	The measurement range of the mag is in Ga
	 *			Zero selects the maximum supported range.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			mag_set_range(unsigned max_g);

	/**
	 * Set the FXOS8701C on-chip anti-alias filter bandwith.
	 *
	 * @param bandwidth The anti-alias filter bandwidth in Hz
	 * 			Zero selects the highest bandwidth
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			accel_set_onchip_lowpass_filter_bandwidth(unsigned bandwidth);

	/**
	 * Set the driver lowpass filter bandwidth.
	 *
	 * @param bandwidth The anti-alias filter bandwidth in Hz
	 * 			Zero selects the highest bandwidth
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			accel_set_driver_lowpass_filter(float samplerate, float bandwidth);

	/**
	 * Set the FXOS8701C internal accel and mag sampling frequency.
	 *
	 * @param frequency	The internal accel and mag sampling frequency is set to not less than
	 *			this value.
	 *			Zero selects the maximum rate supported.
	 * @return		OK if the value can be supported.
	 */
	int			accel_set_samplerate(unsigned frequency);

	/**
	 * Set the FXOS8701CQ internal mag sampling frequency.
	 *
	 * @param frequency	The mag reporting frequency is set to not less than
	 *			this value. (sampling is all way the same as accel
	 *			Zero selects the maximum rate supported.
	 * @return		OK if the value can be supported.
	 */
	int			mag_set_samplerate(unsigned frequency);



	/* this class cannot be copied */
	FXOS8701CQ(const FXOS8701CQ &);
	FXOS8701CQ operator=(const FXOS8701CQ &);
};

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
/**
 * Helper class implementing the mag driver node.
 */
class FXOS8701CQ_mag : public device::CDev
{
public:
	FXOS8701CQ_mag(FXOS8701CQ *parent);
	~FXOS8701CQ_mag();

	virtual ssize_t			read(struct file *filp, char *buffer, size_t buflen);
	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);

	virtual int		init();

protected:
	friend class FXOS8701CQ;

	void				parent_poll_notify();
private:
	FXOS8701CQ				*_parent;

	orb_advert_t			_mag_topic;
	int				_mag_orb_class_instance;
	int				_mag_class_instance;

	void				measure();

	/* this class does not allow copying due to ptr data members */
	FXOS8701CQ_mag(const FXOS8701CQ_mag &);
	FXOS8701CQ_mag operator=(const FXOS8701CQ_mag &);
};
#endif
