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
 * @file fxos8701cq.cpp
 * Driver for the NXP FXOS8701CQ 6-axis sensor with integrated linear accelerometer and
 * magnetometer connected via SPI.
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <ecl/geo/geo.h>

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
#include <drivers/drv_accel.h>
#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
#  include <drivers/drv_mag.h>
#endif
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>

#include <board_config.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>
#include <px4_getopt.h>
#include <systemlib/err.h>
#include <px4_work_queue/ScheduledWorkItem.hpp>

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

/*
  list of registers that will be checked in check_registers(). Note
  that ADDR_WHO_AM_I must be first in the list.
 */
const uint8_t FXOS8701CQ::_checked_registers[FXOS8701C_NUM_CHECKED_REGISTERS] = {
	FXOS8701CQ_WHOAMI,
	FXOS8701CQ_XYZ_DATA_CFG,
	FXOS8701CQ_CTRL_REG1,
	FXOS8701CQ_M_CTRL_REG1,
	FXOS8701CQ_M_CTRL_REG2,
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

FXOS8701CQ::FXOS8701CQ(int bus, const char *path, uint32_t device, enum Rotation rotation) :
	SPI("FXOS8701CQ", path, bus, device, SPIDEV_MODE0,
	    1 * 1000 * 1000),
	ScheduledWorkItem(px4::device_bus_to_wq(get_device_id())),
#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	_mag(new FXOS8701CQ_mag(this)),
	_call_mag_interval(0),
	_mag_reports(nullptr),
	_mag_scale{},
	_mag_range_ga(0.0f),
	_mag_range_scale(0.0f),
	_mag_samplerate(0),
	_mag_read(0),
	_mag_sample_perf(perf_alloc(PC_ELAPSED, "fxos8701cq_mag_read")),
	_last_raw_mag_x(0),
	_last_raw_mag_y(0),
	_last_raw_mag_z(0),
#endif
	_call_accel_interval(0),
	_accel_reports(nullptr),
	_accel_scale{},
	_accel_range_m_s2(0.0f),
	_accel_range_scale(0.0f),
	_accel_samplerate(0),
	_accel_onchip_filter_bandwith(0),
	_accel_topic(nullptr),
	_accel_orb_class_instance(-1),
	_accel_class_instance(-1),
	_accel_read(0),
	_accel_sample_perf(perf_alloc(PC_ELAPSED, "fxos8701cq_acc_read")),
	_bad_registers(perf_alloc(PC_COUNT, "fxos8701cq_bad_reg")),
	_bad_values(perf_alloc(PC_COUNT, "fxos8701cq_bad_val")),
	_accel_duplicates(perf_alloc(PC_COUNT, "fxos8701cq_acc_dupe")),
	_register_wait(0),
	_accel_filter_x(FXOS8701C_ACCEL_DEFAULT_RATE, FXOS8701C_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_y(FXOS8701C_ACCEL_DEFAULT_RATE, FXOS8701C_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_z(FXOS8701C_ACCEL_DEFAULT_RATE, FXOS8701C_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_int(1000000 / FXOS8701C_ACCEL_MAX_OUTPUT_RATE, true),
	_rotation(rotation),
	_constant_accel_count(0),
	_last_temperature(0),
	_checked_next(0)
{
	// enable debug() calls
	_debug_enabled = false;

	_device_id.devid_s.devtype = DRV_ACC_DEVTYPE_FXOS8701C;

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	/* Prime _mag with parents devid. */
	_mag->_device_id.devid = _device_id.devid;
	_mag->_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_FXOS8701C;
#endif

	// default scale factors
	_accel_scale.x_offset = 0.0f;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0.0f;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0.0f;
	_accel_scale.z_scale  = 1.0f;

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	_mag_scale.x_offset = 0.0f;
	_mag_scale.x_scale = 1.0f;
	_mag_scale.y_offset = 0.0f;
	_mag_scale.y_scale = 1.0f;
	_mag_scale.z_offset = 0.0f;
	_mag_scale.z_scale = 1.0f;
#endif
}

FXOS8701CQ::~FXOS8701CQ()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_accel_reports != nullptr) {
		delete _accel_reports;
	}

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)

	if (_mag_reports != nullptr) {
		delete _mag_reports;
	}

#endif

	if (_accel_class_instance != -1) {
		unregister_class_devname(ACCEL_BASE_DEVICE_PATH, _accel_class_instance);
	}

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	delete _mag;
	perf_free(_mag_sample_perf);
#endif

	/* delete the perf counter */
	perf_free(_accel_sample_perf);
	perf_free(_bad_registers);
	perf_free(_bad_values);
	perf_free(_accel_duplicates);
}

int
FXOS8701CQ::init()
{
	int ret = PX4_ERROR;

	/* do SPI init (and probe) first */
	if (SPI::init() != OK) {
		PX4_ERR("SPI init failed");
		return ret;
	}

	/* allocate basic report buffers */
	_accel_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_accel_s));

	if (_accel_reports == nullptr) {
		return ret;
	}

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	_mag_reports = new ringbuffer::RingBuffer(2, sizeof(mag_report));

	if (_mag_reports == nullptr) {
		return ret;
	}

#endif

	// set software low pass filter for controllers
	param_t accel_cut_ph = param_find("IMU_ACCEL_CUTOFF");
	float accel_cut = FXOS8701C_ACCEL_DEFAULT_DRIVER_FILTER_FREQ;

	if (accel_cut_ph != PARAM_INVALID && param_get(accel_cut_ph, &accel_cut) == PX4_OK) {
		accel_set_driver_lowpass_filter(FXOS8701C_ACCEL_DEFAULT_RATE, accel_cut);

	} else {
		PX4_ERR("IMU_ACCEL_CUTOFF param invalid");
	}

	reset();

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	/* do CDev init for the mag device node */
	ret = _mag->init();

	if (ret != OK) {
		PX4_ERR("MAG init failed");
		return PX4_ERROR;
	}

#endif

	/* fill report structures */
	measure();

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	/* advertise sensor topic, measure manually to initialize valid report */
	struct mag_report mrp;
	_mag_reports->get(&mrp);

	/* measurement will have generated a report, publish */
	_mag->_mag_topic = orb_advertise_multi(ORB_ID(sensor_mag), &mrp,
					       &_mag->_mag_orb_class_instance, ORB_PRIO_LOW);

	if (_mag->_mag_topic == nullptr) {
		PX4_ERR("ADVERT ERR");
		return PX4_ERROR;
	}

#endif

	_accel_class_instance = register_class_devname(ACCEL_BASE_DEVICE_PATH);

	/* advertise sensor topic, measure manually to initialize valid report */
	sensor_accel_s arp;
	_accel_reports->get(&arp);

	/* measurement will have generated a report, publish */
	_accel_topic = orb_advertise_multi(ORB_ID(sensor_accel), &arp,
					   &_accel_orb_class_instance, (external()) ? ORB_PRIO_VERY_HIGH : ORB_PRIO_DEFAULT);

	if (_accel_topic == nullptr) {
		PX4_ERR("ADVERT ERR");
		return PX4_ERROR;
	}

	return PX4_OK;
}


void
FXOS8701CQ::reset()
{

	/* enable accel set it To Standby */

	write_checked_reg(FXOS8701CQ_CTRL_REG1, 0);
	write_checked_reg(FXOS8701CQ_XYZ_DATA_CFG, 0);

	/* Use hybird mode to read Accel and Mag */

	write_checked_reg(FXOS8701CQ_M_CTRL_REG1, M_CTRL_REG1_HMS_AM | M_CTRL_REG1_OS(7));

	/* Use the hybird auto increment mode  to read all the data at the same time */

	write_checked_reg(FXOS8701CQ_M_CTRL_REG2, CTRL_REG2_AUTO_INC);

	accel_set_range(FXOS8701C_ACCEL_DEFAULT_RANGE_G);
	accel_set_samplerate(FXOS8701C_ACCEL_DEFAULT_RATE);
	accel_set_driver_lowpass_filter((float)FXOS8701C_ACCEL_DEFAULT_RATE, (float)FXOS8701C_ACCEL_DEFAULT_DRIVER_FILTER_FREQ);

	// we setup the anti-alias on-chip filter as 50Hz. We believe
	// this operates in the analog domain, and is critical for
	// anti-aliasing. The 2 pole software filter is designed to
	// operate in conjunction with this on-chip filter
	accel_set_onchip_lowpass_filter_bandwidth(FXOS8701C_ACCEL_DEFAULT_ONCHIP_FILTER_FREQ);

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	mag_set_range(FXOS8701C_MAG_DEFAULT_RANGE_GA);
#endif
	/* enable  set it To Standby mode at 800 Hz which becomes 400 Hz due to hybird mode */

	write_checked_reg(FXOS8701CQ_CTRL_REG1, CTRL_REG1_DR(0) | CTRL_REG1_ACTIVE);

	_accel_read = 0;
#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	_mag_read = 0;
#endif
}

int
FXOS8701CQ::probe()
{
	/* verify that the device is attached and functioning */
	uint8_t whoami = read_reg(FXOS8701CQ_WHOAMI);
	bool success = (whoami == FXOS8700CQ_WHOAMI_VAL) || (whoami == FXOS8701CQ_WHOAMI_VAL);

	if (success) {
		_checked_values[0] = whoami;
		return OK;
	}

	return -EIO;
}

ssize_t
FXOS8701CQ::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(sensor_accel_s);
	sensor_accel_s *arb = reinterpret_cast<sensor_accel_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_call_accel_interval > 0) {
		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 */
		while (count--) {
			if (_accel_reports->get(arb)) {
				ret += sizeof(*arb);
				arb++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement */
	measure();

	/* measurement will have generated a report, copy it out */
	if (_accel_reports->get(arb)) {
		ret = sizeof(*arb);
	}

	return ret;
}

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
ssize_t
FXOS8701CQ::mag_read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct mag_report);
	mag_report *mrb = reinterpret_cast<mag_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_call_mag_interval > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 */
		while (count--) {
			if (_mag_reports->get(mrb)) {
				ret += sizeof(*mrb);
				mrb++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement */
	_mag_reports->flush();
	_mag->measure();

	/* measurement will have generated a report, copy it out */
	if (_mag_reports->get(mrb)) {
		ret = sizeof(*mrb);
	}

	return ret;
}
#endif

int
FXOS8701CQ::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default polling rate */
			case SENSOR_POLLRATE_DEFAULT:
				return ioctl(filp, SENSORIOCSPOLLRATE, FXOS8701C_ACCEL_DEFAULT_RATE);

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_call_accel_interval == 0);

					/* convert hz to hrt interval via microseconds */
					unsigned interval = 1000000 / arg;

					/* check against maximum sane rate */
					if (interval < 500) {
						return -EINVAL;
					}

					/* adjust filters */
					accel_set_driver_lowpass_filter((float)arg, _accel_filter_x.get_cutoff_freq());

					/* update interval for next measurement */
					/* XXX this is a bit shady, but no other way to adjust... */
					_call_accel_interval = interval;

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

	case ACCELIOCSSCALE: {
			/* copy scale, but only if off by a few percent */
			struct accel_calibration_s *s = (struct accel_calibration_s *) arg;
			float sum = s->x_scale + s->y_scale + s->z_scale;

			if (sum > 2.0f && sum < 4.0f) {
				memcpy(&_accel_scale, s, sizeof(_accel_scale));
				return OK;

			} else {
				return -EINVAL;
			}
		}

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
int
FXOS8701CQ::mag_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default polling rate */
			case SENSOR_POLLRATE_DEFAULT:
				/* 100 Hz is max for mag */
				return mag_ioctl(filp, SENSORIOCSPOLLRATE, 100);

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_call_mag_interval == 0);

					/* convert hz to hrt interval via microseconds */
					unsigned interval = 1000000 / arg;

					/* check against maximum sane rate */
					if (interval < 1000) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					/* XXX this is a bit shady, but no other way to adjust... */
					_call_mag_interval = interval;

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

	case MAGIOCSSCALE:
		/* copy scale in */
		memcpy(&_mag_scale, (struct mag_calibration_s *) arg, sizeof(_mag_scale));
		return OK;

	case MAGIOCGSCALE:
		/* copy scale out */
		memcpy((struct mag_calibration_s *) arg, &_mag_scale, sizeof(_mag_scale));
		return OK;

	case MAGIOCGEXTERNAL:
		/* Even if this sensor is on the "external" SPI bus
		 * it is still fixed to the autopilot assembly,
		 * so always return 0.
		 */
		return 0;

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}
#endif

uint8_t
FXOS8701CQ::read_reg(unsigned reg)
{
	uint8_t cmd[3];

	cmd[0] = DIR_READ(reg);
	cmd[1] = ADDR_7(reg);
	cmd[2] = 0;

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[2];
}

void
FXOS8701CQ::write_reg(unsigned reg, uint8_t value)
{
	uint8_t cmd[3];

	cmd[0] = DIR_WRITE(reg);
	cmd[1] = ADDR_7(reg);
	cmd[2] = value;

	transfer(cmd, nullptr, sizeof(cmd));
}

void
FXOS8701CQ::write_checked_reg(unsigned reg, uint8_t value)
{
	write_reg(reg, value);

	for (uint8_t i = 0; i < FXOS8701C_NUM_CHECKED_REGISTERS; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
		}
	}
}

void
FXOS8701CQ::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_checked_reg(reg, val);
}

int
FXOS8701CQ::accel_set_range(unsigned max_g)
{
	uint8_t setbits = 0;
	float lsb_per_g;
	float max_accel_g;

	if (max_g == 0 || max_g > 8) {
		max_g = 8;
	}

	if (max_g > 4) { //  8g
		setbits = XYZ_DATA_CFG_FS_8G;
		lsb_per_g = 1024;
		max_accel_g = 8;

	} else if (max_g > 2) { //  4g
		setbits = XYZ_DATA_CFG_FS_4G;
		lsb_per_g = 2048;
		max_accel_g = 4;

	} else {                //  2g
		setbits = XYZ_DATA_CFG_FS_2G;
		lsb_per_g = 4096;
		max_accel_g = 2;
	}

	_accel_range_scale = (CONSTANTS_ONE_G / lsb_per_g);
	_accel_range_m_s2 = max_accel_g * CONSTANTS_ONE_G;


	modify_reg(FXOS8701CQ_XYZ_DATA_CFG, XYZ_DATA_CFG_FS_MASK, setbits);

	return OK;
}

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
int
FXOS8701CQ::mag_set_range(unsigned max_ga)
{
	_mag_range_ga = 12;
	_mag_range_scale = 0.001f;

	return OK;
}
#endif

int
FXOS8701CQ::accel_set_onchip_lowpass_filter_bandwidth(unsigned bandwidth)
{
	return OK;
}

int
FXOS8701CQ::accel_set_driver_lowpass_filter(float samplerate, float bandwidth)
{
	_accel_filter_x.set_cutoff_frequency(samplerate, bandwidth);
	_accel_filter_y.set_cutoff_frequency(samplerate, bandwidth);
	_accel_filter_z.set_cutoff_frequency(samplerate, bandwidth);

	return OK;
}

int
FXOS8701CQ::accel_set_samplerate(unsigned frequency)
{
	uint8_t setbits = 0;

	/* The selected ODR is reduced by a factor of two when the device is operated in hybrid mode.*/

	uint8_t  active      = read_reg(FXOS8701CQ_CTRL_REG1) & CTRL_REG1_ACTIVE;

	if (frequency == 0) {
		frequency = FXOS8701C_ACCEL_DEFAULT_RATE;
	}

	if (frequency <= 25) {
		setbits = CTRL_REG1_DR(4); // Use 50 as it is 50 / 2
		_accel_samplerate = 25;

	} else if (frequency <= 50) {
		setbits = CTRL_REG1_DR(3); // Use 100 as it is 100 / 2
		_accel_samplerate = 50;

	} else if (frequency <= 100) {
		setbits = CTRL_REG1_DR(2); // Use 200 as it is 200 / 2
		_accel_samplerate = 100;

	} else if (frequency <= 200) {
		setbits = CTRL_REG1_DR(1); // Use 400 as it is 400 / 2;
		_accel_samplerate = 200;

	} else if (frequency <= 400) {
		setbits = CTRL_REG1_DR(0); // Use 800 as it is 800 / 2;
		_accel_samplerate = 400;

	} else {
		return -EINVAL;
	}

	modify_reg(FXOS8701CQ_CTRL_REG1, CTRL_REG1_ACTIVE, 0);
	modify_reg(FXOS8701CQ_CTRL_REG1, CTRL_REG1_DR_MASK, setbits);
	modify_reg(FXOS8701CQ_CTRL_REG1, 0, active);
	return OK;
}

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
int
FXOS8701CQ::mag_set_samplerate(unsigned frequency)
{
	if (frequency == 0) {
		frequency = 100;
	}

	if (frequency <= 25) {
		_mag_samplerate = 25;

	} else if (frequency <= 50) {
		_mag_samplerate = 50;

	} else if (frequency <= 100) {
		_mag_samplerate = 100;

	} else if (frequency <= 200) {
		_mag_samplerate = 200;

	} else if (frequency <= 400) {
		_mag_samplerate = 400;

	} else {
		return -EINVAL;
	}

	return OK;
}
#endif

void
FXOS8701CQ::start()
{
	/* make sure we are stopped first */
	stop();

	/* reset the report ring */
	_accel_reports->flush();
#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	_mag_reports->flush();
#endif

	/* start polling at the specified rate */
	ScheduleOnInterval(_call_accel_interval - FXOS8701C_TIMER_REDUCTION, 10000);
}

void
FXOS8701CQ::stop()
{
	ScheduleClear();

	/* reset internal states */
	memset(_last_accel, 0, sizeof(_last_accel));

	/* discard unread data in the buffers */
	_accel_reports->flush();
#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	_mag_reports->flush();
#endif
}

void
FXOS8701CQ::Run()
{
	/* make another measurement */
	measure();

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)

	if (hrt_elapsed_time(&_mag_last_measure) >= _call_mag_interval) {
		mag_measure();
	}

#endif
}

void
FXOS8701CQ::check_registers(void)
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

	_checked_next = (_checked_next + 1) % FXOS8701C_NUM_CHECKED_REGISTERS;
}

void
FXOS8701CQ::measure()
{
	/* status register and data as read back from the device */

#pragma pack(push, 1)
	struct {
		uint8_t		cmd[2];
		uint8_t		status;
		int16_t		x;
		int16_t		y;
		int16_t		z;
		int16_t		mx;
		int16_t		my;
		int16_t		mz;
	} raw_accel_mag_report;
#pragma pack(pop)

	sensor_accel_s accel_report;

	/* start the performance counter */
	perf_begin(_accel_sample_perf);

	check_registers();

	if (_register_wait != 0) {
		// we are waiting for some good transfers before using
		// the sensor again.
		_register_wait--;
		perf_end(_accel_sample_perf);
		return;
	}

	/* fetch data from the sensor */
	memset(&raw_accel_mag_report, 0, sizeof(raw_accel_mag_report));
	raw_accel_mag_report.cmd[0] = DIR_READ(FXOS8701CQ_DR_STATUS);
	raw_accel_mag_report.cmd[1] = ADDR_7(FXOS8701CQ_DR_STATUS);
	transfer((uint8_t *)&raw_accel_mag_report, (uint8_t *)&raw_accel_mag_report, sizeof(raw_accel_mag_report));

	if (!(raw_accel_mag_report.status & DR_STATUS_ZYXDR)) {
		perf_end(_accel_sample_perf);
		perf_count(_accel_duplicates);
		return;
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

	accel_report.timestamp = hrt_absolute_time();

	/*
	 * Eight-bit 2’s complement sensor temperature value with 0.96 °C/LSB sensitivity.
	 * Temperature data is only valid between –40 °C and 125 °C. The temperature sensor
	 * output is only valid when M_CTRL_REG1[m_hms] > 0b00. Please note that the
	 * temperature sensor is uncalibrated and its output for a given temperature will vary from
	 * one device to the next
	 */

	_last_temperature = (read_reg(FXOS8701CQ_TEMP)) * 0.96f;

	accel_report.temperature = _last_temperature;

	// report the error count as the sum of the number of bad
	// register reads and bad values. This allows the higher level
	// code to decide if it should use this sensor based on
	// whether it has had failures
	accel_report.error_count = perf_event_count(_bad_registers) + perf_event_count(_bad_values);

	accel_report.x_raw = swap16RightJustify14(raw_accel_mag_report.x);
	accel_report.y_raw = swap16RightJustify14(raw_accel_mag_report.y);
	accel_report.z_raw = swap16RightJustify14(raw_accel_mag_report.z);

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	/* Save off the Mag readings todo: revist integrating theses */

	_last_raw_mag_x = swap16(raw_accel_mag_report.mx);
	_last_raw_mag_y = swap16(raw_accel_mag_report.my);
	_last_raw_mag_z = swap16(raw_accel_mag_report.mz);
#endif

	float xraw_f = accel_report.x_raw;
	float yraw_f = accel_report.y_raw;
	float zraw_f = accel_report.z_raw;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_in_new = ((xraw_f * _accel_range_scale) - _accel_scale.x_offset) * _accel_scale.x_scale;
	float y_in_new = ((yraw_f * _accel_range_scale) - _accel_scale.y_offset) * _accel_scale.y_scale;
	float z_in_new = ((zraw_f * _accel_range_scale) - _accel_scale.z_offset) * _accel_scale.z_scale;

	/*
	  we have logs where the accelerometers get stuck at a fixed
	  large value. We want to detect this and mark the sensor as
	  being faulty
	 */
	if (fabsf(_last_accel[0] - x_in_new) < 0.001f &&
	    fabsf(_last_accel[1] - y_in_new) < 0.001f &&
	    fabsf(_last_accel[2] - z_in_new) < 0.001f &&
	    fabsf(x_in_new) > 20 &&
	    fabsf(y_in_new) > 20 &&
	    fabsf(z_in_new) > 20) {
		_constant_accel_count += 1;

	} else {
		_constant_accel_count = 0;
	}

	if (_constant_accel_count > 100) {
		// we've had 100 constant accel readings with large
		// values. The sensor is almost certainly dead. We
		// will raise the error_count so that the top level
		// flight code will know to avoid this sensor, but
		// we'll still give the data so that it can be logged
		// and viewed
		perf_count(_bad_values);
		_constant_accel_count = 0;
	}

	_last_accel[0] = x_in_new;
	_last_accel[1] = y_in_new;
	_last_accel[2] = z_in_new;

	accel_report.x = _accel_filter_x.apply(x_in_new);
	accel_report.y = _accel_filter_y.apply(y_in_new);
	accel_report.z = _accel_filter_z.apply(z_in_new);

	matrix::Vector3f aval(x_in_new, y_in_new, z_in_new);
	matrix::Vector3f aval_integrated;

	bool accel_notify = _accel_int.put(accel_report.timestamp, aval, aval_integrated, accel_report.integral_dt);
	accel_report.x_integral = aval_integrated(0);
	accel_report.y_integral = aval_integrated(1);
	accel_report.z_integral = aval_integrated(2);

	accel_report.scaling = _accel_range_scale;

	/* return device ID */
	accel_report.device_id = _device_id.devid;

	_accel_reports->force(&accel_report);

	/* notify anyone waiting for data */
	if (accel_notify) {
		poll_notify(POLLIN);

		if (!(_pub_blocked)) {
			/* publish it */
			orb_publish(ORB_ID(sensor_accel), _accel_topic, &accel_report);
		}
	}

	_accel_read++;

	/* stop the perf counter */
	perf_end(_accel_sample_perf);
}

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
void
FXOS8701CQ::mag_measure()
{
	/* status register and data as read back from the device */

	mag_report mag_report {};

	/* start the performance counter */
	perf_begin(_mag_sample_perf);

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

	mag_report.timestamp = hrt_absolute_time();

	_mag_last_measure = mag_report.timestamp;

	mag_report.is_external = external();

	mag_report.x_raw = _last_raw_mag_x;
	mag_report.y_raw = _last_raw_mag_y;
	mag_report.z_raw = _last_raw_mag_z;

	float xraw_f = mag_report.x_raw;
	float yraw_f = mag_report.y_raw;
	float zraw_f = mag_report.z_raw;

	/* apply user specified rotation */
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	mag_report.x = ((xraw_f * _mag_range_scale) - _mag_scale.x_offset) * _mag_scale.x_scale;
	mag_report.y = ((yraw_f * _mag_range_scale) - _mag_scale.y_offset) * _mag_scale.y_scale;
	mag_report.z = ((zraw_f * _mag_range_scale) - _mag_scale.z_offset) * _mag_scale.z_scale;
	mag_report.scaling = _mag_range_scale;
	mag_report.error_count = perf_event_count(_bad_registers) + perf_event_count(_bad_values);

	mag_report.temperature = _last_temperature;
	mag_report.device_id = _mag->_device_id.devid;

	_mag_reports->force(&mag_report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	if (!(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(sensor_mag), _mag->_mag_topic, &mag_report);
	}

	_mag_read++;

	/* stop the perf counter */
	perf_end(_mag_sample_perf);
}
#endif

void
FXOS8701CQ::print_info()
{
	printf("accel reads:          %u\n", _accel_read);
#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	printf("mag reads:            %u\n", _mag_read);
#endif
	perf_print_counter(_accel_sample_perf);
#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	perf_print_counter(_mag_sample_perf);
#endif
	perf_print_counter(_bad_registers);
	perf_print_counter(_bad_values);
	perf_print_counter(_accel_duplicates);
	_accel_reports->print_info("accel reports");
#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	_mag_reports->print_info("mag reports");
#endif
	::printf("checked_next: %u\n", _checked_next);

	for (uint8_t i = 0; i < FXOS8701C_NUM_CHECKED_REGISTERS; i++) {
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
FXOS8701CQ::print_registers()
{
	const struct {
		uint8_t reg;
		const char *name;
	} regmap[] = {
		DEF_REG(FXOS8701CQ_DR_STATUS),
		DEF_REG(FXOS8701CQ_OUT_X_MSB),
		DEF_REG(FXOS8701CQ_XYZ_DATA_CFG),
		DEF_REG(FXOS8701CQ_WHOAMI),
		DEF_REG(FXOS8701CQ_CTRL_REG1),
		DEF_REG(FXOS8701CQ_CTRL_REG2),
		DEF_REG(FXOS8701CQ_M_DR_STATUS),
		DEF_REG(FXOS8701CQ_M_OUT_X_MSB),
		DEF_REG(FXOS8701CQ_M_CTRL_REG1),
		DEF_REG(FXOS8701CQ_M_CTRL_REG2),
		DEF_REG(FXOS8701CQ_M_CTRL_REG3),
	};

	for (uint8_t i = 0; i < sizeof(regmap) / sizeof(regmap[0]); i++) {
		printf("0x%02x %s\n", read_reg(regmap[i].reg), regmap[i].name);
	}
}

void
FXOS8701CQ::test_error()
{
	// trigger an error
	write_reg(FXOS8701CQ_CTRL_REG1, 0);
}

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
FXOS8701CQ_mag::FXOS8701CQ_mag(FXOS8701CQ *parent) :
	CDev("FXOS8701C_mag", FXOS8701C_DEVICE_PATH_MAG),
	_parent(parent),
	_mag_topic(nullptr),
	_mag_orb_class_instance(-1),
	_mag_class_instance(-1)
{
}

FXOS8701CQ_mag::~FXOS8701CQ_mag()
{
	if (_mag_class_instance != -1) {
		unregister_class_devname(MAG_BASE_DEVICE_PATH, _mag_class_instance);
	}
}

int
FXOS8701CQ_mag::init()
{
	int ret;

	ret = CDev::init();

	if (ret == OK) {
		_mag_class_instance = register_class_devname(MAG_BASE_DEVICE_PATH);
	}

	return ret;
}

void
FXOS8701CQ_mag::parent_poll_notify()
{
	poll_notify(POLLIN);
}

ssize_t
FXOS8701CQ_mag::read(struct file *filp, char *buffer, size_t buflen)
{
	return _parent->mag_read(filp, buffer, buflen);
}

int
FXOS8701CQ_mag::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case DEVIOCGDEVICEID:
		return (int)CDev::ioctl(filp, cmd, arg);
		break;

	default:
		return _parent->mag_ioctl(filp, cmd, arg);
	}
}

void
FXOS8701CQ_mag::measure()
{
	_parent->mag_measure();
}

#endif

/**
 * Local functions in support of the shell command.
 */
namespace fxos8701cq
{

FXOS8701CQ	*g_dev;

void	start(bool external_bus, enum Rotation rotation);
void	test();
void	reset();
void	info();
void	stop();
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
#if defined(PX4_SPI_BUS_EXT) && defined(PX4_SPIDEV_EXT_ACCEL_MAG)
		g_dev = new FXOS8701CQ(PX4_SPI_BUS_EXT, FXOS8701C_DEVICE_PATH_ACCEL, PX4_SPIDEV_EXT_ACCEL_MAG, rotation);
#else
		PX4_ERR("External SPI not available");
		exit(0);
#endif

	} else {
		g_dev = new FXOS8701CQ(PX4_SPI_BUS_SENSORS, FXOS8701C_DEVICE_PATH_ACCEL, PX4_SPIDEV_ACCEL_MAG, rotation);
	}

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating FXOS8701C obj");
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(FXOS8701C_DEVICE_PATH_ACCEL, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	int fd_mag;
	fd_mag = open(FXOS8701C_DEVICE_PATH_MAG, O_RDONLY);

	/* don't fail if open cannot be opened */
	if (0 <= fd_mag) {
		if (ioctl(fd_mag, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
			goto fail;
		}
	}

	close(fd_mag);
#endif

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
	int rv = 1;
	int fd_accel = -1;
	sensor_accel_s accel_report;
	ssize_t sz;
#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	int fd_mag = -1;
	int ret;
	struct mag_report m_report;
#endif

	/* get the driver */
	fd_accel = open(FXOS8701C_DEVICE_PATH_ACCEL, O_RDONLY);

	if (fd_accel < 0) {
		PX4_ERR("%s open failed", FXOS8701C_DEVICE_PATH_ACCEL);
		goto exit_none;
	}

	/* do a simple demand read */
	sz = read(fd_accel, &accel_report, sizeof(sensor_accel_s));

	if (sz != sizeof(sensor_accel_s)) {
		PX4_ERR("immediate read failed");
		goto exit_with_accel;
	}


	PX4_INFO("accel x: \t% 9.5f\tm/s^2", (double)accel_report.x);
	PX4_INFO("accel y: \t% 9.5f\tm/s^2", (double)accel_report.y);
	PX4_INFO("accel z: \t% 9.5f\tm/s^2", (double)accel_report.z);
	PX4_INFO("accel x: \t%d\traw", (int)accel_report.x_raw);
	PX4_INFO("accel y: \t%d\traw", (int)accel_report.y_raw);
	PX4_INFO("accel z: \t%d\traw", (int)accel_report.z_raw);

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	/* get the driver */
	fd_mag = open(FXOS8701C_DEVICE_PATH_MAG, O_RDONLY);

	if (fd_mag < 0) {
		PX4_ERR("%s open failed", FXOS8701C_DEVICE_PATH_MAG);
		goto exit_with_accel;
	}

	/* check if mag is onboard or external */
	if ((ret = ioctl(fd_mag, MAGIOCGEXTERNAL, 0)) < 0) {
		PX4_ERR("failed to get if mag is onboard or external");
		goto exit_with_mag_accel;
	}

	PX4_INFO("mag device active: %s", ret ? "external" : "onboard");

	/* do a simple demand read */
	sz = read(fd_mag, &m_report, sizeof(m_report));

	if (sz != sizeof(m_report)) {
		PX4_ERR("immediate read failed");
		goto exit_with_mag_accel;
	}

	PX4_INFO("mag x: \t% 9.5f\tga", (double)m_report.x);
	PX4_INFO("mag y: \t% 9.5f\tga", (double)m_report.y);
	PX4_INFO("mag z: \t% 9.5f\tga", (double)m_report.z);
	PX4_INFO("mag x: \t%d\traw", (int)m_report.x_raw);
	PX4_INFO("mag y: \t%d\traw", (int)m_report.y_raw);
	PX4_INFO("mag z: \t%d\traw", (int)m_report.z_raw);
#endif

	/* reset to default polling */
	if (ioctl(fd_accel, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("reset to default polling");

	} else {
		rv = 0;
	}

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
exit_with_mag_accel:
	close(fd_mag);
#endif

exit_with_accel:

	close(fd_accel);

	reset();

	if (rv == 0) {
		PX4_INFO("PASS");
	}

exit_none:
	exit(rv);
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(FXOS8701C_DEVICE_PATH_ACCEL, O_RDONLY);
	int rv = 1;

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

#if !defined(BOARD_HAS_NOISY_FXOS8700_MAG)
	fd = open(FXOS8701C_DEVICE_PATH_MAG, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("mag could not be opened, external mag might be used");

	} else {
		/* no need to reset the mag as well, the reset() is the same */
		if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
			PX4_ERR("mag pollrate reset failed");

		} else {
			rv = 0;
		}
	}

	close(fd);
#endif

	exit(rv);
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

void
stop()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running\n");
		exit(1);
	}

	delete g_dev;
	g_dev = nullptr;

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
	PX4_INFO("missing command: try 'start', 'info', 'stop', 'test', 'reset', 'testerror' or 'regdump'");
	PX4_INFO("options:");
	PX4_INFO("    -X    (external bus)");
	PX4_INFO("    -R rotation");
}

} // namespace

int
fxos8701cq_main(int argc, char *argv[])
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
			fxos8701cq::usage();
			exit(0);
		}
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.

	 */
	if (!strcmp(verb, "start")) {
		fxos8701cq::start(external_bus, rotation);
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		fxos8701cq::test();
	}

	if (!strcmp(verb, "stop")) {
		fxos8701cq::stop();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		fxos8701cq::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		fxos8701cq::info();
	}

	/*
	 * dump device registers
	 */
	if (!strcmp(verb, "regdump")) {
		fxos8701cq::regdump();
	}

	/*
	 * trigger an error
	 */
	if (!strcmp(verb, "testerror")) {
		fxos8701cq::test_error();
	}

	errx(1, "unrecognized command, try 'start', 'stop', 'test', 'reset', 'info', 'testerror' or 'regdump'");
}
