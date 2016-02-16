/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file lsm9ds1.cpp
 *
 * Driver for the STM LSM9DS1 magnetometer connected via SPI.
 *
 * @author Fábio Azevedo
 *
 * based on the l3gd20 and lsm303d driver
 */

#include <px4_config.h>

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

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/conversions.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/spi.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#define DIR_READ			0x80
#define DIR_WRITE			0x00

#define LSM_DEVICE_PATH_MAG         "/dev/lsm9ds1_mag"
#define LSM_DEVICE_PATH_MAG_EXT     "/dev/lsm9ds1_mag_ext"


/* Orientation on board */
#define SENSOR_BOARD_ROTATION_000_DEG	0
#define SENSOR_BOARD_ROTATION_090_DEG	1
#define SENSOR_BOARD_ROTATION_180_DEG	2
#define SENSOR_BOARD_ROTATION_270_DEG	3

// LSM9DS1 registers magnetometer
#define OFFSET_X_REG_L_M        0x05
#define OFFSET_X_REG_H_M        0x06
#define OFFSET_Y_REG_L_M        0x07
#define OFFSET_Y_REG_H_M        0x08
#define OFFSET_Z_REG_L_M        0x09
#define OFFSET_Z_REG_H_M        0x0A
#define WHO_AM_I_M              0x0F
#define CTRL_REG1_M             0x20
#define CTRL_REG2_M             0x21
#define CTRL_REG3_M             0x22
#define CTRL_REG4_M             0x23
#define CTRL_REG5_M             0x24
#define STATUS_REG_M            0x27
#define OUT_X_L_M               0x28
#define OUT_X_H_M               0x29
#define OUT_Y_L_M               0x2A
#define OUT_Y_H_M               0x2B
#define OUT_Z_L_M               0x2C
#define OUT_Z_H_M               0x2D
#define INT_CFG_M               0x30
#define INT_SRC_M               0x31
#define INT_THS_L_M             0x32
#define INT_THS_H_M             0x33

// Configuration bits LSM9DS1
#define LSM_WHO_AM_I_M			0x3D

//Magnetometer ODR CTRL_REG1_M
#define LSM_M_ODR_0_625_HZ      (0<<4) | (0<<3) | (0<<2)
#define LSM_M_ODR_1_25_HZ       (0<<4) | (0<<3) | (1<<2)
#define LSM_M_ODR_2_5_HZ        (0<<4) | (1<<3) | (0<<2)
#define LSM_M_ODR_5_HZ          (0<<4) | (1<<3) | (1<<2)
#define LSM_M_ODR_10_HZ         (1<<4) | (0<<3) | (0<<2)
#define LSM_M_ODR_20_HZ         (1<<4) | (0<<3) | (1<<2)
#define LSM_M_ODR_40_HZ         (1<<4) | (1<<3) | (0<<2)
#define LSM_M_ODR_80_HZ         (1<<4) | (1<<3) | (1<<2)

//Magnetometer range CTRL_REG3_M
#define LSM_M_RANGE_4_G         (0<<6) | (0<<5)
#define LSM_M_RANGE_8_G         (0<<6) | (1<<5)
#define LSM_M_RANGE_12_G        (1<<6) | (0<<5)
#define LSM_M_RANGE_16_G        (1<<6) | (1<<5)

//CTRL_REG1_M - self-test mag ODR
#define TEMP_COMP               (1<<7)
#define LOW_PWR_M               (0<<6) | (0<<5)
#define MED_PER_M               (0<<6) | (1<<5)
#define HIGH_PER_M              (1<<6) | (0<<5)
#define UHIGH_PER_M             (1<<6) | (1<<5)

//CTRL_REG3_M - disable I2C and operating mode (pins 3, 4 and 6 must be forced to 0)
#define CTRL_REG3_M_INIT        ~((1<<6) | (1<<4) | (1<<3))

#define I2C_DISABLE_M           (1<<7)

#define LP                      (1<<5)
#define SIM                     (1<<2)
#define CONTINUOUS_MODE         (0<<1) | (0<<0)
#define SINGLE_CONV_MODE        (0<<1) | (1<<0)
#define POWER_DOWN              (1<<1) | (1<<0) // Could also be (1<<1) | (0<<0)

//CTRL_REG4_M - (pins 0 and 4-7 must be forced to 0)
#define CTRL_REG4_M_INIT        ~((1<<7) | (1<<6) | (1<<5) | (1<<4) | (1<<0))

#define LOW_PWR_M_Z             (0<<3) | (0<<2)
#define MED_PER_M_Z             (0<<3) | (1<<2)
#define HIGH_PER_M_Z            (1<<3) | (0<<2)
#define UHIGH_PER_M_Z           (1<<3) | (1<<2)

//CTRL_REG5_M - (pins 0-5 must be forced to 0)
#define CTRL_REG5_M_INIT        ~((1<<5) | (1<<4) | (1<<3) | (1<<2) | (1<<1) | (1<<0))

#define BDU_M                   (1<<6)
#define FAST_READ               (1<<7)

//INT_CFG_M - (pins 3 and 4 must be forced to 0)
#define INT_CFG_M_INIT          ~((1<<4) | (1<<3))

#define ALL_AXES_INT_M_EN       (1<<7) | (1<<6) | (1<<5)
#define IEA_M                   (1<<2)
#define IEL_M                   (1<<1)
#define IEN_M                   (1<<0)

/***************************************************/
#define LSM_WHOAMI_M                        0x3D
#define ADDR_INCR               (1<<6)
#define STATUS_ZYXDA            (1<<3)

/* default values for this device */
#define LSM9DS1_MAG_DEFAULT_RANGE_GA		8
#define LSM9DS1_MAG_DEFAULT_RATE			80


#ifdef PX4_SPI_BUS_EXT
#define EXTERNAL_BUS PX4_SPI_BUS_EXT
#else
#define EXTERNAL_BUS 0
#endif

#ifndef SENSOR_BOARD_ROTATION_DEFAULT
#define SENSOR_BOARD_ROTATION_DEFAULT		SENSOR_BOARD_ROTATION_000_DEG
#endif


#define LSM9DS1_LOW_BUS_SPEED				1000*1000
#define LSM9DS1_HIGH_BUS_SPEED				11*1000*1000


#define LSM9DS1_TIMER_REDUCTION				200


extern "C" { __EXPORT int lsm9ds1mag_main(int argc, char *argv[]); }


class LSM9DS1_mag : public device::SPI
{
public:
    LSM9DS1_mag(int bus, const char *path, spi_dev_e device, enum Rotation rotation);
    virtual ~LSM9DS1_mag();

    virtual int         init();

    virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
    virtual int         ioctl(struct file *filp, int cmd, unsigned long arg);

    /**
     * Diagnostics - print some basic information about the driver.
     */
    void                print_info();

    // print register dump
    void                print_registers();

    // trigger an error
    void                test_error();

protected:
    virtual int         probe();

private:

    struct hrt_call		_call;
    unsigned            _call_interval;

    ringbuffer::RingBuffer	*_reports;

    struct mag_scale	_mag_scale;
    float               _mag_range_scale;
    float               _mag_range_ga;
    orb_advert_t		_mag_topic;
    int                 _orb_class_instance;
    int                 _class_instance;

    unsigned            _current_rate;

    unsigned            _read;

    perf_counter_t		_sample_perf;
    perf_counter_t		_errors;
    perf_counter_t		_bad_registers;
    perf_counter_t		_duplicates;

    uint8_t             _register_wait;

    enum Rotation		_rotation;

    // this is used to support runtime checking of key
    // configuration registers to detect SPI bus errors and sensor
    // reset
#define LSM9DS1_MAG_NUM_CHECKED_REGISTERS 8
    static const uint8_t	_checked_registers[LSM9DS1_MAG_NUM_CHECKED_REGISTERS];
    uint8_t			_checked_values[LSM9DS1_MAG_NUM_CHECKED_REGISTERS];
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
     * Reset the driver
     */
    void			reset();

    /**
     * disable I2C on the chip
     */
    void			disable_i2c();

    /**
     * Get the internal / external state
     *
     * @return true if the sensor is not on the main MCU board
     */
    bool			is_external() { return (_bus == EXTERNAL_BUS); }

    /**
     * Static trampoline from the hrt_call context; because we don't have a
     * generic hrt wrapper yet.
     *
     * Called by the HRT in interrupt context l3gd20at the specified rate if
     * automatic polling is enabled.
     *
     * @param arg		Instance pointer for the driver that is polling.
     */
    static void		measure_trampoline(void *arg);

    /**
     * check key registers for correct values
     */
    void			check_registers(void);

    /**
     * Fetch measurements from the sensor and update the report ring.
     */
    void			measure();

    /**
     * Read a register from the LSM9DS1_MAG
     *
     * @param		The register to read.
     * @return		The value that was read.
     */
    uint8_t			read_reg(unsigned reg);

    /**
     * Write a register in the LSM9DS1_MAG
     *
     * @param reg		The register to write.
     * @param value		The new value to write.
     */
    void			write_reg(unsigned reg, uint8_t value);

    /**
     * Modify a register in the LSM9DS1_MAG
     *
     * Bits are cleared before bits are set.
     *
     * @param reg		The register to modify.
     * @param clearbits	Bits in the register to clear.
     * @param setbits	Bits in the register to set.
     */
    void			modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

    /**
     * Write a register in the LSM9DS1_MAG, updating _checked_values
     *
     * @param reg		The register to write.
     * @param value		The new value to write.
     */
    void			write_checked_reg(unsigned reg, uint8_t value);

    /**
     * Set the LSM9DS1_MAG measurement range.
     *
     * @param max_ga	The measurement range is set to permit reading at least
     *			this rate in gauss.
     *			Zero selects the maximum supported range.
     * @return		OK if the value can be supported, -ERANGE otherwise.
     */
    int			set_range(unsigned max_ga);

    /**
     * Set the LSM9DS1_MAG internal sampling frequency.
     *
     * @param frequency	The internal sampling frequency is set to not less than
     *			this value.
     *			Zero selects the maximum rate supported.
     * @return		OK if the value can be supported.
     */
    int			set_samplerate(unsigned frequency);


    /**
     * Self test
     *
     * @return 0 on success, 1 on failure
     */
    int 			self_test();

    /* this class does not allow copying */
    LSM9DS1_mag(const LSM9DS1_mag &);
    LSM9DS1_mag operator=(const LSM9DS1_mag &);
};

/*
  list of registers that will be checked in check_registers(). Note
  that ADDR_WHO_AM_I must be first in the list.
 */
const uint8_t LSM9DS1_mag::_checked_registers[LSM9DS1_MAG_NUM_CHECKED_REGISTERS] = {   WHO_AM_I_M,
                                                                                       CTRL_REG1_M,
                                                                                       CTRL_REG2_M,
                                                                                       CTRL_REG3_M,
                                                                                       CTRL_REG4_M,
                                                                                       CTRL_REG5_M,
                                                                                       INT_CFG_M
                                                                                   };


LSM9DS1_mag::LSM9DS1_mag(int bus, const char *path, spi_dev_e device, enum Rotation rotation) :
    SPI("LSM9DS1_MAG", path, bus, device, SPIDEV_MODE3, LSM9DS1_LOW_BUS_SPEED),
    _call{},
    _call_interval(0),
    _reports(nullptr),
    _mag_scale{},
    _mag_range_scale(0.0f),
    _mag_range_ga(0.0f),
    _mag_topic(nullptr),
    _orb_class_instance(-1),
    _class_instance(-1),
    _current_rate(0),
    _read(0),
    _sample_perf(perf_alloc(PC_ELAPSED, "lsm9ds1mag_read")),
    _errors(perf_alloc(PC_COUNT, "lsm9ds1mag_errors")),
    _bad_registers(perf_alloc(PC_COUNT, "lsm9ds1mag_bad_registers")),
    _duplicates(perf_alloc(PC_COUNT, "lsm9ds1mag_duplicates")),
    _register_wait(0),
    _rotation(rotation),
    _checked_next(0)
{
    // enable debug() calls
    _debug_enabled = true;

    _device_id.devid_s.devtype = DRV_MAG_DEVTYPE_LSM9DS1;

    // default scale factors
    _mag_scale.x_offset = 0;
    _mag_scale.x_scale  = 1.0f;
    _mag_scale.y_offset = 0;
    _mag_scale.y_scale  = 1.0f;
    _mag_scale.z_offset = 0;
    _mag_scale.z_scale  = 1.0f;
}

LSM9DS1_mag::~LSM9DS1_mag()
{
    /* make sure we are truly inactive */
    stop();

    /* free any existing reports */
    if (_reports != nullptr) {
        delete _reports;
    }

    if (_class_instance != -1) {
        unregister_class_devname(MAG_BASE_DEVICE_PATH, _class_instance);
    }

    /* delete the perf counter */
    perf_free(_sample_perf);
    perf_free(_errors);
    perf_free(_bad_registers);
    perf_free(_duplicates);
}

int
LSM9DS1_mag::init()
{
    int ret = ERROR;

    /* do SPI init (and probe) first */
    if (SPI::init() != OK) {
        goto out;
    }

    /* allocate basic report buffers */
    _reports = new ringbuffer::RingBuffer(2, sizeof(mag_report));

    if (_reports == nullptr) {
        goto out;
    }

    _class_instance = register_class_devname(MAG_BASE_DEVICE_PATH);

    reset();

    measure();

    /* advertise sensor topic, measure manually to initialize valid report */
    struct mag_report mrp;
    _reports->get(&mrp);

    _mag_topic = orb_advertise_multi(ORB_ID(sensor_mag), &mrp,
                      &_orb_class_instance, (is_external()) ? ORB_PRIO_VERY_HIGH : ORB_PRIO_DEFAULT);

    if (_mag_topic == nullptr) {
        DEVICE_DEBUG("failed to create sensor_mag publication");
    }

    ret = OK;
out:
    return ret;
}

int
LSM9DS1_mag::probe()
{
    /* read dummy value to void to clear SPI statemachine on sensor */
    (void)read_reg(WHO_AM_I_M);

    uint8_t v = 0;

    if ((v = read_reg(WHO_AM_I_M)) == LSM_WHOAMI_M) {
        _checked_values[0] = v;
        return OK;
    }

    return -EIO;
}


ssize_t
LSM9DS1_mag::read(struct file *filp, char *buffer, size_t buflen)
{
    unsigned count = buflen / sizeof(struct mag_report);
    struct mag_report *mbuf = reinterpret_cast<struct mag_report *>(buffer);
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
            if (_reports->get(mbuf)) {
                ret += sizeof(*mbuf);
                mbuf++;
            }
        }

        /* if there was no data, warn the caller */
        return ret ? ret : -EAGAIN;
    }

    /* manual measurement */
    _reports->flush();
    measure();

    /* measurement will have generated a report, copy it out */
    if (_reports->get(mbuf)) {
        ret = sizeof(*mbuf);
    }

    return ret;
}


int
LSM9DS1_mag::ioctl(struct file *filp, int cmd, unsigned long arg)
{
    switch (cmd) {

    case SENSORIOCSPOLLRATE: {
            switch (arg) {

            /* switching to manual polling */
            case SENSOR_POLLRATE_MANUAL:
                stop();
                _call_interval = 0;
                return OK;

            /* external signalling not supported */
            case SENSOR_POLLRATE_EXTERNAL:

            /* zero would be bad */
            case 0:
                return -EINVAL;

            /* set default/max polling rate */
            case SENSOR_POLLRATE_MAX:
            case SENSOR_POLLRATE_DEFAULT:

                return ioctl(filp, SENSORIOCSPOLLRATE, LSM9DS1_MAG_DEFAULT_RATE);

            /* adjust to a legal polling interval in Hz */
            default: {
                    /* do we need to start internal polling? */
                    bool want_start = (_call_interval == 0);

                    /* convert hz to hrt interval via microseconds */
                    unsigned ticks = 1000000 / arg;

                    /* check against maximum sane rate */
                    if (ticks < 1000) {
                        return -EINVAL;
                    }

                    /* update interval for next measurement */
                    /* XXX this is a bit shady, but no other way to adjust... */
                    _call_interval = ticks;

                    _call.period = _call_interval - LSM9DS1_TIMER_REDUCTION;

                    /* if we need to start the poll state machine, do it */
                    if (want_start) {
                        start();
                    }

                    return OK;
                }
            }
        }

    case SENSORIOCGPOLLRATE:
        if (_call_interval == 0) {
            return SENSOR_POLLRATE_MANUAL;
        }

        return 1000000 / _call_interval;

    case SENSORIOCSQUEUEDEPTH: {
            /* lower bound is mandatory, upper bound is a sanity check */
            if ((arg < 1) || (arg > 100)) {
                return -EINVAL;
            }

            irqstate_t flags = irqsave();

            if (!_reports->resize(arg)) {
                irqrestore(flags);
                return -ENOMEM;
            }

            irqrestore(flags);

            return OK;
        }

    case SENSORIOCGQUEUEDEPTH:
        return _reports->size();

    case SENSORIOCRESET:
        reset();
        return OK;

    case MAGIOCSSAMPLERATE:
        return set_samplerate(arg);

    case MAGIOCGSAMPLERATE:
        return _current_rate;

    case MAGIOCSLOWPASS:
    case MAGIOCGLOWPASS:
        /*not implemented yet*/
        return -EINVAL;

    case MAGIOCSSCALE:
        /* copy scale in */
        memcpy(&_mag_scale, (struct mag_scale *) arg, sizeof(_mag_scale));
        return OK;

    case MAGIOCGSCALE:
        /* copy scale out */
        memcpy((struct mag_scale *) arg, &_mag_scale, sizeof(_mag_scale));
        return OK;

    case MAGIOCSRANGE:
        /* arg should be in ga */
        return set_range(arg);

    case MAGIOCGRANGE:
        /* convert to dps and round */
        return _mag_range_ga;

    case MAGIOCSELFTEST:
        return self_test();

    default:
        /* give it to the superclass */
        return SPI::ioctl(filp, cmd, arg);
    }
}

uint8_t
LSM9DS1_mag::read_reg(unsigned reg)
{
    uint8_t cmd[2];

    cmd[0] = reg | DIR_READ;
    cmd[1] = 0;

    transfer(cmd, cmd, sizeof(cmd));

    return cmd[1];
}

void
LSM9DS1_mag::write_reg(unsigned reg, uint8_t value)
{
    uint8_t	cmd[2];

    cmd[0] = reg | DIR_WRITE;
    cmd[1] = value;

    transfer(cmd, nullptr, sizeof(cmd));
}

void
LSM9DS1_mag::write_checked_reg(unsigned reg, uint8_t value)
{
    write_reg(reg, value);

    for (uint8_t i = 0; i < LSM9DS1_MAG_NUM_CHECKED_REGISTERS; i++) {
        if (reg == _checked_registers[i]) {
            _checked_values[i] = value;
        }
    }
}


void
LSM9DS1_mag::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
    uint8_t	val;

    val = read_reg(reg);
    val &= ~clearbits;
    val |= setbits;
    write_checked_reg(reg, val);
}

int
LSM9DS1_mag::set_range(unsigned max_Ga)
{
    uint8_t setbits = 0;
    uint8_t clearbits = LSM_M_RANGE_16_G;

    float lsb_per_ga;
    float new_range;

    if (max_Ga == 0) {
        max_Ga = 16;
    }

    if (max_Ga <= 4) {
        new_range = 4;
        setbits |= LSM_M_RANGE_4_G;
        lsb_per_ga = 8192;

    } else if (max_Ga <= 8) {
        new_range = 8;
        setbits |= LSM_M_RANGE_8_G;
        lsb_per_ga = 4096;

    } else if (max_Ga <= 12) {
        new_range = 12;
        setbits |= LSM_M_RANGE_12_G;
        lsb_per_ga = 2730.67f;

    } else if (max_Ga > 12) {
        new_range = 16;
        setbits |= LSM_M_RANGE_16_G;
        lsb_per_ga = 2048;
    }
    else {
        return -EINVAL;
    }

    _mag_range_ga = new_range;
    _mag_range_scale = 1 / lsb_per_ga;

    modify_reg(CTRL_REG2_M, clearbits, setbits);

    return OK;
}

int
LSM9DS1_mag::set_samplerate(unsigned frequency)
{
    uint8_t setbits = 0;
    uint8_t clearbits = LSM_M_ODR_80_HZ;

    if (frequency == 0 || frequency == LSM9DS1_MAG_DEFAULT_RATE) {
        frequency = 80;
    }

    if (frequency <= 0.625f) {
        _current_rate = 0.625f;
        setbits |= LSM_M_ODR_0_625_HZ;

    } else if (frequency <= 1.25f) {
        _current_rate = 1.25f;
        setbits |= LSM_M_ODR_1_25_HZ;

    } else if (frequency <= 2.5f) {
        _current_rate = 2.5f;
        setbits |= LSM_M_ODR_2_5_HZ;

    } else if (frequency <= 5) {
        _current_rate = 5;
        setbits |= LSM_M_ODR_5_HZ;

    } else if (frequency <= 10) {
        _current_rate = 10;
        setbits |= LSM_M_ODR_10_HZ;

    } else if (frequency <= 20) {
        _current_rate = 20;
        setbits |= LSM_M_ODR_20_HZ;

    }else if (frequency <= 40) {
        _current_rate = 40;
        setbits |= LSM_M_ODR_40_HZ;

    }else if (frequency > 40) {
        _current_rate = 80;
        setbits |= LSM_M_ODR_80_HZ;
    }
    else {
        return -EINVAL;
    }

    modify_reg(CTRL_REG1_M, clearbits, setbits);

    return OK;
}


void
LSM9DS1_mag::start()
{
    /* make sure we are stopped first */
    stop();

    /* reset the report ring */
    _reports->flush();

    /* start polling at the specified rate */
    hrt_call_every(&_call,
               1000,
               _call_interval - LSM9DS1_TIMER_REDUCTION,
               (hrt_callout)&LSM9DS1_mag::measure_trampoline, this);
}

void
LSM9DS1_mag::stop()
{
    hrt_cancel(&_call);
}

void
LSM9DS1_mag::disable_i2c(void)
{
    modify_reg(CTRL_REG3_M, I2C_DISABLE_M, I2C_DISABLE_M);
}


void
LSM9DS1_mag::reset()
{
    // ensure the chip doesn't interpret any other bus traffic as I2C
    disable_i2c(); //Also done on the configuration... Just to ensure a correct configuration.

    /* set default configuration */
    write_checked_reg(CTRL_REG1_M,  TEMP_COMP | UHIGH_PER_M); //Temperature compensation and ultra-high performance for X and Y axes
    write_checked_reg(CTRL_REG2_M,  0);
    write_checked_reg(CTRL_REG3_M,  I2C_DISABLE_M | CONTINUOUS_MODE); //Disable I2C, continuous data update
    write_checked_reg(CTRL_REG4_M,  UHIGH_PER_M_Z);  //Ultra-high performance Z axis
    write_checked_reg(CTRL_REG5_M,  BDU_M); //Only update data when MSB and LSB were read
    write_checked_reg(INT_CFG_M,    ALL_AXES_INT_M_EN | IEA_M | IEL_M | IEN_M);

    set_samplerate(LSM9DS1_MAG_DEFAULT_RATE);
    set_range(LSM9DS1_MAG_DEFAULT_RANGE_GA);

    _read = 0;
}

void
LSM9DS1_mag::measure_trampoline(void *arg)
{
    LSM9DS1_mag *dev = (LSM9DS1_mag *)arg;

    /* make another measurement */
    dev->measure();
}

void
LSM9DS1_mag::check_registers(void)
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

    _checked_next = (_checked_next + 1) % LSM9DS1_MAG_NUM_CHECKED_REGISTERS;
}

void
LSM9DS1_mag::measure()
{
    /* status register and data as read back from the device */
#pragma pack(push, 1)
    struct {
        uint8_t		cmd;
        //int8_t		temp;
        uint8_t		status;
        int16_t		x;
        int16_t		y;
        int16_t		z;
    } raw_report;
#pragma pack(pop)

    mag_report report;

    /* start the performance counter */
    perf_begin(_sample_perf);

    check_registers();

    /* fetch data from the sensor */
    memset(&raw_report, 0, sizeof(raw_report));
    raw_report.cmd = STATUS_REG_M | DIR_READ | ADDR_INCR;
    transfer((uint8_t *)&raw_report, (uint8_t *)&raw_report, sizeof(raw_report));

    if (!(raw_report.status && STATUS_ZYXDA)) {
        perf_end(_sample_perf);
        perf_count(_duplicates);
        return;
    }

    report.timestamp = hrt_absolute_time();
    report.error_count = perf_event_count(_bad_registers);

    report.x_raw = ((raw_report.x == -32768) ? 32767 : -raw_report.x);
    report.y_raw = ((raw_report.y == -32768) ? 32767 : -raw_report.y);
    report.z_raw = raw_report.z;

    float xraw_f = report.x_raw;
    float yraw_f = report.y_raw;
    float zraw_f = report.z_raw;

    // apply user specified rotation
    rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

    report.x = ((xraw_f * _mag_range_scale) - _mag_scale.x_offset) * _mag_scale.x_scale;
    report.y = ((yraw_f * _mag_range_scale) - _mag_scale.y_offset) * _mag_scale.y_scale;
    report.z = ((zraw_f * _mag_range_scale) - _mag_scale.z_offset) * _mag_scale.z_scale;


    report.scaling = _mag_range_scale;
    report.range_ga = (float)_mag_range_ga;

    _reports->force(&report);

    /* notify anyone waiting for data */
    poll_notify(POLLIN);

    /* publish for subscribers */
    if (!(_pub_blocked)) {
        /* publish it */
        orb_publish(ORB_ID(sensor_mag), _mag_topic, &report);
    }

    _read++;

    /* stop the perf counter */
    perf_end(_sample_perf);
}


void
LSM9DS1_mag::print_info()
{
    printf("mag reads:          %u\n", _read);
    perf_print_counter(_sample_perf);
    perf_print_counter(_errors);
    perf_print_counter(_bad_registers);
    perf_print_counter(_duplicates);
    _reports->print_info("report queue");
    ::printf("checked_next: %u\n", _checked_next);

    for (uint8_t i = 0; i < LSM9DS1_MAG_NUM_CHECKED_REGISTERS; i++) {
        uint8_t v = read_reg(_checked_registers[i]);

        if (v != _checked_values[i]) {
            ::printf("reg %02x:%02x should be %02x\n",
                 (unsigned)_checked_registers[i],
                 (unsigned)v,
                 (unsigned)_checked_values[i]);
        }
    }
}

void
LSM9DS1_mag::print_registers()
{
    printf("LSM9DS1_MAG registers\n");

    for (uint8_t reg = 0x0F; reg <= 0x33; reg++) {
        uint8_t v = read_reg(reg);
        printf("%02x:%02x ", (unsigned)reg, (unsigned)v);

        if ((reg + 1) % 16 == 0) {
            printf("\n");
        }
    }

    printf("\n");
}

void
LSM9DS1_mag::test_error()
{
    // trigger a deliberate error
    write_reg(CTRL_REG3_M, (1<<2) | (1<<3));
}

int
LSM9DS1_mag::self_test()
{
    /* evaluate mag offsets, complain if offset -> zero or larger than 2 gauss */
    if (fabsf(_mag_scale.x_offset) > 2 || fabsf(_mag_scale.x_offset) < 0.000001f) {
        return 1;
    }

    if (fabsf(_mag_scale.x_scale - 1.0f) > 0.3f) {
        return 1;
    }

    if (fabsf(_mag_scale.y_offset) > 2 || fabsf(_mag_scale.y_offset) < 0.000001f) {
        return 1;
    }

    if (fabsf(_mag_scale.y_scale - 1.0f) > 0.3f) {
        return 1;
    }

    if (fabsf(_mag_scale.z_offset) > 2 || fabsf(_mag_scale.z_offset) < 0.000001f) {
        return 1;
    }

    if (fabsf(_mag_scale.z_scale - 1.0f) > 0.3f) {
        return 1;
    }

    return 0;
}


/**
 * Local functions in support of the shell command.
 */
namespace lsm9ds1mag
{

LSM9DS1_mag	*m_dev;

void	usage();
void	start(bool external_bus, enum Rotation rotation);
void	test();
void	reset();
void	info();
void	regdump();
void	test_error();

/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * started or failed to detect the sensor.
 */
void
start(bool external_bus, enum Rotation rotation)
{
    int fd;

    if (m_dev != nullptr) {
        errx(0, "already started");
    }

    /* create the driver */
    if (external_bus) {
#ifdef PX4_SPI_BUS_EXT
        m_dev = new LSM9DS1_mag(PX4_SPI_BUS_EXT, LSM_DEVICE_PATH_MAG_EXT, (spi_dev_e)PX4_SPIDEV_EXT_MAG, rotation);
#else
        errx(0, "External SPI not available");
#endif

    } else {
        m_dev = new LSM9DS1_mag(PX4_SPI_BUS_SENSORS, LSM_DEVICE_PATH_MAG, (spi_dev_e)PX4_SPIDEV_MAG, rotation);
    }

    if (m_dev == nullptr) {
        goto fail;
    }

    if (OK != m_dev->init()) {
        goto fail;
    }

    /* set the poll rate to default, starts automatic data collection */
    fd = open(LSM_DEVICE_PATH_MAG, O_RDONLY);

    if (fd < 0) {
        goto fail;
    }

    if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        goto fail;
    }

    close(fd);

    exit(0);
fail:

    if (m_dev != nullptr) {
        delete m_dev;
        m_dev = nullptr;
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
    int fd_mag = -1;
    struct mag_report m_report;
    ssize_t sz;

    /* get the driver */
    fd_mag = open(LSM_DEVICE_PATH_MAG, O_RDONLY);

    if (fd_mag < 0) {
        err(1, "%s open failed", LSM_DEVICE_PATH_MAG);
    }

    /* reset to manual polling */
    if (ioctl(fd_mag, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MANUAL) < 0) {
        err(1, "reset to manual polling");
    }

    /* do a simple demand read */
    sz = read(fd_mag, &m_report, sizeof(m_report));

    if (sz != sizeof(m_report)) {
        err(1, "immediate mag read failed");
    }

    warnx("mag x: \t% 9.5f\tGauss", (double)m_report.x);
    warnx("mag y: \t% 9.5f\tGauss", (double)m_report.y);
    warnx("mag z: \t% 9.5f\tGauss", (double)m_report.z);
    warnx("mag x: \t%d\traw", (int)m_report.x_raw);
    warnx("mag y: \t%d\traw", (int)m_report.y_raw);
    warnx("mag z: \t%d\traw", (int)m_report.z_raw);
    warnx("mag range: %8.4f\tGauss", (double)m_report.range_ga);

    if (ioctl(fd_mag, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        err(1, "reset to default polling");
    }

    close(fd_mag);

    /* XXX add poll-rate tests here too */
    errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
    int fd = open(LSM_DEVICE_PATH_MAG, O_RDONLY);

    if (fd < 0) {
        err(1, "failed ");
    }

    if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
        err(1, "driver reset failed");
    }

    if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        err(1, "mag pollrate reset failed");
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
    if (m_dev == nullptr) {
        errx(1, "driver not running\n");
    }

    printf("state @ %p\n", m_dev);
    m_dev->print_info();

    exit(0);
}


/**
 * Dump the register information
 */
void
regdump(void)
{
    if (m_dev == nullptr) {
        errx(1, "driver not running");
    }

    printf("regdump @ %p\n", m_dev);
    m_dev->print_registers();

    exit(0);
}

/**
 * trigger an error
 */
void
test_error(void)
{
    if (m_dev == nullptr) {
        errx(1, "driver not running");
    }

    printf("regdump @ %p\n", m_dev);
    m_dev->test_error();

    exit(0);
}


void
usage()
{
    warnx("missing command: try 'start', 'info', 'test', 'reset', 'testerror' or 'regdump'");
    warnx("options:");
    warnx("    -X    (external bus)");
    warnx("    -R rotation");
}

} // namespace


int
lsm9ds1mag_main(int argc, char *argv[])
{
    bool external_bus = false;
    int ch;
    enum Rotation rotation = ROTATION_NONE;

    /* jump over start/off/etc and look at options first */
    while ((ch = getopt(argc, argv, "XR:")) != EOF) {
        switch (ch) {
        case 'X':
            external_bus = true;
            break;

        case 'R':
            rotation = (enum Rotation)atoi(optarg);
            break;

        default:
            lsm9ds1mag::usage();
            exit(0);
        }
    }

    const char *verb = argv[optind];

    /*
     * Start/load the driver.

     */
    if (!strcmp(verb, "start")) {
        lsm9ds1mag::start(external_bus, rotation);
    }

    /*
     * Test the driver/device.
     */
    if (!strcmp(verb, "test")) {
        lsm9ds1mag::test();
    }

    /*
     * Reset the driver.
     */
    if (!strcmp(verb, "reset")) {
        lsm9ds1mag::reset();
    }

    /*
     * Print driver information.
     */
    if (!strcmp(verb, "info")) {
        lsm9ds1mag::info();
    }

    /*
     * Print register information.
     */
    if (!strcmp(verb, "regdump")) {
        lsm9ds1mag::regdump();
    }

    /*
     * trigger an error
     */
    if (!strcmp(verb, "testerror")) {
        lsm9ds1mag::test_error();
    }

    errx(1, "unrecognized command, try 'start', 'test', 'reset', 'info', 'testerror' or 'regdump'");
}


