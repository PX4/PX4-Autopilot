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
 * Driver for the STM LSM9DS1 (accelerometer and gyroscope) connected via SPI.
 *
 * @author Fábio Azevedo
 *
 * based on the mpu9250 driver
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
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#define DIR_READ			0x80
#define DIR_WRITE			0x00

#define LSM9DS1_DEVICE_PATH_ACCEL		"/dev/lsm9ds1_accel"
#define LSM9DS1_DEVICE_PATH_GYRO		"/dev/lsm9ds1_gyro"
#define LSM9DS1_DEVICE_PATH_ACCEL_EXT   "/dev/lsm9ds1_accel_ext"
#define LSM9DS1_DEVICE_PATH_GYRO_EXT    "/dev/lsm9ds1_gyro_ext"


/* Orientation on board */
#define SENSOR_BOARD_ROTATION_000_DEG	0
#define SENSOR_BOARD_ROTATION_090_DEG	1
#define SENSOR_BOARD_ROTATION_180_DEG	2
#define SENSOR_BOARD_ROTATION_270_DEG	3

// LSM9DS1 registers accel+gyro
#define WHO_AM_I                0x0F
#define ACT_THS                 0x04
#define ACT_DUR                 0x05
#define INT_GEN_CFG_XL          0x06
#define INT_GEN_THS_X_XL        0x07
#define INT_GEN_THS_Y_XL        0x08
#define INT_GEN_THS_Z_XL        0x09
#define INT_GEN_DUR_XL          0x0A
#define REFERENCE_G             0x0B
#define INT1_CTRL               0x0C
#define INT2_CTRL               0x0D
#define CTRL_REG1_G             0x10
#define CTRL_REG2_G             0x11
#define CTRL_REG3_G             0x12
#define ORIENT_CFG_G            0x13
#define INT_GEN_SRC_G           0x14
#define OUT_TEMP_L              0x15
#define OUT_TEMP_H              0x16
#define STATUS_REG1             0x17
#define OUT_X_L_G               0x18
#define OUT_X_H_G               0x19
#define OUT_Y_L_G               0x1A
#define OUT_Y_H_G               0x1B
#define OUT_Z_L_G               0x1C
#define OUT_Z_H_G               0x1D
#define CTRL_REG4               0x1E
#define CTRL_REG5_XL            0x1F
#define CTRL_REG6_XL            0x20
#define CTRL_REG7_XL            0x21
#define CTRL_REG8               0x22
#define CTRL_REG9               0x23
#define CTRL_REG10              0x24
#define INT_GEN_SRC_XL          0x26
#define STATUS_REG2             0x27
#define OUT_X_L_XL              0x28
#define OUT_X_H_XL              0x29
#define OUT_Y_L_XL              0x2A
#define OUT_Y_H_XL              0x2B
#define OUT_Z_L_XL              0x2C
#define OUT_Z_H_XL              0x2D
#define FIFO_CTRL               0x2E
#define FIFO_SRC                0x2F
#define INT_GEN_CFG_G           0x30
#define INT_GEN_THS_XH_G        0x31
#define INT_GEN_THS_XL_G        0x32
#define INT_GEN_THS_YH_G        0x33
#define INT_GEN_THS_YL_G        0x34
#define INT_GEN_THS_ZH_G        0x35
#define INT_GEN_THS_ZL_G        0x36
#define INT_GEN_DUR_G           0x37

//Index XL is equivalent to A --> accelerometer

//Accelerometer and gyroscope ODR CTRL_REG1_G
#define LSM_AG_ODR_14_9_HZ      (0<<7) | (0<<6) | (1<<5)
#define LSM_AG_ODR_59_5_HZ      (0<<7) | (1<<6) | (0<<5)
#define LSM_AG_ODR_119_HZ       (0<<7) | (1<<6) | (1<<5)
#define LSM_AG_ODR_238_HZ       (1<<7) | (0<<6) | (0<<5)
#define LSM_AG_ODR_476_HZ       (1<<7) | (0<<6) | (1<<5)
#define LSM_AG_ODR_952_HZ       (1<<7) | (1<<6) | (0<<5)

//Accelerometer only ODR CTRL_REG6_XL
#define LSM_A_ODR_10_HZ         (0<<7) | (0<<6) | (1<<5)
#define LSM_A_ODR_50_HZ         (0<<7) | (1<<6) | (0<<5)
#define LSM_A_ODR_119_HZ        (0<<7) | (1<<6) | (1<<5)
#define LSM_A_ODR_238_HZ        (1<<7) | (0<<6) | (0<<5)
#define LSM_A_ODR_476_HZ        (1<<7) | (0<<6) | (1<<5)
#define LSM_A_ODR_952_HZ        (1<<7) | (1<<6) | (0<<5)

//Gyroscope range CTRL_REG1_G
#define LSM_G_RANGE_245_DPS     (0<<4) | (0<<3)
#define LSM_G_RANGE_500_DPS     (0<<4) | (1<<3)
#define LSM_G_RANGE_2000_DPS    (1<<4) | (1<<3)

//Accelerometer range CTRL_REG6_XL
#define LSM_A_RANGE_2_g         (0<<4) | (0<<3)
#define LSM_A_RANGE_4_g         (1<<4) | (0<<3)
#define LSM_A_RANGE_8_g         (1<<4) | (1<<3)
#define LSM_A_RANGE_16_g        (0<<4) | (1<<3)

//Magnetometer range CTRL_REG3_M
#define LSM_M_RANGE_4_G         (0<<6) | (0<<5)
#define LSM_M_RANGE_8_G         (0<<6) | (1<<5)
#define LSM_M_RANGE_12_G        (1<<6) | (0<<5)
#define LSM_M_RANGE_16_G        (1<<6) | (1<<5)


//--CONFIGURATION BITS---//

//INT1_CTRL - Interrupt 1
#define DRDY_ACCEL              (1<<6) | (1<<0)
#define DRDY_GYRO               (1<<7) | (1<<1)

//CTRL_REG1_G - Gyro ODR, Scale and Bandwidth (pin 2 must be forced to 0)
#define CTRL_REG1_G_INIT        ~(1<<2)

//CTRL_REG2_G - (pins 4-7 must be forced to 0)
#define CTRL_REG2_G_INIT        ~((1<<7) | (1<<6) | (1<<5) | (1<<4))

//CTRL_REG3_G - (pins 4 and 5 must be forced to 0)
#define CTRL_REG3_G_INIT        ~((1<<5) | (1<<4))
#define LP_MODE                 (1<<7)
#define HP_EN                   (1<<6)
#define HPCF_G_0                (0<<3) | (0<<2) | (0<<1) | (0<<0)
#define HPCF_G_1                (0<<3) | (0<<2) | (0<<1) | (1<<0)
#define HPCF_G_2                (0<<3) | (0<<2) | (1<<1) | (0<<0)
#define HPCF_G_3                (0<<3) | (0<<2) | (1<<1) | (1<<0)
#define HPCF_G_4                (0<<3) | (1<<2) | (0<<1) | (0<<0)
#define HPCF_G_5                (0<<3) | (1<<2) | (0<<1) | (1<<0)
#define HPCF_G_6                (0<<3) | (1<<2) | (1<<1) | (0<<0)
#define HPCF_G_7                (0<<3) | (1<<2) | (1<<1) | (1<<0)
#define HPCF_G_8                (1<<3) | (0<<2) | (0<<1) | (0<<0)
#define HPCF_G_9                (1<<3) | (0<<2) | (0<<1) | (1<<0)

//ORIENT_CFG_G - orientation (pins 6 and 7 must be forced to 0)
#define ORIENT_CFG_G_INIT       ~((1<<7) | (1<<6))
#define SIGNX_G_P               (1<<5)
#define SIGNX_G_N               (0<<5)
#define SIGNY_G_P               (1<<4)
#define SIGNY_G_N               (0<<4)
#define SIGNZ_G_P               (1<<3)
#define SIGNZ_G_N               (0<<3)

//CTRL_REG4 - axis enable (pins 2,6 and 7 must be forced to 0)
#define CTRL_REG4_INIT          ~((1<<7) | (1<<6) | (1<<2))
#define ZEN_G                   (1<<5)
#define YEN_G                   (1<<4)
#define XEN_G                   (1<<3)
#define ALL_EN_G                (1<<5) | (1<<4) | (1<<3)

//CTRL_REG5_XL - decimation and axis enable (pins 0-2 must be forced to 0)
#define CTRL_REG5_XL_INIT       ~((1<<2) | (1<<1) | (1<<0))
#define DEC_0                   (0<<7) | (0<<6)
#define DEC_2                   (0<<7) | (1<<6)
#define DEC_4                   (1<<7) | (0<<6)
#define DEC_8                   (1<<7) | (1<<6)
#define ZEN_XL                  (1<<5)
#define YEN_XL                  (1<<4)
#define XEN_XL                  (1<<3)
#define ALL_EN_XL               (1<<5) | (1<<4) | (1<<3)

//CTRL_REG6_XL - Accel ODR, Scale and Bandwidth
#define BW_SCAL_ODR             (0<<2)
#define BW_SCAL_ODR_BW_XL       (1<<2)
#define BW_XL_408_HZ            (0<<1) | (0<<0)
#define BW_XL_211_HZ            (0<<1) | (1<<0)
#define BW_XL_105_HZ            (1<<1) | (0<<0)
#define BW_XL_50_HZ             (1<<1) | (1<<0)

//CTRL_REG7_XL - (pins 1,3 and 4 must be forced to 0)
#define CTRL_REG7_XL_INIT       ~((1<<4) | (1<<3) | (1<<1))
#define HR                      (1<<7)
#define DCF_50                  (0<<6) | (0<<5)
#define DCF_100                 (0<<6) | (1<<5)
#define DCF_9                   (1<<6) | (0<<5)
#define DCF_400                 (1<<6) | (1<<5)
#define FDS                     (1<<2)
#define HPIS1                   (1<<0)

//CTRL_REG8
#define BOOT                    (1<<7)
#define BDU                     (1<<6)
#define H_LACTIVE               (1<<5)
#define PP                      (0<<4)
#define OD                      (1<<4)
#define SPI_4_WIRE              (0<<3)
#define SPI_3_WIRE              (1<<3)
#define IF_ADD_INC              (1<<2)
#define BLE                     (1<<1)
#define SW_RESET                (1<<0)

//CTRL_REG9 - (pins 5 and 7 must be forced to 0)
#define CTRL_REG9_INIT          ~((1<<7) | (1<<5))
#define SLEEP_G                 (1<<6)
#define FIFO_TEMP_EN            (1<<4)
#define DRDY_MASK_BIT           (1<<3)
#define I2C_DISABLE             (1<<2)
#define FIFO_EN                 (1<<1)
#define STOP_ON_FTH             (1<<0)

//CTRL_REG10 - self-test (pins 1 and 3-7 must be forced to 0)
#define CTRL_REG10_INIT         ~((1<<7)| (1<<6) | (1<<5) | (1<<4) | (1<<3) | (1<<1))
#define ST_G                    (1<<2)
#define ST_XL                   (1<<0)

#define LSM_WHOAMI_AG                       0x68

/* default values for this device */
#define LSM9DS1_ACCEL_DEFAULT_RANGE_G		4
#define LSM9DS1_GYRO_DEFAULT_RANGE_DPS		245
#define LSM9DS1_AG_DEFAULT_RATE             952
#define LSM9DS1_AG_MAX_RATE                 952

#define LSM9DS1_AG_DEFAULT_ONCHIP_FILTER_FREQ	50
#define LSM9DS1_AG_DEFAULT_DRIVER_FILTER_FREQ	30

/*------------------------------------------------------------------*/

#define LSM9DS1_ONE_G                       9.80665f

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

class LSM9DS1_gyro;

class LSM9DS1 : public device::SPI
{
public:
    LSM9DS1(int bus, const char *path_accel, const char *path_gyro, spi_dev_e device, enum Rotation rotation);
    virtual ~LSM9DS1();

    virtual int		init();

    virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
    virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

    /**
     * Diagnostics - print some basic information about the driver.
     */
    void			print_info();

    void			print_registers();

    // deliberately cause a sensor error
    void 			test_error();

protected:
    virtual int		probe();

    friend class LSM9DS1_gyro;

    virtual ssize_t		gyro_read(struct file *filp, char *buffer, size_t buflen);
    virtual int		gyro_ioctl(struct file *filp, int cmd, unsigned long arg);

private:
    LSM9DS1_gyro		*_gyro;
    uint8_t			_whoami;	/** whoami result */

    struct hrt_call		_call;
    unsigned		_call_interval;

    ringbuffer::RingBuffer	*_accel_reports;

    struct accel_scale	_accel_scale;
    float			_accel_range_scale;
    float			_accel_range_m_s2;
    orb_advert_t		_accel_topic;
    int			_accel_orb_class_instance;
    int			_accel_class_instance;

    ringbuffer::RingBuffer	*_gyro_reports;

    struct gyro_scale	_gyro_scale;
    float			_gyro_range_scale;
    float			_gyro_range_rad_s;

    unsigned		_dlpf_freq;

    float		_accel_sample_rate;
    float		_gyro_sample_rate;
    perf_counter_t		_accel_reads;
    perf_counter_t		_gyro_reads;
    perf_counter_t		_sample_perf;
    perf_counter_t		_bad_transfers;
    perf_counter_t		_bad_registers;
    perf_counter_t		_good_transfers;
    perf_counter_t		_reset_retries;
    perf_counter_t		_duplicates;
    perf_counter_t		_controller_latency_perf;

    uint8_t			_register_wait;
    uint64_t		_reset_wait;

    math::LowPassFilter2p	_accel_filter_x;
    math::LowPassFilter2p	_accel_filter_y;
    math::LowPassFilter2p	_accel_filter_z;
    math::LowPassFilter2p	_gyro_filter_x;
    math::LowPassFilter2p	_gyro_filter_y;
    math::LowPassFilter2p	_gyro_filter_z;

    Integrator		_accel_int;
    Integrator		_gyro_int;

    enum Rotation		_rotation;

    // this is used to support runtime checking of key
    // configuration registers to detect SPI bus errors and sensor
    // reset
#define LSM9DS1_NUM_CHECKED_REGISTERS 13
    static const uint8_t	_checked_registers[LSM9DS1_NUM_CHECKED_REGISTERS];
    uint8_t			_checked_values[LSM9DS1_NUM_CHECKED_REGISTERS];
    uint8_t			_checked_bad[LSM9DS1_NUM_CHECKED_REGISTERS];
    uint8_t			_checked_next;

    // last temperature reading for print_info()
    float			_last_temperature;

    // keep last accel reading for duplicate detection
    uint16_t		_last_accel[3];
    bool			_got_duplicate;

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
    int			reset();

    /**
     * Static trampoline from the hrt_call context; because we don't have a
     * generic hrt wrapper yet.
     *
     * Called by the HRT in interrupt context at the specified rate if
     * automatic polling is enabled.
     *
     * @param arg		Instance pointer for the driver that is polling.
     */
    static void		measure_trampoline(void *arg);

    /**
     * Fetch measurements from the sensor and update the report buffers.
     */
    void			measure();

    /**
     * Read a register from the LSM9DS1
     *
     * @param		The register to read.
     * @return		The value that was read.
     */
    uint8_t			read_reg(unsigned reg, uint32_t speed = LSM9DS1_LOW_BUS_SPEED);
    uint16_t		read_reg16(unsigned reg);

    /**
     * Write a register in the LSM9DS1
     *
     * @param reg		The register to write.
     * @param value		The new value to write.
     */
    void			write_reg(unsigned reg, uint8_t value);

    /**
     * Modify a register in the LSM9DS1
     *
     * Bits are cleared before bits are set.
     *
     * @param reg		The register to modify.
     * @param clearbits	Bits in the register to clear.
     * @param setbits	Bits in the register to set.
     */
    void			modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

    /**
     * Write a register in the LSM9DS1, updating _checked_values
     *
     * @param reg		The register to write.
     * @param value		The new value to write.
     */
    void			write_checked_reg(unsigned reg, uint8_t value);

    /**
     * Set the LSM9DS1 measurement range.
     *
     * @param max_g		The maximum G value the range must support.
     * @param max_dps	The maximum DPS value the range must support.
     * @return		OK if the value can be supported, -ERANGE otherwise.
     */
    int			set_accel_range(unsigned max_g);
    int			set_gyro_range(unsigned max_dps);

    /**
     * Swap a 16-bit value read from the LSM9DS1 to native byte order.
     */
    uint16_t		swap16(uint16_t val) { return (val >> 8) | (val << 8);	}

    /**
     * Get the internal / external state
     *
     * @return true if the sensor is not on the main MCU board
     */
    bool			is_external() { return (_bus == EXTERNAL_BUS); }

    /**
     * Measurement self test
     *
     * @return 0 on success, 1 on failure
     */
    int 			self_test();

    /**
     * Accel self test
     *
     * @return 0 on success, 1 on failure
     */
    int 			accel_self_test();

    /**
     * Gyro self test
     *
     * @return 0 on success, 1 on failure
     */
    int 			gyro_self_test();

    /*
      set low pass filter frequency
     */
    void _set_dlpf_filter(uint16_t frequency_hz);

    /*
      set sample rate (approximate) - 10 - 952 Hz
    */
    int accel_set_sample_rate(float desired_sample_rate_hz);
    int gyro_set_sample_rate(float desired_sample_rate_hz);
    /*
      check that key registers still have the right value
     */
    void check_registers(void);

    /* do not allow to copy this class due to pointer data members */
    LSM9DS1(const LSM9DS1 &);
    LSM9DS1 operator=(const LSM9DS1 &);

#pragma pack(push, 1)
    /**
     * Report conversation within the LSM9DS1, including command byte and
     * interrupt status.
     */
    struct LSMReport {
        uint8_t		cmd;
        int16_t		gyro_x;
        int16_t		gyro_y;
        int16_t		gyro_z;
        int16_t		accel_x;
        int16_t		accel_y;
        int16_t		accel_z;
    };
#pragma pack(pop)
};

/*
  list of registers that will be checked in check_registers(). Note
  that LSM WHOAMI must be first in the list.
 */
const uint8_t LSM9DS1::_checked_registers[LSM9DS1_NUM_CHECKED_REGISTERS] = { WHO_AM_I,
                                                                             INT1_CTRL,
                                                                             INT2_CTRL,
                                                                             CTRL_REG1_G,
                                                                             CTRL_REG2_G,
                                                                             CTRL_REG3_G,
                                                                             CTRL_REG4,
                                                                             CTRL_REG5_XL,
                                                                             CTRL_REG6_XL,
                                                                             CTRL_REG7_XL,
                                                                             CTRL_REG8,
                                                                             CTRL_REG9,
                                                                             CTRL_REG10
                                                                             };

/**
 * Helper class implementing the gyro driver node.
 */
class LSM9DS1_gyro : public device::CDev
{
public:
    LSM9DS1_gyro(LSM9DS1 *parent, const char *path);
    ~LSM9DS1_gyro();

    virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
    virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

    virtual int		init();

protected:
    friend class LSM9DS1;

    void			parent_poll_notify();

private:
    LSM9DS1			*_parent;
    orb_advert_t		_gyro_topic;
    int			_gyro_orb_class_instance;
    int			_gyro_class_instance;

    /* do not allow to copy this class due to pointer data members */
    LSM9DS1_gyro(const LSM9DS1_gyro &);
    LSM9DS1_gyro operator=(const LSM9DS1_gyro &);
};

/** driver 'main' command */
extern "C" { __EXPORT int lsm9ds1_main(int argc, char *argv[]); }

LSM9DS1::LSM9DS1(int bus, const char *path_accel, const char *path_gyro, spi_dev_e device, enum Rotation rotation) :
    SPI("LSM9DS1", path_accel, bus, device, SPIDEV_MODE3, LSM9DS1_LOW_BUS_SPEED),
    _gyro(new LSM9DS1_gyro(this, path_gyro)),
    _whoami(0),
    _call{},
    _call_interval(0),
    _accel_reports(nullptr),
    _accel_scale{},
    _accel_range_scale(0.0f),
    _accel_range_m_s2(0.0f),
    _accel_topic(nullptr),
    _accel_orb_class_instance(-1),
    _accel_class_instance(-1),
    _gyro_reports(nullptr),
    _gyro_scale{},
    _gyro_range_scale(0.0f),
    _gyro_range_rad_s(0.0f),
    _dlpf_freq(0),
    _accel_sample_rate(LSM9DS1_AG_DEFAULT_RATE),
    _gyro_sample_rate(LSM9DS1_AG_DEFAULT_RATE),
    _accel_reads(perf_alloc(PC_COUNT, "lsm9ds1_accel_read")),
    _gyro_reads(perf_alloc(PC_COUNT, "lsm9ds1_gyro_read")),
    _sample_perf(perf_alloc(PC_ELAPSED, "lsm9ds1_read")),
    _bad_transfers(perf_alloc(PC_COUNT, "lsm9ds1_bad_transfers")),
    _bad_registers(perf_alloc(PC_COUNT, "lsm9ds1_bad_registers")),
    _good_transfers(perf_alloc(PC_COUNT, "lsm9ds1_good_transfers")),
    _reset_retries(perf_alloc(PC_COUNT, "lsm9ds1_reset_retries")),
    _duplicates(perf_alloc(PC_COUNT, "lsm9ds1_duplicates")),
    _controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency")),
    _register_wait(0),
    _reset_wait(0),
    _accel_filter_x(LSM9DS1_AG_DEFAULT_RATE, LSM9DS1_AG_DEFAULT_DRIVER_FILTER_FREQ),
    _accel_filter_y(LSM9DS1_AG_DEFAULT_RATE, LSM9DS1_AG_DEFAULT_DRIVER_FILTER_FREQ),
    _accel_filter_z(LSM9DS1_AG_DEFAULT_RATE, LSM9DS1_AG_DEFAULT_DRIVER_FILTER_FREQ),
    _gyro_filter_x(LSM9DS1_AG_DEFAULT_RATE, LSM9DS1_AG_DEFAULT_DRIVER_FILTER_FREQ),
    _gyro_filter_y(LSM9DS1_AG_DEFAULT_RATE, LSM9DS1_AG_DEFAULT_DRIVER_FILTER_FREQ),
    _gyro_filter_z(LSM9DS1_AG_DEFAULT_RATE, LSM9DS1_AG_DEFAULT_DRIVER_FILTER_FREQ),
    _accel_int(1000000 / LSM9DS1_AG_MAX_RATE),
    _gyro_int(1000000 / LSM9DS1_AG_MAX_RATE, true),
    _rotation(rotation),
    _checked_next(0),
    _last_temperature(0),
    _last_accel{},
    _got_duplicate(false)
{
    // disable debug() calls
    _debug_enabled = false;

    _device_id.devid_s.devtype = DRV_ACC_DEVTYPE_LSM9DS1;

    /* Prime _gyro with parents devid. */
    _gyro->_device_id.devid = _device_id.devid;
    _gyro->_device_id.devid_s.devtype = DRV_GYR_DEVTYPE_LSM9DS1;

    // default accel scale factors
    _accel_scale.x_offset = 0;
    _accel_scale.x_scale  = 1.0f;
    _accel_scale.y_offset = 0;
    _accel_scale.y_scale  = 1.0f;
    _accel_scale.z_offset = 0;
    _accel_scale.z_scale  = 1.0f;

    // default gyro scale factors
    _gyro_scale.x_offset = 0;
    _gyro_scale.x_scale  = 1.0f;
    _gyro_scale.y_offset = 0;
    _gyro_scale.y_scale  = 1.0f;
    _gyro_scale.z_offset = 0;
    _gyro_scale.z_scale  = 1.0f;

    memset(&_call, 0, sizeof(_call));
}

LSM9DS1::~LSM9DS1()
{
    /* make sure we are truly inactive */
    stop();

    /* delete the gyro subdriver */
    delete _gyro;

    /* free any existing reports */
    if (_accel_reports != nullptr) {
        delete _accel_reports;
    }

    if (_gyro_reports != nullptr) {
        delete _gyro_reports;
    }

    if (_accel_class_instance != -1) {
        unregister_class_devname(ACCEL_BASE_DEVICE_PATH, _accel_class_instance);
    }

    /* delete the perf counter */
    perf_free(_sample_perf);
    perf_free(_accel_reads);
    perf_free(_gyro_reads);
    perf_free(_bad_transfers);
    perf_free(_bad_registers);
    perf_free(_good_transfers);
    perf_free(_reset_retries);
    perf_free(_duplicates);
}

int
LSM9DS1::init()
{
    int ret;

    /* do SPI init (and probe) first */
    ret = SPI::init();

    /* if probe/setup failed, bail now */
    if (ret != OK) {
        DEVICE_DEBUG("SPI setup failed");
        return ret;
    }

    /* allocate basic report buffers */
    _accel_reports = new ringbuffer::RingBuffer(2, sizeof(accel_report));

    if (_accel_reports == nullptr) {
        goto out;
    }

    _gyro_reports = new ringbuffer::RingBuffer(2, sizeof(gyro_report));

    if (_gyro_reports == nullptr) {
        goto out;
    }

    if (reset() != OK) {
        goto out;
    }

    /* Initialize offsets and scales */
    _accel_scale.x_offset = 0;
    _accel_scale.x_scale  = 1.0f;
    _accel_scale.y_offset = 0;
    _accel_scale.y_scale  = 1.0f;
    _accel_scale.z_offset = 0;
    _accel_scale.z_scale  = 1.0f;

    _gyro_scale.x_offset = 0;
    _gyro_scale.x_scale  = 1.0f;
    _gyro_scale.y_offset = 0;
    _gyro_scale.y_scale  = 1.0f;
    _gyro_scale.z_offset = 0;
    _gyro_scale.z_scale  = 1.0f;


    /* do CDev init for the gyro device node, keep it optional */
    ret = _gyro->init();

    /* if probe/setup failed, bail now */
    if (ret != OK) {
        DEVICE_DEBUG("gyro init failed");
        return ret;
    }

    _accel_class_instance = register_class_devname(ACCEL_BASE_DEVICE_PATH);

    measure();

    /* advertise sensor topic, measure manually to initialize valid report */
    struct accel_report arp;
    _accel_reports->get(&arp);

    /* measurement will have generated a report, publish */
    _accel_topic = orb_advertise_multi(ORB_ID(sensor_accel), &arp,
                       &_accel_orb_class_instance, (is_external()) ? ORB_PRIO_MAX - 1 : ORB_PRIO_HIGH - 1);

    if (_accel_topic == nullptr) {
        warnx("ADVERT FAIL");
    }


    /* advertise sensor topic, measure manually to initialize valid report */
    struct gyro_report grp;
    _gyro_reports->get(&grp);

    _gyro->_gyro_topic = orb_advertise_multi(ORB_ID(sensor_gyro), &grp,
                 &_gyro->_gyro_orb_class_instance, (is_external()) ? ORB_PRIO_MAX - 1 : ORB_PRIO_HIGH - 1);

    if (_gyro->_gyro_topic == nullptr) {
        warnx("ADVERT FAIL");
    }

out:
    return ret;
}


int LSM9DS1::reset()
{
    write_checked_reg(INT1_CTRL, DRDY_ACCEL | DRDY_GYRO);
    write_checked_reg(INT2_CTRL, 0);
    write_checked_reg(CTRL_REG1_G, 0);
    write_checked_reg(CTRL_REG2_G, 0);
    write_checked_reg(CTRL_REG3_G, 0);
    write_checked_reg(CTRL_REG4, ALL_EN_G);
    write_checked_reg(CTRL_REG5_XL, ALL_EN_XL);
    write_checked_reg(CTRL_REG6_XL, 0);
    write_checked_reg(CTRL_REG7_XL, 0);
    write_checked_reg(CTRL_REG8, (BDU | IF_ADD_INC));
    write_checked_reg(CTRL_REG9, (DRDY_MASK_BIT | I2C_DISABLE));
    write_checked_reg(CTRL_REG10, 0);

    set_accel_range(LSM9DS1_ACCEL_DEFAULT_RANGE_G);
    accel_set_sample_rate(LSM9DS1_AG_DEFAULT_RATE);

    set_gyro_range(LSM9DS1_GYRO_DEFAULT_RANGE_DPS);
    gyro_set_sample_rate(LSM9DS1_AG_DEFAULT_RATE);

    //_set_dlpf_filter(LSM9DS1_AG_DEFAULT_ONCHIP_FILTER_FREQ);

    uint8_t retries = 10;

    while (retries--) {
        bool all_ok = true;

        for (uint8_t i = 0; i < LSM9DS1_NUM_CHECKED_REGISTERS; i++) {
            if (read_reg(_checked_registers[i]) != _checked_values[i]) {
                write_reg(_checked_registers[i], _checked_values[i]);
                all_ok = false;
            }
        }

        if (all_ok) {
            break;
        }
    }

    _accel_reads = 0;
    _gyro_reads = 0;

    return OK;
}

int
LSM9DS1::probe()
{
    /* look for device ID */
    _whoami = read_reg(WHO_AM_I);

    // verify product revision
    switch (_whoami) {
    case LSM_WHOAMI_AG:
        memset(_checked_values, 0, sizeof(_checked_values));
        memset(_checked_bad, 0, sizeof(_checked_bad));
        _checked_values[0] = _whoami;
        _checked_bad[0] = _whoami;
        return OK;
    }

    DEVICE_DEBUG("unexpected whoami 0x%02x", _whoami);
    return -EIO;
}

int
LSM9DS1::accel_set_sample_rate(float frequency)
{
    uint8_t setbits = 0;
    uint8_t clearbits = (LSM_A_ODR_952_HZ | (1<<5));

    if ((int)frequency == 0) {
        frequency = 952.0f;
    }

    if (frequency <= 10.0f) {
        setbits |= LSM_A_ODR_10_HZ;
        _accel_sample_rate = 10;

    } else if (frequency <= 50.0f) {
        setbits |= LSM_A_ODR_50_HZ;
        _accel_sample_rate = 50;

    } else if (frequency <= 119.0f) {
        setbits |= LSM_A_ODR_119_HZ;
        _accel_sample_rate = 119;

    } else if (frequency <= 238.0f) {
        setbits |= LSM_A_ODR_238_HZ;
        _accel_sample_rate = 238;

    } else if (frequency <= 476.0f) {
        setbits |= LSM_A_ODR_476_HZ;
        _accel_sample_rate = 476;

    } else if (frequency > 476.0f) {
        setbits |= LSM_A_ODR_952_HZ;
        _accel_sample_rate = 952;

    } else {
        return -EINVAL;
    }

    modify_reg(CTRL_REG6_XL, clearbits, setbits);

    return OK;
}

int
LSM9DS1::gyro_set_sample_rate(float frequency)
{
    uint8_t setbits = 0;
    uint8_t clearbits = (LSM_AG_ODR_952_HZ | (1<<5));

    if ((int)frequency == 0) {
        frequency = 952;
    }

    if (frequency <= 14.9f) {
        setbits |= LSM_AG_ODR_14_9_HZ;
        _gyro_sample_rate = 14.9f;

    } else if (frequency <= 59.5f) {
        setbits |= LSM_AG_ODR_59_5_HZ;
        _gyro_sample_rate = 59.5f;

    } else if (frequency <= 119.0f) {
        setbits |= LSM_AG_ODR_119_HZ;
        _gyro_sample_rate = 119;

    } else if (frequency <= 238.0f) {
        setbits |= LSM_AG_ODR_238_HZ;
        _gyro_sample_rate = 238;

    } else if (frequency <= 476.0f) {
        setbits |= LSM_AG_ODR_476_HZ;
        _gyro_sample_rate = 476;

    } else if (frequency > 476.0f) {
        setbits |= LSM_AG_ODR_952_HZ;
        _gyro_sample_rate = 952;

    } else {
        return -EINVAL;
    }

    modify_reg(CTRL_REG1_G, clearbits, setbits);

    return OK;
}

void
LSM9DS1::_set_dlpf_filter(uint16_t bandwidth)
{
    uint8_t setbits = BW_SCAL_ODR_BW_XL;
    uint8_t clearbits = BW_XL_50_HZ;

    if (bandwidth == 0) {
        _dlpf_freq = 408;
        clearbits = BW_SCAL_ODR_BW_XL | BW_XL_50_HZ;
        setbits = 0;
    }

    if (bandwidth <= 50) {
        setbits |= BW_XL_50_HZ;
        _dlpf_freq = 50;

    } else if (bandwidth <= 105) {
        setbits |= BW_XL_105_HZ;
        _dlpf_freq = 105;

    } else if (bandwidth <= 211) {
        setbits |= BW_XL_211_HZ;
        _dlpf_freq = 211;

    } else if (bandwidth <= 408) {
        setbits |= BW_XL_408_HZ;
        _dlpf_freq = 408;

    }
    modify_reg(CTRL_REG6_XL, clearbits, setbits);

}

ssize_t
LSM9DS1::read(struct file *filp, char *buffer, size_t buflen)
{
    unsigned count = buflen / sizeof(accel_report);

    /* buffer must be large enough */
    if (count < 1) {
        return -ENOSPC;
    }

    /* if automatic measurement is not enabled, get a fresh measurement into the buffer */
    if (_call_interval == 0) {
        _accel_reports->flush();
        measure();
    }

    /* if no data, error (we could block here) */
    if (_accel_reports->empty()) {
        return -EAGAIN;
    }

    perf_count(_accel_reads);

    /* copy reports out of our buffer to the caller */
    accel_report *arp = reinterpret_cast<accel_report *>(buffer);
    int transferred = 0;

    while (count--) {
        if (!_accel_reports->get(arp)) {
            break;
        }

        transferred++;
        arp++;
    }

    /* return the number of bytes transferred */
    return (transferred * sizeof(accel_report));
}

int
LSM9DS1::self_test()
{
    if (perf_event_count(_sample_perf) == 0) {
        measure();
    }

    /* return 0 on success, 1 else */
    return (perf_event_count(_sample_perf) > 0) ? 0 : 1;
}

int
LSM9DS1::accel_self_test()
{
    if (self_test()) {
        return 1;
    }

    /* inspect accel offsets */
    if (fabsf(_accel_scale.x_offset) < 0.000001f) {
        return 1;
    }

    if (fabsf(_accel_scale.x_scale - 1.0f) > 0.4f || fabsf(_accel_scale.x_scale - 1.0f) < 0.000001f) {
        return 1;
    }

    if (fabsf(_accel_scale.y_offset) < 0.000001f) {
        return 1;
    }

    if (fabsf(_accel_scale.y_scale - 1.0f) > 0.4f || fabsf(_accel_scale.y_scale - 1.0f) < 0.000001f) {
        return 1;
    }

    if (fabsf(_accel_scale.z_offset) < 0.000001f) {
        return 1;
    }

    if (fabsf(_accel_scale.z_scale - 1.0f) > 0.4f || fabsf(_accel_scale.z_scale - 1.0f) < 0.000001f) {
        return 1;
    }

    return 0;
}

int
LSM9DS1::gyro_self_test()
{
    if (self_test()) {
        return 1;
    }

    /*
     * Maximum deviation of 30 degrees
     */
    const float max_offset = (float)(30 * M_PI_F / 180.0f);
    /* 30% scale error is chosen to catch completely faulty units but
     * to let some slight scale error pass. Requires a rate table or correlation
     * with mag rotations + data fit to
     * calibrate properly and is not done by default.
     */
    const float max_scale = 0.3f;

    /* evaluate gyro offsets, complain if offset -> zero or larger than 30 dps. */
    if (fabsf(_gyro_scale.x_offset) > max_offset) {
        return 1;
    }

    /* evaluate gyro scale, complain if off by more than 30% */
    if (fabsf(_gyro_scale.x_scale - 1.0f) > max_scale) {
        return 1;
    }

    if (fabsf(_gyro_scale.y_offset) > max_offset) {
        return 1;
    }

    if (fabsf(_gyro_scale.y_scale - 1.0f) > max_scale) {
        return 1;
    }

    if (fabsf(_gyro_scale.z_offset) > max_offset) {
        return 1;
    }

    if (fabsf(_gyro_scale.z_scale - 1.0f) > max_scale) {
        return 1;
    }

    /* check if all scales are zero */
    if ((fabsf(_gyro_scale.x_offset) < 0.000001f) &&
        (fabsf(_gyro_scale.y_offset) < 0.000001f) &&
        (fabsf(_gyro_scale.z_offset) < 0.000001f)) {
        /* if all are zero, this device is not calibrated */
        return 1;
    }

    return 0;
}

/*
  deliberately trigger an error in the sensor to trigger recovery
 */
void
LSM9DS1::test_error()
{
    write_reg(CTRL_REG8, SW_RESET);
    ::printf("error triggered\n");
    print_registers();
}

ssize_t
LSM9DS1::gyro_read(struct file *filp, char *buffer, size_t buflen)
{
    unsigned count = buflen / sizeof(gyro_report);

    /* buffer must be large enough */
    if (count < 1) {
        return -ENOSPC;
    }

    /* if automatic measurement is not enabled, get a fresh measurement into the buffer */
    if (_call_interval == 0) {
        _gyro_reports->flush();
        measure();
    }

    /* if no data, error (we could block here) */
    if (_gyro_reports->empty()) {
        return -EAGAIN;
    }

    perf_count(_gyro_reads);

    /* copy reports out of our buffer to the caller */
    gyro_report *grp = reinterpret_cast<gyro_report *>(buffer);
    int transferred = 0;

    while (count--) {
        if (!_gyro_reports->get(grp)) {
            break;
        }

        transferred++;
        grp++;
    }

    /* return the number of bytes transferred */
    return (transferred * sizeof(gyro_report));
}


int
LSM9DS1::ioctl(struct file *filp, int cmd, unsigned long arg)
{
    switch (cmd) {

    case SENSORIOCRESET:
        return reset();

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
                return ioctl(filp, SENSORIOCSPOLLRATE, LSM9DS1_AG_MAX_RATE);

            case SENSOR_POLLRATE_DEFAULT:
                return ioctl(filp, SENSORIOCSPOLLRATE, LSM9DS1_AG_DEFAULT_RATE);

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

                    // adjust filters
                    float cutoff_freq_hz = _accel_filter_x.get_cutoff_freq();
                    float sample_rate = 1.0e6f / ticks;
                    _set_dlpf_filter(cutoff_freq_hz);
                    _accel_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
                    _accel_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
                    _accel_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz);


                    float cutoff_freq_hz_gyro = _gyro_filter_x.get_cutoff_freq();
                    _set_dlpf_filter(cutoff_freq_hz_gyro);
                    _gyro_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
                    _gyro_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
                    _gyro_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);

                    /* update interval for next measurement */
                    /* XXX this is a bit shady, but no other way to adjust... */
                    _call_interval = ticks;

                    /*
                      set call interval faster then the sample time. We
                      then detect when we have duplicate samples and reject
                      them. This prevents aliasing due to a beat between the
                      stm32 clock and the lsm9ds1 clock
                     */
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

            if (!_accel_reports->resize(arg)) {
                irqrestore(flags);
                return -ENOMEM;
            }

            irqrestore(flags);

            return OK;
        }

    case SENSORIOCGQUEUEDEPTH:
        return _accel_reports->size();

    case ACCELIOCGSAMPLERATE:
        return _accel_sample_rate;

    case ACCELIOCSSAMPLERATE:
        return accel_set_sample_rate(arg);

    case ACCELIOCGLOWPASS:
        return _accel_filter_x.get_cutoff_freq();

    case ACCELIOCSLOWPASS:
        // set software filtering
        _accel_filter_x.set_cutoff_frequency(1.0e6f / _call_interval, arg);
        _accel_filter_y.set_cutoff_frequency(1.0e6f / _call_interval, arg);
        _accel_filter_z.set_cutoff_frequency(1.0e6f / _call_interval, arg);
        return OK;

    case ACCELIOCSSCALE: {
            /* copy scale, but only if off by a few percent */
            struct accel_scale *s = (struct accel_scale *) arg;
            float sum = s->x_scale + s->y_scale + s->z_scale;

            if (sum > 2.0f && sum < 4.0f) {
                memcpy(&_accel_scale, s, sizeof(_accel_scale));
                return OK;

            } else {
                return -EINVAL;
            }
        }

    case ACCELIOCGSCALE:
        /* copy scale out */
        memcpy((struct accel_scale *) arg, &_accel_scale, sizeof(_accel_scale));
        return OK;

    case ACCELIOCSRANGE:
        return set_accel_range(arg);

    case ACCELIOCGRANGE:
        return (unsigned long)((_accel_range_m_s2) / LSM9DS1_ONE_G + 0.5f);

    case ACCELIOCSELFTEST:
        return accel_self_test();

#ifdef ACCELIOCSHWLOWPASS

    case ACCELIOCSHWLOWPASS:
        _set_dlpf_filter(arg);
        return OK;
#endif

#ifdef ACCELIOCGHWLOWPASS

    case ACCELIOCGHWLOWPASS:
        return _dlpf_freq;
#endif


    default:
        /* give it to the superclass */
        return SPI::ioctl(filp, cmd, arg);
    }
}

int
LSM9DS1::gyro_ioctl(struct file *filp, int cmd, unsigned long arg)
{
    switch (cmd) {

    /* these are shared with the accel side */
    case SENSORIOCSPOLLRATE:
    case SENSORIOCGPOLLRATE:
    case SENSORIOCRESET:
        return ioctl(filp, cmd, arg);

    case SENSORIOCSQUEUEDEPTH: {
            /* lower bound is mandatory, upper bound is a sanity check */
            if ((arg < 1) || (arg > 100)) {
                return -EINVAL;
            }

            irqstate_t flags = irqsave();

            if (!_gyro_reports->resize(arg)) {
                irqrestore(flags);
                return -ENOMEM;
            }

            irqrestore(flags);

            return OK;
        }

    case SENSORIOCGQUEUEDEPTH:
        return _gyro_reports->size();

    case GYROIOCGSAMPLERATE:
        return _gyro_sample_rate;

    case GYROIOCSSAMPLERATE:
        return gyro_set_sample_rate(arg);

    case GYROIOCGLOWPASS:
        return _gyro_filter_x.get_cutoff_freq();

    case GYROIOCSLOWPASS:
        // set software filtering
        _gyro_filter_x.set_cutoff_frequency(1.0e6f / _call_interval, arg);
        _gyro_filter_y.set_cutoff_frequency(1.0e6f / _call_interval, arg);
        _gyro_filter_z.set_cutoff_frequency(1.0e6f / _call_interval, arg);
        return OK;

    case GYROIOCSSCALE:
        /* copy scale in */
        memcpy(&_gyro_scale, (struct gyro_scale *) arg, sizeof(_gyro_scale));
        return OK;

    case GYROIOCGSCALE:
        /* copy scale out */
        memcpy((struct gyro_scale *) arg, &_gyro_scale, sizeof(_gyro_scale));
        return OK;

    case GYROIOCSRANGE:
        return set_gyro_range(arg);

    case GYROIOCGRANGE:
        return (unsigned long)(_gyro_range_rad_s * 180.0f / M_PI_F + 0.5f);

    case GYROIOCSELFTEST:
        return gyro_self_test();

#ifdef GYROIOCSHWLOWPASS

    case GYROIOCSHWLOWPASS:
        _set_dlpf_filter(arg);
        return OK;
#endif

#ifdef GYROIOCGHWLOWPASS

    case GYROIOCGHWLOWPASS:
        return _dlpf_freq;
#endif

    default:
        /* give it to the superclass */
        return SPI::ioctl(filp, cmd, arg);
    }
}

uint8_t
LSM9DS1::read_reg(unsigned reg, uint32_t speed)
{
    uint8_t cmd[2] = { (uint8_t)(reg | DIR_READ), 0};

    // general register transfer at low clock speed
    set_frequency(speed);

    transfer(cmd, cmd, sizeof(cmd));

    return cmd[1];
}

uint16_t
LSM9DS1::read_reg16(unsigned reg)
{
    uint8_t cmd[3] = { (uint8_t)(reg | DIR_READ), 0, 0 };

    // general register transfer at low clock speed
    set_frequency(LSM9DS1_LOW_BUS_SPEED);

    transfer(cmd, cmd, sizeof(cmd));

    return (uint16_t)(cmd[1] << 8) | cmd[2];
}

void
LSM9DS1::write_reg(unsigned reg, uint8_t value)
{
    uint8_t	cmd[2];

    cmd[0] = reg | DIR_WRITE;
    cmd[1] = value;

    // general register transfer at low clock speed
    set_frequency(LSM9DS1_LOW_BUS_SPEED);

    transfer(cmd, nullptr, sizeof(cmd));
}

void
LSM9DS1::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
    uint8_t	val;

    val = read_reg(reg,LSM9DS1_LOW_BUS_SPEED);
    val &= ~clearbits;
    val |= setbits;
    write_checked_reg(reg, val);
}

void
LSM9DS1::write_checked_reg(unsigned reg, uint8_t value)
{
    write_reg(reg, value);

    for (uint8_t i = 0; i < LSM9DS1_NUM_CHECKED_REGISTERS; i++) {
        if (reg == _checked_registers[i]) {
            _checked_values[i] = value;
            _checked_bad[i] = value;
        }
    }
}


int
LSM9DS1::set_accel_range(unsigned max_g)
{
    uint8_t setbits = 0;
    uint8_t clearbits = LSM_A_RANGE_8_g;
    float lsb_per_g;

    if (max_g == 0) {
        max_g = 16;
    }

    if (max_g <= 2) {
        _accel_range_m_s2 = 2.0f * LSM9DS1_ONE_G;
        setbits |= LSM_A_RANGE_2_g;
        lsb_per_g = 16384;

    } else if (max_g <= 4) {
        _accel_range_m_s2 = 4.0f * LSM9DS1_ONE_G;
        setbits |= LSM_A_RANGE_4_g;
        lsb_per_g = 8192;

    } else if (max_g <= 8) {
        _accel_range_m_s2 = 8.0f * LSM9DS1_ONE_G;
        setbits |= LSM_A_RANGE_8_g;
        lsb_per_g = 4096;

    } else if (max_g >8) {
        _accel_range_m_s2 = 16.0f * LSM9DS1_ONE_G;
        setbits |= LSM_A_RANGE_16_g;
        lsb_per_g = 2048;

    } else {
        return -EINVAL;
    }

    _accel_range_scale = LSM9DS1_ONE_G / lsb_per_g;

    modify_reg(CTRL_REG6_XL, clearbits, setbits);

    return OK;
}

int
LSM9DS1::set_gyro_range(unsigned max_dps)
{
    uint8_t setbits = 0;
    uint8_t clearbits = LSM_G_RANGE_2000_DPS;
    float lsb_per_dps = 0.0f;

    if (max_dps == 0) {
        max_dps = 2000;
    }

    if (max_dps <= 245) {
        _gyro_range_rad_s = (245.0f/ 180.0f) * M_PI_F;
        setbits |= LSM_G_RANGE_245_DPS;
        lsb_per_dps = 133.747;

    } else if (max_dps <= 500) {
        _gyro_range_rad_s = (500.0f / 180.0f) * M_PI_F;
        setbits |= LSM_G_RANGE_500_DPS;
        lsb_per_dps = 65.536;

    } else if (max_dps > 500) {
        _gyro_range_rad_s = (2000.0f / 180.0f) * M_PI_F;
        setbits |= LSM_G_RANGE_2000_DPS;
        lsb_per_dps = 16.384;

    } else {
        return -EINVAL;
    }

    _gyro_range_scale = (M_PI_F / (180.0f * lsb_per_dps));

    modify_reg(CTRL_REG1_G, clearbits, setbits);

    return OK;
}


void
LSM9DS1::start()
{
    /* make sure we are stopped first */
    stop();

    /* discard any stale data in the buffers */
    _accel_reports->flush();
    _gyro_reports->flush();

    /* start polling at the specified rate */
    hrt_call_every(&_call,
               1000,
               _call_interval - LSM9DS1_TIMER_REDUCTION,
               (hrt_callout)&LSM9DS1::measure_trampoline, this);
}

void
LSM9DS1::stop()
{
    hrt_cancel(&_call);
}

void
LSM9DS1::measure_trampoline(void *arg)
{
    LSM9DS1 *dev = reinterpret_cast<LSM9DS1 *>(arg);

    /* make another measurement */
    dev->measure();
}


void
LSM9DS1::check_registers(void)
{
    uint8_t v;

    if ((v = read_reg(_checked_registers[_checked_next], LSM9DS1_LOW_BUS_SPEED)) !=
        _checked_values[_checked_next]) {
        _checked_bad[_checked_next] = v;

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
          bus.
         */
        if (_register_wait == 0 || _checked_next == 0) {
            // if the product_id is wrong then reset the
            // sensor completely
            write_reg(CTRL_REG8, BOOT | SW_RESET);
            _reset_wait = hrt_absolute_time() + 10000;
            _checked_next = 0;

        } else {
            write_reg(_checked_registers[_checked_next], _checked_values[_checked_next]);
            // waiting 3ms between register writes seems
            // to raise the chance of the sensor
            // recovering considerably
            _reset_wait = hrt_absolute_time() + 3000;
        }

        _register_wait = 20;
    }

    _checked_next = (_checked_next + 1) % LSM9DS1_NUM_CHECKED_REGISTERS;
}



void
LSM9DS1::measure()
{
    if (hrt_absolute_time() < _reset_wait) {
        // we're waiting for a reset to complete
        return;
    }

    struct LSMReport lsm_report;

    struct Report {
        int16_t		accel_x;
        int16_t		accel_y;
        int16_t		accel_z;
        int16_t		temp;
        int16_t		gyro_x;
        int16_t		gyro_y;
        int16_t		gyro_z;
    } report;

    /* start measuring */
    perf_begin(_sample_perf);

    /*
     * Fetch the full set of measurements from the LSM9DS1 in one pass.
     */
    lsm_report.cmd = OUT_X_L_G | DIR_READ;

    set_frequency(LSM9DS1_LOW_BUS_SPEED);

    uint8_t		status = read_reg(STATUS_REG1, LSM9DS1_LOW_BUS_SPEED);

    if (OK != transfer((uint8_t *)&lsm_report, ((uint8_t *)&lsm_report), sizeof(lsm_report))) {
        return;
    }
    stm32_gpiowrite(GPIO_SYNC_ODROID, 1);

    stm32_gpiowrite(GPIO_SYNC_ODROID, 0);

    check_registers();

    if ((!(status && (0x01)))  && (!(status && (0x02)))) {
        perf_end(_sample_perf);
        perf_count(_duplicates);
        _got_duplicate = true;
        return;
    }

    _last_accel[0] = lsm_report.accel_x;
    _last_accel[1] = lsm_report.accel_y;
    _last_accel[2] = lsm_report.accel_z;
    _got_duplicate = false;

    uint8_t temp_l = read_reg(OUT_TEMP_L, LSM9DS1_LOW_BUS_SPEED);
    uint8_t temp_h = read_reg(OUT_TEMP_H, LSM9DS1_LOW_BUS_SPEED);
    temp_h=(temp_h && 0x0F);

    report.temp = ((temp_h<<8) + temp_l);

    report.accel_x = lsm_report.accel_x;
    report.accel_y = lsm_report.accel_y;
    report.accel_z = lsm_report.accel_z;

    report.gyro_x = lsm_report.gyro_x;
    report.gyro_y = lsm_report.gyro_y;
    report.gyro_z = lsm_report.gyro_z;

    if (report.accel_x == 0 &&
        report.accel_y == 0 &&
        report.accel_z == 0 &&
        report.temp == 0 &&
        report.gyro_x == 0 &&
        report.gyro_y == 0 &&
        report.gyro_z == 0) {
        // all zero data - probably a SPI bus error
        perf_count(_bad_transfers);
        perf_end(_sample_perf);
        // note that we don't call reset() here as a reset()
        // costs 20ms with interrupts disabled. That means if
        // the lsm9ds1 does go bad it would cause a FMU failure,
        // regardless of whether another sensor is available,
        return;
    }

    perf_count(_good_transfers);

    if (_register_wait != 0) {
        // we are waiting for some good transfers before using
        // the sensor again. We still increment
        // _good_transfers, but don't return any data yet
        _register_wait--;
        return;
    }


    /*
     * negate y
     */

    report.accel_y = ((report.accel_y == -32768) ? 32767 : -report.accel_y);
    report.gyro_y = ((report.gyro_y == -32768) ? 32767 : -report.gyro_y);

    /*
     * Report buffers.
     */
    accel_report		arb;
    gyro_report		grb;

    /*
     * Adjust and scale results to m/s^2.
     */
    grb.timestamp = arb.timestamp = hrt_absolute_time();

    // report the error count as the sum of the number of bad
    // transfers and bad register reads. This allows the higher
    // level code to decide if it should use this sensor based on
    // whether it has had failures
    grb.error_count = arb.error_count = perf_event_count(_bad_transfers) + perf_event_count(_bad_registers);

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


    /* NOTE: Axes have been swapped to match the board a few lines above. */

    arb.x_raw = report.accel_x;
    arb.y_raw = report.accel_y;
    arb.z_raw = report.accel_z;

    float xraw_f = report.accel_x;
    float yraw_f = report.accel_y;
    float zraw_f = report.accel_z;

    // apply user specified rotation
    rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

    float x_in_new = ((xraw_f * _accel_range_scale) - _accel_scale.x_offset) * _accel_scale.x_scale;
    float y_in_new = ((yraw_f * _accel_range_scale) - _accel_scale.y_offset) * _accel_scale.y_scale;
    float z_in_new = ((zraw_f * _accel_range_scale) - _accel_scale.z_offset) * _accel_scale.z_scale;

    arb.x = _accel_filter_x.apply(x_in_new);
    arb.y = _accel_filter_y.apply(y_in_new);
    arb.z = _accel_filter_z.apply(z_in_new);

    math::Vector<3> aval(x_in_new, y_in_new, z_in_new);
    math::Vector<3> aval_integrated;

    bool accel_notify = _accel_int.put(arb.timestamp, aval, aval_integrated, arb.integral_dt);
    arb.x_integral = aval_integrated(0);
    arb.y_integral = aval_integrated(1);
    arb.z_integral = aval_integrated(2);

    arb.scaling = _accel_range_scale;
    arb.range_m_s2 = _accel_range_m_s2;

    _last_temperature = 25 + report.temp * 0.0625f;

    arb.temperature_raw = report.temp;
    arb.temperature = _last_temperature;

    grb.x_raw = report.gyro_x;
    grb.y_raw = report.gyro_y;
    grb.z_raw = report.gyro_z;

    xraw_f = report.gyro_x;
    yraw_f = report.gyro_y;
    zraw_f = report.gyro_z;

    // apply user specified rotation
    rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

    float x_gyro_in_new = ((xraw_f * _gyro_range_scale) - _gyro_scale.x_offset) * _gyro_scale.x_scale;
    float y_gyro_in_new = ((yraw_f * _gyro_range_scale) - _gyro_scale.y_offset) * _gyro_scale.y_scale;
    float z_gyro_in_new = ((zraw_f * _gyro_range_scale) - _gyro_scale.z_offset) * _gyro_scale.z_scale;

    grb.x = _gyro_filter_x.apply(x_gyro_in_new);
    grb.y = _gyro_filter_y.apply(y_gyro_in_new);
    grb.z = _gyro_filter_z.apply(z_gyro_in_new);

    math::Vector<3> gval(x_gyro_in_new, y_gyro_in_new, z_gyro_in_new);
    math::Vector<3> gval_integrated;

    bool gyro_notify = _gyro_int.put(arb.timestamp, gval, gval_integrated, grb.integral_dt);
    grb.x_integral = gval_integrated(0);
    grb.y_integral = gval_integrated(1);
    grb.z_integral = gval_integrated(2);

    grb.scaling = _gyro_range_scale;
    grb.range_rad_s = _gyro_range_rad_s;

    grb.temperature_raw = report.temp;
    grb.temperature = _last_temperature;

    _accel_reports->force(&arb);
    _gyro_reports->force(&grb);

    /* notify anyone waiting for data */
    if (accel_notify) {
        poll_notify(POLLIN);
    }

    if (gyro_notify) {
        _gyro->parent_poll_notify();
    }

    if (accel_notify && !(_pub_blocked)) {
        /* log the time of this report */
        perf_begin(_controller_latency_perf);
        /* publish it */
        orb_publish(ORB_ID(sensor_accel), _accel_topic, &arb);
    }

    if (gyro_notify && !(_pub_blocked)) {
        /* publish it */
        orb_publish(ORB_ID(sensor_gyro), _gyro->_gyro_topic, &grb);
    }

    /* stop measuring */
    perf_end(_sample_perf);
}

void
LSM9DS1::print_info()
{
    perf_print_counter(_sample_perf);
    perf_print_counter(_accel_reads);
    perf_print_counter(_gyro_reads);
    perf_print_counter(_bad_transfers);
    perf_print_counter(_bad_registers);
    perf_print_counter(_good_transfers);
    perf_print_counter(_reset_retries);
    perf_print_counter(_duplicates);
    _accel_reports->print_info("accel queue");
    _gyro_reports->print_info("gyro queue");
    ::printf("checked_next: %u\n", _checked_next);

    for (uint8_t i = 0; i < LSM9DS1_NUM_CHECKED_REGISTERS; i++) {
        uint8_t v = read_reg(_checked_registers[i], LSM9DS1_LOW_BUS_SPEED);

        if (v != _checked_values[i]) {
            ::printf("reg %02x:%02x should be %02x\n",
                 (unsigned)_checked_registers[i],
                 (unsigned)v,
                 (unsigned)_checked_values[i]);
        }

        if (v != _checked_bad[i]) {
            ::printf("reg %02x:%02x was bad %02x\n",
                 (unsigned)_checked_registers[i],
                 (unsigned)v,
                 (unsigned)_checked_bad[i]);
        }
    }

    ::printf("temperature: %.1f\n", (double)_last_temperature);
}


void
LSM9DS1::print_registers()
{
    printf("LSM9DS1 registers\n");

    for (uint8_t reg = 0x0F; reg <= 0x24; reg++) {
        uint8_t v = read_reg(reg);
        printf("%02x:%02x ", (unsigned)reg, (unsigned)v);

        if (reg % 13 == 0) {
            printf("\n");
        }
    }

    printf("\n");
}


LSM9DS1_gyro::LSM9DS1_gyro(LSM9DS1 *parent, const char *path) :
    CDev("LSM9DS1_gyro", path),
    _parent(parent),
    _gyro_topic(nullptr),
    _gyro_orb_class_instance(-1),
    _gyro_class_instance(-1)
{
}

LSM9DS1_gyro::~LSM9DS1_gyro()
{
    if (_gyro_class_instance != -1) {
        unregister_class_devname(GYRO_BASE_DEVICE_PATH, _gyro_class_instance);
    }
}

int
LSM9DS1_gyro::init()
{
    int ret;

    // do base class init
    ret = CDev::init();

    /* if probe/setup failed, bail now */
    if (ret != OK) {
        DEVICE_DEBUG("gyro init failed");
        return ret;
    }

    _gyro_class_instance = register_class_devname(GYRO_BASE_DEVICE_PATH);

    return ret;
}

void
LSM9DS1_gyro::parent_poll_notify()
{
    poll_notify(POLLIN);
}

ssize_t
LSM9DS1_gyro::read(struct file *filp, char *buffer, size_t buflen)
{
    return _parent->gyro_read(filp, buffer, buflen);
}

int
LSM9DS1_gyro::ioctl(struct file *filp, int cmd, unsigned long arg)
{

    switch (cmd) {
    case DEVIOCGDEVICEID:
        return (int)CDev::ioctl(filp, cmd, arg);
        break;

    default:
        return _parent->gyro_ioctl(filp, cmd, arg);
    }
}

/**
 * Local functions in support of the shell command.
 */
namespace lsm9ds1
{

LSM9DS1	*g_dev_int; // on internal bus
LSM9DS1	*g_dev_ext; // on external bus

void	start(bool, enum Rotation);
void	stop(bool);
void	test(bool);
void	reset(bool);
void	info(bool);
void	regdump(bool);
void	testerror(bool);
void	usage();

/**
 * Start the driver.
 *
 * This function only returns if the driver is up and running
 * or failed to detect the sensor.
 */
void
start(bool external_bus, enum Rotation rotation)
{
    int fd;
    LSM9DS1 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;
    const char *path_accel = external_bus ? LSM9DS1_DEVICE_PATH_ACCEL_EXT : LSM9DS1_DEVICE_PATH_ACCEL;
    const char *path_gyro  = external_bus ? LSM9DS1_DEVICE_PATH_GYRO_EXT : LSM9DS1_DEVICE_PATH_GYRO;

    if (*g_dev_ptr != nullptr)
        /* if already started, the still command succeeded */
    {
        errx(0, "already started");
    }

    /* create the driver */
    if (external_bus) {
#ifdef PX4_SPI_BUS_EXT
        *g_dev_ptr = new LSM9DS1(PX4_SPI_BUS_EXT, path_accel, path_gyro, (spi_dev_e)PX4_SPIDEV_EXT_ACCEL_GYRO, rotation);
#else
        errx(0, "External SPI not available");
#endif

    } else {
        *g_dev_ptr = new LSM9DS1(PX4_SPI_BUS_SENSORS, path_accel, path_gyro, (spi_dev_e)PX4_SPIDEV_ACCEL_GYRO, rotation);
    }

    if (*g_dev_ptr == nullptr) {
        goto fail;
    }

    if (OK != (*g_dev_ptr)->init()) {
        goto fail;
    }

    /* set the poll rate to default, starts automatic data collection */
    fd = open(path_accel, O_RDONLY);

    if (fd < 0) {
        goto fail;
    }

    if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        goto fail;
    }

    close(fd);

    exit(0);
fail:

    if (*g_dev_ptr != nullptr) {
        delete(*g_dev_ptr);
        *g_dev_ptr = nullptr;
    }

    errx(1, "driver start failed");
}


void
stop(bool external_bus)
{
    LSM9DS1 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

    if (*g_dev_ptr != nullptr) {
        delete *g_dev_ptr;
        *g_dev_ptr = nullptr;

    } else {
        /* warn, but not an error */
        warnx("already stopped.");
    }

    exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test(bool external_bus)
{
    const char *path_accel = external_bus ? LSM9DS1_DEVICE_PATH_ACCEL_EXT : LSM9DS1_DEVICE_PATH_ACCEL;
    const char *path_gyro  = external_bus ? LSM9DS1_DEVICE_PATH_GYRO_EXT : LSM9DS1_DEVICE_PATH_GYRO;
    accel_report a_report;
    gyro_report g_report;
    ssize_t sz;

    /* get the driver */
    int fd = open(path_accel, O_RDONLY);

    if (fd < 0)
        err(1, "%s open failed (try 'lsm9ds1 start')",
            path_accel);

    /* get the driver */
    int fd_gyro = open(path_gyro, O_RDONLY);

    if (fd_gyro < 0) {
        err(1, "%s open failed", path_gyro);
    }

    /* reset to manual polling */
    if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MANUAL) < 0) {
        err(1, "reset to manual polling");
    }

    /* do a simple demand read */
    sz = read(fd, &a_report, sizeof(a_report));

    if (sz != sizeof(a_report)) {
        warnx("ret: %d, expected: %d", sz, sizeof(a_report));
        err(1, "immediate acc read failed");
    }

    warnx("single read");
    warnx("time:     %lld", a_report.timestamp);
    warnx("acc  x:  \t%8.4f\tm/s^2", (double)a_report.x);
    warnx("acc  y:  \t%8.4f\tm/s^2", (double)a_report.y);
    warnx("acc  z:  \t%8.4f\tm/s^2", (double)a_report.z);
    warnx("acc  x:  \t%d\traw 0x%0x", (short)a_report.x_raw, (unsigned short)a_report.x_raw);
    warnx("acc  y:  \t%d\traw 0x%0x", (short)a_report.y_raw, (unsigned short)a_report.y_raw);
    warnx("acc  z:  \t%d\traw 0x%0x", (short)a_report.z_raw, (unsigned short)a_report.z_raw);
    warnx("acc range: %8.4f m/s^2 (%8.4f g)", (double)a_report.range_m_s2,
          (double)(a_report.range_m_s2 / LSM9DS1_ONE_G));

    /* do a simple demand read */
    sz = read(fd_gyro, &g_report, sizeof(g_report));

    if (sz != sizeof(g_report)) {
        warnx("ret: %d, expected: %d", sz, sizeof(g_report));
        err(1, "immediate gyro read failed");
    }

    warnx("gyro x: \t% 9.5f\trad/s", (double)g_report.x);
    warnx("gyro y: \t% 9.5f\trad/s", (double)g_report.y);
    warnx("gyro z: \t% 9.5f\trad/s", (double)g_report.z);
    warnx("gyro x: \t%d\traw", (int)g_report.x_raw);
    warnx("gyro y: \t%d\traw", (int)g_report.y_raw);
    warnx("gyro z: \t%d\traw", (int)g_report.z_raw);
    warnx("gyro range: %8.4f rad/s (%d deg/s)", (double)g_report.range_rad_s,
          (int)((g_report.range_rad_s / M_PI_F) * 180.0f + 0.5f));

    warnx("temp:  \t%8.4f\tdeg celsius", (double)a_report.temperature);
    warnx("temp:  \t%d\traw 0x%0x", (short)a_report.temperature_raw, (unsigned short)a_report.temperature_raw);

    /* reset to default polling */
    if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        err(1, "reset to default polling");
    }

    close(fd);
    close(fd_gyro);

    /* XXX add poll-rate tests here too */

    reset(external_bus);
    errx(0, "PASS");
}


/**
 * Reset the driver.
 */
void
reset(bool external_bus)
{
    const char *path_accel = external_bus ? LSM9DS1_DEVICE_PATH_ACCEL_EXT : LSM9DS1_DEVICE_PATH_ACCEL;
    int fd = open(path_accel, O_RDONLY);

    if (fd < 0) {
        err(1, "failed ");
    }

    if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
        err(1, "driver reset failed");
    }

    if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        err(1, "driver poll restart failed");
    }

    close(fd);

    exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info(bool external_bus)
{
    LSM9DS1 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

    if (*g_dev_ptr == nullptr) {
        errx(1, "driver not running");
    }

    printf("state @ %p\n", *g_dev_ptr);
    (*g_dev_ptr)->print_info();

    exit(0);
}

/**
 * Dump the register information
 */
void
regdump(bool external_bus)
{
    LSM9DS1 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

    if (*g_dev_ptr == nullptr) {
        errx(1, "driver not running");
    }

    printf("regdump @ %p\n", *g_dev_ptr);
    (*g_dev_ptr)->print_registers();

    exit(0);
}

/**
 * deliberately produce an error to test recovery
 */
void
testerror(bool external_bus)
{
    LSM9DS1 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

    if (*g_dev_ptr == nullptr) {
        errx(1, "driver not running");
    }

    (*g_dev_ptr)->test_error();

    exit(0);
}

void
usage()
{
    warnx("missing command: try 'start', 'info', 'test', 'stop',\n'reset', 'regdump', 'testerror'");
    warnx("options:");
    warnx("    -X    (external bus)");
    warnx("    -R rotation");
}

} // namespace


int
lsm9ds1_main(int argc, char *argv[])
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
            lsm9ds1::usage();
            exit(0);
        }
    }

    const char *verb = argv[optind];

    /*
     * Start/load the driver.

     */
    if (!strcmp(verb, "start")) {
        lsm9ds1::start(external_bus, rotation);
    }

    if (!strcmp(verb, "stop")) {
        lsm9ds1::stop(external_bus);
    }

    /*
     * Test the driver/device.
     */
    if (!strcmp(verb, "test")) {
        lsm9ds1::test(external_bus);
    }

    /*
     * Reset the driver.
     */
    if (!strcmp(verb, "reset")) {
        lsm9ds1::reset(external_bus);
    }

    /*
     * Print driver information.
     */
    if (!strcmp(verb, "info")) {
        lsm9ds1::info(external_bus);
    }

    /*
     * Print register information.
     */
    if (!strcmp(verb, "regdump")) {
        lsm9ds1::regdump(external_bus);
    }

    if (!strcmp(verb, "testerror")) {
        lsm9ds1::testerror(external_bus);
    }

    lsm9ds1::usage();
    exit(1);
}
