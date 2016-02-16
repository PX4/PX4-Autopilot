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
 * @file bmi160.cpp
 *
 * Driver for bmi 160 connected via SPI.
 *
 * @author Fábio Azevedo
 *
 *
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

#define DIR_READ                0x80
#define DIR_WRITE               0x00

#define BMI160_DEVICE_PATH_ACCEL    "/dev/bmi160_accel"
#define BMI160_DEVICE_PATH_GYRO		"/dev/bmi160_gyro"
#define BMI160_DEVICE_PATH_MAG		"/dev/bmi160_mag"
#define BMI160_DEVICE_PATH_ACCEL_EXT    "/dev/bmi160_accel_ext"
#define BMI160_DEVICE_PATH_GYRO_EXT		"/dev/bmi160_gyro_ext"
#define BMI160_DEVICE_PATH_MAG_EXT	"/dev/bmi160_mag_ext"

// BMI 160 registers

#define BMIREG_CHIP_ID          0x00
#define BMIREG_ERR_REG          0x02
#define BMIREG_PMU_STATUS       0x03
#define BMIREG_DATA_0           0x04
#define BMIREG_DATA_1           0x05
#define BMIREG_DATA_2           0x06
#define BMIREG_DATA_3           0x07
#define BMIREG_DATA_4           0x08
#define BMIREG_DATA_5           0x09
#define BMIREG_DATA_6           0x0A
#define BMIREG_DATA_7           0x0B
#define BMIREG_GYR_X_L          0x0C
#define BMIREG_GYR_X_H          0x0D
#define BMIREG_GYR_Y_L          0x0E
#define BMIREG_GYR_Y_H          0x0F
#define BMIREG_GYR_Z_L          0x10
#define BMIREG_GYR_Z_H          0x11
#define BMIREG_ACC_X_L          0x12
#define BMIREG_ACC_X_H          0x13
#define BMIREG_ACC_Y_L          0x14
#define BMIREG_ACC_Y_H          0x15
#define BMIREG_ACC_Z_L          0x16
#define BMIREG_ACC_Z_H          0x17
#define BMIREG_SENSORTIME0      0x18
#define BMIREG_SENSORTIME1      0x19
#define BMIREG_SENSORTIME2      0x1A
#define BMIREG_STATUS           0x1B
#define BMIREG_INT_STATUS_0     0x1C
#define BMIREG_INT_STATUS_1     0x1D
#define BMIREG_INT_STATUS_2     0x1E
#define BMIREG_INT_STATUS_3     0x1F
#define BMIREG_TEMP_0           0x20
#define BMIREG_TEMP_1           0x21
#define BMIREG_FIFO_LEN_0       0x22
#define BMIREG_FIFO_LEN_1       0x23
#define BMIREG_FIFO_DATA        0x24
#define BMIREG_ACC_CONF         0x40
#define BMIREG_ACC_RANGE        0x41
#define BMIREG_GYR_CONF         0x42
#define BMIREG_GYR_RANGE        0x43
#define BMIREG_MAG_CONF         0x44
#define BMIREG_FIFO_DOWNS       0x45
#define BMIREG_FIFO_CONFIG_0    0x46
#define BMIREG_FIFO_CONFIG_1    0x47
#define BMIREG_MAG_IF_0         0x4B
#define BMIREG_MAG_IF_1         0x4C
#define BMIREG_MAG_IF_2         0x4D
#define BMIREG_MAG_IF_3         0x4E
#define BMIREG_MAG_IF_4         0x4F
#define BMIREG_INT_EN_0         0x50
#define BMIREG_INT_EN_1         0x51
#define BMIREG_INT_EN_2         0x52
#define BMIREG_INT_OUT_CTRL     0x53
#define BMIREG_INT_LANTCH       0x54
#define BMIREG_INT_MAP_0        0x55
#define BMIREG_INT_MAP_1        0x56
#define BMIREG_INT_MAP_2        0x57
#define BMIREG_INT_DATA_0       0x58
#define BMIREG_INT_DATA_1       0x59
#define BMIREG_INT_LH_0         0x5A
#define BMIREG_INT_LH_1         0x5B
#define BMIREG_INT_LH_2         0x5C
#define BMIREG_INT_LH_3         0x5D
#define BMIREG_INT_LH_4         0x5E
#define BMIREG_INT_MOT_0        0x5F
#define BMIREG_INT_MOT_1        0x60
#define BMIREG_INT_MOT_2        0x61
#define BMIREG_INT_MOT_3        0x62
#define BMIREG_INT_TAP_0        0x63
#define BMIREG_INT_TAP_1        0x64
#define BMIREG_INT_ORIE_0       0x65
#define BMIREG_INT_ORIE_1       0x66
#define BMIREG_INT_FLAT_0       0x67
#define BMIREG_INT_FLAT_1       0x68
#define BMIREG_FOC_CONF         0x69
#define BMIREG_CONF             0x6A
#define BMIREG_IF_CONF          0x6B
#define BMIREG_PMU_TRIGGER      0x6C
#define BMIREG_SELF_TEST        0x6D
#define BMIREG_NV_CONF          0x70
#define BMIREG_OFFSET_ACC_X     0x71
#define BMIREG_OFFSET_ACC_Y     0x72
#define BMIREG_OFFSET_ACC_Z     0x73
#define BMIREG_OFFSET_GYR_X     0x74
#define BMIREG_OFFSET_GYR_Y     0x75
#define BMIREG_OFFSET_GYR_Z     0x76
#define BMIREG_OFFSET_EN        0x77
#define BMIREG_STEP_CONT_0      0x78
#define BMIREG_STEP_CONT_1      0x79
#define BMIREG_STEP_CONF_0      0x7A
#define BMIREG_STEP_CONF_1      0x7B
#define BMIREG_CMD              0x7E


// Configuration bits BMI 160
#define BMI160_WHO_AM_I         0xD1

//BMIREG_STATUS           0x1B
#define BMI_DRDY_ACCEL          (1<<7)
#define BMI_DRDY_GYRO           (1<<6)
#define BMI_DRDY_MAG            (1<<5)
#define BMI_GYRO_SELF_TEST_OK   (1<<1)

//BMIREG_INT_STATUS_1     0x1D
#define BMI_DRDY_INT            (1<<4)

//BMIREG_ACC_CONF         0x40
#define BMI_ACCEL_RATE_25_32    (0<<3) | (0<<2) | (0<<1) | (1<<0)
#define BMI_ACCEL_RATE_25_16    (0<<3) | (0<<2) | (1<<1) | (0<<0)
#define BMI_ACCEL_RATE_25_8     (0<<3) | (0<<2) | (1<<1) | (1<<0)
#define BMI_ACCEL_RATE_25_4     (0<<3) | (1<<2) | (0<<1) | (0<<0)
#define BMI_ACCEL_RATE_25_2     (0<<3) | (1<<2) | (0<<1) | (1<<0)
#define BMI_ACCEL_RATE_25       (0<<3) | (1<<2) | (1<<1) | (0<<0)
#define BMI_ACCEL_RATE_50       (0<<3) | (1<<2) | (1<<1) | (1<<0)
#define BMI_ACCEL_RATE_100      (1<<3) | (0<<2) | (0<<1) | (0<<0)
#define BMI_ACCEL_RATE_200      (1<<3) | (0<<2) | (0<<1) | (1<<0)
#define BMI_ACCEL_RATE_400      (1<<3) | (0<<2) | (1<<1) | (0<<0)
#define BMI_ACCEL_RATE_800      (1<<3) | (0<<2) | (1<<1) | (1<<0)
#define BMI_ACCEL_RATE_1600     (1<<3) | (1<<2) | (0<<1) | (0<<0)
#define BMI_ACCEL_US            (0<<7)
#define BMI_ACCEL_BWP_NORMAL    (0<<6) | (1<<5) | (0<<4)
#define BMI_ACCEL_BWP_OSR2      (0<<6) | (0<<5) | (1<<4)
#define BMI_ACCEL_BWP_OSR4      (0<<6) | (0<<5) | (0<<4)

//BMIREG_ACC_RANGE        0x41
#define BMI_ACCEL_RANGE_2_G     (0<<3) | (0<<2) | (1<<1) | (1<<0)
#define BMI_ACCEL_RANGE_4_G     (0<<3) | (1<<2) | (0<<1) | (1<<0)
#define BMI_ACCEL_RANGE_8_G     (1<<3) | (0<<2) | (0<<1) | (0<<0)
#define BMI_ACCEL_RANGE_16_G    (1<<3) | (1<<2) | (0<<1) | (0<<0)

//BMIREG_GYR_CONF         0x42
#define BMI_GYRO_RATE_25        (0<<3) | (1<<2) | (1<<1) | (0<<0)
#define BMI_GYRO_RATE_50        (0<<3) | (1<<2) | (1<<1) | (1<<0)
#define BMI_GYRO_RATE_100       (1<<3) | (0<<2) | (0<<1) | (0<<0)
#define BMI_GYRO_RATE_200       (1<<3) | (0<<2) | (0<<1) | (1<<0)
#define BMI_GYRO_RATE_400       (1<<3) | (0<<2) | (1<<1) | (0<<0)
#define BMI_GYRO_RATE_800       (1<<3) | (0<<2) | (1<<1) | (1<<0)
#define BMI_GYRO_RATE_1600      (1<<3) | (1<<2) | (0<<1) | (0<<0)
#define BMI_GYRO_RATE_3200      (1<<3) | (1<<2) | (0<<1) | (1<<0)
#define BMI_GYRO_BWP_NORMAL     (1<<5) | (0<<4)
#define BMI_GYRO_BWP_OSR2       (0<<5) | (1<<4)
#define BMI_GYRO_BWP_OSR4       (0<<5) | (0<<4)

//BMIREG_GYR_RANGE        0x43
#define BMI_GYRO_RANGE_2000_DPS (0<<2) | (0<<1) | (0<<0)
#define BMI_GYRO_RANGE_1000_DPS (0<<2) | (0<<1) | (1<<0)
#define BMI_GYRO_RANGE_500_DPS  (0<<2) | (1<<1) | (0<<0)
#define BMI_GYRO_RANGE_250_DPS  (0<<2) | (1<<1) | (1<<0)
#define BMI_GYRO_RANGE_125_DPS  (1<<2) | (0<<1) | (0<<0)

//BMIREG_INT_EN_1         0x51
#define BMI_DRDY_INT_EN         (1<<4)

//BMIREG_INT_OUT_CTRL     0x53
#define BMI_INT1_EN             (1<<3) | (0<<2) | (1<<1)    //Data Ready on INT1 High

//BMIREG_INT_MAP_1        0x56
#define BMI_DRDY_INT1           (1<<7)

//BMIREG_IF_CONF          0x6B
#define BMI_SPI_3_WIRE          (1<<0)
#define BMI_SPI_4_WIRE          (0<<0)
#define BMI_AUTO_DIS_SEC        (0<<5) | (0<<4)
#define BMI_I2C_OIS_SEC         (0<<5) | (1<<4)
#define BMI_AUTO_MAG_SEC        (1<<5) | (0<<4)

//BMIREG_NV_CONF          0x70
#define BMI_SPI                 (1<<0)

//BMIREG_CMD              0x7E
#define BMI_ACCEL_NORMAL_MODE   0x11 //Wait at least 3.8 ms before another CMD
#define BMI_GYRO_NORMAL_MODE    0x15 //Wait at least 80 ms before another CMD
#define BMI160_SOFT_RESET       0xB6

#define BMI160_ACCEL_DEFAULT_RANGE_G		4
#define BMI160_GYRO_DEFAULT_RANGE_DPS		2000
#define BMI160_ACCEL_DEFAULT_RATE           800
#define BMI160_ACCEL_MAX_RATE               1600
#define BMI160_GYRO_DEFAULT_RATE            800
#define BMI160_GYRO_MAX_RATE                3200

#define BMI160_ACCEL_DEFAULT_ONCHIP_FILTER_FREQ	324
#define BMI160_ACCEL_DEFAULT_DRIVER_FILTER_FREQ	50

#define BMI160_GYRO_DEFAULT_ONCHIP_FILTER_FREQ	254.6f
#define BMI160_GYRO_DEFAULT_DRIVER_FILTER_FREQ	50

#define BMI160_ONE_G                       9.80665f

#define BMI160_LOW_BUS_SPEED				1000*1000
#define BMI160_HIGH_BUS_SPEED				11*1000*1000

#define BMI160_TIMER_REDUCTION				200

#ifdef PX4_SPI_BUS_EXT
#define EXTERNAL_BUS PX4_SPI_BUS_EXT
#else
#define EXTERNAL_BUS 0
#endif

class BMI160_gyro;

class BMI160 : public device::SPI
{
public:
    BMI160(int bus, const char *path_accel, const char *path_gyro, spi_dev_e device, enum Rotation rotation);
    virtual ~BMI160();

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

    friend class BMI160_gyro;

    virtual ssize_t		gyro_read(struct file *filp, char *buffer, size_t buflen);
    virtual int		gyro_ioctl(struct file *filp, int cmd, unsigned long arg);

private:
    BMI160_gyro		*_gyro;
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
#define BMI160_NUM_CHECKED_REGISTERS 10
    static const uint8_t	_checked_registers[BMI160_NUM_CHECKED_REGISTERS];
    uint8_t			_checked_values[BMI160_NUM_CHECKED_REGISTERS];
    uint8_t			_checked_bad[BMI160_NUM_CHECKED_REGISTERS];
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
     * Read a register from the BMI160
     *
     * @param		The register to read.
     * @return		The value that was read.
     */
    uint8_t			read_reg(unsigned reg, uint32_t speed = BMI160_LOW_BUS_SPEED);
    uint16_t		read_reg16(unsigned reg);

    /**
     * Write a register in the BMI160
     *
     * @param reg		The register to write.
     * @param value		The new value to write.
     */
    void			write_reg(unsigned reg, uint8_t value);

    /**
     * Modify a register in the BMI160
     *
     * Bits are cleared before bits are set.
     *
     * @param reg		The register to modify.
     * @param clearbits	Bits in the register to clear.
     * @param setbits	Bits in the register to set.
     */
    void			modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

    /**
     * Write a register in the BMI160, updating _checked_values
     *
     * @param reg		The register to write.
     * @param value		The new value to write.
     */
    void			write_checked_reg(unsigned reg, uint8_t value);

    /**
     * Set the BMI160 measurement range.
     *
     * @param max_g		The maximum G value the range must support.
     * @param max_dps	The maximum DPS value the range must support.
     * @return		OK if the value can be supported, -ERANGE otherwise.
     */
    int			set_accel_range(unsigned max_g);
    int			set_gyro_range(unsigned max_dps);

    /**
     * Swap a 16-bit value read from the BMI160 to native byte order.
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
    BMI160(const BMI160 &);
    BMI160 operator=(const BMI160 &);

#pragma pack(push, 1)
    /**
     * Report conversation within the BMI160, including command byte and
     * interrupt status.
     */
    struct BMIReport {
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
  that ADDR_WHO_AM_I must be first in the list.
 */
const uint8_t BMI160::_checked_registers[BMI160_NUM_CHECKED_REGISTERS] = {    BMIREG_CHIP_ID,
                                                                              BMIREG_ACC_CONF,
                                                                              BMIREG_ACC_RANGE,
                                                                              BMIREG_GYR_CONF,
                                                                              BMIREG_GYR_RANGE,
                                                                              BMIREG_INT_EN_1,
                                                                              BMIREG_INT_OUT_CTRL,
                                                                              BMIREG_INT_MAP_1,
                                                                              BMIREG_IF_CONF,
                                                                              BMIREG_NV_CONF
                                                                          };

/**
 * Helper class implementing the gyro driver node.
 */
class BMI160_gyro : public device::CDev
{
public:
    BMI160_gyro(BMI160 *parent, const char *path);
    ~BMI160_gyro();

    virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
    virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

    virtual int		init();

protected:
    friend class BMI160;

    void			parent_poll_notify();

private:
    BMI160			*_parent;
    orb_advert_t		_gyro_topic;
    int			_gyro_orb_class_instance;
    int			_gyro_class_instance;

    /* do not allow to copy this class due to pointer data members */
    BMI160_gyro(const BMI160_gyro &);
    BMI160_gyro operator=(const BMI160_gyro &);
};


/** driver 'main' command */
extern "C" { __EXPORT int bmi160_main(int argc, char *argv[]); }

BMI160::BMI160(int bus, const char *path_accel, const char *path_gyro, spi_dev_e device, enum Rotation rotation) :
    SPI("BMI160", path_accel, bus, device, SPIDEV_MODE3, BMI160_LOW_BUS_SPEED),
    _gyro(new BMI160_gyro(this, path_gyro)),
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
    _accel_sample_rate(BMI160_ACCEL_DEFAULT_RATE),
    _gyro_sample_rate(BMI160_GYRO_DEFAULT_RATE),
    _accel_reads(perf_alloc(PC_COUNT, "bmi160_accel_read")),
    _gyro_reads(perf_alloc(PC_COUNT, "bmi160_gyro_read")),
    _sample_perf(perf_alloc(PC_ELAPSED, "bmi160_read")),
    _bad_transfers(perf_alloc(PC_COUNT, "bmi160_bad_transfers")),
    _bad_registers(perf_alloc(PC_COUNT, "bmi160_bad_registers")),
    _good_transfers(perf_alloc(PC_COUNT, "bmi160_good_transfers")),
    _reset_retries(perf_alloc(PC_COUNT, "bmi160_reset_retries")),
    _duplicates(perf_alloc(PC_COUNT, "bmi160_duplicates")),
    _controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency")),
    _register_wait(0),
    _reset_wait(0),
    _accel_filter_x(BMI160_ACCEL_DEFAULT_RATE, BMI160_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
    _accel_filter_y(BMI160_ACCEL_DEFAULT_RATE, BMI160_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
    _accel_filter_z(BMI160_ACCEL_DEFAULT_RATE, BMI160_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
    _gyro_filter_x(BMI160_GYRO_DEFAULT_RATE, BMI160_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
    _gyro_filter_y(BMI160_GYRO_DEFAULT_RATE, BMI160_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
    _gyro_filter_z(BMI160_GYRO_DEFAULT_RATE, BMI160_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
    _accel_int(1000000 / BMI160_ACCEL_MAX_RATE),
    _gyro_int(1000000 / BMI160_GYRO_MAX_RATE, true),
    _rotation(rotation),
    _checked_next(0),
    _last_temperature(0),
    _last_accel{},
    _got_duplicate(false)
{
    // disable debug() calls
    _debug_enabled = false;

    _device_id.devid_s.devtype = DRV_ACC_DEVTYPE_BMI160;

    /* Prime _gyro with parents devid. */
    _gyro->_device_id.devid = _device_id.devid;
    _gyro->_device_id.devid_s.devtype = DRV_GYR_DEVTYPE_BMI160;

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


BMI160::~BMI160()
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
BMI160::init()
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


int BMI160::reset()
{
    write_reg(BMIREG_CONF,(1<<1));  //Enable NVM programming

    write_checked_reg(BMIREG_ACC_CONF,      BMI_ACCEL_US | BMI_ACCEL_BWP_NORMAL); //Normal operation, no decimation
    write_checked_reg(BMIREG_ACC_RANGE,     0);
    write_checked_reg(BMIREG_GYR_CONF,      BMI_GYRO_BWP_NORMAL);   //Normal operation, no decimation
    write_checked_reg(BMIREG_GYR_RANGE,     0);
    write_checked_reg(BMIREG_INT_EN_1,      BMI_DRDY_INT_EN); //Enable DRDY interrupt
    write_checked_reg(BMIREG_INT_OUT_CTRL,  BMI_INT1_EN);   //Enable interrupts on pin INT1
    write_checked_reg(BMIREG_INT_MAP_1,     BMI_DRDY_INT1); //DRDY interrupt on pin INT1
    write_checked_reg(BMIREG_IF_CONF,       BMI_SPI_4_WIRE | BMI_AUTO_DIS_SEC); //Disable secondary interface; Work in SPI 4-wire mode
    write_checked_reg(BMIREG_NV_CONF,       BMI_SPI); //Disable I2C interface

    set_accel_range(BMI160_ACCEL_DEFAULT_RANGE_G);
    accel_set_sample_rate(BMI160_ACCEL_DEFAULT_RATE);

    set_gyro_range(BMI160_GYRO_DEFAULT_RANGE_DPS);
    gyro_set_sample_rate(BMI160_GYRO_DEFAULT_RATE);


    //_set_dlpf_filter(BMI160_ACCEL_DEFAULT_ONCHIP_FILTER_FREQ); //NOT CONSIDERING FILTERING YET

    //Enable Accelerometer in normal mode
    write_reg(BMIREG_CMD,BMI_ACCEL_NORMAL_MODE);
    up_udelay(4100);
    //usleep(4100);

    //Enable Gyroscope in normal mode
    write_reg(BMIREG_CMD,BMI_GYRO_NORMAL_MODE);
    up_udelay(80300);
    //usleep(80300);

    uint8_t retries = 10;

    while (retries--) {
        bool all_ok = true;

        for (uint8_t i = 0; i < BMI160_NUM_CHECKED_REGISTERS; i++) {
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
BMI160::probe()
{
    /* look for device ID */
    _whoami = read_reg(BMIREG_CHIP_ID);

    // verify product revision
    switch (_whoami) {
    case BMI160_WHO_AM_I:
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
BMI160::accel_set_sample_rate(float frequency)
{
    uint8_t setbits = 0;
    uint8_t clearbits = (BMI_ACCEL_RATE_25_8 | BMI_ACCEL_RATE_1600);

    if ((int)frequency == 0) {
        frequency = 1600;
    }

    if (frequency <= 25/32) {
        setbits |= BMI_ACCEL_RATE_25_32;
        _accel_sample_rate = 25/32;

    } else if (frequency <= 25/16) {
        setbits |= BMI_ACCEL_RATE_25_16;
        _accel_sample_rate = 25/16;

    } else if (frequency <= 25/16) {
        setbits |= BMI_ACCEL_RATE_25_16;
        _accel_sample_rate = 25/16;

    } else if (frequency <= 25/8) {
        setbits |= BMI_ACCEL_RATE_25_8;
        _accel_sample_rate = 25/8;

    } else if (frequency <= 25/4) {
        setbits |= BMI_ACCEL_RATE_25_4;
        _accel_sample_rate = 25/4;

    } else if (frequency <= 25/2) {
        setbits |= BMI_ACCEL_RATE_25_2;
        _accel_sample_rate = 25/2;

    } else if (frequency <= 25) {
        setbits |= BMI_ACCEL_RATE_25;
        _accel_sample_rate = 25;

    } else if (frequency <= 50) {
        setbits |= BMI_ACCEL_RATE_50;
        _accel_sample_rate = 50;

    } else if (frequency <= 100) {
        setbits |= BMI_ACCEL_RATE_100;
        _accel_sample_rate = 100;

    } else if (frequency <= 200) {
        setbits |= BMI_ACCEL_RATE_200;
        _accel_sample_rate = 200;

    } else if (frequency <= 400) {
        setbits |= BMI_ACCEL_RATE_400;
        _accel_sample_rate = 400;

    } else if (frequency <= 800) {
        setbits |= BMI_ACCEL_RATE_800;
        _accel_sample_rate = 800;

    } else if (frequency > 800) {
        setbits |= BMI_ACCEL_RATE_1600;
        _accel_sample_rate = 1600;

    } else {
        return -EINVAL;
    }

    modify_reg(BMIREG_ACC_CONF, clearbits, setbits);

    return OK;
}

int
BMI160::gyro_set_sample_rate(float frequency)
{
    uint8_t setbits = 0;
    uint8_t clearbits = (BMI_GYRO_RATE_200 | BMI_GYRO_RATE_25);

    if ((int)frequency == 0) {
        frequency = 3200;
    }

    if (frequency <= 25) {
        setbits |= BMI_GYRO_RATE_25;
        _gyro_sample_rate = 25;

    } else if (frequency <= 50) {
        setbits |= BMI_GYRO_RATE_50;
        _gyro_sample_rate = 50;

    } else if (frequency <= 100) {
        setbits |= BMI_GYRO_RATE_100;
        _gyro_sample_rate = 100;

    } else if (frequency <= 200) {
        setbits |= BMI_GYRO_RATE_200;
        _gyro_sample_rate = 200;

    } else if (frequency <= 400) {
        setbits |= BMI_GYRO_RATE_400;
        _gyro_sample_rate = 400;

    } else if (frequency <= 800) {
        setbits |= BMI_GYRO_RATE_800;
        _gyro_sample_rate = 800;

    } else if (frequency <= 1600) {
        setbits |= BMI_GYRO_RATE_1600;
        _gyro_sample_rate = 1600;

    } else if (frequency > 1600) {
        setbits |= BMI_GYRO_RATE_3200;
        _gyro_sample_rate = 3200;

    } else {
        return -EINVAL;
    }

    modify_reg(BMIREG_GYR_CONF, clearbits, setbits);

    return OK;
}

void
BMI160::_set_dlpf_filter(uint16_t bandwidth)
{
    _dlpf_freq = 0;
    bandwidth = bandwidth;   //TO BE IMPLEMENTED
    /*uint8_t setbits = BW_SCAL_ODR_BW_XL;
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
    modify_reg(CTRL_REG6_XL, clearbits, setbits);*/
}

ssize_t
BMI160::read(struct file *filp, char *buffer, size_t buflen)
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
BMI160::self_test()
{
    if (perf_event_count(_sample_perf) == 0) {
        measure();
    }

    /* return 0 on success, 1 else */
    return (perf_event_count(_sample_perf) > 0) ? 0 : 1;
}

int
BMI160::accel_self_test()
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
BMI160::gyro_self_test()
{
    if (self_test()) {
        return 1;
    }

    /*
     * Maximum deviation of 10 degrees
     */
    const float max_offset = (float)(10 * M_PI_F / 180.0f);
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
BMI160::test_error()
{
    write_reg(BMIREG_CMD, BMI160_SOFT_RESET);
    ::printf("error triggered\n");
    print_registers();
}

ssize_t
BMI160::gyro_read(struct file *filp, char *buffer, size_t buflen)
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
BMI160::ioctl(struct file *filp, int cmd, unsigned long arg)
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
                return ioctl(filp, SENSORIOCSPOLLRATE, BMI160_GYRO_MAX_RATE);

            case SENSOR_POLLRATE_DEFAULT:
                if(BMI160_GYRO_DEFAULT_RATE > BMI160_ACCEL_DEFAULT_RATE)
                {
                    return ioctl(filp, SENSORIOCSPOLLRATE, BMI160_GYRO_DEFAULT_RATE);
                    warnx("GYROOOOOOOOO");
                }
                else
                {
                    return ioctl(filp, SENSORIOCSPOLLRATE, BMI160_ACCEL_DEFAULT_RATE); //Polling at the highest frequency. We may get duplicate values on the sensors
                    warnx("ACCELLLLLLLLLLLL");
                }

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
                      stm32 clock and the bmi160 clock
                     */
                    _call.period = _call_interval - BMI160_TIMER_REDUCTION;

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
        return (unsigned long)((_accel_range_m_s2) / BMI160_ONE_G + 0.5f);

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
BMI160::gyro_ioctl(struct file *filp, int cmd, unsigned long arg)
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
BMI160::read_reg(unsigned reg, uint32_t speed)
{
    uint8_t cmd[2] = { (uint8_t)(reg | DIR_READ), 0};

    // general register transfer at low clock speed
    set_frequency(speed);

    transfer(cmd, cmd, sizeof(cmd));

    return cmd[1];
}

uint16_t
BMI160::read_reg16(unsigned reg)
{
    uint8_t cmd[3] = { (uint8_t)(reg | DIR_READ), 0, 0 };

    // general register transfer at low clock speed
    set_frequency(BMI160_LOW_BUS_SPEED);

    transfer(cmd, cmd, sizeof(cmd));

    return (uint16_t)(cmd[1] << 8) | cmd[2];
}

void
BMI160::write_reg(unsigned reg, uint8_t value)
{
    uint8_t	cmd[2];

    cmd[0] = reg | DIR_WRITE;
    cmd[1] = value;

    // general register transfer at low clock speed
    set_frequency(BMI160_LOW_BUS_SPEED);

    transfer(cmd, nullptr, sizeof(cmd));
}

void
BMI160::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
    uint8_t	val;

    val = read_reg(reg, BMI160_LOW_BUS_SPEED);
    val &= ~clearbits;
    val |= setbits;
    write_checked_reg(reg, val);
}

void
BMI160::write_checked_reg(unsigned reg, uint8_t value)
{
    write_reg(reg, value);

    for (uint8_t i = 0; i < BMI160_NUM_CHECKED_REGISTERS; i++) {
        if (reg == _checked_registers[i]) {
            _checked_values[i] = value;
            _checked_bad[i] = value;
        }
    }
}

int
BMI160::set_accel_range(unsigned max_g)
{
    uint8_t setbits = 0;
    uint8_t clearbits = BMI_ACCEL_RANGE_2_G | BMI_ACCEL_RANGE_16_G;
    float lsb_per_g;
    float max_accel_g;

    if (max_g == 0) {
        max_g = 16;
    }

    if (max_g <= 2) {
        max_accel_g = 2;
        setbits |= BMI_ACCEL_RANGE_2_G;
        lsb_per_g = 16384;

    } else if (max_g <= 4) {
        max_accel_g = 4;
        setbits |= BMI_ACCEL_RANGE_4_G;
        lsb_per_g = 8192;

    } else if (max_g <= 8) {
        max_accel_g = 8;
        setbits |= BMI_ACCEL_RANGE_8_G;
        lsb_per_g = 4096;

    } else if (max_g <= 16) {
        max_accel_g = 16;
        setbits |= BMI_ACCEL_RANGE_16_G;
        lsb_per_g = 2048;

    } else {
        return -EINVAL;
    }

    _accel_range_scale = (BMI160_ONE_G / lsb_per_g);
    _accel_range_m_s2 = max_accel_g * BMI160_ONE_G;

    modify_reg(BMIREG_ACC_RANGE, clearbits, setbits);

    return OK;
}

int
BMI160::set_gyro_range(unsigned max_dps)
{
    uint8_t setbits = 0;
    uint8_t clearbits = BMI_GYRO_RANGE_125_DPS | BMI_GYRO_RANGE_250_DPS;
    float lsb_per_dps;
    float max_gyro_dps;

    if (max_dps == 0) {
        max_dps = 2000;
    }

    if (max_dps <= 125) {
        max_gyro_dps = 125;
        lsb_per_dps = 262.4;
        setbits |= BMI_GYRO_RANGE_125_DPS;

    } else if (max_dps <= 250) {
        max_gyro_dps = 250;
        lsb_per_dps = 131.2;
        setbits |= BMI_GYRO_RANGE_250_DPS;

    } else if (max_dps <= 500) {
        max_gyro_dps = 500;
        lsb_per_dps = 65.6;
        setbits |= BMI_GYRO_RANGE_500_DPS;

    } else if (max_dps <= 1000) {
        max_gyro_dps = 1000;
        lsb_per_dps = 32.8;
        setbits |= BMI_GYRO_RANGE_1000_DPS;

    } else if (max_dps <= 2000) {
        max_gyro_dps = 2000;
        lsb_per_dps = 16.4;
        setbits |= BMI_GYRO_RANGE_1000_DPS;

    } else {
        return -EINVAL;
    }

    _gyro_range_rad_s = (max_gyro_dps / 180.0f * M_PI_F);
    _gyro_range_scale = (M_PI_F / ( 180.0f * lsb_per_dps));

    modify_reg(BMIREG_GYR_RANGE, clearbits, setbits);

    return OK;
}

void
BMI160::start()
{
    /* make sure we are stopped first */
    stop();

    /* discard any stale data in the buffers */
    _accel_reports->flush();
    _gyro_reports->flush();

    /* start polling at the specified rate */
    hrt_call_every(&_call,
               1000,
               _call_interval - BMI160_TIMER_REDUCTION,
               (hrt_callout)&BMI160::measure_trampoline, this);
    reset();
}

void
BMI160::stop()
{
    hrt_cancel(&_call);
}

void
BMI160::measure_trampoline(void *arg)
{
    BMI160 *dev = reinterpret_cast<BMI160 *>(arg);

    /* make another measurement */
    dev->measure();
}

void
BMI160::check_registers(void)
{
    uint8_t v;

    if ((v = read_reg(_checked_registers[_checked_next], BMI160_LOW_BUS_SPEED)) !=
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
            write_reg(BMIREG_CMD, BMI160_SOFT_RESET);
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

    _checked_next = (_checked_next + 1) % BMI160_NUM_CHECKED_REGISTERS;
}

void
BMI160::measure()
{
    if (hrt_absolute_time() < _reset_wait) {
        // we're waiting for a reset to complete
        return;
    }

    struct BMIReport bmi_report;

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
     * Fetch the full set of measurements from the BMI160 in one pass.
     */
    bmi_report.cmd = BMIREG_GYR_X_L | DIR_READ;

    set_frequency(BMI160_LOW_BUS_SPEED);

    uint8_t		status = read_reg(BMIREG_STATUS, BMI160_LOW_BUS_SPEED);

    if (OK != transfer((uint8_t *)&bmi_report, ((uint8_t *)&bmi_report), sizeof(bmi_report))) {
        return;
    }
    stm32_gpiowrite(GPIO_SYNC_ODROID, 1);

    stm32_gpiowrite(GPIO_SYNC_ODROID, 0);

    check_registers();

    if ((!(status && (0x80)))  && (!(status && (0x04)))) {
        perf_end(_sample_perf);
        perf_count(_duplicates);
        _got_duplicate = true;
        return;
    }

    _last_accel[0] = bmi_report.accel_x;
    _last_accel[1] = bmi_report.accel_y;
    _last_accel[2] = bmi_report.accel_z;
    _got_duplicate = false;

    uint8_t temp_l = read_reg(BMIREG_TEMP_0, BMI160_LOW_BUS_SPEED);
    uint8_t temp_h = read_reg(BMIREG_TEMP_1, BMI160_LOW_BUS_SPEED);

    report.temp = ((temp_h<<8) + temp_l);

    report.accel_x = bmi_report.accel_x;
    report.accel_y = bmi_report.accel_y;
    report.accel_z = bmi_report.accel_z;

    report.gyro_x = bmi_report.gyro_x;
    report.gyro_y = bmi_report.gyro_y;
    report.gyro_z = bmi_report.gyro_z;

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
        // the bmi160 does go bad it would cause a FMU failure,
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

    _last_temperature = 23 + report.temp * 1.0f/512.0f;

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
BMI160::print_info()
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

    for (uint8_t i = 0; i < BMI160_NUM_CHECKED_REGISTERS; i++) {
        uint8_t v = read_reg(_checked_registers[i], BMI160_LOW_BUS_SPEED);

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
BMI160::print_registers()
{
    printf("BMI160 registers\n");

    for (uint8_t reg = 0x40; reg <= 0x47; reg++) {
        uint8_t v = read_reg(reg);
        printf("%02x:%02x ", (unsigned)reg, (unsigned)v);

        if (reg % 13 == 0) {
            printf("\n");
        }
    }

    printf("\n");
}


BMI160_gyro::BMI160_gyro(BMI160 *parent, const char *path) :
    CDev("BMI160_gyro", path),
    _parent(parent),
    _gyro_topic(nullptr),
    _gyro_orb_class_instance(-1),
    _gyro_class_instance(-1)
{
}

BMI160_gyro::~BMI160_gyro()
{
    if (_gyro_class_instance != -1) {
        unregister_class_devname(GYRO_BASE_DEVICE_PATH, _gyro_class_instance);
    }
}

int
BMI160_gyro::init()
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
BMI160_gyro::parent_poll_notify()
{
    poll_notify(POLLIN);
}

ssize_t
BMI160_gyro::read(struct file *filp, char *buffer, size_t buflen)
{
    return _parent->gyro_read(filp, buffer, buflen);
}

int
BMI160_gyro::ioctl(struct file *filp, int cmd, unsigned long arg)
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
namespace bmi160
{

BMI160	*g_dev_int; // on internal bus
BMI160	*g_dev_ext; // on external bus

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
    BMI160 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;
    const char *path_accel = external_bus ? BMI160_DEVICE_PATH_ACCEL_EXT : BMI160_DEVICE_PATH_ACCEL;
    const char *path_gyro  = external_bus ? BMI160_DEVICE_PATH_GYRO_EXT : BMI160_DEVICE_PATH_GYRO;

    if (*g_dev_ptr != nullptr)
        /* if already started, the still command succeeded */
    {
        errx(0, "already started");
    }

    /* create the driver */
    if (external_bus) {
#ifdef PX4_SPI_BUS_EXT
        *g_dev_ptr = new BMI160(PX4_SPI_BUS_EXT, path_accel, path_gyro, (spi_dev_e)PX4_SPIDEV_EXT_BMI, rotation);
#else
        errx(0, "External SPI not available");
#endif

    } else {
        *g_dev_ptr = new BMI160(PX4_SPI_BUS_SENSORS, path_accel, path_gyro, (spi_dev_e)PX4_SPIDEV_BMI, rotation);
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
    BMI160 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

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
    const char *path_accel = external_bus ? BMI160_DEVICE_PATH_ACCEL_EXT : BMI160_DEVICE_PATH_ACCEL;
    const char *path_gyro  = external_bus ? BMI160_DEVICE_PATH_GYRO_EXT : BMI160_DEVICE_PATH_GYRO;
    accel_report a_report;
    gyro_report g_report;
    ssize_t sz;

    /* get the driver */
    int fd = open(path_accel, O_RDONLY);

    if (fd < 0)
        err(1, "%s open failed (try 'bmi160 start')",
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
          (double)(a_report.range_m_s2 / BMI160_ONE_G));

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
    const char *path_accel = external_bus ? BMI160_DEVICE_PATH_ACCEL_EXT : BMI160_DEVICE_PATH_ACCEL;
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
    BMI160 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

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
    BMI160 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

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
    BMI160 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

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
bmi160_main(int argc, char *argv[])
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
            bmi160::usage();
            exit(0);
        }
    }

    const char *verb = argv[optind];

    /*
     * Start/load the driver.

     */
    if (!strcmp(verb, "start")) {
        bmi160::start(external_bus, rotation);
    }

    if (!strcmp(verb, "stop")) {
        bmi160::stop(external_bus);
    }

    /*
     * Test the driver/device.
     */
    if (!strcmp(verb, "test")) {
        bmi160::test(external_bus);
    }

    /*
     * Reset the driver.
     */
    if (!strcmp(verb, "reset")) {
        bmi160::reset(external_bus);
    }

    /*
     * Print driver information.
     */
    if (!strcmp(verb, "info")) {
        bmi160::info(external_bus);
    }

    /*
     * Print register information.
     */
    if (!strcmp(verb, "regdump")) {
        bmi160::regdump(external_bus);
    }

    if (!strcmp(verb, "testerror")) {
        bmi160::testerror(external_bus);
    }

    bmi160::usage();
    exit(1);
}

