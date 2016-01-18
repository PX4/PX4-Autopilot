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
 * Driver for the STM LSM9DS1 connected via SPI.
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
#include <drivers/drv_mag.h>
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
#define LSM9DS1_DEVICE_PATH_MAG         "/dev/lsm9ds1_mag"


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

//Index XL is equivalent to A --> accelerometer

// Configuration bits LSM9DS1
#define LSM_WHO_AM_I_A_G		0x68
#define LSM_WHO_AM_I_M			0x3D

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

//Magnetometer ODR CTRL_REG1_M
#define LSM_M_ODR_0_625_HZ      (0<<4) | (0<<3) | (0<<2)
#define LSM_M_ODR_1_25_HZ       (0<<4) | (0<<3) | (1<<2)
#define LSM_M_ODR_2_5_HZ        (0<<4) | (1<<3) | (0<<2)
#define LSM_M_ODR_5_HZ          (0<<4) | (1<<3) | (1<<2)
#define LSM_M_ODR_10_HZ         (1<<4) | (0<<3) | (0<<2)
#define LSM_M_ODR_20_HZ         (1<<4) | (0<<3) | (1<<2)
#define LSM_M_ODR_40_HZ         (1<<4) | (1<<3) | (0<<2)
#define LSM_M_ODR_80_HZ         (1<<4) | (1<<3) | (1<<2)


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

//CTRL_REG1_M - self-test mag ODR
#define TEMP_COMP               (1<<7)
#define LOW_PWR_M               (0<<6) | (0<<5)
#define MED_PER_M               (0<<6) | (1<<5)
#define HIGH_PER_M              (1<<6) | (0<<5)
#define UHIGH_PER_M             (1<<6) | (1<<5)

#define LOW_PWR_M_Z             (0<<3) | (0<<2)
#define MED_PER_M_Z             (0<<3) | (1<<2)
#define HIGH_PER_M_Z            (1<<3) | (0<<2)
#define UHIGH_PER_M_Z           (1<<3) | (1<<2)


#define LSM_WHOAMI_AG                       0x68
#define LSM_WHOAMI_M                        0x3D

#define ADDR_INCR               (1<<6)

/* default values for this device */
#define LSM9DS1_ACCEL_DEFAULT_RANGE_G		16
#define LSM9DS1_GYRO_DEFAULT_RANGE_DS		245
#define LSM9DS1_AG_DEFAULT_RATE             952
#define LSM9DS1_AG_MAX_RATE                 952

//VER FILTRO!!!!!!!!!!!!!!!!!!

#define LSM9DS1_AG_DEFAULT_ONCHIP_FILTER_FREQ	50
#define LSM9DS1_AG_DEFAULT_DRIVER_FILTER_FREQ	50

/*#define LSM9DS1_ACCEL_MAX_OUTPUT_RATE			280*/

/*------------------------------------------------------------------*/

#define LSM9DS1_MAG_DEFAULT_RANGE_GA		4
#define LSM9DS1_MAG_DEFAULT_RATE			80
#define LSM9DS1_M_DEFAULT_FILTER_FREQ       30

#define LSM9DS1_ONE_G                       9.80665f

#ifdef PX4_SPI_BUS_EXT
#define EXTERNAL_BUS PX4_SPI_BUS_EXT
#else
#define EXTERNAL_BUS 0
#endif

#ifndef SENSOR_BOARD_ROTATION_DEFAULT
#define SENSOR_BOARD_ROTATION_DEFAULT		SENSOR_BOARD_ROTATION_270_DEG
#endif


#define LSM9DS1_LOW_BUS_SPEED				1000*1000
#define LSM9DS1_HIGH_BUS_SPEED				11*1000*1000

#define LSM9DS1_MAX_OFFSET			0.45f /**< max offset: 25 degrees/s */

/*
  we set the timer interrupt to run a bit faster than the desired
  sample rate and then throw away duplicates using the data ready bit.
  This time reduction is enough to cope with worst case timing jitter
  due to other timers
 */
#define LSM9DS1_TIMER_REDUCTION				200

extern "C" { __EXPORT int lsm9ds1_main(int argc, char *argv[]); }

class LSM9DS1_gyro;

class LSM9DS1 : public device::SPI
{
    
public:
	LSM9DS1(int bus, const char *path, spi_dev_e device, enum Rotation rotation);
	virtual ~LSM9DS1();

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


    friend class 		LSM9DS1_gyro;

    virtual ssize_t		gyro_read(struct file *filp, char *buffer, size_t buflen);
    virtual int		gyro_ioctl(struct file *filp, int cmd, unsigned long arg);

private:

    LSM9DS1_gyro		*_gyro;

	struct hrt_call		_accel_call;
    struct hrt_call		_gyro_call;

	unsigned		_call_accel_interval;
    unsigned		_call_gyro_interval;

	ringbuffer::RingBuffer	*_accel_reports;
    ringbuffer::RingBuffer		*_gyro_reports;

	struct accel_scale	_accel_scale;
	unsigned		_accel_range_m_s2;
	float			_accel_range_scale;
	unsigned		_accel_samplerate;
	unsigned		_accel_onchip_filter_bandwith;

    struct gyro_scale	_gyro_scale;
    unsigned		_gyro_range_rad_s;
    float			_gyro_range_scale;
    unsigned		_gyro_samplerate;
    unsigned		_gyro_onchip_filter_bandwith;

	orb_advert_t		_accel_topic;
	int			_accel_orb_class_instance;
	int			_accel_class_instance;

    /*orb_advert_t		_gyro_topic;
    int			_gyro_orb_class_instance;
    int			_gyro_class_instance;*/

	unsigned		_accel_read;
    unsigned		_gyro_read;

	perf_counter_t		_accel_sample_perf;
    perf_counter_t		_gyro_sample_perf;
	perf_counter_t		_bad_registers;
	perf_counter_t		_bad_values;
	perf_counter_t		_accel_duplicates;
    perf_counter_t      _gyro_duplicates;

	uint8_t			_register_wait;

	math::LowPassFilter2p	_accel_filter_x;
	math::LowPassFilter2p	_accel_filter_y;
	math::LowPassFilter2p	_accel_filter_z;

    math::LowPassFilter2p	_gyro_filter_x;
    math::LowPassFilter2p	_gyro_filter_y;
    math::LowPassFilter2p	_gyro_filter_z;

    Integrator		_accel_int;
    Integrator		_gyro_int;


	enum Rotation		_rotation;

	// values used to
	float			_last_accel[3];
	uint8_t			_constant_accel_count;

	// last temperature value
	float			_last_temperature;

	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset
#define LSM9DS1_NUM_CHECKED_REGISTERS 13
	static const uint8_t	_checked_registers[LSM9DS1_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_values[LSM9DS1_NUM_CHECKED_REGISTERS];
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
	 * Get the internal / external state
	 *
	 * @return true if the sensor is not on the main MCU board
	 */
    bool			is_external() { return (_bus == EXTERNAL_BUS); }
    //bool			is_external() { return false; }

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
	 * Static trampoline for the mag because it runs at a lower rate
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
    static void		gyro_measure_trampoline(void *arg);

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
    void			gyro_measure();

	/**
	 * Accel self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int			accel_self_test();

	/**
	 * Mag self test
	 *
	 * @return 0 on success, 1 on failure
	 */
    int			gyro_self_test();

	/**
	 * Read a register from the LSM9DS1
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t			read_reg(unsigned reg);

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
	 * Set the LSM9DS1 accel measurement range.
	 *
	 * @param max_g	The measurement range of the accel is in g (9.81m/s^2)
	 *			Zero selects the maximum supported range.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
     */
	int			accel_set_range(unsigned max_g);

	/**
	 * Set the LSM9DS1 mag measurement range.
	 *
	 * @param max_ga	The measurement range of the mag is in Ga
	 *			Zero selects the maximum supported range.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */

    int			gyro_set_range(unsigned max_g);

	/**
	 * Set the LSM9DS1 on-chip anti-alias filter bandwith.
	 *
	 * @param bandwidth The anti-alias filter bandwidth in Hz
	 * 			Zero selects the highest bandwidth
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			accel_set_onchip_lowpass_filter_bandwidth(unsigned bandwidth);

    /**
    * Set the LSM9DS1 on-chip anti-alias filter bandwith.
    *
    * @param bandwidth The anti-alias filter bandwidth in Hz
    * 			Zero selects the highest bandwidth
    * @return		OK if the value can be supported, -ERANGE otherwise.
    */
   int			gyro_set_onchip_lowpass_filter_bandwidth(unsigned bandwidth);

	/**
	 * Set the driver lowpass filter bandwidth.
	 *
	 * @param bandwidth The anti-alias filter bandwidth in Hz
	 * 			Zero selects the highest bandwidth
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			accel_set_driver_lowpass_filter(float samplerate, float bandwidth);

    /**
     * Set the driver lowpass filter bandwidth.
     *
     * @param bandwidth The anti-alias filter bandwidth in Hz
     * 			Zero selects the highest bandwidth
     * @return		OK if the value can be supported, -ERANGE otherwise.
     */
    int			gyro_set_driver_lowpass_filter(float samplerate, float bandwidth);

	/**
     * Set the LSM9DS1 internal accel sampling frequency.
	 *
	 * @param frequency	The internal accel sampling frequency is set to not less than
	 *			this value.
	 *			Zero selects the maximum rate supported.
	 * @return		OK if the value can be supported.
	 */
	int			accel_set_samplerate(unsigned frequency);

	/**
	 * Set the LSM9DS1 internal mag sampling frequency.
	 *
	 * @param frequency	The internal mag sampling frequency is set to not less than
	 *			this value.
	 *			Zero selects the maximum rate supported.
	 * @return		OK if the value can be supported.
	 */
    int			gyro_set_samplerate(unsigned frequency);

	/* this class cannot be copied */
	LSM9DS1(const LSM9DS1 &);
	LSM9DS1 operator=(const LSM9DS1 &);
};

/*
  list of registers that will be checked in check_registers(). Note
  that ADDR_WHO_AM_I must be first in the list.
 */
const uint8_t LSM9DS1::_checked_registers[LSM9DS1_NUM_CHECKED_REGISTERS] = {    WHO_AM_I,
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
    LSM9DS1_gyro(LSM9DS1 *parent);
    ~LSM9DS1_gyro();

	virtual ssize_t			read(struct file *filp, char *buffer, size_t buflen);
	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);

	virtual int		init();

protected:
	friend class LSM9DS1;

	void				parent_poll_notify();
private:
	LSM9DS1				*_parent;

    orb_advert_t			_gyro_topic;
    int				_gyro_orb_class_instance;
    int				_gyro_class_instance;

	void				measure();

	void				measure_trampoline(void *arg);

	/* this class does not allow copying due to ptr data members */
    LSM9DS1_gyro(const LSM9DS1_gyro &);
    LSM9DS1_gyro operator=(const LSM9DS1_gyro &);
};

LSM9DS1::LSM9DS1(int bus, const char *path, spi_dev_e device, enum Rotation rotation) :
	SPI("LSM9DS1", path, bus, device, SPIDEV_MODE3,
        LSM9DS1_LOW_BUS_SPEED  /* will be rounded to 10.4 MHz, within safety margins for LSM9DS1 */),
    _gyro(new LSM9DS1_gyro(this)),
	_accel_call{},
    _gyro_call{},
	_call_accel_interval(0),
    _call_gyro_interval(0),
	_accel_reports(nullptr),
    _gyro_reports(nullptr),
	_accel_scale{},
	_accel_range_m_s2(0.0f),
	_accel_range_scale(0.0f),
	_accel_samplerate(0),
	_accel_onchip_filter_bandwith(0),
    _gyro_scale{},
    _gyro_range_rad_s(0.0f),
    _gyro_range_scale(0.0f),
    _gyro_samplerate(0),
    _gyro_onchip_filter_bandwith(0),
    _accel_topic(nullptr),
	_accel_orb_class_instance(-1),
	_accel_class_instance(-1),
    /*_gyro_topic(nullptr),
    _gyro_orb_class_instance(-1),
    _gyro_class_instance(-1),*/
	_accel_read(0),
    _gyro_read(0),
	_accel_sample_perf(perf_alloc(PC_ELAPSED, "lsm9ds1_accel_read")),
    _gyro_sample_perf(perf_alloc(PC_ELAPSED, "lsm9ds1_gyro_read")),
	_bad_registers(perf_alloc(PC_COUNT, "lsm9ds1_bad_registers")),
	_bad_values(perf_alloc(PC_COUNT, "lsm9ds1_bad_values")),
	_accel_duplicates(perf_alloc(PC_COUNT, "lsm9ds1_accel_duplicates")),
    _gyro_duplicates(perf_alloc(PC_COUNT, "lsm9ds1_gyro_duplicates")),
	_register_wait(0),
	_accel_filter_x(LSM9DS1_AG_DEFAULT_RATE, LSM9DS1_AG_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_y(LSM9DS1_AG_DEFAULT_RATE, LSM9DS1_AG_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_z(LSM9DS1_AG_DEFAULT_RATE, LSM9DS1_AG_DEFAULT_DRIVER_FILTER_FREQ),
    _gyro_filter_x(LSM9DS1_AG_DEFAULT_RATE, LSM9DS1_AG_DEFAULT_DRIVER_FILTER_FREQ),
    _gyro_filter_y(LSM9DS1_AG_DEFAULT_RATE, LSM9DS1_AG_DEFAULT_DRIVER_FILTER_FREQ),
    _gyro_filter_z(LSM9DS1_AG_DEFAULT_RATE, LSM9DS1_AG_DEFAULT_DRIVER_FILTER_FREQ),
    _accel_int(1000000 / LSM9DS1_AG_MAX_RATE, true),
    _gyro_int(1000000 / LSM9DS1_AG_MAX_RATE, true),
	_rotation(rotation),
	_constant_accel_count(0),
	_last_temperature(0),
	_checked_next(0)
{


	// enable debug() calls
	_debug_enabled = true;

    //_device_id.devid = PX4_SPIDEV_BARO;
	_device_id.devid_s.devtype = DRV_ACC_DEVTYPE_LSM9DS1;

	/* Prime _mag with parents devid. */
    _gyro->_device_id.devid = _device_id.devid;
    //_gyro->_device_id.devid = PX4_SPIDEV_ACCEL_MAG;
    //_mag->_device_id.devid = PX4_SPIDEV_MAG;
    _gyro->_device_id.devid_s.devtype = DRV_GYR_DEVTYPE_LSM9DS1;


	// default scale factors
	_accel_scale.x_offset = 0.0f;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0.0f;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0.0f;
	_accel_scale.z_scale  = 1.0f;

    _gyro_scale.x_offset = 0.0f;
    _gyro_scale.x_scale = 1.0f;
    _gyro_scale.y_offset = 0.0f;
    _gyro_scale.y_scale = 1.0f;
    _gyro_scale.z_offset = 0.0f;
    _gyro_scale.z_scale = 1.0f;
}

LSM9DS1::~LSM9DS1()
{
	/* make sure we are truly inactive */
	stop();

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

    delete _gyro;

	/* delete the perf counter */
	perf_free(_accel_sample_perf);
    perf_free(_gyro_sample_perf);
	perf_free(_bad_registers);
	perf_free(_bad_values);
	perf_free(_accel_duplicates);
    perf_free(_gyro_duplicates);
}

int
LSM9DS1::init()
{
	int ret = ERROR;

	/* do SPI init (and probe) first */
	if (SPI::init() != OK) {
		warnx("SPI init failed");
		goto out;
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

    reset();

	/* do CDev init for the mag device node */
    ret = _gyro->init();

	if (ret != OK) {
        warnx("GYRO init failed");
		goto out;
	}

	/* fill report structures */
	measure();
    //_gyro->measure();

	/* advertise sensor topic, measure manually to initialize valid report */
    struct gyro_report grp;
    _gyro_reports->get(&grp);

	/* measurement will have generated a report, publish */
    _gyro->_gyro_topic = orb_advertise_multi(ORB_ID(sensor_gyro), &grp,
                           &_gyro->_gyro_orb_class_instance, ORB_PRIO_DEFAULT);

    if (_gyro->_gyro_topic == nullptr) {
		warnx("ADVERT ERR");
	}


	_accel_class_instance = register_class_devname(ACCEL_BASE_DEVICE_PATH);

	/* advertise sensor topic, measure manually to initialize valid report */
	struct accel_report arp;
	_accel_reports->get(&arp);

	/* measurement will have generated a report, publish */
	_accel_topic = orb_advertise_multi(ORB_ID(sensor_accel), &arp,
                       &_accel_orb_class_instance, (is_external()) ? ORB_PRIO_VERY_HIGH-1 : ORB_PRIO_DEFAULT);

	if (_accel_topic == nullptr) {
		warnx("ADVERT ERR");
	}

out:
	return ret;
}

void
LSM9DS1::disable_i2c(void)
{
	write_checked_reg(CTRL_REG9, I2C_DISABLE);
}


void
LSM9DS1::reset()
{
	// ensure the chip doesn't interpret any other bus traffic as I2C
	disable_i2c();
    //disable_i2c_mag();
    
	/* enable accel*/
    //write_reg(CTRL_REG8,0x80);
    write_checked_reg(CTRL_REG5_XL, ALL_EN_XL);
    write_checked_reg(CTRL_REG6_XL, (LSM_A_ODR_952_HZ | LSM_A_RANGE_16_g | BW_SCAL_ODR_BW_XL | BW_XL_50_HZ));
    //write_checked_reg(CTRL_REG1_G, LSM_AG_ODR_952_HZ | LSM_G_RANGE_2000_DPS | 0x02);
    write_checked_reg(CTRL_REG7_XL, HR);
    write_checked_reg(CTRL_REG8, (BDU | IF_ADD_INC));
    write_checked_reg(CTRL_REG9, (DRDY_MASK_BIT | I2C_DISABLE));
    //write_checked_reg(CTRL_REG9, (I2C_DISABLE));
    write_checked_reg(CTRL_REG10, 0);

    accel_set_range(LSM9DS1_ACCEL_DEFAULT_RANGE_G);
	accel_set_samplerate(LSM9DS1_AG_DEFAULT_RATE);
	accel_set_driver_lowpass_filter((float)LSM9DS1_AG_DEFAULT_RATE, (float)LSM9DS1_AG_DEFAULT_DRIVER_FILTER_FREQ);
    accel_set_onchip_lowpass_filter_bandwidth(LSM9DS1_AG_DEFAULT_ONCHIP_FILTER_FREQ);

    write_checked_reg(INT1_CTRL, 0xC3);
    //write_checked_reg(INT1_CTRL, 0x81);
    write_checked_reg(INT2_CTRL, 0);
    write_checked_reg(CTRL_REG1_G, LSM_AG_ODR_952_HZ | LSM_G_RANGE_2000_DPS | 0x02);
    write_checked_reg(CTRL_REG2_G, 0);
    write_checked_reg(CTRL_REG3_G, 0);
    write_checked_reg(CTRL_REG4, ALL_EN_G);
    gyro_set_range(LSM9DS1_GYRO_DEFAULT_RANGE_DS);
    gyro_set_samplerate(LSM9DS1_AG_DEFAULT_RATE);
    gyro_set_driver_lowpass_filter((float)LSM9DS1_AG_DEFAULT_RATE, (float)LSM9DS1_AG_DEFAULT_DRIVER_FILTER_FREQ);

    // we setup the anti-alias on-chip filter as 50Hz. We believe
	// this operates in the analog domain, and is critical for
	// anti-aliasing. The 2 pole software filter is designed to
	// operate in conjunction with this on-chip filter

    //gyro_set_onchip_lowpass_filter_bandwidth(LSM9DS1_AG_DEFAULT_ONCHIP_FILTER_FREQ);

	_accel_read = 0;
    _gyro_read = 0;
}

int
LSM9DS1::probe()
{
	/* read dummy value to void to clear SPI statemachine on sensor */
	(void)read_reg(WHO_AM_I);

	/* verify that the device is attached and functioning */
	bool success = (read_reg(WHO_AM_I) == LSM_WHOAMI_AG);

	if (success) {
		_checked_values[0] = LSM_WHOAMI_AG;
        printf("WHOAMI:          %u\n", _checked_values[0]);
		return OK;
	}

	return -EIO;
}

ssize_t
LSM9DS1::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct accel_report);
	accel_report *arb = reinterpret_cast<accel_report *>(buffer);
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


ssize_t
LSM9DS1::gyro_read(struct file *filp, char *buffer, size_t buflen)
{
    unsigned count = buflen / sizeof(struct gyro_report);
    gyro_report *grb = reinterpret_cast<gyro_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
    if (_call_gyro_interval > 0) {

        /*reset()
		 * While there is space in the caller's buffer, and reports, copy them.
		 */
		while (count--) {
            if (_gyro_reports->get(grb)) {
                ret += sizeof(*grb);
                grb++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement */
    _gyro_reports->flush();
    _gyro->measure();

	/* measurement will have generated a report, copy it out */
    if (_gyro_reports->get(grb)) {
        ret = sizeof(*grb);
	}

	return ret;
}


int
LSM9DS1::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_call_accel_interval = 0;
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
					bool want_start = (_call_accel_interval == 0);

					/* convert hz to hrt interval via microseconds */
					unsigned ticks = 1000000 / arg;

					/* check against maximum sane rate */
					if (ticks < 500) {
						return -EINVAL;
					}

					/* adjust filters */
					accel_set_driver_lowpass_filter((float)arg, _accel_filter_x.get_cutoff_freq());

					/* update interval for next measurement */
					/* XXX this is a bit shady, but no other way to adjust... */
					_call_accel_interval = ticks;

					_accel_call.period = _call_accel_interval - LSM9DS1_TIMER_REDUCTION;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_call_accel_interval == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return 1000000 / _call_accel_interval;

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

	case SENSORIOCRESET:
		reset();
		return OK;

	case ACCELIOCSSAMPLERATE:
		return accel_set_samplerate(arg);

	case ACCELIOCGSAMPLERATE:
		return _accel_samplerate;

	case ACCELIOCSLOWPASS: {
			return accel_set_driver_lowpass_filter((float)_accel_samplerate, (float)arg);
		}

	case ACCELIOCGLOWPASS:
		return static_cast<int>(_accel_filter_x.get_cutoff_freq());

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

	case ACCELIOCSRANGE:
		/* arg needs to be in G */
		return accel_set_range(arg);

	case ACCELIOCGRANGE:
		/* convert to m/s^2 and return rounded in G */
		return (unsigned long)((_accel_range_m_s2) / LSM9DS1_ONE_G + 0.5f);

	case ACCELIOCGSCALE:
		/* copy scale out */
		memcpy((struct accel_scale *) arg, &_accel_scale, sizeof(_accel_scale));
		return OK;

	case ACCELIOCSELFTEST:
		return accel_self_test();

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}


int
LSM9DS1::gyro_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
                _call_gyro_interval = 0;
				return OK;

			/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT:
				/* 80 Hz is max for mag */
                return gyro_ioctl(filp, SENSORIOCSPOLLRATE, LSM9DS1_AG_DEFAULT_RATE);

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
                    bool want_start = (_call_gyro_interval == 0);

					/* convert hz to hrt interval via microseconds */
					unsigned ticks = 1000000 / arg;

					/* check against maximum sane rate */
					if (ticks < 1000) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					/* XXX this is a bit shady, but no other way to adjust... */
                    _gyro_call.period = _call_gyro_interval = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
        if (_call_gyro_interval == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

        return 1000000 / _call_gyro_interval;

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

	case SENSORIOCRESET:
		reset();
		return OK;

    case GYROIOCSSAMPLERATE:
        return gyro_set_samplerate(arg);

    case GYROIOCGSAMPLERATE:
        return _gyro_samplerate;

    case GYROIOCSLOWPASS:
    case GYROIOCGLOWPASS:
		/* not supported, no internal filtering */
		return -EINVAL;

    case GYROIOCSSCALE:
		/* copy scale in */
        memcpy(&_gyro_scale, (struct gyro_scale *) arg, sizeof(_gyro_scale));
		return OK;

    case GYROIOCGSCALE:
		/* copy scale out */
        memcpy((struct gyro_scale *) arg, &_gyro_scale, sizeof(_gyro_scale));
		return OK;

    case GYROIOCSRANGE:
        return gyro_set_range(arg);

    case GYROIOCGRANGE:
        return _gyro_range_rad_s;

    case GYROIOCSELFTEST:
        return gyro_self_test();

    //case GYROIOCGEXTERNAL:
		/* Even if this sensor is on the "external" SPI bus
		 * it is still fixed to the autopilot assembly,
		 * so always return 0.
		 */
        //return 0;

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

int
LSM9DS1::accel_self_test()
{
	if (_accel_read == 0) {
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
    /* evaluate gyro offsets, complain if offset -> zero or larger than 25 dps */
    if (fabsf(_gyro_scale.x_offset) > LSM9DS1_MAX_OFFSET || fabsf(_gyro_scale.x_offset) < 0.000001f) {
        return 1;
    }

    if (fabsf(_gyro_scale.x_scale - 1.0f) > 0.3f) {
        return 1;
    }

    if (fabsf(_gyro_scale.y_offset) > LSM9DS1_MAX_OFFSET || fabsf(_gyro_scale.y_offset) < 0.000001f) {
        return 1;
    }

    if (fabsf(_gyro_scale.y_scale - 1.0f) > 0.3f) {
        return 1;
    }

    if (fabsf(_gyro_scale.z_offset) > LSM9DS1_MAX_OFFSET || fabsf(_gyro_scale.z_offset) < 0.000001f) {
        return 1;
    }

    if (fabsf(_gyro_scale.z_scale - 1.0f) > 0.3f) {
        return 1;
    }

    return 0;
}

uint8_t
LSM9DS1::read_reg(unsigned reg)
{
	uint8_t cmd[2];

	cmd[0] = reg | DIR_READ;
	cmd[1] = 0;

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

void
LSM9DS1::write_reg(unsigned reg, uint8_t value)
{
	uint8_t	cmd[2];

	cmd[0] = reg | DIR_WRITE;
	cmd[1] = value;

	transfer(cmd, nullptr, sizeof(cmd));
}

void
LSM9DS1::write_checked_reg(unsigned reg, uint8_t value)
{
	write_reg(reg, value);

	for (uint8_t i = 0; i < LSM9DS1_NUM_CHECKED_REGISTERS; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
		}
	}
}

void
LSM9DS1::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_checked_reg(reg, val);
}

int
LSM9DS1::accel_set_range(unsigned max_g)
{
	uint8_t setbits = 0;
	uint8_t clearbits = LSM_A_RANGE_8_g;
	float new_scale_g_digit = 0.0f;

	if (max_g == 0) {
		max_g = 16;
	}

	if (max_g <= 2) {
		_accel_range_m_s2 = 2.0f * LSM9DS1_ONE_G;
		setbits |= LSM_A_RANGE_2_g;
		new_scale_g_digit = 0.061e-3f;

	} else if (max_g <= 4) {
		_accel_range_m_s2 = 4.0f * LSM9DS1_ONE_G;
		setbits |= LSM_A_RANGE_4_g;
		new_scale_g_digit = 0.122e-3f;

	} else if (max_g <= 8) {
		_accel_range_m_s2 = 8.0f * LSM9DS1_ONE_G;
		setbits |= LSM_A_RANGE_8_g;
		new_scale_g_digit = 0.244e-3f;

	} else if (max_g <= 16) {
		_accel_range_m_s2 = 16.0f * LSM9DS1_ONE_G;
		setbits |= LSM_A_RANGE_16_g;
        new_scale_g_digit = 0.488e-3f;

	} else {
		return -EINVAL;
	}

	_accel_range_scale = new_scale_g_digit * LSM9DS1_ONE_G;


	modify_reg(CTRL_REG6_XL, clearbits, setbits);

	return OK;
}

int
LSM9DS1::gyro_set_range(unsigned max_dps)
{
	uint8_t setbits = 0;
    uint8_t clearbits = LSM_G_RANGE_2000_DPS;
    float new_scale_dps_digit = 0.0f;

    if (max_dps == 0) {
        max_dps = 2000;
	}

    if (max_dps <= 245) {
        _gyro_range_rad_s = 245.0f/ 180.0f * M_PI_F;
        setbits |= LSM_G_RANGE_245_DPS;
        new_scale_dps_digit = 7.47681e-3f;

    } else if (max_dps <= 500) {
        _gyro_range_rad_s = 500.0f/ (180.0f * M_PI_F);
        setbits |= LSM_G_RANGE_500_DPS;
        new_scale_dps_digit = 15.25879e-3f;

    } else if (max_dps <= 2000) {
        _gyro_range_rad_s = (2000.0f / 180.0f * M_PI_F);
        setbits |= LSM_G_RANGE_2000_DPS;
        new_scale_dps_digit = 61.03516e-3f;

    } else {
		return -EINVAL;
	}

    _gyro_range_scale = (new_scale_dps_digit/ 180.0f * M_PI_F);

    modify_reg(CTRL_REG1_G, clearbits, setbits);

	return OK;
}

int
LSM9DS1::accel_set_onchip_lowpass_filter_bandwidth(unsigned bandwidth)
{
	uint8_t setbits = 0;
	uint8_t clearbits = BW_XL_50_HZ;

	if (bandwidth == 0) {
		bandwidth = 773;
	}

	if (bandwidth <= 50) {
		setbits |= BW_XL_50_HZ;
		_accel_onchip_filter_bandwith = 50;

	} else if (bandwidth <= 105) {
		setbits |= BW_XL_105_HZ;
		_accel_onchip_filter_bandwith = 105;

	} else if (bandwidth <= 211) {
		setbits |= BW_XL_211_HZ;
		_accel_onchip_filter_bandwith = 211;

	} else if (bandwidth <= 408) {
		setbits |= BW_XL_408_HZ;
		_accel_onchip_filter_bandwith = 408;

	} else {
		return -EINVAL;
	}
	modify_reg(CTRL_REG6_XL, clearbits, setbits);

	return OK;
}

int
LSM9DS1::accel_set_driver_lowpass_filter(float samplerate, float bandwidth)
{
	_accel_filter_x.set_cutoff_frequency(samplerate, bandwidth);
	_accel_filter_y.set_cutoff_frequency(samplerate, bandwidth);
	_accel_filter_z.set_cutoff_frequency(samplerate, bandwidth);

	return OK;
}

int
LSM9DS1::gyro_set_driver_lowpass_filter(float samplerate, float bandwidth)
{
    _gyro_filter_x.set_cutoff_frequency(samplerate, bandwidth);
    _gyro_filter_y.set_cutoff_frequency(samplerate, bandwidth);
    _gyro_filter_z.set_cutoff_frequency(samplerate, bandwidth);

    return OK;
}

int
LSM9DS1::accel_set_samplerate(unsigned frequency)
{
	uint8_t setbits = 0;
	uint8_t clearbits = (LSM_A_ODR_952_HZ | (1<<5));

	if (frequency == 0 || frequency == LSM9DS1_AG_DEFAULT_RATE) {
		frequency = 952;
	}

	if (frequency <= 10) {
		setbits |= LSM_A_ODR_10_HZ;
		_accel_samplerate = 10;

	} else if (frequency <= 50) {
		setbits |= LSM_A_ODR_50_HZ;
		_accel_samplerate = 50;

	} else if (frequency <= 119) {
		setbits |= LSM_A_ODR_119_HZ;
		_accel_samplerate = 119;

	} else if (frequency <= 238) {
		setbits |= LSM_A_ODR_238_HZ;
		_accel_samplerate = 238;

	} else if (frequency <= 476) {
        setbits |= LSM_A_ODR_476_HZ;
		_accel_samplerate = 476;

	} else if (frequency > 476) {
		setbits |= LSM_A_ODR_952_HZ;
		_accel_samplerate = 952;

	} else {
		return -EINVAL;
	}

	modify_reg(CTRL_REG6_XL, clearbits, setbits);

	return OK;
}

int
LSM9DS1::gyro_set_samplerate(unsigned frequency)
{
    uint8_t setbits = 0;
    uint8_t clearbits = (LSM_AG_ODR_952_HZ | (1<<5));

    if (frequency == 0 || frequency == LSM9DS1_AG_DEFAULT_RATE) {
        frequency = 952;
    }

    if (frequency <= 14.9f) {
        setbits |= LSM_AG_ODR_14_9_HZ;
        _gyro_samplerate = 14.9f;

    } else if (frequency <= 59.5f) {
        setbits |= LSM_AG_ODR_59_5_HZ;
        _gyro_samplerate = 59.5f;

    } else if (frequency <= 119) {
        setbits |= LSM_AG_ODR_119_HZ;
        _gyro_samplerate = 119;

    } else if (frequency <= 238) {
        setbits |= LSM_AG_ODR_238_HZ;
        _gyro_samplerate = 238;

    } else if (frequency <= 476) {
        setbits |= LSM_AG_ODR_476_HZ;
        _gyro_samplerate = 476;

    } else if (frequency > 476) {
        setbits |= LSM_AG_ODR_952_HZ;
        _gyro_samplerate = 952;

    } else {
        return -EINVAL;
    }

    modify_reg(CTRL_REG1_G, clearbits, setbits);

    return OK;
}

void
LSM9DS1::start()
{
	/* make sure we are stopped first */
	stop();
    reset();
	/* reset the report ring */
	_accel_reports->flush();
    _gyro_reports->flush();

	/* start polling at the specified rate */
	hrt_call_every(&_accel_call,
		       1000,
		       _call_accel_interval - LSM9DS1_TIMER_REDUCTION,
		       (hrt_callout)&LSM9DS1::measure_trampoline, this);
    hrt_call_every(&_gyro_call, 1000, _call_gyro_interval, (hrt_callout)&LSM9DS1::gyro_measure_trampoline, this);
}

void
LSM9DS1::stop()
{
	hrt_cancel(&_accel_call);
    hrt_cancel(&_gyro_call);

	/* reset internal states */
	memset(_last_accel, 0, sizeof(_last_accel));

	/* discard unread data in the buffers */
	_accel_reports->flush();
    _gyro_reports->flush();
}

void
LSM9DS1::measure_trampoline(void *arg)
{
	LSM9DS1 *dev = (LSM9DS1 *)arg;

	/* make another measurement */
	dev->measure();
}

void
LSM9DS1::gyro_measure_trampoline(void *arg)
{
	LSM9DS1 *dev = (LSM9DS1 *)arg;

	/* make another measurement */
    dev->gyro_measure();
}

void
LSM9DS1::check_registers(void)
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

	_checked_next = (_checked_next + 1) % LSM9DS1_NUM_CHECKED_REGISTERS;
}

void
LSM9DS1::measure()
{
	/* status register and data as read back from the device */

#pragma pack(push, 1)
	struct {
		uint8_t		cmd;
		uint8_t		status;
		int16_t		x;
		int16_t		y;
		int16_t		z;
	} raw_accel_report;
#pragma pack(pop)

	accel_report accel_report;

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
	memset(&raw_accel_report, 0, sizeof(raw_accel_report));
	raw_accel_report.cmd = STATUS_REG2 | DIR_READ;
	transfer((uint8_t *)&raw_accel_report, (uint8_t *)&raw_accel_report, sizeof(raw_accel_report));

    //ESTÁ A FALHAR AQUI!!!!!!!

    if (!(raw_accel_report.status & (0x01))) { //REG 0x27 XLDA (accelerometer new data available)
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

	// use the temperature from the last mag reading
    
    accel_report.temperature = _last_temperature;
    //accel_report.temperature = 25; // VER GYRO TEMP
    
    
	// report the error count as the sum of the number of bad
	// register reads and bad values. This allows the higher level
	// code to decide if it should use this sensor based on
	// whether it has had failures
	accel_report.error_count = perf_event_count(_bad_registers) + perf_event_count(_bad_values);

    accel_report.x_raw = raw_accel_report.x;
    accel_report.y_raw = -raw_accel_report.y;
    accel_report.z_raw = raw_accel_report.z;

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

	math::Vector<3> aval(x_in_new, y_in_new, z_in_new);
	math::Vector<3> aval_integrated;

	bool accel_notify = _accel_int.put(accel_report.timestamp, aval, aval_integrated, accel_report.integral_dt);
	accel_report.x_integral = aval_integrated(0);
	accel_report.y_integral = aval_integrated(1);
	accel_report.z_integral = aval_integrated(2);

	accel_report.scaling = _accel_range_scale;
	accel_report.range_m_s2 = _accel_range_m_s2;

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

void
LSM9DS1::gyro_measure()
{
	/* status register and data as read back from the device */
#pragma pack(push, 1)
	struct {
		uint8_t		cmd;
        //int16_t		temperature;
		uint8_t		status;
		int16_t		x;
		int16_t		y;
		int16_t		z;
    } raw_gyro_report;
#pragma pack(pop)

    gyro_report gyro_report;
    memset(&gyro_report, 0, sizeof(gyro_report));

	/* start the performance counter */
    perf_begin(_gyro_sample_perf);

	/* fetch data from the sensor */
    memset(&raw_gyro_report, 0, sizeof(raw_gyro_report));
    raw_gyro_report.cmd = STATUS_REG1 | DIR_READ;
    //raw_gyro_report.cmd = OUT_TEMP_L | DIR_READ;
    transfer((uint8_t *)&raw_gyro_report, (uint8_t *)&raw_gyro_report, sizeof(raw_gyro_report));

    if (!(raw_gyro_report.status & (0x02))) { //REG 0x17 GDA (gyroscope new data available)
        perf_end(_gyro_sample_perf);
        perf_count(_gyro_duplicates);
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


    gyro_report.timestamp = hrt_absolute_time();

    gyro_report.x_raw = raw_gyro_report.x;
    gyro_report.y_raw = -raw_gyro_report.y;
    gyro_report.z_raw = raw_gyro_report.z;

    float xraw_f = gyro_report.x_raw;
    float yraw_f = gyro_report.y_raw;
    float zraw_f = gyro_report.z_raw;

	/* apply user specified rotation */
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

    float xin = ((xraw_f * _gyro_range_scale) - _gyro_scale.x_offset) * _gyro_scale.x_scale;
    float yin = ((yraw_f * _gyro_range_scale) - _gyro_scale.y_offset) * _gyro_scale.y_scale;
    float zin = ((zraw_f * _gyro_range_scale) - _gyro_scale.z_offset) * _gyro_scale.z_scale;

    gyro_report.x = _gyro_filter_x.apply(xin);
    gyro_report.y = _gyro_filter_y.apply(yin);
    gyro_report.z = _gyro_filter_z.apply(zin);

    math::Vector<3> gval(xin, yin, zin);
    math::Vector<3> gval_integrated;

    bool gyro_notify = _gyro_int.put(gyro_report.timestamp, gval, gval_integrated, gyro_report.integral_dt);
    gyro_report.x_integral = gval_integrated(0);
    gyro_report.y_integral = gval_integrated(1);
    gyro_report.z_integral = gval_integrated(2);


    gyro_report.scaling = _gyro_range_scale;
    gyro_report.range_rad_s = (float)_gyro_range_rad_s;
    gyro_report.error_count = perf_event_count(_bad_registers) + perf_event_count(_bad_values);

	/* remember the temperature. The datasheet isn't clear, but it
	 * seems to be a signed offset from 25 degrees C in units of 0.125C
	 */
    //_last_temperature = 25 + (raw_gyro_report.temperature * 0.125f);

    uint8_t temp_l = read_reg(OUT_TEMP_L);
    uint8_t temp_h = read_reg(OUT_TEMP_H);
    temp_h=(temp_h && 0x0F);


    _last_temperature = 25 + ((temp_h<<8) + temp_l) * 0.0625f;
    gyro_report.temperature = _last_temperature;

    _gyro_reports->force(&gyro_report);

    if (gyro_notify) {
        /* notify anyone waiting for data */
        _gyro->parent_poll_notify();

        /* publish for subscribers */
        if (!(_pub_blocked)) {
            /* publish it */
            orb_publish(ORB_ID(sensor_gyro), _gyro->_gyro_topic, &gyro_report);
        }
    }

    _gyro_read++;

	/* stop the perf counter */
    perf_end(_gyro_sample_perf);
}

void
LSM9DS1::print_info()
{
	printf("accel reads:          %u\n", _accel_read);
    printf("gyro reads:            %u\n", _gyro_read);
	perf_print_counter(_accel_sample_perf);
    perf_print_counter(_gyro_sample_perf);
	perf_print_counter(_bad_registers);
	perf_print_counter(_bad_values);
	perf_print_counter(_accel_duplicates);
    perf_print_counter(_gyro_duplicates);
	_accel_reports->print_info("accel reports");
    _gyro_reports->print_info("gyro reports");
	::printf("checked_next: %u\n", _checked_next);

	for (uint8_t i = 0; i < LSM9DS1_NUM_CHECKED_REGISTERS; i++) {
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
LSM9DS1::print_registers()
{
	/*const struct {
		uint8_t reg;
		const char *name;
	} regmap[] = {
		{ ADDR_WHO_AM_I,    "WHO_AM_I" },
		{ 0x02,             "I2C_CONTROL1" },
		{ 0x15,             "I2C_CONTROL2" },
		{ ADDR_STATUS_A,    "STATUS_A" },
		{ ADDR_STATUS_M,    "STATUS_M" },
		{ ADDR_CTRL_REG0,   "CTRL_REG0" },
		{ ADDR_CTRL_REG1,   "CTRL_REG1" },
		{ ADDR_CTRL_REG2,   "CTRL_REG2" },
		{ ADDR_CTRL_REG3,   "CTRL_REG3" },
		{ ADDR_CTRL_REG4,   "CTRL_REG4" },
		{ ADDR_CTRL_REG5,   "CTRL_REG5" },
		{ ADDR_CTRL_REG6,   "CTRL_REG6" },
		{ ADDR_CTRL_REG7,   "CTRL_REG7" },
		{ ADDR_OUT_TEMP_L,  "TEMP_L" },
		{ ADDR_OUT_TEMP_H,  "TEMP_H" },
		{ ADDR_INT_CTRL_M,  "INT_CTRL_M" },
		{ ADDR_INT_SRC_M,   "INT_SRC_M" },
		{ ADDR_REFERENCE_X, "REFERENCE_X" },
        { ADDR_REFERENCE_Y, "REFERENCE_Y" },
		{ ADDR_REFERENCE_Z, "REFERENCE_Z" },
		{ ADDR_OUT_X_L_A,   "ACCEL_XL" },
		{ ADDR_OUT_X_H_A,   "ACCEL_XH" },
		{ ADDR_OUT_Y_L_A,   "ACCEL_YL" },
		{ ADDR_OUT_Y_H_A,   "ACCEL_YH" },
		{ ADDR_OUT_Z_L_A,   "ACCEL_ZL" },
		{ ADDR_OUT_Z_H_A,   "ACCEL_ZH" },
		{ ADDR_FIFO_CTRL,   "FIFO_CTRL" },
		{ ADDR_FIFO_SRC,    "FIFO_SRC" },
		{ ADDR_IG_CFG1,     "IG_CFG1" },
		{ ADDR_IG_SRC1,     "IG_SRC1" },
		{ ADDR_IG_THS1,     "IG_THS1" },
		{ ADDR_IG_DUR1,     "IG_DUR1" },
		{ ADDR_IG_CFG2,     "IG_CFG2" },
		{ ADDR_IG_SRC2,     "IG_SRC2" },
		{ ADDR_IG_THS2,     "IG_THS2" },
		{ ADDR_IG_DUR2,     "IG_DUR2" },
		{ ADDR_CLICK_CFG,   "CLICK_CFG" },
		{ ADDR_CLICK_SRC,   "CLICK_SRC" },
		{ ADDR_CLICK_THS,   "CLICK_THS" },
		{ ADDR_TIME_LIMIT,  "TIME_LIMIT" },
		{ ADDR_TIME_LATENCY, "TIME_LATENCY" },
		{ ADDR_TIME_WINDOW, "TIME_WINDOW" },
		{ ADDR_ACT_THS,     "ACT_THS" },
		{ ADDR_ACT_DUR,     "ACT_DUR" }
	};

	for (uint8_t i = 0; i < sizeof(regmap) / sizeof(regmap[0]); i++) {
		printf("0x%02x %s\n", read_reg(regmap[i].reg), regmap[i].name);
	}*/
}

void
LSM9DS1::test_error()
{
	// trigger an error
	//write_reg(ADDR_CTRL_REG3, 0);
}

LSM9DS1_gyro::LSM9DS1_gyro(LSM9DS1 *parent) :
    CDev("LSM9DS1_gyro", LSM9DS1_DEVICE_PATH_GYRO),
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

	ret = CDev::init();

	if (ret != OK) {
		goto out;
	}

    _gyro_class_instance = register_class_devname(GYRO_BASE_DEVICE_PATH);

out:
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

void
LSM9DS1_gyro::measure()
{
    _parent->gyro_measure();
}

void
LSM9DS1_gyro::measure_trampoline(void *arg)
{
    _parent->gyro_measure_trampoline(arg);
}

/**
 * Local functions in support of the shell command.
 */
namespace lsm9ds1
{

LSM9DS1	*g_dev;

void	start(bool external_bus, enum Rotation rotation, unsigned range);
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
start(bool external_bus, enum Rotation rotation, unsigned range)
{
    int fd, fd_gyro;

    if ((g_dev != nullptr)) {
		errx(0, "already started");
	}

	/* create the driver */
	if (external_bus) {
#ifdef PX4_SPI_BUS_EXT
		g_dev = new LSM9DS1(PX4_SPI_BUS_EXT, LSM9DS1_DEVICE_PATH_ACCEL, (spi_dev_e)PX4_SPIDEV_EXT_ACCEL_GYRO, rotation);
#else
		errx(0, "External SPI not available");
#endif

	} else {
        g_dev = new LSM9DS1(PX4_SPI_BUS_SENSORS, LSM9DS1_DEVICE_PATH_ACCEL, (spi_dev_e)PX4_SPIDEV_ACCEL_GYRO, rotation);
	}

	if (g_dev == nullptr) {
		warnx("failed instantiating LSM9DS1 ACCEL obj");
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(LSM9DS1_DEVICE_PATH_ACCEL, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	if (ioctl(fd, ACCELIOCSRANGE, range) < 0) {
		goto fail;
	}

    fd_gyro = open(LSM9DS1_DEVICE_PATH_GYRO, O_RDONLY);

	/* don't fail if open cannot be opened */
    if (0 <= fd_gyro) {
        if (ioctl(fd_gyro, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
			goto fail;
		}
	}

	close(fd);
    close(fd_gyro);

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
    int fd_accel = -1;
    struct accel_report a_report;
    ssize_t sz;
    int ret;

    /* get the driver */
    fd_accel = open(LSM9DS1_DEVICE_PATH_ACCEL, O_RDONLY);

    if (fd_accel < 0) {
        err(1, "%s open failed", LSM9DS1_DEVICE_PATH_ACCEL);
    }

    /* do a simple demand read */
    sz = read(fd_accel, &a_report, sizeof(a_report));

    if (sz != sizeof(a_report)) {
        err(1, "immediate read accel failed");
    }


    warnx("accel x: \t% 9.5f\tm/s^2", (double)a_report.x);
    warnx("accel y: \t% 9.5f\tm/s^2", (double)a_report.y);
    warnx("accel z: \t% 9.5f\tm/s^2", (double)a_report.z);
    warnx("accel x: \t%d\traw", (int)a_report.x_raw);
    warnx("accel y: \t%d\traw", (int)a_report.y_raw);
    warnx("accel z: \t%d\traw", (int)a_report.z_raw);

    warnx("accel range: %8.4f m/s^2", (double)a_report.range_m_s2);

    if (ERROR == (ret = ioctl(fd_accel, ACCELIOCGLOWPASS, 0))) {
        warnx("accel antialias filter bandwidth: fail");

    } else {
        warnx("accel antialias filter bandwidth: %d Hz", ret);
    }

    int fd_gyro = -1;
    struct gyro_report g_report;

    /* get the driver */
    fd_gyro = open(LSM9DS1_DEVICE_PATH_GYRO, O_RDONLY);

    if (fd_gyro < 0) {
        err(1, "%s open failed", LSM9DS1_DEVICE_PATH_GYRO);
    }

    /* do a simple demand read */
    sz = read(fd_gyro, &g_report, sizeof(g_report));

    if (sz != sizeof(g_report)) {
        err(1, "immediate read failed");
    }

    warnx("gyro x: \t% 9.5f\trad/s", (double)g_report.x);
    warnx("gyro y: \t% 9.5f\trad/s", (double)g_report.y);
    warnx("gyro z: \t% 9.5f\trad/s", (double)g_report.z);
    warnx("gyro x: \t%d\traw", (int)g_report.x_raw);
    warnx("gyro y: \t%d\traw", (int)g_report.y_raw);
    warnx("gyro z: \t%d\traw", (int)g_report.z_raw);
    warnx("gyro range: %8.4f rad/s (%d deg/s)", (double)g_report.range_rad_s,
          (int)((g_report.range_rad_s / M_PI_F) * 180.0f + 0.5f));

    /* reset to default polling */
    if (ioctl(fd_accel, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        err(1, "reset to default polling");
    }

    close(fd_accel);
    close(fd_gyro);

    reset();
    errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
    int fd = open(LSM9DS1_DEVICE_PATH_ACCEL, O_RDONLY);

    if (fd < 0) {
        err(1, "failed ");
    }

    if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
        err(1, "driver reset failed");
    }

    if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        err(1, "accel pollrate reset failed");
    }

    close(fd);

    fd = open(LSM9DS1_DEVICE_PATH_GYRO, O_RDONLY);

    if (fd < 0) {
        warnx("gyro could not be opened, external mag might be used");

    } else {
        /* no need to reset the gyro as well, the reset() is the same */
        if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
            err(1, "gyro pollrate reset failed");
        }
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
        errx(1, "driver not running\n");
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
        errx(1, "driver not running\n");
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
        errx(1, "driver not running\n");
    }

    g_dev->test_error();

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
lsm9ds1_main(int argc, char *argv[])
{
    bool external_bus = false;
    int ch;
    enum Rotation rotation = ROTATION_NONE;
    int accel_range = 16;

    /* jump over start/off/etc and look at options first */
    while ((ch = getopt(argc, argv, "XR:a:")) != EOF) {
        switch (ch) {
        case 'X':
            external_bus = true;
            break;

        case 'R':
            rotation = (enum Rotation)atoi(optarg);
            break;

        case 'a':
            accel_range = atoi(optarg);
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
        lsm9ds1::start(external_bus, rotation, accel_range);
    }

    /*
     * Test the driver/device.
     */
    if (!strcmp(verb, "test")) {
        lsm9ds1::test();
    }

    /*
     * Reset the driver.
     */
    if (!strcmp(verb, "reset")) {
        lsm9ds1::reset();
    }

    /*
     * Print driver information.
     */
    if (!strcmp(verb, "info")) {
        lsm9ds1::info();
    }

    /*
     * dump device registers
     */
    if (!strcmp(verb, "regdump")) {
        lsm9ds1::regdump();
    }

    /*
     * trigger an error
     */
    if (!strcmp(verb, "testerror")) {
        lsm9ds1::test_error();
    }

    errx(1, "unrecognized command, try 'start', 'test', 'reset', 'info', 'testerror' or 'regdump'");
}
