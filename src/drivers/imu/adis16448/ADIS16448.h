/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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
 * @file adis16448.cpp
 *
 * Driver for the Analog device ADIS16448 connected via SPI.
 *
 * @author Amir Melzer
 * @author Andrew Tridgell
 * @author Pat Hickey
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <ecl/geo/geo.h>
#include <perf/perf_counter.h>

#include <drivers/device/spi.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_mag.h>
#include <conversion/rotation.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <systemlib/err.h>

#include "ADIS16448_accel.h"
#include "ADIS16448_gyro.h"
#include "ADIS16448_mag.h"

#define DIR_READ                     0x00
#define DIR_WRITE                    0x80

#define ADIS16448_DEVICE_PATH_ACCEL  "/dev/adis16448_accel"
#define ADIS16448_DEVICE_PATH_GYRO   "/dev/adis16448_gyro"
#define ADIS16448_DEVICE_PATH_MAG    "/dev/adis16448_mag"

/* ADIS16448 registers */
#define ADIS16448_FLASH_CNT          0x00  /* Flash memory write count */
#define ADIS16448_SUPPLY_OUT         0x02  /* Power supply measurement */
#define ADIS16448_XGYRO_OUT          0x04  /* X-axis gyroscope output */
#define ADIS16448_YGYRO_OUT          0x06  /* Y-axis gyroscope output */
#define ADIS16448_ZGYRO_OUT          0x08  /* Z-axis gyroscope output */
#define ADIS16448_XACCL_OUT          0x0A  /* X-axis accelerometer output */
#define ADIS16448_YACCL_OUT          0x0C  /* Y-axis accelerometer output */
#define ADIS16448_ZACCL_OUT          0x0E  /* Z-axis accelerometer output */
#define ADIS16448_XMAGN_OUT          0x10  /* X-axis magnetometer measurement */
#define ADIS16448_YMAGN_OUT          0x12  /* Y-axis magnetometer measurement */
#define ADIS16448_ZMAGN_OUT          0x14  /* Z-axis magnetometer measurement */
#define ADIS16448_BARO_OUT           0x16  /* Barometric pressure output */
#define ADIS16448_TEMP_OUT           0x18  /* Temperature output */

/* Calibration parameters */
#define ADIS16448_XGYRO_OFF          0x1A  /* X-axis gyroscope bias offset factor */
#define ADIS16448_YGYRO_OFF          0x1C  /* Y-axis gyroscope bias offset factor */
#define ADIS16448_ZGYRO_OFF          0x1E  /* Z-axis gyroscope bias offset factor */
#define ADIS16448_XACCL_OFF          0x20  /* X-axis acceleration bias offset factor */
#define ADIS16448_YACCL_OFF          0x22  /* Y-axis acceleration bias offset factor */
#define ADIS16448_ZACCL_OFF          0x24  /* Z-axis acceleration bias offset factor */
#define ADIS16448_XMAGN_HIC          0x26  /* X-axis magnetometer, hard-iron factor */
#define ADIS16448_YMAGN_HIC          0x28  /* Y-axis magnetometer, hard-iron factor */
#define ADIS16448_ZMAGN_HIC          0x2A  /* Z-axis magnetometer, hard-iron factor */
#define ADIS16448_XMAGN_SIC          0x2C  /* X-axis magnetometer, soft-iron factor */
#define ADIS16448_YMAGN_SIC          0x2E  /* Y-axis magnetometer, soft-iron factor */
#define ADIS16448_ZMAGN_SIC          0x30  /* Z-axis magnetometer, soft-iron factor */

#define ADIS16448_GPIO_CTRL          0x32  /* Auxiliary digital input/output control */
#define ADIS16448_MSC_CTRL           0x34  /* Miscellaneous control */
#define ADIS16448_SMPL_PRD           0x36  /* Internal sample period (rate) control */
#define ADIS16448_SENS_AVG           0x38  /* Dynamic range and digital filter control */
#define ADIS16448_SLP_CNT            0x3A  /* Sleep mode control */
#define ADIS16448_DIAG_STAT          0x3C  /* System status */

/* Alarm functions */
#define ADIS16448_GLOB_CMD           0x3E  /* System command */
#define ADIS16448_ALM_MAG1           0x40  /* Alarm 1 amplitude threshold */
#define ADIS16448_ALM_MAG2           0x42  /* Alarm 2 amplitude threshold */
#define ADIS16448_ALM_SMPL1          0x44  /* Alarm 1 sample size */
#define ADIS16448_ALM_SMPL2          0x46  /* Alarm 2 sample size */
#define ADIS16448_ALM_CTRL           0x48  /* Alarm control */

#define ADIS16334_LOT_ID1            0x52  /* Lot identification code 1 */
#define ADIS16334_LOT_ID2            0x54  /* Lot identification code 2 */
#define ADIS16448_PRODUCT_ID         0x56  /* Product identifier */
#define ADIS16334_SERIAL_NUMBER      0x58  /* Serial number, lot specific */

#define ADIS16448_Product            0x4040/* Product ID Description for ADIS16448 */

#define BITS_SMPL_PRD_NO_TAP_CFG     (0<<8)
#define BITS_SMPL_PRD_2_TAP_CFG      (1<<8)
#define BITS_SMPL_PRD_4_TAP_CFG      (2<<8)
#define BITS_SMPL_PRD_8_TAP_CFG      (3<<8)
#define BITS_SMPL_PRD_16_TAP_CFG     (4<<8)

#define BITS_GYRO_DYN_RANGE_1000_CFG (4<<8)
#define BITS_GYRO_DYN_RANGE_500_CFG  (2<<8)
#define BITS_GYRO_DYN_RANGE_250_CFG  (1<<8)

#define BITS_FIR_NO_TAP_CFG          (0<<0)
#define BITS_FIR_2_TAP_CFG           (1<<0)
#define BITS_FIR_4_TAP_CFG           (2<<0)
#define BITS_FIR_8_TAP_CFG           (3<<0)
#define BITS_FIR_16_TAP_CFG          (4<<0)
#define BITS_FIR_32_TAP_CFG          (5<<0)
#define BITS_FIR_64_TAP_CFG          (6<<0)
#define BITS_FIR_128_TAP_CFG         (7<<0)


#define ADIS16448_ACCEL_DEFAULT_RATE               100
#define ADIS16448_ACCEL_DEFAULT_DRIVER_FILTER_FREQ 30

#define ADIS16448_GYRO_DEFAULT_RATE                100
#define ADIS16448_GYRO_DEFAULT_DRIVER_FILTER_FREQ  30

#define ADIS16448_MAG_DEFAULT_RATE                 100
#define ADIS16448_MAG_DEFAULT_DRIVER_FILTER_FREQ   30

#define ADIS16448_ACCEL_MAX_OUTPUT_RATE 1221
#define ADIS16448_GYRO_MAX_OUTPUT_RATE  1221

#define FW_FILTER                       false

#define SPI_BUS_SPEED                   1000000
#define T_STALL                         9

#define ACCEL_INITIAL_SENSITIVITY       (1.0f / 1200.0f)
#define GYRO_INITIAL_SENSITIVITY        250
#define MAG_INITIAL_SENSITIVITY         (1.0f / 7.0f)

#define ACCEL_DYNAMIC_RANGE             18.0f   // G's
#define GYRO_DYNAMIC_RANGE              1000.0f // Degrees/sec
#define MAG_DYNAMIC_RANGE               1900.0f // Gauss

/* Forward class declarations. */
class ADIS16448_accel;
class ADIS16448_gyro;
class ADIS16448_mag;

class ADIS16448 : public device::SPI
{
public:
	ADIS16448(int bus, const char *path_accel, const char *path_gyro, const char *path_mag,
		  uint32_t device, enum Rotation rotation);
	virtual ~ADIS16448();

	virtual int init();
	virtual int ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver and sensor.
	 */
	void print_info();
	void print_calibration_data();

protected:
	virtual int probe();

	friend class ADIS16448_accel;
	friend class ADIS16448_gyro;
	friend class ADIS16448_mag;

	virtual ssize_t accel_read(struct file *filp, char *buffer, size_t buflen);
	virtual ssize_t gyro_read(struct file *filp, char *buffer, size_t buflen);
	virtual ssize_t mag_read(struct file *filp, char *buffer, size_t buflen);

	virtual int accel_ioctl(struct file *filp, int cmd, unsigned long arg);
	virtual int gyro_ioctl(struct file *filp, int cmd, unsigned long arg);
	virtual int mag_ioctl(struct file *filp, int cmd, unsigned long arg);

private:

	/**
	 * Fetch measurements from the sensor and update the report buffers.
	 */
	int measure();

	/**
	 * Static trampoline from the hrt_call context; because we don't have a
	 * generic hrt wrapper yet.
	 * Called by the HRT in interrupt context at the specified rate if
	 * automatic polling is enabled.
	 *
	 * @param arg       Instance pointer for the driver that is polling.
	 */
	static void measure_trampoline(void *arg);

	/**
	 * convert 12 bit integer format to int16.
	 */
	int16_t convert_12bit_to_int16(int16_t word);

	/**
	 * Modify a register in the ADIS16448
	 * Bits are cleared before bits are set.
	 *
	 * @param reg       The register to modify.
	 * @param clearbits Bits in the register to clear.
	 * @param setbits   Bits in the register to set.
	 */
	void modify_reg16(unsigned reg, uint16_t clearbits, uint16_t setbits);

	/**
	 * Resets the chip and measurements ranges, but not scale and offset.
	 */
	int reset();

	/**
	 * Read a register from the ADIS16448
	 * @arg reg The register to read.
	 * @return Returns the register value.
	 */
	uint16_t read_reg16(unsigned reg);

	/**
	 * Write a register in the ADIS16448
	 * @param reg       The register to write.
	 * @param value     The new value to write.
	 */
	void write_reg16(unsigned reg, uint16_t value);

	/**
	 * Measurement self test.
	 * @return 0 on success, 1 on failure.
	 */
	int self_test();

	/**
	 * Set low pass filter frequency.
	 */
	void _set_dlpf_filter(uint16_t frequency_hz);

	/**
	 * Set IMU to factory default.
	 */
	void _set_factory_defaults();

	/**
	 * Set the gyroscope dynamic range.
	 */
	void _set_gyro_dyn_range(uint16_t desired_gyro_dyn_range);

	/**
	 * Set sample rate (approximate) - 1kHz to 5Hz.
	 */
	void _set_sample_rate(uint16_t desired_sample_rate_hz);

	/**
	 * Start automatic measurement.
	 */
	void start();

	/**
	 * Stop automatic measurement.
	 */
	void stop();

	/**
	 * Swap a 16-bit value read from the ADIS16448 to native byte order.
	 */
	uint16_t swap16(uint16_t val) { return (val >> 8) | (val << 8); }


#pragma pack(push, 1) // Ensure proper memory alignment.
	//  Report conversation within the ADIS16448, including command byte and interrupt status.
	struct ADISReport {
		uint16_t cmd{0};
		uint16_t status{0};
		uint16_t accel_x{0};
		uint16_t accel_y{0};
		uint16_t accel_z{0};
		uint16_t gyro_x{0};
		uint16_t gyro_y{0};
		uint16_t gyro_z{0};
		uint16_t mag_x{0};
		uint16_t mag_y{0};
		uint16_t mag_z{0};
		uint16_t baro{0};
		uint16_t temp{0};
	};
#pragma pack(pop)

	struct hrt_call _call {};

	uint16_t _product_ID{0};    // Product ID code.

	unsigned int _call_interval{0};

	ADIS16448_accel *_accel{nullptr};
	ADIS16448_gyro *_gyro{nullptr};
	ADIS16448_mag *_mag{nullptr};

	ringbuffer::RingBuffer *_accel_reports{nullptr};
	ringbuffer::RingBuffer *_gyro_reports{nullptr};
	ringbuffer::RingBuffer *_mag_reports{nullptr};

	struct accel_calibration_s _accel_cal {accel_calibration_s()};
	struct gyro_calibration_s _gyro_cal {gyro_calibration_s()};
	struct mag_calibration_s _mag_cal {mag_calibration_s()};

	float _accel_range_scale{0.0f};
	float _accel_range_m_s2{0.0f};

	float _gyro_range_scale{0.0f};
	float _gyro_range_rad_s{0.0f};

	float _mag_range_scale{0.0f};
	float _mag_range_mgauss{0.0f};

	unsigned int _sample_rate{100};  // Init sampling frequency set to 100Hz.

	perf_counter_t _accel_reads   = perf_counter_t(perf_alloc(PC_COUNT, "adis16448_accel_read"));
	perf_counter_t _gyro_reads    = perf_counter_t(perf_alloc(PC_COUNT, "adis16448_gyro_read"));
	perf_counter_t _mag_reads     = perf_counter_t(perf_alloc(PC_COUNT, "adis16448_mag_read"));
	perf_counter_t _sample_perf   = perf_counter_t(perf_alloc(PC_ELAPSED, "adis16448_read"));
	perf_counter_t _bad_transfers = perf_counter_t(perf_alloc(PC_COUNT, "adis16448_bad_transfers"));

	math::LowPassFilter2p _accel_filter_x = math::LowPassFilter2p(ADIS16448_ACCEL_DEFAULT_RATE,
						ADIS16448_ACCEL_DEFAULT_DRIVER_FILTER_FREQ);
	math::LowPassFilter2p _accel_filter_y = math::LowPassFilter2p(ADIS16448_ACCEL_DEFAULT_RATE,
						ADIS16448_ACCEL_DEFAULT_DRIVER_FILTER_FREQ);
	math::LowPassFilter2p _accel_filter_z = math::LowPassFilter2p(ADIS16448_ACCEL_DEFAULT_RATE,
						ADIS16448_ACCEL_DEFAULT_DRIVER_FILTER_FREQ);
	math::LowPassFilter2p _gyro_filter_x  = math::LowPassFilter2p(ADIS16448_GYRO_DEFAULT_RATE,
						ADIS16448_GYRO_DEFAULT_DRIVER_FILTER_FREQ);
	math::LowPassFilter2p _gyro_filter_y  = math::LowPassFilter2p(ADIS16448_GYRO_DEFAULT_RATE,
						ADIS16448_GYRO_DEFAULT_DRIVER_FILTER_FREQ);
	math::LowPassFilter2p _gyro_filter_z  = math::LowPassFilter2p(ADIS16448_GYRO_DEFAULT_RATE,
						ADIS16448_GYRO_DEFAULT_DRIVER_FILTER_FREQ);
	math::LowPassFilter2p _mag_filter_y   = math::LowPassFilter2p(ADIS16448_MAG_DEFAULT_RATE,
						ADIS16448_MAG_DEFAULT_DRIVER_FILTER_FREQ);
	math::LowPassFilter2p _mag_filter_x   = math::LowPassFilter2p(ADIS16448_MAG_DEFAULT_RATE,
						ADIS16448_MAG_DEFAULT_DRIVER_FILTER_FREQ);
	math::LowPassFilter2p _mag_filter_z   = math::LowPassFilter2p(ADIS16448_MAG_DEFAULT_RATE,
						ADIS16448_MAG_DEFAULT_DRIVER_FILTER_FREQ);

	Integrator _accel_int{};
	Integrator _gyro_int{};

	enum Rotation _rotation = Rotation();

	// Disallow copy construction and move assignment of this class due to pointer data members.
	ADIS16448(const ADIS16448 &) = delete;
	ADIS16448 operator=(const ADIS16448 &) = delete;
};
