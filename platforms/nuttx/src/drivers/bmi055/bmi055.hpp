#ifndef BMI055_HPP_
#define BMI055_HPP_

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

#define DIR_READ                0x80
#define DIR_WRITE               0x00

#define BMI055_DEVICE_PATH_ACCEL    "/dev/bmi055_accel"
#define BMI055_DEVICE_PATH_GYRO		"/dev/bmi055_gyro"

#define BMI055_DEVICE_PATH_ACCEL_EXT    "/dev/bmi055_accel_ext"
#define BMI055_DEVICE_PATH_GYRO_EXT		"/dev/bmi055_gyro_ext"

// BMI055 Accel registers

#define BMI055_ACC_CHIP_ID          0x00
#define BMI055_ACC_X_L              0x02
#define BMI055_ACC_X_H              0x03
#define BMI055_ACC_Y_L              0x04
#define BMI055_ACC_Y_H              0x05
#define BMI055_ACC_Z_L              0x06
#define BMI055_ACC_Z_H              0x07
#define BMI055_ACC_TEMP             0x08
#define BMI055_ACC_INT_STATUS_0     0x09
#define BMI055_ACC_INT_STATUS_1     0x0A
#define BMI055_ACC_INT_STATUS_2     0x0B
#define BMI055_ACC_INT_STATUS_3     0x0C
#define BMI055_ACC_FIFO_STATUS      0x0E
#define BMI055_ACC_RANGE            0x0F
#define BMI055_ACC_BW               0x10
#define BMI055_ACC_PMU_LPW          0x11
#define BMI055_ACC_PMU_LOW_POWER    0x12
#define BMI055_ACC_DATA_CTRL        0x13
#define BMI055_ACC_SOFTRESET        0x14
#define BMI055_ACC_INT_EN_0         0x16
#define BMI055_ACC_INT_EN_1         0x17
#define BMI055_ACC_INT_EN_2         0x18
#define BMI055_ACC_INT_MAP_0        0x19
#define BMI055_ACC_INT_MAP_1        0x1A
#define BMI055_ACC_INT_MAP_2        0x1B
#define BMI055_ACC_INT_SRC          0x1E
#define BMI055_ACC_INT_OUT_CTRL     0x20
#define BMI055_ACC_INT_LATCH        0x21
#define BMI055_ACC_INT_LH_0         0x22
#define BMI055_ACC_INT_LH_1         0x23
#define BMI055_ACC_INT_LH_2         0x24
#define BMI055_ACC_INT_LH_3         0x25
#define BMI055_ACC_INT_LH_4         0x26
#define BMI055_ACC_INT_MOT_0        0x27
#define BMI055_ACC_INT_MOT_1        0x28
#define BMI055_ACC_INT_MOT_2        0x29
#define BMI055_ACC_INT_TAP_0        0x2A
#define BMI055_ACC_INT_TAP_1        0x2B
#define BMI055_ACC_INT_ORIE_0       0x2C
#define BMI055_ACC_INT_ORIE_1       0x2D
#define BMI055_ACC_INT_FLAT_0       0x2E
#define BMI055_ACC_INT_FLAT_1       0x2F
#define BMI055_ACC_FIFO_CONFIG_0    0x30
#define BMI055_ACC_SELF_TEST        0x32
#define BMI055_ACC_EEPROM_CTRL      0x33
#define BMI055_ACC_SERIAL_CTRL      0x34
#define BMI055_ACC_OFFSET_CTRL      0x36
#define BMI055_ACC_OFC_SETTING      0x37
#define BMI055_ACC_OFFSET_X         0x38
#define BMI055_ACC_OFFSET_Y         0x39
#define BMI055_ACC_OFFSET_Z         0x3A
#define BMI055_ACC_TRIM_GPO         0x3B
#define BMI055_ACC_TRIM_GP1         0x3C
#define BMI055_ACC_FIFO_CONFIG_1    0x3E
#define BMI055_ACC_FIFO_DATA        0x3F


// BMI055 Gyro registers

#define BMI055_GYR_CHIP_ID          0x00
#define BMI055_GYR_X_L              0x02
#define BMI055_GYR_X_H              0x03
#define BMI055_GYR_Y_L              0x04
#define BMI055_GYR_Y_H              0x05
#define BMI055_GYR_Z_L              0x06
#define BMI055_GYR_Z_H              0x07
#define BMI055_GYR_INT_STATUS_0     0x09
#define BMI055_GYR_INT_STATUS_1     0x0A
#define BMI055_GYR_INT_STATUS_2     0x0B
#define BMI055_GYR_INT_STATUS_3     0x0C
#define BMI055_GYR_FIFO_STATUS      0x0E
#define BMI055_GYR_RANGE            0x0F
#define BMI055_GYR_BW               0x10
#define BMI055_GYR_LPM1             0x11
#define BMI055_GYR_LPM2             0x12
#define BMI055_GYR_RATE_HBW         0x13
#define BMI055_GYR_SOFTRESET        0x14
#define BMI055_GYR_INT_EN_0         0x15
#define BMI055_GYR_INT_EN_1         0x16
#define BMI055_GYR_INT_MAP_0        0x17
#define BMI055_GYR_INT_MAP_1        0x18
#define BMI055_GYR_INT_MAP_2        0x19
#define BMI055_GYRO_0_REG           0x1A
#define BMI055_GYRO_1_REG           0x1B
#define BMI055_GYRO_2_REG           0x1C
#define BMI055_GYRO_3_REG           0x1E
#define BMI055_GYR_INT_LATCH        0x21
#define BMI055_GYR_INT_LH_0         0x22
#define BMI055_GYR_INT_LH_1         0x23
#define BMI055_GYR_INT_LH_2         0x24
#define BMI055_GYR_INT_LH_3         0x25
#define BMI055_GYR_INT_LH_4         0x26
#define BMI055_GYR_INT_LH_5         0x27
#define BMI055_GYR_SOC              0x31
#define BMI055_GYR_A_FOC            0x32
#define BMI055_GYR_TRIM_NVM_CTRL    0x33
#define BMI055_BGW_SPI3_WDT         0x34
#define BMI055_GYR_OFFSET_COMP      0x36
#define BMI055_GYR_OFFSET_COMP_X    0x37
#define BMI055_GYR_OFFSET_COMP_Y    0x38
#define BMI055_GYR_OFFSET_COMP_Z    0x39
#define BMI055_GYR_TRIM_GPO         0x3A
#define BMI055_GYR_TRIM_GP1         0x3B
#define BMI055_GYR_SELF_TEST        0x3C
#define BMI055_GYR_FIFO_CONFIG_0    0x3D
#define BMI055_GYR_FIFO_CONFIG_1    0x3E
#define BMI055_GYR_FIFO_DATA        0x3F


// BMI055 Accelerometer Chip-Id
#define BMI055_ACC_WHO_AM_I         0xFA

// BMI055 Gyroscope Chip-Id
#define BMI055_GYR_WHO_AM_I         0x0F



//BMI055_ACC_BW           0x10
#define BMI055_ACCEL_BW_7_81      (1<<3) | (0<<2) | (0<<1) | (0<<0)
#define BMI055_ACCEL_BW_15_63     (1<<3) | (0<<2) | (0<<1) | (1<<0)
#define BMI055_ACCEL_BW_31_25     (1<<3) | (0<<2) | (1<<1) | (0<<0)
#define BMI055_ACCEL_BW_62_5      (1<<3) | (0<<2) | (1<<1) | (1<<0)
#define BMI055_ACCEL_BW_125       (1<<3) | (1<<2) | (0<<1) | (0<<0)
#define BMI055_ACCEL_BW_250       (1<<3) | (1<<2) | (0<<1) | (1<<0)
#define BMI055_ACCEL_BW_500       (1<<3) | (1<<2) | (1<<1) | (0<<0)
#define BMI055_ACCEL_BW_1000      (1<<3) | (1<<2) | (1<<1) | (1<<0)

//BMI055_ACC_PMU_LPW     0x11
#define BMI055_ACCEL_NORMAL         (0<<7) | (0<<6) | (0<<5)
#define BMI055_ACCEL_DEEP_SUSPEND   (0<<7) | (0<<6) | (1<<5)
#define BMI055_ACCEL_LOW_POWER      (0<<7) | (1<<6) | (0<<5)
#define BMI055_ACCEL_SUSPEND        (1<<7) | (0<<6) | (0<<5)


//BMI055_ACC_RANGE        0x0F
#define BMI055_ACCEL_RANGE_2_G      (0<<3) | (0<<2) | (1<<1) | (1<<0)
#define BMI055_ACCEL_RANGE_4_G      (0<<3) | (1<<2) | (0<<1) | (1<<0)
#define BMI055_ACCEL_RANGE_8_G      (1<<3) | (0<<2) | (0<<1) | (0<<0)
#define BMI055_ACCEL_RANGE_16_G     (1<<3) | (1<<2) | (0<<1) | (0<<0)

//BMI055_GYR_BW         0x10
#define BMI055_GYRO_RATE_100        (0<<3) | (1<<2) | (1<<1) | (1<<0)
#define BMI055_GYRO_RATE_200        (0<<3) | (1<<2) | (1<<1) | (0<<0)
#define BMI055_GYRO_RATE_400        (0<<3) | (0<<2) | (1<<1) | (1<<0)
#define BMI055_GYRO_RATE_1000       (0<<3) | (0<<2) | (1<<1) | (0<<0)
#define BMI055_GYRO_RATE_2000       (0<<3) | (0<<2) | (0<<1) | (1<<0)

//BMI055_GYR_LPM1      0x11
#define BMI055_GYRO_NORMAL          (0<<7) | (0<<5)
#define BMI055_GYRO_DEEP_SUSPEND    (0<<7) | (1<<5)
#define BMI055_GYRO_SUSPEND         (1<<7) | (0<<5)

//BMI055_GYR_RANGE        0x0F
#define BMI055_GYRO_RANGE_2000_DPS  (0<<2) | (0<<1) | (0<<0)
#define BMI055_GYRO_RANGE_1000_DPS  (0<<2) | (0<<1) | (1<<0)
#define BMI055_GYRO_RANGE_500_DPS   (0<<2) | (1<<1) | (0<<0)
#define BMI055_GYRO_RANGE_250_DPS   (0<<2) | (1<<1) | (1<<0)
#define BMI055_GYRO_RANGE_125_DPS   (1<<2) | (0<<1) | (0<<0)


//BMI055_ACC_INT_EN_1      0x17
#define BMI055_ACC_DRDY_INT_EN      (1<<4)

//BMI055_GYR_INT_EN_0         0x15
#define BMI055_GYR_DRDY_INT_EN      (1<<7)

//BMI055_ACC_INT_MAP_1         0x1A
#define BMI055_ACC_DRDY_INT1        (1<<0)

//BMI055_GYR_INT_MAP_1     0x18
#define BMI055_GYR_DRDY_INT1        (1<<0)



//Soft-reset command Value
#define BMI055_SOFT_RESET       0xB6

// Default and Max values
#define BMI055_ACCEL_DEFAULT_RANGE_G		16
#define BMI055_GYRO_DEFAULT_RANGE_DPS		2000
#define BMI055_ACCEL_DEFAULT_RATE           1000
#define BMI055_ACCEL_MAX_RATE               1000
#define BMI055_ACCEL_MAX_PUBLISH_RATE       280
#define BMI055_GYRO_DEFAULT_RATE            1000
#define BMI055_GYRO_MAX_RATE                1000
#define BMI055_GYRO_MAX_PUBLISH_RATE        BMI055_ACCEL_MAX_PUBLISH_RATE

#define BMI055_ACCEL_DEFAULT_DRIVER_FILTER_FREQ 50

#define BMI055_GYRO_DEFAULT_DRIVER_FILTER_FREQ  50

#define BMI055_ONE_G                       9.80665f

#define BMI055_BUS_SPEED				10*1000*1000

#define BMI055_TIMER_REDUCTION				200

/* Mask definitions for ACCD_X_LSB, ACCD_Y_LSB and ACCD_Z_LSB Register */
#define BMI055_NEW_DATA_MASK                 0x01

/* Mask definitions for Gyro bandwidth */
#define BMI055_GYRO_BW_MASK                  0x0F

#ifdef PX4_SPI_BUS_EXT
#define EXTERNAL_BUS PX4_SPI_BUS_EXT
#else
#define EXTERNAL_BUS 0
#endif

class BMI055 : public device::SPI
{

protected:

	uint8_t         _whoami;    /** whoami result */

	struct hrt_call     _call;
	unsigned        _call_interval;



	unsigned        _dlpf_freq;

	perf_counter_t      _sample_perf;
	perf_counter_t      _bad_transfers;
	perf_counter_t      _bad_registers;
	perf_counter_t      _good_transfers;
	perf_counter_t      _reset_retries;
	perf_counter_t      _duplicates;
	perf_counter_t      _controller_latency_perf;

	uint8_t         _register_wait;
	uint64_t        _reset_wait;

	enum Rotation       _rotation;

	uint8_t         _checked_next;

	/**
	* Read a register from the BMI055
	*
	* @param       The register to read.
	* @return      The value that was read.
	*/
	uint8_t         read_reg(unsigned reg);
	uint16_t        read_reg16(unsigned reg);

	/**
	* Write a register in the BMI055
	*
	* @param reg       The register to write.
	* @param value     The new value to write.
	*/
	void            write_reg(unsigned reg, uint8_t value);

	/* do not allow to copy this class due to pointer data members */
	BMI055(const BMI055 &);
	BMI055 operator=(const BMI055 &);

public:

	BMI055(const char *name, const char *devname, int bus, enum spi_dev_e device, enum spi_mode_e mode, uint32_t frequency,
	       enum Rotation rotation);
	virtual ~BMI055();


};



class BMI055_accel : public BMI055
{
public:
	BMI055_accel(int bus, const char *path_accel, spi_dev_e device, enum Rotation rotation);
	virtual ~BMI055_accel();

	virtual int     init();

	virtual ssize_t     read(struct file *filp, char *buffer, size_t buflen);
	virtual int     ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void            print_info();

	void            print_registers();

	// deliberately cause a sensor error
	void            test_error();

protected:
	virtual int     probe();


private:

	ringbuffer::RingBuffer  *_accel_reports;

	struct accel_calibration_s  _accel_scale;
	float           _accel_range_scale;
	float           _accel_range_m_s2;
	orb_advert_t        _accel_topic;
	int         _accel_orb_class_instance;
	int         _accel_class_instance;



	float       _accel_sample_rate;
	perf_counter_t      _accel_reads;

	math::LowPassFilter2p   _accel_filter_x;
	math::LowPassFilter2p   _accel_filter_y;
	math::LowPassFilter2p   _accel_filter_z;

	Integrator      _accel_int;


	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset
#define BMI055_ACCEL_NUM_CHECKED_REGISTERS 5
	static const uint8_t    _checked_registers[BMI055_ACCEL_NUM_CHECKED_REGISTERS];
	uint8_t         _checked_values[BMI055_ACCEL_NUM_CHECKED_REGISTERS];
	uint8_t         _checked_bad[BMI055_ACCEL_NUM_CHECKED_REGISTERS];


	// last temperature reading for print_info()
	float           _last_temperature;

	bool            _got_duplicate;

	/**
	* Start automatic measurement.
	*/
	void            start();

	/**
	* Stop automatic measurement.
	*/
	void            stop();

	/**
	* Reset chip.
	*
	* Resets the chip and measurements ranges, but not scale and offset.
	*/
	int         reset();

	/**
	* Static trampoline from the hrt_call context; because we don't have a
	* generic hrt wrapper yet.
	*
	* Called by the HRT in interrupt context at the specified rate if
	* automatic polling is enabled.
	*
	* @param arg       Instance pointer for the driver that is polling.
	*/
	static void     measure_trampoline(void *arg);

	/**
	* Fetch measurements from the sensor and update the report buffers.
	*/
	void            measure();


	/**
	* Modify a register in the BMI055_accel
	*
	* Bits are cleared before bits are set.
	*
	* @param reg       The register to modify.
	* @param clearbits Bits in the register to clear.
	* @param setbits   Bits in the register to set.
	*/
	void            modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	* Write a register in the BMI055_accel, updating _checked_values
	*
	* @param reg       The register to write.
	* @param value     The new value to write.
	*/
	void            write_checked_reg(unsigned reg, uint8_t value);

	/**
	* Set the BMI055_accel measurement range.
	*
	* @param max_g     The maximum G value the range must support.
	* @return      OK if the value can be supported, -EINVAL otherwise.
	*/
	int         set_accel_range(unsigned max_g);


	/**
	* Get the internal / external state
	*
	* @return true if the sensor is not on the main MCU board
	*/
	bool            is_external() { return (_bus == EXTERNAL_BUS); }

	/**
	* Measurement self test
	*
	* @return 0 on success, 1 on failure
	*/
	int             self_test();

	/**
	* Accel self test
	*
	* @return 0 on success, 1 on failure
	*/
	int             accel_self_test();

	/*
	set accel sample rate
	*/
	int accel_set_sample_rate(float desired_sample_rate_hz);

	/*
	check that key registers still have the right value
	*/
	void check_registers(void);

	/* do not allow to copy this class due to pointer data members */
	BMI055_accel(const BMI055_accel &);
	BMI055_accel operator=(const BMI055_accel &);

};



class BMI055_gyro : public BMI055
{
public:
	BMI055_gyro(int bus, const char *path_gyro, spi_dev_e device, enum Rotation rotation);
	virtual ~BMI055_gyro();

	virtual int     init();

	virtual ssize_t     read(struct file *filp, char *buffer, size_t buflen);
	virtual int     ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void            print_info();

	void            print_registers();

	// deliberately cause a sensor error
	void            test_error();

protected:
	virtual int     probe();
private:

	ringbuffer::RingBuffer  *_gyro_reports;

	struct gyro_calibration_s   _gyro_scale;
	float           _gyro_range_scale;
	float           _gyro_range_rad_s;

	orb_advert_t        _gyro_topic;
	int         _gyro_orb_class_instance;
	int         _gyro_class_instance;



	float       _gyro_sample_rate;
	perf_counter_t      _gyro_reads;

	math::LowPassFilter2p   _gyro_filter_x;
	math::LowPassFilter2p   _gyro_filter_y;
	math::LowPassFilter2p   _gyro_filter_z;

	Integrator      _gyro_int;


	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset
#define BMI055_GYRO_NUM_CHECKED_REGISTERS 7
	static const uint8_t    _checked_registers[BMI055_GYRO_NUM_CHECKED_REGISTERS];
	uint8_t         _checked_values[BMI055_GYRO_NUM_CHECKED_REGISTERS];
	uint8_t         _checked_bad[BMI055_GYRO_NUM_CHECKED_REGISTERS];


	// last temperature reading for print_info()
	float           _last_temperature;


	/**
	 * Start automatic measurement.
	 */
	void            start();

	/**
	 * Stop automatic measurement.
	 */
	void            stop();

	/**
	 * Reset chip.
	 *
	 * Resets the chip and measurements ranges, but not scale and offset.
	 */
	int         reset();

	/**
	 * Static trampoline from the hrt_call context; because we don't have a
	 * generic hrt wrapper yet.
	 *
	 * Called by the HRT in interrupt context at the specified rate if
	 * automatic polling is enabled.
	 *
	 * @param arg       Instance pointer for the driver that is polling.
	 */
	static void     measure_trampoline(void *arg);

	/**
	 * Fetch measurements from the sensor and update the report buffers.
	 */
	void            measure();


	/**
	 * Modify a register in the BMI055_gyro
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg       The register to modify.
	 * @param clearbits Bits in the register to clear.
	 * @param setbits   Bits in the register to set.
	 */
	void            modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Write a register in the BMI055_gyro, updating _checked_values
	 *
	 * @param reg       The register to write.
	 * @param value     The new value to write.
	 */
	void            write_checked_reg(unsigned reg, uint8_t value);

	/**
	 * Set the BMI055_gyro measurement range.
	 *
	 * @param max_dps   The maximum DPS value the range must support.
	 * @return      OK if the value can be supported, -EINVAL otherwise.
	 */
	int         set_gyro_range(unsigned max_dps);


	/**
	 * Get the internal / external state
	 *
	 * @return true if the sensor is not on the main MCU board
	 */
	bool            is_external() { return (_bus == EXTERNAL_BUS); }

	/**
	 * Measurement self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int             self_test();


	/**
	 * Gyro self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int             gyro_self_test();


	/*
	 * set gyro sample rate
	 */
	int gyro_set_sample_rate(float desired_sample_rate_hz);

	/*
	 * check that key registers still have the right value
	 */
	void check_registers(void);

	/* do not allow to copy this class due to pointer data members */
	BMI055_gyro(const BMI055_gyro &);
	BMI055_gyro operator=(const BMI055_gyro &);

#pragma pack(push, 1)
	/**
	 * Report conversation within the BMI055_gyro, including command byte and
	 * interrupt status.
	 */
	struct BMI_GyroReport {
		uint8_t     cmd;
		int16_t     gyro_x;
		int16_t     gyro_y;
		int16_t     gyro_z;
	};

#pragma pack(pop)
};



#endif /* BMI055_HPP_ */
