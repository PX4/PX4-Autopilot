#ifndef BMI160_HPP_
#define BMI160_HPP_

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

#include <perf/perf_counter.h>
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
#include <px4_work_queue/ScheduledWorkItem.hpp>

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

#define BMI160_ACCEL_DEFAULT_RANGE_G		16
#define BMI160_GYRO_DEFAULT_RANGE_DPS		2000
#define BMI160_ACCEL_DEFAULT_RATE           800
#define BMI160_ACCEL_MAX_RATE               1600
#define BMI160_ACCEL_MAX_PUBLISH_RATE       280
#define BMI160_GYRO_DEFAULT_RATE            800
#define BMI160_GYRO_MAX_RATE                3200
#define BMI160_GYRO_MAX_PUBLISH_RATE        BMI160_ACCEL_MAX_PUBLISH_RATE

#define BMI160_ACCEL_DEFAULT_ONCHIP_FILTER_FREQ	324
#define BMI160_ACCEL_DEFAULT_DRIVER_FILTER_FREQ	50

#define BMI160_GYRO_DEFAULT_ONCHIP_FILTER_FREQ	254.6f
#define BMI160_GYRO_DEFAULT_DRIVER_FILTER_FREQ	50

#define BMI160_BUS_SPEED				10*1000*1000

#define BMI160_TIMER_REDUCTION				200

class BMI160_gyro;

class BMI160 : public device::SPI, public px4::ScheduledWorkItem
{
public:
	BMI160(int bus, const char *path_accel, const char *path_gyro, uint32_t device, enum Rotation rotation);
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

	unsigned		_call_interval;

	ringbuffer::RingBuffer	*_accel_reports;

	struct accel_calibration_s	_accel_scale;
	float			_accel_range_scale;
	float			_accel_range_m_s2;
	orb_advert_t		_accel_topic;
	int			_accel_orb_class_instance;
	int			_accel_class_instance;

	ringbuffer::RingBuffer	*_gyro_reports;

	struct gyro_calibration_s	_gyro_scale;
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

	void		Run() override;

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
	uint8_t			read_reg(unsigned reg);
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
	 * Measurement self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			self_test();

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



#endif /* BMI160_HPP_ */
