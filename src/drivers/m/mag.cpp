#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/spi.h>
#include <drivers/drv_accel.h>
//#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>

#include "mag.h"
#include "mpu9250.h"


/* mpu9250 master i2c bus specific register address and bit definitions */

#define MPUREG_I2C_MST_STATUS		0x36


#define BIT_I2C_READ_FLAG           0x80

#define MPUREG_I2C_MST_CTRL         0x24
#define MPUREG_I2C_SLV0_ADDR        0x25
#define MPUREG_I2C_SLV0_REG         0x26
#define MPUREG_I2C_SLV0_CTRL        0x27

#define MPUREG_I2C_SLV4_ADDR        0x31
#define MPUREG_I2C_SLV4_REG         0x32
#define MPUREG_I2C_SLV4_DO          0x33
#define MPUREG_I2C_SLV4_CTRL        0x34
#define MPUREG_I2C_SLV4_DI          0x35

#define MPUREG_EXT_SENS_DATA_00     0x49

#define MPUREG_I2C_SLV0_D0          0x63
#define MPUREG_I2C_MST_DELAY_CTRL   0x67
#define MPUREG_USER_CTRL            0x6A

#define BIT_I2C_SLV0_NACK           0x01
#define BIT_I2C_FIFO_EN             0x40
#define BIT_I2C_MST_EN              0x20
#define BIT_I2C_IF_DIS              0x10
#define BIT_FIFO_RST                0x04
#define BIT_I2C_MST_RST             0x02
#define BIT_SIG_COND_RST            0x01

#define BIT_I2C_SLV0_EN             0x80
#define BIT_I2C_SLV0_BYTE_SW        0x40
#define BIT_I2C_SLV0_REG_DIS        0x20
#define BIT_I2C_SLV0_REG_GRP        0x10

#define BIT_I2C_MST_MULT_MST_EN     0x80
#define BIT_I2C_MST_WAIT_FOR_ES     0x40
#define BIT_I2C_MST_SLV_3_FIFO_EN   0x20
#define BIT_I2C_MST_P_NSR           0x10
#define BITS_I2C_MST_CLOCK_258HZ    0x08
#define BITS_I2C_MST_CLOCK_400HZ    0x0D

#define BIT_I2C_SLV0_DLY_EN         0x01
#define BIT_I2C_SLV1_DLY_EN         0x02
#define BIT_I2C_SLV2_DLY_EN         0x04
#define BIT_I2C_SLV3_DLY_EN         0x08


/* ak8963 register address and bit definitions */

#define AK8963_I2C_ADDR         0x0C
#define AK8963_DEVICE_ID        0x48

#define AK8963REG_WIA           0x00
#define AK8963REG_ST1           0x02
#define AK8963REG_HXL           0x03
#define AK8963REG_ASAX          0x10
#define AK8963REG_CNTL1         0x0A
#define AK8963REG_CNTL2         0x0B

#define AK8963_SINGLE_MEAS_MODE 0x01
#define AK8963_CONTINUOUS_MODE1 0x02
#define AK8963_CONTINUOUS_MODE2 0x06
#define AK8963_POWERDOWN_MODE   0x00
#define AK8963_SELFTEST_MODE    0x08
#define AK8963_FUZE_MODE        0x0F
#define AK8963_16BIT_ADC        0x10
#define AK8963_14BIT_ADC        0x00
#define AK8963_RESET            0x01


MPU9250_mag::MPU9250_mag(MPU9250 *parent, const char *path) :
	CDev("MPU9250_mag", path),
	_parent(parent),
	_mag_reading_data(false),
	_last_x(0),
	_last_y(0),
	_last_z(0),
	_rru(0),
	dxh(0),
	dxl(0),
	dyh(0),
	dyl(0),
	dzh(0),
	dzl(0)
{
}

MPU9250_mag::~MPU9250_mag()
{
}

int
MPU9250_mag::init()
{
	int ret;

	ret = CDev::init();

	/* if setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("MPU9250 mag init failed");
		return ret;
	}

printf("sizeof(struct ak8963_regs) = %d\n", sizeof(struct ak8963_regs));

	ak8963_setup();

	return ret;
}

int32_t period = 0;

int16_t x_raw = 0;
int16_t y_raw = 0;
int16_t z_raw = 0;

uint8_t cnt0 = 0;
uint8_t cnt1 = 0;
uint8_t cnt2 = 0;
uint8_t cnt3 = 0;
uint8_t cnt4 = 0;
/*
	struct ak8963_regs {
		uint8_t id;
		uint8_t info;
		uint8_t st1;
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t st2;
	};
 */
	uint64_t		_ts;
	uint64_t		_ts_last;

int glitched = 0;

bool
MPU9250_mag::measure(struct ak8963_regs data)
{
	/* monitor for if data overrun flag is ever set */
	if (data.st1 & 0x02) {
//		printf("overrun\n");
		cnt4++;
	}
	/* monitor for if magnetic sensor overflow flag is ever set noting that st2
	 * is usually not even refreshed, but will always be in the same place in the
	 * mpu's buffers regardless, hence the actual count would be bogus
	 */
	if (data.st2 & 0x08) {
		printf("overflow\n");
//		cnt4++;
	}

	uint8_t err = _parent->read_reg(MPUREG_I2C_MST_STATUS);
	if (err & BIT_I2C_SLV0_NACK) {
		printf("i2c nack\n");
		cnt4++;
	}

	if (data.st1 & 0x01) {
		if (false == _mag_reading_data) {
			cnt1++;
			_parent->write_reg(MPUREG_I2C_SLV0_CTRL, BIT_I2C_SLV0_EN | sizeof(struct ak8963_regs));
			_mag_reading_data = true;
			return false;

		} else {
			cnt2++;
			_parent->write_reg(MPUREG_I2C_SLV0_CTRL, BIT_I2C_SLV0_EN | 1);
			_mag_reading_data = false;
//			if (0 == _rru) {
//				return false;  // this prevents the rapid repeat update
//			}
		}

	} else {
		if (true == _mag_reading_data) {
			cnt3++;
			_parent->write_reg(MPUREG_I2C_SLV0_CTRL, BIT_I2C_SLV0_EN | 1);
			_mag_reading_data = false;
		} else {
//			cnt4++;
			_rru++;
		}

		return false;
	}

//	cnt4++;

	_ts = hrt_absolute_time();
	period = _ts - _ts_last;
	_ts_last = _ts;

	x_raw = data.x;
	y_raw = data.y;
	z_raw = data.z;

	_rru = 0;

#define DELTA 200

	if (data.x > (_last_x + DELTA)) {
		glitched = 1;
		printf("glitch +x\n");
		if (dxh++ < 3) {
			data.x = data.x - 256;
		}
	} else {
		dxh = 0;
	}
	if (data.x < (_last_x - DELTA)) {
		glitched = 2;
		printf("glitch -x\n");
		if (dxl++ < 3) {
			data.x = data.x + 256;
		}
	} else {
		dxl = 0;
	}

	if (data.y > (_last_y + DELTA)) {
		glitched = 3;
		printf("glitch +y\n");
		if (dyh++ < 3) {
			data.y = data.y - 256;
		}
	} else {
		dyh = 0;
	}
	if (data.y < (_last_y - DELTA)) {
		glitched = 4;
		printf("glitch -y\n");
		if (dyl++ < 3) {
			data.y = data.y + 256;
		}
	} else {
		dyl = 0;
	}

	if (data.z > (_last_z + DELTA)) {
		glitched = 5;
		printf("glitch +z\n");
		if (dzh++ < 3) {
			data.z = data.z - 256;
		}
	} else {
		dzh = 0;
	}
	if (data.z < (_last_z - DELTA)) {
		glitched = 6;
		printf("glitch -z\n");
		if (dzl++ < 3) {
			data.z = data.z + 256;
		}
	} else {
		dzl = 0;
	}

	_last_x = data.x;
	_last_y = data.y;
	_last_z = data.z;

	if ((period < 8000) || (period > 12000)) {
		glitched = 7;
		printf("glitch dt %u\n", period);
	}

	return true;
}

void
MPU9250_mag::set_passthrough(uint8_t reg, uint8_t size, uint8_t *out)
{
	uint8_t addr;

	_parent->write_reg(MPUREG_I2C_SLV0_CTRL, 0); // ensure slave r/w is disabled before changing the registers

	if (out) {
		_parent->write_reg(MPUREG_I2C_SLV0_D0, *out);
		addr = AK8963_I2C_ADDR;

	} else {
		addr = AK8963_I2C_ADDR | BIT_I2C_READ_FLAG;
	}

	_parent->write_reg(MPUREG_I2C_SLV0_ADDR, addr);
	_parent->write_reg(MPUREG_I2C_SLV0_REG,  reg);
	_parent->write_reg(MPUREG_I2C_SLV0_CTRL, size | BIT_I2C_SLV0_EN);
}

void
MPU9250_mag::read_block(uint8_t reg, uint8_t *val, uint8_t count)
{
	uint8_t addr = reg | 0x80;
	uint8_t tx[32] = { addr, };
	uint8_t rx[32];

	_parent->transfer(tx, rx, count + 1);
	memcpy(val, rx + 1, count);
}

void
MPU9250_mag::passthrough_read(uint8_t reg, uint8_t *buf, uint8_t size)
{
	set_passthrough(reg, size);
	usleep(25 + 25 * size); // wait for the value to be read from slave
	read_block(MPUREG_EXT_SENS_DATA_00, buf, size);
	_parent->write_reg(MPUREG_I2C_SLV0_CTRL, 0); // disable new reads
}

bool
MPU9250_mag::ak8963_check_id(void)
{
	for (int i = 0; i < 5; i++) {
		uint8_t deviceid = 0;
		passthrough_read(AK8963REG_WIA, &deviceid, 0x01);

		if (AK8963_DEVICE_ID == deviceid) {
			return true;
		}
	}

	return false;
}
/*
 * 400kHz I2C bus speed = 2.5us per bit = 25us per byte
 */
void
MPU9250_mag::passthrough_write(uint8_t reg, uint8_t val)
{
	set_passthrough(reg, 1, &val);
//	usleep(50); // wait for the value to be written to slave
	usleep(250); // wait for the value to be written to slave
	_parent->write_reg(MPUREG_I2C_SLV0_CTRL, 0); // disable new writes
}

void
MPU9250_mag::ak8963_reset(void)
{
	passthrough_write(AK8963REG_CNTL2, AK8963_RESET);
}

bool
MPU9250_mag::ak8963_setup(void)
{
	// enable the I2C master to slaves on the aux bus
	uint8_t user_ctrl = _parent->read_reg(MPUREG_USER_CTRL);
	_parent->write_reg(MPUREG_USER_CTRL, user_ctrl | BIT_I2C_MST_EN);
	_parent->write_reg(MPUREG_I2C_MST_CTRL, BIT_I2C_MST_P_NSR | BITS_I2C_MST_CLOCK_400HZ);
//	_parent->write_reg(MPUREG_I2C_MST_CTRL, BIT_I2C_MST_P_NSR | BIT_I2C_MST_WAIT_FOR_ES | BITS_I2C_MST_CLOCK_258HZ);

//	_parent->write_reg(MPUREG_I2C_MST_CTRL, BIT_I2C_MST_MULT_MST_EN | BITS_I2C_MST_CLOCK_400HZ);

	/* Hard-code divider (9) for internal sample rate, 1 kHz, resulting in a
	 * slave sampling rate of 100Hz
	 */
//	_parent->write_reg(MPUREG_I2C_SLV4_CTRL, 1);
//	_parent->write_reg(MPUREG_I2C_MST_DELAY_CTRL, BIT_I2C_SLV0_DLY_EN);

	if (!ak8963_check_id()) {
		::printf("AK8963: bad id\n");
	}

	usleep(250);
//	passthrough_write(AK8963REG_CNTL1, AK8963_CONTINUOUS_MODE1 | AK8963_16BIT_ADC);
	passthrough_write(AK8963REG_CNTL1, AK8963_CONTINUOUS_MODE2 | AK8963_16BIT_ADC);
//	passthrough_write(AK8963REG_CNTL1, AK8963_SINGLE_MEAS_MODE | AK8963_16BIT_ADC);

	set_passthrough(AK8963REG_ST1, 1);
//	set_passthrough(AK8963REG_ST1, sizeof(struct ak8963_regs));
//	_parent->write_reg(MPUREG_I2C_SLV0_CTRL, BIT_I2C_SLV0_EN | sizeof(struct ak8963_regs));

	return true;
}
