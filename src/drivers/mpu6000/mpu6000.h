/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file .h
 *
 * Shared defines for the mpu6000 driver.
 */

#pragma once

#if defined(PX4_I2C_MPU6050_ADDR) || \
	defined(PX4_I2C_MPU6000_ADDR) || \
	defined(PX4_I2C_ICM_20608_G_ADDR)
#  define USE_I2C
#endif


#define DIR_READ			0x80
#define DIR_WRITE			0x00

#define MPU_DEVICE_PATH_ACCEL		"/dev/mpu6000_accel"
#define MPU_DEVICE_PATH_GYRO		"/dev/mpu6000_gyro"
#define MPU_DEVICE_PATH_ACCEL1		"/dev/mpu6000_accel1"
#define MPU_DEVICE_PATH_GYRO1		"/dev/mpu6000_gyro1"
#define MPU_DEVICE_PATH_ACCEL_EXT	"/dev/mpu6000_accel_ext"
#define MPU_DEVICE_PATH_GYRO_EXT	"/dev/mpu6000_gyro_ext"
#define MPU_DEVICE_PATH_ACCEL_EXT1	"/dev/mpu6000_accel_ext1"
#define MPU_DEVICE_PATH_GYRO_EXT1	"/dev/mpu6000_gyro_ext1"

#define ICM20602_DEVICE_PATH_ACCEL		"/dev/icm20602_accel"
#define ICM20602_DEVICE_PATH_GYRO		"/dev/icm20602_gyro"
#define ICM20602_DEVICE_PATH_ACCEL1		"/dev/icm20602_accel1"
#define ICM20602_DEVICE_PATH_GYRO1		"/dev/icm20602_gyro1"
#define ICM20602_DEVICE_PATH_ACCEL_EXT	"/dev/icm20602_accel_ext"
#define ICM20602_DEVICE_PATH_GYRO_EXT	"/dev/icm20602_gyro_ext"
#define ICM20602_DEVICE_PATH_ACCEL_EXT1	"/dev/icm20602_accel_ext1"
#define ICM20602_DEVICE_PATH_GYRO_EXT1	"/dev/icm20602_gyro_ext1"

#define ICM20608_DEVICE_PATH_ACCEL		"/dev/icm20608_accel"
#define ICM20608_DEVICE_PATH_GYRO		"/dev/icm20608_gyro"
#define ICM20608_DEVICE_PATH_ACCEL1		"/dev/icm20608_accel1"
#define ICM20608_DEVICE_PATH_GYRO1		"/dev/icm20608_gyro1"
#define ICM20608_DEVICE_PATH_ACCEL_EXT	"/dev/icm20608_accel_ext"
#define ICM20608_DEVICE_PATH_GYRO_EXT	"/dev/icm20608_gyro_ext"
#define ICM20608_DEVICE_PATH_ACCEL_EXT1	"/dev/icm20608_accel_ext1"
#define ICM20608_DEVICE_PATH_GYRO_EXT1	"/dev/icm20608_gyro_ext1"

#define ICM20689_DEVICE_PATH_ACCEL		"/dev/icm20689_accel"
#define ICM20689_DEVICE_PATH_GYRO		"/dev/icm20689_gyro"

// MPU 6000 registers
#define MPUREG_WHOAMI			0x75
#define MPUREG_SMPLRT_DIV		0x19
#define MPUREG_CONFIG			0x1A
#define MPUREG_GYRO_CONFIG		0x1B
#define MPUREG_ACCEL_CONFIG		0x1C
#define MPUREG_FIFO_EN			0x23
#define MPUREG_INT_PIN_CFG		0x37
#define MPUREG_INT_ENABLE		0x38
#define MPUREG_INT_STATUS		0x3A
#define MPUREG_ACCEL_XOUT_H		0x3B
#define MPUREG_ACCEL_XOUT_L		0x3C
#define MPUREG_ACCEL_YOUT_H		0x3D
#define MPUREG_ACCEL_YOUT_L		0x3E
#define MPUREG_ACCEL_ZOUT_H		0x3F
#define MPUREG_ACCEL_ZOUT_L		0x40
#define MPUREG_TEMP_OUT_H		0x41
#define MPUREG_TEMP_OUT_L		0x42
#define MPUREG_GYRO_XOUT_H		0x43
#define MPUREG_GYRO_XOUT_L		0x44
#define MPUREG_GYRO_YOUT_H		0x45
#define MPUREG_GYRO_YOUT_L		0x46
#define MPUREG_GYRO_ZOUT_H		0x47
#define MPUREG_GYRO_ZOUT_L		0x48
#define MPUREG_USER_CTRL		0x6A
#define MPUREG_PWR_MGMT_1		0x6B
#define MPUREG_PWR_MGMT_2		0x6C
#define MPUREG_FIFO_COUNTH		0x72
#define MPUREG_FIFO_COUNTL		0x73
#define MPUREG_FIFO_R_W			0x74
#define MPUREG_PRODUCT_ID		0x0C
#define MPUREG_TRIM1			0x0D
#define MPUREG_TRIM2			0x0E
#define MPUREG_TRIM3			0x0F
#define MPUREG_TRIM4			0x10
#define MPU_GYRO_DLPF_CFG_256HZ_NOLPF2	0x00
#define MPU_GYRO_DLPF_CFG_188HZ		0x01
#define MPU_GYRO_DLPF_CFG_98HZ		0x02
#define MPU_GYRO_DLPF_CFG_42HZ		0x03
#define MPU_GYRO_DLPF_CFG_20HZ		0x04
#define MPU_GYRO_DLPF_CFG_10HZ		0x05
#define MPU_GYRO_DLPF_CFG_5HZ		0x06
#define MPU_GYRO_DLPF_CFG_2100HZ_NOLPF	0x07
#define MPU_DLPF_CFG_MASK		0x07

// Configuration bits MPU 3000 and MPU 6000 (not revised)?
#define BIT_SLEEP				0x40
#define BIT_H_RESET				0x80
#define BITS_CLKSEL				0x07
#define MPU_CLK_SEL_PLLGYROX	0x01
#define MPU_CLK_SEL_PLLGYROZ	0x03
#define MPU_EXT_SYNC_GYROX		0x02
#define BITS_GYRO_ST_X			0x80
#define BITS_GYRO_ST_Y			0x40
#define BITS_GYRO_ST_Z			0x20
#define BITS_FS_250DPS			0x00
#define BITS_FS_500DPS			0x08
#define BITS_FS_1000DPS			0x10
#define BITS_FS_2000DPS			0x18
#define BITS_FS_MASK			0x18
#define BIT_INT_ANYRD_2CLEAR	0x10
#define BIT_RAW_RDY_EN			0x01
#define BIT_I2C_IF_DIS			0x10
#define BIT_INT_STATUS_DATA		0x01

#define MPU_WHOAMI_6000			0x68
#define ICM_WHOAMI_20602		0x12
#define ICM_WHOAMI_20608		0xaf
#define ICM_WHOAMI_20689		0x98

// ICM2608 specific registers

#define ICMREG_ACCEL_CONFIG2		0x1D
#define ICM_ACC_DLPF_CFG_1046HZ_NOLPF	0x00
#define ICM_ACC_DLPF_CFG_218HZ		0x01
#define ICM_ACC_DLPF_CFG_99HZ		0x02
#define ICM_ACC_DLPF_CFG_44HZ		0x03
#define ICM_ACC_DLPF_CFG_21HZ		0x04
#define ICM_ACC_DLPF_CFG_10HZ		0x05
#define ICM_ACC_DLPF_CFG_5HZ		0x06
#define ICM_ACC_DLPF_CFG_420HZ		0x07
/* this is an undocumented register which
   if set incorrectly results in getting a 2.7m/s/s offset
   on the Y axis of the accelerometer
*/
#define MPUREG_ICM_UNDOC1		0x11
#define MPUREG_ICM_UNDOC1_VALUE	0xc9

// Product ID Description for ICM2602
// Read From device

#define ICM20602_REV_02		2

// Product ID Description for ICM2608

#define ICM20608_REV_FF		0xff // In the past, was thought to be not returning a value. But seem repeatable.

// Product ID Description for ICM2689

#define ICM20689_REV_FE		0xfe
#define ICM20689_REV_03		0x03

// Product ID Description for MPU6000
// high 4 bits 	low 4 bits
// Product Name	Product Revision
#define MPU6000ES_REV_C4		0x14
#define MPU6000ES_REV_C5		0x15
#define MPU6000ES_REV_D6		0x16
#define MPU6000ES_REV_D7		0x17
#define MPU6000ES_REV_D8		0x18
#define MPU6000_REV_C4			0x54
#define MPU6000_REV_C5			0x55
#define MPU6000_REV_D6			0x56
#define MPU6000_REV_D7			0x57
#define MPU6000_REV_D8			0x58
#define MPU6000_REV_D9			0x59
#define MPU6000_REV_D10			0x5A
#define MPU6050_REV_D8			0x28	// TODO:Need verification

#define MPU6000_ACCEL_DEFAULT_RANGE_G				16
#define MPU6000_ACCEL_DEFAULT_RATE					1000
#define MPU6000_ACCEL_MAX_OUTPUT_RATE				280
#define MPU6000_ACCEL_DEFAULT_DRIVER_FILTER_FREQ	30

#define MPU6000_GYRO_DEFAULT_RANGE_G				8
#define MPU6000_GYRO_DEFAULT_RATE					1000
/* rates need to be the same between accel and gyro */
#define MPU6000_GYRO_MAX_OUTPUT_RATE				MPU6000_ACCEL_MAX_OUTPUT_RATE
#define MPU6000_GYRO_DEFAULT_DRIVER_FILTER_FREQ		30

#define MPU6000_DEFAULT_ONCHIP_FILTER_FREQ			42

#define MPU6000_ONE_G					9.80665f

#ifdef PX4_SPI_BUS_EXT
#define EXTERNAL_BUS PX4_SPI_BUS_EXT
#else
#define EXTERNAL_BUS 0
#endif

#define MPUIOCGIS_I2C	(unsigned)(DEVIOCGDEVICEID+100)

#pragma pack(push, 1)
/**
 * Report conversation within the MPU6000, including command byte and
 * interrupt status.
 */
struct MPUReport {
	uint8_t		cmd;
	uint8_t		status;
	uint8_t		accel_x[2];
	uint8_t		accel_y[2];
	uint8_t		accel_z[2];
	uint8_t		temp[2];
	uint8_t		gyro_x[2];
	uint8_t		gyro_y[2];
	uint8_t		gyro_z[2];
};
#pragma pack(pop)

#define MPU_MAX_READ_BUFFER_SIZE (sizeof(MPUReport) + 1)
#define MPU_MAX_WRITE_BUFFER_SIZE (2)
/*
  The MPU6000 can only handle high bus speeds on the sensor and
  interrupt status registers. All other registers have a maximum 1MHz
  Communication with all registers of the device is performed using either
  I2C at 400kHz or SPI at 1MHz. For applications requiring faster communications,
  the sensor and interrupt registers may be read using SPI at 20MHz
 */
#define MPU6000_LOW_BUS_SPEED				0
#define MPU6000_HIGH_BUS_SPEED				0x8000
#  define MPU6000_IS_HIGH_SPEED(r) 			((r) & MPU6000_HIGH_BUS_SPEED)
#  define MPU6000_REG(r) 					((r) &~MPU6000_HIGH_BUS_SPEED)
#  define MPU6000_SET_SPEED(r, s) 			((r)|(s))
#  define MPU6000_HIGH_SPEED_OP(r) 			MPU6000_SET_SPEED((r), MPU6000_HIGH_BUS_SPEED)
#  define MPU6000_LOW_SPEED_OP(r)			MPU6000_REG((r))

/* interface factories */
extern device::Device *MPU6000_SPI_interface(int bus, int device_type, bool external_bus);
extern device::Device *MPU6000_I2C_interface(int bus, int device_type, bool external_bus);
extern int MPU6000_probe(device::Device *dev, int device_type);

typedef device::Device *(*MPU6000_constructor)(int, int, bool);
