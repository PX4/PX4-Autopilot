/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 * @file drv_sensor.h
 *
 * Common sensor API and ioctl definitions.
 */

#ifndef _DRV_SENSOR_H
#define _DRV_SENSOR_H

#include <px4_defines.h>
#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_device.h"

/**
 * Sensor type definitions.
 *
 * Used to create a unique device id for redundant and combo sensors
 */

#define DRV_MAG_DEVTYPE_HMC5883  0x01
#define DRV_MAG_DEVTYPE_LSM303D  0x02
#define DRV_MAG_DEVTYPE_MAGSIM   0x03
#define DRV_MAG_DEVTYPE_MPU9250  0x04
#define DRV_MAG_DEVTYPE_LIS3MDL  0x05
#define DRV_MAG_DEVTYPE_IST8310  0x06
#define DRV_MAG_DEVTYPE_RM3100   0x07
#define DRV_MAG_DEVTYPE_QMC5883  0x08
#define DRV_MAG_DEVTYPE_AK09916  0x09
#define DRV_DEVTYPE_ICM20948     0x0A
#define DRV_ACC_DEVTYPE_LSM303D  0x11
#define DRV_ACC_DEVTYPE_BMA180   0x12
#define DRV_ACC_DEVTYPE_MPU6000  0x13
#define DRV_ACC_DEVTYPE_ACCELSIM 0x14
#define DRV_ACC_DEVTYPE_MPU9250  0x16
#define DRV_ACC_DEVTYPE_BMI160   0x17
#define DRV_GYR_DEVTYPE_MPU6000  0x21
#define DRV_GYR_DEVTYPE_L3GD20   0x22
#define DRV_GYR_DEVTYPE_GYROSIM  0x23
#define DRV_GYR_DEVTYPE_MPU9250  0x24
#define DRV_GYR_DEVTYPE_BMI160   0x25
#define DRV_RNG_DEVTYPE_MB12XX   0x31
#define DRV_RNG_DEVTYPE_LL40LS   0x32
#define DRV_ACC_DEVTYPE_MPU6050  0x33
#define DRV_ACC_DEVTYPE_MPU6500  0x34
#define DRV_GYR_DEVTYPE_MPU6050  0x35
#define DRV_GYR_DEVTYPE_MPU6500  0x36
#define DRV_ACC_DEVTYPE_ICM20602	0x37
#define DRV_GYR_DEVTYPE_ICM20602	0x38
#define DRV_ACC_DEVTYPE_ICM20608	0x39
#define DRV_GYR_DEVTYPE_ICM20608	0x3A
#define DRV_ACC_DEVTYPE_ICM20689	0x3B
#define DRV_GYR_DEVTYPE_ICM20689	0x3C
#define DRV_BARO_DEVTYPE_MS5611		0x3D
#define DRV_BARO_DEVTYPE_MS5607		0x3E
#define DRV_BARO_DEVTYPE_BMP280		0x3F
#define DRV_BARO_DEVTYPE_LPS25H		0x40
#define DRV_ACC_DEVTYPE_BMI055		0x41
#define DRV_GYR_DEVTYPE_BMI055		0x42
#define DRV_MAG_DEVTYPE_BMM150		0x43
#define DRV_BARO_DEVTYPE_BMP285		0x44
#define DRV_DIFF_PRESS_DEVTYPE_ETS3	0x45
#define DRV_DIFF_PRESS_DEVTYPE_MS4525	0x46
#define DRV_DIFF_PRESS_DEVTYPE_MS5525	0x47
#define DRV_DIFF_PRESS_DEVTYPE_SDP31	0x48
#define DRV_DIFF_PRESS_DEVTYPE_SDP32	0x49
#define DRV_DIFF_PRESS_DEVTYPE_SDP33	0x50
#define DRV_BARO_DEVTYPE_MPL3115A2	0x51
#define DRV_ACC_DEVTYPE_FXOS8701C	0x52

#define DRV_GYR_DEVTYPE_FXAS2100C	0x54
#define DRV_ACC_DEVTYPE_ADIS16448	0x55
#define DRV_MAG_DEVTYPE_ADIS16448	0x56
#define DRV_GYR_DEVTYPE_ADIS16448	0x57
#define DRV_BARO_DEVTYPE_LPS22HB	0x58
#define DRV_ACC_DEVTYPE_ADIS16477	0x59
#define DRV_GYR_DEVTYPE_ADIS16477	0x60
#define DRV_ACC_DEVTYPE_LSM303AGR	0x61
#define DRV_MAG_DEVTYPE_LSM303AGR	0x62
#define DRV_ACC_DEVTYPE_ADIS16497	0x63
#define DRV_GYR_DEVTYPE_ADIS16497	0x64
#define DRV_BARO_DEVTYPE_BAROSIM	0x65
#define DRV_DEVTYPE_BMI088		0x66
#define DRV_DEVTYPE_BMP388		0x67

/*
 * ioctl() definitions
 *
 * Note that a driver may not implement all of these operations, but
 * if the operation is implemented it should conform to this API.
 */

#define _SENSORIOCBASE		(0x2000)
#define _SENSORIOC(_n)		(_PX4_IOC(_SENSORIOCBASE, _n))

/**
 * Set the driver polling rate to (arg) Hz, or one of the SENSOR_POLLRATE
 * constants
 */
#define SENSORIOCSPOLLRATE	_SENSORIOC(0)

#define SENSOR_POLLRATE_DEFAULT		1000003	/**< poll at driver normal rate */

/**
 * Reset the sensor to its default configuration
 */
#define SENSORIOCRESET		_SENSORIOC(4)

#endif /* _DRV_SENSOR_H */
