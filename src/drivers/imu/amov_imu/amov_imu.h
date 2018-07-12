/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 * Shared defines for the AMOV_IMU driver.
 *
 *
 * @author Jin Wu (Zarathustra)
 * @author Shuangqi Mo (Qi Xiao)
 * @author Gen Wang (Better)
 * @email: jin_wu_uestc@hotmail.com
 * @web: https://github.com/zarathustr
 */

#pragma once

#include <perf/perf_counter.h>
#include <drivers/drv_hrt.h>

#define DRV_ACC_DEVTYPE_AMOV_IMU	0x63
#define DRV_GYR_DEVTYPE_AMOV_IMU	0x64
#define DRV_MAG_DEVTYPE_AMOV_IMU	0x65
#define DRV_BARO_DEVTYPE_AMOV_IMU	0x66

#define MAX_DEV_NUM   3

#define DIR_READ			0x80
#define DIR_WRITE			0x00

#define AMOV_IMU_DEVICE_PATH_ACCEL1		"/dev/amov_imu_accel1"
#define AMOV_IMU_DEVICE_PATH_GYRO1		"/dev/amov_imu_gyro1"
#define AMOV_IMU_DEVICE_PATH_MAG1		"/dev/amov_imu_mag1"
#define AMOV_IMU_DEVICE_PATH_ACCEL2		"/dev/amov_imu_accel2"
#define AMOV_IMU_DEVICE_PATH_GYRO2		"/dev/amov_imu_gyro2"
#define AMOV_IMU_DEVICE_PATH_MAG2		"/dev/amov_imu_mag2"
#define AMOV_IMU_DEVICE_PATH_ACCEL3		"/dev/amov_imu_accel3"
#define AMOV_IMU_DEVICE_PATH_GYRO3		"/dev/amov_imu_gyro3"
#define AMOV_IMU_DEVICE_PATH_MAG3		"/dev/amov_imu_mag3"

#define AMOV_IMU_MODEL_IMU1_HEAD        0xA5
#define AMOV_IMU_MODEL_IMU2_HEAD        0xA6
#define AMOV_IMU_MODEL_IMU3_HEAD        0xA7
#define AMOV_IMU_MODEL_AHRS1_HEAD       0xA8
#define AMOV_IMU_MODEL_AHRS2_HEAD       0xA9
#define AMOV_IMU_MODEL_AHRS3_HEAD       0xA9
#define AMOV_IMU_MODEL_INSGPS1_HEAD     0xAA
#define AMOV_IMU_MODEL_INSGPS2_HEAD     0xAB
#define AMOV_IMU_MODEL_INSGPS3_HEAD     0xAC
#define AMOV_IMU_MODEL_INSGPS4_HEAD     0xAD

#define AMOV_IMU_MODEL_NUM              10

#define AMOV_IMU_MODEL_IMU1_LEN         32
#define AMOV_IMU_MODEL_IMU2_LEN         44
#define AMOV_IMU_MODEL_IMU3_LEN         48
#define AMOV_IMU_MODEL_AHRS1_LEN        48
#define AMOV_IMU_MODEL_AHRS2_LEN        60
#define AMOV_IMU_MODEL_AHRS3_LEN        60
#define AMOV_IMU_MODEL_INSGPS1_LEN      80
#define AMOV_IMU_MODEL_INSGPS2_LEN      120
#define AMOV_IMU_MODEL_INSGPS3_LEN      120
#define AMOV_IMU_MODEL_INSGPS4_LEN      160

// AMOV IMU registers
#define AMOV_IMU_REG_WHOAMI             0x27
#define AMOV_IMU_REG_REG1               0x28
#define AMOV_IMU_REG_REG2               0x29
#define AMOV_IMU_REG_DATA               0x2A
#define AMOV_IMU_REG_RESET              0x2B

#define AMOV_IMU_WHOAMI                         0x72

#define AMOV_IMU_ACCEL_DEFAULT_RANGE_G				16
#define AMOV_IMU_ACCEL_DEFAULT_RATE				1000
#define AMOV_IMU_ACCEL_MAX_OUTPUT_RATE				280
#define AMOV_IMU_ACCEL_DEFAULT_DRIVER_FILTER_FREQ               30

#define AMOV_IMU_GYRO_DEFAULT_RANGE_G				8
#define AMOV_IMU_GYRO_DEFAULT_RATE				1000
/* rates need to be the same between accel and gyro */
#define AMOV_IMU_GYRO_MAX_OUTPUT_RATE				AMOV_IMU_ACCEL_MAX_OUTPUT_RATE
#define AMOV_IMU_GYRO_DEFAULT_DRIVER_FILTER_FREQ		30

#define AMOV_IMU_DEFAULT_ONCHIP_FILTER_FREQ			42

#define AMOV_IMU_ONE_G                                          9.80665f

#ifdef PX4_SPI_BUS_EXT
#define EXTERNAL_BUS PX4_SPI_BUS_EXT
#else
#define EXTERNAL_BUS 0
#endif

#define AMOV_IMU_IOCGIS_I2C         (unsigned)(DEVIOCGDEVICEID + 100)

#pragma pack(push, 1)

union fnum {
	float f;
	uint8_t bytes[4];
};

union dnum {
	float d;
	uint8_t bytes[8];
};

struct AMOVReport {
	fnum		gyro_x;
	fnum		gyro_y;
	fnum		gyro_z;
	fnum		accel_x;
	fnum		accel_y;
	fnum		accel_z;
	int16_t		temp;
	fnum        mag_x;
	fnum        mag_y;
	fnum        mag_z;
//    fnum        q0;
//    fnum        q1;
//    fnum        q2;
//    fnum        q3;
//    fnum        baro_alt;
//    fnum        vel_ned_x;
//    fnum        vel_ned_y;
//    fnum        vel_ned_z;
//    dnum        pos_lat;
//    dnum        pos_lon;
//    dnum        pos_alt;
//    uint8_t     sat_num;
};

struct IMU3DMPacket {
	uint8_t buf[256];
};

#pragma pack(pop)

const uint32_t CRC32_Table[256] = {
	0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA,
	0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3,
	0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988,
	0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91,
	0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE,
	0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
	0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC,
	0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5,
	0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172,
	0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B,
	0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940,
	0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
	0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116,
	0x21B4F4B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F,
	0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924,
	0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D,
	0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A,
	0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
	0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818,
	0x7F6A0DBB, 0x086D3D2D, 0x91646C97, 0xE6635C01,
	0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E,
	0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457,
	0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C,
	0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
	0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2,
	0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB,
	0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0,
	0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9,
	0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086,
	0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
	0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4,
	0x59B33D17, 0x2EB40D81, 0xB7BD5C3B, 0xC0BA6CAD,
	0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A,
	0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683,
	0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8,
	0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
	0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE,
	0xF762575D, 0x806567CB, 0x196C3671, 0x6E6B06E7,
	0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC,
	0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5,
	0xD6D6A3E8, 0xA1D1937E, 0x38D8C2C4, 0x4FDFF252,
	0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
	0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60,
	0xDF60EFC3, 0xA867DF55, 0x316E8EEF, 0x4669BE79,
	0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236,
	0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F,
	0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB36A04,
	0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
	0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A,
	0x9C0906A9, 0xEB0E363F, 0x72076785, 0x05005713,
	0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38,
	0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21,
	0x86D3D2D4, 0xF1D4E242, 0x68DDB3F8, 0x1FDA836E,
	0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
	0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C,
	0x8F659EFF, 0xF862AE69, 0x616BFFD3, 0x166CCF45,
	0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2,
	0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB,
	0xAED16A4A, 0xD9D65ADC, 0x40DF0B66, 0x37D83BF0,
	0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
	0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6,
	0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF,
	0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94,
	0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D
};

inline uint32_t calculate_CRC32(void *pStart, uint32_t uSize)
{
#define INIT  0xffffffff
#define XOROT 0xffffffff

	uint32_t uCRCValue;
	uint8_t *pData;

	/* init the start value */
	uCRCValue = INIT;
	pData = (uint8_t *)pStart;

	/* calculate CRC */
	while (uSize --) {
		uCRCValue = CRC32_Table[(uCRCValue ^ *pData++) & 0xFF] ^ (uCRCValue >> 8);
	}

	/* XOR the output value */
	return uCRCValue ^ XOROT;
}

#define AMOV_IMU_MAX_READ_BUFFER_SIZE (sizeof(AMOVReport) + 1)
#define AMOV_IMU_MAX_WRITE_BUFFER_SIZE (2)

#define AMOV_IMU_LOW_BUS_SPEED				0
#define AMOV_IMU_HIGH_BUS_SPEED				0x8000
#  define AMOV_IMU_IS_HIGH_SPEED(r) 			((r) & AMOV_IMU_HIGH_BUS_SPEED)
#  define AMOV_IMU_REG(r) 				((r) &~AMOV_IMU_HIGH_BUS_SPEED)
#  define AMOV_IMU_SET_SPEED(r, s) 			((r)|(s))
#  define AMOV_IMU_HIGH_SPEED_OP(r) 			AMOV_IMU_SET_SPEED((r), AMOV_IMU_HIGH_BUS_SPEED)
#  define AMOV_IMU_LOW_SPEED_OP(r)			AMOV_IMU_REG((r))

/* interface factories */
extern device::Device *AMOV_IMU_SPI_interface(int bus, int device_type, bool external_bus);
extern device::Device *AMOV_IMU_UART_interface(const char *uart_name, const char *uart_alias, bool is_dma);
extern int AMOV_IMU_probe(device::Device *dev, int device_type);
