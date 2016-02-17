/****************************************************************************
 *
 *   Copyright (c) 2015 Mark Charlebois. All rights reserved.
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
 * @file px4_i2c.h
 *
 * Includes device headers depending on the build target
 */

#pragma once

#define PX4_I2C_M_READ           0x0001          /* read data, from slave to master */

#if defined(__PX4_ROS)

#error "Devices not supported in ROS"

#elif defined (__PX4_NUTTX)
/*
 * Building for NuttX
 */
#include <sys/ioctl.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <nuttx/i2c.h>
#include <nuttx/irq.h>
#include <nuttx/wqueue.h>
#include <chip.h>
#include <arch/board/board.h>
#include <arch/chip/chip.h>
#include "up_internal.h"
#include "up_arch.h"

#define px4_i2c_msg_t i2c_msg_s

typedef struct i2c_dev_s px4_i2c_dev_t;

#elif defined(__PX4_POSIX)
#include <stdint.h>

#define I2C_M_READ           0x0001          /* read data, from slave to master */
#define I2C_M_TEN            0x0002          /* ten bit address */
#define I2C_M_NORESTART      0x0080          /* message should not begin with (re-)start of transfer */

// NOTE - This is a copy of the NuttX i2c_msg_s structure
typedef struct {
	uint16_t  addr;                  /* Slave address */
	uint16_t  flags;                 /* See I2C_M_* definitions */
	uint8_t  *buffer;
	int       length;
} px4_i2c_msg_t;

// NOTE - This is a copy of the NuttX i2c_ops_s structure
typedef struct {
	const struct px4_i2c_ops_t *ops; /* I2C vtable */
} px4_i2c_dev_t;

// FIXME - Empty defines for I2C ops
// Original version commented out
//#define I2C_SETFREQUENCY(d,f) ((d)->ops->setfrequency(d,f))
#define I2C_SETFREQUENCY(d,f)
//#define SPI_SELECT(d,id,s) ((d)->ops->select(d,id,s))
#define SPI_SELECT(d,id,s)

// FIXME - Stub implementation
// Original version commented out
//#define I2C_TRANSFER(d,m,c) ((d)->ops->transfer(d,m,c))
inline int I2C_TRANSFER(px4_i2c_dev_t *dev, px4_i2c_msg_t *msg, int count);
inline int I2C_TRANSFER(px4_i2c_dev_t *dev, px4_i2c_msg_t *msg, int count) { return 0; }

#ifdef __PX4_QURT

struct i2c_msg {
	uint16_t  addr;                  /* Slave address */
	uint16_t  flags;                 /* See I2C_M_* definitions */
	uint8_t  *buf;
	int       len;
};

#define I2C_RDWR 0x0FFF

struct i2c_rdwr_ioctl_data {
	struct i2c_msg *msgs;   /* pointers to i2c_msgs */
	uint32_t nmsgs;         /* number of i2c_msgs */
};

// FIXME - The functions are not implemented on QuRT/DSPAL
int ioctl(int fd, int flags, unsigned long data);
int write(int fd, const char *buffer, int buflen);
#endif
#else
#error "No target platform defined"
#endif
