/*
 *   Copyright (C) 2012 Lorenz Meier. All rights reserved.
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
 * 3. Neither the name of the author or the names of contributors may be
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
 */

/*
 * Generic driver for I2C EEPROMs with 8 bit or 16 bit addressing
 */

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include <nuttx/i2c.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include "stm32_internal.h"
#include "px4fmu-internal.h"

#include <arch/board/drv_eeprom.h>

/* Split I2C transfers into smaller chunks to make sure to stay within tight timeout limits */

/* check defines */
#ifndef MAX_EEPROMS
  #error MAX_EEPROMS number must be defined (1-3)
#endif

#if (MAX_EEPROMS > 3)
  #error Currently only a maximum of three EEPROMS is supported, add missing code around here: __FILE__:__LINE__
#endif
static int eeprom_open0(FAR struct file *filp);
static int eeprom_close0(FAR struct file *filp);
static ssize_t eeprom_read0(struct file *filp, FAR char *buffer, size_t buflen);
static ssize_t eeprom_write0(struct file *filp, FAR const char *buffer, size_t buflen);
static off_t eeprom_seek0(FAR struct file *filp, off_t offset, int whence);
#if (MAX_EEPROMS > 1)
static int eeprom_open1(FAR struct file *filp);
static int eeprom_close1(FAR struct file *filp);
static ssize_t eeprom_read1(struct file *filp, FAR char *buffer, size_t buflen);
static ssize_t eeprom_write1(struct file *filp, FAR const char *buffer, size_t buflen);
static off_t eeprom_seek1(FAR struct file *filp, off_t offset, int whence);
#endif
#if (MAX_EEPROMS > 2)
static int eeprom_open2(FAR struct file *filp);
static int eeprom_close2(FAR struct file *filp);
static ssize_t eeprom_read2(struct file *filp, FAR char *buffer, size_t buflen);
static ssize_t eeprom_write2(struct file *filp, FAR const char *buffer, size_t buflen);
static off_t eeprom_seek2(FAR struct file *filp, off_t offset, int whence);
#endif

static const struct file_operations eeprom_fops[MAX_EEPROMS] = {{
	.open  = eeprom_open0,
	.close = eeprom_close0,
	.read  = eeprom_read0,
	.write = eeprom_write0,
	.seek  = eeprom_seek0,
	.ioctl = 0,
#ifndef CONFIG_DISABLE_POLL
	.poll  = 0
#endif
}
#if (MAX_EEPROMS > 1)
,{
	.open  = eeprom_open1,
	.close = eeprom_close1,
	.read  = eeprom_read1,
	.write = eeprom_write1,
	.seek  = eeprom_seek1,
}
#endif
#if (MAX_EEPROMS > 2)
,{
	.open  = eeprom_open2,
	.close = eeprom_close2,
	.read  = eeprom_read2,
	.write = eeprom_write2,
	.seek  = eeprom_seek2,
}
#endif
};

static FAR struct eeprom_dev_s
{
	struct i2c_dev_s	*i2c;
	uint8_t eeprom_address;
	uint16_t eeprom_size_bytes;
	uint16_t eeprom_page_size_bytes;
	uint16_t eeprom_page_write_time;
	off_t offset;
	bool is_open;
} eeprom_dev[MAX_EEPROMS];

static int
eeprom_open0(FAR struct file *filp)
{
	/* only allow one open at a time */
	if (eeprom_dev[0].is_open) {
		errno = EBUSY;
		return -EBUSY;
	}
	/* reset pointer */
	//eeprom_dev[0].is_open = true;
	eeprom_dev[0].offset = 0;
	return OK;
}
#if (MAX_EEPROMS > 1)
static int
eeprom_open1(FAR struct file *filp)
{
	/* only allow one open at a time */
	if (eeprom_dev[1].is_open) {
		errno = EBUSY;
		return -EBUSY;
	}
	/* reset pointer */
	//eeprom_dev[1].is_open = true;
	eeprom_dev[1].offset = 0;
	return OK;
}
#endif
#if (MAX_EEPROMS > 2)
static int
eeprom_open2(FAR struct file *filp)
{
	/* only allow one open at a time */
	if (eeprom_dev[2].is_open) {
		errno = EBUSY;
		return -EBUSY;
	}
	/* reset pointer */
	//eeprom_dev[2].is_open = true;
	eeprom_dev[2].offset = 0;
	return OK;
}
#endif

static int
eeprom_close0(FAR struct file *filp)
{
	eeprom_dev[0].is_open = false;
	return OK;
}
#if (MAX_EEPROMS > 1)
static int
eeprom_close1(FAR struct file *filp)
{
	eeprom_dev[1].is_open = false;
	return OK;
}
#endif
#if (MAX_EEPROMS > 2)
static int
eeprom_close2(FAR struct file *filp)
{
	eeprom_dev[2].is_open = false;
	return OK;
}
#endif

static int
eeprom_read_internal(int dev, uint16_t len, uint8_t *data)
{
	/* abort if the number of requested bytes exceeds the EEPROM size */
	if (eeprom_dev[dev].offset + len > eeprom_dev[dev].eeprom_size_bytes)
	{
		errno = ENOSPC;
		return -ENOSPC;
	}

	/* set device address */
	I2C_SETADDRESS(eeprom_dev[dev].i2c, eeprom_dev[dev].eeprom_address, 7);

	uint8_t cmd[2] = {0, 0}; /* first (or only) part of address */
	/* second part of address, omitted if eeprom has 256 bytes or less */
	int ret = 0;
	int remaining = len;
	int readcounts = 0;

	while (remaining > 0)
	{
		/* read all requested bytes over potentially multiple pages */
		//int readlen = (remaining < eeprom_dev[dev].eeprom_page_size_bytes) ? remaining : eeprom_dev[dev].eeprom_page_size_bytes;
		int read_offset = eeprom_dev[dev].offset + len - remaining;//+ write_counts*eeprom_dev[dev].eeprom_page_size_bytes;
		/* set read length to page border */
		int readlen = eeprom_dev[dev].eeprom_page_size_bytes - (read_offset % eeprom_dev[dev].eeprom_page_size_bytes);//(remaining < eeprom_dev[dev].eeprom_page_size_bytes) ? remaining : eeprom_dev[dev].eeprom_page_size_bytes;
		/* cap read length if not a full page read is needed */
		if (readlen > remaining) readlen = remaining;

		if (eeprom_dev[dev].eeprom_size_bytes <= 256)
		{
			cmd[0] = (read_offset); /* set at first byte */
			/* 8 bit addresses */
			ret = I2C_WRITEREAD(eeprom_dev[dev].i2c, cmd, 1, (data+(readcounts*eeprom_dev[dev].eeprom_page_size_bytes)), readlen);
		}
		else
		{
			/* 16 bit addresses */
			/* EEPROM: first address high, then address low */
			cmd[0] = (((uint16_t)read_offset) >> 8);
			cmd[1] = (((uint8_t)read_offset));
			ret = I2C_WRITEREAD(eeprom_dev[dev].i2c, cmd, 2, (data+(readcounts*eeprom_dev[dev].eeprom_page_size_bytes)), readlen);
		}

		/* abort on error */
		if (ret < 0) break;

		/* handled another chunk */
		remaining -= readlen;
		readcounts++;
	}

	/* use the negated value from I2C_TRANSFER to fill errno */
	errno = -ret;

	/* return len if data was read, < 0 else */
	if (ret == OK)
		eeprom_dev[dev].offset += len;
		return len;

	/* no data, return negated value from I2C_TRANSFER */
	return ret;
}

static int
eeprom_write_internal(int dev, uint16_t len, const uint8_t *data)
{
	/* abort if the number of requested bytes exceeds the EEPROM size */
	if (eeprom_dev[dev].offset + len > eeprom_dev[dev].eeprom_size_bytes)
	{
		errno = ENOSPC;
		return -ENOSPC;
	}

	int ret = 0;
	int remaining = len;
	int write_counts = 0;

	uint8_t write_buf[2];

	while (remaining > 0)
	{
		/* write all requested bytes over potentially multiple pages */
		int write_offset = eeprom_dev[dev].offset + len - remaining;//+ write_counts*eeprom_dev[dev].eeprom_page_size_bytes;
		/* set write length to page border */
		int writelen = eeprom_dev[dev].eeprom_page_size_bytes - (write_offset % eeprom_dev[dev].eeprom_page_size_bytes);//(remaining < eeprom_dev[dev].eeprom_page_size_bytes) ? remaining : eeprom_dev[dev].eeprom_page_size_bytes;
		/* cap write length if not a full page write is requested */
		if (writelen > remaining) writelen = remaining;

		if (eeprom_dev[dev].eeprom_size_bytes <= 256)
		{
			write_buf[0] = (write_offset); /* set at first byte */
			/* 8 bit addresses */

			const uint8_t* data_ptr = (data+(write_offset));

			struct i2c_msg_s msgv_eeprom_write[2] = {
					{
							.addr   = eeprom_dev[dev].eeprom_address,
							.flags  = I2C_M_NORESTART,
							.buffer = write_buf,
							.length = 1
					},
					{
							.addr   = eeprom_dev[dev].eeprom_address,
							.flags  = I2C_M_NORESTART,
							.buffer = (uint8_t*)data_ptr,
							.length = writelen
					}
			};


			if ( (ret = I2C_TRANSFER(eeprom_dev[dev].i2c, msgv_eeprom_write, 2)) == OK )
			{
				//printf("SUCCESS WRITING EEPROM 8BIT ADDR: %d, bytes: %d\n", ret, writelen);
			}
		}
		else
		{
			/* 16 bit addresses */
			/* EEPROM: first address high, then address low */
			write_buf[0] = (((uint16_t)write_offset) >> 8);
			write_buf[1] = (((uint8_t)write_offset));

			const uint8_t* data_ptr = data+(write_counts*eeprom_dev[dev].eeprom_page_size_bytes);

			struct i2c_msg_s msgv_eeprom_write[2] = {
					{
							.addr   = eeprom_dev[dev].eeprom_address,
							.flags  = I2C_M_NORESTART,
							.buffer = write_buf,
							.length = 2
					},
					{
							.addr   = eeprom_dev[dev].eeprom_address,
							.flags  = I2C_M_NORESTART,
							.buffer = (uint8_t*)data_ptr,
							.length = writelen
					}
			};


			if ( (ret = I2C_TRANSFER(eeprom_dev[dev].i2c, msgv_eeprom_write, 2)) == OK )
			{
				//printf("SUCCESS WRITING EEPROM 16BIT ADDR: %d, bytes: %d\n", ret, writelen);
			}
		}

		/* abort on error */
		if (ret < 0) break;

		/* handled another chunk */
		remaining -= writelen;
		write_counts++;
		/* wait for the device to write the page */
		usleep(eeprom_dev[dev].eeprom_page_write_time);
	}

	/* use the negated value from I2C_TRANSFER to fill errno */
	errno = -ret;

	/* return length if data was written, < 0 else */
	if (ret == OK)
		eeprom_dev[dev].offset += len;
		return len;

	/* no data, return negated value from I2C_TRANSFER */
	return ret;
}

static ssize_t
eeprom_read0(struct file *filp, char *buffer, size_t buflen)
{
	return eeprom_read_internal(0, buflen, (uint8_t *)buffer);
}
#if (MAX_EEPROMS > 1)
static ssize_t
eeprom_read1(struct file *filp, char *buffer, size_t buflen)
{
	return eeprom_read_internal(1, buflen, (uint8_t *)buffer);
}
#endif
#if (MAX_EEPROMS > 2)
static ssize_t
eeprom_read2(struct file *filp, char *buffer, size_t buflen)
{
	return eeprom_read_internal(2, buflen, (uint8_t *)buffer);
}
#endif

static ssize_t
eeprom_write0(struct file *filp, const char *buffer, size_t buflen)
{
	return eeprom_write_internal(0, buflen, (const uint8_t *)buffer);
}
#if (MAX_EEPROMS > 1)
static ssize_t
eeprom_write1(struct file *filp, const char *buffer, size_t buflen)
{
	return eeprom_write_internal(1, buflen, (const uint8_t *)buffer);
}
#endif
#if (MAX_EEPROMS > 2)
static ssize_t
eeprom_write2(struct file *filp, const char *buffer, size_t buflen)
{
	return eeprom_write_internal(2, buflen, (const uint8_t *)buffer);
}
#endif

static off_t eeprom_seek0(FAR struct file *filp, off_t offset, int whence)
{
	switch (whence)
	{
	case SEEK_SET:
		if (offset < eeprom_dev[0].eeprom_size_bytes - 1) {
			eeprom_dev[0].offset = offset;
		} else {
			errno = ESPIPE;
			return -ESPIPE;
		}
		break;
	case SEEK_CUR:
		if (eeprom_dev[0].offset + offset < eeprom_dev[0].eeprom_size_bytes - 1) {
			eeprom_dev[0].offset = eeprom_dev[0].offset + offset;
		} else {
			errno = ESPIPE;
			return -ESPIPE;
		}
		break;
	case SEEK_END:
		errno = ESPIPE;
		return -ESPIPE;
		break;
	}
	return eeprom_dev[0].offset;
}
#if (MAX_EEPROMS > 1)
static off_t eeprom_seek1(FAR struct file *filp, off_t offset, int whence)
{
	switch (whence)
	{
	case SEEK_SET:
		if (offset < eeprom_dev[1].eeprom_size_bytes - 1) {
			eeprom_dev[1].offset = offset;
		} else {
			errno = ESPIPE;
			return -ESPIPE;
		}
		break;
	case SEEK_CUR:
		if (eeprom_dev[1].offset + offset < eeprom_dev[1].eeprom_size_bytes - 1) {
			eeprom_dev[1].offset = eeprom_dev[1].offset + offset;
		} else {
			errno = ESPIPE;
			return -ESPIPE;
		}
		break;
	case SEEK_END:
		errno = ESPIPE;
		return -ESPIPE;
		break;
	}
	return eeprom_dev[1].offset;
}
#endif
#if (MAX_EEPROMS > 2)
static off_t eeprom_seek2(FAR struct file *filp, off_t offset, int whence)
{
	switch (whence)
	{
	case SEEK_SET:
		if (offset < eeprom_dev[2].eeprom_size_bytes - 1) {
			eeprom_dev[2].offset = offset;
		} else {
			errno = ESPIPE;
			return -ESPIPE;
		}
		break;
	case SEEK_CUR:
		if (eeprom_dev[2].offset + offset < eeprom_dev[2].eeprom_size_bytes - 1) {
			eeprom_dev[2].offset = eeprom_dev[2].offset + offset;
		} else {
			errno = ESPIPE;
			return -ESPIPE;
		}
		break;
	case SEEK_END:
		errno = ESPIPE;
		return -ESPIPE;
		break;
	}
	return eeprom_dev[2].offset;
}
#endif

int
eeprom_attach(struct i2c_dev_s *i2c, uint8_t device_address, uint16_t total_size_bytes, uint16_t page_size_bytes, uint16_t page_write_time_us, const char* device_name, uint8_t fail_if_missing)
{
	static int eeprom_dev_counter = 0;
	eeprom_dev[eeprom_dev_counter].i2c = i2c;
	eeprom_dev[eeprom_dev_counter].eeprom_address = device_address;
	eeprom_dev[eeprom_dev_counter].eeprom_size_bytes = total_size_bytes;
	eeprom_dev[eeprom_dev_counter].eeprom_page_size_bytes = page_size_bytes;
	eeprom_dev[eeprom_dev_counter].eeprom_page_write_time = page_write_time_us;
	eeprom_dev[eeprom_dev_counter].offset = 0;
	eeprom_dev[eeprom_dev_counter].is_open = false;

	int ret;

	if (fail_if_missing) {
		/* read first value */
		uint8_t read_test;
		ret = (eeprom_read_internal(eeprom_dev_counter, 1, &read_test) == 1) ? OK : ERROR;
	} else {
		ret = OK;
	}

	/* make ourselves available */
	if (ret == OK)
	{
		register_driver(device_name, &(eeprom_fops[eeprom_dev_counter]), 0666, NULL);
		eeprom_dev_counter++;
	}

	/* Return 0 for device found, error number else */
	return ret;
}
