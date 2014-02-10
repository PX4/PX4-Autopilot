/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
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
 ****************************************************************************
 *
 * Portions of this module are derived from code attributed:
 * **************************************************************
 *        LZSS.C -- A Data Compression Program
 * ***************************************************************
 *        4/6/1989 Haruhiko Okumura
 *        Use, distribute, and modify this program freely.
 *        Please send me your improved versions.
 *                PC-VAN          SCIENCE
 *                NIFTY-Serve     PAF01022
 *                CompuServe      74050,1022
 * **************************************************************
 *
 ****************************************************************************/

/**
 * @file	drv_zromdisk.cpp
 *
 * Simple compressed ROM block device, suitable for hosting the ROMFS.
 */

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <sys/types.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <assert.h>
#include <debug.h>

#include <drivers/device/device.h>
#include <systemlib/perf_counter.h>

#ifdef CONFIG_FS_WRITABLE
# define REGISTER_FUNCTION ramdisk_register
#else
# define REGISTER_FUNCTION romdisk_register
#endif 

extern "C" __EXPORT int REGISTER_FUNCTION (int minor, uint8_t *buffer, uint32_t nsectors, uint16_t sectsize);

class ZROMDisk : public device::BDev
{
public:
	/*
	 * Header structure identifying the characteristics of the compressed data.
	 */
	struct ZRDHeader {
		uint8_t		magic;
		uint8_t		log_n;
		uint8_t		f;
		uint8_t		threshold;
		uint32_t	uncomp_len;
		unsigned char	data[];
	};

	ZROMDisk(const char *devname, const unsigned char *image, uint32_t image_size, uint32_t sectorsize);
	~ZROMDisk();

	virtual int	open(struct inode *inode);
	virtual ssize_t	read(struct inode *inode, unsigned char *buffer, size_t start_sector, size_t nsectors);
	virtual int	geometry(struct inode *inode, struct geometry *g);

private:
	struct State 
	{
		unsigned	discard;
		unsigned	resid;
		const unsigned char *src;
		unsigned char	*dst;
		unsigned char	*text_buf;

		int		_getc()      { return *src++; }
		void		_putc(int c) 
		{
			if (discard > 0) {
				discard--;
			} else if (resid > 0) {
				*dst++ = c;
				resid--;
			}
		}
	};

	static const unsigned	_magic = 'Z';

	const ZRDHeader		*const _hdr;
	const unsigned		_sectorsize;
	const unsigned		_image_size;
	perf_counter_t		_read_perf;
	perf_counter_t		_seq;
	State			*_s;
	unsigned		_next;

	void			_lzss_decode(State &s);
};


ZROMDisk::ZROMDisk(const char *devname, const unsigned char *image, unsigned image_size, uint32_t sectorsize) :
	BDev("zromdisk", devname),
	_hdr(reinterpret_cast<const ZRDHeader *>(image)),
	_sectorsize(sectorsize),
	_image_size(image_size),
	_read_perf(perf_alloc(PC_ELAPSED, "zromdisk read")),
	_seq(perf_alloc(PC_COUNT, "zromdisk sequential")),
	_s(nullptr),
	_next(0xffffffff)
{
	log("@%p ss%u", image, sectorsize);
}

ZROMDisk::~ZROMDisk()
{
	if (_read_perf != nullptr)
		perf_free(_read_perf);
}

int
ZROMDisk::open(struct inode *inode)
{

	if (_hdr->magic != _magic) {
		log("bad header magic");
		return -EINVAL;
	}

	return BDev::open(inode);
}

ssize_t
ZROMDisk::read(struct inode *inode, unsigned char *buffer, size_t start_sector, size_t nsectors)
{
	ssize_t result;
	State s;

	lock();
	_s = &s;
	perf_begin(_read_perf);

	if (start_sector == _next)
		perf_count(_seq);
	_next = start_sector + nsectors;

	/* read entirely outside device */
	if ((start_sector * _sectorsize) > _hdr->uncomp_len) {
		result = 0;
		goto out;
	}

	/* read overlaps end of device - shorten it up */
	if (((start_sector + nsectors) * _sectorsize) > _hdr->uncomp_len) {
		nsectors = (_hdr->uncomp_len / _sectorsize) - start_sector;
	}

	s.discard = start_sector * _sectorsize;
	s.resid = nsectors * _sectorsize;
	s.src = &_hdr->data[0];
	s.dst = buffer;

	{
		unsigned buffer_size = (1 << _hdr->log_n) + _hdr->f - 1;
		s.text_buf = (unsigned char *)malloc(buffer_size);

		if (s.text_buf == nullptr) {
			log("can't get %u bytes", buffer_size);
			result = -ENOMEM;
			goto out;
		}
	}

	_lzss_decode(s);

	free(s.text_buf);

	if (s.resid > 0) {
		log("short read by %u", s.resid);
		result = -EIO;
	} else {
		result = nsectors;
	}

out:
	perf_end(_read_perf);
	unlock();

	return result;
}

int
ZROMDisk::geometry(struct inode *inode, struct geometry *g)
{

	if (!g)
		return -EINVAL;

	g->geo_available = true;
	g->geo_mediachanged = false;
	g->geo_writeenabled = false;
	g->geo_nsectors = (_hdr->uncomp_len + _sectorsize - 1) / _sectorsize;
	g->geo_sectorsize = _sectorsize;

	return OK;
}

void
ZROMDisk::_lzss_decode(State &s)
{
	int  i, j, k, r, c;
	unsigned int  flags;
	unsigned N = (1 << _hdr->log_n);
	unsigned F = _hdr->f;
	unsigned THRESHOLD = _hdr->threshold;

	r = N - F;
	memset(s.text_buf, ' ', r);
	flags = 0;

	while (s.resid > 0) {
		if (((flags >>= 1) & 256) == 0) {
			c = s._getc();
			flags = c | 0xff00;             /* uses higher byte cleverly */
		}                                       /* to count eight */

		if (flags & 1) {
			c = s._getc();
			s._putc(c);
			s.text_buf[r++] = c;
			r &= (N - 1);

		} else {
			i = s._getc();
			j = s._getc();

			i |= ((j & 0xf0) << 4);
			j = (j & 0x0f) + THRESHOLD;

			for (k = 0; k <= j; k++) {
				c = s.text_buf[(i + k) & (N - 1)];

				s._putc(c);
				s.text_buf[r++] = c;
				r &= (N - 1);
			}
		}
	}
}

ZROMDisk *zrd;

/*
 * Overrides ram/romdisk_register exported by the NuttX ramdisk driver.
 */
int
REGISTER_FUNCTION (int minor, uint8_t *buffer, uint32_t nsectors, uint16_t sectsize)
{
	static char devname[16];
	sprintf(devname, "/dev/ram%d", minor);

	ASSERT(buffer != nullptr);
	ASSERT(nsectors > 0);
	ASSERT(sectsize > 0);

	/* image size is an approximation due to sector size truncation elsewhere */
	zrd = new ZROMDisk(devname, buffer, (nsectors * sectsize) + sectsize - 1, sectsize);
	ASSERT(zrd != nullptr);
	if (zrd->init() != 0) {
		delete zrd;
		zrd = nullptr;
	}

	return (zrd == nullptr) ? -1 : OK;
}
