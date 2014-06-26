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
#include <systemlib/lzss.h>

#ifdef CONFIG_FS_WRITABLE
# define REGISTER_FUNCTION ramdisk_register
#else
# define REGISTER_FUNCTION romdisk_register
#endif 

extern "C" __EXPORT int REGISTER_FUNCTION (int minor, uint8_t *buffer, uint32_t nsectors, uint16_t sectsize);

class ZROMDisk : public device::BDev
{
public:

	ZROMDisk(const char *devname, const unsigned char *image, uint32_t image_size, uint32_t sectorsize);
	~ZROMDisk();

	virtual int	open(struct inode *inode);
	virtual ssize_t	read(struct inode *inode, unsigned char *buffer, size_t start_sector, size_t nsectors);
	virtual int	geometry(struct inode *inode, struct geometry *g);

private:
	struct ZRDChunk {
		uint32_t	skip;
		unsigned char	data[];
	};

	struct ZRDHeader {
		uint8_t		magic;
		uint8_t		log_n;
		uint8_t		f;
		uint8_t		threshold;
		uint32_t	chunksize;
		uint32_t	uncomp_len;
		ZRDChunk	chunk0;
	};

	static const unsigned	_magic = 'Z';

	const ZRDHeader		*const _hdr;
	const unsigned		_sectorsize;
	const unsigned		_image_size;
	perf_counter_t		_read_perf;

	ssize_t			_read(unsigned char *buffer, size_t start_sector, size_t nsectors);
	const ZRDChunk		*_chunkdata(unsigned chunk);
};


ZROMDisk::ZROMDisk(const char *devname, const unsigned char *image, unsigned image_size, uint32_t sectorsize) :
	BDev("zromdisk", devname),
	_hdr(reinterpret_cast<const ZRDHeader *>(image)),
	_sectorsize(sectorsize),
	_image_size(image_size),
	_read_perf(perf_alloc(PC_ELAPSED, "zromdisk read"))
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
	ssize_t result = 0;
	lock();
	perf_begin(_read_perf);

	/* read entirely outside device */
	if ((start_sector * _sectorsize) > _hdr->uncomp_len)
		goto out;

	/* read overlaps end of device - shorten it up */
	if (((start_sector + nsectors) * _sectorsize) > _hdr->uncomp_len) {
		nsectors = (_hdr->uncomp_len / _sectorsize) - start_sector;
	}

	while (result < (ssize_t)nsectors) {
		ssize_t partial = _read(buffer, start_sector, nsectors - result);

		if (partial <= 0)
			break;

		result += partial;
		start_sector += partial;
		buffer += partial * _sectorsize;
	}

out:
	perf_end(_read_perf);
	unlock();

	return result;
}

ssize_t
ZROMDisk::_read(unsigned char *buffer, size_t start_sector, size_t nsectors)
{
	unsigned chunk = (start_sector * _sectorsize) / _hdr->chunksize;
	unsigned offset = (start_sector * _sectorsize) % _hdr->chunksize;
	unsigned space = _hdr->chunksize - offset;
	if (nsectors > (space / _sectorsize))
		nsectors = space / _sectorsize;

	/* find the chunk containing the start of the operation */
	const ZRDChunk *cp = _chunkdata(chunk);

	/* no chunk -> no data */
	if (cp == nullptr)
		return 0;

	struct lzss_decomp decomp;

	decomp.discard = offset;
	decomp.resid = nsectors * _sectorsize;
	decomp.src = &cp->data[0];
	decomp.dst = buffer;

	decomp.N = (1 << _hdr->log_n);
	decomp.F = _hdr->f;
	decomp.THRESHOLD = _hdr->threshold;

	if (lzss_decompress(&decomp) != OK)
		return -1;

	/* we could support short reads here, but it shouldn't happen so the complexity isn't worth it */
	return (decomp.resid > 0) ? 0 : nsectors;
}

const ZROMDisk::ZRDChunk *
ZROMDisk::_chunkdata(unsigned chunk)
{
	const ZRDChunk *cp = &_hdr->chunk0;

	while (chunk--) {
		if (cp->skip == 0)
			return nullptr;

		const unsigned char *p = reinterpret_cast<const unsigned char *>(cp);
		p += cp->skip;
		cp = reinterpret_cast<const ZRDChunk *>(p);
	}
	return cp;
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
