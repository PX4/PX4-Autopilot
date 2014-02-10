/****************************************************************************
 *
 *   Copyright (C) 2012,2014 PX4 Development Team. All rights reserved.
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
 * @file bdev.cpp
 *
 * Block device base class.
 */

#include "device.h"

#include <sys/ioctl.h>
#include <arch/irq.h>

#include <stdlib.h>
#include <stdio.h>

namespace device
{

/*
 * The standard NuttX operation dispatch table can't call C++ member functions
 * directly, so we have to bounce them through this dispatch table.
 */
static int	bdev_open(struct inode *inode);
static int	bdev_close(struct inode *inode);
static ssize_t	bdev_read(struct inode *inode, unsigned char *buffer, size_t start_sector, unsigned int nsectors);
#ifdef CONFIG_FS_WRITABLE
static ssize_t	bdev_write(struct inode *inode, const unsigned char *buffer, size_t start_sector, unsigned int nsectors);
#endif
static int	bdev_geometry(struct inode *inode, struct geometry *geometry);
static int	bdev_ioctl(struct inode *inode, int cmd, unsigned long arg);

/**
 * Block device indirection table.
 *
 * Every bdev we register gets the same function table; we use the private data
 * field in the inode to store the instance pointer.
 *
 * Note that we use the GNU extension syntax here because we don't get designated
 * initialisers in gcc 4.6.
 */
const struct block_operations BDev::bops = {
open	: bdev_open,
close	: bdev_close,
read	: bdev_read,
#ifdef CONFIG_FS_WRITABLE
write	: bdev_write,
#endif
geometry: bdev_geometry,
ioctl	: bdev_ioctl,
};

BDev::BDev(const char *name,
	   const char *devname) :
	// base class
	Device(name),
	// public
	// protected
	// private
	_devname(devname),
	_registered(false),
	_open_count(0)
{
}

BDev::~BDev()
{
	if (_registered)
		unregister_blockdriver(_devname);
}

int
BDev::init()
{
	// base class init first
	int ret = Device::init();

	if (ret != OK)
		goto out;

	// now register the driver
	if (_devname != nullptr) {
		ret = register_blockdriver(_devname, &bops, 0, (void *)this);

		if (ret != OK)
			goto out;

		_registered = true;
	}

out:
	return ret;
}

/*
 * Default implementations of the block device interface
 */
int
BDev::open(struct inode *inode)
{
	int ret = OK;

	lock();
	/* increment the open count */
	_open_count++;

	if (_open_count == 1) {

		/* first-open callback may decline the open */
		ret = open_first(inode);

		if (ret != OK)
			_open_count--;
	}

	unlock();

	return ret;
}

int
BDev::open_first(struct inode *inode)
{
	return OK;
}

int
BDev::close(struct inode *inode)
{
	int ret = OK;

	lock();

	if (_open_count > 0) {
		/* decrement the open count */
		_open_count--;

		/* callback cannot decline the close */
		if (_open_count == 0)
			ret = close_last(inode);

	} else {
		ret = -EBADF;
	}

	unlock();

	return ret;
}

int
BDev::close_last(struct inode *inode)
{
	return OK;
}

ssize_t
BDev::read(struct inode *inode, unsigned char *buffer, size_t start_sector, unsigned int nsectors)
{
	return -ENOSYS;
}

ssize_t
BDev::write(struct inode *inode, const unsigned char *buffer, size_t start_sector, unsigned int nsectors)
{
	return -ENOSYS;
}

int
BDev::geometry(struct inode *inode, struct geometry *geometry)
{
	return -ENOSYS;
}

int
BDev::ioctl(struct inode *inode, int cmd, unsigned long arg)
{
	switch (cmd) {

	}

	return -ENOTTY;
}

static int
bdev_open(struct inode *inode)
{
	BDev *bdev = (BDev *)(inode->i_private);

	return bdev->open(inode);
}

static int
bdev_close(struct inode *inode)
{
	BDev *bdev = (BDev *)(inode->i_private);

	return bdev->close(inode);
}

static ssize_t
bdev_read(struct inode *inode, unsigned char *buffer, size_t start_sector, unsigned int nsectors)
{
	BDev *bdev = (BDev *)(inode->i_private);

	return bdev->read(inode, buffer, start_sector, nsectors);
}

#ifdef CONFIG_FS_WRITABLE
static ssize_t
bdev_write(struct inode *inode, const unsigned char *buffer, size_t start_sector, unsigned int nsectors)
{
	BDev *bdev = (BDev *)(inode->i_private);

	return bdev->write(inode, buffer, start_sector, nsectors);
}
#endif

static int
bdev_geometry(struct inode *inode, struct geometry *geometry)
{
	BDev *bdev = (BDev *)(inode->i_private);

	return bdev->geometry(inode, geometry);
}

static int
bdev_ioctl(struct inode *inode, int cmd, unsigned long arg)
{
	BDev *bdev = (BDev *)(inode->i_private);

	return bdev->ioctl(inode, cmd, arg);
}

} // namespace device
