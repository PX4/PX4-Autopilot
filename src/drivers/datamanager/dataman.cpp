/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file dataman.cpp
 *
 * DATAMANAGER driver.
 */

#include <nuttx/config.h>
#include <drivers/device/device.h>
#include <drivers/drv_dataman.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <systemlib/err.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <semaphore.h>
#include <errno.h>

struct filedesc {
	int		fd;
	int		offset;
	unsigned char	persist;
};

class DATAMANAGER : device::CDev
{
public:
	DATAMANAGER();
	virtual ~DATAMANAGER();

	virtual int	open(struct file *filp);
	virtual int	close(struct file *filp);
	virtual ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	virtual ssize_t	write(struct file *filp, const char *buffer, size_t buflen);
	virtual int	ioctl(struct file *filp, int cmd, unsigned long arg);
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int dataman_main(int argc, char *argv[]);

namespace dm
{
DATAMANAGER	*g_dev = nullptr;

sem_t g_mutex;

unsigned g_key_sizes[DM_KEY_NUM_KEYS] = {
	DM_KEY_RTL_POINT_MAX,
	DM_KEY_RETURN_POINT_MAX,
	DM_KEY_SAFE_POINTS_MAX,
	DM_KEY_WAY_POINTS_MAX,
	DM_KEY_FENCE_POINTS_MAX
};

unsigned int g_key_offsets[DM_KEY_NUM_KEYS];

void	start();
void	stop();
int	seek(int fh, unsigned offset, int set);
void	lock(void);
void	unlock(void);

/**
 * Start the driver.
 */
void
start()
{
	if (g_dev)
		errx(1, "already started");

	/* create the driver */
	g_dev = new DATAMANAGER();

	if (g_dev == nullptr)
		errx(1, "driver start failed");

	warnx("Ready");

	exit(0);
}

/**
 * Stop the driver.
 */
void
stop()
{
	if (g_dev) {
		delete g_dev;
		g_dev = nullptr;
	}

	warnx("Stopped");

	exit(0);
}

/* NUTTX is not really POSIX compliant */
int
seek(int fh, unsigned offset, int set)
{
	int result;

	if (set == SEEK_END) {
		/* ouch... NUTTX doesn't support seek to end!!! */
		char buffer[64];

		::lseek(fh, 0, SEEK_SET);

		while (::read(fh, buffer, sizeof(buffer)) != 0);

		result = ::lseek(fh, 0, SEEK_CUR);
	}

	else
		result = ::lseek(fh, offset, set);

	return result;
}

void
lock(void)
{
	sem_wait(&g_mutex);
}

void
unlock(void)
{
	sem_post(&g_mutex);
}

}

DATAMANAGER::DATAMANAGER() : CDev("dataman", DATAMANAGER_DEVICE_PATH)
{
	CDev::init();

	/* Set up and get exclusive use */
	sem_init(&dm::g_mutex, 0, 1);

	/* Initialize per primary key offset table */
	dm::g_key_offsets[0] = 0;

	for (unsigned i = 1; i < DM_KEY_NUM_KEYS; i++) {
		dm::g_key_offsets[i] = dm::g_key_offsets[i - 1] + (dm::g_key_sizes[i - 1] * (DM_MAX_DATA_SIZE + 2));
	}

	dm::g_dev = this;
}

DATAMANAGER::~DATAMANAGER()
{
	dm::g_dev = nullptr;
}

int
DATAMANAGER::open(struct file *f)
{
	f->f_priv = malloc(sizeof(filedesc));

	if (f->f_priv) {
		filedesc *desc = (filedesc *)(f->f_priv);
		desc->offset = 0;
		desc->persist = DM_PERSIST_VOLATILE;
		desc->fd = ::open("/fs/microsd/data", O_RDWR | O_CREAT | O_BINARY);

		if (desc->fd < 0)
			return desc->fd;

		return (int)f;
	}

	set_errno(ENOENT);
	return -1;
}

int
DATAMANAGER::close(struct file *f)
{
	filedesc *desc = (filedesc *)(f->f_priv);

	if (desc) {
		// Can't close... file system is locked!!!
		//if (desc->fd >= 0)
		//	::close(desc->fd);
		free(desc);
		f->f_priv = nullptr;
	}

	return OK;
}

ssize_t
DATAMANAGER::write(struct file *f, const char *buf, size_t count)
{
	filedesc *desc = (filedesc *)(f->f_priv);
	unsigned char buffer[DM_MAX_DATA_SIZE + 2];
	size_t len;

	if (desc == nullptr) {
		set_errno(ENOENT);
		return -1;
	}

	if (desc->fd < 0) {
		set_errno(ENOENT);
		return -1;
	}

	/* Make sure caller has given us data we can handle */
	if (count > DM_MAX_DATA_SIZE) {
		set_errno(E2BIG);
		return -1;
	}

	/* Write out the data */
	buffer[0] = count;
	buffer[1] = desc->persist;
	memcpy(buffer + 2, buf, count);
	count += 2;

	len = -1;
	dm::lock();

	if (dm::seek(desc->fd, desc->offset, SEEK_SET) == desc->offset)
		len = ::write(desc->fd, buffer, count);

	dm::unlock();

	if (len != count) {
		set_errno(EIO);
		return -1;
	}

	return count - 2;
}

ssize_t
DATAMANAGER::read(struct file *f, char *buf, size_t count)
{
	filedesc *desc = (filedesc *)(f->f_priv);
	unsigned char buffer[DM_MAX_DATA_SIZE + 2];
	int len;

	if (desc == nullptr) {
		set_errno(ENOENT);
		return -1;
	}

	if (desc->fd < 0) {
		set_errno(ENOENT);
		return -1;
	}

	/* Make sure the caller hasn't asked for something we can't handle */
	if (count > DM_MAX_DATA_SIZE) {
		set_errno(EINVAL);
		return -1;
	}

	len = -1;
	dm::lock();

	if (dm::seek(desc->fd, desc->offset, SEEK_SET) == desc->offset)
		len = ::read(desc->fd, buffer, count + 2);

	dm::unlock();

	/* Check for length issues */
	if (len < 0)
		return -1;

	if (len == 0)
		buffer[0] = 0;

	if (buffer[0] > 0) {
		if (buffer[0] > count) {
			set_errno(EBADSLT);
			return -1;
		}

		/* Looks good, copy it to the caller's buffer */
		memcpy(buf, buffer + 2, buffer[0]);
	}

	return buffer[0];
}

int
DATAMANAGER::ioctl(struct file *f, int cmd, unsigned long arg)
{
	int result = OK;
	filedesc *desc = (filedesc *)(f->f_priv);
	unsigned char buffer[2];

	unsigned char p_key, s_key;
	size_t offset;

	if (desc == nullptr) {
		set_errno(ENOENT);
		return -1;
	}

	if (desc->fd == -1) {
		set_errno(ENOENT);
		return -1;
	}

	switch (cmd) {
	case DM_SET_KEY:

		p_key = arg >> 8;
		s_key = arg;

		if (p_key >= DM_KEY_NUM_KEYS) {
			set_errno(EINVAL);
			result = -1;
			break;
		}

		if (s_key >= dm::g_key_sizes[p_key]) {
			set_errno(EINVAL);
			result = -1;
			break;
		}

		desc->offset = dm::g_key_offsets[p_key] + (s_key * (DM_MAX_DATA_SIZE + 2));

		break;

	case DM_SET_PERSIST:

		desc->persist = arg;
		break;

	case DM_INIT:

		/* Loop through all of the data segments and delete those that are not persistent */
		offset = 0; 

		dm::lock();

		while (1) {
			size_t len;

			/* Get data segment at current offset */
			if (dm::seek(desc->fd, offset, SEEK_SET) != (int)offset) {
				result = -1;
				break;
			}

			len = ::read(desc->fd, buffer, sizeof(buffer));

			if (len == 0)
				break;

			/* check if segment contains data */
			if (buffer[0]) {
				int clear_entry = 0;

				/* Whether data gets deleted depends on reset type and data segment's persistence setting */
				if (arg == DM_INIT_REASON_POWER_ON) {
					if (buffer[1] != DM_PERSIST_POWER_ON_RESET) {
						clear_entry = 1;
					}

				} else {
					if ((buffer[1] != DM_PERSIST_POWER_ON_RESET) && (buffer[1] != DM_PERSIST_IN_FLIGHT_RESET)) {
						clear_entry = 1;
					}
				}

				/* Set segment to unused if data does not persist */
				if (clear_entry) {
					if (dm::seek(desc->fd, offset, SEEK_SET) != (int)offset) {
						result = -1;
						break;
					}

					buffer[0] = 0;

					len = ::write(desc->fd, buffer, 1);

					if (len != 1) {
						result = -1;
						break;
					}
				}
			}

			offset += DM_MAX_DATA_SIZE + 2;
		}

		dm::unlock();

		break;

	default:
		result = -1;
		break;
	}

	return result;
}

int
dataman_main(int argc, char *argv[])
{

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start"))
		dm::start();

	if (!strcmp(argv[1], "stop"))
		dm::stop();

	errx(1, "unrecognized command, try 'start', 'stop'");
}
