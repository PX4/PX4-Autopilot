/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <board_config.h>
#include <nuttx/config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <poll.h>
#include <assert.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <nuttx/fs/fs.h>

#include <crc32.h>

#ifdef CONFIG_BOARD_CRASHDUMP

#include <systemlib/hardfault_log.h>
#include "chip.h"

#ifdef HAS_SSARC

#include <ssarc_dump.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SSARC_DUMP_FILES      5
#define MAX_OPENCNT           (255) /* Limit of uint8_t */
#define SSARCH_HEADER_SIZE    (sizeof(struct ssarcfh_s))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ssarc_data_s {
	uint8_t     data[PX4_SSARC_BLOCK_DATA];
	uint8_t     padding[PX4_SSARC_BLOCK_SIZE - PX4_SSARC_BLOCK_DATA];
};

struct ssarcfh_s {
	int32_t     fileno;        /* The minor number */
	uint32_t    len;           /* Total Bytes in this file */
	uint8_t     padding0[8];
	uint32_t    clean;         /* No data has been written to the file */
	uint8_t     padding1[12];
	struct timespec lastwrite; /* Last write time */
	uint8_t     padding2[8];
	struct ssarc_data_s data[];        /* Data in the file */
};

struct ssarc_dump_s {
	sem_t    exclsem;            /* For atomic accesses to this structure */
	uint8_t  refs;               /* Number of references */
	struct ssarcfh_s *pf;      /* File in progmem */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     ssarc_dump_open(struct file *filep);
static int     ssarc_dump_close(struct file *filep);
static off_t   ssarc_dump_seek(struct file *filep, off_t offset,
			       int whence);
static ssize_t ssarc_dump_read(struct file *filep, char *buffer,
			       size_t len);
static ssize_t ssarc_dump_write(struct file *filep,
				const char *buffer, size_t len);
static int     ssarc_dump_ioctl(struct file *filep, int cmd,
				unsigned long arg);
static int     ssarc_dump_poll(struct file *filep,
			       struct pollfd *fds, bool setup);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     ssarc_dump_unlink(struct inode *inode);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/


static const struct file_operations ssarc_dump_fops = {
	.open   = ssarc_dump_open,
	.close  = ssarc_dump_close,
	.read   = ssarc_dump_read,
	.write  = ssarc_dump_write,
	.seek   = ssarc_dump_seek,
	.ioctl  = ssarc_dump_ioctl,
	.poll   = ssarc_dump_poll,
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
	.unlink = ssarc_dump_unlink
#endif
};

static struct ssarc_dump_s g_ssarc[SSARC_DUMP_FILES];

/****************************************************************************
 * Name: ssarc_dump_semgive
 ****************************************************************************/

static void ssarc_dump_semgive(struct ssarc_dump_s *priv)
{
	nxsem_post(&priv->exclsem);
}

/****************************************************************************
 * Name: ssarc_dump_semtake
 *
 * Description:
 *   Take a semaphore handling any exceptional conditions
 *
 * Input Parameters:
 *   priv - A reference to the CAN peripheral state
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static int ssarc_dump_semtake(struct ssarc_dump_s *priv)
{
	return nxsem_wait_uninterruptible(&priv->exclsem);
}

/****************************************************************************
 * Name: ssarc_dump_open
 *
 * Description: Open the device
 *
 ****************************************************************************/

static int ssarc_dump_open(struct file *filep)
{
	struct inode *inode = filep->f_inode;
	struct ssarc_dump_s *pmf;
	int ret;

	DEBUGASSERT(inode && inode->i_private);
	pmf = (struct ssarc_dump_s *)inode->i_private;

	/* Increment the reference count */

	ret = ssarc_dump_semtake(pmf);

	if (ret < 0) {
		return ret;
	}

	if (pmf->refs == MAX_OPENCNT) {
		return -EMFILE;

	} else {
		pmf->refs++;
	}

	ssarc_dump_semgive(pmf);
	return OK;
}

/****************************************************************************
 * Name: ssarc_dump_internal_close
 *
 * Description:
 *    Close Progmem entry; Recalculate the time and crc
 *
 ****************************************************************************/

static int ssarc_dump_internal_close(struct ssarcfh_s *pf)
{
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	pf->lastwrite.tv_sec = ts.tv_sec;
	pf->lastwrite.tv_nsec = ts.tv_nsec;
	return pf->len;
}

/****************************************************************************
 * Name: ssarc_dump_close
 *
 * Description: close the device
 *
 ****************************************************************************/

static int ssarc_dump_close(struct file *filep)
{
	struct inode *inode = filep->f_inode;
	struct ssarc_dump_s *pmf;
	int ret = OK;

	DEBUGASSERT(inode && inode->i_private);
	pmf = (struct ssarc_dump_s *)inode->i_private;

	ret = ssarc_dump_semtake(pmf);

	if (ret < 0) {
		return ret;
	}

	if (pmf->refs == 0) {
		ret = -EIO;

	} else {
		pmf->refs--;

		if (pmf->refs == 0) {
			if (pmf->pf->clean == 0) {
				ssarc_dump_internal_close(pmf->pf);
			}
		}
	}

	ssarc_dump_semgive(pmf);
	return ret;
}

/****************************************************************************
 * Name: ssarc_dump_seek
 ****************************************************************************/

static off_t ssarc_dump_seek(struct file *filep, off_t offset,
			     int whence)
{
	struct inode *inode = filep->f_inode;
	struct ssarc_dump_s *pmf;
	off_t newpos;
	int ret;

	DEBUGASSERT(inode && inode->i_private);
	pmf = (struct ssarc_dump_s *)inode->i_private;

	ret = ssarc_dump_semtake(pmf);

	if (ret < 0) {
		return (off_t)ret;
	}

	/* Determine the new, requested file position */

	switch (whence) {
	case SEEK_CUR:
		newpos = filep->f_pos + offset;
		break;

	case SEEK_SET:
		newpos = offset;
		break;

	case SEEK_END:
		newpos = pmf->pf->len + offset;
		break;

	default:

		/* Return EINVAL if the whence argument is invalid */

		ssarc_dump_semgive(pmf);
		return -EINVAL;
	}

	/* Opengroup.org:
	 *
	 *  "The lseek() function shall allow the file offset to be set beyond the
	 *   end of the existing data in the file. If data is later written at this
	 *   point, subsequent reads of data in the gap shall return bytes with the
	 *   value 0 until data is actually written into the gap."
	 *
	 * We can conform to the first part, but not the second.  But return EINVAL
	 * if "...the resulting file offset would be negative for a regular file,
	 *     block special file, or directory."
	 */

	if (newpos >= 0) {
		filep->f_pos = newpos;
		ret = newpos;

	} else {
		ret = -EINVAL;
	}

	ssarc_dump_semgive(pmf);
	return ret;
}

/****************************************************************************
 * Name: ssarc_dump_read
 ****************************************************************************/

static ssize_t ssarc_dump_read(struct file *filep, char *buffer,
			       size_t len)
{
	struct inode *inode = filep->f_inode;
	struct ssarc_dump_s *pmf;
	int ret;

	DEBUGASSERT(inode && inode->i_private);
	pmf = (struct ssarc_dump_s *)inode->i_private;

	ret = ssarc_dump_semtake(pmf);

	if (ret < 0) {
		return (ssize_t)ret;
	}

	/* Trim len if read would go beyond end of device */

	if ((filep->f_pos + len) > pmf->pf->len) {
		len = pmf->pf->len - filep->f_pos;
	}

	int offset = filep->f_pos % PX4_SSARC_BLOCK_DATA;
	int abs_pos = filep->f_pos / PX4_SSARC_BLOCK_DATA;
	size_t to_read = len;

	if (offset != 0) {
		memcpy(buffer, &pmf->pf->data[abs_pos], PX4_SSARC_BLOCK_DATA - offset);
		to_read -= (PX4_SSARC_BLOCK_DATA - offset);
		abs_pos++;
	}

	for (size_t i = 0; i < (len / PX4_SSARC_BLOCK_DATA); i++) {
		if (to_read >= PX4_SSARC_BLOCK_DATA) {
			memcpy(buffer, &pmf->pf->data[abs_pos], PX4_SSARC_BLOCK_DATA);
			abs_pos++;
			buffer += PX4_SSARC_BLOCK_DATA;
			to_read -= PX4_SSARC_BLOCK_DATA;

		} else {
			memcpy(buffer, &pmf->pf->data[abs_pos], to_read);
			buffer += to_read;
			abs_pos++;
		}
	}

	filep->f_pos += len;
	ssarc_dump_semgive(pmf);
	return len;
}

/****************************************************************************
 * Name: ssarc_dump_internal_write
 ****************************************************************************/

static ssize_t ssarc_dump_internal_write(struct ssarcfh_s *pf,
		const char *buffer,
		off_t offset, size_t len)
{
	/* Write data */
	for (size_t i = 0; i <= (len / PX4_SSARC_BLOCK_DATA); i++) {
		memcpy(&pf->data[offset + i], &buffer[PX4_SSARC_BLOCK_DATA * i],
		       PX4_SSARC_BLOCK_DATA);
	}

	return len;
}

/****************************************************************************
 * Name: ssarc_dump_write
 ****************************************************************************/

static ssize_t ssarc_dump_write(struct file *filep,
				const char *buffer, size_t len)
{
	struct inode *inode = filep->f_inode;
	struct ssarc_dump_s *pmf;
	int ret = -EFBIG;

	DEBUGASSERT(inode && inode->i_private);
	pmf = (struct ssarc_dump_s *)inode->i_private;

	/* Forbid writes past the end of the device */

	if (filep->f_pos < (int)pmf->pf->len) {
		/* Clamp len to avoid crossing the end of the memory */

		if ((filep->f_pos + len) > pmf->pf->len) {
			len = pmf->pf->len - filep->f_pos;
		}

		ret = ssarc_dump_semtake(pmf);

		if (ret < 0) {
			return (ssize_t)ret;
		}

		ret = len; /* save number of bytes written */

		ssarc_dump_internal_write(pmf->pf, buffer, filep->f_pos, len);
		filep->f_pos += len;

		ssarc_dump_semgive(pmf);
	}

	return ret;
}

/****************************************************************************
 * Name: ssarc_dump_poll
 ****************************************************************************/

static int ssarc_dump_poll(struct file *filep, struct pollfd *fds,
			   bool setup)
{
	if (setup) {
		fds->revents |= (fds->events & (POLLIN | POLLOUT));

		if (fds->revents != 0) {
			nxsem_post(fds->sem);
		}
	}

	return OK;
}

/****************************************************************************
 * Name: ssarc_dump_ioctl
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int ssarc_dump_ioctl(struct file *filep, int cmd,
			    unsigned long arg)
{
	struct inode *inode = filep->f_inode;
	struct ssarc_dump_s *pmf;
	int ret = -ENOTTY;

	DEBUGASSERT(inode && inode->i_private);
	pmf = (struct ssarc_dump_s *)inode->i_private;

	if (cmd == SSARC_DUMP_GETDESC_IOCTL) {
		struct ssarc_s *desc = (struct ssarc_s *)((uintptr_t)arg);

		ret = ssarc_dump_semtake(pmf);

		if (ret < 0) {
			return ret;
		}

		if (!desc) {
			ret = -EINVAL;

		} else {
			desc->fileno = pmf->pf->fileno;
			desc->len = pmf->pf->len;
			desc->flags = ((pmf->pf->clean) ? 0 : 2);
			desc->lastwrite = pmf->pf->lastwrite;

			ret = OK;
		}

		ssarc_dump_semgive(pmf);

	}

	return ret;
}

/****************************************************************************
 * Name: ssarc_dump_unlink
 *
 * Description:
 *  This function will remove the remove the file from the file system
 *  it will zero the contents and time stamp. It will leave the fileno
 *  and pointer to the Progmem intact.
 *  It should be called called on the file used for the crash dump
 *  to remove it from visibility in the file system after it is created or
 *  read thus arming it.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int ssarc_dump_unlink(struct inode *inode)
{
	struct ssarc_dump_s *pmf;
	int ret;

	DEBUGASSERT(inode && inode->i_private);
	pmf = (struct ssarc_dump_s *)inode->i_private;

	ret = ssarc_dump_semtake(pmf);

	if (ret < 0) {
		return ret;
	}

	pmf->pf->lastwrite.tv_nsec = 0;
	pmf->pf->lastwrite.tv_sec = 0;
	pmf->refs = 0;

	ssarc_dump_semgive(pmf);
	nxsem_destroy(&pmf->exclsem);
	return 0;
}
#endif

/****************************************************************************
 * Name: ssarc_dump_probe
 *
 * Description: Based on the number of files defined and their sizes
 * Initializes the base pointers to the file entries.
 *
 ****************************************************************************/

static int ssarc_dump_probe(int *ent, struct ssarc_dump_s pdev[])
{
	int i, j;
	int alloc;
	int size;
	int avail;
	struct ssarcfh_s *pf = (struct ssarcfh_s *) PX4_SSARC_DUMP_BASE;
	int ret = -EFBIG;

	avail = PX4_SSARC_DUMP_SIZE;

	for (i = 0; (i < SSARC_DUMP_FILES) && ent[i] && (avail > 0);
	     i++) {
		/* Validate the actual allocations against what is in the PROGMEM */

		size = ent[i];

		/* Use all that is left */

		if (size == -1) {
			size = avail;
			size -= SSARCH_HEADER_SIZE;
		}

		/* Add in header size and keep aligned to blocks */

		alloc = (size / PX4_SSARC_BLOCK_DATA) * PX4_SSARC_BLOCK_SIZE;

		/* Does it fit? */

		if (size <= avail) {
			ret = i + 1;

			if ((int)pf->len != size || pf->fileno != i) {
				pf->len = size;
				pf->clean = 1;
				pf->fileno = i;
				pf->lastwrite.tv_sec = 0;
				pf->lastwrite.tv_nsec = 0;

				for (j = 0; j < (size / PX4_SSARC_BLOCK_DATA); j++) {
					memset(pf->data[j].data, 0, PX4_SSARC_BLOCK_DATA);
				}
			}

			pdev[i].pf = pf;
			pf = (struct ssarcfh_s *)((uint32_t *)pf + ((alloc + sizeof(struct ssarcfh_s)) / 4));
			nxsem_init(&g_ssarc[i].exclsem, 0, 1);
		}

		avail -= (size + PX4_SSARC_HEADER_SIZE);
	}

	return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: ssarc_dump_initialize
 *
 * Description:
 *   Initialize the Battery Backed up SRAM driver.
 *
 * Input Parameters:
 *   devpath - the path to instantiate the files.
 *   sizes   - Pointer to a any array of file sizes to create
 *             the last entry should be 0
 *             A size of -1 will use all the remaining spaces
 *
 * Returned Value:
 *   Number of files created on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int ssarc_dump_initialize(char *devpath, int *sizes)
{
	int i;
	int fcnt;
	char devname[32];

	int ret = OK;

	if (devpath == NULL) {
		return -EINVAL;
	}

	i = strlen(devpath);

	if (i == 0 || i > (int)sizeof(devname) - 3) {
		return -EINVAL;
	}

	memset(g_ssarc, 0, sizeof(g_ssarc));

	fcnt = ssarc_dump_probe(sizes, g_ssarc);

	for (i = 0; i < fcnt && ret >= OK; i++) {
		snprintf(devname, sizeof(devname), "%s%d", devpath, i);
		ret = register_driver(devname, &ssarc_dump_fops, 0666, &g_ssarc[i]);
	}

	return ret < OK ? ret : fcnt;
}

/****************************************************************************
 * Function: ssarc_dump_savepanic
 *
 * Description:
 *   Saves the panic context in a previously allocated PROGMEM file
 *
 * Input Parameters:
 *   fileno  - the value returned by the ioctl GETDESC_IOCTL
 *   context - Pointer to a any array of bytes to save
 *   length  - The length of the data pointed to byt context
 *
 * Returned Value:
 *   Length saved or negated errno.
 *
 * Assumptions:
 *
 ****************************************************************************/


int ssarc_dump_savepanic(int fileno, uint8_t *context, int length)
{
	struct ssarcfh_s *pf;
	int ret = -ENOSPC;

	/* On a bad day we could panic while panicking, (and we debug assert)
	 * this is a potential feeble attempt at only writing the first
	 * panic's context to the file
	 */

	static bool once = false;

	if (!once) {
		once = true;

		DEBUGASSERT(fileno > 0 && fileno < SSARC_DUMP_FILES);

		pf = g_ssarc[fileno].pf;

		/* If the g_ssarc has been nulled out we return ENXIO.
		 *
		 * As once ensures we will keep the first dump. Checking the time for
		 * 0 protects from over writing a previous crash dump that has not
		 * been saved to long term storage and erased.  The dreaded reboot
		 * loop.
		 */

		if (pf == NULL) {
			ret = -ENXIO;

		} else if (pf->lastwrite.tv_sec == 0 && pf->lastwrite.tv_nsec == 0) {
			/* Clamp length if too big  */

			if (length > (int)pf->len) {
				length = pf->len;
			}

			ssarc_dump_internal_write(pf, (char *) context, 0, length);

			/* Seal the file */

			ssarc_dump_internal_close(pf);
			ret = length;
		}
	}

	return ret;
}

#endif /* HAS_SSARC */
#endif /* SYSTEMCMDS_HARDFAULT_LOG */
