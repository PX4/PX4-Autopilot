/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
#include <nuttx/progmem.h>

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

#ifdef HAS_PROGMEM

#include <px4_platform/progmem_dump.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PROGMEM_DUMP_FILES    5
#define MAX_OPENCNT           (255) /* Limit of uint8_t */
#define PROGMEM_HEADER_SIZE   (sizeof(struct progmemfh_s))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct progmemfh_s {
	int32_t     fileno;        /* The minor number */
	uint32_t    len;           /* Total Bytes in this file */
	uint32_t    clean;         /* No data has been written to the file */
	struct timespec lastwrite; /* Last write time */
	uint8_t     padding[PROGMEM_DUMP_HEADER_PAD];  /* Ensure padding for quick write of data */
	uint8_t     data[];        /* Data in the file */
};

static_assert(sizeof(struct progmemfh_s) == PROGMEM_DUMP_ALIGNMENT, "progmemfh_s doesn't match write alignment");

struct progmem_dump_s {
	sem_t    exclsem;            /* For atomic accesses to this structure */
	uint8_t  refs;               /* Number of references */
	struct progmemfh_s *pf;      /* File in progmem */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     progmem_dump_open(struct file *filep);
static int     progmem_dump_close(struct file *filep);
static off_t   progmem_dump_seek(struct file *filep, off_t offset,
				 int whence);
static ssize_t progmem_dump_read(struct file *filep, char *buffer,
				 size_t len);
static ssize_t progmem_dump_write(struct file *filep,
				  const char *buffer, size_t len);
static int     progmem_dump_ioctl(struct file *filep, int cmd,
				  unsigned long arg);
static int     progmem_dump_poll(struct file *filep,
				 struct pollfd *fds, bool setup);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     progmem_dump_unlink(struct inode *inode);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/


static const struct file_operations progmem_dump_fops = {
	.open   = progmem_dump_open,
	.close  = progmem_dump_close,
	.read   = progmem_dump_read,
	.write  = progmem_dump_write,
	.seek   = progmem_dump_seek,
	.ioctl  = progmem_dump_ioctl,
	.poll   = progmem_dump_poll,
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
	.unlink = progmem_dump_unlink
#endif
};

/* Store ulog location in RAM to save on FLash wear */
static char ulog_file_loc[HARDFAULT_MAX_ULOG_FILE_LEN] = {0};

static struct progmem_dump_s g_progmem[PROGMEM_DUMP_FILES];

/****************************************************************************
 * Name: progmem_check_erase
 ****************************************************************************/

static int progmem_check_erase(char *buffer, size_t len)
{
	size_t i;

	for (i = 0; i < len; i++) {
		if (buffer[i] != PROGMEM_DUMP_ERASE_VALUE) {
			return 0;
		}
	}

	return 1;
}

/****************************************************************************
 * Name: progmem_dump_semgive
 ****************************************************************************/

static void progmem_dump_semgive(struct progmem_dump_s *priv)
{
	nxsem_post(&priv->exclsem);
}

/****************************************************************************
 * Name: progmem_dump_semtake
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

static int progmem_dump_semtake(struct progmem_dump_s *priv)
{
	return nxsem_wait_uninterruptible(&priv->exclsem);
}

/****************************************************************************
 * Name: progmem_dump_open
 *
 * Description: Open the device
 *
 ****************************************************************************/

static int progmem_dump_open(struct file *filep)
{
	struct inode *inode = filep->f_inode;
	struct progmem_dump_s *pmf;
	int ret;

	DEBUGASSERT(inode && inode->i_private);
	pmf = (struct progmem_dump_s *)inode->i_private;

	/* Increment the reference count */

	ret = progmem_dump_semtake(pmf);

	if (ret < 0) {
		return ret;
	}

	if (pmf->refs == MAX_OPENCNT) {
		return -EMFILE;

	} else {
		pmf->refs++;
	}

	progmem_dump_semgive(pmf);
	return OK;
}

/****************************************************************************
 * Name: progmem_dump_internal_close
 *
 * Description:
 *    Close Progmem entry; Recalculate the time and crc
 *
 ****************************************************************************/

static int progmem_dump_internal_close(struct progmemfh_s *pf)
{
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	up_progmem_write((size_t)pf + offsetof(struct progmemfh_s, lastwrite),
			 &ts, sizeof(ts));
	return pf->len;
}

/****************************************************************************
 * Name: progmem_dump_close
 *
 * Description: close the device
 *
 ****************************************************************************/

static int progmem_dump_close(struct file *filep)
{
	struct inode *inode = filep->f_inode;
	struct progmem_dump_s *pmf;
	int ret = OK;

	DEBUGASSERT(inode && inode->i_private);
	pmf = (struct progmem_dump_s *)inode->i_private;

	ret = progmem_dump_semtake(pmf);

	if (ret < 0) {
		return ret;
	}

	if (pmf->refs == 0) {
		ret = -EIO;

	} else {
		pmf->refs--;

		if (pmf->refs == 0) {
			if (pmf->pf->clean == 0) {
				progmem_dump_internal_close(pmf->pf);
			}
		}
	}

	progmem_dump_semgive(pmf);
	return ret;
}

/****************************************************************************
 * Name: progmem_dump_seek
 ****************************************************************************/

static off_t progmem_dump_seek(struct file *filep, off_t offset,
			       int whence)
{
	struct inode *inode = filep->f_inode;
	struct progmem_dump_s *pmf;
	off_t newpos;
	int ret;

	DEBUGASSERT(inode && inode->i_private);
	pmf = (struct progmem_dump_s *)inode->i_private;

	ret = progmem_dump_semtake(pmf);

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

		progmem_dump_semgive(pmf);
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

	progmem_dump_semgive(pmf);
	return ret;
}

/****************************************************************************
 * Name: progmem_dump_read
 ****************************************************************************/

static ssize_t progmem_dump_read(struct file *filep, char *buffer,
				 size_t len)
{
	struct inode *inode = filep->f_inode;
	struct progmem_dump_s *pmf;
	int ret;

	DEBUGASSERT(inode && inode->i_private);
	pmf = (struct progmem_dump_s *)inode->i_private;

	ret = progmem_dump_semtake(pmf);

	if (ret < 0) {
		return (ssize_t)ret;
	}

	/* Trim len if read would go beyond end of device */

	if ((filep->f_pos + len) > pmf->pf->len) {
		len = pmf->pf->len - filep->f_pos;
	}

	memcpy(buffer, &pmf->pf->data[filep->f_pos], len);
	filep->f_pos += len;
	progmem_dump_semgive(pmf);
	return len;
}

/****************************************************************************
 * Name: progmem_dump_internal_write
 ****************************************************************************/

static ssize_t progmem_dump_internal_write(struct progmemfh_s *pf,
		const char *buffer,
		off_t offset, size_t len)
{
	if (pf->clean) {
		/* Update Header */
		uint32_t clean = 0;
		up_progmem_write((size_t)pf + offsetof(struct progmemfh_s, clean),
				 &clean, sizeof(clean));

		/* Write data */
		up_progmem_write((size_t)&pf->data[offset], buffer, len);
		return len;

	} else {
		/* It's dirty and since it's flash we can't write again */
		return 0;
	}
}

/****************************************************************************
 * Name: progmem_dump_write
 ****************************************************************************/

static ssize_t progmem_dump_write(struct file *filep,
				  const char *buffer, size_t len)
{
	struct inode *inode = filep->f_inode;
	struct progmem_dump_s *pmf;
	int ret = -EFBIG;

	DEBUGASSERT(inode && inode->i_private);
	pmf = (struct progmem_dump_s *)inode->i_private;

	/* Forbid writes past the end of the device */

	if (filep->f_pos < (int)pmf->pf->len) {
		/* Clamp len to avoid crossing the end of the memory */

		if ((filep->f_pos + len) > pmf->pf->len) {
			len = pmf->pf->len - filep->f_pos;
		}

		ret = progmem_dump_semtake(pmf);

		if (ret < 0) {
			return (ssize_t)ret;
		}

		ret = len; /* save number of bytes written */

		if (pmf->pf->fileno == HARDFAULT_ULOG_FILENO) {
			/* Write ULOG location to ram to save on flash wear */
			strncpy(ulog_file_loc, buffer, HARDFAULT_MAX_ULOG_FILE_LEN);

		} else {
			progmem_dump_internal_write(pmf->pf, buffer, filep->f_pos, len);
			filep->f_pos += len;
		}

		progmem_dump_semgive(pmf);
	}

	return ret;
}

/****************************************************************************
 * Name: progmem_dump_poll
 ****************************************************************************/

static int progmem_dump_poll(struct file *filep, struct pollfd *fds,
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
 * Name: progmem_dump_ioctl
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int progmem_dump_ioctl(struct file *filep, int cmd,
			      unsigned long arg)
{
	struct inode *inode = filep->f_inode;
	struct progmem_dump_s *pmf;
	int ret = -ENOTTY;

	DEBUGASSERT(inode && inode->i_private);
	pmf = (struct progmem_dump_s *)inode->i_private;

	if (cmd == PROGMEM_DUMP_GETDESC_IOCTL) {
		struct progmem_s *desc = (struct progmem_s *)((uintptr_t)arg);

		ret = progmem_dump_semtake(pmf);

		if (ret < 0) {
			return ret;
		}

		if (!desc) {
			ret = -EINVAL;

		} else {
			desc->fileno = pmf->pf->fileno;
			desc->len = pmf->pf->len;
			desc->flags = ((pmf->pf->clean) ? 0 : 2);

			if (progmem_check_erase((char *)&pmf->pf->lastwrite,
						sizeof(pmf->pf->lastwrite))) {
				desc->lastwrite.tv_sec = 0;
				desc->lastwrite.tv_nsec = 0;

			} else {
				desc->lastwrite = pmf->pf->lastwrite;
			}

			ret = OK;
		}

		progmem_dump_semgive(pmf);

	} else if (cmd == PROGMEM_DUMP_CLEAR_IOCTL) {
		ret = progmem_dump_semtake(pmf);

		if (ret < 0) {
			return ret;
		}

		/* Our dump data is beyond the progmem neraseblocks
		 * so we just start with that block
		 */
		int i;
		size_t block = up_progmem_neraseblocks();
		size_t erase_size = up_progmem_erasesize(block);
		size_t to_erase = PROGMEM_DUMP_SIZE;
		uint32_t lens[PROGMEM_DUMP_FILES];

		for (i = 0; i < PROGMEM_DUMP_FILES; i++) {
			lens[i] = g_progmem[i].pf->len;
		}

		while (to_erase > 0) {
			up_progmem_eraseblock(block);
			block--;
			to_erase = to_erase - erase_size;
		}

		for (i = 0; i < PROGMEM_DUMP_FILES; i++) {
			struct progmemfh_s progmem_header;
			/* Make sure ERASE_VALUE is set so we can do back-to-back write */
			memset(&progmem_header, PROGMEM_DUMP_ERASE_VALUE, sizeof(struct progmemfh_s));
			progmem_header.fileno = i;
			progmem_header.len = lens[i];
			up_progmem_write((size_t)g_progmem[i].pf,
					 (void *)&progmem_header, sizeof(struct progmemfh_s));
		}

		ret = OK;

		progmem_dump_semgive(pmf);
	}

	return ret;
}

/****************************************************************************
 * Name: progmem_dump_unlink
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
static int progmem_dump_unlink(struct inode *inode)
{
	struct progmem_dump_s *pmf;
	int ret;

	DEBUGASSERT(inode && inode->i_private);
	pmf = (struct progmem_dump_s *)inode->i_private;

	ret = progmem_dump_semtake(pmf);

	if (ret < 0) {
		return ret;
	}

	progmem_dump_semgive(pmf);
	nxsem_destroy(&pmf->exclsem);
	return 0;
}
#endif

/****************************************************************************
 * Name: progmem_dump_probe
 *
 * Description: Based on the number of files defined and their sizes
 * Initializes the base pointers to the file entries.
 *
 ****************************************************************************/

static int progmem_dump_probe(int *ent, struct progmem_dump_s pdev[])
{
	int i;
	int alloc;
	int size;
	int avail;
	struct progmemfh_s *pf = (struct progmemfh_s *) PROGMEM_DUMP_BASE;
	int ret = -EFBIG;

	avail = PROGMEM_DUMP_SIZE;

	for (i = 0; (i < PROGMEM_DUMP_FILES) && ent[i] && (avail > 0);
	     i++) {
		/* Validate the actual allocations against what is in the PROGMEM */

		size = ent[i];

		/* Use all that is left */

		if (size == -1) {
			size = (avail - (avail % PROGMEM_DUMP_ALIGNMENT));
			size -= PROGMEM_HEADER_SIZE;
		}

		/* Add in header size and keep aligned */

		alloc = (((size + PROGMEM_DUMP_ALIGNMENT) / PROGMEM_DUMP_ALIGNMENT) + 1) * PROGMEM_DUMP_ALIGNMENT;

		if (size % PROGMEM_DUMP_ALIGNMENT != 0) {
			alloc = (2 * PROGMEM_DUMP_ALIGNMENT) +
				((size / PROGMEM_DUMP_ALIGNMENT) * PROGMEM_DUMP_ALIGNMENT);

		} else {
			alloc = PROGMEM_DUMP_ALIGNMENT +
				((size / PROGMEM_DUMP_ALIGNMENT) * PROGMEM_DUMP_ALIGNMENT);
		}

		/* Does it fit? */

		if (alloc <= avail) {
			ret = i + 1;

			if ((int)pf->len != size || pf->fileno != i) {
				/* Not Valid so wipe the file in PROGMEM */
				struct progmemfh_s progmem_header;
				/* Make sure ERASE_VALUE is set so we can do back-to-back write */
				memset(&progmem_header, PROGMEM_DUMP_ERASE_VALUE, sizeof(struct progmemfh_s));
				progmem_header.fileno = i;
				progmem_header.len = size;
				up_progmem_write((size_t)pf, (void *)&progmem_header, sizeof(struct progmemfh_s));
			}

			pdev[i].pf = pf;
			pf = (struct progmemfh_s *)((uint32_t *)pf + (alloc / 4));
			nxsem_init(&g_progmem[i].exclsem, 0, 1);
		}

		avail -= alloc;
	}

	return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: progmem_dump_initialize
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

int progmem_dump_initialize(char *devpath, int *sizes)
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

	memset(g_progmem, 0, sizeof(g_progmem));

	fcnt = progmem_dump_probe(sizes, g_progmem);

	for (i = 0; i < fcnt && ret >= OK; i++) {
		snprintf(devname, sizeof(devname), "%s%d", devpath, i);
		ret = register_driver(devname, &progmem_dump_fops, 0666, &g_progmem[i]);
	}

	return ret < OK ? ret : fcnt;
}

/****************************************************************************
 * Function: progmem_dump_savepanic
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


int progmem_dump_savepanic(int fileno, uint8_t *context, int length)
{
	struct progmemfh_s *pf;
	int ret = -ENOSPC;

	/* On a bad day we could panic while panicking, (and we debug assert)
	 * this is a potential feeble attempt at only writing the first
	 * panic's context to the file
	 */

	static bool once = false;

	if (!once) {
		once = true;

		DEBUGASSERT(fileno > 0 && fileno < PROGMEM_DUMP_FILES);

		pf = g_progmem[fileno].pf;

		/* If the g_progmem has been nulled out we return ENXIO.
		 *
		 * As once ensures we will keep the first dump. Checking the time for
		 * 0 protects from over writing a previous crash dump that has not
		 * been saved to long term storage and erased.  The dreaded reboot
		 * loop.
		 */

		if (pf == NULL) {
			ret = -ENXIO;

		} else if (progmem_check_erase((char *)&pf->lastwrite,
					       sizeof(pf->lastwrite))) {
			/* Clamp length if too big  */

			if (length > (int)pf->len) {
				length = pf->len;
			}

			progmem_dump_internal_write(pf, (char *) context, 0, length);

			/* Seal the file */

			progmem_dump_internal_close(pf);
			ret = length;
		}

		/* Write ulog file location to flash */
		pf = g_progmem[HARDFAULT_ULOG_FILENO].pf;

		if (pf == NULL) {
			ret = -ENXIO;

		} else if (progmem_check_erase((char *)&pf->lastwrite,
					       sizeof(pf->lastwrite))) {

			progmem_dump_internal_write(pf, ulog_file_loc, 0, HARDFAULT_MAX_ULOG_FILE_LEN);

			/* Seal the file */

			progmem_dump_internal_close(pf);
		}
	}

	return ret;
}

#endif /* HAS_PROGMEM */
#endif /* SYSTEMCMDS_HARDFAULT_LOG */
