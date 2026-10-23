/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/pgalloc.h>

#ifndef CONFIG_ARCH_PGPOOL_MAPPING
#  error "kmmap needs CONFIG_ARCH_PGPOOL_MAPPING"
#endif

#if CONFIG_ARCH_PGPOOL_PBASE != CONFIG_ARCH_PGPOOL_VBASE
#  error "kmmap needs CONFIG_ARCH_PGPOOL_PBASE=CONFIG_ARCH_PGPOOL_VBASE mapping"
#endif

#define MAP_FAILED	  ((void*)-1)

/* Access NuttX private headers as no public interface is available here */

extern int inode_lock(void);
extern void inode_unlock(void);

struct shmfs_object_s {
	size_t length;
	void *paddr[];
};

void *px4_mmap(void *start, size_t length, int prot, int flags, int fd, off_t offset)
{
	struct file *filep;
	struct shmfs_object_s *object;
	int ret;

	if (fs_getfilep(fd, &filep) < 0) {
		ret = -EBADF;
		goto errout;
	}

	/* Limitation: only 1 page can be mapped. To map more pages, need more logic
	 * in place (define shared memory are for kernel, keep mappings in list, etc
	 */

	if (length > MM_PGSIZE) {
		ret = -ENOMEM;
		goto errout;
	}

	ret = inode_lock();

	if (ret < 0) {
		goto errout;
	}

	/* Return the physical address */

	object = (struct shmfs_object_s *)filep->f_inode->i_private;

	if (!object) {
		ret = -EINVAL;
		goto errout_with_lock;
	}

	filep->f_inode->i_crefs++;
	inode_unlock();
	return object->paddr[0];

errout_with_lock:
	inode_unlock();
errout:
	set_errno(-ret);
	return MAP_FAILED;
}

int px4_munmap(void *start, size_t length)
{
	/* There is no need for unmap on the kernel side */

	return OK;
}
