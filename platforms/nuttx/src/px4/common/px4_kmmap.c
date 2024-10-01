/****************************************************************************
 * px4_kmmap.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/mman.h>
#include <sys/types.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mm/kmap.h>
#include <nuttx/pgalloc.h>

/* This is naughty, but only option as these are in NuttX private headers */

extern int inode_lock(void);
extern void inode_unlock(void);

struct shmfs_object_s {
	size_t length;
	void *paddr;
};

void *px4_mmap(void *start, size_t length, int prot, int flags, int fd, off_t offset)
{
	struct file *filep;
	struct shmfs_object_s *object;
	void **pages;
	void *vaddr;
	unsigned int npages;
	int ret;

	if (fs_getfilep(fd, &filep) < 0) {
		ret = -EBADF;
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

	/* Map the object to kernel */

	pages = &object->paddr;
	npages = MM_NPAGES(object->length);

	/* Do the mapping */

	vaddr = kmm_map(pages, npages, PROT_READ | PROT_WRITE);

	if (!vaddr) {
		ret = -ENOMEM;
		goto errout_with_lock;
	}

	filep->f_inode->i_crefs++;
	inode_unlock();
	fs_putfilep(filep);
	return vaddr;

errout_with_lock:
	inode_unlock();
errout:
	fs_putfilep(filep);
	set_errno(-ret);
	return MAP_FAILED;
}

int px4_munmap(void *start, size_t length)
{
	kmm_unmap(start);
	return OK;
}
